"""Gazebo target geometry projection for navigation telemetry."""

import rospy
import tf


class NavigationMetricsTargetEvaluationProjectionMixin:
    """Project a configured Gazebo target into the calibrated RGB camera."""

    @staticmethod
    def _target_eval_bbox_iou(first, second):
        left = max(float(first[0]), float(second[0]))
        top = max(float(first[1]), float(second[1]))
        right = min(float(first[2]), float(second[2]))
        bottom = min(float(first[3]), float(second[3]))
        intersection = max(0.0, right - left) * max(0.0, bottom - top)
        first_area = max(0.0, float(first[2]) - float(first[0])) * max(
            0.0, float(first[3]) - float(first[1])
        )
        second_area = max(0.0, float(second[2]) - float(second[0])) * max(
            0.0, float(second[3]) - float(second[1])
        )
        union = first_area + second_area - intersection
        return 0.0 if union <= 1e-9 else intersection / union

    def _target_eval_world_to_odom_point(self, state, world_point):
        """Map a Gazebo world point through the sampled world/odom base relation."""
        robot_world = state["robot_world"]
        odom = state["odom"]
        delta_world = (
            float(world_point[0]) - robot_world["position"][0],
            float(world_point[1]) - robot_world["position"][1],
            float(world_point[2]) - robot_world["position"][2],
        )
        in_base = self._rotate_point(
            self._quaternion_conjugate(robot_world["orientation"]), delta_world
        )
        in_odom = self._rotate_point(odom["orientation"], in_base)
        return (
            odom["position"][0] + in_odom[0],
            odom["position"][1] + in_odom[1],
            odom["position"][2] + in_odom[2],
        )

    def _target_eval_projected_box_locked(self, source_stamp, state):
        """Project the configured Gazebo target bounds into the RGB camera."""
        if self.target_eval_camera is None:
            return None, "camera_info_unavailable"
        camera = self.target_eval_camera
        try:
            translation, rotation = self.tf_listener.lookupTransform(
                camera["frame"],
                self.target_eval_odom_frame,
                rospy.Time.from_sec(source_stamp),
            )
        except (
            tf.Exception,
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as exc:
            self.target_eval_tf_failures += 1
            return None, "camera_tf_unavailable:%s" % type(exc).__name__

        target = state["target_world"]
        local_points = [self.target_eval_center]
        for sx in (-0.5, 0.5):
            for sy in (-0.5, 0.5):
                for sz in (-0.5, 0.5):
                    local_points.append((
                        self.target_eval_center[0] + sx * self.target_eval_size[0],
                        self.target_eval_center[1] + sy * self.target_eval_size[1],
                        self.target_eval_center[2] + sz * self.target_eval_size[2],
                    ))

        camera_points = []
        for local_point in local_points:
            rotated = self._rotate_point(target["orientation"], local_point)
            world_point = (
                target["position"][0] + rotated[0],
                target["position"][1] + rotated[1],
                target["position"][2] + rotated[2],
            )
            odom_point = self._target_eval_world_to_odom_point(state, world_point)
            rotated_camera = self._rotate_point(rotation, odom_point)
            camera_points.append((
                rotated_camera[0] + translation[0],
                rotated_camera[1] + translation[1],
                rotated_camera[2] + translation[2],
            ))

        center = camera_points[0]
        if not self.target_eval_min_depth <= center[2] <= self.target_eval_max_depth:
            return {
                "exposed": False,
                "reason": "center_depth_out_of_range",
                "center_depth_m": center[2],
                "target_depth_min_m": None,
                "target_depth_max_m": None,
                "predicted_box_px": None,
                "predicted_box_normalized": None,
            }, None
        projected = []
        for point in camera_points[1:]:
            if point[2] <= self.target_eval_min_depth:
                continue
            projected.append((
                camera["fx"] * point[0] / point[2] + camera["cx"],
                camera["fy"] * point[1] / point[2] + camera["cy"],
            ))
        if len(projected) < 2:
            return {
                "exposed": False,
                "reason": "target_bounds_behind_camera",
                "center_depth_m": center[2],
                "target_depth_min_m": None,
                "target_depth_max_m": None,
                "predicted_box_px": None,
                "predicted_box_normalized": None,
            }, None
        predicted_box = (
            min(point[0] for point in projected),
            min(point[1] for point in projected),
            max(point[0] for point in projected),
            max(point[1] for point in projected),
        )
        center_px = (
            camera["fx"] * center[0] / center[2] + camera["cx"],
            camera["fy"] * center[1] / center[2] + camera["cy"],
        )
        intersection_width = max(
            0.0, min(float(camera["width"]), predicted_box[2]) - max(0.0, predicted_box[0])
        )
        intersection_height = max(
            0.0, min(float(camera["height"]), predicted_box[3]) - max(0.0, predicted_box[1])
        )
        in_margin = (
            self.target_eval_edge_margin
            <= center_px[0]
            <= camera["width"] - self.target_eval_edge_margin
            and self.target_eval_edge_margin
            <= center_px[1]
            <= camera["height"] - self.target_eval_edge_margin
        )
        exposed = intersection_width > 0.0 and intersection_height > 0.0 and in_margin
        # Detector boxes and the truth projection use the same normalized xyxy
        # coordinate system, clipped to the visible RGB image.
        visible_box_px = (
            max(0.0, min(float(camera["width"]), predicted_box[0])),
            max(0.0, min(float(camera["height"]), predicted_box[1])),
            max(0.0, min(float(camera["width"]), predicted_box[2])),
            max(0.0, min(float(camera["height"]), predicted_box[3])),
        )
        predicted_box_normalized = (
            visible_box_px[0] / float(camera["width"]),
            visible_box_px[1] / float(camera["height"]),
            visible_box_px[2] / float(camera["width"]),
            visible_box_px[3] / float(camera["height"]),
        )
        # The transformed cuboid corners bound the camera-axis depth of the
        # physical target. The depth evaluator uses this interval rather than
        # a brittle centre-pixel equality check.
        target_depths = [
            point[2] for point in camera_points
            if point[2] > self.target_eval_min_depth
        ]
        return {
            "exposed": exposed,
            "reason": "frustum_exposed" if exposed else "outside_image_or_margin",
            "center_depth_m": center[2],
            "target_depth_min_m": min(target_depths) if target_depths else None,
            "target_depth_max_m": max(target_depths) if target_depths else None,
            "center_px": center_px,
            "predicted_box_px": predicted_box,
            "predicted_box_normalized": predicted_box_normalized,
        }, None
