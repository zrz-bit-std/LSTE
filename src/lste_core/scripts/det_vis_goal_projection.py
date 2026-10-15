"""Project the final navigation goal into the current camera image."""

import cv2
import rospy
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # noqa: F401  # Register Point/PointStamped transforms.
import tf2_ros


class DetectionGoalProjectionMixin:
    """Draw the /lste/final_goal hint and its camera projection when visible."""

    @staticmethod
    def _draw_goal_hint(image, text):
        height, width = image.shape[:2]
        font = cv2.FONT_HERSHEY_DUPLEX
        scale = 0.68
        thickness = 2
        text_size, baseline = cv2.getTextSize(text, font, scale, thickness)
        margin = 10
        x = max(margin, width - text_size[0] - margin)
        y = height - margin
        cv2.rectangle(
            image,
            (x - 6, y - text_size[1] - baseline - 4),
            (x + text_size[0] + 6, y + baseline + 4),
            (0, 0, 0),
            thickness=-1,
        )
        cv2.putText(image, text, (x, y), font, scale, (0, 255, 0), thickness, cv2.LINE_AA)

    def _draw_global_goal_indicator(self, image, image_header=None):
        if self.global_goal_point is None:
            self._draw_goal_hint(image, "Global goal not received")
            return
        goal_x, goal_y, _ = self.global_goal_point
        self._draw_goal_hint(image, f"Global goal (odom): x={goal_x:.2f}, y={goal_y:.2f}")
        if not self.has_camera_info:
            return

        camera_frame = getattr(image_header, "frame_id", "") or self.camera_frame_id
        if not camera_frame:
            return
        goal_point = PointStamped()
        goal_point.header.frame_id = self.global_goal_frame or "odom"
        # In detection-frame mode, use the source image time.  Projecting with
        # latest TF moves a fixed goal while the camera is turning.
        goal_point.header.stamp = getattr(image_header, "stamp", rospy.Time(0))
        goal_point.point.x = goal_x
        goal_point.point.y = goal_y
        goal_point.point.z = 0.0
        try:
            point_in_camera = self.tf_buffer.transform(goal_point, camera_frame, rospy.Duration(0.05))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(5.0, "TF lookup for global goal failed: %s", exc)
            return
        if point_in_camera.point.z <= 1e-3:
            rospy.logwarn_throttle(5.0, "Global goal behind camera (z=%.3f)", point_in_camera.point.z)
            return
        try:
            pixel_x, pixel_y = self.camera_model.project3dToPixel(
                (point_in_camera.point.x, point_in_camera.point.y, point_in_camera.point.z)
            )
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Project global goal failed: %s", exc)
            return

        height, width = image.shape[:2]
        pixel = (int(round(pixel_x)), int(round(pixel_y)))
        if not (0 <= pixel[0] < width and 0 <= pixel[1] < height):
            rospy.logwarn_throttle(5.0, "Global goal projection outside image: (%.1f, %.1f)", pixel_x, pixel_y)
            return
        cv2.circle(image, pixel, 12, (0, 255, 0), 3)
        cv2.circle(image, pixel, 6, (0, 255, 0), -1)
        cv2.putText(
            image,
            "GLOBAL",
            (pixel[0] - 25, max(12, pixel[1] - 14)),
            cv2.FONT_HERSHEY_DUPLEX,
            0.58,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
