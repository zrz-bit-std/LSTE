"""Camera-ray projection and TF helpers for :class:`GoalManager`.

These helpers are intentionally kept independent of goal arbitration.  They
only convert an image detection or a laser direction into the frame used by
the navigation stack, while the host object supplies the current sensor
messages and TF buffer.
"""

import math
from typing import Optional

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped, TransformStamped
from tf.transformations import quaternion_matrix


class GoalManagerProjectionMixin:
    """Methods shared by the ROS node and projection-focused tests."""

    def det_heading_world(self, det, stamp=None) -> Optional[float]:
        """Return an image ray's odom heading at its camera exposure time."""
        if self.camera_info is None or self.camera_frame is None:
            rospy.logwarn_throttle(5.0, "GoalManager: camera info not ready")
            return None
        width = int(self.camera_info.width) if self.camera_info.width else 0
        height = int(self.camera_info.height) if self.camera_info.height else 0
        if width <= 0 or height <= 0:
            return None

        pixel = (
            max(0, min(width - 1, int(float(det.cx) * width))),
            max(0, min(height - 1, int(float(det.cy) * height))),
        )
        ray = self.camera_model.projectPixelTo3dRay(pixel)
        tf_stamp = stamp if stamp is not None else rospy.Time(0)
        transform = self.lookup_transform("odom", self.camera_frame, tf_stamp)
        if transform is None:
            if tf_stamp.to_sec() > 0.0:
                rospy.logwarn_throttle(
                    1.0,
                    "GoalManager: source-stamped camera TF unavailable; "
                    "discarding projection stamp=%.6f",
                    tf_stamp.to_sec(),
                )
            return None

        rotation = transform.transform.rotation
        matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
        direction = matrix[:3, :3].dot(np.asarray(ray[:3], dtype=float))
        return math.atan2(direction[1], direction[0])

    def det_origin_world(self, stamp=None) -> Optional[tuple]:
        """Return the camera ray origin in odom at the image exposure time.

        Target geometry must not combine a source-stamped bearing with the
        robot pose from a later callback.  The camera TF translation is the
        physically correct origin for the projected ray and is available from
        the same lookup contract as :meth:`det_heading_world`.
        """
        if self.camera_frame is None:
            return None
        tf_stamp = stamp if stamp is not None else rospy.Time(0)
        transform = self.lookup_transform("odom", self.camera_frame, tf_stamp)
        if transform is None:
            return None
        translation = transform.transform.translation
        try:
            origin = float(translation.x), float(translation.y)
        except (AttributeError, TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in origin):
            return None
        return origin

    def clip_distance(self, heading_world: float, desired: float) -> Optional[float]:
        """Clip a projected goal ray against the current laser scan."""
        if self.latest_scan is None:
            return desired

        scan = self.latest_scan
        scan_frame = self.scan_frame or scan.header.frame_id
        if not scan_frame:
            return desired
        scan_yaw = self.frame_yaw(scan_frame)
        if scan_yaw is None:
            return desired

        increment = scan.angle_increment or 1e-6
        index = int(round((_wrap_angle(heading_world - scan_yaw) - scan.angle_min) / increment))
        if not scan.ranges:
            return desired
        index = max(0, min(len(scan.ranges) - 1, index))
        window = self.scan_window_bins
        values = scan.ranges[max(0, index - window): min(len(scan.ranges), index + window + 1)]
        finite_ranges = [value for value in values if math.isfinite(value) and value > 0.05]
        if not finite_ranges:
            return desired

        clipped = min(desired, min(finite_ranges) - self.safety_margin)
        if clipped <= 0.2:
            rospy.logwarn_throttle(2.0, "GoalManager: clipped dist too small (%.2f)", clipped)
            return None
        return clipped

    def transform_point(self, point: PointStamped, target_frame: str) -> Optional[PointStamped]:
        """Transform a stamped point, returning ``None`` on unavailable TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                point.header.frame_id,
                point.header.stamp,
                rospy.Duration(0.05),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(5.0, "GoalManager: TF lookup failed: %s", exc)
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
        matrix[:3, 3] = [translation.x, translation.y, translation.z]
        vector = np.array([point.point.x, point.point.y, point.point.z, 1.0], dtype=float)
        output = matrix.dot(vector)

        result = PointStamped()
        result.header.frame_id = target_frame
        result.header.stamp = transform.header.stamp or rospy.Time.now()
        result.point.x, result.point.y, result.point.z = map(float, output[:3])
        return result

    def lookup_transform(
        self, target: str, source: str, stamp: rospy.Time
    ) -> Optional[TransformStamped]:
        """Look up TF with one consistent timeout and warning policy."""
        try:
            return self.tf_buffer.lookup_transform(
                target, source, stamp, rospy.Duration(0.05)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(5.0, "GoalManager: TF lookup failed: %s", exc)
            return None

    def frame_yaw(self, frame: str) -> Optional[float]:
        """Return a frame's yaw in ``odom`` coordinates."""
        transform = self.lookup_transform("odom", frame, rospy.Time(0))
        if transform is None:
            return None
        rotation = transform.transform.rotation
        matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
        return math.atan2(matrix[1, 0], matrix[0, 0])


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
