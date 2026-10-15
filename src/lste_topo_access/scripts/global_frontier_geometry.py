#!/usr/bin/env python3

"""TF and planar geometry helpers for global-frontier exploration."""

import math

import numpy as np
import rospy
from tf.transformations import quaternion_matrix


class GlobalFrontierGeometryMixin:
    def planar_xy_projector(self, target_frame, source_frame):
        """Capture one planar TF transform for a coherent planning snapshot.

        Candidate scoring may project hundreds of map cells. Looking up TF for
        each one is both expensive and internally inconsistent if SLAM updates
        halfway through the scan, so callers obtain this closure once.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(0.15),
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                3.0,
                "Global frontier waiting for %s <- %s transform: %s",
                target_frame,
                source_frame,
                exc,
            )
            return None
        rotation = transform.transform.rotation
        matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
        translation = transform.transform.translation

        def project(x, y):
            point = matrix[:3, :3].dot(np.array([x, y, 0.0], dtype=float))
            return point[0] + translation.x, point[1] + translation.y

        return project

    def transform_xy(self, target_frame, source_frame, x, y):
        """Transform one point through the current planar TF relation."""
        projector = self.planar_xy_projector(target_frame, source_frame)
        if projector is None:
            return None
        return projector(x, y)

    def transform_yaw(self, target_frame, source_frame, yaw):
        """Transform a planar yaw using the same TF lookup as ``transform_xy``."""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(0.15),
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                3.0,
                "Global frontier waiting for %s <- %s yaw transform: %s",
                target_frame,
                source_frame,
                exc,
            )
            return None
        rotation = transform.transform.rotation
        tf_yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return math.atan2(
            math.sin(float(yaw) + tf_yaw),
            math.cos(float(yaw) + tf_yaw),
        )

    @staticmethod
    def heading_delta(x, y, robot_xy, robot_yaw):
        """Return the absolute bearing change needed to reach a candidate."""
        if robot_xy is None or robot_yaw is None:
            return None
        bearing = math.atan2(y - robot_xy[1], x - robot_xy[0])
        return abs(math.atan2(
            math.sin(bearing - robot_yaw),
            math.cos(bearing - robot_yaw),
        ))

    @staticmethod
    def _angle_delta(first, second):
        return math.atan2(math.sin(first - second), math.cos(first - second))
