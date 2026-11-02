#!/usr/bin/env python3

"""TF and planar geometry helpers for global-frontier exploration."""

import math

import numpy as np
import rospy
from tf.transformations import quaternion_matrix


def map_snapshot_projector(node, message):
    """Return only the immutable odom-to-raster transform for this exact map."""
    try:
        key=(message.header.frame_id or 'map',message.header.stamp.to_sec())
    except AttributeError:
        return None
    if key != getattr(node,'_place_partition_tf_key',None):return None
    project=getattr(node,'_place_partition_projector',None)
    return project if callable(project) else None


def inverse_planar_projector(project):
    """Invert the captured planar affine transform without a second TF lookup."""
    zero = np.asarray(project(0., 0.), dtype=float)
    matrix = np.column_stack((np.asarray(project(1., 0.))-zero,
                              np.asarray(project(0., 1.))-zero))
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(zero)):
        return None
    if abs(float(np.linalg.det(matrix))) < 1e-9:
        return None
    inverse = np.linalg.inv(matrix)
    def unproject(x, y):
        result = inverse.dot(np.asarray((x, y), dtype=float)-zero)
        return float(result[0]), float(result[1])
    return unproject


class GlobalFrontierGeometryMixin:
    def planar_xy_projector(self, target_frame, source_frame, stamp=None):
        """Capture one planar TF transform for a coherent planning snapshot.

        Candidate scoring may project hundreds of map cells. Looking up TF for
        each one is both expensive and internally inconsistent if SLAM updates
        halfway through the scan, so callers obtain this closure once.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0) if stamp is None else stamp,
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
