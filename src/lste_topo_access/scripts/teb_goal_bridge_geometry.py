"""Geometry and frame conversion for the TEB action bridge.

These helpers are intentionally kept at the action boundary.  Mission code
uses source-frame goals, while move_base health checks must compare poses in
the global action frame.
"""

import copy
import math

import rospy
import tf


class TebGoalBridgeGeometryMixin:
    @staticmethod
    def _as_bool(value):
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    @staticmethod
    def _yaw(message):
        q = message.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def _angle_delta(first, second):
        return math.atan2(math.sin(first - second), math.cos(first - second))

    def _same_goal(self, first, second):
        if first is None or second is None:
            return False
        if (first.header.frame_id or "") != (second.header.frame_id or ""):
            return False
        dx = first.pose.position.x - second.pose.position.x
        dy = first.pose.position.y - second.pose.position.y
        return (
            math.hypot(dx, dy) <= self.position_epsilon
            and (
                not self.compare_goal_yaw
                or abs(self._angle_delta(self._yaw(first), self._yaw(second)))
                <= self.yaw_epsilon
            )
        )

    @staticmethod
    def _normalize_goal(message):
        goal = copy.deepcopy(message)
        if not goal.header.frame_id:
            goal.header.frame_id = "odom"
        return goal

    def _goal_in_global_frame(self, source_goal):
        """Transform a source goal once at action-dispatch time."""
        goal = copy.deepcopy(source_goal)
        source_frame = (goal.header.frame_id or "").strip().lstrip("/") or "odom"
        goal.header.frame_id = source_frame
        if source_frame == self.global_frame:
            goal.header.stamp = rospy.Time.now()
            return goal
        goal.header.stamp = rospy.Time(0)
        try:
            self.tf_listener.waitForTransform(
                self.global_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(0.5),
            )
            transformed = self.tf_listener.transformPose(self.global_frame, goal)
        except (
            tf.Exception,
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                3.0,
                "TEB goal bridge waiting for %s <- %s transform: %s",
                self.global_frame,
                source_frame,
                exc,
            )
            return None
        transformed.header.frame_id = self.global_frame
        transformed.header.stamp = rospy.Time.now()
        return transformed

    def _feedback_in_global_frame(self, feedback_pose):
        """Convert move_base feedback to the action goal frame.

        MoveBase feedback normally arrives in ``odom`` while online-SLAM
        actions use ``map``.  Comparing the raw coordinates would create a
        false near-goal signal and unsafe early handoffs.
        """
        pose = copy.deepcopy(feedback_pose)
        source_frame = (pose.header.frame_id or "").strip().lstrip("/") or "odom"
        pose.header.frame_id = source_frame
        if source_frame == self.global_frame:
            return pose
        pose.header.stamp = rospy.Time(0)
        try:
            self.tf_listener.waitForTransform(
                self.global_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(0.05),
            )
            transformed = self.tf_listener.transformPose(self.global_frame, pose)
        except (
            tf.Exception,
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as exc:
            self.feedback_transform_failures += 1
            rospy.logwarn_throttle(
                3.0,
                "TEB goal bridge cannot transform feedback %s -> %s: %s",
                source_frame,
                self.global_frame,
                exc,
            )
            return None
        transformed.header.frame_id = self.global_frame
        return transformed
