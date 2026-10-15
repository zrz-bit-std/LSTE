"""Route-frame and action-contract queries for the TEB turn supervisor."""

import copy
import math

import rospy
import tf

from teb_turn_supervisor_contract import (
    FRONTIER_ENDPOINT_KIND,
    FRONTIER_SOURCE,
    LOCAL_EGRESS_KIND,
    PORTAL_TRANSITION_KIND,
    SHARP_ENTRY_CONTINUITY_BLOCK_RAD,
    TURN_ROUTE_KIND,
    angle_from_pose,
    is_managed_frontier_route,
    normalize_angle,
)


class TebTurnSupervisorRoutesMixin:
    """Read-only geometry and execution-ownership helpers."""

    @staticmethod
    def _goal_xy(message):
        if message is None:
            return None
        return (
            float(message.pose.position.x),
            float(message.pose.position.y),
        )

    def _goal_yaw_in_pose_frame_locked(self, goal):
        """Transform a route tangent once into the odometry execution frame."""
        if goal is None:
            return None
        source_frame = (goal.header.frame_id or self.pose_frame).strip().lstrip("/")
        if source_frame == self.pose_frame:
            return normalize_angle(angle_from_pose(goal))
        transformed = copy.deepcopy(goal)
        transformed.header.stamp = rospy.Time(0)
        try:
            self.tf_listener.waitForTransform(
                self.pose_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(0.20),
            )
            transformed = self.tf_listener.transformPose(
                self.pose_frame, transformed
            )
        except (
            tf.Exception,
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                3.0,
                "TEB turn supervisor waiting for %s <- %s yaw transform: %s",
                self.pose_frame,
                source_frame,
                exc,
            )
            return None
        return normalize_angle(angle_from_pose(transformed))

    def _goal_distance_in_pose_frame_locked(self, goal):
        """Return planar distance to ``goal`` in the execution pose frame."""
        if goal is None or self.pose is None:
            return None
        source_frame = (goal.header.frame_id or self.pose_frame).strip().lstrip("/")
        if source_frame == self.pose_frame:
            transformed = goal
        else:
            transformed = copy.deepcopy(goal)
            transformed.header.stamp = rospy.Time(0)
            try:
                self.tf_listener.waitForTransform(
                    self.pose_frame,
                    source_frame,
                    rospy.Time(0),
                    rospy.Duration(0.05),
                )
                transformed = self.tf_listener.transformPose(
                    self.pose_frame, transformed
                )
            except (
                tf.Exception,
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ) as exc:
                rospy.logwarn_throttle(
                    3.0,
                    "TEB continuity gate waiting for %s <- %s goal transform: %s",
                    self.pose_frame,
                    source_frame,
                    exc,
                )
                return None
        return math.hypot(
            float(transformed.pose.position.x) - float(self.pose.x),
            float(transformed.pose.position.y) - float(self.pose.y),
        )

    def _intent_matches_goal_locked(self, goal):
        if goal is None or self.latest_intent_goal is None:
            return True
        xy = self._goal_xy(goal)
        return math.hypot(
            xy[0] - self.latest_intent_goal[0],
            xy[1] - self.latest_intent_goal[1],
        ) <= 0.08

    def _is_frontier_endpoint_action_locked(self):
        return (
            self.active_action
            and self.active_action_route_kind == FRONTIER_ENDPOINT_KIND
            and self.active_action_source == FRONTIER_SOURCE
        )

    def _is_portal_transition_action_locked(self):
        """Return whether one certified portal edge owns the active action."""
        return (
            self.active_action
            and self.active_action_route_kind == PORTAL_TRANSITION_KIND
            and self.active_action_source == FRONTIER_SOURCE
        )

    def _is_local_egress_action_locked(self):
        """Return whether the active action is a graph-owned recovery route."""
        return (
            self.active_action
            and self.active_action_route_kind == LOCAL_EGRESS_KIND
            and self.active_action_source == FRONTIER_SOURCE
        )

    def _is_managed_action_locked(self):
        """Return whether this execution adapter owns the active action phase."""
        return bool(
            self.active_action
            and is_managed_frontier_route(
                self.active_action_route_kind,
                self.active_action_source,
            )
        )

    def _is_same_target_segment_continuation_locked(self):
        """Return whether a replacement keeps the confirmed target identity."""
        return (
            self.active_action
            and self.active_action_priority == 2
            and self.active_action_source.startswith("target_")
            and bool(self.active_action_target_track_id)
            and self.latest_intent_priority == 2
            and self.latest_intent_source.startswith("target_")
            and self.latest_target_track_id == self.active_action_target_track_id
        )

    def _is_continuity_eligible_action_locked(self):
        """Return action classes permitted to bridge a transient TEB gap."""
        return (
            self._is_managed_action_locked()
            or self._is_same_target_segment_continuation_locked()
        )

    def _active_action_key_locked(self):
        goal = self.active_action_goal
        if goal is None:
            return None
        return (
            self.active_action_route_kind,
            self.active_action_source,
            int(getattr(self, "active_action_route_id", 0) or 0),
            getattr(self, "active_action_map_epoch", None),
            getattr(self, "active_action_origin_epoch", None),
            getattr(self, "active_action_projection_epoch", None),
            int(
                getattr(self, "active_action_graph_transaction_id", 0) or 0
            ),
            int(
                getattr(self, "active_action_goal_transaction_id", 0) or 0
            ),
            int(getattr(self, "active_action_transaction_id", 0) or 0),
            int(getattr(self, "active_action_generation", 0) or 0),
            round(float(goal.pose.position.x), 3),
            round(float(goal.pose.position.y), 3),
            round(normalize_angle(angle_from_pose(goal)), 3),
        )

    def _initial_navfn_yaw_in_pose_frame_locked(self):
        """Return the first non-zero Navfn path tangent in the odom frame."""
        plan = self.latest_navfn_plan
        if plan is None or len(plan.poses) < 2:
            return None
        source_frame = (plan.header.frame_id or self.pose_frame).strip().lstrip(
            "/"
        ) or self.pose_frame
        transformed = []
        for pose in plan.poses[: min(len(plan.poses), 24)]:
            candidate = copy.deepcopy(pose)
            candidate.header.frame_id = source_frame
            if source_frame != self.pose_frame:
                candidate.header.stamp = rospy.Time(0)
                try:
                    self.tf_listener.waitForTransform(
                        self.pose_frame,
                        source_frame,
                        rospy.Time(0),
                        rospy.Duration(0.05),
                    )
                    candidate = self.tf_listener.transformPose(
                        self.pose_frame, candidate
                    )
                except (
                    tf.Exception,
                    tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException,
                ) as exc:
                    rospy.logwarn_throttle(
                        3.0,
                        "TEB turn supervisor waiting for %s <- %s plan transform: %s",
                        self.pose_frame,
                        source_frame,
                        exc,
                    )
                    return None
            transformed.append(candidate)
        if len(transformed) < 2:
            return None
        first = transformed[0].pose.position
        for candidate in transformed[1:]:
            point = candidate.pose.position
            distance = math.hypot(point.x - first.x, point.y - first.y)
            if distance >= 0.12:
                return normalize_angle(math.atan2(point.y - first.y, point.x - first.x))
        return None

    def _sharp_navfn_entry_heading_error_locked(self):
        """Return a sharp active-route entry error, otherwise ``None``."""
        if self.pose is None:
            return None
        target_yaw = self._initial_navfn_yaw_in_pose_frame_locked()
        if target_yaw is None:
            return None
        error = normalize_angle(target_yaw - self.pose.theta)
        if abs(error) < SHARP_ENTRY_CONTINUITY_BLOCK_RAD:
            return None
        return error

    def _turn_key_for_goal_locked(self, goal, target_yaw=None):
        if goal is None:
            return None
        xy = self._goal_xy(goal)
        if target_yaw is None:
            target_yaw = angle_from_pose(goal)
        return (
            round(xy[0], 3),
            round(xy[1], 3),
            round(normalize_angle(target_yaw), 3),
        )
