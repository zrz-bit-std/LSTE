#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Frame-aware goal construction and atomic final-goal publication."""

import copy
import json
import math
from typing import Optional, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, quaternion_matrix

from goal_context import default_goal_context

class GoalManagerGoalOutputMixin:
    # -------------------- Helpers --------------------
    def _canonical_lifecycle_transaction_id(self) -> int:
        """Return the exact lifecycle identity of the current route."""
        inherited = int(
            getattr(self, "global_frontier_lifecycle_transaction_id", 0)
            or 0
        )
        if inherited > 0:
            # Global Frontier owns the route lifecycle.  A later local event
            # may already be queued, but it must not relabel this route's
            # mission goal, dispatch, or terminal as that future identity.
            return inherited
        manager = getattr(self, "lifecycle_manager", None)
        latest = getattr(manager, "latest_transaction_id", None)
        if callable(latest):
            current = int(latest() or 0)
        else:
            current = int(getattr(manager, "current_transaction_id", 0) or 0)
        return current

    def _canonical_map_epoch(self) -> int:
        """Return a map epoch previously issued by Global Frontier."""
        try:
            return max(0, int(getattr(self, "global_frontier_map_epoch", 0) or 0))
        except (TypeError, ValueError):
            return 0

    def _canonical_goal_identity_ready(self, source: str) -> bool:
        """Require complete upstream identity for the online route pipeline."""
        if not bool(getattr(self, "global_frontier_enabled", False)):
            return True
        lifecycle_id = self._canonical_lifecycle_transaction_id()
        if lifecycle_id <= 0:
            self.publish_goal_arbitration(
                "canonical_identity_rejected",
                reason="missing_lifecycle_transaction_id",
                source=str(source or "unknown"),
            )
            return False
        if source == "global_slam_frontier":
            fields = (
                ("route_id", getattr(self, "global_frontier_route_id", 0)),
                (
                    "graph_transaction_id",
                    getattr(self, "global_frontier_graph_transaction_id", 0),
                ),
                ("map_epoch", self._canonical_map_epoch()),
                (
                    "origin_epoch",
                    getattr(self, "global_frontier_origin_epoch", 0),
                ),
                (
                    "projection_epoch",
                    getattr(self, "global_frontier_projection_epoch", 0),
                ),
            )
            for name, value in fields:
                try:
                    valid = int(value or 0) > 0
                except (TypeError, ValueError):
                    valid = False
                if not valid:
                    self.publish_goal_arbitration(
                        "canonical_identity_rejected",
                        reason="missing_%s" % name,
                        source=source,
                    )
                    return False
        elif self._canonical_map_epoch() <= 0:
            # Target routes are a higher-priority semantic owner, but still
            # execute against the last map snapshot issued by Global Frontier.
            self.publish_goal_arbitration(
                "canonical_identity_rejected",
                reason="missing_map_epoch",
                source=str(source or "unknown"),
            )
            return False
        elif (
            int(getattr(self, "global_frontier_route_id", 0) or 0) <= 0
            or int(getattr(self, "global_frontier_graph_transaction_id", 0) or 0)
            <= 0
        ):
            self.publish_goal_arbitration(
                "canonical_identity_rejected",
                reason="missing_inherited_route_identity",
                source=str(source or "unknown"),
            )
            return False
        return True

    @staticmethod
    def _frame_name(frame: str) -> str:
        return (frame or "odom").strip().lstrip("/") or "odom"

    def robot_xy_in_frame(self, frame: str) -> Optional[Tuple[float, float]]:
        """Return the current robot position in a goal's frame.

        ``/rbt_pose`` is expressed in odom, while an online frontier is a
        stable map-frame point.  Comparing those coordinates directly creates
        a false distance whenever gmapping updates map->odom.  Transforming
        the robot origin for the comparison keeps the mission layer frame
        agnostic without rewriting the goal itself.
        """
        if self.latest_pose is None:
            return None
        target = self._frame_name(frame)
        if target == "odom":
            return float(self.latest_pose.x), float(self.latest_pose.y)
        tfm = self.lookup_transform(target, "odom", rospy.Time(0))
        if tfm is None:
            return None
        rot = tfm.transform.rotation
        mat = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        point = mat[:3, :3].dot(
            np.array([self.latest_pose.x, self.latest_pose.y, 0.0], dtype=float)
        )
        trans = tfm.transform.translation
        return float(point[0] + trans.x), float(point[1] + trans.y)

    def goal_robot_distance(self, goal: Optional[PoseStamped]) -> Optional[float]:
        if goal is None:
            return None
        robot = self.robot_xy_in_frame(goal.header.frame_id)
        if robot is None:
            return None
        return math.hypot(
            float(goal.pose.position.x) - robot[0],
            float(goal.pose.position.y) - robot[1],
        )

    def target_terminal_observation_bridge_ready(self) -> bool:
        """Return whether the bridge released the previous target lease."""
        pending = int(
            getattr(self, "target_terminal_observation_pending_transaction", 0)
            or 0
        )
        if pending <= 0:
            return True
        acknowledged = int(
            getattr(self, "target_terminal_observation_ack_transaction", 0)
            or 0
        )
        return acknowledged >= pending

    def _consume_target_terminal_observation_handoff(self) -> bool:
        """Close the acknowledged boundary when a successor is published."""
        pending = int(
            getattr(self, "target_terminal_observation_pending_transaction", 0)
            or 0
        )
        if pending <= 0 or not self.target_terminal_observation_bridge_ready():
            return False
        self.target_terminal_observation_pending_transaction = 0
        self.target_terminal_observation_ack_transaction = 0
        self.target_terminal_observation_intent_sent = False
        return True

    def publish_target_terminal_observation_intent(self, reason: str) -> bool:
        """Transfer action ownership to post-terminal target observation."""
        if (
            getattr(self, "target_terminal_observation_intent_sent", False)
            or int(
                getattr(
                    self, "target_terminal_observation_pending_transaction", 0
                )
                or 0
            )
            > 0
        ):
            return False
        if not self._canonical_goal_identity_ready("target_terminal_observation"):
            return False
        lifecycle_id = self._canonical_lifecycle_transaction_id()
        self.goal_command_id += 1
        mission_context = default_goal_context(
            getattr(self, "exploration_method", "legacy"),
            getattr(self, "current_task_id", ""),
            getattr(self, "current_mission_id", ""),
            getattr(self, "current_task_version", ""),
        )
        mission_context["goal_role"] = "semantic_target_observation"
        intent = {
            "event": "target_terminal_observation",
            "transaction_id": int(self.goal_command_id),
            "source": "target_terminal_observation",
            "priority": 2,
            "route_kind": "direct_goal",
            "mission_route_kind": "direct_goal",
            "route_id": int(getattr(self, "global_frontier_route_id", 0) or 0),
            "graph_transaction_id": int(
                getattr(self, "global_frontier_graph_transaction_id", 0) or 0
            ),
            "map_epoch": (
                self._canonical_map_epoch()
                if getattr(self, "global_frontier_enabled", False)
                else max(1, int(getattr(self, "navigation_map_epoch", 1) or 1))
            ),
            "origin_epoch": max(
                1,
                int(
                    getattr(self, "global_frontier_origin_epoch", None)
                    or self._canonical_map_epoch()
                    or 1
                ),
            ),
            "projection_epoch": max(
                1,
                int(
                    getattr(self, "global_frontier_projection_epoch", None)
                    or self._canonical_map_epoch()
                    or 1
                ),
            ),
            "task_id": str(getattr(self, "current_task_id", "")),
            "mission_id": str(getattr(self, "current_mission_id", "")),
            "task_version": str(getattr(self, "current_task_version", "")),
            "target_epoch": int(getattr(self, "target_observation_epoch", 0)),
            "target_track_id": str(getattr(self, "target_track_id", "") or ""),
            "target_viewpoint_candidate_id": str(
                getattr(self, "target_viewpoint_candidate_id", "") or ""
            ),
            "target_viewpoint_attempt_id": str(
                getattr(self, "target_viewpoint_attempt_id", "") or ""
            ),
            "goal_context": mission_context,
            "lifecycle_transaction_id": lifecycle_id,
            "reason": str(reason or "target_terminal_observation"),
        }
        self.pub_goal_command.publish(
            String(data=json.dumps(intent, sort_keys=True))
        )
        self.pub_goal_intent.publish(String(data=json.dumps(intent, sort_keys=True)))
        self.target_terminal_observation_intent_sent = True
        self.target_terminal_observation_pending_transaction = int(
            self.goal_command_id
        )
        self.target_terminal_observation_ack_transaction = 0
        self.publish_goal_arbitration(
            "target_terminal_observation_intent_published",
            reason=str(reason or "target_terminal_observation"),
            transaction_id=int(self.goal_command_id),
        )
        return True

    def make_goal_pose(self, xyz: Tuple[float, float, float], yaw: Optional[float] = None) -> PoseStamped:
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "odom"
        goal.pose.position.x = float(xyz[0])
        goal.pose.position.y = float(xyz[1])
        goal.pose.position.z = float(xyz[2])
        # 始终填充合法四元数，避免下游解析 yaw 失败
        if yaw is None:
            goal.pose.orientation.w = 1.0
        else:
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(yaw))
            goal.pose.orientation.x = qx
            goal.pose.orientation.y = qy
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw
        return goal

    def publish_goal(self, goal: PoseStamped, force_republish: bool = False):
        previous_goal = self.last_goal
        frontier_context = None
        frontier_context_changed = False
        if self.goal_source == "global_slam_frontier":
            frontier_context = self.frontier_mission_goal_context()
            frontier_context_changed = self.frontier_goal_context_changed()
        terminal_override = False
        if (
            self.controller_mode == "teb"
            and
            previous_goal is not None
            and self.teb_terminal_goal is not None
            and self.last_goal_source == "global_slam_frontier"
        ):
            terminal_delta = math.hypot(
                self.teb_terminal_goal.pose.position.x - previous_goal.pose.position.x,
                self.teb_terminal_goal.pose.position.y - previous_goal.pose.position.y,
            )
            terminal_override = terminal_delta <= max(
                self.global_frontier_update_radius, 0.30
            )
            if terminal_override and self.latest_pose is not None:
                # A stale/mis-associated action status can arrive while the
                # robot is still far from the source waypoint.  Never let
                # that status authorize a large branch jump and an immediate
                # in-place stop; require physical proximity as well.
                terminal_distance = self.goal_robot_distance(previous_goal)
                if terminal_distance is None:
                    terminal_distance = float("inf")
                terminal_override = (
                    terminal_distance <= self.global_frontier_early_handoff_radius
                )
        if (
            previous_goal is not None
            and self.goal_source == "global_slam_frontier"
            and self.last_goal_source == "global_slam_frontier"
            and self.latest_pose is not None
        ):
            distance_to_previous = self.goal_robot_distance(previous_goal)
            if distance_to_previous is None:
                distance_to_previous = float("inf")
            goal_delta = math.hypot(
                goal.pose.position.x - previous_goal.pose.position.x,
                goal.pose.position.y - previous_goal.pose.position.y,
            )
            if self.controller_mode == "teb" and not terminal_override:
                # TEB owns one atomic move_base action.  Do not cancel it from
                # this mission-layer callback based on a second wall clock:
                # the bridge coalesces map updates and uses action feedback to
                # hand off only near the endpoint or after real no-progress.
                # Keeping both timeout policies active caused a healthy action
                # at 0.24 m from its goal to be preempted at exactly 15 s.
                if goal_delta > 0.03:
                    rospy.loginfo_throttle(
                        3.0,
                        "GoalManager: forward frontier update to TEB bridge "
                        "distance=%.2fm update_delta=%.2fm",
                        distance_to_previous,
                        goal_delta,
                    )
            else:
                elapsed = float("inf")
                if self.frontier_goal_sent_at is not None:
                    elapsed = max(0.0, rospy.Time.now().to_sec() - self.frontier_goal_sent_at)
                if (
                    not terminal_override
                    and goal_delta >= self.global_frontier_jump_distance
                    and distance_to_previous > self.global_frontier_jump_release_radius
                ):
                    rospy.loginfo_throttle(
                        3.0,
                        "GoalManager: hold distant frontier branch jump "
                        "distance=%.2fm update_delta=%.2fm release_radius=%.2fm",
                        distance_to_previous,
                        goal_delta,
                        self.global_frontier_jump_release_radius,
                    )
                    return False
                if (
                    not terminal_override
                    and elapsed < self.global_frontier_min_hold_time
                    and goal_delta < self.global_frontier_jump_distance
                ):
                    rospy.loginfo_throttle(
                        3.0,
                        "GoalManager: hold frontier minimum dwell elapsed=%.1fs/%.1fs "
                        "distance=%.2fm update_delta=%.2fm",
                        elapsed,
                        self.global_frontier_min_hold_time,
                        distance_to_previous,
                        goal_delta,
                    )
                    return False
                if (
                    not terminal_override
                    and distance_to_previous > self.global_frontier_update_radius
                    and goal_delta < self.global_frontier_jump_distance
                ):
                    rospy.loginfo_throttle(
                        3.0,
                        "GoalManager: hold frontier goal distance=%.2fm "
                        "update_delta=%.2fm",
                        distance_to_previous,
                        goal_delta,
                    )
                    return False
        if terminal_override:
            rospy.loginfo(
                "GoalManager: replacing completed frontier action despite "
                "small update delta"
            )
            self.teb_terminal_goal = None
        # Do not rebroadcast an unchanged goal on every 5 Hz timer tick.  Apart
        # from wasting bandwidth, those messages used to look like target
        # movement to downstream consumers and made diagnosis impossible.
        # Fixed-goal mode is the exception: its latched periodic republish is
        # intentional for late controller subscribers.
        if (
            previous_goal is not None
            and not force_republish
            and math.hypot(
                goal.pose.position.x - previous_goal.pose.position.x,
                goal.pose.position.y - previous_goal.pose.position.y,
            ) <= 0.03
            and (
                self.last_goal_source == self.goal_source
                or (
                    self.last_goal_source.startswith("target_")
                    and self.goal_source.startswith("target_")
                )
            )
            and not frontier_context_changed
        ):
            return False
        if not self._canonical_goal_identity_ready(self.goal_source):
            return False
        self.last_goal = goal
        self.last_goal_source = self.goal_source
        self._consume_target_terminal_observation_handoff()
        if self.goal_source == "global_slam_frontier":
            self.last_frontier_goal_context = frontier_context
            self.last_published_frontier_route_id = int(
                getattr(self, "global_frontier_route_id", 0) or 0
            )
            self.teb_frontier_goal_history.append(copy.deepcopy(goal))
            # Keep enough history for coalesced route updates and native
            # in-place segment replacements without retaining a full mission.
            del self.teb_frontier_goal_history[:-12]
        if self.goal_source.startswith("target_"):
            self.target_execution_state = "TARGET_EXECUTING"
        if self.goal_source == "global_slam_frontier":
            self.frontier_goal_sent_at = rospy.Time.now().to_sec()
        lifecycle_id = self._canonical_lifecycle_transaction_id()
        # Publish the mission decision before the pose.  The bridge can then
        # classify the following PoseStamped before it considers dispatching an
        # action, avoiding a race between a target takeover and a frontier
        # update.  Priority is deliberately coarse: it describes ownership,
        # not a controller tuning value.
        mission_context = default_goal_context(
            getattr(self, "exploration_method", "legacy"),
            getattr(self, "current_task_id", ""),
            getattr(self, "current_mission_id", ""),
            getattr(self, "current_task_version", ""),
        )
        if self.goal_source.startswith("target_"):
            mission_context["goal_role"] = "semantic_target"
        intent = {
            "source": self.goal_source,
            "priority": self.goal_intent_priority(self.goal_source),
            # GoalManager owns ``transaction_id`` below.  Keep the graph
            # planner's identity separate so a mission update cannot make a
            # stale graph route appear current.
            "graph_transaction_id": 0,
            "lifecycle_transaction_id": lifecycle_id,
            # These fields identify the long-lived semantic mission.  The
            # PoseStamped below remains only the short-lived executable point.
            "task_id": str(getattr(self, "current_task_id", "")),
            "mission_id": str(getattr(self, "current_mission_id", "")),
            "task_version": str(getattr(self, "current_task_version", "")),
            "goal": [
                round(float(goal.pose.position.x), 4),
                round(float(goal.pose.position.y), 4),
            ],
            "goal_context": mission_context,
        }
        if self.goal_source == "global_slam_frontier":
            intent["route_kind"] = self.global_frontier_route_kind
            intent["mission_route_kind"] = self.global_frontier_mission_route_kind
            intent["route_id"] = int(self.global_frontier_route_id)
            intent["transition_kind"] = self.global_frontier_transition_kind
            intent["predecessor_route_id"] = int(
                self.global_frontier_predecessor_route_id
            )
            predecessor_contract = getattr(
                self, "global_frontier_predecessor_contract", None
            )
            if isinstance(predecessor_contract, dict):
                intent["predecessor_contract"] = dict(predecessor_contract)
            intent["transition_distance_to_previous_endpoint"] = (
                self.global_frontier_transition_distance
            )
            intent["goal_context"] = dict(frontier_context)
            intent["map_epoch"] = max(
                1,
                int(
                    getattr(self, "global_frontier_map_epoch", None)
                    or getattr(self, "navigation_map_epoch", 1)
                    or 1
                ),
            )
            intent["origin_epoch"] = max(
                1,
                int(
                    getattr(self, "global_frontier_origin_epoch", None)
                    or intent["map_epoch"]
                    or 1
                ),
            )
            intent["projection_epoch"] = max(
                1,
                int(
                    getattr(self, "global_frontier_projection_epoch", None)
                    or intent["map_epoch"]
                    or 1
                ),
            )
            intent["graph_action"] = str(
                getattr(self, "global_frontier_graph_action", "") or ""
            )
            intent["graph_obligation_kind"] = str(
                getattr(self, "global_frontier_graph_obligation_kind", "") or ""
            )
            intent["graph_transaction_id"] = int(
                getattr(self, "global_frontier_graph_transaction_id", 0) or 0
            )
        if self.goal_source.startswith("target_"):
            intent["target_epoch"] = int(self.target_observation_epoch)
            intent["route_kind"] = "direct_goal"
            intent["mission_route_kind"] = "direct_goal"
            # A target segment inherits the current Global Frontier route
            # identity. GoalManager does not create a parallel route namespace.
            target_route_id = int(
                getattr(self, "global_frontier_route_id", 0) or 0
            )
            if target_route_id <= 0:
                # A direct-target isolation profile has no Global Frontier
                # route namespace. Reuse the next semantic command number as
                # its local route identity; online target routes continue to
                # inherit the canonical frontier route above.
                target_route_id = max(1, int(self.goal_command_id) + 1)
            intent["route_id"] = target_route_id
            intent["graph_transaction_id"] = int(
                getattr(self, "global_frontier_graph_transaction_id", 0) or 0
            )
            intent["map_epoch"] = (
                self._canonical_map_epoch()
                if getattr(self, "global_frontier_enabled", False)
                else max(1, int(getattr(self, "navigation_map_epoch", 1) or 1))
            )
            intent["origin_epoch"] = max(
                1,
                int(
                    getattr(self, "global_frontier_origin_epoch", None)
                    or intent["map_epoch"]
                    or 1
                ),
            )
            intent["projection_epoch"] = max(
                1,
                int(
                    getattr(self, "global_frontier_projection_epoch", None)
                    or intent["map_epoch"]
                    or 1
                ),
            )
            intent["target_approach_epoch"] = int(
                getattr(
                    getattr(self, "target_approach_transaction", None),
                    "target_epoch",
                    0,
                )
                or 0
            )
            intent["target_track_id"] = self.target_track_id
            intent["target_state"] = self.target_execution_state
            intent["target_viewpoint_candidate_id"] = str(
                getattr(self, "target_viewpoint_candidate_id", "") or ""
            )
            intent["target_viewpoint_attempt_id"] = str(
                getattr(self, "target_viewpoint_attempt_id", "") or ""
            )
        # The graph identity is present at the top level for both frontier and
        # target routes. Keep the nested context derived from that same
        # identity so Bridge adoption cannot observe two route contracts.
        if int(intent.get("graph_transaction_id", 0) or 0) > 0:
            context = (
                dict(frontier_context)
                if frontier_context is not None
                else dict(mission_context)
            )
            context["graph_transaction_id"] = int(
                intent["graph_transaction_id"]
            )
            intent["goal_context"] = context
        # This is the execution boundary. Publishing pose and ownership in
        # one latched message prevents a freshly restarted bridge from first
        # receiving an obsolete final_goal and only later its replacement
        # metadata. The two legacy publications below remain observer APIs.
        self.goal_command_id += 1
        # Carry the same monotonic transaction through the legacy pose topic.
        # StreamingNavfnPlanner uses it to reject an older latched pose that
        # arrives after a newer bridge-approved mission transaction.
        goal.header.seq = int(self.goal_command_id) & 0xFFFFFFFF
        command = dict(intent)
        q = goal.pose.orientation
        command.update({
            "event": "mission_goal",
            "transaction_id": int(self.goal_command_id),
            "frame_id": (goal.header.frame_id or "odom").strip().lstrip("/") or "odom",
            "yaw": round(math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            ), 4),
            "stamp": rospy.Time.now().to_sec(),
            "lifecycle_transaction_id": lifecycle_id,
        })
        self.pub_goal_command.publish(String(data=json.dumps(command, sort_keys=True)))
        self.pub_goal_intent.publish(String(data=json.dumps(intent, sort_keys=True)))
        self.pub_goal.publish(goal)
        rospy.loginfo_throttle(2.0, "Publish /lste/final_goal: x=%.2f y=%.2f state=%s",
                               goal.pose.position.x, goal.pose.position.y, self.current_state)
        if self.debug_goal_log:
            self.log_goal_diagnostic(goal, previous_goal)
