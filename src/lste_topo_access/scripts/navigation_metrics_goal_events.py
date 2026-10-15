"""Goal-event telemetry for :mod:`lste_navigation_metrics`.

This module observes the two interfaces that define an execution target:
``/lste/final_goal`` carries the geometric point, while
``/lste/mission_goal`` carries its source and route transaction.  Keeping the
pair together prevents goal-change diagnostics from being interleaved with
velocity, scan, or planner callbacks in the composition root.
"""

import json
import math
import time

import rospy


class NavigationMetricsGoalEventsMixin:
    """Record target changes and their mission-level provenance."""

    def on_goal(self, message):
        xy = (float(message.pose.position.x), float(message.pose.position.y))
        frame = (message.header.frame_id or "odom").strip().lstrip("/") or "odom"
        with self.lock:
            self._failure_record_context_locked(
                "goal",
                "geometric_goal",
                {
                    "goal": [round(xy[0], 4), round(xy[1], 4)],
                    "frame_id": frame,
                    "source_stamp": message.header.stamp.to_sec(),
                },
            )
            self.goal_messages += 1
            if self.last_goal_xy is not None:
                delta = (
                    math.hypot(xy[0] - self.last_goal_xy[0], xy[1] - self.last_goal_xy[1])
                    if frame == self.last_goal_frame
                    else float("nan")
                )
                if math.isfinite(delta):
                    self.goal_delta_sum += delta
                    self.goal_delta_max = max(self.goal_delta_max, delta)
                if math.isfinite(delta) and delta > 0.03:
                    self.goal_changes += 1
                    self.goal_last_change_ros = rospy.Time.now().to_sec()
                    self.goal_last_change_wall = time.monotonic()
                    previous_goal = self.last_goal_xy
                    pose = self._pose_xy_in_frame_locked(frame)
                    previous_distance = (
                        float("nan") if pose is None else
                        math.hypot(previous_goal[0] - pose[0], previous_goal[1] - pose[1])
                    )
                    new_distance = (
                        float("nan") if pose is None else
                        math.hypot(xy[0] - pose[0], xy[1] - pose[1])
                    )
                    old_bearing = (
                        float("nan") if pose is None else
                        math.atan2(previous_goal[1] - pose[1], previous_goal[0] - pose[0])
                    )
                    new_bearing = (
                        float("nan") if pose is None else
                        math.atan2(xy[1] - pose[1], xy[0] - pose[0])
                    )
                    self._write(
                        "INFO",
                        "goal_change",
                        delta_m=round(delta, 4),
                        goal=[round(xy[0], 3), round(xy[1], 3)],
                        previous_goal=[round(previous_goal[0], 3), round(previous_goal[1], 3)],
                        pose=None if pose is None else [round(value, 3) for value in pose],
                        previous_distance_m=None if not math.isfinite(previous_distance) else round(previous_distance, 4),
                        new_distance_m=None if not math.isfinite(new_distance) else round(new_distance, 4),
                        old_bearing_rad=None if not math.isfinite(old_bearing) else round(old_bearing, 4),
                        new_bearing_rad=None if not math.isfinite(new_bearing) else round(new_bearing, 4),
                        controller_source=self.controller_source,
                        controller_reason=self.controller_reason,
                        goal_source=self.goal_source,
                        goal_hold_seconds=(
                            None if self.last_goal_publish_wall is None else
                            round(time.monotonic() - self.last_goal_publish_wall, 3)
                        ),
                        command=[round(float(self.command.linear.x), 4), round(float(self.command.angular.z), 4)],
                        scan_forward_min=None if not math.isfinite(self.scan_forward_minimum) else round(self.scan_forward_minimum, 4),
                        scan_min=None if not math.isfinite(self.scan_minimum) else round(self.scan_minimum, 4),
                        source_stamp=message.header.stamp.to_sec(),
                        pose_frame=frame,
                        goal_changes=self.goal_changes,
                        transition_kind=self.goal_transition_kind,
                        predecessor_route_id=self.goal_predecessor_route_id,
                        transition_distance_to_previous_endpoint=self.goal_transition_distance,
                    )
                    if self.goal_source == "global_slam_frontier":
                        if self.goal_transition_kind == "prefix_continuation":
                            self.continuous_goal_transitions += 1
                        elif self.goal_transition_kind == "endpoint_divergence":
                            self.divergent_goal_transitions += 1
                        elif self.goal_transition_kind == "terminal_prefetched_successor":
                            self.terminal_goal_transitions += 1
                        else:
                            self.unknown_goal_transitions += 1
                        self.last_goal_transition_wall = time.monotonic()
            self.last_goal_xy = xy
            self.goal = xy
            self.goal_frame = frame
            self.last_goal_frame = frame
            self.goal_message = message
            self.last_goal_publish_wall = time.monotonic()

    def on_mission_goal(self, message):
        """Record the source metadata paired with the next geometric goal."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        if payload.get("event") == "target_terminal_observation":
            with self.lock:
                self._failure_record_context_locked(
                    "goal", "target_terminal_observation", payload
                )
                # The intent is published before bridge cancellation. Arm
                # the expected PREEMPTED correlation at the semantic boundary
                # so callback ordering cannot turn this normal handoff into a
                # failure episode.
                if self.pending_target_terminal_observation_preemptions <= 0:
                    self.pending_target_terminal_observation_preemptions = 1
                self._write(
                    "INFO",
                    "target_terminal_observation_intent",
                    transaction_id=int(payload.get("transaction_id", 0) or 0),
                    target_epoch=int(payload.get("target_epoch", 0) or 0),
                    target_track_id=str(payload.get("target_track_id", "") or ""),
                    reason=str(payload.get("reason", "") or ""),
                )
            return
        if payload.get("event") != "mission_goal":
            return
        with self.lock:
            self._failure_record_context_locked("goal", "mission_goal", payload)
            self.mission_goal_messages += 1
            self.goal_source = str(payload.get("source", self.goal_source))
            self.goal_transaction_id = max(
                self.goal_transaction_id,
                int(payload.get("transaction_id", 0) or 0),
            )
            self.goal_transition_kind = str(
                payload.get("transition_kind", "unknown")
            ).strip().lower() or "unknown"
            try:
                self.goal_predecessor_route_id = max(
                    0, int(payload.get("predecessor_route_id", 0) or 0)
                )
            except (TypeError, ValueError):
                self.goal_predecessor_route_id = 0
            try:
                transition_distance = payload.get(
                    "transition_distance_to_previous_endpoint"
                )
                self.goal_transition_distance = (
                    None if transition_distance is None else float(transition_distance)
                )
            except (TypeError, ValueError):
                self.goal_transition_distance = None
            goal_context = payload.get("goal_context")
            goal_context = goal_context if isinstance(goal_context, dict) else {}
            self._write(
                "INFO",
                "mission_goal_transaction",
                transaction_id=self.goal_transaction_id,
                # GoalManager owns the semantic transaction; transport action
                # generation is allocated by TEB Bridge and is zero until the
                # command is adopted. Preserve the complete lineage so the
                # short dispatch gate can match both boundaries explicitly.
                action_generation=int(payload.get("action_generation", 0) or 0),
                lifecycle_transaction_id=int(
                    payload.get("lifecycle_transaction_id", 0) or 0
                ),
                graph_transaction_id=int(
                    payload.get("graph_transaction_id", 0)
                    or goal_context.get("graph_transaction_id", 0)
                    or 0
                ),
                map_epoch=payload.get("map_epoch"),
                origin_epoch=payload.get("origin_epoch"),
                projection_epoch=payload.get("projection_epoch"),
                source=self.goal_source,
                priority=int(payload.get("priority", 0) or 0),
                route_id=int(payload.get("route_id", 0) or 0),
                route_kind=str(payload.get("route_kind", "")),
                transition_kind=self.goal_transition_kind,
                predecessor_route_id=self.goal_predecessor_route_id,
                transition_distance_to_previous_endpoint=self.goal_transition_distance,
                goal_context=payload.get("goal_context"),
                frame_id=str(payload.get("frame_id", "")),
                goal=payload.get("goal"),
            )
