#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TEB terminal-result callbacks for GoalManager navigation transactions."""

import copy
import json
import math

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from goal_manager_modes import CATCH_TARGET_MODE, EXPLORE_SUS_C_MODE
from lifecycle_manager import EventType


class GoalManagerTebCallbacksMixin:
    """Callbacks that commit or recover a single TEB navigation action."""

    @staticmethod
    def _bridge_status_identity(payload):
        """Extract the transport identity observed from one Bridge status."""
        def integer(*values):
            for value in values:
                try:
                    return int(value or 0)
                except (TypeError, ValueError):
                    continue
            return 0

        return {
            "route_id": integer(
                payload.get("route_id"), payload.get("active_route_id")
            ),
            "action_generation": integer(
                payload.get("action_generation"),
                payload.get("active_action_generation"),
            ),
            "transaction_id": integer(
                payload.get("transaction_id"),
                payload.get("active_goal_transaction_id"),
            ),
            "lifecycle_transaction_id": integer(
                payload.get("lifecycle_transaction_id")
            ),
            "graph_transaction_id": integer(
                payload.get("graph_transaction_id"),
                payload.get("active_graph_transaction_id"),
            ),
            "map_epoch": payload.get(
                "map_epoch", payload.get("active_route_map_epoch")
            ),
            "origin_epoch": payload.get(
                "origin_epoch", payload.get("active_origin_epoch")
            ),
            "projection_epoch": payload.get(
                "projection_epoch", payload.get("active_projection_epoch")
            ),
        }

    def gather_teb_goal_terminal(self, msg: PoseStamped):
        enqueue = getattr(self, "_enqueue_lifecycle_event", None)
        if callable(enqueue):
            return enqueue(EventType.TEB_GOAL_TERMINAL, msg)
        return GoalManagerTebCallbacksMixin.apply_teb_goal_terminal(self, msg)

    def gather_teb_bridge_status(self, message: String):
        enqueue = getattr(self, "_enqueue_lifecycle_event", None)
        if callable(enqueue):
            return enqueue(EventType.TEB_BRIDGE_STATUS, message)
        return GoalManagerTebCallbacksMixin.apply_teb_bridge_status(self, message)

    def apply_teb_bridge_status(self, message: String):
        """Consume only the bridge ACK for this manager's pending boundary."""
        try:
            payload = json.loads(message.data)
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        bridge_identity = self._bridge_status_identity(payload)
        self.bridge_action_identity = bridge_identity
        self.bridge_action_generation = int(
            bridge_identity["action_generation"]
        )
        if str(payload.get("event", "")).strip() != (
            "target_terminal_observation_released"
        ):
            return
        try:
            transaction_id = int(payload.get("transaction_id", 0) or 0)
        except (TypeError, ValueError):
            return
        pending = int(
            getattr(self, "target_terminal_observation_pending_transaction", 0)
            or 0
        )
        if transaction_id <= 0 or pending <= 0 or transaction_id != pending:
            return
        self.target_terminal_observation_ack_transaction = transaction_id
        self.next_update_time = 0.0
        self.publish_goal_arbitration(
            "target_terminal_observation_bridge_released",
            transaction_id=transaction_id,
            target_epoch=int(payload.get("target_epoch", 0) or 0),
            target_track_id=str(payload.get("target_track_id", "") or ""),
            controller_lease=str(payload.get("controller_lease", "") or ""),
        )
        rospy.loginfo(
            "GoalManager: bridge released target terminal transaction=%d; "
            "successor planning is enabled",
            transaction_id,
        )

    def on_teb_goal_terminal(self, msg: PoseStamped):
        """Release exactly one committed segment after a TEB terminal result.

        Frontier and visual-target segments share one move_base action, but
        they do not share a rolling setpoint lifecycle. A target detector
        frame may update a bearing while an action is active; only this
        callback authorizes the next target segment.
        """
        # TEB stays alive on its own mux input for hot switching. Its terminal
        # topic must not release an SA-PPO frontier commitment.
        if self.controller_mode != "teb" or self.last_goal is None:
            return
        if self.last_goal_source.startswith("target_"):
            if (
                self.last_goal_source == "target_parallax"
                and not getattr(self, "target_follow_confirmed", False)
            ):
                # The side-step is an evidence action, not a target approach
                # segment.  Its terminal only unlocks the next detector frame;
                # treating it as a completed target would consume the semantic
                # obligation before triangulation has happened.
                self.target_parallax_goal = None
                self.target_parallax_active_side = ""
                self.target_parallax_completed = True
                self.target_execution_state = "TARGET_PARALLAX_REOBSERVE"
                self.next_update_time = 0.0
                self.publish_goal_arbitration(
                    "target_parallax_viewpoint_reached",
                    target_track_id=self.target_track_id,
                    goal=[
                        round(float(msg.pose.position.x), 3),
                        round(float(msg.pose.position.y), 3),
                    ],
                    target_candidate_viewpoint_translation=round(
                        float(getattr(self, "target_candidate_viewpoint_translation", 0.0)),
                        3,
                    ),
                )
                rospy.loginfo(
                    "GoalManager: target parallax viewpoint reached; "
                    "waiting for fresh multi-view evidence"
                )
                return
            if (
                msg.header.frame_id
                and msg.header.frame_id != self.last_goal.header.frame_id
            ):
                return
            distance = math.hypot(
                msg.pose.position.x - self.last_goal.pose.position.x,
                msg.pose.position.y - self.last_goal.pose.position.y,
            )
            if distance > max(self.global_frontier_update_radius, 0.30):
                rospy.logwarn_throttle(
                    3.0,
                    "GoalManager: ignoring TEB target terminal for stale goal delta=%.2fm",
                    distance,
                )
                return
            # These routes only acquire a new observation. They do not
            # establish target-navigation ownership, so their terminal must
            # release the temporary target action before the next frontier
            # transaction is considered. Keeping ``last_goal_source`` here
            # would make the next terminal look like a target approach and
            # leave the persistent bridge holding priority=2 forever.
            if self.last_goal_source in {
                "target_candidate_room_search",
                "target_reacquisition_sweep",
                "target_viewpoint_retry",
                "target_candidate_pending",
            } and not getattr(self, "target_follow_confirmed", False):
                released_source = self.last_goal_source
                self.target_reacquire_goal = None
                self.target_reacquire_started = None
                self.target_last_goal = None
                self.target_segment_terminal_ready = False
                self.target_reinspection_pending = True
                self.target_execution_state = "TARGET_REINSPECTION"
                self.last_goal = None
                self.last_goal_source = "waiting_global_slam_frontier"
                self.goal_source = "waiting_global_slam_frontier"
                self.next_update_time = 0.0
                self.publish_goal_arbitration(
                    "target_observation_action_terminal",
                    target_track_id=self.target_track_id,
                    action_source=released_source,
                    goal=[
                        round(float(msg.pose.position.x), 3),
                        round(float(msg.pose.position.y), 3),
                    ],
                    target_ownership="released",
                    next_owner="global_slam_frontier",
                )
                rospy.loginfo(
                    "GoalManager: released unconfirmed target observation "
                    "action source=%s; frontier may select the next room view",
                    released_source,
                )
                return
            target_distance = float("inf")
            if self.target_last_goal is not None:
                target_distance = math.hypot(
                    msg.pose.position.x - self.target_last_goal.pose.position.x,
                    msg.pose.position.y - self.target_last_goal.pose.position.y,
                )
            if target_distance <= max(self.target_goal_reached_radius, 0.30):
                terminal_now = rospy.Time.now().to_sec()
                self.target_segment_terminal_ready = True
                self.target_completed_segments += 1
                self.target_approach_track_id = self.target_track_id
                self.target_goal_detection_stamp = None
                self.target_execution_state = "TARGET_CANDIDATE"
                self.target_terminal_reobserve_pending = True
                self.target_terminal_reobserve_epoch = int(
                    self.target_observation_epoch
                )
                # Two independent post-arrival frames avoid deciding from the
                # distant pre-arrival image. The observation window is the
                # bounded fallback for slower detector backends.
                self.target_terminal_reobserve_min_epoch = (
                    self.target_terminal_reobserve_epoch + 2
                )
                self.target_terminal_reobserve_until = (
                    terminal_now + self.target_observation_hold
                )
                self.target_terminal_observation_intent_sent = False
                self.target_observation_hold_until = max(
                    self.target_observation_hold_until,
                    self.target_terminal_reobserve_until,
                )
                self.set_navigation_hold(True, "target_terminal_reobserve")
                observation_gate = getattr(
                    self, "target_observation_gate", None
                )
                if observation_gate is not None:
                    observation_gate.begin_reobserve(
                        self.target_track_id,
                        int(getattr(self, "target_detector_frame_epoch", 0)),
                        terminal_now,
                    )
                approach_transaction = getattr(
                    self, "target_approach_transaction", None
                )
                if approach_transaction is not None:
                    approach_transaction.segment_arrived(
                        self.target_track_id,
                        terminal_now,
                        candidate_id=getattr(
                            self, "target_viewpoint_candidate_id", ""
                        ),
                        attempt_id=getattr(
                            self, "target_viewpoint_attempt_id", ""
                        ),
                    )
                # Completion votes must describe the post-approach view.
                self.reset_target_close_confirmation()
                self.reset_target_terminal_observation()
                self.next_update_time = 0.0
                self.publish_goal_arbitration(
                    "target_approach_terminal",
                    target_track_id=self.target_track_id,
                    approach_track_id=self.target_approach_track_id,
                    completed_segments=self.target_completed_segments,
                    target_viewpoint_candidate_id=getattr(
                        self, "target_viewpoint_candidate_id", ""
                    ),
                    target_viewpoint_attempt_id=getattr(
                        self, "target_viewpoint_attempt_id", ""
                    ),
                    goal=[
                        round(float(msg.pose.position.x), 3),
                        round(float(msg.pose.position.y), 3),
                    ],
                    target_approach_transaction=(
                        None
                        if getattr(self, "target_approach_transaction", None) is None
                        else self.target_approach_transaction.snapshot().__dict__
                    ),
                )
                rospy.loginfo(
                    "GoalManager: TEB terminal committed target segment "
                    "(%.2f,%.2f); completed_segments=%d; next segment waits "
                    "for timer boundary",
                    msg.pose.position.x,
                    msg.pose.position.y,
                    self.target_completed_segments,
                )
            return
        if self.last_goal_source != "global_slam_frontier":
            return
        if getattr(self, "target_room_entry_retry_blocked", False):
            self.target_room_entry_retry_blocked = False
            self.target_room_entry_blocked_route_id = 0
            self.publish_goal_arbitration(
                "target_room_entry_retry_unblocked",
                reason="frontier_terminal_after_failed_doorway_route",
                route_id=int(getattr(self, "global_frontier_route_id", 0) or 0),
            )
        # Match the committed frontier action history. ``last_goal`` may
        # already point at the next mission intent when move_base completes
        # the prior action.
        candidates = [
            candidate
            for candidate in self.teb_frontier_goal_history
            if not msg.header.frame_id
            or candidate.header.frame_id == msg.header.frame_id
        ]
        if not candidates:
            candidates = [self.last_goal]
        matched = min(
            candidates,
            key=lambda candidate: math.hypot(
                msg.pose.position.x - candidate.pose.position.x,
                msg.pose.position.y - candidate.pose.position.y,
            ),
        )
        distance = math.hypot(
            msg.pose.position.x - matched.pose.position.x,
            msg.pose.position.y - matched.pose.position.y,
        )
        if distance > max(self.global_frontier_update_radius, 0.30):
            rospy.logwarn_throttle(
                3.0,
                "GoalManager: ignoring TEB frontier terminal for unknown action "
                "goal delta=%.2fm latest_delta=%.2fm",
                distance,
                math.hypot(
                    msg.pose.position.x - self.last_goal.pose.position.x,
                    msg.pose.position.y - self.last_goal.pose.position.y,
                ),
            )
            return
        self.teb_terminal_goal = msg
        self.frontier_terminal_count = int(
            getattr(self, "frontier_terminal_count", 0) or 0
        ) + 1
        # The frontier node selects the successor. Let its update reach this
        # node on the first callback instead of waiting one cadence.
        self.next_update_time = 0.0
        if self.target_blocked:
            # Frontier progress, rather than a detector frame, is the proof
            # that a previously blocked visual route can be tried again.
            blocked_goal = self.target_blocked_goal
            blocked_reason = self.target_blocked_reason
            self.target_blocked = False
            self.target_blocked_goal = None
            self.target_blocked_since = None
            self.target_blocked_reason = ""
            self.target_execution_state = "TARGET_CANDIDATE"
            self.publish_goal_arbitration(
                "target_route_released",
                reason="frontier_progress",
                blocked_reason=blocked_reason,
                blocked_goal=(
                    None
                    if blocked_goal is None
                    else [
                        round(float(blocked_goal.pose.position.x), 3),
                        round(float(blocked_goal.pose.position.y), 3),
                    ]
                ),
                frontier_goal=[
                    round(float(msg.pose.position.x), 3),
                    round(float(msg.pose.position.y), 3),
                ],
            )
            rospy.loginfo(
                "GoalManager: frontier progress reopens blocked target route "
                "reason=%s frontier=(%.2f,%.2f)",
                blocked_reason,
                msg.pose.position.x,
                msg.pose.position.y,
            )
        rospy.loginfo(
            "GoalManager: TEB terminal matched frontier goal (%.2f,%.2f); "
            "next frontier update may replace it",
            msg.pose.position.x,
            msg.pose.position.y,
        )

    def gather_teb_goal_failure(self, message: String):
        enqueue = getattr(self, "_enqueue_lifecycle_event", None)
        if callable(enqueue):
            return enqueue(EventType.TEB_GOAL_FAILURE, message)
        return GoalManagerTebCallbacksMixin.apply_teb_goal_failure(
            self, message
        )

    def on_teb_goal_failure(self, message: String):
        """Release a failed visual action to the map planner.

        This is event-driven: neither a retry timer nor a detector frame may
        re-submit a failed target before a frontier action makes map progress.
        """
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict) or self.controller_mode != "teb":
            return
        failure_status = str(payload.get("status", "")).strip().upper()
        failure_reason = str(
            payload.get("reason", "target_route_failed")
        ).strip().lower()
        fail_fast_failure = bool(
            failure_status in {
                "ABORTED",
                "REJECTED",
                "RECALLED",
                "LOST",
                "NO_PROGRESS",
                "NAVFN_NO_PATH",
                "FAILED",
            }
            or any(
                marker in failure_reason
                for marker in (
                    "abort",
                    "no_progress",
                    "stalled",
                    "unreachable",
                    "no_path",
                )
            )
        )
        if (
            getattr(self, "last_goal_source", "") == "target_parallax"
            and not getattr(self, "target_follow_confirmed", False)
            and not fail_fast_failure
        ):
            # A failed side-step is a failed viewpoint attempt, not evidence
            # that the semantic target or its Place is blocked.  Let the
            # second deterministic side choice be compiled on the next tick.
            self.target_parallax_goal = None
            active_side = str(
                getattr(self, "target_parallax_active_side", "") or ""
            )
            if active_side and active_side not in getattr(
                self, "target_parallax_failed_sides", []
            ):
                self.target_parallax_failed_sides.append(active_side)
            self.target_parallax_active_side = ""
            self.target_execution_state = "TARGET_PARALLAX_RETRY"
            self.target_parallax_completed = False
            self.next_update_time = 0.0
            self.publish_goal_arbitration(
                "target_parallax_viewpoint_failed",
                target_track_id=self.target_track_id,
                reason=str(payload.get("reason", "controller_failure")),
                attempt=int(getattr(self, "target_parallax_attempts", 0)),
                failed_side=active_side or None,
            )
            rospy.logwarn(
                "GoalManager: target parallax viewpoint failed; trying "
                "another observation side"
            )
            return
        raw_goal = payload.get("goal")
        failure_goal = None
        if isinstance(raw_goal, (list, tuple)) and len(raw_goal) >= 2:
            failure_goal = self.make_goal_pose(
                (float(raw_goal[0]), float(raw_goal[1]), 0.0), 0.0
            )
            failure_goal.header.frame_id = (
                str(payload.get("goal_frame", "map") or "map")
                .strip()
                .lstrip("/")
                or "map"
            )
        if self.last_goal is None and failure_goal is None:
            # A failure without either the current mission goal or a goal in
            # its payload has no safe identity to release. Keep the event in
            # metrics, but do not invent a route from an unbound callback.
            return
        failure_track_id = str(payload.get("target_track_id", "")).strip()
        if (
            failure_track_id
            and self.target_track_id
            and failure_track_id != self.target_track_id
        ):
            rospy.loginfo(
                "GoalManager: ignore target failure from obsolete track=%s current=%s",
                failure_track_id,
                self.target_track_id,
            )
            return
        # A queued loss-driven reacquisition pose does not own the action.
        # Match the failure against the committed target segment instead.
        associated_goal = self.target_last_goal
        if associated_goal is None and str(
            getattr(self, "last_goal_source", "") or ""
        ).startswith("target_"):
            associated_goal = self.last_goal
        if associated_goal is None:
            # The bridge publishes the failed endpoint explicitly. It is safe
            # to use that immutable endpoint for the release transaction when
            # GoalManager already cleared its mutable goal cache.
            associated_goal = failure_goal
        if associated_goal is None:
            return
        target_delta = None
        if failure_goal is not None:
            target_delta = self.pose_distance(failure_goal, associated_goal)
        if (
            target_delta is not None
            and target_delta > max(self.target_goal_reached_radius, 0.75)
            and not failure_track_id
        ):
            rospy.logwarn(
                "GoalManager: ignore stale target failure delta=%.2fm current=(%.2f,%.2f)",
                target_delta,
                associated_goal.pose.position.x,
                associated_goal.pose.position.y,
            )
            return

        now = rospy.Time.now().to_sec()
        blocked_goal = copy.deepcopy(associated_goal)
        if fail_fast_failure:
            if self.last_goal_source == "target_room_entry":
                # Doorway-entry is a task route, not a visual target. Release
                # only this cached handoff so frontier exploration can obtain
                # a new map snapshot and prove the doorway again.
                self.target_room_entry_goal = None
                self.target_room_entry_retry_blocked = True
                self.target_room_entry_blocked_route_id = int(
                    getattr(self, "global_frontier_route_id", 0) or 0
                )
                self.publish_goal_arbitration(
                    "target_room_entry_route_blocked",
                    reason="controller_failure_requires_frontier_progress",
                    route_id=self.target_room_entry_blocked_route_id,
                )
            release = getattr(self, "release_failed_target_route", None)
            if callable(release):
                release(
                    reason=failure_reason or "target_route_failed",
                    status=failure_status or "FAILED",
                    failure_goal=failure_goal,
                    target_track_id=failure_track_id or self.target_track_id,
                )
            self.target_reacquire_goal = None
            self.target_reacquire_started = None
            self.target_reacquire_attempts = self.target_reacquire_max_attempts
            self.effective_mode = EXPLORE_SUS_C_MODE
            self.pub_access_mode.publish(String(data=self.effective_mode))
            self.set_navigation_hold(False, "target_route_failed")
            self.goal_source = "waiting_global_slam_frontier"
            self.next_update_time = 0.0
            return
        approach_transaction = getattr(
            self, "target_approach_transaction", None
        )
        viewpoint_candidate_id = str(
            payload.get(
                "target_viewpoint_candidate_id",
                getattr(self, "target_viewpoint_candidate_id", ""),
            )
            or ""
        )
        viewpoint_attempt_id = str(
            payload.get(
                "target_viewpoint_attempt_id",
                getattr(self, "target_viewpoint_attempt_id", ""),
            )
            or ""
        )
        if approach_transaction is not None:
            failed_viewpoint = approach_transaction.mark_viewpoint_failed(
                failure_track_id or self.target_track_id,
                now,
                candidate_id=viewpoint_candidate_id,
                attempt_id=viewpoint_attempt_id,
                reason=str(payload.get("reason", "target_route_failed")),
            )
            if failed_viewpoint is not None and not (
                approach_transaction.viewpoint_alternatives_exhausted()
            ):
                # This is a local option failure, not semantic target loss.
                # Keep the target identity and let the next timer cycle select
                # another route candidate from the same option set.
                self.target_blocked = False
                self.target_blocked_goal = None
                self.target_blocked_since = None
                self.target_blocked_reason = ""
                self.target_failure_count += 1
                self.target_last_goal = None
                self.target_goal_detection_stamp = None
                self.target_segment_terminal_ready = False
                self.target_cache_advances = 0
                self.target_terminal_blind_advances = 0
                self.target_execution_state = "TARGET_VIEWPOINT_RETRY"
                self.effective_mode = CATCH_TARGET_MODE
                self.pub_access_mode.publish(String(data=self.effective_mode))
                self.goal_source = "target_viewpoint_retry"
                self.next_update_time = 0.0
                self.publish_goal_arbitration(
                    "target_viewpoint_attempt_failed",
                    reason=str(payload.get("reason", "target_route_failed")),
                    status=str(payload.get("status", "unknown")),
                    target_track_id=failure_track_id or self.target_track_id,
                    target_viewpoint_candidate_id=failed_viewpoint.candidate_id,
                    target_viewpoint_attempt_id=failed_viewpoint.attempt_id,
                    target_viewpoint_failure_reason=failed_viewpoint.failure_reason,
                    target_viewpoint_alternatives_remaining=(
                        approach_transaction.viewpoint_alternative_available(
                            excluding=failed_viewpoint.candidate_id
                        )
                    ),
                    target_approach_transaction=(
                        approach_transaction.snapshot().__dict__
                    ),
                )
                rospy.logwarn(
                    "GoalManager: target viewpoint failed; retaining target "
                    "track and selecting another option candidate=%s attempt=%s",
                    failed_viewpoint.candidate_id,
                    failed_viewpoint.attempt_id,
                )
                return
            if failed_viewpoint is not None:
                self.publish_goal_arbitration(
                    "target_viewpoint_alternatives_exhausted",
                    reason="all_known_viewpoints_closed",
                    target_track_id=failure_track_id or self.target_track_id,
                    target_viewpoint_candidate_id=failed_viewpoint.candidate_id,
                    target_viewpoint_attempt_id=failed_viewpoint.attempt_id,
                    target_viewpoint_ledger=(
                        approach_transaction.viewpoint_ledger.snapshot().to_dict()
                    ),
                )
        if approach_transaction is not None:
            approach_transaction.mark_failed(
                now,
                str(payload.get("reason", "target_route_failed")),
            )
        self.target_blocked = True
        self.target_blocked_goal = blocked_goal
        self.target_blocked_since = now
        self.target_blocked_reason = str(payload.get("reason", "unknown"))
        self.target_failure_count += 1
        self.target_execution_state = "TARGET_BLOCKED"
        # Preserve visual evidence for a later re-plan, but remove the failed
        # segment so the next target step starts at the new robot pose.
        self.target_last_goal = None
        self.target_goal_detection_stamp = None
        self.target_segment_terminal_ready = False
        self.target_cache_advances = 0
        self.target_terminal_blind_advances = 0
        # A loss-driven reacquisition pose belongs to the failed transaction;
        # it must not preempt the recovery frontier.
        self.target_reacquire_goal = None
        self.target_reacquire_started = None
        self.target_reacquire_attempts = self.target_reacquire_max_attempts
        self.effective_mode = EXPLORE_SUS_C_MODE
        self.pub_access_mode.publish(String(data=self.effective_mode))
        self.goal_source = "target_route_failed"
        self.next_update_time = 0.0
        self.publish_goal_arbitration(
            "target_route_failed",
            reason=self.target_blocked_reason,
            status=str(payload.get("status", "unknown")),
            target_epoch=int(payload.get("target_epoch", self.target_observation_epoch)),
            failure_count=int(self.target_failure_count),
            blocked_goal=[
                round(float(blocked_goal.pose.position.x), 3),
                round(float(blocked_goal.pose.position.y), 3),
            ],
            blocked_goal_frame=blocked_goal.header.frame_id,
            target_approach_transaction=(
                None
                if getattr(self, "target_approach_transaction", None) is None
                else self.target_approach_transaction.snapshot().__dict__
            ),
        )
        semantic_hint = self.target_semantic_hint_map(
            distance=self.target_pursuit_hint_distance(),
            use_observation_origin=True,
        )
        self.request_global_frontier_replan(
            "target_route_failed",
            target_track_id=failure_track_id or self.target_track_id,
            blocked_goal=[
                round(float(blocked_goal.pose.position.x), 3),
                round(float(blocked_goal.pose.position.y), 3),
            ],
            # A direct target remains actionable even if local geometry
            # invalidates its short approach route; monitor context is not
            # part of this recovery path.
            target_pursuit=bool(semantic_hint is not None),
            target_room_claim=True,
            semantic_hint_map=(
                None
                if semantic_hint is None
                else [
                    round(float(semantic_hint[0]), 3),
                    round(float(semantic_hint[1]), 3),
                ]
            ),
        )
        rospy.logwarn(
            "GoalManager: target route failed; wait for a frontier branch "
            "recomputed from the current pose"
        )

    def apply_teb_goal_terminal(self, msg):
        return GoalManagerTebCallbacksMixin.on_teb_goal_terminal(self, msg)

    def apply_teb_goal_failure(self, message):
        return GoalManagerTebCallbacksMixin.on_teb_goal_failure(
            self, message
        )
