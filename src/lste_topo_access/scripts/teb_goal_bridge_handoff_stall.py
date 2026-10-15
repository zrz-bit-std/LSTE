"""Stall ownership policy for an active TEB move_base action."""

import math

import rospy
from actionlib_msgs.msg import GoalStatus

from clock_provider import now_for


class TebGoalBridgeHandoffStallMixin:
    def maybe_handoff_locked(self):
        """Handle a real no-progress failure without stealing healthy routes."""
        # GoalManager deliberately freezes the base while collecting a
        # close-range target observation.  This gate is a mission-level
        # observation transaction, not controller no-progress; the same bridge
        # owner must see it before evaluating the stall clock.
        if bool(getattr(self, "navigation_hold", False)):
            return
        now = now_for(self)
        if not self._handoff_is_eligible_locked(now):
            return
        pending_delta = self._pending_goal_delta_locked()
        if self._try_frontier_progress_transition_locked(pending_delta):
            return
        stalled = self._action_is_stalled_locked(now)
        target_progress_basis, target_stalled = self._target_stall_state_locked(
            now, pending_delta
        )
        if (
            pending_delta < self.handoff_distance
            and not target_stalled
            and not stalled
        ):
            return
        # Endpoint proximity alone is not a failure: move_base may publish
        # SUCCEEDED on this callback turn. Only a real no-progress window may
        # transfer an action away from its current owner.
        if not stalled and not target_stalled:
            return
        if self._defer_native_teb_reorientation_locked(now, pending_delta):
            return
        if self.active_intent_priority < 2:
            self._report_stalled_frontier_locked(now, pending_delta)
            return
        self.frontier_stale_wait_started_monotonic = 0.0
        if target_stalled:
            self._latch_target_failure_locked(
                "target_%s_stalled" % target_progress_basis,
                status_text="NO_PROGRESS",
                cancel_action=True,
            )
            return
        self._cancel_stalled_action_locked(now, pending_delta)

    def _handoff_is_eligible_locked(self, now):
        """Return whether the watchdog may inspect the active action."""
        if self.frontier_continuous_prefetch_handoff_pending is not None:
            self._expire_frontier_continuous_prefetch_handoff_locked(now)
            return False
        # A persistent frontier endpoint has already emitted its semantic
        # terminal while the transport action is intentionally held for the
        # next StreamingNavfn mission command.  Its zero command is expected;
        # the generic no-progress watchdog must not turn that boundary into a
        # controller failure before the successor arrives.
        if getattr(self, "persistent_frontier_endpoint_pending", None) is not None:
            return False
        if (
            not self.action_active
            or self.handoff_requested
            or self.latest_goal is None
            or self.last_dispatched_goal is None
        ):
            return False
        if (
            self.turn_supervisor_state == "TURNING"
            and self.active_intent_priority == 0
            and self.latest_intent_priority == 0
        ):
            # The supervisor owns this short atomic turn. A progress watchdog
            # must not reintroduce a stop/restart race while it closes yaw.
            return False
        return now - self.last_dispatch_monotonic >= self.handoff_min_interval

    def _try_frontier_progress_transition_locked(self, pending_delta):
        """Let healthy frontier endpoint and prefetch transitions complete first."""
        # An endpoint inside its observation region is not a failed point goal.
        # These checks intentionally precede the generic no-progress watchdog.
        return bool(
            self._start_frontier_continuous_prefetch_handoff_locked(pending_delta)
            or self._complete_frontier_observation_locked(pending_delta)
        )

    def _action_is_stalled_locked(self, now):
        # Goal-distance feedback and Navfn remaining distance can move
        # backwards/forwards during local oscillation.  Use the physical
        # odometry clock for the controller health decision; the old feedback
        # clock remains a compatibility fallback when no pose was received.
        progress_time = float(
            getattr(self, "active_motion_progress_monotonic", 0.0) or 0.0
        )
        if progress_time <= 0.0:
            progress_time = float(
                getattr(self, "active_progress_monotonic", 0.0) or 0.0
            )
        return bool(
            progress_time > 0.0
            and now - progress_time >= self.progress_timeout
        )

    def _target_stall_state_locked(self, now, pending_delta):
        """Return the target-progress evidence for mission-level recovery."""
        progress_basis, progress_age = self._target_progress_state_locked(now)
        target_stalled = bool(
            self.active_intent_priority >= 2
            and progress_age >= self.progress_timeout
            and pending_delta < self.handoff_distance
        )
        return progress_basis, target_stalled

    def _defer_native_teb_reorientation_locked(self, now, pending_delta):
        """Keep one healthy native TEB rotation under its bounded yaw budget."""
        if not self._teb_reorientation_progressing_locked(now):
            return False
        self.teb_reorientation_deferrals += 1
        if now - self.teb_reorientation_last_status_monotonic >= 1.0:
            self.teb_reorientation_last_status_monotonic = now
            duration = now - self.teb_reorientation_started_monotonic
            progress_age = now - self.teb_reorientation_last_yaw_progress_monotonic
            self.publish_bridge_status(
                "teb_in_place_reorientation",
                reason="native_teb_yaw_progress",
                duration=round(duration, 3),
                yaw_progress=round(self.teb_reorientation_total_yaw, 4),
                yaw_progress_age=round(progress_age, 3),
                extension_budget=round(self.teb_reorientation_max_extension, 3),
                pending_delta=round(pending_delta, 3),
                feedback_distance=(
                    None
                    if self.active_feedback_distance is None
                    else round(self.active_feedback_distance, 3)
                ),
            )
            rospy.loginfo(
                "TEB goal bridge defers stale handoff during native "
                "reorientation: yaw_progress=%.1fdeg duration=%.1fs",
                math.degrees(self.teb_reorientation_total_yaw),
                duration,
            )
        return True

    def _report_stalled_frontier_locked(self, now, pending_delta):
        """Record frontier stall evidence, leaving recovery to move_base."""
        physical_progress_time = float(
            getattr(self, "active_motion_progress_monotonic", 0.0) or 0.0
        )
        physical_stalled = bool(
            physical_progress_time > 0.0
            and now - physical_progress_time >= self.progress_timeout
        )
        if bool(getattr(self, "persistent_execution", False)) and physical_stalled:
            # Persistent execution owns one long-lived action.  Leaving a
            # physically stalled frontier to MoveBase lets its recovery loop
            # consume the experiment budget without producing a semantic
            # route failure.  Use the same typed terminal path as feedback
            # loss so Global Frontier can release the exact lease and choose a
            # successor.
            self._cancel_stalled_frontier_locked(
                now,
                pending_delta,
                reason="controller_stall_no_physical_progress",
            )
            return
        if self._feedback_stall_requires_terminal_locked(now):
            self._cancel_stalled_frontier_locked(now, pending_delta)
            return
        # A frontier route belongs to move_base after dispatch. TEB can make no
        # XY progress while resolving a corner or its own recovery. Cancelling
        # it here would turn that local recovery into PREEMPTED and an avoidable
        # zero-velocity gap. GlobalFrontier observes conclusive terminal state.
        if now - self.frontier_stale_wait_started_monotonic < 1.0:
            return
        self.frontier_stale_wait_started_monotonic = now
        self.publish_bridge_status(
            "frontier_stall_observed",
            pending_delta=round(pending_delta, 3),
            feedback_distance=(
                None
                if self.active_feedback_distance is None
                else round(self.active_feedback_distance, 3)
            ),
            progress_age=round(now - self.active_progress_monotonic, 3)
            if self.active_progress_monotonic > 0.0
            else None,
            action_owner="move_base",
        )
        rospy.loginfo(
            "TEB goal bridge leaves stalled frontier action to move_base: "
            "pending_delta=%.2fm feedback_distance=%s progress_age=%.1fs",
            pending_delta,
            "n/a"
            if self.active_feedback_distance is None
            else "%.2f" % self.active_feedback_distance,
            now - self.active_progress_monotonic
            if self.active_progress_monotonic > 0.0
            else 0.0,
        )

    def _feedback_stall_requires_terminal_locked(self, now):
        """Return true for a persistent route with no recoverable TEB feedback."""
        if not (
            bool(getattr(self, "persistent_execution", False))
            and bool(getattr(self, "action_active", False))
            and str(
                getattr(self, "active_intent_source", "unknown") or "unknown"
            ).strip().lower() == "global_slam_frontier"
            and int(getattr(self, "active_intent_priority", 0) or 0) == 0
            and int(getattr(self, "active_route_id", 0) or 0) > 0
            and not bool(getattr(self, "teb_feedback_valid", False))
        ):
            return False
        feedback_time = float(
            getattr(self, "latest_teb_feedback_monotonic", 0.0) or 0.0
        )
        feedback_age = (
            float(now) - feedback_time if feedback_time > 0.0 else float("inf")
        )
        feedback_missing = bool(
            not getattr(self, "teb_feedback_valid", False)
            or feedback_time <= 0.0
            or feedback_age >= float(self.progress_timeout)
        )
        if not feedback_missing:
            return False
        progress_started = float(
            getattr(self, "active_progress_monotonic", 0.0) or 0.0
        )
        if progress_started <= 0.0:
            progress_started = float(
                getattr(self, "last_dispatch_monotonic", 0.0) or 0.0
            )
        return bool(
            progress_started > 0.0
            and float(now) - progress_started >= float(self.progress_timeout)
        )

    def _cancel_stalled_frontier_locked(self, now, pending_delta, reason=None):
        """Close a feedback-deadlocked persistent frontier route once."""
        route_id = int(getattr(self, "active_route_id", 0) or 0)
        if route_id <= 0:
            return False
        contract = getattr(self, "active_action_contract", None)
        contract = dict(contract) if hasattr(contract, "get") else {}
        action_key = self._action_identity_key_locked(contract)
        if self._terminal_tombstone_for_key_locked(action_key) is not None:
            self.publish_bridge_status(
                "controller_stall_duplicate_ignored",
                route_id=route_id,
                action_generation=(
                    0 if action_key is None else int(action_key[1])
                ),
                reason="terminal_key_already_applied",
                dedupe_key=(
                    None
                    if action_key is None
                    else [int(action_key[0]), int(action_key[1])]
                ),
            )
            return False
        progress_time = float(
            getattr(self, "active_progress_monotonic", 0.0) or 0.0
        )
        if progress_time <= 0.0:
            progress_time = float(
                getattr(self, "last_dispatch_monotonic", 0.0) or 0.0
            )
        progress_age = (
            None
            if progress_time <= 0.0
            else max(0.0, float(now) - progress_time)
        )
        feedback_time = float(
            getattr(self, "latest_teb_feedback_monotonic", 0.0) or 0.0
        )
        feedback_age = (
            None
            if feedback_time <= 0.0
            else max(0.0, float(now) - feedback_time)
        )
        reason = str(reason or "controller_stall_feedback_lost")
        feedback_was_valid = bool(getattr(self, "teb_feedback_valid", False))
        feedback_was_invalid_reason = str(
            getattr(self, "teb_feedback_invalid_reason", "no_feedback")
            or "no_feedback"
        )
        invalidate = getattr(self, "_invalidate_teb_feedback_locked", None)
        if callable(invalidate):
            # Make the causal boundary explicit before publishing the typed
            # stall. The feedback channel has no wire identity, so this event
            # must be emitted while the old route/action generation is still
            # the bridge owner.
            invalidate("controller_stall", clear_planner=True)
        self.handoff_count += 1
        self.publish_bridge_status(
            "controller_stall",
            status=int(GoalStatus.ABORTED),
            status_text="CONTROLLER_STALL",
            reason=reason,
            route_id=route_id,
            route_kind=str(getattr(self, "active_route_kind", "") or ""),
            mission_route_kind=str(
                getattr(self, "active_mission_route_kind", "") or ""
            ),
            action_generation=int(getattr(self, "action_generation", 0) or 0),
            transaction_id=int(
                getattr(self, "active_goal_transaction_id", 0) or 0
            ),
            lifecycle_transaction_id=int(
                self._canonical_lifecycle_transaction_id()
            ),
            graph_transaction_id=int(
                getattr(self, "active_graph_transaction_id", 0) or 0
            ),
            map_epoch=getattr(self, "active_route_map_epoch", None),
            origin_epoch=getattr(self, "active_origin_epoch", None),
            projection_epoch=getattr(self, "active_projection_epoch", None),
            feedback_valid=feedback_was_valid,
            feedback_invalid_reason=feedback_was_invalid_reason,
            feedback_age=(
                None if feedback_age is None else round(feedback_age, 4)
            ),
            pending_delta=round(float(pending_delta), 4),
            progress_age=(
                None if progress_age is None else round(progress_age, 4)
            ),
            physical_motion_progress_age=(
                None
                if getattr(self, "active_motion_progress_monotonic", 0.0) <= 0.0
                else round(
                    float(now - self.active_motion_progress_monotonic), 4
                )
            ),
            controller_lease="revoked",
        )
        # Reuse the existing failed-terminal path so the global executor sees
        # an identity-checked route failure and can start its normal recovery.
        self._publish_failed_terminal_locked(
            GoalStatus.ABORTED,
            False,
            contract or getattr(self, "active_action_contract", None),
            failure_reason=reason,
        )
        clear_intent = getattr(
            self, "_clear_frontier_intent_after_release_locked", None
        )
        if callable(clear_intent):
            # The terminal was already published with the old contract. Clear
            # only the replayable latest goal, so the next bridge tick cannot
            # dispatch the failed endpoint before Global Frontier replans.
            clear_intent()
        rospy.logwarn(
            "TEB goal bridge closed persistent frontier controller stall: "
            "route_id=%d feedback_reason=%s progress_age=%.3fs",
            route_id,
            getattr(self, "teb_feedback_invalid_reason", "no_feedback"),
            0.0 if progress_age is None else progress_age,
        )
        return True

    def _cancel_stalled_action_locked(self, now, pending_delta):
        """Cancel an eligible non-frontier stale action and report its evidence."""
        reason = "no_feedback_progress"
        self.handoff_count += 1
        self._commit_termination(
            "stalled_action_handoff",
            source_goal=getattr(self, "last_dispatched_goal", None),
            action_contract=getattr(self, "active_action_contract", None),
            watchdog_reason="stalled_action_handoff",
            publish_terminal=False,
            arm_watchdog=False,
        )
        self.publish_bridge_status(
            "handoff_requested",
            reason=reason,
            pending_delta=round(pending_delta, 3),
            feedback_distance=(
                None
                if self.active_feedback_distance is None
                else round(self.active_feedback_distance, 3)
            ),
            progress_age=round(now - self.active_progress_monotonic, 3)
            if self.active_progress_monotonic > 0.0
            else None,
            handoffs=int(self.handoff_count),
        )
        rospy.logwarn(
            "TEB goal bridge handing off stale action: reason=%s "
            "pending_delta=%.2fm feedback_distance=%s progress_age=%.1fs",
            reason,
            pending_delta,
            "n/a"
            if self.active_feedback_distance is None
            else "%.2f" % self.active_feedback_distance,
            now - self.active_progress_monotonic
            if self.active_progress_monotonic > 0.0
            else 0.0,
        )
