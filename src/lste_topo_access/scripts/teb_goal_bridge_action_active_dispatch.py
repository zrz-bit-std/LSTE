"""Dispatch policy while a MoveBase action is already active."""

import rospy

from clock_provider import now_for

from goal_context import goal_context_identity


class TebGoalBridgeActionActiveDispatchMixin:
    def _handle_continuous_prefetch_handoff_locked(self):
        """Promote only the exact successor pre-admitted by Global Frontier.

        A pending handoff is also a bounded wait state.  Returning ``True``
        means the caller must not apply any generic replacement policy.
        """
        continuous_handoff = self.frontier_continuous_prefetch_handoff_pending
        if continuous_handoff is None:
            return False
        if (
            self.latest_intent_source == "global_slam_frontier"
            and self.latest_intent_priority == 0
            and self.latest_route_kind == "frontier_endpoint"
            and self.latest_route_id == int(continuous_handoff["successor_route_id"])
            and not self._same_goal(self.latest_goal, self.last_dispatched_goal)
        ):
            fields = {
                "route_id": int(continuous_handoff["route_id"]),
                "successor_route_id": int(continuous_handoff["successor_route_id"]),
                "feedback_distance": round(
                    float(continuous_handoff["feedback_distance"]), 3
                ),
                "pending_delta": round(
                    float(continuous_handoff["pending_delta"]), 3
                ),
                "handoff_wait_seconds": round(
                    now_for(self)
                    - float(continuous_handoff["started_monotonic"]),
                    4,
                ),
            }
            if self._send_goal_locked(
                self.latest_goal,
                reason="frontier_continuous_prefetch_handoff",
                replacement=True,
                replacement_kind="frontier_prefetched_successor",
                **fields
            ):
                self.frontier_continuous_prefetch_handoff_pending = None
                self.frontier_continuous_prefetch_handoff_count += 1
                self.terminal_count += 1
                self.publish_bridge_status(
                    "frontier_continuous_prefetch_handoff_completed", **fields
                )
                rospy.loginfo(
                    "TEB goal bridge replaced validated frontier successor "
                    "without stop: route_id=%d -> %d",
                    fields["route_id"], fields["successor_route_id"],
                )
        return True

    def _current_action_matches_latest_intent_locked(self):
        return bool(
            self._same_goal(self.latest_goal, self.last_dispatched_goal)
            and self.latest_intent_source == self.active_intent_source
            and self.latest_intent_priority == self.active_intent_priority
            and self.latest_route_kind == self.active_route_kind
            and self.latest_mission_route_kind
            == self.active_mission_route_kind
            and self.latest_route_id == self.active_route_id
            and goal_context_identity(self.latest_goal_context)
            == goal_context_identity(self.active_goal_context)
        )

    def _release_completed_turn_to_endpoint_locked(self):
        if not (
            self.turn_transition_ready
            and self.active_intent_priority == 0
            and self.latest_intent_priority == 0
            and self.active_route_kind == "frontier_turn_connector"
            and self.latest_intent_source == "global_slam_frontier"
            and self.latest_route_kind == "frontier_endpoint"
            and not self._same_goal(self.latest_goal, self.last_dispatched_goal)
        ):
            return False
        if self._send_goal_locked(
            self.latest_goal,
            reason="turn_completed_route_release",
            replacement=True,
            replacement_kind="turn_phase_transition",
            from_route_kind=self.active_route_kind,
            to_route_kind=self.latest_route_kind,
        ):
            self.turn_transition_ready = False
        return True

    def _defer_frontier_update_during_turn_locked(self):
        if not (
            self.turn_supervisor_state == "TURNING"
            and self.active_intent_priority == 0
            and self.latest_intent_priority == 0
        ):
            return False
        pending_signature = self._intent_signature_locked()
        if pending_signature != self.deferred_signature:
            self.deferred_signature = pending_signature
            self.deferred_goal_updates += 1
            rospy.loginfo_throttle(
                3.0,
                "TEB goal bridge queues frontier update during atomic turn: "
                "deferred=%d",
                self.deferred_goal_updates,
            )
        return True

    def _try_priority_intent_handoff_locked(self):
        if not (
            self.latest_intent_priority > self.active_intent_priority
            and not self._same_goal(self.latest_goal, self.last_dispatched_goal)
        ):
            return False
        pending_delta = self._pending_goal_delta_locked()
        self.priority_handoff_count += 1
        if self.allow_in_place_replacement:
            if not self._send_goal_locked(
                self.latest_goal,
                reason="higher_priority_intent",
                replacement=True,
                replacement_kind="priority_intent",
                pending_delta=round(pending_delta, 3),
                from_source=self.active_intent_source,
                to_source=self.latest_intent_source,
                handoffs=int(self.priority_handoff_count),
            ):
                self.priority_handoff_count -= 1
        else:
            self._request_cancel_for_pending_goal_locked(
                reason="higher_priority_intent",
                pending_delta=pending_delta,
            )
        return True

    def _try_safe_segment_handoff_locked(self):
        allow_route_handoff = (
            self.allow_route_continuation_replacement
            and self._safe_route_continuation_pending_locked()
        )
        allow_target_handoff = self._safe_target_segment_pending_locked()
        return bool(
            (
                self.allow_in_place_replacement
                or allow_route_handoff
                or allow_target_handoff
            )
            and self.maybe_segment_handoff_locked()
        )

    def _defer_active_goal_locked(self, reason):
        pending_signature = self._intent_signature_locked()
        if pending_signature == self.deferred_signature:
            return
        self.deferred_signature = pending_signature
        self.deferred_goal_updates += 1
        now = now_for(self)
        if now - self.deferred_goal_log_wall < 2.0:
            return
        self.deferred_goal_log_wall = now
        latest = self.latest_goal
        self.publish_bridge_status(
            "goal_deferred",
            reason=reason,
            latest_goal=[
                round(latest.pose.position.x, 3),
                round(latest.pose.position.y, 3),
            ],
        )
        rospy.loginfo(
            "TEB goal bridge queued latest goal while action is active: "
            "deferred=%d reason=%s",
            self.deferred_goal_updates,
            reason,
        )

    def _dispatch_active_action_locked(self, reason):
        """Apply ordered handoff policy without replacing healthy actions."""
        if self.persistent_execution:
            return
        if self._handle_continuous_prefetch_handoff_locked():
            return
        if self._current_action_matches_latest_intent_locked():
            return
        if self._release_completed_turn_to_endpoint_locked():
            return
        if self._defer_frontier_update_during_turn_locked():
            return
        if self._try_priority_intent_handoff_locked():
            return
        if self._try_safe_segment_handoff_locked():
            return
        self._defer_active_goal_locked(reason)
