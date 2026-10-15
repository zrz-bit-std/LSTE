"""In-place segment replacement policy for the TEB action bridge."""

import math

import rospy

from clock_provider import now_for


class TebGoalBridgeHandoffSegmentMixin:
    def maybe_segment_handoff_locked(self):
        """Replace one compatible near-endpoint segment without a stop gap."""
        context = self._segment_handoff_context_locked()
        if context is None:
            return False
        limits = self._segment_handoff_limits_locked(context)
        if limits is None:
            return False
        return self._send_segment_replacement_locked(context, limits)

    def _segment_handoff_context_locked(self):
        """Validate semantic ownership before considering numeric thresholds."""
        if (
            not self.action_active
            or self.handoff_requested
            or self.latest_goal is None
            or self.last_dispatched_goal is None
            or self.active_feedback_distance is None
            or self.latest_intent_priority != self.active_intent_priority
            or self.active_intent_priority not in (0, 2)
        ):
            return None
        if (
            self.turn_supervisor_state == "TURNING"
            and self.active_intent_priority == 0
            and self.latest_intent_priority == 0
        ):
            # A turn is a semantic action: the supervisor owns angle closure.
            return None
        if now_for(self) - self.last_dispatch_monotonic < self.min_update_interval:
            return None
        pending_delta = self._pending_goal_delta_locked()
        frontier_branch = bool(
            self.active_intent_priority == 0
            and pending_delta > self.frontier_replacement_max_delta
        )
        route_continuation = self._route_continuation_for_replacement_locked()
        # A different frontier branch remains a normal action boundary. A
        # visual update must also advance the same stable target track.
        if self.active_intent_priority == 0 and not route_continuation:
            return None
        if (
            self.active_intent_priority >= 2
            and not self._safe_target_segment_pending_locked()
        ):
            return None
        return {
            "pending_delta": pending_delta,
            "frontier_branch": frontier_branch,
            "route_continuation": route_continuation,
        }

    def _route_continuation_for_replacement_locked(self):
        """Allow replacement only within one GlobalFrontier route identity."""
        route_kinds = {
            "frontier_connector",
            "frontier_turn_connector",
            "frontier_endpoint",
        }
        return bool(
            self.active_intent_priority == 0
            and self.latest_intent_priority == 0
            and self.active_intent_source == "global_slam_frontier"
            and self.latest_intent_source == "global_slam_frontier"
            and self.active_route_id > 0
            and self.active_route_id == self.latest_route_id
            and self.latest_route_kind in route_kinds
            and self.active_route_kind in route_kinds
        )

    def _segment_handoff_limits_locked(self, context):
        """Apply distance, delta, and sharp-branch limits to one safe route."""
        pending_delta = context["pending_delta"]
        frontier_branch = context["frontier_branch"]
        route_continuation = context["route_continuation"]
        minimum_distance = (
            self.frontier_early_handoff_min_distance
            if frontier_branch
            else self.in_place_replacement_min_distance
        )
        maximum_distance = (
            self.frontier_early_handoff_max_distance
            if frontier_branch
            else self.in_place_replacement_max_distance
        )
        if route_continuation:
            # Validated connectors may replace until just outside TEB terminal
            # tolerance, avoiding a race to SUCCEEDED and a stop/restart pulse.
            minimum_distance = self.frontier_sharp_replacement_min_distance
            maximum_distance = self.frontier_early_handoff_max_distance
        maximum_delta = (
            self.frontier_early_handoff_max_delta
            if frontier_branch
            else (
                self.in_place_replacement_max_delta
                if self.active_intent_priority >= 2
                else self.frontier_replacement_max_delta
            )
        )
        heading_delta, sharp_frontier_branch = self._sharp_frontier_branch_locked(
            context, minimum_distance, maximum_distance
        )
        if sharp_frontier_branch is None:
            return None
        if sharp_frontier_branch and not route_continuation:
            minimum_distance = self.frontier_sharp_replacement_min_distance
            maximum_distance = self.frontier_stale_recovery_max_distance
        if self.active_feedback_distance > maximum_distance:
            return None
        if self.active_feedback_distance < minimum_distance:
            rospy.loginfo_throttle(
                3.0,
                "TEB goal bridge waits for terminal action near goal: "
                "feedback_distance=%.2fm min_replacement_distance=%.2fm",
                self.active_feedback_distance,
                minimum_distance,
            )
            return None
        minimum_delta = (
            self.target_early_handoff_min_delta
            if self.active_intent_priority >= 2
            else self.frontier_replacement_min_delta
        )
        if pending_delta < minimum_delta:
            return None
        if pending_delta > maximum_delta:
            rospy.loginfo_throttle(
                3.0,
                "TEB goal bridge defers oversized replacement: "
                "pending_delta=%.2fm max_hot_start_delta=%.2fm "
                "feedback_distance=%.2fm intent=%s",
                pending_delta,
                maximum_delta,
                self.active_feedback_distance,
                self.active_intent_source,
            )
            return None
        return {
            "heading_delta": heading_delta,
            "sharp_frontier_branch": sharp_frontier_branch,
        }

    def _sharp_frontier_branch_locked(self, context, _minimum_distance, _maximum_distance):
        """Return sharp-branch state, rejecting an unsafe topology transition."""
        if not context["frontier_branch"]:
            return None, False
        heading_delta = self._pending_heading_delta_locked()
        sharp_frontier_branch = bool(
            heading_delta is not None
            and heading_delta > self.frontier_early_handoff_max_heading_delta
        )
        if not sharp_frontier_branch or context["route_continuation"]:
            return heading_delta, sharp_frontier_branch
        # A sharp branch is normally held until terminal completion. Near a
        # terminal endpoint, hot replacement prevents the zero-speed gap.
        if (
            self.frontier_sharp_replacement_min_distance
            <= self.active_feedback_distance
            <= self.frontier_stale_recovery_max_distance
            and context["pending_delta"] <= self.frontier_early_handoff_max_delta
        ):
            return heading_delta, True
        rospy.loginfo_throttle(
            3.0,
            "TEB goal bridge defers sharp frontier replacement: "
            "heading_delta=%.1fdeg feedback_distance=%.2fm",
            math.degrees(heading_delta),
            self.active_feedback_distance,
        )
        return heading_delta, None

    def _send_segment_replacement_locked(self, context, limits):
        """Submit the approved replacement and update the matching counter."""
        pending_delta = context["pending_delta"]
        route_continuation = context["route_continuation"]
        sharp_frontier_branch = limits["sharp_frontier_branch"]
        heading_delta = limits["heading_delta"]
        feedback_distance = self.active_feedback_distance
        replacement_kind = self._segment_replacement_kind_locked(
            route_continuation, sharp_frontier_branch
        )
        if replacement_kind == "target_segment":
            self.target_segment_handoff_count += 1
            handoffs = self.target_segment_handoff_count
        else:
            self.frontier_segment_handoff_count += 1
            handoffs = self.frontier_segment_handoff_count
        sent = self._send_goal_locked(
            self.latest_goal,
            reason=(
                "sharp_frontier_transition"
                if sharp_frontier_branch
                else "near_segment_end"
            ),
            replacement=True,
            replacement_kind=replacement_kind,
            pending_delta=round(pending_delta, 3),
            feedback_distance=round(feedback_distance, 3),
            heading_delta_deg=(
                None if heading_delta is None else round(math.degrees(heading_delta), 1)
            ),
            handoffs=int(handoffs),
            early_branch=bool(context["frontier_branch"]),
            topology_transition=bool(sharp_frontier_branch and not route_continuation),
            route_continuation=bool(route_continuation),
        )
        if not sent:
            if replacement_kind == "target_segment":
                self.target_segment_handoff_count -= 1
            else:
                self.frontier_segment_handoff_count -= 1
            return False
        rospy.loginfo(
            "TEB goal bridge replaced %s segment in-place: "
            "feedback_distance=%.2fm pending_delta=%.2fm handoffs=%d",
            replacement_kind,
            feedback_distance,
            pending_delta,
            handoffs,
        )
        return True

    def _segment_replacement_kind_locked(self, route_continuation, sharp_frontier_branch):
        if self.active_intent_priority >= 2:
            return "target_segment"
        if not route_continuation:
            return "frontier_sharp_branch" if sharp_frontier_branch else "frontier_segment"
        if self.latest_route_kind == "frontier_connector":
            return "frontier_route_connector"
        if self.latest_route_kind == "frontier_turn_connector":
            return "frontier_route_turn_connector"
        return "frontier_route_endpoint"

    def _safe_route_continuation_pending_locked(self):
        """Return whether the queued goal is a non-turn segment of this route."""
        return bool(
            self._route_continuation_for_replacement_locked()
            and self.latest_route_kind != "frontier_turn_connector"
        )

    def _safe_target_segment_pending_locked(self):
        """Return whether the queued goal advances the active visual track."""
        return bool(
            self.active_intent_priority == 2
            and self.latest_intent_priority == 2
            and self.active_intent_source.startswith("target_")
            and self.latest_intent_source.startswith("target_")
            and bool(self.active_target_track_id)
            and self.active_target_track_id == self.latest_target_track_id
        )
