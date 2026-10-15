"""Transfer a completed frontier action to its validated successor.

This module owns the optional no-stop handoff and its explicit fallback.  It
does not select the successor: GlobalFrontier remains the sole planning
authority and this bridge only checks that the committed route can be handed
over safely.
"""

import copy
import math

import rospy

from clock_provider import now_for


class TebGoalBridgePersistentFrontierHandoffMixin:
    def _start_frontier_continuous_prefetch_handoff_locked(self, pending_delta):
        """Promote one tangent-continuous frontier successor without a stop."""
        if (
            not self.frontier_continuous_prefetch_handoff_enabled
            or self.frontier_continuous_prefetch_handoff_pending is not None
            or self.frontier_observation_completion_pending is not None
            or not self.action_active
            or self.active_intent_priority != 0
            or self.latest_intent_priority != 0
            or self.active_intent_source != "global_slam_frontier"
            or self.latest_intent_source != "global_slam_frontier"
            or self.active_route_kind != "frontier_endpoint"
            or self.latest_route_kind != "frontier_endpoint"
            or self.active_route_id <= 0
            or self.active_feedback_distance is None
            or self.active_feedback_distance
            > self.frontier_observation_completion_radius
            or self.last_dispatched_goal is None
            or self.prefetched_frontier_goal is None
            or self.prefetched_frontier_route_id != self.active_route_id + 1
        ):
            return False

        source_goal = copy.deepcopy(self.last_dispatched_goal)
        feedback_distance = float(self.active_feedback_distance)
        pending_delta = math.hypot(
            self.prefetched_frontier_goal[0] - source_goal.pose.position.x,
            self.prefetched_frontier_goal[1] - source_goal.pose.position.y,
        )
        prefetch_heading_delta, prefetch_heading_basis = (
            self._frontier_prefetch_heading_delta_locked(
                source_goal, self.prefetched_frontier_goal
            )
        )
        if (
            prefetch_heading_delta is not None
            and prefetch_heading_delta > self.frontier_early_handoff_max_heading_delta
        ):
            route_pair = (
                int(self.active_route_id),
                int(self.prefetched_frontier_route_id),
            )
            if route_pair not in self.frontier_prefetch_requires_turn_pairs:
                self.frontier_prefetch_requires_turn_pairs.add(route_pair)
                self.publish_bridge_status(
                    "frontier_prefetch_requires_turn",
                    route_id=route_pair[0],
                    successor_route_id=route_pair[1],
                    heading_delta_deg=round(math.degrees(prefetch_heading_delta), 3),
                    max_heading_delta_deg=round(
                        math.degrees(self.frontier_early_handoff_max_heading_delta), 3
                    ),
                    heading_basis=prefetch_heading_basis,
                    lifecycle="terminal_then_successor_dispatch",
                )
                rospy.loginfo(
                    "TEB goal bridge deferred continuous frontier handoff: "
                    "route_id=%d successor_route_id=%d heading=%.1fdeg limit=%.1fdeg",
                    route_pair[0],
                    route_pair[1],
                    math.degrees(prefetch_heading_delta),
                    math.degrees(self.frontier_early_handoff_max_heading_delta),
                )
            return False
        pending = {
            "generation": int(self.action_generation),
            "route_id": int(self.active_route_id),
            "successor_route_id": int(self.prefetched_frontier_route_id),
            "source_goal": source_goal,
            "feedback_distance": feedback_distance,
            "pending_delta": float(pending_delta),
            "heading_delta": float(prefetch_heading_delta),
            "heading_basis": prefetch_heading_basis,
            "started_monotonic": now_for(self),
        }
        self.frontier_continuous_prefetch_handoff_pending = pending
        termination = self._commit_termination(
            "frontier_continuous_prefetch_handoff",
            source_goal=source_goal,
            action_contract=getattr(self, "active_action_contract", None),
            watchdog_reason="frontier_continuous_prefetch_handoff",
        )
        if not termination.get("committed", False):
            self.frontier_continuous_prefetch_handoff_pending = None
            return False
        self._clear_target_failure_locked("frontier_continuous_prefetch_progress")
        self.publish_bridge_status(
            "frontier_continuous_prefetch_handoff_requested",
            route_id=int(pending["route_id"]),
            successor_route_id=int(pending["successor_route_id"]),
            completion_radius=round(self.frontier_observation_completion_radius, 3),
            feedback_distance=round(feedback_distance, 3),
            pending_delta=round(float(pending_delta), 3),
            heading_delta_deg=round(math.degrees(prefetch_heading_delta), 3),
            heading_basis=prefetch_heading_basis,
            timeout_seconds=round(
                self.frontier_continuous_prefetch_handoff_timeout, 3
            ),
            lifecycle="terminal_then_successor_dispatch_after_release_ack",
        )
        rospy.loginfo(
            "TEB goal bridge requested continuous frontier handoff: "
            "route_id=%d successor_route_id=%d distance=%.2fm delta=%.2fm "
            "heading=%.1fdeg",
            pending["route_id"],
            pending["successor_route_id"],
            feedback_distance,
            pending_delta,
            math.degrees(prefetch_heading_delta),
        )
        return True

    def _expire_frontier_continuous_prefetch_handoff_locked(self, now):
        """Use the normal terminal handoff when successor promotion expires."""
        pending = self.frontier_continuous_prefetch_handoff_pending
        if pending is None:
            return False
        if now - float(pending["started_monotonic"]) < (
            self.frontier_continuous_prefetch_handoff_timeout
        ):
            return True
        self.frontier_continuous_prefetch_handoff_pending = None
        self.frontier_continuous_prefetch_handoff_fallback_count += 1
        # The source terminal was already delivered to the mission planner.
        # Close any remaining transport lease through the same termination
        # helper; the old action generation remains a tombstone.
        self.frontier_observation_completion_pending = pending
        self.handoff_requested = True
        self._commit_termination(
            "prefetch_promotion_timeout",
            source_goal=pending["source_goal"],
            action_contract=getattr(self, "active_action_contract", None),
            watchdog_reason="prefetch_promotion_timeout",
            publish_terminal=False,
        )
        self.publish_bridge_status(
            "frontier_continuous_prefetch_handoff_fallback",
            route_id=int(pending["route_id"]),
            successor_route_id=int(pending["successor_route_id"]),
            waited_seconds=round(
                now - float(pending["started_monotonic"]), 3
            ),
            lifecycle="prefetch_promotion_timeout_then_terminal_handoff",
        )
        rospy.logwarn(
            "TEB goal bridge continuous frontier handoff timed out; "
            "falling back to terminal dispatch route_id=%d",
            pending["route_id"],
        )
        return True
