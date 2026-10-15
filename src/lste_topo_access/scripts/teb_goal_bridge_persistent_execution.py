"""Finish one persistent frontier observation region.

Endpoint-report validation, target-approach validation, and successor handoff
have separate modules.  This module contains the observation-envelope rule:
the point action is complete only after TEB has safely slowed at the endpoint.
"""

import copy
import math

import rospy

from clock_provider import now_for


class TebGoalBridgePersistentExecutionMixin:
    def _complete_frontier_observation_locked(self, pending_delta):
        """Finish a frontier observation region and hand its successor over.

        ``move_base`` has no notion of an exploration observation region: it
        keeps optimizing to a point even after the route has supplied all the
        information the frontier manager needs. Do this only for a committed
        endpoint with either a newly published route command or a validated
        lifecycle-only prefetch. The terminal pose remains the original source
        goal so the frontier manager can atomically promote that exact route.
        """
        completion_radius = self.frontier_observation_completion_radius
        if (
            completion_radius <= 0.0
            or self.frontier_observation_completion_pending is not None
            or getattr(self, "persistent_frontier_endpoint_pending", None)
            is not None
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
            > completion_radius
            or not self._frontier_observation_speed_ready_locked()
            or self.last_dispatched_goal is None
        ):
            return False
        published_successor = self.latest_route_id > self.active_route_id
        prefetched_successor = (
            self.prefetched_frontier_goal is not None
            and self.prefetched_frontier_route_id == self.active_route_id + 1
        )
        if not published_successor and not prefetched_successor:
            return False
        if prefetched_successor and not published_successor:
            pending_delta = math.hypot(
                self.prefetched_frontier_goal[0]
                - self.last_dispatched_goal.pose.position.x,
                self.prefetched_frontier_goal[1]
                - self.last_dispatched_goal.pose.position.y,
            )

        feedback_distance = float(self.active_feedback_distance)
        source_goal = copy.deepcopy(self.last_dispatched_goal)
        action_contract = getattr(self, "active_action_contract", None)
        old_generation = int(
            (action_contract or {}).get(
                "generation", getattr(self, "action_generation", 0)
            )
            or 0
        )
        self.frontier_observation_completion_pending = {
            "generation": old_generation,
            "route_id": int(self.active_route_id),
            "source_goal": source_goal,
            "feedback_distance": feedback_distance,
            "pending_delta": float(pending_delta),
            "completion_radius": float(completion_radius),
            "successor_route_id": int(
                self.latest_route_id
                if published_successor
                else self.prefetched_frontier_route_id
            ),
        }
        self.handoff_requested = True
        # The termination helper first revokes the old planner/feedback
        # identity, then closes the transport and publishes the typed terminal.
        # Global Frontier releases the semantic route and the successor remains
        # blocked until the exact release ACK arrives.
        termination = self._commit_termination(
            "frontier_observation_completion",
            source_goal=source_goal,
            action_contract=action_contract,
            watchdog_reason="frontier_observation_completion",
        )
        if not termination.get("committed", False):
            self.frontier_observation_completion_pending = None
            return False
        self._clear_target_failure_locked("frontier_observation_progress")
        self.publish_bridge_status(
            "frontier_observation_completion_requested",
            route_id=int(self.active_route_id),
            completion_radius=round(completion_radius, 3),
            feedback_distance=round(feedback_distance, 3),
            pending_delta=round(float(pending_delta), 3),
            successor_route_id=int(
                self.latest_route_id
                if published_successor
                else self.prefetched_frontier_route_id
            ),
            lifecycle="observation_region_then_terminal_handoff",
        )
        rospy.loginfo(
            "TEB goal bridge completed frontier %s region: "
            "route_id=%d distance=%.2fm radius=%.2fm successor_delta=%.2fm",
            "observation",
            self.active_route_id,
            feedback_distance,
            completion_radius,
            pending_delta,
        )
        return True

    def _frontier_observation_speed_ready_locked(self):
        """Return true after TEB has actually held its endpoint speed envelope.

        TEB feedback contains a predicted selected trajectory. At a point-goal
        terminal it can remain at the last cruise value or stop publishing
        entirely, while the planner command has already been zero for several
        control cycles. Prefer the latter when it is fresh; retain the
        feedback fallback for launch configurations without the raw topic.
        """
        now = now_for(self)
        raw_fresh = (
            self.latest_teb_planner_linear is not None
            and now - self.latest_teb_planner_command_monotonic
            <= self.teb_planner_command_timeout
        )
        if raw_fresh:
            return (
                abs(float(self.latest_teb_planner_linear))
                <= self.frontier_observation_completion_max_linear_speed
                and self.teb_planner_stationary_since > 0.0
                and now - self.teb_planner_stationary_since
                >= self.frontier_observation_stationary_hold
            )
        if (
            not bool(getattr(self, "teb_feedback_valid", False))
            or int(getattr(self, "teb_feedback_generation", 0) or 0)
            != int(getattr(self, "action_generation", 0) or 0)
            or self.latest_teb_selected_linear is None
        ):
            return False
        age = now - self.latest_teb_feedback_monotonic
        return (
            age <= self.teb_reorientation_feedback_timeout
            and abs(float(self.latest_teb_selected_linear))
            <= self.frontier_observation_completion_max_linear_speed
        )

    def _target_progress_state_locked(self, now):
        """Return the target-route health clock and the evidence behind it."""
        if (
            self.active_navfn_plan_points
            and self.active_navfn_progress_monotonic > 0.0
        ):
            return (
                "navfn_path_remaining",
                now - self.active_navfn_progress_monotonic,
            )
        if self.active_motion_progress_monotonic > 0.0:
            return (
                "physical_motion",
                now - self.active_motion_progress_monotonic,
            )
        return ("no_feedback", float("inf"))
