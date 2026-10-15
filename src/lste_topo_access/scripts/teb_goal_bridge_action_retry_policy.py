"""Idle-action dispatch and failed-frontier ownership policy.

Global Frontier owns route selection.  After ``move_base`` rejects or aborts a
frontier route, this policy keeps the bridge passive until Global Frontier
publishes a new route ID.  The bridge may still dispatch a higher-priority
target or an explicit forced controller transition.
"""

import rospy
from actionlib_msgs.msg import GoalStatus

from clock_provider import now_for


class TebGoalBridgeActionRetryPolicyMixin:
    _FAILED_FRONTIER_STATUSES = (
        GoalStatus.ABORTED,
        GoalStatus.REJECTED,
        GoalStatus.RECALLED,
        GoalStatus.LOST,
    )

    def _latest_intent_matches_last_dispatch_locked(self):
        """Return whether the pending intent is the same semantic dispatch.

        The endpoint is intentionally excluded from this comparison.  A new
        graph route can reuse the exact same coordinate, while a repeated
        publication of one route must remain a no-op after success.  Older
        bridge instances without an identity record are treated as changed so
        they cannot silently inherit the old coordinate-only suppression.
        """
        identity = getattr(self, "last_dispatch_identity", None)
        if not isinstance(identity, dict):
            return False
        return bool(
            int(identity.get("transaction_id", 0) or 0)
            == int(getattr(self, "latest_goal_transaction_id", 0) or 0)
            and int(identity.get("route_id", 0) or 0)
            == int(getattr(self, "latest_route_id", 0) or 0)
            and int(identity.get("graph_transaction_id", 0) or 0)
            == int(getattr(self, "latest_graph_transaction_id", 0) or 0)
            and str(identity.get("route_kind", "") or "").strip().lower()
            == str(getattr(self, "latest_route_kind", "") or "").strip().lower()
            and str(identity.get("mission_route_kind", "") or "").strip().lower()
            == str(
                getattr(self, "latest_mission_route_kind", "") or ""
            ).strip().lower()
            and str(identity.get("source", "unknown") or "unknown").strip().lower()
            == str(
                getattr(self, "latest_intent_source", "unknown") or "unknown"
            ).strip().lower()
            and int(identity.get("priority", 0) or 0)
            == int(getattr(self, "latest_intent_priority", 0) or 0)
        )

    def _frontier_route_released_locked(self, source, priority, route_id):
        """Return whether a command belongs to a released frontier lease.

        The release record is intentionally identity based.  A nearby goal is
        not a replacement route; only a strictly newer Global Frontier route
        can take the controller lease again.
        """
        try:
            route_id = int(route_id or 0)
        except (TypeError, ValueError):
            route_id = 0
        try:
            released_route_id = int(
                getattr(self, "frontier_lease_released_route_id", 0) or 0
            )
        except (TypeError, ValueError):
            released_route_id = 0
        return bool(
            str(source or "").strip().lower() == "global_slam_frontier"
            and int(priority or 0) == 0
            and route_id > 0
            and released_route_id > 0
            and route_id <= released_route_id
        )

    def _frontier_route_is_stale_locked(self, source, priority, route_id):
        """Reject an out-of-order frontier route after a newer route exists."""
        try:
            route_id = int(route_id or 0)
        except (TypeError, ValueError):
            route_id = 0
        if not (
            str(source or "").strip().lower() == "global_slam_frontier"
            and int(priority or 0) == 0
            and route_id > 0
        ):
            return False
        current_route_id = max(
            int(getattr(self, "active_route_id", 0) or 0),
            int(getattr(self, "latest_route_id", 0) or 0),
            int(getattr(self, "frontier_lease_released_route_id", 0) or 0),
        )
        return route_id < current_route_id

    def _accept_newer_frontier_route_locked(self, source, priority, route_id):
        """Advance the release tombstone only for a newer route identity."""
        try:
            route_id = int(route_id or 0)
        except (TypeError, ValueError):
            route_id = 0
        try:
            released_route_id = int(
                getattr(self, "frontier_lease_released_route_id", 0) or 0
            )
        except (TypeError, ValueError):
            released_route_id = 0
        if not (
            str(source or "").strip().lower() == "global_slam_frontier"
            and int(priority or 0) == 0
            and route_id > released_route_id
        ):
            return False
        self.frontier_lease_released_route_id = 0
        self.frontier_lease_released_reason = ""
        return True

    def _failed_frontier_route_waiting_for_global_replacement_locked(self):
        """Whether the latest intent repeats a route that just failed.

        This deliberately uses the route identity, not XY proximity.  A
        replacement endpoint can be nearby but is a distinct Global Frontier
        decision and therefore always has a new ``route_id``.
        """
        route_id = int(
            getattr(self, "active_route_id", 0)
            or getattr(self, "failed_route_id", 0)
            or 0
        )
        route_source = str(
            getattr(self, "active_intent_source", "unknown")
            if int(getattr(self, "active_route_id", 0) or 0) > 0
            else getattr(self, "failed_route_source", "unknown")
        ).strip().lower()
        route_priority = int(
            getattr(self, "active_intent_priority", 0)
            if int(getattr(self, "active_route_id", 0) or 0) > 0
            else getattr(self, "failed_route_priority", 0)
        )
        return bool(
            not self.action_active
            and self.last_result_status in self._FAILED_FRONTIER_STATUSES
            and route_source == "global_slam_frontier"
            and route_priority == 0
            and route_id > 0
            and self.latest_intent_source == "global_slam_frontier"
            and int(self.latest_intent_priority) == 0
            and int(self.latest_route_id) == route_id
        )

    def _hold_failed_frontier_route_locked(self, reason):
        rospy.logwarn_throttle(
            3.0,
            "TEB goal bridge holds failed frontier route_id=%d; waiting for "
            "Global Frontier replacement: status=%s trigger=%s",
            int(
                getattr(self, "active_route_id", 0)
                or getattr(self, "failed_route_id", 0)
                or 0
            ),
            GoalStatus.to_string(self.last_result_status),
            reason,
        )

    def _dispatch_inactive_action_locked(self, force, reason):
        """Dispatch an idle action without taking route choice from Frontier."""
        # A failed frontier route is identified by its durable route ID, not
        # by endpoint coordinates. The graph may publish a nearby refinement
        # while retaining the same route identity; redispatching it would
        # resurrect the exact action that just failed and starve the graph
        # executive of its terminal boundary.
        if (
            not force
            and self._failed_frontier_route_waiting_for_global_replacement_locked()
        ):
            self._hold_failed_frontier_route_locked(reason)
            return
        if (
            not force
            and self._frontier_route_released_locked(
                self.latest_intent_source,
                self.latest_intent_priority,
                self.latest_route_id,
            )
        ):
            self._hold_failed_frontier_route_locked(
                "released_frontier_route:%d" % int(self.latest_route_id)
            )
            return
        same_goal = self._same_goal(self.latest_goal, self.last_dispatched_goal)
        same_dispatch = self._latest_intent_matches_last_dispatch_locked()
        if not force and same_goal and same_dispatch:
            if self.action_active or self.last_result_status == GoalStatus.SUCCEEDED:
                return
            if self._failed_frontier_route_waiting_for_global_replacement_locked():
                self._hold_failed_frontier_route_locked(reason)
                return
            if (
                now_for(self) - self.last_result_monotonic
                < self.goal_retry_interval
            ):
                return
            reason = "retry_move_base_goal"

        if (
            not force
            and self.last_dispatched_goal is not None
            and now_for(self) - self.last_dispatch_monotonic
            < self.min_update_interval
        ):
            return

        if force and self.action_active:
            self._commit_termination(
                "forced_action_replacement",
                source_goal=getattr(self, "last_dispatched_goal", None),
                action_contract=getattr(self, "active_action_contract", None),
                watchdog_reason="forced_action_replacement",
                publish_terminal=False,
                arm_watchdog=False,
            )

        self._send_goal_locked(self.latest_goal, reason=reason, replacement=False)
