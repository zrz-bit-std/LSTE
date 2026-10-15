"""Top-level action dispatch composition for the TEB goal bridge.

The façade keeps the historical import stable.  Focused siblings contain the
active-action policy, idle retry policy, and concrete action-client calls.
"""

import rospy

from teb_goal_bridge_action_active_dispatch import (
    TebGoalBridgeActionActiveDispatchMixin,
)
from teb_goal_bridge_action_client import TebGoalBridgeActionClientMixin
from teb_goal_bridge_action_retry_policy import TebGoalBridgeActionRetryPolicyMixin


class TebGoalBridgeActionDispatchMixin(
    TebGoalBridgeActionActiveDispatchMixin,
    TebGoalBridgeActionRetryPolicyMixin,
    TebGoalBridgeActionClientMixin,
):
    def dispatch_locked(self, force, reason):
        """Dispatch the latest intent after common lifecycle admission checks."""
        if self.latest_goal is None or not self._is_active_mode():
            return
        if self._transport_handoff_blocks_dispatch_locked():
            rospy.loginfo_throttle(
                2.0,
                "TEB goal bridge holds successor dispatch until old MoveBase "
                "action reaches terminal: reason=%s",
                reason,
            )
            return
        # Prepare has revoked the old action generation and requested a
        # controller release. Until Global Frontier's exact ACK arrives, a
        # timer wake must not resurrect the same semantic route as a new
        # MoveBase action.
        pending_prepare = getattr(self, "pending_terminal_prepare", None)
        if isinstance(pending_prepare, dict):
            rospy.loginfo_throttle(
                2.0,
                "TEB goal bridge holds dispatch during terminal release "
                "route_id=%s reason=%s",
                pending_prepare.get("old_contract", {}).get("route_id", 0),
                reason,
            )
            return
        # Transport may already have reached PREEMPTED while the semantic
        # controller-lease release is still awaiting the upstream owner.  Do
        # not let a duplicate mission_goal, or an out-of-order successor,
        # replace the immutable predecessor contract before that ACK is
        # applied.  The release path carries the exact contract explicitly.
        pending_release = getattr(
            self, "pending_lease_release_contract", None
        )
        if isinstance(pending_release, dict):
            old_contract = pending_release.get("old_contract")
            old_contract = (
                old_contract if isinstance(old_contract, dict)
                else pending_release
            )
            old_source = str(
                old_contract.get("source", "unknown") or "unknown"
            ).strip().lower()
            old_route_id = int(old_contract.get("route_id", 0) or 0)
            if (
                old_source == "global_slam_frontier"
                and old_route_id > 0
                and self.latest_intent_source == "global_slam_frontier"
                and int(self.latest_intent_priority) == 0
            ):
                rospy.loginfo_throttle(
                    2.0,
                    "TEB goal bridge holds dispatch until controller lease "
                    "ACK route_id=%d reason=%s",
                    old_route_id,
                    reason,
                )
                self.publish_bridge_status(
                    "dispatch_deferred_controller_lease_release",
                    route_id=old_route_id,
                    action_generation=int(
                        old_contract.get(
                            "generation",
                            old_contract.get("action_generation", 0),
                        ) or 0
                    ),
                    transaction_id=int(old_contract.get("transaction_id", 0) or 0),
                    lifecycle_transaction_id=int(
                        old_contract.get("lifecycle_transaction_id", 0) or 0
                    ),
                    graph_transaction_id=int(
                        old_contract.get("graph_transaction_id", 0) or 0
                    ),
                    reason="pending_controller_lease_release",
                    deferred_reason=str(reason or "dispatch"),
                )
                return
        if (
            not force
            and self.frontier_portal_wait
            and self.latest_intent_source == "global_slam_frontier"
            and self.latest_intent_priority == 0
            and self.latest_route_id <= self.frontier_portal_wait_route_id
        ):
            rospy.loginfo_throttle(
                3.0,
                "TEB goal bridge waits for a newer portal transaction after "
                "frontier route_id=%d",
                self.frontier_portal_wait_route_id,
            )
            return
        # A goal-manager subscriber can run before SimpleActionClient has
        # reached DONE.  The one-shot timer is a synchronization barrier, not
        # a planning delay.  Explicit controller transitions retain force.
        if (
            not force
            and self.terminal_dispatch_timer is not None
            and reason != "terminal_followup"
        ):
            return
        if not self._action_server_ready():
            return
        if not force and self._target_failure_blocks_latest_locked():
            rospy.loginfo_throttle(
                3.0,
                "TEB goal bridge holds failed target transaction until "
                "frontier progress or a new target epoch: epoch=%d",
                self.target_failure_epoch,
            )
            return

        if self.action_active and not force:
            self._dispatch_active_action_locked(reason)
            return
        self._dispatch_inactive_action_locked(force, reason)
