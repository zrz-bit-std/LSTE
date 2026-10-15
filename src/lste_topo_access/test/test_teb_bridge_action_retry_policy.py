#!/usr/bin/env python3
"""Regression tests for bridge/frontier route-ownership boundaries."""

import ast
from pathlib import Path
import unittest
from types import SimpleNamespace


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
POLICY_SOURCE = SCRIPTS_DIR / "teb_goal_bridge_action_retry_policy.py"


class GoalStatusStub:
    ABORTED = 4
    REJECTED = 5
    RECALLED = 8
    LOST = 9
    SUCCEEDED = 3

    @staticmethod
    def to_string(status):
        return {
            3: "SUCCEEDED",
            4: "ABORTED",
            5: "REJECTED",
            8: "RECALLED",
            9: "LOST",
        }.get(status, "UNKNOWN")


def load_retry_policy():
    tree = ast.parse(POLICY_SOURCE.read_text(encoding="utf-8"), filename=str(POLICY_SOURCE))
    policy = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "TebGoalBridgeActionRetryPolicyMixin"
    )
    namespace = {
        "GoalStatus": GoalStatusStub,
        "rospy": SimpleNamespace(logwarn_throttle=lambda *_args: None),
        "time": SimpleNamespace(monotonic=lambda: 100.0),
        "now_for": lambda _owner: 100.0,
    }
    exec(
        compile(
            ast.Module(body=[policy], type_ignores=[]),
            str(POLICY_SOURCE),
            "exec",
        ),
        namespace,
    )
    return namespace["TebGoalBridgeActionRetryPolicyMixin"]


class RetryPolicyProbe(load_retry_policy()):
    def __init__(self):
        self.action_active = False
        self.last_result_status = GoalStatusStub.ABORTED
        self.active_intent_source = "global_slam_frontier"
        self.active_intent_priority = 0
        self.active_route_id = 26
        self.latest_intent_source = "global_slam_frontier"
        self.latest_intent_priority = 0
        self.latest_route_id = 26
        self.failed_route_id = 0
        self.failed_route_source = "unknown"
        self.failed_route_priority = 0

    def _hold_failed_frontier_route_locked(self, reason):
        self.hold_reason = reason


class InactiveDispatchProbe(load_retry_policy()):
    """Small action-free harness for coordinate/identity dispatch behavior."""

    def __init__(self):
        self.action_active = False
        self.last_result_status = GoalStatusStub.SUCCEEDED
        self.last_result_monotonic = 0.0
        self.last_dispatch_monotonic = 0.0
        self.goal_retry_interval = 0.0
        self.min_update_interval = 0.0
        self.active_intent_source = "unknown"
        self.active_intent_priority = 0
        self.active_route_id = 0
        self.failed_route_id = 0
        self.failed_route_source = "unknown"
        self.failed_route_priority = 0
        self.latest_intent_source = "global_slam_frontier"
        self.latest_intent_priority = 0
        self.latest_route_kind = "frontier_endpoint"
        self.latest_mission_route_kind = "frontier_endpoint"
        self.latest_route_id = 2
        self.latest_goal_transaction_id = 2
        self.latest_goal = self._pose(4.0, 5.0)
        self.last_dispatched_goal = self._pose(4.0, 5.0)
        self.last_dispatch_identity = {
            "transaction_id": 1,
            "route_id": 1,
            "route_kind": "frontier_endpoint",
            "mission_route_kind": "frontier_endpoint",
            "source": "global_slam_frontier",
            "priority": 0,
        }
        self.frontier_lease_released_route_id = 0
        self.sent = []

    @staticmethod
    def _pose(x, y):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)),
        )

    def _same_goal(self, first, second):
        return (
            first is not None
            and second is not None
            and first.pose.position.x == second.pose.position.x
            and first.pose.position.y == second.pose.position.y
        )

    def _send_goal_locked(self, goal, reason, replacement=False):
        self.sent.append((goal, reason, replacement))
        return True


class TebBridgeActionRetryPolicyTest(unittest.TestCase):
    def test_same_coordinate_new_route_is_dispatched_immediately(self):
        probe = InactiveDispatchProbe()

        probe._dispatch_inactive_action_locked(False, "coalesced_global_goal")

        self.assertEqual(len(probe.sent), 1)
        self.assertEqual(probe.sent[0][1], "coalesced_global_goal")

    def test_same_coordinate_same_route_identity_remains_a_noop(self):
        probe = InactiveDispatchProbe()
        probe.latest_route_id = 1
        probe.latest_goal_transaction_id = 1

        probe._dispatch_inactive_action_locked(False, "coalesced_global_goal")

        self.assertEqual(probe.sent, [])

    def test_failed_frontier_route_waits_for_a_new_route_id(self):
        probe = RetryPolicyProbe()

        self.assertTrue(
            probe._failed_frontier_route_waiting_for_global_replacement_locked()
        )

    def test_nearby_goal_cannot_bypass_failed_route_identity_guard(self):
        """A changed XY on the same route must not trigger redispatch."""
        probe = RetryPolicyProbe()
        probe.latest_route_id = 26
        probe.latest_intent_goal = (1.05, 2.0)
        probe.last_dispatched_goal = SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0),
            )
        )

        # The guard returns before coordinate comparison or action-client
        # state is consulted, which is the contract this regression protects.
        probe._dispatch_inactive_action_locked(False, "coalesced_global_goal")

        self.assertEqual(probe.hold_reason, "coalesced_global_goal")

    def test_replacement_route_is_admitted_after_terminal_failure(self):
        probe = RetryPolicyProbe()
        probe.latest_route_id = 27

        self.assertFalse(
            probe._failed_frontier_route_waiting_for_global_replacement_locked()
        )

    def test_higher_priority_target_is_not_blocked_by_frontier_failure(self):
        probe = RetryPolicyProbe()
        probe.latest_intent_source = "global_target"
        probe.latest_intent_priority = 2

        self.assertFalse(
            probe._failed_frontier_route_waiting_for_global_replacement_locked()
        )

    def test_failed_route_identity_survives_action_health_reset(self):
        probe = RetryPolicyProbe()
        probe.active_intent_source = "unknown"
        probe.active_intent_priority = 0
        probe.active_route_id = 0
        probe.failed_route_id = 26
        probe.failed_route_source = "global_slam_frontier"
        probe.failed_route_priority = 0

        self.assertTrue(
            probe._failed_frontier_route_waiting_for_global_replacement_locked()
        )

    def test_failed_route_lease_is_released_by_new_route_id(self):
        probe = RetryPolicyProbe()
        probe.active_intent_source = "unknown"
        probe.active_route_id = 0
        probe.failed_route_id = 26
        probe.failed_route_source = "global_slam_frontier"
        probe.failed_route_priority = 0
        probe.latest_route_id = 27

        self.assertFalse(
            probe._failed_frontier_route_waiting_for_global_replacement_locked()
        )

    def test_released_frontier_route_is_not_dispatchable(self):
        probe = RetryPolicyProbe()
        probe.active_route_id = 0
        probe.active_intent_source = "unknown"
        probe.active_intent_priority = 0
        probe.latest_route_id = 17
        probe.frontier_lease_released_route_id = 17

        self.assertTrue(
            probe._frontier_route_released_locked(
                probe.latest_intent_source,
                probe.latest_intent_priority,
                probe.latest_route_id,
            )
        )

    def test_newer_frontier_route_consumes_release_tombstone(self):
        probe = RetryPolicyProbe()
        probe.frontier_lease_released_route_id = 17

        self.assertTrue(
            probe._accept_newer_frontier_route_locked(
                "global_slam_frontier", 0, 18
            )
        )
        self.assertEqual(probe.frontier_lease_released_route_id, 0)

    def test_older_frontier_route_is_rejected_after_newer_route_arrives(self):
        probe = RetryPolicyProbe()
        probe.active_route_id = 18
        probe.latest_route_id = 18

        self.assertTrue(
            probe._frontier_route_is_stale_locked(
                "global_slam_frontier", 0, 17
            )
        )


if __name__ == "__main__":
    unittest.main()
