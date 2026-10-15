#!/usr/bin/env python3
"""Regression tests for graph-to-controller frontier lease handoff."""

import ast
import json
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
STATUS_SOURCE = SCRIPTS / "teb_goal_bridge_frontier_status.py"
HEALTH_SOURCE = SCRIPTS / "teb_goal_bridge_action_health.py"


class GoalStatusStub:
    PREEMPTED = 2


def load_class(source, class_name, namespace):
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    owner = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    exec(
        compile(ast.Module(body=[owner], type_ignores=[]), str(source), "exec"),
        namespace,
    )
    return namespace[class_name]


STATUS_MIXIN = load_class(
    STATUS_SOURCE,
    "TebGoalBridgeFrontierStatusMixin",
    {"json": json, "math": __import__("math"),
     "rospy": SimpleNamespace(logwarn=lambda *_args, **_kwargs: None)},
)
HEALTH_MIXIN = load_class(
    HEALTH_SOURCE,
    "TebGoalBridgeActionHealthMixin",
    {
        "time": SimpleNamespace(monotonic=lambda: 100.0),
        "now_for": lambda _owner: 100.0,
        "GoalStatus": GoalStatusStub,
        "rospy": SimpleNamespace(logwarn=lambda *_args, **_kwargs: None),
    },
)


class FakeActionClient:
    def __init__(self):
        self.cancel_count = 0

    def cancel_goal(self):
        self.cancel_count += 1


class FakeBridge(STATUS_MIXIN, HEALTH_MIXIN):
    def __init__(self):
        self.lock = threading.RLock()
        self.persistent_execution = False
        self.task_done = False
        self.action_active = True
        self.active_intent_source = "global_slam_frontier"
        self.active_intent_priority = 0
        self.active_route_kind = "portal_transition"
        self.active_route_id = 17
        self.latest_intent_source = "global_slam_frontier"
        self.latest_intent_priority = 0
        self.latest_route_id = 17
        self.latest_goal = object()
        self.last_dispatched_goal = SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0),
            )
        )
        self.frontier_lease_released_route_id = 0
        self.frontier_lease_released_reason = ""
        self.frontier_observation_completion_pending = None
        self.frontier_continuous_prefetch_handoff_pending = None
        self.handoff_requested = False
        self.action_generation = 4
        self.last_result_status = None
        self.last_result_monotonic = 0.0
        self.persistent_target_pending_transaction = 0
        self.action_client = FakeActionClient()
        self.statuses = []
        self.health_cleared = False

    def _is_active_mode(self):
        return True

    def _clear_target_failure_locked(self, _reason):
        pass

    def _clear_failed_route_lease_locked(self):
        pass

    def _clear_action_health_locked(self):
        self.health_cleared = True
        self.active_route_id = 0
        self.active_intent_source = "unknown"
        self.active_intent_priority = 0
        self.active_route_kind = ""

    def publish_bridge_status(self, event, **fields):
        self.statuses.append((event, fields))


class TebBridgeFrontierLeaseTest(unittest.TestCase):
    def test_unavailable_active_route_keeps_controller_until_terminal(self):
        bridge = FakeBridge()

        bridge.on_frontier_status(SimpleNamespace(data=json.dumps({
            "event": "frontier_route_unavailable",
            "route_id": 17,
            "reason": "durable_identity_not_in_current_frontier_snapshot",
        })))

        self.assertEqual(bridge.action_client.cancel_count, 0)
        self.assertFalse(bridge.health_cleared)
        self.assertEqual(bridge.frontier_lease_released_route_id, 0)
        self.assertEqual(bridge.latest_route_id, 17)
        self.assertIsNotNone(bridge.latest_goal)
        self.assertEqual(
            [event for event, _fields in bridge.statuses],
            ["frontier_route_unavailable_deferred"],
        )
        self.assertEqual(
            bridge.statuses[-1][1]["controller_lease"],
            "held_until_terminal",
        )

    def test_unavailable_idle_route_releases_controller_lease(self):
        bridge = FakeBridge()
        bridge.action_active = False

        bridge.on_frontier_status(SimpleNamespace(data=json.dumps({
            "event": "frontier_route_unavailable",
            "route_id": 17,
            "reason": "durable_identity_not_in_current_frontier_snapshot",
        })))

        self.assertEqual(bridge.action_client.cancel_count, 0)
        self.assertTrue(bridge.health_cleared)
        self.assertEqual(bridge.frontier_lease_released_route_id, 17)
        self.assertEqual(bridge.latest_route_id, 0)
        self.assertIsNone(bridge.latest_goal)
        self.assertEqual(
            [event for event, _fields in bridge.statuses],
            ["controller_lease_released", "frontier_route_unavailable"],
        )
        self.assertEqual(
            bridge.statuses[-1][1]["released_route_id"],
            17,
        )

    def test_endpoint_terminal_allows_release_while_action_transport_is_active(self):
        """A persistent endpoint terminal is an explicit lease boundary."""
        bridge = FakeBridge()

        bridge.on_frontier_status(SimpleNamespace(data=json.dumps({
            "event": "frontier_route_unavailable",
            "route_id": 17,
            "reason": "selected_graph_obligation_not_executable_in_snapshot",
            "released_controller_route": {
                "route_id": 17,
                "controller_pending": True,
                "terminal_received": True,
            },
        })))

        self.assertEqual(bridge.action_client.cancel_count, 1)
        self.assertTrue(bridge.health_cleared)
        self.assertEqual(bridge.frontier_lease_released_route_id, 17)
        self.assertEqual(
            [event for event, _fields in bridge.statuses],
            ["controller_lease_released", "frontier_route_unavailable"],
        )
        self.assertIn(
            "endpoint_terminal_",
            bridge.statuses[-1][1]["reason"],
        )


if __name__ == "__main__":
    unittest.main()
