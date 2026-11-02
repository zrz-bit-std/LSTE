"""A completed owned turn must re-arm the XY action-health clock."""

import json
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from teb_goal_bridge_action_health import TebGoalBridgeActionHealthMixin  # noqa: E402
from teb_goal_bridge_handoff_stall import TebGoalBridgeHandoffStallMixin  # noqa: E402
from teb_goal_bridge_teb_runtime import TebGoalBridgeTebRuntimeMixin  # noqa: E402


class TurnCompletionBridge(
    TebGoalBridgeTebRuntimeMixin,
    TebGoalBridgeHandoffStallMixin,
    TebGoalBridgeActionHealthMixin,
):
    def __init__(self, route_kind="frontier_endpoint"):
        self.lock = threading.RLock()
        self.turn_supervisor_state = "TURNING"
        self.turn_supervisor_last_event = "turn_started"
        self.action_active = True
        self.active_route_kind = route_kind
        self.active_route_id = 2
        self.action_generation = 3
        self.active_goal_transaction_id = 2
        self.active_motion_progress_monotonic = 10.0
        self.active_progress_monotonic = 10.0
        self.active_feedback_distance = 1.0
        self.active_best_distance = 1.2
        self.turn_transition_ready = False
        self.progress_timeout = 5.0
        self.dispatches = []

    def dispatch_locked(self, **kwargs):
        self.dispatches.append(kwargs)


class TebBridgeTurnCompletionProgressTest(unittest.TestCase):
    def payload(self, event="turn_completed", **overrides):
        value = {
            "event": event,
            "state": "PASS_THROUGH",
            "route_id": 2,
            "active_route_id": 2,
            "route_kind": "frontier_endpoint",
            "active_route_kind": "frontier_endpoint",
            "action_generation": 3,
            "active_action_generation": 3,
            "transaction_id": 2,
            "active_action_goal_transaction_id": 2,
        }
        value.update(overrides)
        return SimpleNamespace(data=json.dumps(value))

    def call(self, bridge, payload):
        with patch("teb_goal_bridge_teb_runtime.now_for", return_value=20.0):
            with patch("teb_goal_bridge_teb_runtime.rospy.loginfo_throttle"):
                bridge.on_turn_supervisor_status(payload)

    def test_completed_endpoint_turn_refreshes_physical_clock(self):
        bridge = TurnCompletionBridge()
        self.assertTrue(bridge._action_is_stalled_locked(16.0))

        self.call(bridge, self.payload())

        self.assertEqual(bridge.active_motion_progress_monotonic, 20.0)
        self.assertEqual(bridge.active_progress_monotonic, 20.0)
        self.assertFalse(bridge._action_is_stalled_locked(20.1))
        self.assertEqual(bridge.dispatches, [])

    def test_supported_turn_route_kinds_refresh_only_owned_completion(self):
        for route_kind in (
            "frontier_turn_connector",
            "frontier_endpoint",
            "portal_transition",
            "local_egress",
        ):
            with self.subTest(route_kind=route_kind):
                bridge = TurnCompletionBridge(route_kind)
                payload = self.payload(
                    route_kind=route_kind, active_route_kind=route_kind
                )
                self.call(bridge, payload)
                self.assertEqual(bridge.active_motion_progress_monotonic, 20.0)
                self.assertEqual(
                    len(bridge.dispatches),
                    1 if route_kind == "frontier_turn_connector" else 0,
                )

    def test_stale_route_generation_or_transaction_cannot_refresh(self):
        for key, value in (
            ("active_route_id", 9),
            ("active_action_generation", 4),
            ("active_action_goal_transaction_id", 8),
        ):
            with self.subTest(key=key):
                bridge = TurnCompletionBridge()
                self.call(bridge, self.payload(**{key: value}))
                self.assertEqual(bridge.active_motion_progress_monotonic, 10.0)
                self.assertEqual(bridge.active_progress_monotonic, 10.0)

    def test_release_missing_identity_and_turning_state_do_not_refresh(self):
        cases = (
            self.payload(event="turn_released"),
            self.payload(active_action_generation=0),
            self.payload(state="TURNING"),
        )
        for payload in cases:
            with self.subTest(payload=payload.data):
                bridge = TurnCompletionBridge()
                self.call(bridge, payload)
                self.assertEqual(bridge.active_motion_progress_monotonic, 10.0)
                self.assertEqual(bridge.active_progress_monotonic, 10.0)
                self.assertEqual(bridge.dispatches, [])

    def test_inactive_action_and_wrong_route_kind_do_not_refresh(self):
        bridge = TurnCompletionBridge()
        bridge.action_active = False
        self.call(bridge, self.payload())
        self.assertEqual(bridge.active_motion_progress_monotonic, 10.0)

        bridge = TurnCompletionBridge()
        self.call(bridge, self.payload(active_route_kind="another_route"))
        self.assertEqual(bridge.active_motion_progress_monotonic, 10.0)


if __name__ == "__main__":
    unittest.main()
