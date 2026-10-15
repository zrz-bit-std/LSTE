"""Regression for the terminal-observation ownership handoff."""

import ast
import contextlib
import json
from pathlib import Path
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def load_goal_command_method():
    source_path = SCRIPTS / "teb_goal_bridge_mission_input.py"
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    method = next(
        node
        for node in next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef)
            and node.name == "TebGoalBridgeMissionInputMixin"
        ).body
        if isinstance(node, ast.FunctionDef) and node.name == "on_goal_command"
    )
    namespace = {
        "json": json,
        "math": __import__("math"),
        "rospy": SimpleNamespace(
            Time=SimpleNamespace(now=lambda: 123.0),
            logwarn_throttle=lambda *_args, **_kwargs: None,
            loginfo_throttle=lambda *_args, **_kwargs: None,
        ),
        "PoseStamped": lambda: SimpleNamespace(
            header=SimpleNamespace(stamp=None, frame_id=""),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(z=0.0, w=1.0),
            ),
        ),
        "normalize_goal_context": lambda value: value or {},
    }
    exec(
        compile(ast.Module(body=[method], type_ignores=[]), str(source_path), "exec"),
        namespace,
    )
    return namespace["on_goal_command"]


ON_GOAL_COMMAND = load_goal_command_method()


class TargetObservationBridgeFixture:
    on_goal_command = ON_GOAL_COMMAND

    def __init__(self):
        self.lock = contextlib.nullcontext()
        self.latest_goal_transaction_id = 4
        self.action_active = True
        self.persistent_execution = True
        self.active_intent_priority = 2
        self.active_intent_source = "target_parallax"
        self.latest_intent_priority = 2
        self.latest_intent_source = "target_parallax"
        self.persistent_target_pending_transaction = 0
        self.active_action_contract = {"generation": 7}
        self.active_goal_transaction_id = 4
        self.global_frame = "map"
        self.frontier_portal_wait = False
        self.frontier_portal_wait_route_id = 0
        self.cancel_calls = []
        self.clear_calls = []
        self.statuses = []
        self.frontier_accept_calls = 0
        self.persistent_adopt_calls = 0

    def cancel_locked(self, reason):
        self.cancel_calls.append(str(reason))
        self.action_active = False

    def _clear_persistent_target_request_locked(self, reason, force=False):
        self.clear_calls.append((str(reason), bool(force)))

    def _frontier_route_is_stale_locked(self, _source, _priority, _route_id):
        return False

    def _frontier_route_released_locked(self, _source, _priority, _route_id):
        return False

    def _accept_newer_frontier_route_locked(self, _source, _priority, _route_id):
        self.frontier_accept_calls += 1

    def _normalize_goal(self, goal):
        return goal

    def _clear_persistent_target_request_locked(self, _reason, force=False):
        self.clear_calls.append((str(_reason), bool(force)))

    def _publish_persistent_mission_goal_locked(self, _reason):
        return True

    def _adopt_persistent_mission_goal_locked(self):
        self.persistent_adopt_calls += 1

    def _is_active_mode(self):
        return False

    def publish_bridge_status(self, event, **fields):
        self.statuses.append((str(event), fields))


class TargetObservationBridgeTest(unittest.TestCase):
    def test_terminal_observation_cancels_old_action_and_clears_target_request(self):
        bridge = TargetObservationBridgeFixture()
        bridge.on_goal_command(
            SimpleNamespace(
                data=json.dumps(
                    {
                        "event": "target_terminal_observation",
                        "transaction_id": 5,
                        "target_epoch": 9,
                        "target_track_id": "yellow_cup:yellow cup:1",
                        "reason": "navfn_unavailable_after_target_terminal",
                    }
                )
            )
        )

        self.assertEqual(bridge.cancel_calls, ["target_terminal_observation"])
        self.assertEqual(
            bridge.clear_calls,
            [("target_terminal_observation", True)],
        )
        self.assertIsNone(bridge.latest_goal)
        self.assertFalse(bridge.action_active)
        status_events = [event for event, _fields in bridge.statuses]
        self.assertIn("target_terminal_observation_started", status_events)
        self.assertEqual(
            bridge.statuses[-1][0],
            "target_terminal_observation_released",
        )
        self.assertEqual(bridge.statuses[-1][1]["transaction_id"], 5)
        self.assertEqual(
            bridge.statuses[-1][1]["controller_lease"],
            "released",
        )

    def test_frontier_cannot_overwrite_active_target_transaction(self):
        bridge = TargetObservationBridgeFixture()
        bridge.on_goal_command(
            SimpleNamespace(
                data=json.dumps(
                    {
                        "event": "mission_goal",
                        "transaction_id": 5,
                        "priority": 0,
                        "source": "global_slam_frontier",
                        "route_id": 12,
                        "route_kind": "frontier_endpoint",
                        "goal": [4.0, 5.0],
                    }
                )
            )
        )

        self.assertEqual(bridge.frontier_accept_calls, 0)
        self.assertEqual(bridge.latest_goal_transaction_id, 4)
        self.assertEqual(bridge.latest_intent_source, "target_parallax")
        self.assertEqual(bridge.latest_intent_priority, 2)
        self.assertEqual(
            bridge.statuses[-1][0],
            "mission_goal_ignored",
        )
        self.assertEqual(
            bridge.statuses[-1][1]["reason"],
            "higher_priority_target_active",
        )
        self.assertEqual(
            bridge.statuses[-1][1]["active_transaction_id"],
            4,
        )

    def test_frontier_can_take_over_after_target_terminal_boundary(self):
        bridge = TargetObservationBridgeFixture()
        bridge.persistent_target_terminal_boundary_transaction = 4
        bridge.on_goal_command(
            SimpleNamespace(
                data=json.dumps(
                    {
                        "event": "mission_goal",
                        "transaction_id": 5,
                        "priority": 0,
                        "source": "global_slam_frontier",
                        "route_id": 12,
                        "route_kind": "frontier_endpoint",
                        "goal": [4.0, 5.0],
                    }
                )
            )
        )

        self.assertEqual(bridge.frontier_accept_calls, 1)
        self.assertEqual(bridge.persistent_adopt_calls, 1)
        self.assertEqual(bridge.latest_goal_transaction_id, 5)
        self.assertEqual(bridge.latest_intent_source, "global_slam_frontier")
        self.assertEqual(bridge.latest_intent_priority, 0)
        self.assertEqual(bridge.persistent_target_terminal_boundary_transaction, 0)
        self.assertEqual(bridge.statuses[-1][0], "mission_goal_received")


if __name__ == "__main__":
    unittest.main()
