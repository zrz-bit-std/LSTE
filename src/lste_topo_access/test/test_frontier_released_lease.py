#!/usr/bin/env python3
"""Regression tests for delayed controller terminals after graph release."""

import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
SOURCE = SCRIPTS_DIR / "global_frontier_event_callbacks.py"


def _load_callback():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    owner = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierEventCallbacksMixin"
    )
    method = next(
        node for node in owner.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "on_bridge_status"
    )
    namespace = {
        "json": json,
        "time": __import__("time"),
        "now_for": lambda _owner: 100.0,
        "rospy": SimpleNamespace(
            logwarn=lambda *_args, **_kwargs: None,
        ),
    }
    exec(
        compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"),
        namespace,
    )
    return namespace["on_bridge_status"]


class _PortalLedger:
    def __init__(self):
        self.failure_calls = 0

    def get(self, _portal_id):
        return {"id": 5, "state": "crossed"}

    def execution_failed(self, *_args, **_kwargs):
        self.failure_calls += 1
        return {"id": 5, "source_place_id": 3, "state": "failed"}

    def failed(self, *_args, **_kwargs):
        self.failure_calls += 1
        return {"id": 5, "source_place_id": 3, "state": "failed"}


class _Explorer:
    on_bridge_status = _load_callback()

    def __init__(self):
        self.task_done = False
        self.active_frontier = None
        self.active_route_id = 16
        self.last_released_route_id = 16
        self.last_released_route_kind = "portal_transition"
        self.last_released_route_terminal_received = True
        self.last_released_route_controller_pending = True
        self.last_portal_hypothesis_id = 5
        self.portal_hypothesis_ledger = _PortalLedger()
        self.recovery_pending_route_id = 0
        self.recovery_pending_behavior = ""
        self.recovery_pending_reason = ""
        self.recovery_pending_wall = 0.0
        self.last_planning_wall = 42.0
        self.status = []

    def publish_status(self, event, **fields):
        self.status.append((event, fields))


class ReleasedControllerLeaseTest(unittest.TestCase):
    def test_delayed_failure_is_cleanup_after_logical_terminal(self):
        explorer = _Explorer()
        payload = {
            "event": "terminal",
            "active_route_id": 16,
            "active_intent_source": "global_slam_frontier",
            "active_route_kind": "portal_transition",
            "active_mission_route_kind": "portal_transition",
            "status": 4,
            "status_text": "ABORTED",
        }

        explorer.on_bridge_status(SimpleNamespace(data=json.dumps(payload)))

        self.assertFalse(explorer.last_released_route_controller_pending)
        self.assertEqual(explorer.last_planning_wall, 0.0)
        self.assertEqual(explorer.portal_hypothesis_ledger.failure_calls, 0)
        self.assertEqual(explorer.status[-1][0], "execution_terminal_failure")
        self.assertEqual(
            explorer.status[-1][1]["lease_phase"],
            "post_terminal_controller_cleanup",
        )


if __name__ == "__main__":
    unittest.main()
