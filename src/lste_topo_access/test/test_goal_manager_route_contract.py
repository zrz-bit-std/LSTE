#!/usr/bin/env python3
"""Regression coverage for command phase versus mission lifecycle metadata."""

import ast
import json
from pathlib import Path
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
SOURCE = SCRIPTS / "goal_manager_frontier.py"


def load_status_callback():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    mixin = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "GoalManagerFrontierMixin"
    )
    callback = next(
        node for node in mixin.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "on_global_frontier_status"
    )
    rospy = type("Ros", (), {
        "Time": type("Time", (), {
            "now": staticmethod(lambda: type("Now", (), {"to_sec": lambda _self: 0.0})()),
        }),
        "loginfo": staticmethod(lambda *_args, **_kwargs: None),
        "logwarn": staticmethod(lambda *_args, **_kwargs: None),
    })
    namespace = {"json": json, "rospy": rospy, "String": object}
    exec(
        compile(ast.Module(body=[callback], type_ignores=[]), str(SOURCE), "exec"),
        namespace,
    )
    return namespace["on_global_frontier_status"]


class RouteContractLifecycle:
    on_global_frontier_status = load_status_callback()

    def __init__(self):
        self.global_frontier_route_kind = "frontier_connector"
        self.global_frontier_mission_route_kind = "frontier_endpoint"
        self.global_frontier_route_ids_by_goal = {}
        self.frontier_replan_pending_id = 0
        self.frontier_replan_ready_id = 0
        self.latest_global_frontier_goal = object()
        self.last_frontier_goal = object()
        self.global_frontier_route_id = 7
        self.teb_terminal_goal = None
        self.teb_frontier_goal_history = []
        self.frontier_goal_sent_at = None
        self.last_goal_source = "other"
        self.goal_source = "other"
        self.next_update_time = 1.0


class GoalManagerRouteContractTest(unittest.TestCase):
    @staticmethod
    def message(payload):
        return type("Message", (), {"data": json.dumps(payload)})()

    def test_work_item_audit_cannot_overwrite_a_live_connector_phase(self):
        lifecycle = RouteContractLifecycle()

        lifecycle.on_global_frontier_status(self.message({
            "event": "work_item_dispatched",
            "route_id": 7,
            "route_kind": "frontier_endpoint",
        }))

        self.assertEqual(lifecycle.global_frontier_route_kind, "frontier_connector")
        self.assertEqual(
            lifecycle.global_frontier_mission_route_kind, "frontier_endpoint",
        )

    def test_route_command_updates_phase_without_losing_mission_contract(self):
        lifecycle = RouteContractLifecycle()

        lifecycle.on_global_frontier_status(self.message({
            "event": "route_command",
            "route_id": 7,
            "route_kind": "frontier_connector",
            "mission_route_kind": "frontier_endpoint",
            "command_goal": [4.75, 11.85],
        }))

        self.assertEqual(lifecycle.global_frontier_route_kind, "frontier_connector")
        self.assertEqual(
            lifecycle.global_frontier_mission_route_kind, "frontier_endpoint",
        )


if __name__ == "__main__":
    unittest.main()
