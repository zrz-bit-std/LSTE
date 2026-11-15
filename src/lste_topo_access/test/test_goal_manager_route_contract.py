#!/usr/bin/env python3
"""Regression coverage for command phase versus mission lifecycle metadata."""

import ast
import json
from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
SOURCE = SCRIPTS / "goal_manager_frontier.py"
GOAL_OUTPUT_SOURCE = SCRIPTS / "goal_manager_goal_output.py"
CORE_SCRIPTS = SCRIPTS.parents[1] / "lste_core" / "scripts"
for path in (SCRIPTS, CORE_SCRIPTS):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
from utils.route_ownership import (  # noqa: E402
    GRAPH,
    GRAPH_GOAL_ROLES,
    OBSERVATION,
    OBSERVATION_GOAL_ROLES,
)


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
        self.global_frontier_command_enabled = False
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



def load_goal_output_methods():
    tree = ast.parse(
        GOAL_OUTPUT_SOURCE.read_text(encoding="utf-8"),
        filename=str(GOAL_OUTPUT_SOURCE),
    )
    mixin = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GoalManagerGoalOutputMixin"
    )
    methods = [
        node for node in mixin.body
        if isinstance(node, ast.FunctionDef)
        and node.name in {
            "_frontier_goal_ownership",
            "_canonical_goal_identity_ready",
        }
    ]
    namespace = {
        "GRAPH": GRAPH,
        "OBSERVATION": OBSERVATION,
        "GRAPH_GOAL_ROLES": GRAPH_GOAL_ROLES,
        "OBSERVATION_GOAL_ROLES": OBSERVATION_GOAL_ROLES,
    }
    exec(
        compile(
            ast.Module(body=methods, type_ignores=[]),
            str(GOAL_OUTPUT_SOURCE),
            "exec",
        ),
        namespace,
    )
    return namespace


GOAL_OUTPUT_METHODS = load_goal_output_methods()


class GoalOutputOwnership:
    _frontier_goal_ownership = GOAL_OUTPUT_METHODS["_frontier_goal_ownership"]
    _canonical_goal_identity_ready = (
        GOAL_OUTPUT_METHODS["_canonical_goal_identity_ready"]
    )

    def __init__(self):
        self.global_frontier_enabled = True
        self.global_frontier_lifecycle_transaction_id = 7
        self.global_frontier_route_id = 19
        self.global_frontier_map_epoch = 10
        self.global_frontier_origin_epoch = 10
        self.global_frontier_projection_epoch = 10
        self.global_frontier_graph_transaction_id = 0
        self.canonical_route_ownership = GRAPH
        self.canonical_work_item_id = 16
        self.canonical_attempt_id = 6
        self.global_frontier_goal_context = {
            "goal_role": "place_observation_work",
            "work_item_id": 16,
            "attempt_id": 6,
        }
        self.rejections = []

    @staticmethod
    def _canonical_lifecycle_transaction_id():
        return 7

    @staticmethod
    def _canonical_map_epoch():
        return 10

    def publish_goal_arbitration(self, event, **fields):
        self.rejections.append((event, fields))


class GoalOutputOwnershipTest(unittest.TestCase):
    def test_observation_goal_does_not_inherit_stale_graph_ownership(self):
        manager = GoalOutputOwnership()

        self.assertTrue(
            manager._canonical_goal_identity_ready("global_slam_frontier")
        )
        self.assertEqual(manager.rejections, [])

    def test_place_observation_goal_accepts_zero_graph_identity(self):
        manager = GoalOutputOwnership()
        manager.global_frontier_goal_context = {"goal_role": "place_observation"}

        self.assertTrue(
            manager._canonical_goal_identity_ready("global_slam_frontier")
        )

    def test_portal_probe_goal_still_requires_positive_graph_identity(self):
        manager = GoalOutputOwnership()
        manager.global_frontier_goal_context = {"goal_role": "portal_probe"}

        self.assertFalse(
            manager._canonical_goal_identity_ready("global_slam_frontier")
        )
        self.assertEqual(
            manager.rejections[-1][1]["reason"],
            "missing_graph_transaction_id",
        )

    def test_portal_probe_goal_accepts_a_positive_graph_identity(self):
        manager = GoalOutputOwnership()
        manager.global_frontier_goal_context = {"goal_role": "portal_probe"}
        manager.global_frontier_graph_transaction_id = 42

        self.assertTrue(
            manager._canonical_goal_identity_ready("global_slam_frontier")
        )


if __name__ == "__main__":
    unittest.main()
