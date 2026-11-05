#!/usr/bin/env python3
"""Ownership propagation from GoalManager output to the TEB bridge.

These cover the second and third gates that dropped a valid observation route:
GoalManager's mission-goal identity readiness and the bridge's canonical
instruction identity.  Both must allow ``graph_transaction_id == 0`` for an
observation route while keeping graph-owned routes strict.
"""

import ast
import json
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "lste_core" / "scripts"))

GOAL_OUTPUT = ROOT / "src" / "lste_topo_access" / "scripts" / "goal_manager_goal_output.py"
BRIDGE_LIFECYCLE = ROOT / "src" / "lste_topo_access" / "scripts" / "teb_goal_bridge_lifecycle.py"


def load_method(source, class_name, method_name):
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    cls = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    method = next(
        node for node in cls.body
        if isinstance(node, ast.FunctionDef) and node.name == method_name
    )
    holder = ast.ClassDef(
        name="Holder", bases=[], keywords=[], body=[method],
        decorator_list=[],
    )
    module = ast.Module(body=[holder], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {"json": json, "math": math}
    exec(compile(module, str(source), "exec"), namespace)
    return namespace["Holder"]


GOAL_OUTPUT_API = load_method(
    GOAL_OUTPUT, "GoalManagerGoalOutputMixin", "_canonical_goal_identity_ready"
)
BRIDGE_API = load_method(
    BRIDGE_LIFECYCLE,
    "TebGoalBridgeLifecycleMixin",
    "_canonical_instruction_identity",
)


class GoalOutputIdentity:
    def __init__(self, ownership, graph_id, work_item_id=1, attempt_id=2):
        self.global_frontier_enabled = True
        self.global_frontier_route_id = 1
        self.global_frontier_graph_transaction_id = graph_id
        self.global_frontier_origin_epoch = 4
        self.global_frontier_projection_epoch = 4
        self.canonical_route_ownership = ownership
        self.canonical_work_item_id = work_item_id
        self.canonical_attempt_id = attempt_id
        self.events = []

    def _canonical_lifecycle_transaction_id(self):
        return 11

    def _canonical_map_epoch(self):
        return 4

    def publish_goal_arbitration(self, event, **fields):
        self.events.append((event, fields))


def message(payload):
    return type("Message", (), {"data": json.dumps(payload)})()


def mission_payload(ownership, graph_id, **overrides):
    payload = {
        "event": "mission_goal",
        "source": "global_slam_frontier",
        "lifecycle_transaction_id": 11,
        "route_id": 1,
        "graph_transaction_id": graph_id,
        "transaction_id": 1,
        "map_epoch": 4,
        "origin_epoch": 4,
        "projection_epoch": 4,
        "goal": [10.85, -0.65],
        "route_ownership": ownership,
        "work_item_id": 1,
        "attempt_id": 2,
        "goal_context": {
            "goal_role": "place_observation_work",
            "route_ownership": ownership,
            "work_item_id": 1,
            "attempt_id": 2,
        },
    }
    payload.update(overrides)
    return payload


class GoalOutputIdentityTest(unittest.TestCase):
    def test_observation_route_is_ready_with_zero_graph_id(self):
        context = GoalOutputIdentity("observation", 0)
        self.assertTrue(
            GOAL_OUTPUT_API._canonical_goal_identity_ready(
                context, "global_slam_frontier"
            )
        )
        self.assertEqual(context.events, [])

    def test_graph_route_is_not_ready_with_zero_graph_id(self):
        context = GoalOutputIdentity("graph", 0)
        self.assertFalse(
            GOAL_OUTPUT_API._canonical_goal_identity_ready(
                context, "global_slam_frontier"
            )
        )
        self.assertEqual(
            context.events[0][1]["reason"], "missing_graph_transaction_id"
        )

    def test_observation_work_item_requires_attempt(self):
        context = GoalOutputIdentity("observation", 0, attempt_id=0)
        self.assertFalse(
            GOAL_OUTPUT_API._canonical_goal_identity_ready(
                context, "global_slam_frontier"
            )
        )
        self.assertEqual(
            context.events[0][1]["reason"], "missing_observation_attempt_id"
        )

    def test_geometry_observation_needs_no_work_item(self):
        context = GoalOutputIdentity("observation", 0, work_item_id=0, attempt_id=0)
        self.assertTrue(
            GOAL_OUTPUT_API._canonical_goal_identity_ready(
                context, "global_slam_frontier"
            )
        )

    def test_graph_route_with_positive_graph_id_is_ready(self):
        context = GoalOutputIdentity("graph", 5)
        self.assertTrue(
            GOAL_OUTPUT_API._canonical_goal_identity_ready(
                context, "global_slam_frontier"
            )
        )

    def test_target_route_inherits_observation_identity(self):
        context = GoalOutputIdentity("observation", 0)
        self.assertTrue(
            GOAL_OUTPUT_API._canonical_goal_identity_ready(
                context, "target_yellow_cup"
            )
        )

    def test_target_route_still_requires_graph_for_graph_owner(self):
        context = GoalOutputIdentity("graph", 0)
        self.assertFalse(
            GOAL_OUTPUT_API._canonical_goal_identity_ready(
                context, "target_yellow_cup"
            )
        )
        self.assertEqual(
            context.events[0][1]["reason"], "missing_inherited_route_identity"
        )


class BridgeInstructionIdentityTest(unittest.TestCase):
    def test_observation_mission_goal_is_admitted(self):
        _identity, reason = BRIDGE_API._canonical_instruction_identity(
            message(mission_payload("observation", 0))
        )
        self.assertEqual(reason, "")

    def test_graph_mission_goal_with_zero_graph_id_is_rejected(self):
        _identity, reason = BRIDGE_API._canonical_instruction_identity(
            message(mission_payload("graph", 0))
        )
        self.assertEqual(reason, "missing_route_identity")

    def test_observation_work_item_requires_attempt(self):
        payload = mission_payload("observation", 0, attempt_id=0)
        payload["goal_context"].pop("attempt_id", None)
        _identity, reason = BRIDGE_API._canonical_instruction_identity(
            message(payload)
        )
        self.assertEqual(reason, "missing_observation_attempt_id")

    def test_geometry_observation_without_work_item_is_admitted(self):
        payload = mission_payload("observation", 0, work_item_id=0, attempt_id=0)
        payload["goal_context"] = {
            "goal_role": "geometry_frontier",
            "route_ownership": "observation",
        }
        _identity, reason = BRIDGE_API._canonical_instruction_identity(
            message(payload)
        )
        self.assertEqual(reason, "")

    def test_graph_route_with_positive_graph_id_is_admitted(self):
        _identity, reason = BRIDGE_API._canonical_instruction_identity(
            message(mission_payload("graph", 5))
        )
        self.assertEqual(reason, "")

    def test_missing_map_epoch_is_rejected_for_observation(self):
        _identity, reason = BRIDGE_API._canonical_instruction_identity(
            message(mission_payload("observation", 0, map_epoch=0))
        )
        self.assertEqual(reason, "missing_map_epoch")


if __name__ == "__main__":
    unittest.main()
