#!/usr/bin/env python3
"""Ownership contract for graph-owned versus observation-owned routes.

Covers the cold-start deadlock: a GP work item / frontier endpoint owns no
durable graph transaction, so ``graph_transaction_id == 0`` is legal for it.
Portal probe/crossing routes remain strict and still participate in the graph
high-water mark.
"""

import ast
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "lste_core" / "scripts"))

from utils.route_ownership import (  # noqa: E402
    GRAPH,
    OBSERVATION,
    classify_route_ownership,
    graph_route_admission_error,
    observation_identity,
    observation_identity_error,
    normalize_ownership,
)

SOURCE = ROOT / "src" / "lste_topo_access" / "scripts" / "goal_manager_lifecycle.py"


def load_identity_api():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    mixin = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GoalManagerLifecycleMixin"
    )
    wanted = {"_canonical_route_identity", "_canonical_route_stale_reason"}
    body = [
        node for node in mixin.body
        if isinstance(node, ast.FunctionDef) and node.name in wanted
    ]
    cls = ast.ClassDef(
        name="RouteIdentity", bases=[], keywords=[], body=body, decorator_list=[],
    )
    module = ast.Module(body=[cls], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {
        "json": json,
        "classify_route_ownership": classify_route_ownership,
        "GRAPH": GRAPH,
        "OBSERVATION": OBSERVATION,
    }
    exec(compile(module, str(SOURCE), "exec"), namespace)
    return namespace["RouteIdentity"]


class StaleContext:
    def __init__(self, graph_high_water=0, lifecycle_high_water=0):
        self.canonical_route_high_water = 0
        self.canonical_graph_high_water = graph_high_water
        self.canonical_map_epoch_high_water = 0
        self.canonical_origin_epoch_high_water = 0
        self.canonical_projection_epoch_high_water = 0
        self.lifecycle_manager = type(
            "Lifecycle",
            (),
            {"transaction_high_water_mark": lambda _self: lifecycle_high_water},
        )()


def message(payload):
    return type("Message", (), {"data": json.dumps(payload)})()


def observation_payload(**overrides):
    payload = {
        "event": "route_command",
        "route_ownership": OBSERVATION,
        "lifecycle_transaction_id": 11,
        "route_id": 7,
        "graph_transaction_id": 0,
        "map_epoch": 4,
        "origin_epoch": 4,
        "projection_epoch": 4,
        "work_item_id": 3,
        "attempt_id": 2,
        "goal_context": {
            "goal_role": "place_observation_work",
            "route_ownership": OBSERVATION,
            "work_item_id": 3,
            "attempt_id": 2,
        },
    }
    payload.update(overrides)
    return payload


def graph_payload(**overrides):
    payload = {
        "event": "route_command",
        "route_ownership": GRAPH,
        "lifecycle_transaction_id": 11,
        "route_id": 8,
        "graph_transaction_id": 5,
        "map_epoch": 4,
        "origin_epoch": 4,
        "projection_epoch": 4,
        "goal_context": {"goal_role": "portal_probe"},
    }
    payload.update(overrides)
    return payload


class RouteOwnershipClassificationTest(unittest.TestCase):
    def test_explicit_ownership_wins(self):
        self.assertEqual(
            classify_route_ownership({"route_ownership": OBSERVATION}),
            OBSERVATION,
        )
        self.assertEqual(
            classify_route_ownership({"route_ownership": GRAPH}), GRAPH,
        )

    def test_goal_role_infers_observation(self):
        self.assertEqual(
            classify_route_ownership({
                "goal_context": {"goal_role": "place_observation_work"},
            }),
            OBSERVATION,
        )
        self.assertEqual(
            classify_route_ownership({
                "goal_context": {"goal_role": "geometry_frontier"},
            }),
            OBSERVATION,
        )

    def test_portal_role_is_graph_owned(self):
        self.assertEqual(
            classify_route_ownership({
                "goal_context": {"goal_role": "portal_probe"},
            }),
            GRAPH,
        )

    def test_unknown_payload_fails_closed_as_graph(self):
        self.assertEqual(classify_route_ownership({}), GRAPH)
        self.assertEqual(classify_route_ownership({"route_ownership": "bogus"}), GRAPH)
        self.assertEqual(normalize_ownership("bogus"), "")

    def test_observation_identity_reads_context_fallback(self):
        payload = {"goal_context": {"work_item_id": 4, "attempt_id": 9}}
        self.assertEqual(observation_identity(payload), (4, 9))


class GoalManagerRouteIdentityTest(unittest.TestCase):
    api = load_identity_api()

    def test_observation_route_with_zero_graph_id_is_accepted(self):
        identity, reason = self.api._canonical_route_identity(message(observation_payload()))
        self.assertEqual(reason, "")
        self.assertEqual(identity["route_ownership"], OBSERVATION)
        self.assertEqual(identity["graph_transaction_id"], 0)
        self.assertEqual(identity["work_item_id"], 3)
        self.assertEqual(identity["attempt_id"], 2)

    def test_graph_route_with_zero_graph_id_is_rejected(self):
        _identity, reason = self.api._canonical_route_identity(
            message(graph_payload(graph_transaction_id=0))
        )
        self.assertEqual(reason, "missing_graph_transaction_id")

    def test_graph_route_with_positive_graph_id_is_accepted(self):
        _identity, reason = self.api._canonical_route_identity(message(graph_payload()))
        self.assertEqual(reason, "")

    def test_observation_work_item_requires_an_attempt(self):
        payload = observation_payload(attempt_id=0)
        payload["goal_context"].pop("attempt_id", None)
        _identity, reason = self.api._canonical_route_identity(message(payload))
        self.assertEqual(reason, "missing_observation_attempt_id")

    def test_geometry_observation_needs_no_work_item(self):
        payload = observation_payload(work_item_id=None, attempt_id=None)
        payload["goal_context"] = {
            "goal_role": "geometry_frontier",
            "route_ownership": OBSERVATION,
        }
        _identity, reason = self.api._canonical_route_identity(message(payload))
        self.assertEqual(reason, "")

    def test_missing_epoch_is_rejected_for_both_classes(self):
        for payload in (observation_payload(map_epoch=0), graph_payload(map_epoch=0)):
            _identity, reason = self.api._canonical_route_identity(message(payload))
            self.assertEqual(reason, "missing_map_epoch")

    def test_unknown_payload_fails_closed_as_graph(self):
        payload = observation_payload()
        payload.pop("route_ownership")
        payload["goal_context"] = {}
        payload["graph_transaction_id"] = 0
        _identity, reason = self.api._canonical_route_identity(message(payload))
        self.assertEqual(reason, "missing_graph_transaction_id")


class GoalManagerStaleIdentityTest(unittest.TestCase):
    api = load_identity_api()

    def identity(self, payload):
        identity, reason = self.api._canonical_route_identity(message(payload))
        self.assertEqual(reason, "")
        return identity

    def test_observation_route_skips_graph_high_water(self):
        identity = self.identity(observation_payload(graph_transaction_id=0))
        self.assertEqual(
            self.api._canonical_route_stale_reason(
                StaleContext(graph_high_water=5), identity
            ),
            "",
        )

    def test_graph_route_still_checks_graph_high_water(self):
        identity = self.identity(graph_payload(graph_transaction_id=1))
        self.assertEqual(
            self.api._canonical_route_stale_reason(
                StaleContext(graph_high_water=5), identity
            ),
            "graph_transaction_id_below_high_water",
        )

    def test_observation_route_is_stale_when_lifecycle_regresses(self):
        identity = self.identity(observation_payload(lifecycle_transaction_id=3))
        self.assertEqual(
            self.api._canonical_route_stale_reason(
                StaleContext(lifecycle_high_water=10), identity
            ),
            "lifecycle_transaction_id_below_high_water",
        )


class BridgeAdmissionTest(unittest.TestCase):
    def test_observation_route_with_zero_graph_id_is_admitted(self):
        self.assertEqual(
            graph_route_admission_error(
                observation_payload(), route_id=7, source="global_slam_frontier"
            ),
            "",
        )

    def test_graph_route_with_zero_graph_id_is_rejected(self):
        self.assertEqual(
            graph_route_admission_error(
                graph_payload(graph_transaction_id=0),
                route_id=8,
                source="global_slam_frontier",
            ),
            "missing_graph_transaction_id",
        )

    def test_observation_work_item_without_attempt_is_rejected(self):
        payload = observation_payload(attempt_id=0)
        payload["goal_context"].pop("attempt_id", None)
        self.assertEqual(
            graph_route_admission_error(
                payload, route_id=7, source="global_slam_frontier"
            ),
            "missing_observation_attempt_id",
        )

    def test_non_frontier_source_is_not_subject_to_the_contract(self):
        self.assertEqual(
            graph_route_admission_error(
                graph_payload(graph_transaction_id=0), route_id=8, source="brain"
            ),
            "",
        )

    def test_observation_identity_error_is_empty_without_work_item(self):
        self.assertEqual(observation_identity_error({"work_item_id": 0}), "")


if __name__ == "__main__":
    unittest.main()
