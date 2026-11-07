"""Canonical identity for non-frontier GoalManager intents.

A context-follow or in-place-observation goal runs through the same TEB
bridge as a frontier route. It must therefore carry the same canonical map
epoch and inherited route identity; otherwise the bridge rejects it and the
robot stays parked at the previous endpoint.
"""

import ast
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[3]
SOURCE = (
    ROOT / "src" / "lste_topo_access" / "scripts" / "goal_manager_goal_output.py"
)


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
    namespace = {"math": math}
    exec(compile(module, str(source), "exec"), namespace)
    return namespace["Holder"]


GeometryIdentity = load_method(
    SOURCE, "GoalManagerGoalOutputMixin", "_apply_geometry_route_identity"
)


class Owner(GeometryIdentity):
    def __init__(self, enabled=True, epoch=70, route_id=5,
                 origin=None, projection=None):
        self.global_frontier_enabled = enabled
        self.global_frontier_map_epoch = epoch
        self.global_frontier_route_id = route_id
        self.global_frontier_origin_epoch = origin
        self.global_frontier_projection_epoch = projection

    def _canonical_map_epoch(self):
        return max(0, int(self.global_frontier_map_epoch or 0))


class GeometryRouteIdentityTest(unittest.TestCase):
    def test_context_goal_inherits_map_epoch_and_route(self):
        intent = Owner()._apply_geometry_route_identity(
            {"source": "ctx_pair_follow", "route_id": 0}
        )
        self.assertEqual(intent["map_epoch"], 70)
        self.assertEqual(intent["origin_epoch"], 70)
        self.assertEqual(intent["projection_epoch"], 70)
        self.assertEqual(intent["route_id"], 5)
        self.assertEqual(intent["route_ownership"], "observation")

    def test_observation_goal_inherits_identity(self):
        intent = Owner(origin=68, projection=69)._apply_geometry_route_identity(
            {"source": "ctx_observe_in_place"}
        )
        self.assertEqual(intent["map_epoch"], 70)
        self.assertEqual(intent["origin_epoch"], 68)
        self.assertEqual(intent["projection_epoch"], 69)
        self.assertEqual(intent["route_id"], 5)

    def test_missing_route_does_not_invent_one(self):
        intent = Owner(route_id=0)._apply_geometry_route_identity(
            {"source": "ctx_pair_cached"}
        )
        self.assertEqual(intent["map_epoch"], 70)
        self.assertNotIn("route_id", intent)

    def test_frontier_goal_is_untouched(self):
        payload = {"source": "global_slam_frontier", "route_id": 9}
        intent = Owner()._apply_geometry_route_identity(dict(payload))
        self.assertEqual(intent, payload)

    def test_target_goal_is_untouched(self):
        payload = {"source": "target_follow", "route_id": 3}
        intent = Owner()._apply_geometry_route_identity(dict(payload))
        self.assertEqual(intent, payload)

    def test_missing_canonical_epoch_does_not_fabricate_one(self):
        intent = Owner(epoch=0)._apply_geometry_route_identity(
            {"source": "ctx_pair_follow"}
        )
        self.assertNotIn("map_epoch", intent)

    def test_legacy_mode_is_untouched(self):
        payload = {"source": "ctx_pair_follow"}
        intent = Owner(enabled=False)._apply_geometry_route_identity(
            dict(payload)
        )
        self.assertEqual(intent, payload)


if __name__ == "__main__":
    unittest.main()
