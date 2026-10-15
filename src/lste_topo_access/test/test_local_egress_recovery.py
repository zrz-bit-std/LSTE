#!/usr/bin/env python3
"""Regression tests for safe-history local egress recovery."""

import ast
from pathlib import Path
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
SOURCE = SCRIPTS_DIR / "global_frontier_route_state.py"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from global_frontier_route_history import ReachedRouteHistory


def load_methods():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    explorer = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "GlobalFrontierRouteStateMixin"
    )
    names = {"local_egress_anchor", "prepare_local_egress"}
    selected = [
        node for node in explorer.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace


METHODS = load_methods()


class EgressExplorer:
    local_egress_anchor = METHODS["local_egress_anchor"]
    prepare_local_egress = METHODS["prepare_local_egress"]

    def __init__(self):
        self.active_route_kind = "frontier_endpoint"
        self.active_last_robot_xy = (3.0, 0.0)
        self.active_route_history = ReachedRouteHistory()
        self.active_route_history.begin((0.0, 0.0))
        self.active_route_history.remember((1.6, 0.0), 0.1)
        self.active_route_history.remember((3.0, 0.0), 0.1)
        self.active_route_history_frame = "map"
        self.waypoint_release_radius = 0.8
        self.frontier_approach_distance = 1.0
        self.active_route_id = 17
        self.active_frontier_region_id = 8
        self.active_frontier = (10, 11, 4.0, 0.0)
        self.pending_local_egress = None
        self.status = []
        self.map_msg = None

    @staticmethod
    def active_route_history_pose(fallback_map_pose):
        return "map", fallback_map_pose

    def publish_status(self, event, **fields):
        self.status.append((event, fields))


class OdomEgressExplorer:
    local_egress_anchor = METHODS["local_egress_anchor"]

    def __init__(self):
        self.active_last_robot_xy = (99.0, 99.0)
        self.active_route_history = ReachedRouteHistory()
        self.active_route_history.begin((0.0, 0.0))
        self.active_route_history.remember((1.6, 0.0), 0.1)
        self.active_route_history.remember((3.0, 0.0), 0.1)
        self.active_route_history_frame = "odom"
        self.waypoint_release_radius = 0.8
        self.frontier_approach_distance = 1.0
        self.map_msg = type(
            "Map", (), {"header": type("Header", (), {"frame_id": "map"})()}
        )()
        self.transforms = []

    @staticmethod
    def active_route_history_pose(_fallback_map_pose):
        return "odom", (3.0, 0.0)

    def transform_xy(self, target_frame, source_frame, x, y):
        self.transforms.append((target_frame, source_frame, x, y))
        return x + 10.0, y + 20.0


class LocalEgressRecoveryTest(unittest.TestCase):
    def test_selects_nearest_reached_anchor_outside_the_trap_radius(self):
        explorer = EgressExplorer()

        self.assertEqual(
            explorer.active_route_history.nearest_exit_anchor(
                explorer.active_last_robot_xy,
                max(explorer.waypoint_release_radius, explorer.frontier_approach_distance),
            ),
            (1.6, 0.0),
        )

    def test_queues_a_recovery_goal_from_reached_history_only(self):
        explorer = EgressExplorer()

        queued = explorer.prepare_local_egress("stall")

        self.assertTrue(queued)
        self.assertEqual(explorer.pending_local_egress["anchor_map"], (1.6, 0.0))
        self.assertEqual(explorer.pending_local_egress["source_route_id"], 17)
        self.assertEqual(explorer.pending_local_egress["source_region_id"], 8)
        self.assertEqual(explorer.status[0][0], "local_egress_prepared")

    def test_does_not_queue_an_egress_for_a_route_without_an_escape_anchor(self):
        explorer = EgressExplorer()
        explorer.active_route_history.begin((3.0, 0.0))

        self.assertFalse(explorer.prepare_local_egress("stall"))
        self.assertIsNone(explorer.pending_local_egress)

    def test_odom_history_anchor_is_transformed_only_when_selected(self):
        explorer = OdomEgressExplorer()

        self.assertEqual(explorer.local_egress_anchor(), (11.6, 20.0))
        self.assertEqual(explorer.transforms, [("map", "odom", 1.6, 0.0)])


if __name__ == "__main__":
    unittest.main()
