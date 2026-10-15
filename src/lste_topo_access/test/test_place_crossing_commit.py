#!/usr/bin/env python3
"""Regression tests for closing a place after physical doorway crossing."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
SOURCE = SCRIPTS_DIR / "global_frontier_observation_departure.py"


def load_crossing_commit():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    explorer = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierObservationDepartureMixin"
    )
    method = next(
        node for node in explorer.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "commit_departed_place_after_boundary_crossing"
    )
    namespace = {"math": math}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace["commit_departed_place_after_boundary_crossing"]


class RegionMemory:
    def __init__(self):
        self.region = {"id": 7, "state": "open"}

    def by_id(self, region_id):
        return self.region if int(region_id) == 7 else None

    @staticmethod
    def viewpoints_for(_region_id):
        return [(1.0, 1.0)]


class Departure:
    active = True
    region_id = 7
    basis = "observation_boundary"
    anchor_map = (1.0, 1.0)
    minimum_travel_distance = 1.25
    exit_wait_reported = False

    def __init__(self):
        self.cleared = False

    def clear(self):
        self.cleared = True


class CrossingExplorer:
    commit_departed_place_after_boundary_crossing = load_crossing_commit()

    def __init__(self, robot_cell):
        self.place_departure = Departure()
        self.region_memory = RegionMemory()
        self.active_route_id = 14
        self.robot_cell = robot_cell
        self.status = []
        self.commits = 0

    @staticmethod
    def observation_anchor_cells(_message, _known_free, _viewpoints):
        return [(1, 1)]

    @staticmethod
    def observation_footprint_from_anchors(
        _message, known_free, _anchors, components=None,
    ):
        assert components == "structural-components"
        footprint = np.zeros_like(known_free, dtype=bool)
        footprint[1:3, 1:3] = True
        return footprint

    def xy_to_grid_cell(self, _message, _x, _y):
        return self.robot_cell

    def publish_status(self, event, **fields):
        self.status.append((event, fields))

    def commit_active_place_departure(self):
        self.commits += 1
        self.region_memory.region["state"] = "dormant"
        self.place_departure.active = False
        return self.region_memory.region


def snapshot(robot_map=(5.0, 5.0)):
    return SimpleNamespace(
        message=object(),
        robot_map=robot_map,
        map_context=SimpleNamespace(
            known_free=np.ones((5, 5), dtype=bool),
            components="structural-components",
        ),
    )


class PlaceCrossingCommitTest(unittest.TestCase):
    def test_keeps_the_source_place_open_while_the_robot_is_still_inside(self):
        explorer = CrossingExplorer(robot_cell=(2, 2))

        self.assertIsNone(explorer.commit_departed_place_after_boundary_crossing(snapshot()))
        self.assertEqual(explorer.commits, 0)
        self.assertEqual(explorer.status, [])

    def test_closes_the_source_place_after_a_replan_crosses_its_boundary(self):
        explorer = CrossingExplorer(robot_cell=(4, 4))

        closed = explorer.commit_departed_place_after_boundary_crossing(snapshot())

        self.assertEqual(closed["state"], "dormant")
        self.assertEqual(explorer.commits, 1)
        self.assertEqual(explorer.status[0][0], "frontier_place_boundary_crossed")
        self.assertEqual(explorer.status[0][1]["region_id"], 7)

    def test_does_not_close_at_the_departure_anchor_when_a_map_update_drops_it(self):
        # The old implementation recomputed a footprint after selecting the
        # outbound route and closed immediately if the source cell had shifted
        # out of that raster.  An outside cell is not crossing evidence until
        # the base has actually translated away from the departure anchor.
        explorer = CrossingExplorer(robot_cell=(4, 4))

        self.assertIsNone(
            explorer.commit_departed_place_after_boundary_crossing(
                snapshot(robot_map=(1.02, 1.01))
            )
        )
        self.assertEqual(explorer.commits, 0)
        self.assertEqual(explorer.status[0][0], "frontier_place_departure_waiting")

        closed = explorer.commit_departed_place_after_boundary_crossing(
            snapshot(robot_map=(4.0, 4.0))
        )
        self.assertEqual(closed["state"], "dormant")
        self.assertEqual(explorer.commits, 1)

    def test_portal_requires_directional_crossing_before_map_closure(self):
        explorer = CrossingExplorer(robot_cell=(4, 4))
        explorer.place_departure.requires_physical_gate_crossing = True
        explorer.place_departure.physical_gate_crossing_confirmed = False

        self.assertIsNone(explorer.commit_departed_place_after_boundary_crossing(snapshot()))
        self.assertEqual(explorer.commits, 0)
        self.assertEqual(
            explorer.status[-1][0], "frontier_place_departure_waiting",
        )
        self.assertEqual(
            explorer.status[-1][1]["reason"],
            "waiting_for_physical_portal_crossing",
        )

        explorer.place_departure.physical_gate_crossing_confirmed = True
        closed = explorer.commit_departed_place_after_boundary_crossing(snapshot())
        self.assertEqual(closed["state"], "dormant")
        self.assertEqual(explorer.commits, 1)


if __name__ == "__main__":
    unittest.main()
