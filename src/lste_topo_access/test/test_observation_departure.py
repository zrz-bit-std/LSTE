#!/usr/bin/env python3
"""Regression tests for topology-independent observation-place exits."""

import ast
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
SOURCE = SCRIPTS_DIR / "global_frontier_observation_departure.py"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from global_frontier_observation_departure import (  # noqa: E402
    GlobalFrontierObservationDepartureMixin,
)
from global_frontier_place_lifecycle import PlaceDepartureTransaction  # noqa: E402


def load_departure_preparation():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    explorer = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierObservationDepartureMixin"
    )
    method = next(
        node for node in explorer.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "prepare_observation_place_departure"
    )
    namespace = {"np": np}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace["prepare_observation_place_departure"]


class DepartureTransaction:
    def __init__(self):
        self.calls = []
        self.basis = None

    def prepare_region(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        self.basis = kwargs["basis"]
        return args[0].by_id(args[1])


class RegionMemory:
    def __init__(self):
        self.region = {"id": 9, "state": "open"}

    def by_id(self, region_id):
        return self.region if int(region_id) == 9 else None

    @staticmethod
    def viewpoints_for(_region_id):
        return [(1.0, 1.0)]


class DepartureExplorer:
    prepare_observation_place_departure = load_departure_preparation()

    def __init__(self, target_inside=False):
        self.observation_departure_source = SimpleNamespace(
            region_id=9,
            anchor_map=(1.0, 1.0),
        )
        self.target_region_claim_active = False
        self.region_memory = RegionMemory()
        self.place_departure = DepartureTransaction()
        self.last_observation_departure_footprint_cells = 0
        self.last_observation_departure_anchor_count = 0
        self.target_inside = target_inside
        self.published = []

    @staticmethod
    def observation_anchor_cells(_message, _known_free, _anchors):
        return [(1, 1)]

    def observation_footprint_from_anchors(self, _message, known_free, _anchors):
        footprint = np.zeros_like(known_free, dtype=bool)
        if self.target_inside:
            footprint[2, 2] = True
        else:
            footprint[1, 1] = True
        return footprint

    @staticmethod
    def xy_to_grid_cell(_message, _x, _y):
        return 2, 2

    def publish_prepared_place_departure(self, *args, **kwargs):
        self.published.append((args, kwargs))


class StaleComponentMemory:
    """Return an old component owner to exercise exact Place ownership."""

    def __init__(self):
        self.regions = {
            7: {"id": 7, "state": "open", "visits": 1},
            9: {"id": 9, "state": "open", "visits": 1},
        }

    def by_id(self, region_id):
        return self.regions.get(int(region_id))

    def _nearest(self, *_args):
        return self.regions[7], "stale_component_owner"


class ExactOwnerDepartureExplorer(GlobalFrontierObservationDepartureMixin):
    def __init__(self):
        self.current_physical_place_id = 9
        self.region_memory = StaleComponentMemory()
        self.place_departure = PlaceDepartureTransaction()
        self.target_region_claim_active = False
        self.place_departure_min_travel_distance = 0.0
        self.published = []

    @staticmethod
    def component_at_map_position(*_args):
        return {
            "epoch": 4,
            "label": 2,
            "cells": 10,
            "center_x": 2.0,
            "center_y": 3.0,
            "min_x": 1.0,
            "max_x": 3.0,
            "min_y": 2.0,
            "max_y": 4.0,
        }

    def publish_status(self, event, **fields):
        self.published.append((event, fields))

    def publish_prepared_place_departure(self, *args, **kwargs):
        self.published.append((args, kwargs))


class ObservationDepartureTest(unittest.TestCase):
    def test_outside_observed_submap_prepares_exact_source_region(self):
        explorer = DepartureExplorer(target_inside=False)
        selected = SimpleNamespace(x=4.0, y=4.0)

        departed = explorer.prepare_observation_place_departure(
            message=SimpleNamespace(),
            known_free=np.ones((5, 5), dtype=bool),
            selection=selected,
        )

        self.assertIs(departed, explorer.region_memory.region)
        self.assertEqual(len(explorer.place_departure.calls), 1)
        _args, kwargs = explorer.place_departure.calls[0]
        self.assertEqual(kwargs["basis"], "observation_boundary")
        self.assertEqual(len(explorer.published), 1)

    def test_candidate_inside_observed_submap_does_not_close_current_place(self):
        explorer = DepartureExplorer(target_inside=True)
        selected = SimpleNamespace(x=2.0, y=2.0)

        departed = explorer.prepare_observation_place_departure(
            message=SimpleNamespace(),
            known_free=np.ones((5, 5), dtype=bool),
            selection=selected,
        )

        self.assertIsNone(departed)
        self.assertEqual(explorer.place_departure.calls, [])
        self.assertEqual(explorer.published, [])

    def test_structural_departure_uses_exact_current_place_owner(self):
        explorer = ExactOwnerDepartureExplorer()

        departed = explorer.prepare_place_departure(
            message=SimpleNamespace(),
            components=SimpleNamespace(epoch=4),
            known_free=np.ones((2, 2), dtype=bool),
            robot_map=(2.0, 3.0),
            place_hops=1,
        )

        self.assertEqual(departed["id"], 9)
        self.assertEqual(explorer.place_departure.region_id, 9)


if __name__ == "__main__":
    unittest.main()
