"""Regression tests for durable observed-place route barriers."""

from pathlib import Path
import sys
import unittest

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_observation_place_state import (
    GlobalFrontierObservationPlaceStateMixin,
)


def component(label):
    return {
        "epoch": 9,
        "label": int(label),
        "cells": 20,
        "center_x": float(label),
        "center_y": 1.0,
        "min_x": float(label) - 0.5,
        "max_x": float(label) + 0.5,
        "min_y": 0.5,
        "max_y": 1.5,
    }


class Memory:
    def __init__(self):
        self.regions = {
            1: {"component": component(1)},
            2: {"component": component(2)},
        }

    @staticmethod
    def covered_viewpoints():
        return [(1, [(1.0, 1.0)]), (2, [(3.0, 1.0)])]

    def by_id(self, place_id):
        return self.regions.get(int(place_id))


class EnvelopeHost(GlobalFrontierObservationPlaceStateMixin):
    scan_range_max = 10.0
    endpoint_terminal_wait_radius = 0.85
    waypoint_release_radius = 1.20

    def __init__(self):
        self.region_memory = Memory()

    @staticmethod
    def observation_anchor_cells(_message, _known_free, anchors):
        return [(int(y), int(x)) for x, y in anchors]

    @staticmethod
    def observation_footprint_from_anchors(_message, known_free, anchors, **_kwargs):
        footprint = np.zeros_like(known_free, dtype=bool)
        for row, col in anchors:
            footprint[row, col] = True
        return footprint


class CoveredPlaceEnvelopeTest(unittest.TestCase):
    def test_current_place_is_exempt_but_another_observed_place_is_blocked(self):
        host = EnvelopeHost()
        footprint, anchor_count = host.covered_place_observation_footprint(
            message=object(),
            known_free=np.ones((5, 5), dtype=bool),
            components=object(),
            route_anchor_xy=(1.0, 1.0),
            source_component=component(1),
        )

        self.assertEqual(anchor_count, 1)
        self.assertFalse(footprint[1, 1])
        self.assertTrue(footprint[1, 3])

    def test_source_anchor_fallback_survives_an_unlabelled_slam_pose(self):
        host = EnvelopeHost()
        footprint, anchor_count = host.covered_place_observation_footprint(
            message=object(),
            known_free=np.ones((5, 5), dtype=bool),
            components=object(),
            route_anchor_xy=(1.4, 1.0),
            source_component=None,
        )

        self.assertEqual(anchor_count, 1)
        self.assertFalse(footprint[1, 1])
        self.assertTrue(footprint[1, 3])


if __name__ == "__main__":
    unittest.main()
