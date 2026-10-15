#!/usr/bin/env python3
"""Regression tests for the named global-frontier candidate boundary."""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from global_frontier_models import SelectedFrontier
from global_frontier_planning_prefetch import GlobalFrontierPlanningPrefetchMixin


class PrefetchProbe(GlobalFrontierPlanningPrefetchMixin):
    def __init__(self):
        self.prefetched_frontier = None
        self.active_frontier = (1, 1, 1.0, 1.0)
        self.active_route_id = 26
        self.heading_hard_limit = 3.14
        self.place_graph_hop_limit = 1
        self.region_memory = SimpleNamespace(
            candidate_tier=lambda *_args, **_kwargs: ("new", None)
        )
        self.published = []

    @staticmethod
    def choose_valid_frontier(*_args, **_kwargs):
        return (
            4,
            5,
            12.25,
            6.75,
            3.5,
            42.0,
            17.0,
            9.5,
            {
                "epoch": 3,
                "label": 7,
                "cells": 12,
                "center_x": 12.25,
                "center_y": 6.75,
                "min_x": 12.0,
                "max_x": 12.5,
                "min_y": 6.5,
                "max_y": 7.0,
            },
            0,
            "frontier_endpoint",
            (11.5, 6.5),
        )

    @staticmethod
    def route_headings(*_args):
        return 0.25, None

    @staticmethod
    def _candidate_route_heading_delta(*_args):
        return 0.1

    def publish_status(self, event, **fields):
        self.published.append((event, fields))


class FrontierCandidateContractTest(unittest.TestCase):
    def test_optional_portal_field_does_not_shift_prefetch_fields(self):
        component = {"label": 7, "cells": 12}
        selected = SelectedFrontier.from_candidate(
            (
                4,
                5,
                12.25,
                6.75,
                3.5,
                42.0,
                17.0,
                9.5,
                component,
                0,
                "frontier_endpoint",
                (11.5, 6.5),
            )
        )

        self.assertEqual((selected.row, selected.col), (4, 5))
        self.assertEqual((selected.x, selected.y), (12.25, 6.75))
        self.assertEqual(selected.component, component)
        self.assertEqual(selected.place_hops, 0)
        self.assertEqual(selected.route_kind, "frontier_endpoint")
        self.assertEqual(selected.portal_gate_xy, (11.5, 6.5))

    def test_legacy_candidate_without_portal_still_has_named_defaults(self):
        selected = SelectedFrontier.from_candidate(
            (4, 5, 12.25, 6.75, 3.5, 42.0, 17.0, 9.5, None, 0, "frontier_endpoint")
        )

        self.assertEqual(selected.portal_gate_xy, None)
        self.assertEqual(selected.route_kind, "frontier_endpoint")

    def test_prefetch_accepts_the_extended_candidate_contract(self):
        probe = PrefetchProbe()
        message = SimpleNamespace(
            info=SimpleNamespace(resolution=0.1),
        )
        steps = {(4, 5): 7}

        with patch("global_frontier_prefetch_selection.rospy.loginfo"):
            admitted = probe.prefetch_next_frontier(
                message,
                steps,
                frontier=None,
                unknown=None,
                occupied=None,
                now=10.0,
                active_xy=(1.0, 1.0),
                robot_map=(0.0, 0.0),
                route_seed=object(),
            )

        self.assertTrue(admitted)
        self.assertEqual(probe.prefetched_frontier, (4, 5, 12.25, 6.75))
        self.assertEqual(probe.prefetched_goal_map, (12.25, 6.75))
        self.assertEqual(probe.published[0][0], "frontier_prefetched")


if __name__ == "__main__":
    unittest.main()
