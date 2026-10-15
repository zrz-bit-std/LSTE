#!/usr/bin/env python3
"""Regression coverage for geometry-only activation state isolation."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_activation import GlobalFrontierActivationMixin


class GeometryBaselineActivationProbe(GlobalFrontierActivationMixin):
    def __init__(self):
        self.place_memory_enabled = False
        self.current_physical_place_id = 99

    @staticmethod
    def route_crosses_place_boundary(_hops):
        return False

    def activate_frontier_region(self, *_args, **_kwargs):
        raise AssertionError("geometry baseline must not activate a Place")


class FrontierActivationMethodBoundaryTest(unittest.TestCase):
    def test_geometry_baseline_does_not_create_a_place_for_accepted_frontier(self):
        probe = GeometryBaselineActivationProbe()
        selection = SimpleNamespace(
            route_kind="frontier_endpoint",
            place_hops=None,
            x=2.0,
            y=3.0,
            information=7.0,
            component=None,
        )

        region, departure = probe.prepare_selected_frontier_lifecycle(
            message=None,
            components=None,
            known_free=None,
            robot_map=(0.0, 0.0),
            now=1.0,
            selection=selection,
            selection_mode="strict_clearance",
        )

        self.assertIsNone(region)
        self.assertIsNone(departure)
        self.assertIsNone(probe.current_physical_place_id)


if __name__ == "__main__":
    unittest.main()
