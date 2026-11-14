#!/usr/bin/env python3
"""A probe viewpoint inside the release radius settles from geometry.

env04 20261114_193957: the reverse-probe viewpoints were 0.93 m and 0.19 m
from the robot, inside the terminal-wait envelope.  The bridge never
dispatched a TEB action for such a close goal, so no route terminal arrived;
the frontier logged "reached endpoint geometry; waiting for matching TEB
terminal" until the probe active timeout (3x), then the exit blocked.

A probe is an observation, so proximity to its viewpoint is the arrival fact.
"""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_execution_resolution import (  # noqa: E402
    GlobalFrontierExecutionResolutionMixin,
)


class _Host(GlobalFrontierExecutionResolutionMixin):
    def __init__(self, probe_id=1):
        self.active_portal_probe_id = probe_id
        self.active_frontier_region_id = 4
        self.waypoint_release_radius = 0.4
        self.prefetched_frontier = None
        self.prefetched_goal_map = "stale"
        self.place_departure = SimpleNamespace(active=False)
        self.observed = []
        self.released = []

    def mark_frontier_observed(self, x, y, component=None, region_id=None):
        self.observed.append((x, y, component, region_id))
        return {"id": region_id}

    def release_active_frontier(self, **kwargs):
        self.released.append(kwargs)


class ProbeGeometrySettleTest(unittest.TestCase):
    def test_a_sub_metre_viewpoint_settles_the_probe(self):
        host = _Host()
        self.assertTrue(
            host.settle_probe_route_from_geometry(2.75, 4.85, 0.19, "room"))
        self.assertEqual(host.observed, [(2.75, 4.85, "room", 4)])
        self.assertEqual(host.released, [
            {"preserve_place_departure": False}])
        self.assertIsNone(host.prefetched_goal_map)

    def test_a_viewpoint_outside_the_release_radius_keeps_waiting(self):
        host = _Host()
        self.assertFalse(
            host.settle_probe_route_from_geometry(3.25, 4.25, 0.93, "room"))
        self.assertEqual(host.observed, [])
        self.assertEqual(host.released, [])

    def test_a_non_probe_route_is_untouched(self):
        host = _Host(probe_id=None)
        self.assertFalse(
            host.settle_probe_route_from_geometry(2.75, 4.85, 0.19, "room"))
        self.assertEqual(host.observed, [])


if __name__ == "__main__":
    unittest.main()
