"""Pure contracts for source-side doorway observation probes."""

from pathlib import Path
import sys
import unittest

import numpy as np
from types import SimpleNamespace


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_portal_probes import portal_observation_probe
from global_frontier_candidate_portals import GlobalFrontierCandidatePortalMixin


class ProbeHarness(GlobalFrontierCandidatePortalMixin):
    scan_observation_horizon = 4.0
    region_topology_clearance = 0.77
    place_furniture_max_span_m = 2.5

    @staticmethod
    def request(unknown, structural_occupied):
        return SimpleNamespace(
            unknown=unknown,
            components=SimpleNamespace(structural_occupied=structural_occupied),
            message=SimpleNamespace(
                info=SimpleNamespace(resolution=0.1)
            ),
        )


class PortalObservationProbeTest(unittest.TestCase):
    def doorway_walls(self):
        walls = np.zeros((9, 9), dtype=bool)
        # A vertical passage through a horizontal wall: the source is left of
        # the opening, so the probe normal points right. Wall runs extend away
        # from the opening in both tangent directions.
        walls[1:4, 4] = True
        walls[5:8, 4] = True
        return walls

    def test_unknown_wall_bounded_opening_creates_source_side_probe(self):
        unknown = np.zeros((9, 9), dtype=bool)
        unknown[4, 5] = True

        probe = portal_observation_probe(
            unknown,
            self.doorway_walls(),
            (4, 4),
            support_radius=1,
            minimum_wall_span_cells=3,
        )

        self.assertIsNotNone(probe)
        self.assertEqual(probe.opening_cell, (4, 4))
        self.assertEqual(probe.normal, (0, 1))

    def test_unknown_without_opposing_structural_walls_is_not_a_probe(self):
        unknown = np.zeros((9, 9), dtype=bool)
        unknown[4, 5] = True

        self.assertIsNone(portal_observation_probe(
            unknown,
            np.zeros_like(unknown),
            (4, 4),
            support_radius=1,
            minimum_wall_span_cells=3,
        ))

    def test_known_far_side_cannot_become_a_probe(self):
        unknown = np.zeros((9, 9), dtype=bool)

        self.assertIsNone(portal_observation_probe(
            unknown,
            self.doorway_walls(),
            (4, 4),
            support_radius=1,
            minimum_wall_span_cells=3,
        ))

    def test_partially_observed_wall_uses_scan_horizon_for_probe_evidence(self):
        walls = np.zeros((15, 15), dtype=bool)
        walls[2:6, 7] = True
        walls[8:12, 7] = True
        unknown = np.zeros_like(walls)
        unknown[6, 8] = True
        probe = ProbeHarness().portal_observation_probe_for_frontier(
            ProbeHarness.request(unknown, walls), 6, 7,
        )
        self.assertIsNotNone(probe)
        self.assertEqual(probe.opening_cell, (6, 7))


if __name__ == "__main__":
    unittest.main()
