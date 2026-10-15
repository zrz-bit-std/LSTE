#!/usr/bin/env python3
"""Unit tests for the ROS-independent frontier-grid primitives."""

from pathlib import Path
import sys
import unittest

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from global_frontier_grid import (
    bfs,
    conservative_clearance_cells,
    frontier_mask,
    inflate,
    route_path,
)
from global_frontier_observation_coverage import (
    GlobalFrontierObservationCoverageMixin,
)


class FrontierGridTest(unittest.TestCase):
    def test_inflate_keeps_a_circular_clearance_mask(self):
        occupied = np.zeros((5, 5), dtype=bool)
        occupied[2, 2] = True

        inflated = inflate(occupied, 1)

        self.assertTrue(inflated[2, 2])
        self.assertTrue(inflated[1, 2])
        self.assertTrue(inflated[2, 1])
        self.assertFalse(inflated[1, 1])

    def test_clearance_rounding_never_drops_the_required_cell(self):
        self.assertEqual(conservative_clearance_cells(0.52, 0.10), 6)
        self.assertEqual(conservative_clearance_cells(0.30, 0.10), 3)
        self.assertEqual(conservative_clearance_cells(0.0, 0.10), 1)

    def test_observation_endpoint_uses_route_relative_standoff(self):
        steps = bfs(np.ones((1, 12), dtype=bool), (0, 0))
        endpoint = GlobalFrontierObservationCoverageMixin.nearest_safe_approach(
            steps,
            0,
            10,
            4,
            preferred_steps=steps,
            preferred_mask=np.ones_like(steps, dtype=bool),
            standoff_cells=4,
        )
        self.assertEqual(endpoint, (0, 6))

    def test_bfs_route_and_frontier_use_four_connected_cells(self):
        free = np.ones((4, 5), dtype=bool)
        free[1, 2] = False
        unknown = np.zeros_like(free)
        unknown[3, 4] = True

        steps = bfs(free, (0, 0))
        path = route_path(steps, (0, 0), (2, 4))
        frontier = frontier_mask(free, unknown)

        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (2, 4))
        self.assertEqual(len(path) - 1, int(steps[2, 4]))
        self.assertTrue(frontier[2, 4])
        self.assertTrue(frontier[3, 3])
        self.assertFalse(frontier[3, 4])


if __name__ == "__main__":
    unittest.main()
