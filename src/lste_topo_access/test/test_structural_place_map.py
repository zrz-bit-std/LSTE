#!/usr/bin/env python3
"""Regression tests for the architecture-aware online place map.

These maps contain no Gazebo room labels.  The tests only encode observable
occupancy geometry: a compact table must not make two places, while a wall and
doorway still must.
"""

from pathlib import Path
import sys
import unittest

import numpy as np

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from global_frontier_topology import StructuralPlaceMap, TopologicalFreeSpaceComponents


class StructuralPlaceMapTest(unittest.TestCase):
    resolution = 0.10

    def place_map(self, known_free, occupied):
        return StructuralPlaceMap(
            known_free,
            occupied,
            self.resolution,
            place_clearance_m=0.50,
            furniture_max_span_m=2.50,
            epoch=1,
        )

    def test_compact_table_does_not_split_one_structural_place(self):
        # A 3.2 m x 3.2 m observed room with a 2.0 m x 1.4 m conference table
        # leaves physically traversable, but low-clearance, passages around
        # the table. A raw inflated-core map separates its upper/lower cores.
        # The structural map intentionally fills only that compact furnishing
        # while preserving raw occupancy for navigation.
        known_free = np.zeros((48, 48), dtype=bool)
        known_free[8:40, 8:40] = True
        occupied = np.zeros_like(known_free)
        occupied[7:41, 7] = True
        occupied[7:41, 40] = True
        occupied[7, 7:41] = True
        occupied[40, 7:41] = True
        # The table is broad enough that core inflation joins it to both side
        # walls, although the raw free map still has narrow physical passages.
        occupied[18:30, 12:36] = True
        known_free[occupied] = False

        raw_core = known_free & ~StructuralPlaceMap._inflate(occupied, 4)
        raw = TopologicalFreeSpaceComponents(raw_core, epoch=1)
        self.assertGreater(int(raw.labels[12, 24]), 0)
        self.assertGreater(int(raw.labels[34, 24]), 0)
        self.assertNotEqual(int(raw.labels[12, 24]), int(raw.labels[34, 24]))

        places = self.place_map(known_free, occupied)

        self.assertTrue(np.all(places.furniture_mask[18:30, 12:36]))
        self.assertGreater(int(places.labels[12, 24]), 0)
        self.assertEqual(
            int(places.labels[12, 24]), int(places.labels[34, 24])
        )
        # The physical map is an input and must never be mutated by a high
        # level place abstraction.
        self.assertTrue(np.all(occupied[18:30, 12:36]))

    def test_structural_wall_and_doorway_still_separate_places(self):
        # A vertical internal wall separates two rooms. Its 0.8 m doorway is
        # traversable to the vehicle, but the larger structural-place
        # clearance removes that throat and creates two place nodes.
        known_free = np.zeros((60, 80), dtype=bool)
        known_free[6:54, 6:74] = True
        occupied = np.zeros_like(known_free)
        occupied[5:55, 5] = True
        occupied[5:55, 74] = True
        occupied[5, 5:75] = True
        occupied[54, 5:75] = True
        occupied[6:54, 40] = True
        occupied[27:35, 40] = False
        known_free[occupied] = False

        places = self.place_map(known_free, occupied)

        self.assertFalse(np.any(places.furniture_mask[6:54, 40]))
        left = int(places.labels[15, 18])
        right = int(places.labels[15, 62])
        self.assertGreater(left, 0)
        self.assertGreater(right, 0)
        self.assertNotEqual(left, right)

    def test_unobserved_or_long_obstacle_is_never_reclassified_as_furniture(self):
        known_free = np.zeros((60, 80), dtype=bool)
        known_free[10:50, 8:72] = True
        occupied = np.zeros_like(known_free)
        # This looks like an interior partition but its span is 4 m, beyond
        # the structural furnishing limit. It must remain a place boundary.
        occupied[12:48, 39:41] = True
        known_free[occupied] = False

        places = self.place_map(known_free, occupied)

        self.assertFalse(np.any(places.furniture_mask[12:48, 39:41]))
        self.assertTrue(np.all(places.structural_occupied[12:48, 39:41]))


if __name__ == "__main__":
    unittest.main()
