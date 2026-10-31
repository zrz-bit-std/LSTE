"""Lethal-cell encoding contract for the exploration reachability gates.

Run ``20261031_165652`` dispatched goals whose cell was *lethal* in the global
costmap (verified from the sealed bag: ``/map`` = 0, place mask = -1,
``/move_base/global_costmap/costmap`` = 100). Navfn then returned an empty plan,
move_base aborted, and every route died a few seconds after dispatch.

The gates compared against ``>= 253`` -- the internal ``costmap_2d`` encoding --
but ``Costmap2DROS`` publishes the same grid scaled to 0-100 with ``-1`` for
unknown. A lethal 100 was therefore *below* the threshold and every reachability
and endpoint-clearance check silently passed.
"""

from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_planning_costmap import (  # noqa: E402
    GlobalFrontierPlanningCostmapMixin,
    lethal_mask,
    lethal_threshold,
)


class _Harness(GlobalFrontierPlanningCostmapMixin):
    clearance = 0.52
    costmap_inscribed_radius = 0.30


def message(width=15, height=15, resolution=0.10):
    return SimpleNamespace(
        info=SimpleNamespace(width=width, height=height, resolution=resolution),
        header=SimpleNamespace(frame_id="map"),
    )


def grid(lethal_value, unknown_value=-1, size=15):
    data = np.zeros((size, size), dtype=np.int16)
    data[7][7] = lethal_value          # robot cell
    data[3][7] = lethal_value          # 0.40 m away
    data[0][0] = unknown_value
    return data


class LethalEncodingTest(unittest.TestCase):
    def test_scaled_grid_uses_100(self):
        self.assertEqual(lethal_threshold(grid(100)), 100)

    def test_raw_grid_uses_253(self):
        self.assertEqual(lethal_threshold(grid(254)), 253)

    def test_empty_grid_falls_back_to_the_raw_threshold(self):
        # Nothing can be lethal in an empty grid; falling back to the internal
        # encoding is the safe default and never blocks a real candidate.
        self.assertEqual(lethal_threshold(np.zeros((0, 0), dtype=np.int16)), 253)

    def test_mask_flags_lethal_in_both_encodings(self):
        scaled = lethal_mask(grid(100))
        raw = lethal_mask(grid(254))
        self.assertTrue(scaled[3][7])
        self.assertTrue(raw[3][7])
        self.assertFalse(scaled[0][0])   # unknown is not lethal
        self.assertFalse(raw[0][0])

    def test_endpoint_with_lethal_cell_inside_the_margin_is_rejected(self):
        # The gate's margin is ceil((0.52 - 0.30) / 0.10) = 3 cells; the
        # costmap's own inscribed band is expected to cover the footprint.
        harness = _Harness()
        data = np.zeros((15, 15), dtype=np.int16)
        data[7][7] = 0
        data[5][7] = 100          # 2 cells = 0.20 m away
        self.assertFalse(harness._endpoint_has_lethal_margin(
            data, message(), (7, 7)))

    def test_endpoint_clear_of_lethal_is_accepted(self):
        harness = _Harness()
        data = np.zeros((21, 21), dtype=np.int16)
        data[10][10] = 0
        data[10][17] = 100            # 7 cells = 0.70 m away, outside margin
        self.assertTrue(harness._endpoint_has_lethal_margin(
            data, message(width=21, height=21), (10, 10)))

    def test_raw_encoding_is_unchanged(self):
        harness = _Harness()
        data = grid(254)
        self.assertFalse(harness._endpoint_has_lethal_margin(
            data, message(), (7, 7)))


class FreeMaskTest(unittest.TestCase):
    """``costmap_steps`` must treat a lethal 100 as blocked, not as free."""

    @classmethod
    def setUpClass(cls):
        source = (SCRIPTS / "global_frontier_planning_costmap.py").read_text(
            encoding="utf-8")
        cls.source = source

    def test_free_mask_no_longer_uses_the_raw_threshold(self):
        self.assertNotIn("free = data < 253", self.source)
        self.assertIn("free = ~lethal_mask(data)", self.source)

    def test_endpoint_gate_no_longer_uses_the_raw_threshold(self):
        self.assertNotIn("data[cell] >= 253", self.source)
        self.assertIn("lethal_threshold(data)", self.source)

    def test_navfn_startup_probe_uses_the_shared_mask(self):
        navfn = (SCRIPTS / "global_frontier_planning_navfn.py").read_text(
            encoding="utf-8")
        self.assertNotIn("data < 253", navfn)
        self.assertIn("~lethal_mask(data)", navfn)


class SealedRunReplayTest(unittest.TestCase):
    """The exact cell the sealed run dispatched, reconstructed as a grid."""

    def test_goal_cell_100_is_now_rejected(self):
        harness = _Harness()
        # Sized like the sealed 15x15 window; goal 0.40 m from the robot.
        data = np.zeros((15, 15), dtype=np.int16)
        data[7][7] = 0
        data[3][7] = 100        # recorded lethal goal cell
        self.assertFalse(harness._endpoint_has_lethal_margin(
            data, message(), (3, 7)))
        # And the old rule would have accepted it.
        self.assertLess(100, 253)


if __name__ == "__main__":
    unittest.main(verbosity=2)
