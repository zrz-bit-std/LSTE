"""A projected Place scope must not exclude every cell.

Run 20261031_183336 (L) published exactly one such frame: the odom-frame scope
at sim 1611.889 covered a 488x393 grid in which all 191,784 cells were marked
excluded. The native local costmap cannot enforce that (the robot would be
inside the forbidden set), so 2,360 of the 6,400 local cells stayed non-lethal
and the native costmap audit rejected the frame.

The producer now keeps the previous valid scope instead of publishing an
unenforceable raster. Every other scope still has to project exactly.
"""

from pathlib import Path
import sys
import unittest

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_place_navigation import (  # noqa: E402
    constraint_cells,
    degenerate_scope_raster,
)


class DegenerateScopeTest(unittest.TestCase):
    def test_sealed_all_excluded_frame_is_detected(self):
        # The exact shape and content measured from the sealed L bag.
        data = np.full((393, 488), 100, dtype=np.int8)
        self.assertTrue(degenerate_scope_raster(data))

    def test_normal_scope_is_not_degenerate(self):
        data = np.full((393, 488), 100, dtype=np.int8)
        data[10:20, 10:20] = -1          # a real Place region
        self.assertFalse(degenerate_scope_raster(data))

    def test_transparent_scope_is_not_degenerate(self):
        data = np.full((393, 488), -1, dtype=np.int8)
        self.assertFalse(degenerate_scope_raster(data))

    def test_empty_raster_is_not_flagged(self):
        self.assertFalse(degenerate_scope_raster(np.zeros((0, 0), dtype=np.int8)))

    def test_constraint_cells_unchanged(self):
        allowed = np.array([[True, False], [False, True]])
        self.assertEqual(constraint_cells(allowed).tolist(), [[-1, 100], [100, -1]])


class ProducerContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (SCRIPTS / "global_frontier_place_navigation.py").read_text(
            encoding="utf-8")

    def test_publish_refuses_degenerate_raster(self):
        self.assertIn("degenerate_scope_raster(local_data)", self.source)
        self.assertIn("keeping the previous", self.source)

    def test_guard_only_applies_to_bounded_scopes(self):
        self.assertIn(
            "if bounded and degenerate_scope_raster(local_data):", self.source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
