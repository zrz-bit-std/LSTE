"""Endpoint safety contract for the live costmap admission gate."""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_planning_costmap import (  # noqa: E402
    GlobalFrontierPlanningCostmapMixin,
)


class _Harness(GlobalFrontierPlanningCostmapMixin):
    clearance = 0.52
    costmap_inscribed_radius = 0.30


class CostmapEndpointMarginTest(unittest.TestCase):
    @staticmethod
    def message():
        return SimpleNamespace(info=SimpleNamespace(resolution=0.10))

    def test_corner_endpoint_is_rejected_by_extra_teb_margin(self):
        data = np.zeros((21, 21), dtype=np.int16)
        # Costmap lethal cells already contain the 0.30 m inscribed footprint.
        data[10, 5] = 254
        self.assertFalse(
            _Harness()._endpoint_has_lethal_margin(data, self.message(), (10, 8))
        )

    def test_center_of_two_metre_opening_remains_admissible(self):
        data = np.zeros((21, 31), dtype=np.int16)
        data[5:16, 5] = 254
        data[5:16, 25] = 254
        self.assertTrue(
            _Harness()._endpoint_has_lethal_margin(data, self.message(), (10, 15))
        )

    def test_pure_fixture_without_clearance_keeps_legacy_behavior(self):
        harness = GlobalFrontierPlanningCostmapMixin()
        data = np.zeros((5, 5), dtype=np.int16)
        data[2, 0] = 254
        message = self.message()
        self.assertTrue(harness._endpoint_has_lethal_margin(data, message, (2, 2)))


if __name__ == "__main__":
    unittest.main()
