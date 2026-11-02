"""A local master may be one Place-mask revision behind; a stuck one may not.

The Place mask is published before the next costmap update applies it, so the
master published in between still reflects the preceding revision.  Run
``20261101_160923`` produced 31 such masters; 30 are reproduced exactly by the
preceding mask and differ by 1-8 cells out of 4100-4900 (0.02-0.2 %).

The audit now accepts that window only when the master matches the preceding
mask exactly, only within ``MASK_REVISION_LAG_S`` of the newest revision, and
only for a bounded run of consecutive samples.  A master that never applies the
mask, that falls further behind, or that repeats the lag indefinitely still
fails.
"""

from pathlib import Path
import sys
import unittest

import numpy as np
import genpy
from nav_msgs.msg import OccupancyGrid


SCRIPTS = Path(__file__).resolve().parents[3] / "scripts/tests/targeted_navigation"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from audit_place_native_costmaps import (  # noqa: E402
    MAX_MASK_REVISION_LAGS,
    MASK_REVISION_LAG_S,
    exclusion_check,
    mask_revision_lag_allows,
)


def grid(costs, stamp=10.0, origin=(0.0, 0.0), resolution=1.0):
    message = OccupancyGrid()
    values = np.asarray(costs)
    message.header.frame_id = "odom"
    message.header.stamp = genpy.Time.from_sec(stamp)
    message.info.width = values.shape[1]
    message.info.height = values.shape[0]
    message.info.resolution = resolution
    message.info.origin.position.x = origin[0]
    message.info.origin.position.y = origin[1]
    message.info.origin.orientation.w = 1.0
    message.data = values.ravel().tolist()
    return message


def mask(costs, stamp, load=1.0):
    message = grid(costs, stamp=stamp)
    message.header.frame_id = "odom"
    message.info.origin.orientation.w = 1.0
    message.info.map_load_time = genpy.Time.from_sec(load)
    return message


# Newest revision excludes the right column; the preceding one excluded the
# left column instead.
BOUNDED_RIGHT = [[0, 0], [100, 100]]
BOUNDED_LEFT = [[100, 100], [0, 0]]
ENFORCED_LEFT = [[100, 100], [0, 0]]


class RevisionLagTest(unittest.TestCase):
    def setUp(self):
        self.master = grid(ENFORCED_LEFT, stamp=10.2)
        self.newest = mask(BOUNDED_RIGHT, stamp=10.0)
        self.previous = mask(BOUNDED_LEFT, stamp=9.0, load=0.5)
        self.masks = [(9.0, self.previous), (10.0, self.newest)]

    def test_master_that_matches_only_the_preceding_revision_lags(self):
        direct = exclusion_check(self.master, self.newest)
        self.assertGreater(direct["nonlethal_cells"], 0)
        lag = mask_revision_lag_allows(
            self.master, self.masks, 1, self.master.header.stamp.to_sec()
        )
        self.assertIsNotNone(lag)
        self.assertEqual(lag["nonlethal_cells"], 0)

    def test_a_master_that_matches_neither_revision_is_rejected(self):
        payload = np.ones((2, 2), dtype=np.int8) * 50
        neither = grid(payload, stamp=10.2)
        lag = mask_revision_lag_allows(
            neither, self.masks, 1, neither.header.stamp.to_sec()
        )
        # The helper returns the preceding-mask verdict; a master that matches
        # neither revision must come back dirty so the caller still fails.
        self.assertIsNotNone(lag)
        self.assertGreater(lag["nonlethal_cells"], 0)
        self.assertGreater(exclusion_check(neither, self.newest)["nonlethal_cells"], 0)
        self.assertGreater(exclusion_check(neither, self.previous)["nonlethal_cells"], 0)

    def test_the_window_is_time_bounded(self):
        late = grid(ENFORCED_LEFT, stamp=10.2)
        self.assertIsNotNone(
            mask_revision_lag_allows(late, self.masks, 1, 10.0 + MASK_REVISION_LAG_S)
        )
        self.assertIsNone(
            mask_revision_lag_allows(
                late, self.masks, 1, 10.0 + MASK_REVISION_LAG_S + 1e-6
            )
        )

    def test_the_streak_is_bounded(self):
        self.assertIsNone(
            mask_revision_lag_allows(
                self.master, self.masks, 1, 10.2, streak=MAX_MASK_REVISION_LAGS
            )
        )
        self.assertIsNotNone(
            mask_revision_lag_allows(
                self.master, self.masks, 1, 10.2, streak=MAX_MASK_REVISION_LAGS - 1
            )
        )

    def test_without_a_preceding_revision_there_is_no_lag(self):
        self.assertIsNone(
            mask_revision_lag_allows(self.master, self.masks, 0, 10.2)
        )

    def test_strict_when_the_master_already_matches_the_newest(self):
        aligned = grid([[0, 0], [100, 100]], stamp=10.2)
        self.assertEqual(exclusion_check(aligned, self.newest)["nonlethal_cells"], 0)


class ControllerRouteCoverageTest(unittest.TestCase):
    """A controller dispatch route need not be a WorkItem dispatch route."""

    @classmethod
    def setUpClass(cls):
        cls.source = (SCRIPTS / "audit_place_native_costmaps.py").read_text(
            encoding="utf-8"
        )

    def test_controller_only_routes_do_not_keyerror(self):
        self.assertIn("route_coverage.setdefault(route,set()).add(topic)", self.source)
        self.assertNotIn("route_coverage[route].add(topic)", self.source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
