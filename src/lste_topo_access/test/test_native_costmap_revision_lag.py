"""A master that predates a just-published mask is one bounded revision."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "scripts/tests/targeted_navigation"))

import audit_place_native_costmaps as audit  # noqa: E402


class Time:
    def __init__(self, value):
        self.value = float(value)

    def to_sec(self):
        return self.value


def message(stamp):
    return SimpleNamespace(header=SimpleNamespace(stamp=Time(stamp), frame_id="map"))


def masks():
    return [(10.0, message(10.0), True), (11.0, message(11.0), True)]


def evaluate(grid_stamp, index, dirty_stamps=(), **kwargs):
    def fake_check(grid, mask, translation=(0., 0.), rotation=0.):
        return {"nonlethal_cells": int(mask.header.stamp.to_sec() in dirty_stamps),
                "excluded_cells": 1}
    with mock.patch.object(audit, "exclusion_check", side_effect=fake_check):
        return audit.master_or_previous_revision(
            message(grid_stamp), masks(), index, **kwargs)


class PredatesRevisionTest(unittest.TestCase):
    def test_a_master_that_already_applied_the_revision_is_direct(self):
        result, lagged = evaluate(11.5, 1)
        self.assertFalse(lagged)
        self.assertEqual(result["nonlethal_cells"], 0)

    def test_the_observed_0_96_s_lag_is_admitted_and_counted(self):
        # 20261103_124741 route 10: active mask 112.590, global master 111.626.
        result, lagged = evaluate(10.04, 1, lag_seconds=10.0)
        self.assertTrue(lagged)
        self.assertEqual(result["nonlethal_cells"], 0)

    def test_the_same_lag_within_the_real_one_second_budget(self):
        result, lagged = evaluate(11.0 - 0.96, 1, lag_seconds=1.0)
        self.assertTrue(lagged)

    def test_a_lag_beyond_the_budget_still_fails(self):
        with self.assertRaises(ValueError):
            audit.master_or_previous_revision(
                message(11.0 - 1.5), masks(), 1,
                lag_seconds=1.0)

    def test_a_master_older_than_the_previous_revision_fails(self):
        with self.assertRaises(ValueError):
            evaluate(9.0, 1, lag_seconds=10.0)

    def test_a_dirty_previous_revision_still_fails(self):
        with self.assertRaises(ValueError):
            evaluate(10.5, 1, dirty_stamps=(10.0,), lag_seconds=10.0)

    def test_the_consecutive_budget_still_applies(self):
        with self.assertRaises(ValueError):
            evaluate(10.5, 1, lag_seconds=10.0, streak=audit.MAX_MASK_REVISION_LAGS)

    def test_without_a_previous_revision_it_fails(self):
        with self.assertRaises(ValueError):
            evaluate(9.9, 0, lag_seconds=10.0)


class MaskHandoffTest(unittest.TestCase):
    stamp = 441.590
    end = 441.617
    events = [(441.613, {"event": "terminal_prepare"}),
              (441.614, {"event": "controller_lease_released"}),
              (441.617, {"event": "lease_release_ack"})]

    def allowed(self, stamp=None, mode="ordinary", bounded=False,
                events=None, count=0, end=None):
        leases = [{"route_id": 36, "action_generation": 67,
                   "start": 441.211, "end": self.end if end is None else end}]
        return audit.mask_handoff_allowed(
            self.stamp if stamp is None else stamp, mode, bounded,
            leases, self.events if events is None else events, count)

    def test_the_observed_115704_handoff_is_admitted(self):
        self.assertTrue(self.allowed())

    def test_without_the_bridge_terminal_event_it_fails(self):
        self.assertFalse(self.allowed(events=[(441.617, {"event": "lease_release_ack"})]))

    def test_a_mask_dropped_earlier_than_the_window_fails(self):
        self.assertFalse(self.allowed(stamp=441.100))

    def test_a_lease_ending_later_than_the_window_fails(self):
        self.assertFalse(self.allowed(end=442.100))

    def test_the_handoff_budget_is_bounded(self):
        self.assertFalse(self.allowed(count=audit.MAX_MASK_MODE_HANDOFFS))

    def test_a_transparent_lease_conflict_is_not_a_handoff(self):
        self.assertFalse(self.allowed(mode="transparent"))

    def test_a_bounded_mask_conflict_is_not_a_handoff(self):
        self.assertFalse(self.allowed(bounded=True))

    def test_a_terminal_event_outside_the_window_does_not_count(self):
        self.assertFalse(self.allowed(events=[(441.700, {"event": "terminal_prepare"})]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
