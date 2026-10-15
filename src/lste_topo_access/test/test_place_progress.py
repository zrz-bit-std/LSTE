"""Regression tests for the durable Place phase contract."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_place_progress import (
    PLACE_PHASE_BOOTSTRAP,
    PLACE_PHASE_EXIT,
    PLACE_PHASE_OBSERVE,
    PLACE_PHASE_TRANSIT,
    covered_destination_progress,
    derive_place_progress,
)


class WorkLedger:
    def __init__(self, counts):
        self.counts = counts

    def unresolved_count(self, place_id):
        return self.counts.get(int(place_id), 0)


class PlaceProgressTest(unittest.TestCase):
    def test_missing_place_requires_bootstrap(self):
        progress = derive_place_progress()
        self.assertEqual(progress.phase, PLACE_PHASE_BOOTSTRAP)
        self.assertFalse(progress.local_observation_complete)

    def test_new_place_must_be_observed_before_exit(self):
        progress = derive_place_progress(
            place_id=2,
            state="open",
            observed=False,
        )
        self.assertEqual(progress.phase, PLACE_PHASE_OBSERVE)
        self.assertEqual(
            progress.reason,
            "place_observation_required_before_crossing",
        )

    def test_unresolved_work_keeps_observed_place_in_observe_phase(self):
        progress = derive_place_progress(
            place_id=2,
            state="open",
            observed=True,
            unresolved_work_items=1,
        )
        self.assertEqual(progress.phase, PLACE_PHASE_OBSERVE)
        self.assertFalse(progress.local_observation_complete)
        self.assertTrue(progress.has_durable_progress)

    def test_only_empty_observed_place_can_exit(self):
        progress = derive_place_progress(
            place_id=2,
            state="open",
            observed=True,
        )
        self.assertEqual(progress.phase, PLACE_PHASE_EXIT)
        self.assertTrue(progress.local_observation_complete)
        self.assertFalse(progress.has_durable_progress)

    def test_dormant_place_is_transit_only(self):
        progress = derive_place_progress(
            place_id=2,
            state="dormant",
            observed=True,
        )
        self.assertEqual(progress.phase, PLACE_PHASE_TRANSIT)
        self.assertFalse(progress.local_observation_complete)

    def test_unknown_cells_do_not_create_durable_progress(self):
        region = {"id": 2, "state": "dormant", "endpoint_observations": 1}
        progress = covered_destination_progress(region)
        self.assertFalse(progress.has_durable_progress)

    def test_owned_work_item_is_durable_progress(self):
        region = {"id": 2, "state": "dormant", "endpoint_observations": 1}
        progress = covered_destination_progress(
            region,
            work_item_ledger=WorkLedger({2: 1}),
        )
        self.assertTrue(progress.has_durable_progress)
        self.assertEqual(progress.unresolved_work_items, 1)

    def test_pending_portal_evidence_does_not_reopen_local_observation(self):
        progress = derive_place_progress(
            place_id=2,
            state="open",
            observed=True,
            unresolved_portals=1,
        )
        self.assertEqual(progress.phase, PLACE_PHASE_EXIT)
        self.assertTrue(progress.local_observation_complete)
        self.assertTrue(progress.has_durable_progress)


if __name__ == "__main__":
    unittest.main()
