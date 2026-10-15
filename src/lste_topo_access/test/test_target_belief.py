"""Regression tests for task-conditioned target evidence."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_target_belief import (
    BELIEF_CONTEXT_SUPPORTED,
    BELIEF_CONTRADICTED,
    BELIEF_TARGET_CONFIRMED,
    BELIEF_TARGET_SUPPORTED,
    TargetBeliefLedger,
)


class TargetBeliefTest(unittest.TestCase):
    def test_target_evidence_survives_a_lost_detector_frame(self):
        ledger = TargetBeliefLedger("task-v1")
        first = ledger.observe_target(
            "task-v1", 7, labels=("blue mug",), track_id="track-1", now=1.0
        )
        self.assertEqual(first["state"], BELIEF_TARGET_SUPPORTED)
        # No negative/empty detector frame deletes the physical memory.
        self.assertEqual(ledger.evidence(7)["state"], BELIEF_TARGET_SUPPORTED)
        self.assertEqual(ledger.evidence(7)["target_hits"], 1)

    def test_task_epoch_is_a_hard_identity_boundary(self):
        ledger = TargetBeliefLedger("task-v1")
        ledger.observe_target("task-v1", 7, labels=("mug",), now=1.0)
        self.assertIsNone(
            ledger.observe_target("task-v2", 7, labels=("mug",), now=2.0)
        )
        ledger.reset("task-v2")
        self.assertIsNone(ledger.evidence(7))

    def test_track_and_work_item_identity_are_retained(self):
        ledger = TargetBeliefLedger("task-v1")
        ledger.observe_target(
            "task-v1", 3, labels=("yellow cup",), work_item_id=11,
            track_id="track-a", now=1.0,
        )
        ledger.bind_observation_geometry(
            "task-v1", 3, bearing_xy=(3.0, 4.0), origin_xy=(2.0, 1.0),
            work_item_id=11, track_id="track-a", confirmed=True, now=2.0,
        )
        ledger.observe_target(
            "task-v1", 3, labels=("yellow cup",), work_item_id=12,
            track_id="track-b", now=3.0,
        )
        records = ledger.snapshot()
        self.assertEqual(len(records), 2)
        confirmed = next(record for record in records if record["track_id"] == "track-a")
        self.assertEqual(confirmed["state"], BELIEF_TARGET_CONFIRMED)
        self.assertEqual(confirmed["bearing_xy"], (0.6, 0.8))
        self.assertEqual(confirmed["origin_xy"], (2.0, 1.0))
        self.assertEqual(ledger.evidence(3)["target_hits"], 2)

    def test_context_and_negative_evidence_have_discrete_states(self):
        ledger = TargetBeliefLedger("task-v1")
        context = ledger.observe_context(
            "task-v1", 4, labels=("desk", "monitor"), now=1.0
        )
        self.assertEqual(context["state"], BELIEF_CONTEXT_SUPPORTED)
        negative = ledger.observe_negative(
            "task-v1", 5, labels=("no target",), now=2.0
        )
        self.assertEqual(negative["state"], BELIEF_CONTRADICTED)

    def test_direction_matching_is_a_discrete_half_plane_relation(self):
        ledger = TargetBeliefLedger("task-v1")
        ledger.bind_observation_geometry(
            "task-v1", 8, bearing_xy=(1.0, 0.0), confirmed=True, now=1.0
        )
        self.assertTrue(ledger.direction_matches(8, (0.5, 0.5)))
        self.assertFalse(ledger.direction_matches(8, (-1.0, 0.0)))
        self.assertFalse(ledger.direction_matches(9, (1.0, 0.0)))


if __name__ == "__main__":
    unittest.main()
