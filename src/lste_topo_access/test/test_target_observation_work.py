"""Regression tests for the mission-level target observation obligation."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_target_observation_work import (
    WORK_COMPLETED,
    WORK_RELEASED,
    WORK_UNRESOLVED,
    TargetObservationWorkLedger,
)


class TargetObservationWorkTest(unittest.TestCase):
    def test_target_frame_creates_one_place_owned_obligation(self):
        ledger = TargetObservationWorkLedger("task-v1")
        first = ledger.observe_target(
            "task-v1", 4, labels=("yellow cup",), track_id="track-a", now=1.0,
        )
        second = ledger.observe_target(
            "task-v1", 4, labels=("yellow cup",), track_id="track-b", now=2.0,
        )
        self.assertEqual(first["state"], WORK_UNRESOLVED)
        self.assertEqual(second["target_hits"], 2)
        self.assertEqual(ledger.pending_place_ids(), (4,))
        self.assertEqual(ledger.evidence(4)["track_ids"], ["track-a", "track-b"])

    def test_detector_gap_does_not_complete_work(self):
        ledger = TargetObservationWorkLedger("task-v1")
        ledger.observe_target("task-v1", 4, labels=("cup",), now=1.0)
        self.assertTrue(ledger.has_pending(4))
        self.assertEqual(ledger.evidence(4)["state"], WORK_UNRESOLVED)

    def test_only_explicit_task_completion_settles_work(self):
        ledger = TargetObservationWorkLedger("task-v1")
        ledger.observe_target("task-v1", 4, labels=("cup",), now=1.0)
        completed = ledger.complete("task-v1", now=3.0, reason="task_done")
        self.assertEqual(len(completed), 1)
        self.assertEqual(completed[0]["state"], WORK_COMPLETED)
        self.assertFalse(ledger.has_pending(4))

    def test_explicit_track_loss_releases_only_that_target_work(self):
        ledger = TargetObservationWorkLedger("task-v1")
        ledger.observe_target(
            "task-v1", 4, labels=("cup",), track_id="track-a", now=1.0,
        )
        ledger.observe_target(
            "task-v1", 5, labels=("cup",), track_id="track-b", now=1.0,
        )
        ledger.bind_track("task-v1", 4, "track-a")
        ledger.bind_track("task-v1", 5, "track-b")
        released = ledger.release(
            "task-v1", track_id="track-a", now=2.0, reason="track_expired",
        )
        self.assertEqual([item["place_id"] for item in released], [4])
        self.assertEqual(ledger.evidence(4)["state"], WORK_RELEASED)
        self.assertTrue(ledger.has_pending(5))

    def test_late_release_from_an_older_track_cannot_clear_newer_track(self):
        ledger = TargetObservationWorkLedger("task-v1")
        ledger.observe_target(
            "task-v1", 4, labels=("cup",), track_id="track-a", now=1.0,
        )
        ledger.bind_track("task-v1", 4, "track-a")
        ledger.bind_track("task-v1", 4, "track-b")
        self.assertEqual(
            ledger.release("task-v1", track_id="track-a", now=2.0), []
        )
        self.assertTrue(ledger.has_pending(4))
        self.assertEqual(
            ledger.evidence(4)["latest_track_id"], "track-b"
        )

    def test_new_target_evidence_reopens_a_released_place_obligation(self):
        ledger = TargetObservationWorkLedger("task-v1")
        ledger.observe_target(
            "task-v1", 4, labels=("cup",), track_id="track-a", now=1.0,
        )
        ledger.bind_track("task-v1", 4, "track-a")
        ledger.release("task-v1", track_id="track-a", now=2.0)
        reopened = ledger.observe_target(
            "task-v1", 4, labels=("cup",), track_id="track-b", now=3.0,
        )
        self.assertEqual(reopened["state"], WORK_UNRESOLVED)
        self.assertTrue(ledger.has_pending(4))

    def test_task_epoch_prevents_stale_completion_or_evidence(self):
        ledger = TargetObservationWorkLedger("task-v1")
        ledger.observe_target("task-v1", 4, labels=("cup",), now=1.0)
        self.assertEqual(ledger.complete("task-v2", now=2.0), [])
        self.assertIsNone(
            ledger.observe_target("task-v2", 4, labels=("cup",), now=2.0)
        )
        ledger.reset("task-v2")
        self.assertEqual(ledger.snapshot(), [])


if __name__ == "__main__":
    unittest.main()
