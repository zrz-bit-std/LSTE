"""Regression tests for target-owned viewpoint options."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from goal_manager_target_viewpoint_attempt_ledger import (
    ROUTE_REJECTED,
    ROUTE_READY,
    VIEWPOINT_CONTROLLER_FAILED,
    TargetViewpointAttemptLedger,
)


class TargetViewpointAttemptLedgerTest(unittest.TestCase):
    def setUp(self):
        self.ledger = TargetViewpointAttemptLedger()
        self.assertTrue(self.ledger.begin("task:yellow_cup:1", target_epoch=7))

    def _ready(self, index, requested=None, validated=None):
        return self.ledger.mark_route_ready(
            index,
            requested or (float(index), 0.0),
            validated or (float(index) + 0.2, 0.0),
            route_plan=((0.0, 0.0), (float(index) + 0.2, 0.0)),
            map_epoch=3,
            now=1.0,
        )

    def test_controller_failure_consumes_one_option_not_the_target(self):
        first = self._ready(0)
        second = self._ready(1)
        third = self._ready(2)

        active = self.ledger.begin_attempt(first.candidate_id, now=2.0)
        self.assertIsNotNone(active)
        self.assertEqual(active.route_status, "active")
        failed = self.ledger.mark_controller_failed(
            active.candidate_id,
            active.attempt_id,
            "teb_no_progress",
            now=3.0,
        )
        self.assertIsNotNone(failed)
        self.assertEqual(failed.route_status, VIEWPOINT_CONTROLLER_FAILED)
        self.assertTrue(self.ledger.active)
        self.assertTrue(self.ledger.has_open_alternative(first.candidate_id))
        self.assertIsNone(
            self.ledger.begin_attempt(first.candidate_id, now=4.0),
            "a failed physical option must never be redispatched",
        )
        next_attempt = self.ledger.begin_attempt(second.candidate_id, now=5.0)
        self.assertEqual(next_attempt.candidate_id, second.candidate_id)
        self.assertNotEqual(next_attempt.attempt_id, active.attempt_id)
        self.assertFalse(self.ledger.alternatives_exhausted())
        self.assertEqual(third.route_status, ROUTE_READY)

    def test_stale_failure_cannot_close_current_attempt(self):
        first = self._ready(0)
        second = self._ready(1)
        active = self.ledger.begin_attempt(first.candidate_id, now=2.0)
        self.ledger.mark_controller_failed(
            first.candidate_id, active.attempt_id, "old_failure", now=3.0
        )
        current = self.ledger.begin_attempt(second.candidate_id, now=4.0)
        self.assertIsNone(
            self.ledger.mark_controller_failed(
                first.candidate_id,
                current.attempt_id,
                "stale_failure",
                now=5.0,
            )
        )
        self.assertEqual(current.route_status, "active")

    def test_requested_and_validated_endpoints_are_distinct(self):
        candidate = self._ready(
            0, requested=(5.25, 16.55), validated=(5.25, 16.35)
        )
        self.assertEqual(candidate.requested_goal, (5.25, 16.55))
        self.assertEqual(candidate.validated_goal, (5.25, 16.35))
        record = self.ledger.snapshot().to_dict()["candidates"][0]
        self.assertEqual(record["requested_goal"], [5.25, 16.55])
        self.assertEqual(record["validated_goal"], [5.25, 16.35])
        self.assertEqual(record["route_status"], ROUTE_READY)

    def test_same_target_reinspection_preserves_failed_option(self):
        candidate = self._ready(0)
        active = self.ledger.begin_attempt(candidate.candidate_id, now=2.0)
        self.ledger.mark_controller_failed(
            candidate.candidate_id, active.attempt_id, "blocked_corner", now=3.0
        )
        self.assertFalse(self.ledger.begin("task:yellow_cup:1", target_epoch=7))
        self.assertEqual(
            self.ledger.get(candidate.candidate_id).route_status,
            VIEWPOINT_CONTROLLER_FAILED,
        )

    def test_rejected_route_reopens_only_for_new_map_epoch(self):
        candidate = self.ledger.ensure_candidate(0, (1.0, 1.0), map_epoch=3)
        rejected = self.ledger.mark_route_rejected(
            0, map_epoch=3, now=2.0
        )
        self.assertIs(rejected, candidate)
        self.assertEqual(rejected.route_status, ROUTE_REJECTED)
        self.assertEqual(
            self.ledger.ensure_candidate(0, map_epoch=3).route_status,
            ROUTE_REJECTED,
        )
        reopened = self.ledger.ensure_candidate(0, map_epoch=4)
        self.assertEqual(reopened.route_status, "route_pending")

    def test_all_options_must_fail_before_exhaustion(self):
        first = self._ready(0)
        second = self._ready(1)
        first_attempt = self.ledger.begin_attempt(first.candidate_id, now=2.0)
        self.ledger.mark_controller_failed(
            first.candidate_id, first_attempt.attempt_id, "stall", now=3.0
        )
        self.assertFalse(self.ledger.alternatives_exhausted())
        second_attempt = self.ledger.begin_attempt(second.candidate_id, now=4.0)
        self.ledger.mark_controller_failed(
            second.candidate_id, second_attempt.attempt_id, "stall", now=5.0
        )
        self.assertTrue(self.ledger.alternatives_exhausted())


if __name__ == "__main__":
    unittest.main()
