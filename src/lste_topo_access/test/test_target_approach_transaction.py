"""Regression tests for the explicit target-approach lifecycle."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from goal_manager_target_approach_transaction import (
    STATE_APPROACHING,
    STATE_COMPLETED,
    STATE_FAILED,
    STATE_LOST,
    STATE_REOBSERVING,
    TargetApproachTransaction,
)
from goal_manager_target_follow import GoalManagerTargetFollowMixin


class TargetApproachTransactionTest(unittest.TestCase):
    def test_unconfirmed_candidate_expiry_releases_room_obligation(self):
        method = GoalManagerTargetFollowMixin.expire_unconfirmed_target_candidate
        manager = type("CandidateStub", (), {})()
        manager.target_candidate_room_claim_requested = True
        manager.target_follow_confirmed = False
        manager.target_blocked = False
        manager.target_candidate_last_seen = 1.0
        manager.target_last_seen = 1.0
        manager.target_follow_candidate_timeout = 8.0
        manager.target_track_id = "task:cup:1"
        manager.events = []
        manager.cleared = False
        manager.publish_goal_arbitration = lambda event, **fields: manager.events.append(
            (event, fields)
        )
        manager.clear_target_memory = lambda: setattr(manager, "cleared", True)

        self.assertTrue(method(manager, 10.0))
        self.assertTrue(manager.cleared)
        self.assertEqual(manager.events[0][0], "target_candidate_expired")

    def test_candidate_timeout_cannot_clear_active_parallax_action(self):
        method = GoalManagerTargetFollowMixin.expire_unconfirmed_target_candidate
        manager = type("ParallaxCandidateStub", (), {})()
        manager.target_parallax_goal = object()
        manager.target_parallax_completed = False
        manager.target_candidate_room_claim_requested = True
        manager.target_follow_confirmed = False
        manager.target_blocked = False
        manager.events = []
        manager.cleared = False
        manager.publish_goal_arbitration = lambda event, **fields: manager.events.append(
            (event, fields)
        )
        manager.clear_target_memory = lambda: setattr(manager, "cleared", True)

        self.assertFalse(method(manager, 10.0))
        self.assertFalse(manager.cleared)
        self.assertEqual(manager.events, [])

    def test_candidate_timeout_cannot_clear_active_reacquisition_action(self):
        method = GoalManagerTargetFollowMixin.expire_unconfirmed_target_candidate
        manager = type("ReacquisitionCandidateStub", (), {})()
        manager.target_parallax_goal = None
        manager.target_parallax_completed = True
        manager.target_reacquire_goal = object()
        manager.target_reacquire_started = 2.0
        manager.target_candidate_room_claim_requested = True
        manager.target_follow_confirmed = False
        manager.target_blocked = False
        manager.events = []
        manager.cleared = False
        manager.publish_goal_arbitration = lambda event, **fields: manager.events.append(
            (event, fields)
        )
        manager.clear_target_memory = lambda: setattr(manager, "cleared", True)

        self.assertFalse(method(manager, 20.0))
        self.assertFalse(manager.cleared)
        self.assertEqual(manager.events, [])

    class FollowStub(GoalManagerTargetFollowMixin):
        def __init__(self):
            self.target_blocked = False
            self.target_follow_confirmed = True
            self.target_route_continuity_deferred = False
            self.target_last_goal = object()
            self.target_last_heading = None
            self.target_terminal_reobserve_pending = False
            self.target_segment_terminal_ready = True
            self.target_completed_segments = 1
            self.target_track_id = "task:cup:1"
            self.target_approach_track_id = self.target_track_id
            self.target_last_seen = 9.0
            self.follow_target_lost_timeout = 12.0
            self.target_goal_reached_radius = 0.75
            self.controller_mode = "keyboard"
            self.target_observation_epoch = 0
            self.target_terminal_reobserve_epoch = 0
            self.target_terminal_reobserve_min_epoch = 0
            self.target_terminal_reobserve_until = 0.0
            self.target_close_since = None
            self.target_close_last_seen = None
            self.latest_pose = object()
            self.target_approach_transaction = TargetApproachTransaction()
            self.target_approach_transaction.begin(self.target_track_id, 1.0)
            self.target_approach_transaction.segment_committed(
                self.target_track_id, 2.0
            )
            self.target_approach_transaction.segment_arrived(
                self.target_track_id, 3.0
            )
            self.release_calls = 0

        def goal_robot_distance(self, _goal):
            return 0.0

        def can_prepare_target_continuous_handoff(self, _distance):
            return False

        def target_close_confirmation_active(self, _now):
            return False

        def target_tracking_active(self, _now):
            return True

        def release_target_follow_to_frontier(self, _now):
            self.release_calls += 1
            return object()

    def test_confirmed_track_survives_terminal_without_fresh_frame(self):
        manager = self.FollowStub()
        result = manager.goal_from_target_follow(11.0)
        self.assertIs(result, manager.target_last_goal)
        self.assertEqual(manager.release_calls, 0)
        self.assertEqual(manager.goal_source, "target_waiting_fresh_observation")
        self.assertEqual(
            manager.target_approach_transaction.snapshot().state,
            STATE_REOBSERVING,
        )

    def test_approach_is_not_limited_by_a_cache_advance_counter(self):
        transaction = TargetApproachTransaction()
        self.assertTrue(transaction.begin("task:cup:1", 1.0))
        for index in range(1, 8):
            self.assertTrue(transaction.segment_committed("task:cup:1", index))
            self.assertTrue(transaction.segment_arrived("task:cup:1", index + 0.5))
            self.assertTrue(transaction.observe("task:cup:1", index + 0.75))
        snapshot = transaction.snapshot()
        self.assertEqual(snapshot.state, STATE_APPROACHING)
        self.assertEqual(snapshot.segment_count, 7)

    def test_terminal_requires_evidence_or_explicit_loss(self):
        transaction = TargetApproachTransaction()
        transaction.begin("track", 1.0)
        transaction.segment_committed("track", 2.0)
        transaction.segment_arrived("track", 3.0)
        self.assertEqual(transaction.snapshot().state, STATE_REOBSERVING)
        self.assertTrue(transaction.observe("track", 4.0))
        self.assertEqual(transaction.snapshot().state, STATE_APPROACHING)
        self.assertTrue(transaction.mark_lost(5.0))
        self.assertEqual(transaction.snapshot().state, STATE_LOST)

    def test_route_failure_is_distinct_from_target_loss(self):
        transaction = TargetApproachTransaction()
        transaction.begin("track", 1.0)
        self.assertTrue(transaction.mark_failed(2.0, "navfn_empty_plan"))
        self.assertEqual(transaction.snapshot().state, STATE_FAILED)
        self.assertFalse(transaction.observe("track", 3.0))

    def test_reconfirmed_preserved_track_opens_fresh_viewpoint_epoch(self):
        transaction = TargetApproachTransaction()
        self.assertTrue(transaction.begin("track", 1.0, target_epoch=4))
        old_candidate = transaction.viewpoint_ledger.ensure_candidate(0)
        old_id = old_candidate.candidate_id
        old_candidate.route_status = "observed"

        self.assertTrue(transaction.mark_lost(2.0, "target_reinspection_required"))
        self.assertTrue(transaction.begin("track", 3.0, target_epoch=4))

        self.assertEqual(transaction.target_epoch, 5)
        self.assertEqual(
            transaction.viewpoint_ledger.candidate_id(0),
            "track::epoch:5::viewpoint:0",
        )
        self.assertNotEqual(transaction.viewpoint_ledger.candidate_id(0), old_id)
        self.assertIsNone(transaction.viewpoint_ledger.get(old_id))
        self.assertTrue(transaction.viewpoint_ledger.alternatives_exhausted() is False)

    def test_completion_is_explicit_and_terminal(self):
        transaction = TargetApproachTransaction()
        transaction.begin("track", 1.0)
        self.assertTrue(transaction.mark_completed(2.0))
        self.assertEqual(transaction.snapshot().state, STATE_COMPLETED)
        self.assertFalse(transaction.segment_committed("track", 3.0))


if __name__ == "__main__":
    unittest.main()
