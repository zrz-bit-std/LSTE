"""Pure tests for event-driven target Place ownership."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from goal_manager_target_observation_gate import (
    MIN_NEGATIVE_REOBSERVATIONS,
    STATE_LOSS_CERTIFIED,
    STATE_REOBSERVING,
    STATE_TRACKING,
    TargetObservationGate,
)
from goal_manager_target_approach_transaction import (
    STATE_LOST,
    TargetApproachTransaction,
)
from goal_manager_target_follow import GoalManagerTargetFollowMixin


class TargetObservationGateTest(unittest.TestCase):
    def test_detector_gap_does_not_expire_a_confirmed_track(self):
        gate = TargetObservationGate()
        self.assertTrue(gate.confirm("track-a", 10, 1.0))

        # Before a reached segment, empty messages are a detector gap only.
        for epoch in range(11, 31):
            self.assertTrue(gate.observe("track-a", epoch, False, 1000.0 + epoch))

        self.assertEqual(gate.snapshot().state, STATE_TRACKING)
        self.assertFalse(gate.loss_certified("track-a"))

    def test_post_terminal_single_or_double_empty_frame_is_not_loss(self):
        gate = TargetObservationGate()
        gate.confirm("track-a", 10, 1.0)
        self.assertTrue(gate.begin_reobserve("track-a", 10, 2.0))
        self.assertEqual(gate.snapshot().state, STATE_REOBSERVING)

        for epoch in range(11, 11 + MIN_NEGATIVE_REOBSERVATIONS - 1):
            self.assertTrue(gate.observe("track-a", epoch, False, 2000.0 + epoch))
            self.assertFalse(gate.loss_certified("track-a"))

        self.assertEqual(gate.snapshot().negative_streak, MIN_NEGATIVE_REOBSERVATIONS - 1)
        self.assertEqual(gate.snapshot().state, STATE_REOBSERVING)

    def test_target_reappearance_cancels_negative_episode(self):
        gate = TargetObservationGate()
        gate.confirm("track-a", 10, 1.0)
        gate.begin_reobserve("track-a", 10, 2.0)
        gate.observe("track-a", 11, False, 3.0)
        gate.observe("track-a", 12, False, 4.0)
        self.assertTrue(gate.observe("track-a", 13, True, 5.0))

        snapshot = gate.snapshot()
        self.assertEqual(snapshot.state, STATE_TRACKING)
        self.assertEqual(snapshot.negative_streak, 0)
        self.assertFalse(gate.loss_certified("track-a"))

    def test_same_view_cannot_certify_loss_and_new_view_completes_episode(self):
        gate = TargetObservationGate()
        gate.confirm("track-a", 10, 1.0)
        gate.begin_reobserve("track-a", 10, 2.0)
        for epoch in range(11, 11 + MIN_NEGATIVE_REOBSERVATIONS):
            gate.observe(
                "track-a", epoch, False, 2.0 + epoch,
                viewpoint_key=("same-view",),
            )
        self.assertFalse(gate.loss_certified("track-a"))
        self.assertTrue(gate.needs_viewpoint_reinspection)

        # The next route terminal continues the same negative episode rather
        # than resetting it. One empty frame from a new physical viewpoint is
        # enough to combine with the prior negative observations.
        self.assertTrue(gate.begin_reobserve("track-a", 20, 20.0))
        gate.observe(
            "track-a", 21, False, 21.0, viewpoint_key=("new-view",),
        )
        self.assertTrue(gate.loss_certified("track-a"))

    def test_loss_requires_exact_track_and_explicit_post_terminal_episode(self):
        gate = TargetObservationGate()
        gate.confirm("track-a", 20, 1.0)
        gate.begin_reobserve("track-a", 20, 2.0)
        for epoch in range(21, 21 + MIN_NEGATIVE_REOBSERVATIONS):
            gate.observe(
                "track-a",
                epoch,
                False,
                3.0 + epoch,
                viewpoint_key=("view-a" if epoch < 23 else "view-b",),
            )

        self.assertEqual(gate.snapshot().state, STATE_LOSS_CERTIFIED)
        self.assertFalse(gate.loss_certified("track-b"))
        self.assertTrue(gate.loss_certified("track-a"))
        self.assertFalse(gate.begin_reobserve("track-a", 99, 99.0))

    def test_duplicate_detector_epoch_cannot_create_extra_negative_evidence(self):
        gate = TargetObservationGate()
        gate.confirm("track-a", 1, 1.0)
        gate.begin_reobserve("track-a", 1, 2.0)
        self.assertTrue(gate.observe("track-a", 2, False, 3.0))
        self.assertFalse(gate.observe("track-a", 2, False, 4.0))
        self.assertEqual(gate.snapshot().negative_observations, 1)

    def test_only_certified_loss_requests_workitem_release(self):
        class Manager(GoalManagerTargetFollowMixin):
            def __init__(self):
                self.target_track_id = "track-a"
                self.target_follow_confirmed = True
                self.target_observation_gate = TargetObservationGate()
                self.target_approach_transaction = TargetApproachTransaction()
                self.target_approach_transaction.begin("track-a", 1.0)
                self.events = []
                self.release_args = None

            def publish_goal_arbitration(self, event, **fields):
                self.events.append((event, fields))

            def release_target_follow_to_frontier(self, *args, **kwargs):
                self.release_args = (args, kwargs)
                return None

        manager = Manager()
        gate = manager.target_observation_gate
        gate.confirm("track-a", 1, 1.0)
        gate.begin_reobserve("track-a", 1, 2.0)
        for epoch, view in ((2, "a"), (3, "a"), (4, "b")):
            gate.observe("track-a", epoch, False, 2.0 + epoch, (view,))

        manager._release_confirmed_target_after_loss(10.0)

        self.assertEqual(
            manager.target_approach_transaction.snapshot().state, STATE_LOST
        )
        self.assertEqual(manager.events[-1][0], "target_loss_certified")
        self.assertFalse(manager.release_args[1]["preserve_obligation"])
        self.assertEqual(
            manager.release_args[1]["target_release_reason"],
            "target_loss_certified",
        )

    def test_unconfirmed_reinspection_preserves_target_place_obligation(self):
        class Manager(GoalManagerTargetFollowMixin):
            def __init__(self):
                self.target_track_id = "track-a"
                self.target_observation_gate = TargetObservationGate()
                self.target_reinspection_pending = False
                self.target_execution_state = "TARGET_CANDIDATE"
                self.release_args = None

            def set_navigation_hold(self, *_args, **_kwargs):
                return None

            def release_target_follow_to_frontier(self, *args, **kwargs):
                self.release_args = (args, kwargs)
                return None

        manager = Manager()
        manager.target_observation_gate.confirm("track-a", 4, 1.0)

        manager._request_target_reinspection(2.0)

        self.assertTrue(manager.target_reinspection_pending)
        self.assertIsNotNone(manager.release_args)
        self.assertTrue(manager.release_args[1]["preserve_obligation"])


if __name__ == "__main__":
    unittest.main()
