"""Regression for fresh negative close evidence releasing a navigation hold."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from goal_manager_goal_arbitration import GoalManagerGoalArbitrationMixin  # noqa: E402


class Stamp:
    def __init__(self, secs, nsecs=0, seq=0):
        self.secs = int(secs)
        self.nsecs = int(nsecs)
        self.seq = int(seq)

    def to_sec(self):
        return self.secs + self.nsecs / 1e9


class CloseGateFixture(GoalManagerGoalArbitrationMixin):
    maybe_publish_task_done = GoalManagerGoalArbitrationMixin.maybe_publish_task_done

    def __init__(self):
        self.task_done_published = False
        self.target_done_require_locked = False
        self.current_state = 0
        self.target_close_completion_eligible = lambda: True
        self.latest_dets = SimpleNamespace(
            header=SimpleNamespace(stamp=Stamp(19, seq=2))
        )
        self.target_done_max_detection_age = 2.0
        self.target_detection_for_track = lambda _message: SimpleNamespace(
            score=0.8, w=0.02, h=0.02
        )
        self.target_close_score_threshold = lambda: 0.2
        self.target_detection_is_close = lambda _det: False
        self.target_close_last_stamp = (18, 0, 1)
        self.target_close_since = 10.0
        self.target_close_last_seen = 18.0
        self.target_close_hits = 1
        self.target_close_last_stamp_before = self.target_close_last_stamp
        self.target_close_wait_reported = True
        self.navigation_hold_active = True
        self.target_close_confirmation_grace = lambda: 20.0
        self.hold_events = []
        self.events = []

    def reset_target_close_confirmation(self):
        self.target_close_since = None
        self.target_close_hits = 0
        self.target_close_last_stamp = None
        self.target_close_last_seen = None
        self.target_close_wait_reported = False

    def set_navigation_hold(self, active, reason):
        self.navigation_hold_active = bool(active)
        self.hold_events.append((bool(active), str(reason)))

    def publish_goal_arbitration(self, event, **fields):
        self.events.append((event, fields))


class TargetCloseGateTest(unittest.TestCase):
    def test_fresh_negative_frame_releases_hold_immediately(self):
        fixture = CloseGateFixture()
        fixture.maybe_publish_task_done(20.0)

        self.assertFalse(fixture.navigation_hold_active)
        self.assertEqual(fixture.target_close_hits, 0)
        self.assertEqual(
            fixture.hold_events[-1],
            (False, "fresh_close_evidence_rejected"),
        )

    def test_terminal_track_continuity_bridges_box_size_dip(self):
        fixture = CloseGateFixture()
        fixture.target_completed_segments = 1
        fixture.target_follow_confirmed = True
        fixture.target_track_id = "yellow_cup:yellow cup:1"
        fixture.target_approach_track_id = fixture.target_track_id
        fixture.target_segment_terminal_ready = True
        fixture.target_terminal_close_candidate_seen = True
        fixture.target_close_since = None
        fixture.target_close_last_seen = None
        fixture.target_close_hits = 0
        fixture.target_close_last_stamp = None
        fixture.target_done_min_fresh_hits = 3
        fixture.target_done_min_hold_time = 0.3
        fixture.target_terminal_observation_eligible = lambda _det: True
        fixture.pub_task_done = SimpleNamespace(publish=lambda _msg: None)
        fixture.target_detection_is_close = lambda _det: False

        for stamp, now in (
            (Stamp(20, 0, 3), 20.0),
            (Stamp(20, 200000000, 4), 20.2),
            (Stamp(20, 400000000, 5), 20.4),
        ):
            fixture.latest_dets.header.stamp = stamp
            fixture.latest_dets.header.seq = stamp.seq
            fixture.maybe_publish_task_done(now)

        self.assertTrue(fixture.task_done_published)
        self.assertIn(
            "target_terminal_observation_started",
            [event for event, _fields in fixture.events],
        )
        self.assertIn(
            "target_terminal_observation_confirmed",
            [event for event, _fields in fixture.events],
        )


if __name__ == "__main__":
    unittest.main()
