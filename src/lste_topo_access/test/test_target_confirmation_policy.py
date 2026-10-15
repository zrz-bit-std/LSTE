"""Regression tests for the detector-to-target-track confirmation boundary."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

# The policy module imports ROS only for logging/time. Keep this test pure.
sys.modules.setdefault(
    "rospy",
    SimpleNamespace(
        loginfo=lambda *args, **kwargs: None,
        loginfo_throttle=lambda *args, **kwargs: None,
        logwarn=lambda *args, **kwargs: None,
        logwarn_throttle=lambda *args, **kwargs: None,
    ),
)

import goal_manager_detection as detection_policy  # noqa: E402

_apply_confirmation_policy = detection_policy._apply_confirmation_policy


class ConfirmationManager:
    def __init__(self):
        self.target_candidate_hits = 2
        self.target_follow_confirm_hits = 2
        self.target_follow_weak_confirm_hits = 5
        self.target_follow_weak_min_average_score = 0.24
        self.target_candidate_viewpoint_diverse = True
        self.target_candidate_viewpoint_translation = 0.3
        self.target_candidate_viewpoint_yaw_delta = 0.0
        self.target_ray_history = [
            ((0.0, 0.0), (1.0, 0.0)),
            ((0.0, 1.0), (1.0, -0.2)),
            ((0.3, 0.0), (0.98, 0.20)),
        ]
        self.target_done_min_score = 0.4
        self.target_bbox_alpha = 0.35
        self.target_heading_alpha = 0.3
        self.target_max_heading_step = 0.4
        self.target_track_id = "yellow_cup:yellow cup:1"
        self.target_follow_confirmed = False
        self.target_reinspection_pending = False
        self.target_last_seen = None
        self.target_execution_state = "TARGET_CANDIDATE"
        self.target_direct_close_hold_reported = False
        self.target_completed_segments = 0
        self.navigation_hold_active = False
        self.target_terminal_reobserve_pending = False
        self.target_observation_hold_until = 0.0
        self.target_follow_confirm_window = 20.0
        self.events = []

    def publish_goal_arbitration(self, event, **fields):
        self.events.append((event, fields))

    def set_navigation_hold(self, active, _reason):
        self.navigation_hold_active = bool(active)

    def target_detection_is_close(self, _det):
        return False


class TargetConfirmationPolicyTest(unittest.TestCase):
    def setUp(self):
        # The policy is ROS-free apart from diagnostic logging. Keep these
        # unit tests independent of a running roscore and test ordering.
        detection_policy.rospy = SimpleNamespace(
            loginfo=lambda *args, **kwargs: None,
            loginfo_throttle=lambda *args, **kwargs: None,
            logwarn=lambda *args, **kwargs: None,
            logwarn_throttle=lambda *args, **kwargs: None,
        )

    def test_weak_candidate_waits_for_redundant_evidence(self):
        manager = ConfirmationManager()
        detection = SimpleNamespace(
            score=0.31,
            w=0.025,
            h=0.024,
            cx=0.5,
            cy=0.5,
        )
        evidence = SimpleNamespace(
            det=detection,
            score=0.31,
            box_size=0.025,
            now=4.0,
            image_stamp_seconds=3.9,
            center_delta=0.1,
            candidate_average_score=0.30,
        )

        _apply_confirmation_policy(manager, evidence)

        self.assertFalse(manager.target_follow_confirmed)
        self.assertEqual(manager.events, [])

    def test_consistent_weak_candidate_confirms_after_translation_and_five_hits(self):
        manager = ConfirmationManager()
        manager.target_candidate_hits = 5
        detection = SimpleNamespace(
            score=0.31,
            w=0.025,
            h=0.024,
            cx=0.5,
            cy=0.5,
        )
        evidence = SimpleNamespace(
            det=detection,
            score=0.31,
            box_size=0.025,
            now=4.0,
            image_stamp_seconds=3.9,
            center_delta=0.1,
            candidate_average_score=0.30,
        )

        _apply_confirmation_policy(manager, evidence)

        self.assertTrue(manager.target_follow_confirmed)
        self.assertEqual(
            [event for event, _fields in manager.events],
            ["target_follow_confirmed"],
        )


if __name__ == "__main__":
    unittest.main()
