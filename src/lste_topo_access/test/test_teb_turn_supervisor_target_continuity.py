"""Target actions keep TEB ownership across a sharp-entry publish gap."""

import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from teb_turn_supervisor_control import TebTurnSupervisorControlMixin  # noqa: E402
from teb_turn_supervisor_contract import STATE_PASS_THROUGH  # noqa: E402


def twist(linear, angular):
    return SimpleNamespace(
        linear=SimpleNamespace(x=float(linear)),
        angular=SimpleNamespace(z=float(angular)),
    )


class TargetContinuityFixture(TebTurnSupervisorControlMixin):
    def __init__(self):
        self.latest_planner_command = twist(0.0, 0.0)
        self.latest_trajectory_command = twist(0.20, -0.35)
        self.latest_trajectory_command_wall = time.monotonic()
        self.trajectory_feedback_period_ema = None
        self.trajectory_feedback_timeout = 0.35
        self.trajectory_feedback_timeout_cap = 0.60
        self.trajectory_feedback_period_scale = 1.50
        self.trajectory_continuity_enabled = True
        self.state = STATE_PASS_THROUGH
        self.active_action = True
        self.active_action_priority = 2
        self.active_action_source = "target_terminal_advance"
        self.active_action_target_track_id = "yellow_cup:yellow cup:1"
        self.latest_intent_priority = 2
        self.latest_intent_source = "target_terminal_advance"
        self.latest_target_track_id = "yellow_cup:yellow cup:1"
        self.active_action_goal = object()
        self.active_action_source_goal = object()
        self.latest_intent_goal = None
        self.task_done = False
        self.navigation_hold = False
        self.trajectory_continuity_min_forward = 0.05
        self.trajectory_continuity_terminal_radius = 0.75
        self.trajectory_continuity_max_hold = 0.10
        self.trajectory_zero_started_wall = 0.0
        self.trajectory_continuity_goal_distance = None
        self.trajectory_continuity_events = 0
        self.trajectory_continuity_sharp_entry_identity = None
        self.trajectory_continuity_sharp_entry_suppressions = 0
        self.active_action_identity = "target-segment-1"
        self.events = []

    def _sharp_navfn_entry_heading_error_locked(self):
        return math.pi * 0.75

    def _goal_distance_in_pose_frame_locked(self, _goal):
        return 2.0

    def _is_continuity_eligible_action_locked(self):
        return True

    def _is_same_target_segment_continuation_locked(self):
        return True

    def _intent_matches_goal_locked(self, _goal):
        return True

    def publish_status_locked(self, event, **fields):
        self.events.append((str(event), fields))


class NavigationHoldFixture(TebTurnSupervisorControlMixin):
    def __init__(self):
        self.lock = __import__("threading").RLock()
        self.navigation_hold = True
        self.release_calls = []
        self.activate_calls = 0

    def _release_turn_locked(self, reason, completed=False):
        self.release_calls.append((str(reason), bool(completed)))

    def _activate_turn_locked(self):
        self.activate_calls += 1
        return True


class TebTurnSupervisorTargetContinuityTest(unittest.TestCase):
    def test_selected_target_command_survives_sharp_entry_gap(self):
        fixture = TargetContinuityFixture()

        command = fixture._trajectory_continuity_command_locked(time.monotonic())

        self.assertIsNotNone(command)
        self.assertAlmostEqual(command.linear.x, 0.20, places=6)
        self.assertAlmostEqual(command.angular.z, -0.35, places=6)
        self.assertEqual(fixture.trajectory_continuity_sharp_entry_suppressions, 0)

    def test_latest_hold_sample_releases_the_gate_on_the_timer_owner(self):
        fixture = NavigationHoldFixture()

        fixture.apply_navigation_hold_sample(False)

        self.assertFalse(fixture.navigation_hold)
        self.assertEqual(fixture.release_calls, [])
        self.assertEqual(fixture.activate_calls, 1)


if __name__ == "__main__":
    unittest.main()
