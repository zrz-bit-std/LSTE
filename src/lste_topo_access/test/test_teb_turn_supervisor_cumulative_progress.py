import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

SCRIPTS = Path('/home/zrz/work/lste_ws/src/lste_topo_access/scripts')
sys.path.insert(0, str(SCRIPTS))
from teb_turn_supervisor_contract import STATE_TURNING
from teb_turn_supervisor_turn_lifecycle import TebTurnSupervisorTurnLifecycleMixin

class Turn(TebTurnSupervisorTurnLifecycleMixin):
    def __init__(self, target=1.0):
        self.state = STATE_TURNING
        self.turn_target_yaw = target
        self.turn_action_identity = ('route', 7)
        self.active_action_identity = ('route', 7)
        self.turn_last_odom_theta = 0.0
        self.turn_odom_progress_anchor_theta = 0.0
        self.turn_last_odom_progress_wall = 1.0
        self.turn_odom_progress_epsilon = 0.02
        self.turn_odom_sample_count = 0
        self.turn_abs_rotation = 0.0

    def sample(self, theta, now):
        self._observe_turn_odom_pose_locked(SimpleNamespace(theta=theta), now)

class CumulativeProgressTest(unittest.TestCase):
    def test_many_subthreshold_correct_direction_samples_accumulate(self):
        t = Turn(target=1.0)
        for i in range(1, 5):
            t.sample(0.004 * i, 1.0 + i * 0.02)
            self.assertEqual(t.turn_last_odom_progress_wall, 1.0)
        t.sample(0.020, 1.10)
        self.assertEqual(t.turn_last_odom_progress_wall, 1.10)
        self.assertAlmostEqual(t.turn_odom_progress_anchor_theta, 0.020)

    def test_motion_away_from_target_does_not_refresh(self):
        t = Turn(target=1.0)
        for i in range(1, 16):
            t.sample(-0.004 * i, 1.0 + i * 0.1)
        self.assertEqual(t.turn_last_odom_progress_wall, 1.0)

    def test_subthreshold_oscillating_noise_does_not_refresh(self):
        t = Turn(target=1.0)
        for i, theta in enumerate((0.003, -0.003, 0.002, -0.002, 0.001), 1):
            t.sample(theta, 1.0 + i * 0.2)
        self.assertEqual(t.turn_last_odom_progress_wall, 1.0)

    def test_stationary_turn_remains_eligible_for_existing_timeout(self):
        t = Turn(target=1.0)
        timeout = 3.2
        for now in (1.5, 2.0, 3.0, 4.21):
            t.sample(0.0, now)
        age = 4.21 - t.turn_last_odom_progress_wall
        self.assertEqual(t.turn_last_odom_progress_wall, 1.0)
        self.assertGreaterEqual(age, timeout)

    def test_wraparound_uses_shortest_yaw_error(self):
        t = Turn(target=-math.pi + 0.1)
        t.turn_last_odom_theta = math.pi - 0.1
        t.turn_odom_progress_anchor_theta = math.pi - 0.1
        t.turn_last_odom_progress_wall = 1.0
        t.sample(math.pi - 0.08, 1.1)
        self.assertEqual(t.turn_last_odom_progress_wall, 1.1)

if __name__ == '__main__':
    unittest.main()
