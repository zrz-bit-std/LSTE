"""Regression for the post-parallax target reinspection boundary."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import goal_manager_goal_arbitration as arbitration  # noqa: E402
from goal_manager_goal_arbitration import GoalManagerGoalArbitrationMixin  # noqa: E402


class ReacquisitionFixture(GoalManagerGoalArbitrationMixin):
    def __init__(self):
        self.target_blocked = False
        self.target_last_goal = None
        self.target_reacquire_duration = 12.0
        self.target_reacquire_max_attempts = 1
        self.target_reacquire_attempts = 0
        self.target_reacquire_distance = 1.0
        self.target_reacquire_goal = None
        self.target_reacquire_started = None
        self.target_last_seen = 10.0
        self.target_last_heading = 0.0
        self.latest_pose = SimpleNamespace(x=0.0, y=0.0)
        self.target_follow_confirmed = False
        self.target_track_id = "yellow_cup:yellow cup:1"
        self.target_route_validation_last_endpoint = None
        self.events = []

    def target_evidence_timeout(self):
        return 8.0

    def make_goal_pose(self, values, _yaw):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id="odom"),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=float(values[0]), y=float(values[1]))
            ),
        )

    def _pose_in_frame(self, pose, _frame):
        return pose

    def validate_target_route(self, goal, _now, force=False):
        self.target_route_validation_last_endpoint = goal
        self.events.append(("route_validation", bool(force)))
        return True

    def pose_distance(self, _first, _second):
        return 0.0

    def publish_goal_arbitration(self, event, **fields):
        self.events.append((str(event), fields))


class TargetReacquisitionContractTest(unittest.TestCase):
    def setUp(self):
        arbitration.rospy = SimpleNamespace(
            loginfo=lambda *_args, **_kwargs: None,
        )

    def test_force_reinspection_bypasses_freshness_once_after_parallax(self):
        fixture = ReacquisitionFixture()

        self.assertIsNone(fixture.target_reacquisition_goal(11.0))
        first = fixture.target_reacquisition_goal(11.0, force=True)
        second = fixture.target_reacquisition_goal(11.1, force=True)

        self.assertIsNotNone(first)
        self.assertIs(second, first)
        self.assertEqual(fixture.target_reacquire_attempts, 1)
        self.assertEqual(
            [event for event, _fields in fixture.events],
            ["route_validation", "target_reacquisition_route_validated"],
        )


if __name__ == "__main__":
    unittest.main()
