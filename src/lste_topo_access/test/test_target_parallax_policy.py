"""Regression tests for the bounded active-parallax observation action."""

from pathlib import Path
import sys
import math
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from goal_manager_target_follow import GoalManagerTargetFollowMixin  # noqa: E402


class ParallaxFixture(GoalManagerTargetFollowMixin):
    def __init__(self):
        self.target_follow_confirmed = False
        self.target_candidate_viewpoint_diverse = False
        self.target_candidate_hits = 2
        self.target_follow_confirm_hits = 2
        self.target_done_min_score = 0.40
        self.target_candidate = None
        self.latest_pose = SimpleNamespace(x=0.0, y=0.0)
        self.target_last_heading = 1.0
        self.target_parallax_goal = None
        self.target_parallax_completed = False
        self.target_parallax_active_side = ""
        self.target_parallax_attempts = 0
        self.target_parallax_failed_sides = []
        self.target_goal_reached_radius = 0.5
        self.target_track_id = "task:cup:1"
        self.target_execution_state = "TARGET_CANDIDATE"
        self.target_route_validation_last_endpoint = None
        self.events = []
        self.validation_calls = []
        self.observation_handoffs = []

    def target_minimum_viewpoint_distance(self):
        return 0.9

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
        first_call = not self.validation_calls
        self.validation_calls.append((goal.pose.position.x, goal.pose.position.y, force))
        if first_call and "left" not in self.target_parallax_failed_sides:
            return False
        self.target_route_validation_last_endpoint = goal
        return True

    def goal_robot_distance(self, goal):
        return math.hypot(
            float(goal.pose.position.x) - float(self.latest_pose.x),
            float(goal.pose.position.y) - float(self.latest_pose.y),
        )

    def publish_goal_arbitration(self, event, **fields):
        self.events.append((event, fields))

    def publish_target_terminal_observation_intent(self, reason):
        self.observation_handoffs.append(str(reason))
        return True


class TargetParallaxPolicyTest(unittest.TestCase):
    def test_scheduler_source_keeps_active_parallax_until_terminal(self):
        source = (
            Path(__file__).resolve().parents[1]
            / "scripts"
            / "goal_manager_scheduling.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            'getattr(self, "target_parallax_goal", None) is not None',
            source,
        )
        self.assertIn(
            'not getattr(self, "target_parallax_completed", False)',
            source,
        )

    def test_physical_parallax_boundary_requires_viewpoint_diversity(self):
        fixture = ParallaxFixture()
        fixture.target_parallax_goal = fixture.make_goal_pose((0.4, 0.0), 0.0)

        self.assertFalse(fixture._complete_reached_target_parallax(1.0))
        fixture.target_candidate_viewpoint_diverse = True

        self.assertTrue(fixture._complete_reached_target_parallax(1.0))
        self.assertIsNone(fixture.target_parallax_goal)
        self.assertTrue(fixture.target_parallax_completed)
        self.assertEqual(
            fixture.observation_handoffs,
            ["target_parallax_viewpoint_reached_physical"],
        )
        self.assertEqual(
            fixture.events[-1][0],
            "target_parallax_viewpoint_reached_physical",
        )

    def test_one_strong_detection_can_start_bounded_parallax(self):
        fixture = ParallaxFixture()
        fixture.target_candidate_hits = 1
        fixture.target_candidate = SimpleNamespace(
            score=0.57, w=0.033, h=0.050,
        )

        goal = fixture._target_parallax_observation_goal(1.0)

        self.assertIsNotNone(goal)
        self.assertEqual(fixture.events[-1][0], "target_parallax_viewpoint_selected")
        self.assertAlmostEqual(
            math.hypot(
                goal.pose.position.x - fixture.latest_pose.x,
                goal.pose.position.y - fixture.latest_pose.y,
            ),
            0.90,
            places=6,
        )

    def test_rejected_side_is_consumed_and_other_side_is_selected(self):
        fixture = ParallaxFixture()
        goal = fixture._target_parallax_observation_goal(1.0)

        self.assertIsNotNone(goal)
        self.assertEqual(fixture.target_parallax_attempts, 2)
        self.assertEqual(fixture.events[-1][0], "target_parallax_viewpoint_selected")
        self.assertEqual(fixture.events[-1][1]["side"], "right")
        self.assertEqual(fixture.target_execution_state, "TARGET_PARALLAX")

    def test_selected_side_is_reused_without_a_second_route_request(self):
        fixture = ParallaxFixture()
        first = fixture._target_parallax_observation_goal(1.0)
        calls = len(fixture.validation_calls)
        second = fixture._target_parallax_observation_goal(1.1)

        self.assertIs(second, fixture.target_parallax_goal)
        self.assertEqual(first, second)
        self.assertEqual(len(fixture.validation_calls), calls)

    def test_active_parallax_is_not_preempted_when_viewpoint_becomes_diverse(self):
        fixture = ParallaxFixture()
        first = fixture._target_parallax_observation_goal(1.0)
        fixture.target_candidate_viewpoint_diverse = True

        second = fixture._target_parallax_observation_goal(1.1)

        self.assertIs(second, first)
        self.assertEqual(len(fixture.validation_calls), 2)

    def test_controller_failed_side_is_not_reissued(self):
        fixture = ParallaxFixture()
        fixture.target_parallax_failed_sides = ["left"]
        fixture.target_parallax_attempts = 1
        goal = fixture._target_parallax_observation_goal(1.0)

        self.assertIsNotNone(goal)
        self.assertEqual(fixture.events[-1][1]["side"], "right")
        self.assertEqual(len(fixture.validation_calls), 1)


if __name__ == "__main__":
    unittest.main()
