"""Regression for releasing an unconfirmed target observation action."""

import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
SOURCE = SCRIPTS / "goal_manager_teb_callbacks.py"


def load_terminal_method():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    owner = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GoalManagerTebCallbacksMixin"
    )
    method = next(
        node
        for node in owner.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "on_teb_goal_terminal"
    )
    namespace = {
        "ast": ast,
        "copy": __import__("copy"),
        "json": __import__("json"),
        "math": __import__("math"),
        "rospy": SimpleNamespace(
            Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: 10.0)),
            loginfo=lambda *_args, **_kwargs: None,
            logwarn=lambda *_args, **_kwargs: None,
            logwarn_throttle=lambda *_args, **_kwargs: None,
        ),
        "PoseStamped": object,
        "String": object,
    }
    exec(
        compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"),
        namespace,
    )
    return namespace["on_teb_goal_terminal"]


ON_TEB_GOAL_TERMINAL = load_terminal_method()


def pose(frame, x, y):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame),
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)),
    )


class ObservationActionFixture:
    on_teb_goal_terminal = ON_TEB_GOAL_TERMINAL

    def __init__(self):
        self.controller_mode = "teb"
        self.last_goal = pose("map", 4.0, 5.0)
        self.last_goal_source = "target_reacquisition_sweep"
        self.target_follow_confirmed = False
        self.global_frontier_update_radius = 0.3
        self.target_goal_reached_radius = 0.75
        self.target_track_id = "yellow_cup:yellow cup:1"
        self.target_reacquire_goal = object()
        self.target_reacquire_started = 12.0
        self.target_last_goal = None
        self.target_segment_terminal_ready = False
        self.target_reinspection_pending = False
        self.target_execution_state = "TARGET_REACQUISITION"
        self.goal_source = "target_reacquisition_sweep"
        self.next_update_time = 100.0
        self.events = []

    def publish_goal_arbitration(self, event, **fields):
        self.events.append((event, fields))


class TargetObservationActionTerminalTest(unittest.TestCase):
    def test_reacquisition_terminal_releases_frontier_owner(self):
        fixture = ObservationActionFixture()

        fixture.on_teb_goal_terminal(pose("map", 4.0, 5.0))

        self.assertIsNone(fixture.last_goal)
        self.assertEqual(fixture.last_goal_source, "waiting_global_slam_frontier")
        self.assertEqual(fixture.goal_source, "waiting_global_slam_frontier")
        self.assertTrue(fixture.target_reinspection_pending)
        self.assertEqual(fixture.target_execution_state, "TARGET_REINSPECTION")
        self.assertEqual(fixture.next_update_time, 0.0)
        self.assertEqual(fixture.events[-1][0], "target_observation_action_terminal")
        self.assertEqual(
            fixture.events[-1][1]["next_owner"],
            "global_slam_frontier",
        )


if __name__ == "__main__":
    unittest.main()
