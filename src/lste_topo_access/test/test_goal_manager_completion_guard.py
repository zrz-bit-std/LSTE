#!/usr/bin/env python3
"""Keep target completion robust when a timer races a reset callback."""

import ast
import json
import unittest
from pathlib import Path
from types import SimpleNamespace


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "goal_manager_goal_arbitration.py"
)


def load_complete_target_task():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    class_node = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GoalManagerGoalArbitrationMixin"
    )
    method = next(
        node
        for node in class_node.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_complete_target_task"
    )
    namespace = {
        "rospy": SimpleNamespace(loginfo=lambda *_args, **_kwargs: None),
        "Bool": lambda data=False: SimpleNamespace(data=data),
    }
    exec(
        compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"),
        namespace,
    )
    return namespace["_complete_target_task"]


class TargetCompletionGuardTest(unittest.TestCase):
    def test_completion_with_reset_timestamp_none_is_safe_and_zero_hold(self):
        complete = load_complete_target_task()

        class Manager:
            _complete_target_task = complete

            def __init__(self):
                self.target_approach_transaction = None
                self.target_track_id = "yellow_cup:yellow cup:1"
                self.target_approach_track_id = self.target_track_id
                self.target_close_hits = 3
                self.target_close_since = None
                self.events = []
                self.task_done = []
                self.task_done_published = False
                self.pub_task_done = SimpleNamespace(
                    publish=lambda message: self.task_done.append(message.data)
                )

            def publish_goal_arbitration(self, event, **fields):
                self.events.append((event, fields))

        manager = Manager()
        detection = SimpleNamespace(score=0.8, w=0.1, h=0.1)
        manager._complete_target_task(10.0, detection, 0.4, "close_box")

        self.assertTrue(manager.task_done_published)
        self.assertEqual(manager.task_done, [True])
        self.assertEqual(manager.events[0][1]["hold_seconds"], 0.0)
        self.assertEqual(manager.events[1][0], "target_task_completed")


if __name__ == "__main__":
    unittest.main()
