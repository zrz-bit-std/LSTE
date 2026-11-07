"""Navfn reachability gate for context approach goals.

A context hint is a geometric direction, not a proven route.  Publishing an
unreachable hint makes Navfn return an empty plan, the controller stand
still, and the observation window replace the goal with the robot's own
pose.  These tests cover the gate and the immediate pair retirement.
"""

import ast
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

ROOT = Path(__file__).resolve().parents[3]
SOURCE = (
    ROOT / "src" / "lste_topo_access" / "scripts" / "goal_manager_legacy_goals.py"
)


def load_methods(source, class_name, method_names):
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    cls = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    methods = [
        node for node in cls.body
        if isinstance(node, ast.FunctionDef) and node.name in method_names
    ]
    holder = ast.ClassDef(
        name="Holder", bases=[], keywords=[], body=methods,
        decorator_list=[],
    )
    module = ast.Module(body=[holder], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {
        "math": math,
        "rospy": SimpleNamespace(loginfo=lambda *_a, **_k: None),
        "String": lambda data=None: ("String", data),
        "EXPLORE_SUS_C_MODE": "explore_sus_c_mode",
    }
    exec(compile(module, str(source), "exec"), namespace)
    return namespace["Holder"]


ContextRoute = load_methods(
    SOURCE,
    "GoalManagerLegacyGoalsMixin",
    ("_context_goal_route_status", "_retire_unreachable_context_pair"),
)


class _Observation:
    def __init__(self):
        self.reset_calls = 0

    def reset(self):
        self.reset_calls += 1


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Exhaustion:
    def __init__(self):
        self.calls = []

    def mark(self, key, now, cooldown=None):
        self.calls.append((key, now, cooldown))
        return float(now) + float(cooldown)


class GateOwner(ContextRoute):
    def __init__(self, result, state="approach", pose=True, goal=True):
        self.latest_pose = object() if pose else None
        self.ctx_observation = SimpleNamespace(state=state)
        self.result = result
        self.calls = []
        self.goal = object() if goal else None

    def validate_target_route(self, goal, now):
        self.calls.append((goal, now))
        return self.result


class RetireOwner(ContextRoute):
    def __init__(self, pair_key=("plant", "chair", "desk")):
        self.ctx_pair_key = pair_key
        self.ctx_last_goal = object()
        self.ctx_start_time = 5.0
        self.ctx_cooldown_until = 0.0
        self.ctx_observation_pair_key = pair_key
        self.ctx_observation = _Observation()
        self.pub_access_mode = _Publisher()
        self.effective_mode = "catch_ctx_mode"
        self.context_route_blocked_cooldown = 30.0
        self.context_follow_cooldown = 30.0
        self.ctx_pair_exhaustion = _Exhaustion()
        self.events = []

    def publish_goal_arbitration(self, event, **fields):
        self.events.append((event, fields))


class ContextRouteGateTest(unittest.TestCase):
    def test_blocked_goal_is_reported(self):
        owner = GateOwner(False)
        self.assertIs(owner._context_goal_route_status(owner.goal, 12.0), False)
        self.assertEqual(owner.calls, [(owner.goal, 12.0)])

    def test_reachable_and_unavailable_pass_through(self):
        self.assertIs(GateOwner(True)._context_goal_route_status(object(), 1.0), True)
        self.assertIsNone(GateOwner(None)._context_goal_route_status(object(), 1.0))

    def test_observation_window_is_not_validated(self):
        owner = GateOwner(False, state="observe")
        self.assertIsNone(owner._context_goal_route_status(owner.goal, 12.0))
        self.assertEqual(owner.calls, [])

    def test_missing_pose_or_goal_is_a_no_op(self):
        owner = GateOwner(False, pose=False)
        self.assertIsNone(owner._context_goal_route_status(owner.goal, 1.0))
        owner = GateOwner(False, goal=False)
        self.assertIsNone(owner._context_goal_route_status(owner.goal, 1.0))
        self.assertEqual(owner.calls, [])


class UnreachablePairRetirementTest(unittest.TestCase):
    def test_pair_is_retired_with_the_blocked_cooldown(self):
        owner = RetireOwner()
        self.assertIsNone(owner._retire_unreachable_context_pair(100.0))
        self.assertEqual(
            owner.ctx_pair_exhaustion.calls,
            [(("plant", "chair", "desk"), 100.0, 30.0)],
        )
        self.assertIsNone(owner.ctx_last_goal)
        self.assertIsNone(owner.ctx_start_time)
        self.assertEqual(owner.ctx_observation.reset_calls, 1)
        self.assertEqual(owner.effective_mode, "explore_sus_c_mode")
        self.assertEqual(
            owner.pub_access_mode.messages, [("String", "explore_sus_c_mode")]
        )
        self.assertEqual(owner.ctx_cooldown_until, 130.0)
        self.assertEqual(owner.events[0][0], "context_pair_retired")
        self.assertEqual(owner.events[0][1]["reason"], "navfn_empty_plan")

    def test_missing_pair_identity_still_resets_the_window(self):
        owner = RetireOwner(pair_key=None)
        self.assertIsNone(owner._retire_unreachable_context_pair(10.0))
        self.assertEqual(owner.ctx_pair_exhaustion.calls, [])
        self.assertEqual(owner.ctx_observation.reset_calls, 1)
        self.assertEqual(owner.ctx_cooldown_until, 40.0)


if __name__ == "__main__":
    unittest.main()
