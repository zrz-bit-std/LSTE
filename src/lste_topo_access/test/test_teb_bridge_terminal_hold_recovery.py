#!/usr/bin/env python3
"""Regression tests for terminal-held TEB action recovery."""

import ast
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))


def load_method(filename, class_name, method_name, extra=None):
    """Load one production method without importing its ROS-heavy module."""
    path = SCRIPTS / filename
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    owner = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    method = next(
        node for node in owner.body
        if isinstance(node, ast.FunctionDef) and node.name == method_name
    )
    namespace = dict(extra or {})
    exec(
        compile(
            ast.Module(body=[method], type_ignores=[]),
            str(path),
            "exec",
        ),
        namespace,
    )
    return namespace[method_name]


ADMIT_CALLBACK = load_method(
    "teb_goal_bridge_lifecycle.py",
    "TebGoalBridgeLifecycleMixin",
    "_admit_action_callback_generation",
    {"rospy": SimpleNamespace(logwarn=lambda *_args, **_kwargs: None)},
)
TERMINAL_HOLD_STALL = load_method(
    "teb_goal_bridge_handoff_stall.py",
    "TebGoalBridgeHandoffStallMixin",
    "_terminal_hold_no_translation_locked",
    {"math": math},
)


class LifecycleFixture:
    def __init__(self):
        self.action_generation = 130
        self.transport_handoff_pending = {"old_generation": 129}
        self.events = []

    def publish_bridge_status(self, event, **fields):
        self.events.append((event, fields))


class TerminalHoldFixture:
    def __init__(self, reason="planner_contract_terminal_hold",
                 route_id=58, priority=2):
        self.teb_feedback_invalid_reason = reason
        self.active_route_id = route_id
        self.active_intent_priority = priority
        self.pose_odom = SimpleNamespace(x=1.0, y=2.0)
        self.terminal_hold_stall_seconds = 45.0
        self.terminal_hold_stall_route_id = 0
        self.terminal_hold_stall_anchor_xy = None
        self.terminal_hold_stall_since = 0.0
        self.events = []
        self.latches = []
        self.cancels = []

    def publish_bridge_status(self, event, **fields):
        self.events.append((event, fields))

    def _latch_target_failure_locked(self, reason, **fields):
        self.latches.append((reason, fields))
        return True

    def _cancel_stalled_frontier_locked(self, now, pending_delta, reason=None):
        self.cancels.append((now, pending_delta, reason))
        return True


class TerminalHoldAdmissionTest(unittest.TestCase):
    def test_predecessor_done_is_admitted_for_transport_handoff(self):
        fixture = LifecycleFixture()

        self.assertTrue(
            ADMIT_CALLBACK(fixture, "done", 129)
        )
        admitted = next(fields for event, fields in fixture.events
                        if event == "stale_action_callback_admitted")
        self.assertEqual(admitted["callback_action_generation"], 129)
        self.assertEqual(admitted["current_action_generation"], 130)

    def test_other_late_callbacks_are_still_dropped(self):
        fixture = LifecycleFixture()

        self.assertFalse(ADMIT_CALLBACK(fixture, "feedback", 129))
        self.assertFalse(ADMIT_CALLBACK(fixture, "active", 129))
        self.assertFalse(ADMIT_CALLBACK(fixture, "done", 128))
        reasons = [fields.get("reason") for event, fields in fixture.events
                   if event == "stale_action_callback_ignored"]
        self.assertEqual(reasons, ["generation_mismatch"] * 3)

    def test_current_generation_is_admitted_normally(self):
        fixture = LifecycleFixture()

        self.assertTrue(ADMIT_CALLBACK(fixture, "done", 130))


class TerminalHoldWatchdogTest(unittest.TestCase):
    def test_stationary_terminal_hold_latches_target_once(self):
        fixture = TerminalHoldFixture()

        self.assertFalse(
            TERMINAL_HOLD_STALL(fixture, 100.0, 0.4)
        )
        self.assertFalse(
            TERMINAL_HOLD_STALL(fixture, 140.0, 0.4)
        )
        self.assertTrue(
            TERMINAL_HOLD_STALL(fixture, 146.0, 0.4)
        )
        self.assertEqual(
            [reason for reason, _fields in fixture.latches],
            ["target_terminal_hold_stalled"],
        )
        self.assertEqual(fixture.events[-1][0], "terminal_hold_stall")
        self.assertTrue(
            fixture.events[-1][1]["reason"].endswith("no_translation")
        )
        self.assertFalse(
            TERMINAL_HOLD_STALL(fixture, 147.0, 0.4)
        )
        self.assertEqual(len(fixture.latches), 1)

    def test_translation_resets_the_pose_anchor(self):
        fixture = TerminalHoldFixture()

        self.assertFalse(
            TERMINAL_HOLD_STALL(fixture, 100.0, 0.4)
        )
        fixture.pose_odom = SimpleNamespace(x=1.4, y=2.0)
        self.assertFalse(
            TERMINAL_HOLD_STALL(fixture, 140.0, 0.4)
        )
        self.assertFalse(
            TERMINAL_HOLD_STALL(fixture, 170.0, 0.4)
        )
        self.assertEqual(fixture.latches, [])
        self.assertTrue(
            TERMINAL_HOLD_STALL(fixture, 186.0, 0.4)
        )

    def test_frontier_terminal_hold_cancels_the_frontier_route(self):
        fixture = TerminalHoldFixture(route_id=20, priority=0)

        TERMINAL_HOLD_STALL(fixture, 100.0, 0.1)
        self.assertTrue(
            TERMINAL_HOLD_STALL(fixture, 146.0, 0.1)
        )
        self.assertEqual(len(fixture.cancels), 1)
        self.assertEqual(
            fixture.cancels[0][2], "terminal_hold_no_translation"
        )
        self.assertEqual(fixture.latches, [])

    def test_non_terminal_hold_reason_does_not_fire(self):
        fixture = TerminalHoldFixture(reason="planner_zero_command")

        self.assertFalse(
            TERMINAL_HOLD_STALL(fixture, 100.0, 0.4)
        )
        self.assertFalse(
            TERMINAL_HOLD_STALL(fixture, 200.0, 0.4)
        )
        self.assertEqual(fixture.events, [])
        self.assertEqual(fixture.latches, [])

    def test_watchdog_is_wired_into_the_stall_policy(self):
        source = (SCRIPTS / "teb_goal_bridge_handoff_stall.py").read_text()
        self.assertIn(
            "_terminal_hold_no_translation_locked(now, pending_delta)",
            source,
        )


if __name__ == "__main__":
    unittest.main()
