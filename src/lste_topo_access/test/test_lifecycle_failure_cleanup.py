#!/usr/bin/env python3
"""Regression tests for execution cleanup on lifecycle failure."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
DEVEL_PYTHON = Path(__file__).resolve().parents[3] / "devel/lib/python3/dist-packages"
if DEVEL_PYTHON.is_dir() and str(DEVEL_PYTHON) not in sys.path:
    sys.path.insert(0, str(DEVEL_PYTHON))

from lifecycle_manager import State
from goal_manager_lifecycle import GoalManagerLifecycleMixin
from global_frontier_lifecycle import GlobalFrontierLifecycleMixin
from teb_goal_bridge_lifecycle import TebGoalBridgeLifecycleMixin
from teb_turn_supervisor_lifecycle import TebTurnSupervisorLifecycleMixin


class LifecycleFailureCleanupTest(unittest.TestCase):
    def test_lifecycle_transition_event_does_not_collide_with_publish_event(self):
        probes = [
            (GoalManagerLifecycleMixin(), "publish_goal_arbitration"),
            (GlobalFrontierLifecycleMixin(), "publish_status"),
            (TebGoalBridgeLifecycleMixin(), "publish_bridge_status"),
            (TebTurnSupervisorLifecycleMixin(), "publish_status_locked"),
        ]
        for owner, publisher_name in probes:
            owner.lifecycle_manager = SimpleNamespace(current_transaction_id=9)
            owner.lifecycle_transaction_id = lambda: 9
            captured = []

            def publisher(event, **fields):
                captured.append((event, fields))

            setattr(owner, publisher_name, publisher)
            owner._on_lifecycle_transition(State.IDLE, State.DISPATCHED, None)

            self.assertEqual(captured[-1][0], "lifecycle_transition")
            self.assertIsNone(captured[-1][1]["transition_event"])

    def test_bridge_failure_cancels_active_action(self):
        bridge = TebGoalBridgeLifecycleMixin()
        bridge.lifecycle_manager = SimpleNamespace(current_transaction_id=7)
        bridge.active_action = True
        bridge.cancelled = []
        bridge.cancel_locked = lambda reason: bridge.cancelled.append(reason)
        bridge.statuses = []
        bridge.publish_bridge_status = lambda name, **fields: bridge.statuses.append(
            (name, fields)
        )

        bridge._on_lifecycle_transition(State.DISPATCHED, State.FAILED, None)

        self.assertEqual(bridge.cancelled, ["lifecycle_failed"])

    def test_turn_failure_releases_turning_action(self):
        supervisor = TebTurnSupervisorLifecycleMixin()
        supervisor.lifecycle_manager = SimpleNamespace(current_transaction_id=8)
        supervisor.state = "TURNING"
        supervisor.active_action = True
        supervisor.released = []
        supervisor._release_turn_locked = lambda reason, completed=False: (
            supervisor.released.append((reason, completed))
        )
        supervisor.statuses = []
        supervisor.publish_status_locked = lambda name, **fields: supervisor.statuses.append(
            (name, fields)
        )

        supervisor._on_lifecycle_transition(State.DISPATCHED, State.FAILED, None)

        self.assertEqual(supervisor.released, [("lifecycle_failed", False)])
        self.assertFalse(supervisor.active_action)

    def test_failed_transition_settles_portal_attempt_before_release(self):
        owner = GlobalFrontierLifecycleMixin()
        owner.lifecycle_manager = SimpleNamespace(current_transaction_id=9)
        owner.lifecycle_transaction_id = lambda: 9
        owner.active_frontier = object()
        owner.active_portal_probe_id = 7
        settled = []
        released = []
        owner.publish_status = lambda *_args, **_kwargs: None
        owner.settle_active_portal_probe = (
            lambda result, timestamp, reason: settled.append(
                (result, timestamp, reason)
            )
        )
        owner.release_active_frontier = lambda **fields: released.append(fields)

        owner._on_lifecycle_transition(State.DISPATCHED, State.FAILED, None)

        self.assertEqual(settled[0][0], "failed")
        self.assertEqual(settled[0][2], "lifecycle_failed")
        self.assertEqual(released, [{"discard_prefetch": True}])


if __name__ == "__main__":
    unittest.main()
