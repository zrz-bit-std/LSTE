"""The logical target-segment boundary must retain the native action lease."""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from teb_goal_bridge_persistent_target_approach import (  # noqa: E402
    TebGoalBridgePersistentTargetApproachMixin,
)


class PersistentTargetApproachFixture(TebGoalBridgePersistentTargetApproachMixin):
    def __init__(self):
        self.persistent_execution = True
        self.task_done = False
        goal = SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0)
            )
        )
        self.latest_goal = goal
        self.latest_intent_priority = 2
        self.latest_goal_transaction_id = 9
        self.persistent_target_approach_reported_transaction = 0
        self.persistent_installed_target_goal = goal
        self.persistent_installed_target_transaction = 9
        self.latest_target_epoch = 4
        self.latest_target_track_id = "yellow_cup:yellow cup:1"
        self.persistent_target_terminal_boundary_transaction = 0
        self.terminal_count = 0
        self.active_action_contract = {"route_id": 12}
        self.status_events = []
        self.watchdog_calls = []

    def _goal_in_global_frame(self, goal):
        return goal

    def _same_goal(self, _left, _right):
        return True

    def _publish_execution_terminal_locked(self, goal):
        return goal

    def publish_bridge_status(self, event, **fields):
        self.status_events.append((event, fields))

    def _arm_route_lease_watchdog_locked(self, **fields):
        self.watchdog_calls.append(fields)


class PersistentTargetApproachTest(unittest.TestCase):
    def test_logical_target_boundary_does_not_arm_release_ack_watchdog(self):
        fixture = PersistentTargetApproachFixture()

        fixture._handle_persistent_target_approach_locked(
            9, fixture.persistent_installed_target_goal
        )

        self.assertEqual(
            fixture.persistent_target_terminal_boundary_transaction, 9
        )
        self.assertEqual(fixture.persistent_target_approach_reported_transaction, 9)
        self.assertEqual(fixture.watchdog_calls, [])
        terminal = [
            fields
            for event, fields in fixture.status_events
            if event == "persistent_target_approach_terminal"
        ]
        self.assertEqual(len(terminal), 1)


if __name__ == "__main__":
    unittest.main()
