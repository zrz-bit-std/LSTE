"""Regression tests for the target terminal ownership handshake."""

import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from goal_manager_goal_output import GoalManagerGoalOutputMixin  # noqa: E402
from goal_manager_scheduling import GoalManagerSchedulingMixin  # noqa: E402
from goal_manager_teb_callbacks import GoalManagerTebCallbacksMixin  # noqa: E402


class BridgeAckFixture(GoalManagerTebCallbacksMixin):
    def __init__(self):
        self.target_terminal_observation_pending_transaction = 7
        self.target_terminal_observation_ack_transaction = 0
        self.next_update_time = 12.0
        self.events = []

    def publish_goal_arbitration(self, event, **fields):
        self.events.append((str(event), fields))


class OutputFixture(GoalManagerGoalOutputMixin):
    def __init__(self):
        self.target_terminal_observation_pending_transaction = 7
        self.target_terminal_observation_ack_transaction = 7
        self.target_terminal_observation_intent_sent = True


class SchedulingFixture(GoalManagerSchedulingMixin):
    def __init__(self):
        self.target_terminal_observation_pending_transaction = 7
        self.target_terminal_observation_ack_transaction = 0


def message(payload):
    return SimpleNamespace(data=json.dumps(payload))


class TargetTerminalBridgeHandoffTest(unittest.TestCase):
    def test_matching_bridge_ack_releases_only_the_pending_transaction(self):
        fixture = BridgeAckFixture()

        fixture.apply_teb_bridge_status(message({
            "event": "target_terminal_observation_released",
            "transaction_id": 6,
        }))
        self.assertEqual(fixture.target_terminal_observation_ack_transaction, 0)

        fixture.apply_teb_bridge_status(message({
            "event": "target_terminal_observation_released",
            "transaction_id": 7,
            "target_epoch": 9,
            "target_track_id": "yellow_cup:yellow cup:1",
            "controller_lease": "released",
        }))

        self.assertEqual(fixture.target_terminal_observation_ack_transaction, 7)
        self.assertEqual(fixture.next_update_time, 0.0)
        self.assertEqual(
            fixture.events[-1][0],
            "target_terminal_observation_bridge_released",
        )

    def test_successor_cannot_be_planned_before_bridge_ack(self):
        fixture = SchedulingFixture()
        self.assertTrue(fixture._target_terminal_observation_bridge_pending())

        fixture.target_terminal_observation_ack_transaction = 7

        self.assertFalse(fixture._target_terminal_observation_bridge_pending())

    def test_acknowledged_boundary_is_consumed_by_the_successor(self):
        fixture = OutputFixture()

        self.assertTrue(fixture.target_terminal_observation_bridge_ready())
        self.assertTrue(fixture._consume_target_terminal_observation_handoff())
        self.assertEqual(fixture.target_terminal_observation_pending_transaction, 0)
        self.assertEqual(fixture.target_terminal_observation_ack_transaction, 0)
        self.assertFalse(fixture.target_terminal_observation_intent_sent)


if __name__ == "__main__":
    unittest.main()
