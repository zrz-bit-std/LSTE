import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_terminal_replan import GlobalFrontierTerminalReplanMixin


class Lifecycle:
    current_transaction_id = 99

    def enqueue_type(self, event_type, payload, transaction_id=0):
        self.last_event = (event_type, payload, transaction_id)


class Explorer(GlobalFrontierTerminalReplanMixin):
    def __init__(self):
        self.pending_controller_lease_releases = {
            10: {"route_id": 10, "transaction_id": 0},
            19: {
                "route_id": 19, "route_kind": "frontier_endpoint",
                "transaction_id": 21, "lifecycle_transaction_id": 100,
                "action_generation": 26, "graph_transaction_id": 6,
                "map_epoch": 21, "origin_epoch": 21, "projection_epoch": 21,
            },
        }
        self.pending_controller_lease_release = None
        self.awaiting_controller_lease_release_ack = True
        self.last_released_route_id = 19
        self.last_released_route_controller_pending = True
        self.last_planning_wall = 10.0
        self.last_controller_lease_release_ack = None
        self.active_route_id = 0
        self.active_frontier = None
        self.lifecycle_manager = Lifecycle()
        self.status = []

    def publish_status(self, event, **fields):
        self.status.append((event, fields))


class ControllerLeaseReleaseRetryTest(unittest.TestCase):
    def test_duplicate_terminal_does_not_reopen_an_acknowledged_lease(self):
        explorer = Explorer()
        ack = {
            "route_id": 10, "route_kind": "frontier_endpoint",
            "transaction_id": 10, "lifecycle_transaction_id": 99,
            "action_generation": 17, "graph_transaction_id": 10,
            "map_epoch": 60, "origin_epoch": 60, "projection_epoch": 1,
        }
        explorer.last_controller_lease_release_ack = dict(ack)

        self.assertTrue(explorer._controller_release_ack_matches(dict(ack)))
        self.assertFalse(explorer._controller_release_ack_matches(
            dict(ack, action_generation=18)
        ))

    def test_newer_exact_ack_retires_superseded_predecessors(self):
        explorer = Explorer()
        payload = {
            "route_id": 19,
            "released_route_id": 19,
            "route_kind": "frontier_endpoint",
            "released_route_kind": "frontier_endpoint",
            "transaction_id": 21,
            "lifecycle_transaction_id": 100,
            "action_generation": 26,
            "graph_transaction_id": 6,
            "map_epoch": 21,
            "origin_epoch": 21,
            "projection_epoch": 21,
            "controller_lease": "released",
        }

        explorer._accept_controller_lease_release_ack(payload)

        self.assertEqual(explorer.pending_controller_lease_releases, {})
        self.assertFalse(explorer.awaiting_controller_lease_release_ack)
        acknowledged = [
            fields for event, fields in explorer.status
            if event == "controller_lease_release_acknowledged"
        ]
        self.assertEqual(
            acknowledged[0]["superseded_pending_release_route_ids"], [10]
        )


if __name__ == "__main__":
    unittest.main()
