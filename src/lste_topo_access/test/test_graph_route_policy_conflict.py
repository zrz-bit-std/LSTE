"""A retired covered Portal must release its retained graph plan."""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_graph_route_adapter import (  # noqa: E402
    GlobalFrontierGraphRouteAdapterMixin,
)


class _Ledger:
    def __init__(self, records):
        self.records = records

    def get(self, portal_id):
        return self.records.get(int(portal_id))


class GraphPlanPolicyConflictTest(unittest.TestCase):
    def _host(self, record, destination_region, current_place=1):
        host = GlobalFrontierGraphRouteAdapterMixin()
        host.portal_hypothesis_ledger = _Ledger({1: record})
        host.region_memory = SimpleNamespace(
            by_id=lambda place_id: destination_region)
        host.current_physical_place_id = current_place
        return host

    def test_suppressed_portal_with_covered_destination_is_terminal(self):
        host = self._host(
            {
                "id": 1,
                "source_place_id": 1,
                "destination_place_id": 2,
                "exploration_suppressed": True,
                "policy_skip_reason": "negative_room_turn",
            },
            {"id": 2, "state": "dormant", "endpoint_observations": 1},
        )
        conflict = host._graph_plan_suppressed_portal_conflict(1)
        self.assertIsNotNone(conflict)
        self.assertEqual(conflict["portal_id"], 1)
        self.assertEqual(conflict["destination_place_id"], 2)
        self.assertEqual(
            conflict["reason"], "suppressed_portal_destination_covered")

    def test_open_destination_is_not_a_conflict(self):
        host = self._host(
            {
                "id": 1,
                "source_place_id": 1,
                "destination_place_id": 2,
                "exploration_suppressed": True,
            },
            {"id": 2, "state": "open", "endpoint_observations": 0},
        )
        self.assertIsNone(host._graph_plan_suppressed_portal_conflict(1))

    def test_reverse_exit_from_inside_the_room_is_not_a_conflict(self):
        host = self._host(
            {
                "id": 1,
                "source_place_id": 1,
                "destination_place_id": 2,
                "exploration_suppressed": True,
            },
            {"id": 2, "state": "dormant", "endpoint_observations": 1},
            current_place=2,
        )
        self.assertIsNone(host._graph_plan_suppressed_portal_conflict(1))

    def test_live_portal_is_not_a_conflict(self):
        host = self._host(
            {
                "id": 1,
                "source_place_id": 1,
                "destination_place_id": 2,
                "exploration_suppressed": False,
            },
            {"id": 2, "state": "dormant", "endpoint_observations": 1},
        )
        self.assertIsNone(host._graph_plan_suppressed_portal_conflict(1))


if __name__ == "__main__":
    unittest.main()
