#!/usr/bin/env python3
"""Covered Places discard residual WIs and follow the physical DFS return."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_place_dfs import PlaceDfsTraversal  # noqa: E402
from global_frontier_portal_arrival_commit import (  # noqa: E402
    GlobalFrontierPortalArrivalCommitMixin,
)
from global_frontier_work_items import (  # noqa: E402
    PlaceWorkItemLedger,
    WORK_RESOLVED,
)


class FakePortalLedger:
    def __init__(self):
        self.records = {
            2: {
                "id": 2,
                "source_place_id": 1,
                "exploration_suppressed": False,
            },
            4: {
                "id": 4,
                "source_place_id": 3,
                "exploration_suppressed": False,
            },
        }

    @staticmethod
    def place_is_covered(region):
        state = str(region.get("state", "")).strip().lower()
        return state in ("dormant", "ready_to_exit") or int(
            region.get("endpoint_observations", 0) or 0
        ) > 0

    def get(self, portal_id):
        return self.records.get(int(portal_id))


class FakeRegionMemory:
    def __init__(self, covered=True):
        self.covered = covered

    def by_id(self, place_id):
        return {
            "id": int(place_id),
            "state": "dormant" if self.covered else "open",
            "endpoint_observations": 1 if self.covered else 0,
        }


class FakeNode(GlobalFrontierPortalArrivalCommitMixin):
    def __init__(self, covered=True):
        self.place_work_items = PlaceWorkItemLedger(0.7)
        self.region_memory = FakeRegionMemory(covered=covered)
        self.portal_hypothesis_ledger = FakePortalLedger()
        self.place_dfs = PlaceDfsTraversal()
        self.place_dfs.bootstrap(1)
        self.place_dfs.enter(1, 3, 2)
        self.pending_graph_resume_place_id = None
        self.pending_graph_resume_portal_id = None
        self.pending_graph_resume_destination_place_id = None
        self.active_semantic_room = None
        self.current_place_scope_valid = True
        self.current_place_scope_invalidated_reason = ""
        self.current_place_scope_frontier_work_evidence = None
        self._place_partition_key = None
        self.place_entry_rehydration_pending = None
        self.events = []

    def publish_status(self, name, **kwargs):
        self.events.append((name, kwargs))

    def _promote_next_semantic_portal(self, parent_place_id, now):
        return None


class CoveredRoomExitPolicyTest(unittest.TestCase):
    def make_node(self, covered=True):
        node = FakeNode(covered=covered)
        entry_id = node.place_work_items.ensure_portal_entry(
            4, 3, (12.0, -1.2), 1, -1.0, 1.0,
        )
        exit_id = node.place_work_items.ensure_place_exit(
            3, 2, 1, (6.0, 2.0), (6.0, 3.0), now=1.0,
        )
        active_room = {
            "place_id": 3,
            "portal_id": 2,
            "node_id": 3,
            "branch_id": 5,
            "source_place_id": 1,
        }
        return node, entry_id, exit_id, active_room

    def commit(self, node, active_room):
        room_stack = [{"place_id": 1}, active_room]
        node._commit_semantic_room_return(
            active_room,
            2,
            1,
            10.0,
            node.portal_hypothesis_ledger,
            None,
            room_stack,
        )

    def test_covered_room_discards_residual_portal_work_and_pops(self):
        node, entry_id, exit_id, active_room = self.make_node(covered=True)

        self.commit(node, active_room)

        self.assertEqual(node.place_dfs.snapshot()["stack"], [1])
        self.assertEqual(node.place_dfs.nodes[3]["state"], "completed")
        self.assertEqual(node.place_work_items.unresolved_ids(3), [])
        self.assertEqual(
            node.place_work_items._items[entry_id]["state"], WORK_RESOLVED
        )
        self.assertTrue(node.place_work_items._items[entry_id]["dismissed"])
        self.assertTrue(node.place_work_items._items[exit_id]["dismissed"])
        names = [name for name, _ in node.events]
        self.assertIn("covered_room_unresolved_work_discarded", names)
        self.assertIn("semantic_room_closed", names)
        closed = [
            kwargs for name, kwargs in node.events
            if name == "semantic_room_closed"
        ][0]
        self.assertEqual(
            set(closed["closed_work_item_ids"]), {entry_id, exit_id},
        )
        portal = node.portal_hypothesis_ledger.records[4]
        self.assertTrue(portal["exploration_suppressed"])
        self.assertEqual(
            portal["policy_skip_reason"], "covered_place_exit_priority",
        )

    def test_uncovered_room_keeps_resume_path(self):
        node, entry_id, _exit_id, active_room = self.make_node(covered=False)

        self.commit(node, active_room)

        self.assertEqual(node.place_dfs.snapshot()["stack"], [1])
        self.assertEqual(node.place_dfs.nodes[3]["state"], "suspended")
        self.assertEqual(node.place_work_items.unresolved_ids(3), [entry_id])
        self.assertFalse(
            node.place_work_items._items[entry_id].get("dismissed", False)
        )
        names = [name for name, _ in node.events]
        self.assertNotIn("covered_room_unresolved_work_discarded", names)
        self.assertIn("place_dfs_suspended_incomplete", names)


if __name__ == "__main__":
    unittest.main()
