#!/usr/bin/env python3
"""Semantic Portal entry owns exactly one active source-side obligation."""

from pathlib import Path
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_work_items import (  # noqa: E402
    ATTEMPT_SUSPENDED,
    WORK_RESOLVED,
    WORK_UNRESOLVED,
    PlaceWorkItemLedger,
)


class SemanticEntryPruningTest(unittest.TestCase):
    def setUp(self):
        self.ledger = PlaceWorkItemLedger(0.5, 0.5)
        self.frontier = self.ledger.dispatch(1, (1.0, 1.0), 0.0)
        self.entry_top = self.ledger.ensure_portal_entry(
            1, 1, (2.0, 2.0), 1, 0.9, 0.0,
        )
        self.entry_low = self.ledger.ensure_portal_entry(
            2, 1, (3.0, 3.0), 2, 0.8, 0.0,
        )

    def test_prune_resolves_frontier_but_keeps_portal_entries(self):
        pruned = self.ledger.prune_frontier_observations(1, 1.0)
        self.assertEqual(pruned, [self.frontier])
        self.assertEqual(self.ledger._items[self.frontier]["state"], WORK_RESOLVED)
        self.assertTrue(self.ledger._items[self.frontier]["dismissed"])
        self.assertEqual(
            self.ledger._items[self.entry_top]["state"], WORK_UNRESOLVED,
        )
        self.assertEqual(
            self.ledger._items[self.entry_low]["state"], WORK_UNRESOLVED,
        )

    def test_only_top_portal_entry_remains_active(self):
        deferred = self.ledger.defer_portal_entries(1, 1, 1.0)
        self.assertEqual(deferred, [self.entry_low])
        self.assertEqual(
            self.ledger._items[self.entry_top]["state"], WORK_UNRESOLVED,
        )
        self.assertEqual(
            self.ledger._items[self.entry_low]["state"], WORK_RESOLVED,
        )
        self.assertEqual(
            self.ledger._items[self.entry_low]["resolution"],
            "deferred_lower_semantic_priority",
        )

    def test_prune_suspends_a_live_attempt(self):
        item = self.ledger._items[self.frontier]
        item["active_attempt_id"] = 1
        item["attempts"] = [{
            "id": 1, "state": "active", "map_epoch": 1,
            "map_goal": [1.0, 1.0], "viewpoint_xy": [1.0, 1.0],
            "route_kind": "frontier_endpoint", "reason": "route_dispatched",
        }]
        self.ledger.prune_frontier_observations(1, 2.0)
        self.assertEqual(item["state"], WORK_RESOLVED)
        self.assertEqual(item["attempts"][0]["state"], ATTEMPT_SUSPENDED)
        self.assertIsNone(item["active_attempt_id"])


if __name__ == "__main__":
    unittest.main()


class ClosePlacePreservesSemanticEntryTest(unittest.TestCase):
    def setUp(self):
        self.ledger = PlaceWorkItemLedger(0.5, 0.5)
        self.frontier = self.ledger.dispatch(1, (1.0, 1.0), 0.0)
        self.entry = self.ledger.ensure_portal_entry(
            1, 1, (8.05, -0.05), 1, 0.95, 0.0,
        )

    def test_close_place_resolves_frontier_but_keeps_portal_entry(self):
        closed = self.ledger.close_place(1, 1.0)
        self.assertEqual(closed, [self.frontier])
        self.assertEqual(self.ledger._items[self.frontier]["state"], WORK_RESOLVED)
        self.assertEqual(self.ledger._items[self.entry]["state"], WORK_UNRESOLVED)
        self.assertIn(self.entry, self.ledger.unresolved_ids(1))

    def test_scoped_close_keeps_portal_entry(self):
        self.ledger.close_scoped_place(1, 2, None, 1.0)
        self.assertEqual(self.ledger._items[self.entry]["state"], WORK_UNRESOLVED)
        self.assertEqual(self.ledger._items[self.entry]["place_id"], 1)

    def test_resolve_portal_entry_closes_it_after_crossing(self):
        self.ledger.close_place(1, 1.0)
        self.ledger.resolve_portal_entry(1, 2.0)
        self.assertEqual(self.ledger._items[self.entry]["state"], WORK_RESOLVED)
        self.assertNotIn(self.entry, self.ledger.unresolved_ids(1))


class EndpointCoverageDoesNotCloseEntryTest(unittest.TestCase):
    def test_endpoint_coverage_keeps_portal_entry_unresolved(self):
        ledger = PlaceWorkItemLedger(0.5, 0.5)
        entry = ledger.ensure_portal_entry(2, 1, (8.05, -0.05), 1, 0.97, 0.0)
        resolved = ledger.resolve(
            entry, 1.0, "endpoint_coverage_recorded",
        )
        self.assertIsNone(resolved)
        self.assertEqual(ledger._items[entry]["state"], WORK_UNRESOLVED)
        ledger.resolve_portal_entry(2, 2.0)
        self.assertEqual(ledger._items[entry]["state"], WORK_RESOLVED)


class ResolvePortalEntryFinishesAttemptTest(unittest.TestCase):
    def test_place_commit_resolution_finishes_the_active_attempt(self):
        ledger = PlaceWorkItemLedger(0.5, 0.5)
        entry = ledger.ensure_portal_entry(
            2, 1, (8.05, -0.05), 1, 0.97, 0.0,
        )
        ledger.dispatch_existing(entry, 0.5, viewpoint_xy=(8.05, -0.05))
        self.assertIsNotNone(ledger.active_attempt_id(entry))

        ledger.resolve_portal_entry(
            2, 1.0, reason="portal_destination_place_committed",
        )

        item = ledger._items[entry]
        self.assertEqual(item["state"], WORK_RESOLVED)
        self.assertIsNone(ledger.active_attempt_id(entry))
        self.assertEqual(
            item["last_reason"], "portal_destination_place_committed",
        )
