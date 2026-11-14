#!/usr/bin/env python3
"""Physical-evidence departure closes the Place and defers its work.

env04 runs 20261114_092746 and 20261114_131837 crossed a doorway (the map
proved the robot outside), but unresolved local work made the departure
transaction *suspend* the source Place.  The Place stayed current while the
robot stood outside its mask, so every local Navfn path was rejected at its
first point (4095 and 721 navfn_place_scope_path_leak lines).

A map-proven departure now requires closure: the suspend branch is skipped and
unresolved WorkItems are archived (deferred) instead of resolved, so a later
re-entry can resume them.
"""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_observation_departure import (  # noqa: E402
    GlobalFrontierObservationDepartureMixin,
)
from global_frontier_place_lifecycle import (  # noqa: E402
    PlaceDepartureTransaction,
)
from global_frontier_work_items import (  # noqa: E402
    PlaceWorkItemLedger,
)


class _Memory:
    def __init__(self, state="open"):
        self.region = {
            "id": 4,
            "state": state,
            "visits": 2,
            "completions": 1,
            "endpoint_observations": 3,
            "last_reason": "no_local_executable_frontier",
        }
        self.closed = []
        self.suspended = []

    def by_id(self, region_id):
        return self.region if int(region_id) == 4 else None

    @staticmethod
    def viewpoints_for(_region_id):
        return []

    def close(self, _x, _y, _now, reason, component=None, region_id=None):
        self.closed.append(int(region_id))
        self.region["state"] = "dormant"
        self.region["last_reason"] = reason
        return self.region

    def suspend(self, region, _now, reason):
        self.suspended.append(int(region["id"]))
        region["state"] = "suspended"
        region["last_reason"] = reason
        return region


def _prepare(transaction, memory, basis="structural_transition"):
    transaction.prepare_region(
        memory, 4, (3.40, 4.90), 1, False, basis=basis)


class TransactionPhysicalCloseTest(unittest.TestCase):
    def test_physical_evidence_closes_instead_of_suspending(self):
        memory = _Memory()
        transaction = PlaceDepartureTransaction()
        _prepare(transaction, memory)
        transaction.suspend_local_work_on_commit()
        self.assertTrue(transaction.require_close_on_physical_evidence())

        closed, _anchor, _hops = transaction.commit(memory, 10.0)

        self.assertEqual(memory.closed, [4])
        self.assertEqual(memory.suspended, [])
        self.assertEqual(closed["state"], "dormant")

    def test_portal_departure_still_suspends_with_pending_work(self):
        memory = _Memory()
        transaction = PlaceDepartureTransaction()
        _prepare(transaction, memory)
        transaction.suspend_local_work_on_commit()

        closed, _anchor, _hops = transaction.commit(memory, 10.0)

        self.assertEqual(memory.closed, [])
        self.assertEqual(memory.suspended, [4])
        self.assertEqual(closed["state"], "suspended")

    def test_transit_only_ignores_physical_close(self):
        memory = _Memory(state="dormant")
        transaction = PlaceDepartureTransaction()
        _prepare(transaction, memory, basis="covered_transit")
        self.assertFalse(transaction.require_close_on_physical_evidence())


class DeferredWorkLedgerTest(unittest.TestCase):
    def test_defer_hides_work_and_restore_brings_it_back(self):
        ledger = PlaceWorkItemLedger(merge_radius=0.3)
        item_id = ledger.dispatch(4, (4.0, 5.0), 1.0)
        self.assertEqual(ledger.unresolved_count(4), 1)

        deferred = ledger.defer_place_work(4, 2.0)

        self.assertEqual(deferred, [item_id])
        self.assertEqual(ledger.unresolved_count(4), 0)
        self.assertEqual(ledger.unresolved_ids(4), [])

        restored = ledger.restore_deferred_work(4, 3.0)

        self.assertEqual(restored, [item_id])
        self.assertEqual(ledger.unresolved_count(4), 1)
        self.assertEqual(ledger.unresolved_ids(4), [item_id])

    def test_defer_does_not_touch_portal_obligations(self):
        ledger = PlaceWorkItemLedger(merge_radius=0.3)
        entry_id = ledger.ensure_portal_entry(
            9, 4, (4.0, 5.0), 1, 0.5, 1.0)

        deferred = ledger.defer_place_work(4, 2.0)

        self.assertEqual(deferred, [])
        self.assertIsNotNone(ledger.snapshot())
        self.assertTrue(any(
            item["id"] == entry_id for item in ledger.snapshot()))


class CommitIntegrationTest(unittest.TestCase):
    class _Host(GlobalFrontierObservationDepartureMixin):
        def __init__(self, memory, ledger):
            self.region_memory = memory
            self.place_work_items = ledger
            self.portal_probe_ledger = None
            self.observation_departure_source = None
            self.place_departure = PlaceDepartureTransaction()
            _prepare(self.place_departure, memory)
            self.events = []

        def publish_status(self, event, **fields):
            self.events.append((event, fields))

    def test_commit_defers_work_and_publishes_the_audit_event(self):
        memory = _Memory()
        ledger = PlaceWorkItemLedger(merge_radius=0.3)
        work_id = ledger.dispatch(4, (4.0, 5.0), 1.0)
        host = self._Host(memory, ledger)
        host.place_departure.suspend_local_work_on_commit()
        host.place_departure.require_close_on_physical_evidence()

        closed = host.commit_active_place_departure()

        self.assertEqual(closed["state"], "dormant")
        self.assertEqual(memory.closed, [4])
        self.assertEqual(memory.suspended, [])
        self.assertEqual(ledger.unresolved_count(4), 0)
        deferred = [
            fields for name, fields in host.events
            if name == "frontier_place_work_deferred"
        ]
        self.assertEqual(len(deferred), 1)
        self.assertEqual(deferred[0]["work_item_ids"], [work_id])
        self.assertEqual(deferred[0]["region_id"], 4)

    def test_portal_commit_keeps_the_suspend_behaviour(self):
        memory = _Memory()
        ledger = PlaceWorkItemLedger(merge_radius=0.3)
        ledger.dispatch(4, (4.0, 5.0), 1.0)
        host = self._Host(memory, ledger)
        host.place_departure.suspend_local_work_on_commit()

        closed = host.commit_active_place_departure()

        self.assertEqual(closed["state"], "suspended")
        self.assertEqual(memory.closed, [])
        self.assertEqual(ledger.unresolved_count(4), 1)
        self.assertNotIn(
            "frontier_place_work_deferred",
            [name for name, _fields in host.events])


if __name__ == "__main__":
    unittest.main()
