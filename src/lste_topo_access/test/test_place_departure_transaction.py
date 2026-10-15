#!/usr/bin/env python3
"""Unit tests for the atomic place-departure transaction."""

from pathlib import Path
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from global_frontier_place_lifecycle import (
    LocalEgressPlaceLease,
    PlaceDepartureTransaction,
)


class OpenPlaceMemory:
    def __init__(self):
        self.region = {"id": 7, "state": "open"}
        self.close_calls = []
        self.suspend_calls = []

    def _nearest(self, _x, _y, _component):
        return self.region, "component_exact"

    def by_id(self, region_id):
        return self.region if int(region_id) == int(self.region["id"]) else None

    def close(self, *args, **kwargs):
        self.close_calls.append((args, kwargs))
        self.region["state"] = "dormant"
        return self.region

    def suspend(self, region, now, reason):
        self.suspend_calls.append((region, now, reason))
        region["state"] = "suspended"
        return True


class PendingWorkItems:
    def unresolved_observation_count(self, _region_id, portal_probe_ledger=None):
        del portal_probe_ledger
        return 2


def component():
    return {
        "epoch": 3,
        "label": 2,
        "cells": 40,
        "center_x": 3.0,
        "center_y": 4.0,
        "min_x": 2.0,
        "max_x": 4.0,
        "min_y": 3.0,
        "max_y": 5.0,
    }


class PlaceDepartureTransactionTest(unittest.TestCase):
    def test_prepare_defers_source_place_closure_until_success(self):
        memory = OpenPlaceMemory()
        transaction = PlaceDepartureTransaction()

        prepared = transaction.prepare(
            memory,
            component(),
            robot_map=(1.5, 2.5),
            place_hops=1,
            target_region_claim_active=False,
        )

        self.assertIs(prepared, memory.region)
        self.assertTrue(transaction.active)
        self.assertEqual(memory.close_calls, [])

        closed, anchor, hops = transaction.commit(memory, now=42.0)

        self.assertIs(closed, memory.region)
        self.assertEqual(anchor, (1.5, 2.5))
        self.assertEqual(hops, 1)
        self.assertFalse(transaction.active)
        self.assertEqual(len(memory.close_calls), 1)
        args, kwargs = memory.close_calls[0]
        self.assertEqual(args[:4], (1.5, 2.5, 42.0, "no_local_executable_frontier"))
        self.assertEqual(kwargs["region_id"], 7)

    def test_cleared_or_rejected_transition_never_closes_a_place(self):
        memory = OpenPlaceMemory()
        transaction = PlaceDepartureTransaction()

        self.assertIsNone(
            transaction.prepare(
                memory,
                component(),
                robot_map=(1.5, 2.5),
                place_hops=1,
                target_region_claim_active=True,
            )
        )
        transaction.prepare(
            memory,
            component(),
            robot_map=(1.5, 2.5),
            place_hops=1,
            target_region_claim_active=False,
        )
        transaction.clear()  # Route failed, was superseded, or task ended.

        self.assertEqual(transaction.commit(memory, now=42.0), (None, None, None))
        self.assertEqual(memory.close_calls, [])

    def test_exact_observation_region_can_prepare_without_a_live_component(self):
        memory = OpenPlaceMemory()
        transaction = PlaceDepartureTransaction()

        prepared = transaction.prepare_region(
            memory,
            region_id=7,
            anchor_map=(3.0, 4.0),
            place_hops=1,
            target_region_claim_active=False,
        )

        self.assertIs(prepared, memory.region)
        self.assertEqual(transaction.basis, "observation_boundary")
        closed, anchor, hops = transaction.commit(memory, now=42.0)
        self.assertIs(closed, memory.region)
        self.assertEqual(anchor, (3.0, 4.0))
        self.assertEqual(hops, 1)
        self.assertEqual(transaction.last_committed_basis, "observation_boundary")

    def test_ready_to_exit_place_closes_only_after_departure_commit(self):
        memory = OpenPlaceMemory()
        memory.region["state"] = "ready_to_exit"
        transaction = PlaceDepartureTransaction()

        prepared = transaction.prepare_region(
            memory,
            region_id=7,
            anchor_map=(3.0, 4.0),
            place_hops=1,
            target_region_claim_active=False,
        )

        self.assertIs(prepared, memory.region)
        self.assertEqual(memory.close_calls, [])
        closed, _anchor, _hops = transaction.commit(memory, now=42.0)
        self.assertIs(closed, memory.region)
        self.assertEqual(closed["state"], "dormant")

    def test_novel_branch_suspends_source_without_closing_local_work(self):
        memory = OpenPlaceMemory()
        transaction = PlaceDepartureTransaction()
        transaction.prepare_region(
            memory,
            region_id=7,
            anchor_map=(3.0, 4.0),
            place_hops=1,
            target_region_claim_active=False,
        )
        self.assertTrue(transaction.suspend_local_work_on_commit())

        suspended, _anchor, _hops = transaction.commit(memory, now=42.0)

        self.assertIs(suspended, memory.region)
        self.assertEqual(suspended["state"], "suspended")
        self.assertEqual(memory.close_calls, [])
        self.assertEqual(len(memory.suspend_calls), 1)

    def test_any_departure_protects_pending_local_work(self):
        memory = OpenPlaceMemory()
        transaction = PlaceDepartureTransaction()
        transaction.prepare_region(
            memory,
            region_id=7,
            anchor_map=(3.0, 4.0),
            place_hops=1,
            target_region_claim_active=False,
            basis="structural_transition",
        )

        self.assertTrue(
            transaction.suspend_if_local_work_pending(PendingWorkItems())
        )
        suspended, _anchor, _hops = transaction.commit(memory, now=42.0)

        self.assertIs(suspended, memory.region)
        self.assertEqual(suspended["state"], "suspended")
        self.assertEqual(memory.close_calls, [])

    def test_completed_place_transit_never_closes_or_reopens_it(self):
        """A graph corridor is travel provenance, not new room work."""
        memory = OpenPlaceMemory()
        memory.region["state"] = "dormant"
        transaction = PlaceDepartureTransaction()

        prepared = transaction.prepare_region(
            memory,
            region_id=7,
            anchor_map=(3.0, 4.0),
            place_hops=1,
            target_region_claim_active=False,
            basis="covered_transit",
            allow_dormant=True,
        )

        self.assertIs(prepared, memory.region)
        closed, anchor, hops = transaction.commit(memory, now=42.0)

        self.assertIs(closed, memory.region)
        self.assertEqual(anchor, (3.0, 4.0))
        self.assertEqual(hops, 1)
        self.assertEqual(transaction.last_committed_basis, "covered_transit")
        self.assertEqual(memory.close_calls, [])

    def test_replan_cannot_overwrite_an_uncommitted_departure(self):
        memory = OpenPlaceMemory()
        transaction = PlaceDepartureTransaction()
        first = transaction.prepare(
            memory,
            component(),
            robot_map=(1.5, 2.5),
            place_hops=1,
            target_region_claim_active=False,
        )

        replacement = transaction.prepare_region(
            memory,
            region_id=7,
            anchor_map=(9.0, 9.0),
            place_hops=1,
            target_region_claim_active=False,
            basis="observation_boundary",
        )

        self.assertIs(first, memory.region)
        self.assertIsNone(replacement)
        self.assertEqual(transaction.anchor_map, (1.5, 2.5))
        self.assertEqual(transaction.basis, "structural_transition")


class LocalEgressPlaceLeaseTest(unittest.TestCase):
    def test_recovery_keeps_the_source_place_until_rebinding(self):
        lease = LocalEgressPlaceLease()

        self.assertTrue(lease.queue(7))
        self.assertTrue(lease.activate())
        self.assertTrue(lease.complete())
        self.assertTrue(lease.rebind_pending)
        self.assertEqual(lease.source_region_id, 7)

        lease.clear()
        self.assertFalse(lease.rebind_pending)
        self.assertIsNone(lease.source_region_id)

    def test_failed_recovery_returns_its_exact_source_place(self):
        lease = LocalEgressPlaceLease()
        lease.queue(9)
        lease.activate()

        self.assertEqual(lease.fail(), 9)
        self.assertEqual(lease.phase, "idle")


if __name__ == "__main__":
    unittest.main()
