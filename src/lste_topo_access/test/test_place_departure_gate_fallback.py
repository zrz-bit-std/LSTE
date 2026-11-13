"""Bounded departure gate proofs.

The place-scope deadlock (env04 runs 20261113_193744, 20261112_003850,
20261112_111316, 20261112_185655, 20261112_234957, 20261113_001541,
20261113_091621, 20261113_092649, 20261113_102343) happens when a prepared
place departure waits for a certified physical portal crossing that the route
actually being executed can never produce:

* the portal transaction is aborted at ``destination_standoff``
  (closed_before_retry), or
* the route is replaced by an ordinary frontier after the proof was armed.

The source Place then never closes, ``current_physical_place_id`` stays on a
room the robot already left, and every local Navfn path is rejected at its
first point (outside the Place mask).

These tests pin the three bounds:
1. aborting the portal transaction downgrades the pending departure;
2. selecting a non-portal route downgrades it;
3. after a bounded wait the ordinary map evidence may decide, including the
   "robot is outside the source Place mask" fact.
"""
from pathlib import Path
from types import SimpleNamespace
import sys
import time
import unittest
from unittest.mock import patch

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_observation_departure import (  # noqa: E402
    GlobalFrontierObservationDepartureMixin,
)
from global_frontier_place_lifecycle import (  # noqa: E402
    PlaceDepartureTransaction,
)


def _region(region_id=4):
    return {
        "id": region_id,
        "state": "open",
        "last_reason": "no_local_executable_frontier",
        "visits": 1,
        "completions": 1,
        "endpoint_observations": 1,
    }


def _memory():
    return SimpleNamespace(
        by_id=lambda region_id: _region(region_id),
        viewpoints_for=lambda region_id: [],
    )


def _transaction(requires=True, confirmed=False):
    transaction = PlaceDepartureTransaction()
    transaction.prepare_region(
        _memory(), 4, (3.40, 4.90), 1, False,
        basis="structural_transition",
    )
    if requires:
        transaction.require_physical_gate_crossing()
    transaction.physical_gate_crossing_confirmed = confirmed
    return transaction


class _Footprint:
    def __init__(self, covered):
        self.covered = covered

    def __getitem__(self, _cell):
        return self.covered


class _Host(GlobalFrontierObservationDepartureMixin):
    def __init__(self, transaction, mask, footprint_covered):
        self.place_departure = transaction
        self.region_memory = _memory()
        self.active_route_id = 19
        self.current_place_scope_mask = mask
        self.current_place_scope_valid = True
        self.current_place_scope_id = 4
        self.place_departure_gate_wait_timeout = 8.0
        self.events = []
        self.commits = 0
        self.publish_status = lambda name, **fields: self.events.append(
            (name, fields))
        self.observation_anchor_cells = lambda *args, **kwargs: [(1, 1)]
        self.observation_footprint_from_anchors = (
            lambda *args, **kwargs: _Footprint(footprint_covered))
        self.xy_to_grid_cell = lambda *args, **kwargs: (0, 0)

    def commit_active_place_departure(self):
        self.commits += 1
        return "committed"


def _snapshot():
    return SimpleNamespace(
        message=object(),
        map_context=SimpleNamespace(known_free=None, components=None),
        robot_map=(6.0, 5.0),
    )


def _mask(inside):
    return np.array([[inside, inside], [inside, inside]], dtype=bool)


class TransactionDowngradeTest(unittest.TestCase):
    def test_active_gate_wait_can_be_downgraded(self):
        transaction = _transaction()
        self.assertTrue(transaction.allow_map_evidence_fallback("aborted"))
        self.assertFalse(transaction.requires_physical_gate_crossing)
        self.assertEqual(transaction.map_fallback_reason, "aborted")
        self.assertTrue(transaction.active)

    def test_inactive_or_confirmed_transaction_is_not_downgraded(self):
        inactive = PlaceDepartureTransaction()
        self.assertFalse(inactive.allow_map_evidence_fallback("aborted"))
        confirmed = _transaction(confirmed=True)
        self.assertFalse(confirmed.allow_map_evidence_fallback("aborted"))
        self.assertTrue(confirmed.requires_physical_gate_crossing)

    def test_clear_resets_the_fallback_state(self):
        transaction = _transaction()
        transaction.allow_map_evidence_fallback("aborted")
        transaction.gate_wait_started_wall = 10.0
        transaction.clear()
        self.assertIsNone(transaction.gate_wait_started_wall)
        self.assertEqual(transaction.map_fallback_reason, "")
        self.assertFalse(transaction.gate_fallback_reported)


class SyncGateRequirementTest(unittest.TestCase):
    def test_portal_route_arms_the_proof(self):
        transaction = PlaceDepartureTransaction()
        transaction.prepare_region(
            _memory(), 4, (3.40, 4.90), 1, False,
            basis="structural_transition",
        )
        host = _Host(transaction, _mask(True), False)
        host.sync_departure_gate_requirement(
            SimpleNamespace(route_kind="portal_transition"))
        self.assertTrue(transaction.requires_physical_gate_crossing)

    def test_replaced_route_downgrades_the_proof(self):
        transaction = _transaction()
        host = _Host(transaction, _mask(True), False)
        host.sync_departure_gate_requirement(
            SimpleNamespace(route_kind="frontier_endpoint"))
        self.assertFalse(transaction.requires_physical_gate_crossing)
        self.assertEqual(
            transaction.map_fallback_reason, "departure_route_not_portal")

    def test_route_replacement_publishes_an_audit_event(self):
        transaction = _transaction()
        host = _Host(transaction, _mask(True), False)
        host.sync_departure_gate_requirement(
            SimpleNamespace(route_kind="frontier_endpoint"))
        downgraded = [
            fields for name, fields in host.events
            if name == "frontier_place_departure_gate_downgraded"
        ]
        self.assertEqual(len(downgraded), 1)
        fields = downgraded[0]
        self.assertEqual(fields["reason"], "departure_route_not_portal")
        self.assertEqual(fields["region_id"], 4)
        self.assertEqual(fields["route_id"], 19)
        self.assertEqual(fields["departure_basis"], "structural_transition")
        self.assertEqual(fields["source"], [3.4, 4.9])

    def test_repeated_downgrade_publishes_once(self):
        transaction = _transaction()
        host = _Host(transaction, _mask(True), False)
        self.assertTrue(host.downgrade_departure_gate_requirement("aborted"))
        self.assertFalse(host.downgrade_departure_gate_requirement("aborted"))
        names = [name for name, _fields in host.events]
        self.assertEqual(
            names.count("frontier_place_departure_gate_downgraded"), 1)

    def test_confirmed_crossing_publishes_no_downgrade_event(self):
        transaction = _transaction(confirmed=True)
        host = _Host(transaction, _mask(True), False)
        self.assertFalse(host.downgrade_departure_gate_requirement("aborted"))
        self.assertNotIn(
            "frontier_place_departure_gate_downgraded",
            [name for name, _fields in host.events])

    def test_no_pending_departure_is_a_noop(self):
        host = _Host(PlaceDepartureTransaction(), _mask(True), False)
        self.assertFalse(host.sync_departure_gate_requirement(
            SimpleNamespace(route_kind="frontier_endpoint")))


class BoundedGateWaitTest(unittest.TestCase):
    def test_young_wait_only_reports(self):
        transaction = _transaction()
        transaction.gate_wait_started_wall = 997.0
        host = _Host(transaction, _mask(False), False)
        with patch("time.monotonic", return_value=1000.0):
            result = host.commit_departed_place_after_boundary_crossing(
                _snapshot())
        self.assertIsNone(result)
        self.assertEqual(host.commits, 0)
        self.assertIn(
            "frontier_place_departure_waiting",
            [name for name, _fields in host.events])

    def test_timeout_falls_through_to_the_mask_evidence(self):
        transaction = _transaction()
        transaction.gate_wait_started_wall = 991.0
        host = _Host(transaction, _mask(False), False)
        with patch("time.monotonic", return_value=1000.0):
            result = host.commit_departed_place_after_boundary_crossing(
                _snapshot())
        self.assertEqual(result, "committed")
        self.assertEqual(host.commits, 1)
        names = [name for name, _fields in host.events]
        self.assertIn("frontier_place_departure_gate_fallback", names)
        self.assertIn("frontier_place_boundary_crossed", names)

    def test_timeout_does_not_commit_from_inside_the_room(self):
        transaction = _transaction()
        transaction.gate_wait_started_wall = 991.0
        host = _Host(transaction, _mask(True), True)
        with patch("time.monotonic", return_value=1000.0):
            result = host.commit_departed_place_after_boundary_crossing(
                _snapshot())
        self.assertIsNone(result)
        self.assertEqual(host.commits, 0)

    def test_timeout_accepts_the_observation_footprint_evidence(self):
        transaction = _transaction()
        transaction.gate_wait_started_wall = 991.0
        host = _Host(transaction, _mask(True), False)
        with patch("time.monotonic", return_value=1000.0):
            result = host.commit_departed_place_after_boundary_crossing(
                _snapshot())
        self.assertEqual(result, "committed")
        self.assertEqual(host.commits, 1)
        evidence = [
            fields.get("evidence")
            for name, fields in host.events
            if name == "frontier_place_boundary_crossed"
        ]
        self.assertEqual(evidence, [None])

    def test_active_portal_transaction_keeps_waiting(self):
        transaction = _transaction()
        transaction.gate_wait_started_wall = 991.0
        host = _Host(transaction, _mask(False), False)
        host.portal_transaction = SimpleNamespace(active=True)
        with patch("time.monotonic", return_value=1000.0):
            result = host.commit_departed_place_after_boundary_crossing(
                _snapshot())
        self.assertIsNone(result)
        self.assertEqual(host.commits, 0)
        self.assertIn(
            "frontier_place_departure_waiting",
            [name for name, _fields in host.events])

    def test_gate_standstill_is_not_closed_by_the_mask_evidence(self):
        transaction = _transaction()
        transaction.gate_wait_started_wall = 991.0
        host = _Host(transaction, _mask(False), True)
        snapshot = _snapshot()
        snapshot.robot_map = (3.60, 4.90)  # 0.2 m from the departure anchor
        with patch("time.monotonic", return_value=1000.0):
            result = host.commit_departed_place_after_boundary_crossing(
                snapshot)
        self.assertIsNone(result)
        self.assertEqual(host.commits, 0)

    def test_zero_timeout_keeps_the_old_wait(self):
        transaction = _transaction()
        transaction.gate_wait_started_wall = 1.0
        host = _Host(transaction, _mask(False), False)
        host.place_departure_gate_wait_timeout = 0.0
        with patch("time.monotonic", return_value=1000.0):
            result = host.commit_departed_place_after_boundary_crossing(
                _snapshot())
        self.assertIsNone(result)
        self.assertEqual(host.commits, 0)


if __name__ == "__main__":
    unittest.main()
