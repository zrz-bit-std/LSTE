#!/usr/bin/env python3
"""A ready reverse egress registers its own source-side probe.

env04 20261114_144747: place 6 completed, the exit through portal 5 became
ready, and then the graph idled in durable_portal_probe_unavailable with
source_place_id=6 (candidate_count=0).  The only probe on that doorway
belonged to the parent Place and had been consumed by the crossing, so the
entered Place had nothing to rehydrate and the exit never materialized.

_ensure_exit_reverse_probe() registers the reverse-direction probe from the
committed Portal hypothesis (physical gate + inside point), once per Place.
"""
from pathlib import Path
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_planning_snapshot import (  # noqa: E402
    GlobalFrontierPlanningSnapshotMixin,
)
from global_frontier_portal_probe_ledger import (  # noqa: E402
    PortalProbeLedger,
)


class _Hypotheses:
    def __init__(self, record):
        self.record = record

    def get(self, portal_id):
        return self.record if int(portal_id) == 5 else None


class _Host(GlobalFrontierPlanningSnapshotMixin):
    def __init__(self, record):
        self.portal_probe_ledger = PortalProbeLedger(match_radius=0.5)
        self.portal_hypothesis_ledger = _Hypotheses(record)
        self.events = []

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


def _portal():
    return {
        "id": 5,
        "source_place_id": 4,
        "destination_place_id": 6,
        "state": "crossed",
        "physical_gate": [10.976, -6.332],
        "physical_destination": [9.326, -6.614],
        "map_gate": [10.854, -6.445],
        "map_destination": [10.050, -6.850],
    }


class ExitReverseProbeTest(unittest.TestCase):
    def test_registers_one_reverse_probe_for_the_exit(self):
        host = _Host(_portal())
        probe = host._ensure_exit_reverse_probe(
            6, {"portal_id": 5}, 100.0)

        self.assertIsInstance(probe, dict)
        self.assertEqual(probe["source_place_id"], 6)
        records = host.portal_probe_ledger.pending_for_source(6)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]["id"], probe["id"])
        gate = records[0]["physical_gate_xy"]
        self.assertAlmostEqual(gate[0], 10.976, places=3)
        self.assertAlmostEqual(gate[1], -6.332, places=3)
        # normal points from the room interior out through the gate
        normal = records[0]["normal_xy"]
        self.assertGreater(normal[0], 0.9)
        names = [name for name, _fields in host.events]
        self.assertEqual(
            names.count("portal_exit_reverse_probe_registered"), 1)

    def test_a_pending_probe_is_not_registered_twice(self):
        host = _Host(_portal())
        host._ensure_exit_reverse_probe(6, {"portal_id": 5}, 100.0)
        again = host._ensure_exit_reverse_probe(
            6, {"portal_id": 5}, 101.0)

        self.assertIsNone(again)
        self.assertEqual(
            len(host.portal_probe_ledger.pending_for_source(6)), 1)
        names = [name for name, _fields in host.events]
        self.assertEqual(
            names.count("portal_exit_reverse_probe_registered"), 1)

    def test_unknown_portal_registers_nothing(self):
        host = _Host(None)
        self.assertIsNone(
            host._ensure_exit_reverse_probe(6, {"portal_id": 5}, 100.0))
        self.assertEqual(host.events, [])


if __name__ == "__main__":
    unittest.main()
