#!/usr/bin/env python3
"""An obsolete reverse probe is dropped when its Place exit completes.

env04 20261114_173825: place 4 exit completed, but its reverse probe stayed
source_arrived and the planner repeated durable_portal_probe_unavailable with
projection_collapsed_to_source_viewpoint / viewpoint_already_attempted while
the robot idled at the doorway.
"""
from pathlib import Path
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_portal_arrival_commit import (  # noqa: E402
    GlobalFrontierPortalArrivalCommitMixin,
)
from global_frontier_portal_probe_ledger import (  # noqa: E402
    PortalProbeLedger,
)


def _ledger_with_probe():
    ledger = PortalProbeLedger(match_radius=0.5)
    record = ledger.observe(
        4,
        (5.080, 4.672),
        (0.896, 0.444),
        map_gate_xy=(5.1, 4.7),
        now=10.0,
        observation_source="exit_reverse",
    )
    return ledger, int(record["id"])


class _Host(GlobalFrontierPortalArrivalCommitMixin):
    def __init__(self, ledger):
        self.portal_probe_ledger = ledger
        self.events = []

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


class ProbeLedgerDropTest(unittest.TestCase):
    def test_drop_removes_the_pending_identity(self):
        ledger, probe_id = _ledger_with_probe()
        self.assertEqual(len(ledger.pending_for_source(4)), 1)

        dropped = ledger.drop(probe_id)

        self.assertIsNotNone(dropped)
        self.assertIsNone(ledger.get(probe_id))
        self.assertEqual(ledger.pending_for_source(4), [])


class ExitCompletionDropsProbeTest(unittest.TestCase):
    def test_completed_exit_drops_the_source_probe(self):
        ledger, probe_id = _ledger_with_probe()
        host = _Host(ledger)

        dropped = host._settle_completed_exit_probes(4, 4, 100.0)

        self.assertEqual(dropped, [probe_id])
        self.assertEqual(ledger.pending_for_source(4), [])
        events = [
            fields for name, fields in host.events
            if name == "portal_probe_dropped_on_exit"
        ]
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["probe_ids"], [probe_id])
        self.assertEqual(events[0]["place_id"], 4)

    def test_no_probe_is_a_noop(self):
        ledger = PortalProbeLedger(match_radius=0.5)
        host = _Host(ledger)
        self.assertEqual(host._settle_completed_exit_probes(4, 4, 100.0), [])
        self.assertEqual(host.events, [])


if __name__ == "__main__":
    unittest.main()
