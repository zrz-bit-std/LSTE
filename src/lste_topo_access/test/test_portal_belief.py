#!/usr/bin/env python3
"""Regression tests for persistent certified-portal identities."""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from global_frontier_portal_belief import PortalHypothesisLedger
from global_frontier_candidate_portals import GlobalFrontierCandidatePortalMixin
from global_frontier_models import PortalSource
from global_frontier_portal_selection import GlobalFrontierPortalSelectionMixin
from global_frontier_portal_probe_ledger import PortalProbeLedger


class PlaceGraphMemory:
    def __init__(self, regions):
        self.regions = {
            int(region["id"]): dict(region) for region in regions
        }

    def by_id(self, region_id):
        return self.regions.get(int(region_id))


class WorkItemStub:
    def __init__(self, counts):
        self.counts = {int(place_id): int(count) for place_id, count in counts.items()}

    def unresolved_count(self, place_id):
        return self.counts.get(int(place_id), 0)


class RehydrationSelector(GlobalFrontierCandidatePortalMixin):
    def __init__(self, ledger):
        self.portal_hypothesis_ledger = ledger
        self.last_portal_unbound_reused = 0
        self.last_portal_unbound_reprojection_skips = 0
        self.current_task_version = "task-v1"

    @staticmethod
    def transform_xy(_target, _source, x, y):
        return float(x) + 1.0, float(y) + 1.0

    @staticmethod
    def nearest_reachable_cell(_message, steps, x, y):
        return (
            int(np.floor(y)),
            int(np.floor(x)),
        ) if steps[int(np.floor(y)), int(np.floor(x))] >= 0 else None


class SelectionReuse(GlobalFrontierPortalSelectionMixin):
    def __init__(self, ledger):
        self.portal_hypothesis_ledger = ledger
        self.current_physical_place_id = 7
        self.current_task_version = "task-v1"
        self.events = []

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


class PortalBeliefTest(unittest.TestCase):
    def test_slam_updates_reuse_one_physical_portal_identity(self):
        ledger = PortalHypothesisLedger(match_radius=0.30)
        first, created = ledger.certify(
            3,
            (4.0, 2.0),
            (4.7, 2.0),
            now=1.0,
            task_version="task-a",
            map_gate_xy=(4.0, 2.0),
            map_destination_xy=(4.7, 2.0),
        )
        second, created_again = ledger.certify(
            3,
            (4.12, 2.04),
            (4.79, 2.02),
            now=2.0,
            task_version="task-a",
            map_gate_xy=(10.0, 10.0),
            map_destination_xy=(10.7, 10.0),
        )
        self.assertTrue(created)
        self.assertFalse(created_again)
        self.assertEqual(first["id"], second["id"])
        self.assertEqual(second["certification_count"], 2)
        self.assertEqual(second["map_gate"], [10.0, 10.0])

    def test_slam_projection_does_not_rewrite_physical_portal_witness(self):
        ledger = PortalHypothesisLedger(match_radius=0.30)
        first, _created = ledger.certify(
            3,
            (4.0, 2.0),
            (4.7, 2.0),
            now=1.0,
            map_gate_xy=(4.0, 2.0),
            map_destination_xy=(4.7, 2.0),
        )

        refreshed, created = ledger.certify(
            3,
            (4.12, 2.04),
            (4.79, 2.02),
            now=2.0,
            map_gate_xy=(3.1, 2.8),
            map_destination_xy=(3.1, 3.5),
        )

        self.assertFalse(created)
        self.assertEqual(refreshed["id"], first["id"])
        self.assertEqual(refreshed["physical_gate"], [4.0, 2.0])
        self.assertEqual(refreshed["physical_destination"], [4.7, 2.0])
        self.assertEqual(refreshed["map_gate"], [3.1, 2.8])
        self.assertEqual(refreshed["map_destination"], [3.1, 3.5])

    def test_opposite_direction_is_a_distinct_portal_identity(self):
        ledger = PortalHypothesisLedger(match_radius=0.30)
        forward, _created = ledger.certify(
            3, (4.0, 2.0), (4.7, 2.0), now=1.0,
        )
        reverse, reverse_created = ledger.certify(
            3, (4.04, 2.02), (3.35, 2.02), now=2.0,
        )

        self.assertTrue(reverse_created)
        self.assertNotEqual(forward["id"], reverse["id"])

    def test_source_place_keeps_separate_doors(self):
        ledger = PortalHypothesisLedger(match_radius=0.20)
        first, _ = ledger.certify(2, (1.0, 0.0), (1.7, 0.0), now=1.0)
        second, _ = ledger.certify(2, (3.0, 0.0), (3.7, 0.0), now=1.0)
        self.assertNotEqual(first["id"], second["id"])

    def test_crossed_wide_gate_matches_other_jamb_without_normal(self):
        ledger = PortalHypothesisLedger(match_radius=0.30)
        edge, _created = ledger.certify(
            1, (2.84, 1.28), (2.84, 1.59), now=1.0,
        )
        ledger.crossed(edge["id"], now=2.0)
        ledger.bind_destination(edge["id"], 2, now=2.0)

        # The second observation is about one metre along the same two-metre
        # opening and its instantaneous normal points back through the gate.
        matched = ledger.crossed_gate_for_destination(
            2, (1.83, 1.285), gate_radius=1.25,
        )

        self.assertIsNotNone(matched)
        self.assertEqual(matched["id"], edge["id"])

    def test_destination_binding_rejects_graph_self_loop(self):
        ledger = PortalHypothesisLedger()
        edge, _created = ledger.certify(
            2, (0.0, 0.0), (0.0, 0.8), now=1.0,
        )

        self.assertIsNone(ledger.bind_destination(edge["id"], 2, now=2.0))
        self.assertIsNone(edge["destination_place_id"])
        self.assertEqual(edge["self_loop_rejection_count"], 1)

    def test_crossed_portal_execution_failure_preserves_transit_identity(self):
        ledger = PortalHypothesisLedger()
        edge, _created = ledger.certify(
            1, (0.0, 0.0), (0.0, 0.8), now=1.0,
        )
        ledger.crossed(edge["id"], now=2.0)
        ledger.bind_destination(edge["id"], 2, now=2.0)

        failed = ledger.execution_failed(edge["id"], now=3.0)

        self.assertEqual(failed["state"], "crossed")
        self.assertEqual(failed["destination_place_id"], 2)
        self.assertEqual(failed["failure_count"], 0)
        self.assertEqual(failed["execution_failure_count"], 1)

    def test_bound_portal_destination_is_immutable(self):
        ledger = PortalHypothesisLedger()
        edge, _created = ledger.certify(
            1, (0.0, 0.0), (0.0, 0.8), now=1.0,
        )
        ledger.crossed(edge["id"], now=2.0)
        self.assertIsNotNone(ledger.bind_destination(edge["id"], 2, now=2.0))

        self.assertIsNone(ledger.bind_destination(edge["id"], 3, now=3.0))
        self.assertEqual(edge["destination_place_id"], 2)

    def test_failed_portal_cannot_be_resurrected_by_crossing(self):
        ledger = PortalHypothesisLedger()
        edge, _created = ledger.certify(
            1, (0.0, 0.0), (0.0, 0.8), now=1.0,
        )
        ledger.failed(edge["id"], now=2.0)

        self.assertIsNone(ledger.crossed(edge["id"], now=3.0))
        self.assertEqual(edge["state"], "failed")

    def test_selector_rejects_same_wide_gate_from_crossed_destination(self):
        ledger = PortalHypothesisLedger(match_radius=0.30)
        edge, _created = ledger.certify(
            1, (2.84, 1.28), (2.84, 1.59), now=1.0,
        )
        ledger.crossed(edge["id"], now=2.0)
        ledger.bind_destination(edge["id"], 2, now=2.0)
        selector = SelectionReuse(ledger)
        selector.current_physical_place_id = 2
        selector.region_memory = SimpleNamespace(
            portal_entry_match_radius=1.25,
        )
        request = SimpleNamespace(
            message=SimpleNamespace(header=SimpleNamespace(frame_id="map")),
            map_to_physical_xy=lambda x, y: (x, y),
            now=3.0,
        )
        source = PortalSource(
            frontier_row=1,
            frontier_col=2,
            identity_row=1,
            identity_col=2,
            gate_row=1,
            gate_col=1,
            endpoint_row=1,
            endpoint_col=2,
        )

        self.assertIsNone(
            selector._refresh_or_certify_portal_hypothesis(
                request, source, (1.83, 1.285), (1.83, 0.975),
            )
        )
        self.assertEqual(len(ledger.snapshot()), 1)
        self.assertEqual(
            [event for event, _fields in selector.events],
            ["portal_hypothesis_suppressed"],
        )

    def test_lifecycle_is_explicit_and_task_versions_are_audit_data(self):
        ledger = PortalHypothesisLedger()
        record, _ = ledger.certify(
            1, (0.0, 0.0), (0.8, 0.0), now=1.0, task_version="v1"
        )
        ledger.select(record["id"], now=2.0)
        ledger.crossed(record["id"], now=3.0)
        snapshot = ledger.snapshot()[0]
        self.assertEqual(snapshot["state"], "crossed")
        self.assertEqual(snapshot["selection_count"], 1)
        self.assertEqual(snapshot["crossing_count"], 1)
        self.assertEqual(snapshot["task_versions"], ["v1"])

    def test_failed_portal_remains_known_without_being_deleted(self):
        ledger = PortalHypothesisLedger()
        record, _ = ledger.certify(1, (0.0, 0.0), (0.8, 0.0), now=1.0)
        ledger.failed(record["id"], now=2.0)
        self.assertEqual(ledger.get(record["id"])["state"], "failed")
        self.assertEqual(len(ledger.snapshot()), 1)

    def test_repeated_snapshot_does_not_reopen_failed_physical_edge(self):
        ledger = PortalHypothesisLedger()
        record, _created = ledger.certify(
            1, (0.0, 0.0), (0.8, 0.0), now=1.0,
        )
        ledger.failed(record["id"], now=2.0)
        refreshed, created = ledger.certify(
            1, (0.0, 0.0), (0.8, 0.0), now=3.0,
        )
        self.assertFalse(created)
        self.assertEqual(refreshed["id"], record["id"])
        self.assertEqual(refreshed["state"], "failed")
        self.assertEqual(ledger.unbound_for_source(1), [])

    def test_failed_physical_edge_cannot_be_selected_again(self):
        ledger = PortalHypothesisLedger()
        record, _created = ledger.certify(
            1, (0.0, 0.0), (0.8, 0.0), now=1.0,
        )
        ledger.failed(record["id"], now=2.0)

        self.assertIsNone(ledger.select(record["id"], now=3.0))
        self.assertEqual(ledger.get(record["id"])["state"], "failed")

    def test_failed_edge_is_not_recertified_from_a_new_snapshot(self):
        ledger = PortalHypothesisLedger()
        record, _created = ledger.certify(
            7, (1.0, 1.0), (2.0, 1.0), now=1.0,
        )
        ledger.failed(record["id"], now=2.0)

        selector = SelectionReuse(ledger)
        request = SimpleNamespace(
            message=SimpleNamespace(header=SimpleNamespace(frame_id="map")),
            map_to_physical_xy=lambda x, y: (x, y),
            now=3.0,
        )
        source = PortalSource(
            frontier_row=1,
            frontier_col=2,
            identity_row=1,
            identity_col=2,
            gate_row=1,
            gate_col=1,
            endpoint_row=1,
            endpoint_col=2,
        )

        self.assertIsNone(
            selector._refresh_or_certify_portal_hypothesis(
                request, source, (1.0, 1.0), (2.0, 1.0),
            )
        )
        self.assertEqual(len(ledger.snapshot()), 1)
        self.assertEqual(ledger.get(record["id"])["state"], "failed")

    def test_unbound_rejection_is_scoped_to_the_current_map_epoch(self):
        ledger = PortalHypothesisLedger()
        record, _created = ledger.certify(
            1, (2.0, 0.0), (3.0, 0.0), now=1.0,
        )
        ledger.mark_unbound_rejected(record["id"], 7, "navfn_unreachable")
        self.assertEqual(ledger.unbound_for_source(1, map_epoch=7), [])
        self.assertEqual(
            [item["id"] for item in ledger.unbound_for_source(1, map_epoch=8)],
            [record["id"]],
        )

    def test_covered_to_covered_edge_without_graph_progress_is_rejected(self):
        ledger = PortalHypothesisLedger()
        edge, _created = ledger.certify(
            1, (0.0, 0.0), (0.8, 0.0), now=1.0,
        )
        ledger.bind_destination(edge["id"], 2, now=2.0)
        memory = PlaceGraphMemory([
            {"id": 1, "state": "dormant", "endpoint_observations": 1},
            {"id": 2, "state": "dormant", "endpoint_observations": 1},
        ])

        self.assertFalse(
            ledger.allows_covered_transit(
                memory.by_id(1),
                memory.by_id(2),
                place_memory=memory,
            )
        )

    def test_live_unknown_boundary_allows_covered_transit_before_next_portal_is_bound(self):
        ledger = PortalHypothesisLedger()
        memory = PlaceGraphMemory([
            {"id": 1, "state": "dormant", "endpoint_observations": 1},
            {"id": 2, "state": "dormant", "endpoint_observations": 1},
        ])
        self.assertTrue(
            ledger.allows_covered_transit(
                memory.by_id(1),
                memory.by_id(2),
                place_memory=memory,
                snapshot_progress=True,
            )
        )

    def test_covered_transit_is_allowed_when_graph_reaches_unobserved_place(self):
        ledger = PortalHypothesisLedger()
        edge, _created = ledger.certify(
            1, (0.0, 0.0), (0.8, 0.0), now=1.0,
        )
        ledger.bind_destination(edge["id"], 2, now=2.0)
        branch, _created = ledger.certify(
            2, (1.0, 0.0), (1.8, 0.0), now=3.0,
        )
        ledger.bind_destination(branch["id"], 3, now=4.0)
        ledger.crossed(branch["id"], now=5.0)
        memory = PlaceGraphMemory([
            {"id": 1, "state": "dormant", "endpoint_observations": 1},
            {"id": 2, "state": "dormant", "endpoint_observations": 1},
            {"id": 3, "state": "open", "endpoint_observations": 0},
        ])

        self.assertTrue(
            ledger.allows_covered_transit(
                memory.by_id(1),
                memory.by_id(2),
                place_memory=memory,
            )
        )

    def test_bound_but_not_crossed_portal_is_not_graph_transit(self):
        ledger = PortalHypothesisLedger()
        entry, _created = ledger.certify(
            1, (0.0, 0.0), (0.8, 0.0), now=1.0,
        )
        ledger.bind_destination(entry["id"], 2, now=2.0)
        branch, _created = ledger.certify(
            2, (1.0, 0.0), (1.8, 0.0), now=3.0,
        )
        ledger.bind_destination(branch["id"], 3, now=4.0)
        memory = PlaceGraphMemory([
            {"id": 1, "state": "dormant", "endpoint_observations": 1},
            {"id": 2, "state": "dormant", "endpoint_observations": 1},
            {"id": 3, "state": "open", "endpoint_observations": 0},
        ])

        # Destination binding records a hypothesis only.  Until the physical
        # crossing commits, it must not create a graph path to Place 3.
        self.assertFalse(
            ledger.allows_covered_transit(
                memory.by_id(1), memory.by_id(2), place_memory=memory,
            )
        )

        ledger.crossed(branch["id"], now=5.0)
        self.assertTrue(
            ledger.allows_covered_transit(
                memory.by_id(1), memory.by_id(2), place_memory=memory,
            )
        )

    def test_wide_gate_fallback_requires_reverse_direction_when_known(self):
        ledger = PortalHypothesisLedger(match_radius=0.30)
        edge, _created = ledger.certify(
            1, (2.84, 1.28), (2.84, 1.59), now=1.0,
        )
        ledger.crossed(edge["id"], now=2.0)
        ledger.bind_destination(edge["id"], 2, now=2.0)

        self.assertIsNotNone(
            ledger.crossed_gate_for_destination(
                2,
                (1.83, 1.285),
                gate_radius=1.25,
                candidate_direction_xy=(0.0, -1.0),
            )
        )
        self.assertIsNone(
            ledger.crossed_gate_for_destination(
                2,
                (1.83, 1.285),
                gate_radius=1.25,
                candidate_direction_xy=(0.0, 1.0),
            )
        )

    def test_covered_transit_is_allowed_for_unresolved_work_item(self):
        ledger = PortalHypothesisLedger()
        edge, _created = ledger.certify(
            1, (0.0, 0.0), (0.8, 0.0), now=1.0,
        )
        ledger.bind_destination(edge["id"], 2, now=2.0)
        memory = PlaceGraphMemory([
            {"id": 1, "state": "dormant", "endpoint_observations": 1},
            {"id": 2, "state": "dormant", "endpoint_observations": 1},
        ])

        self.assertTrue(
            ledger.allows_covered_transit(
                memory.by_id(1),
                memory.by_id(2),
                place_memory=memory,
                work_item_ledger=WorkItemStub({2: 1}),
            )
        )

    def test_covered_transit_is_allowed_for_pending_portal_probe(self):
        ledger = PortalHypothesisLedger()
        edge, _created = ledger.certify(
            1, (0.0, 0.0), (0.8, 0.0), now=1.0,
        )
        ledger.bind_destination(edge["id"], 2, now=2.0)
        probes = PortalProbeLedger()
        probes.observe(2, (1.0, 0.0), (1.0, 0.0), now=3.0)
        memory = PlaceGraphMemory([
            {"id": 1, "state": "dormant", "endpoint_observations": 1},
            {"id": 2, "state": "dormant", "endpoint_observations": 1},
        ])

        self.assertTrue(
            ledger.allows_covered_transit(
                memory.by_id(1),
                memory.by_id(2),
                place_memory=memory,
                portal_probe_ledger=probes,
            )
        )

    def test_unbound_portal_is_graph_progress(self):
        ledger = PortalHypothesisLedger()
        edge, _created = ledger.certify(
            2, (0.0, 0.0), (0.8, 0.0), now=1.0,
        )
        memory = PlaceGraphMemory([
            {"id": 1, "state": "dormant", "endpoint_observations": 1},
            {"id": 2, "state": "dormant", "endpoint_observations": 1},
        ])

        self.assertTrue(
            ledger.has_unresolved_path(2, place_memory=memory)
        )
        self.assertIsNone(edge["destination_place_id"])

    def test_unbound_query_returns_only_reusable_source_portals(self):
        ledger = PortalHypothesisLedger()
        reusable, _created = ledger.certify(
            7, (0.0, 0.0), (1.0, 0.0), now=1.0,
        )
        selected, _created = ledger.certify(
            7, (2.0, 0.0), (3.0, 0.0), now=2.0,
        )
        ledger.select(selected["id"], now=3.0)
        bound, _created = ledger.certify(
            7, (4.0, 0.0), (5.0, 0.0), now=4.0,
        )
        ledger.bind_destination(bound["id"], 8, now=5.0)
        failed, _created = ledger.certify(
            7, (6.0, 0.0), (7.0, 0.0), now=6.0,
        )
        ledger.failed(failed["id"], now=7.0)

        self.assertEqual(
            [record["id"] for record in ledger.unbound_for_source(7)],
            [reusable["id"], selected["id"]],
        )
        self.assertEqual(ledger.unbound_for_source(8), [])

    def test_unbound_portal_is_reprojected_to_current_map_and_keeps_id(self):
        ledger = PortalHypothesisLedger()
        record, _created = ledger.certify(
            7,
            (1.0, 1.0),
            (3.0, 1.0),
            now=1.0,
            map_gate_xy=(100.0, 100.0),
            map_destination_xy=(101.0, 100.0),
        )
        selector = RehydrationSelector(ledger)
        message = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            info=SimpleNamespace(
                resolution=1.0,
                origin=SimpleNamespace(
                    position=SimpleNamespace(x=-1.0, y=-1.0),
                ),
            ),
        )
        request = SimpleNamespace(
            message=message,
            steps=np.ones((8, 8), dtype=np.int32),
            now=2.0,
        )
        context = SimpleNamespace(
            source_place_id=7,
            place_graph_ready=False,
        )
        portal_sources = {}

        selector._populate_adjacent_portal_sources(
            request, context, portal_sources,
        )

        source = portal_sources[("hypothesis", record["id"])]
        self.assertIsInstance(source, PortalSource)
        self.assertEqual(source.hypothesis_id, record["id"])
        self.assertEqual((source.gate_row, source.gate_col), (3, 3))
        self.assertEqual((source.endpoint_row, source.endpoint_col), (2, 4))
        self.assertEqual(ledger.get(record["id"])["map_gate"], [2.0, 2.0])
        self.assertEqual(
            ledger.get(record["id"])["map_destination"], [4.0, 2.0]
        )
        self.assertEqual(selector.last_portal_unbound_reused, 1)

    def test_selector_reuses_explicit_unbound_hypothesis_id(self):
        ledger = PortalHypothesisLedger()
        record, _created = ledger.certify(
            7, (1.0, 1.0), (3.0, 1.0), now=1.0,
        )
        selector = SelectionReuse(ledger)
        request = SimpleNamespace(
            message=SimpleNamespace(header=SimpleNamespace(frame_id="map")),
            map_to_physical_xy=lambda x, y: (x, y),
            now=2.0,
        )
        source = PortalSource(
            frontier_row=1,
            frontier_col=3,
            identity_row=1,
            identity_col=3,
            gate_row=1,
            gate_col=1,
            endpoint_row=1,
            endpoint_col=3,
            hypothesis_id=record["id"],
        )

        reused = selector._refresh_or_certify_portal_hypothesis(
            request, source, (1.0, 1.0), (3.0, 1.0),
        )

        self.assertIs(reused, record)
        self.assertEqual(len(ledger.snapshot()), 1)
        # Reuse telemetry is emitted only after full portal admission; this
        # helper test checks the identity boundary before that later stage.
        self.assertEqual(selector.events, [])

    def test_unbound_portal_query_stops_after_destination_is_bound(self):
        ledger = PortalHypothesisLedger()
        edge, _created = ledger.certify(
            4, (2.0, 3.0), (4.0, 3.0), now=1.0,
        )
        self.assertEqual([item["id"] for item in ledger.unbound_for_source(4)], [edge["id"]])
        ledger.refresh_projection(
            edge["id"], map_gate_xy=(2.5, 3.0), map_destination_xy=(4.5, 3.0), now=2.0,
        )
        self.assertEqual(ledger.get(edge["id"])["map_gate"], [2.5, 3.0])
        ledger.bind_destination(edge["id"], 9, now=3.0)
        self.assertEqual(ledger.unbound_for_source(4), [])


if __name__ == "__main__":
    unittest.main()
