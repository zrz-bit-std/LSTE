"""Pure tests for durable active portal-probe obligations."""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_portal_probe_ledger import (
    PROBE_AWAITING_PROJECTION,
    PROBE_CERTIFIED,
    PROBE_DESTINATION_ACTIVE,
    PROBE_OBSERVED,
    PROBE_PENDING,
    PROBE_SOURCE_ARRIVED,
    PortalProbeLedger,
)
from global_frontier_portal_belief import PortalHypothesisLedger
from global_frontier_portal_probe_lifecycle import (
    GlobalFrontierPortalProbeLifecycleMixin,
)
from global_frontier_portal_probes import (
    PortalObservationProbe,
    portal_verification_viewpoints,
    source_verification_viewpoints,
)
from global_frontier_candidate_portals import GlobalFrontierCandidatePortalMixin


class ProbeLifecycleStub(GlobalFrontierPortalProbeLifecycleMixin):
    def __init__(self):
        self.current_physical_place_id = 7
        self.portal_probe_ledger = PortalProbeLedger(match_radius=0.35)
        self.active_portal_probe_id = None
        self.active_route_id = 4
        self.events = []

    @staticmethod
    def cell_xy(_message, row, col):
        return float(col), float(row)

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


class ProbeCandidateStub(GlobalFrontierCandidatePortalMixin):
    def __init__(self, ledger):
        self.portal_probe_ledger = ledger
        self.frontier_approach_distance = 1.5
        self.completed_radius = 1.25
        self.clearance = 0.52
        self.teb_xy_goal_tolerance = 0.50

    @staticmethod
    def cell_xy(_message, row, col):
        return float(col), float(row)

    @staticmethod
    def nearest_safe_approach(_steps, row, col, _radius, **_kwargs):
        return int(row), int(col)

    @staticmethod
    def candidate_costmap_distance(_validation, _x, _y):
        return None

    @staticmethod
    def nearest_reachable_cell(_message, _steps, x, y):
        return int(round(y)), int(round(x))


class PortalProbeLedgerTest(unittest.TestCase):
    def test_verification_ladder_keeps_one_identity_and_changes_view_geometry(self):
        viewpoints = portal_verification_viewpoints(
            (0.0, 0.0), (1.0, 0.0), 1.0,
        )

        self.assertEqual(
            [name for name, _point in viewpoints],
            [
                "destination_normal",
                "source_lateral_left",
                "source_lateral_right",
            ],
        )
        self.assertEqual(viewpoints[0][1], (1.0, 0.0))
        self.assertEqual(viewpoints[1][1], (-1.0, 1.0))
        self.assertEqual(viewpoints[2][1], (-1.0, -1.0))

    def test_source_verification_ladder_stays_on_source_side(self):
        viewpoints = source_verification_viewpoints(
            (0.0, 0.0), (1.0, 0.0), 1.0,
        )

        self.assertEqual(
            [name for name, _point in viewpoints],
            [
                "source_normal",
                "source_lateral_left",
                "source_lateral_right",
            ],
        )
        self.assertEqual(viewpoints[0][1], (-1.0, 0.0))
        self.assertEqual(viewpoints[1][1], (-1.0, 1.0))
        self.assertEqual(viewpoints[2][1], (-1.0, -1.0))

    def test_probe_viewpoint_must_remain_near_the_opening(self):
        stub = ProbeCandidateStub(PortalProbeLedger())
        self.assertTrue(
            stub.portal_probe_viewpoint_is_local(
                (2.0, 2.0), (3.4, 2.0), resolution=0.1,
            )
        )
        self.assertFalse(
            stub.portal_probe_viewpoint_is_local(
                (2.0, 2.0), (5.0, 2.0), resolution=0.1,
            )
        )

    def test_source_probe_compiles_a_distinct_source_side_viewpoint(self):
        stub = ProbeCandidateStub(PortalProbeLedger())
        request = SimpleNamespace(
            message=SimpleNamespace(
                info=SimpleNamespace(resolution=0.1),
            ),
            steps=np.zeros((20, 20), dtype=np.int32),
            preferred_steps=np.zeros((20, 20), dtype=np.int32),
            selection_tier="strict_clearance",
        )
        probe = PortalObservationProbe((10, 10), (0, 1), 3, "source")

        viewpoint = stub.source_portal_probe_viewpoint(
            request, probe, (10.0, 10.0),
        )

        self.assertIsNotNone(viewpoint)
        # Grid normal (row=0, col=1) points toward +x, so the source stance
        # must be behind the gate and cannot reuse the gate cell itself.
        self.assertLess(viewpoint[1], 10)

    def test_map_updates_reuse_one_physical_direction(self):
        ledger = PortalProbeLedger(match_radius=0.35)
        first = ledger.observe(
            7, (4.0, 2.0), (1.0, 0.0),
            map_gate_xy=(4.0, 2.0), opening_cell=(10, 20), now=1.0,
        )
        second = ledger.observe(
            7, (4.18, 2.06), (1.0, 0.0),
            map_gate_xy=(4.6, 1.4), opening_cell=(11, 19), now=2.0,
        )

        self.assertEqual(first["id"], second["id"])
        self.assertEqual(second["state"], PROBE_PENDING)
        self.assertEqual(second["opening_cell"], [11, 19])

    def test_opposite_directions_are_distinct_obligations(self):
        ledger = PortalProbeLedger(match_radius=0.35)
        forward = ledger.observe(7, (4.0, 2.0), (1.0, 0.0), now=1.0)
        reverse = ledger.observe(7, (4.0, 2.0), (-1.0, 0.0), now=1.0)

        self.assertNotEqual(forward["id"], reverse["id"])

    def test_observed_probe_cannot_be_redispatched_after_map_jitter(self):
        ledger = PortalProbeLedger(match_radius=0.35)
        record = ledger.observe(7, (4.0, 2.0), (1.0, 0.0), now=1.0)
        self.assertIsNotNone(ledger.start(record["id"], now=2.0))
        settled = ledger.finish(
            record["id"], "observed", now=3.0,
            reason="source_viewpoint_observed",
        )
        self.assertEqual(settled["state"], PROBE_OBSERVED)
        updated = ledger.observe(7, (4.15, 2.0), (1.0, 0.0), now=4.0)
        self.assertEqual(updated["id"], record["id"])
        self.assertFalse(ledger.available(updated["id"]))

    def test_source_arrival_waits_for_directed_portal_certification(self):
        ledger = PortalProbeLedger(match_radius=0.35)
        record = ledger.observe(7, (4.0, 2.0), (1.0, 0.0), now=1.0)
        ledger.start(record["id"], now=2.0)
        arrived = ledger.finish(
            record["id"], "source_arrived", now=3.0,
            reason="source_viewpoint_arrived",
        )
        self.assertEqual(arrived["state"], PROBE_SOURCE_ARRIVED)
        self.assertFalse(ledger.available(record["id"]))
        certified = ledger.certify_matching(
            7, (4.1, 2.0), (1.0, 0.0), portal_id=11, now=4.0,
        )
        self.assertEqual(certified["state"], PROBE_CERTIFIED)
        self.assertEqual(certified["portal_id"], 11)

    def test_source_arrival_rehydrates_one_new_destination_viewpoint(self):
        ledger = PortalProbeLedger(match_radius=0.35)
        record = ledger.observe(7, (4.0, 2.0), (1.0, 0.0), now=1.0)
        ledger.start(record["id"], now=2.0, viewpoint_xy=(5.0, 2.0))
        arrived = ledger.finish(
            record["id"], "source_arrived", now=3.0,
            reason="source_viewpoint_arrived",
        )

        # The same physical coordinate is a new fact when the probe changes
        # from source-side arrival to destination-side observation.
        self.assertTrue(ledger.destination_available(record["id"], (5.0, 2.0)))
        self.assertTrue(ledger.destination_available(record["id"], (6.0, 2.0)))
        resumed = ledger.start_destination(
            record["id"], now=4.0, viewpoint_xy=(6.0, 2.0),
        )
        self.assertEqual(resumed["state"], PROBE_DESTINATION_ACTIVE)
        self.assertEqual(resumed["last_phase"], "source")
        self.assertEqual(resumed["active_phase"], "destination")

        still_pending = ledger.finish(
            record["id"], "source_arrived", now=5.0,
            reason="destination_evidence_inconclusive",
        )
        self.assertEqual(still_pending["state"], PROBE_SOURCE_ARRIVED)
        self.assertEqual(still_pending["last_phase"], "destination")
        self.assertEqual(still_pending["viewpoint_history"], [[5.0, 2.0], [6.0, 2.0]])

    def test_viewpoint_provenance_scopes_phase_and_map_epoch(self):
        ledger = PortalProbeLedger(match_radius=0.35)
        record = ledger.observe(
            7,
            (4.0, 2.0),
            (1.0, 0.0),
            map_epoch=10,
            now=1.0,
        )
        ledger.start(
            record["id"],
            now=2.0,
            viewpoint_xy=(5.0, 2.0),
            map_epoch=10,
        )
        source = ledger.finish(
            record["id"],
            "source_arrived",
            now=3.0,
            reason="source_viewpoint_arrived",
        )

        self.assertFalse(
            ledger.viewpoint_available(
                record["id"], (5.0, 2.0), phase="source", map_epoch=10,
            )
        )
        self.assertTrue(
            ledger.viewpoint_available(
                record["id"], (5.0, 2.0), phase="destination", map_epoch=10,
            )
        )
        self.assertEqual(source["viewpoint_history"], [[5.0, 2.0]])
        self.assertEqual(
            source["viewpoint_records"],
            [{
                "xy": [5.0, 2.0],
                "phase": "source",
                "map_epoch": 10,
                "outcome": "source_arrived",
            }],
        )

        destination = ledger.start_destination(
            record["id"],
            now=4.0,
            viewpoint_xy=(5.0, 2.0),
            map_epoch=10,
        )
        self.assertEqual(destination["active_phase"], "destination")
        self.assertEqual(destination["active_map_epoch"], 10)
        settled = ledger.finish(
            record["id"],
            "source_arrived",
            now=5.0,
            reason="destination_evidence_inconclusive",
        )
        self.assertFalse(
            ledger.destination_available(
                record["id"], (5.0, 2.0), map_epoch=10,
            )
        )
        self.assertEqual(ledger.pending_for_source(7, map_epoch=10), [])
        self.assertEqual(
            [item["id"] for item in ledger.pending_for_source(7, map_epoch=11)],
            [record["id"]],
        )
        self.assertTrue(
            ledger.destination_available(
                record["id"], (5.0, 2.0), map_epoch=11,
            )
        )
        self.assertEqual(
            settled["viewpoint_records"][-1],
            {
                "xy": [5.0, 2.0],
                "phase": "destination",
                "map_epoch": 10,
                "outcome": "source_arrived",
            },
        )

    def test_pending_for_source_includes_source_arrived_obligations(self):
        ledger = PortalProbeLedger(match_radius=0.35)
        record = ledger.observe(7, (4.0, 2.0), (1.0, 0.0), now=1.0)
        ledger.start(record["id"], now=2.0, viewpoint_xy=(5.0, 2.0))
        ledger.finish(record["id"], "source_arrived", now=3.0)

        self.assertEqual(
            [item["id"] for item in ledger.pending_for_source(7)],
            [record["id"]],
        )

    def test_destination_view_is_bounded_by_map_evidence_epoch(self):
        ledger = PortalProbeLedger(match_radius=0.35)
        record = ledger.observe(
            7,
            (4.0, 2.0),
            (1.0, 0.0),
            map_epoch=10,
            now=1.0,
        )
        ledger.start(record["id"], now=2.0, viewpoint_xy=(5.0, 2.0))
        ledger.finish(record["id"], "source_arrived", now=3.0)
        ledger.start_destination(record["id"], now=4.0, viewpoint_xy=(6.0, 2.0))
        ledger.finish(
            record["id"],
            "source_arrived",
            now=5.0,
            reason="destination_evidence_inconclusive",
        )

        self.assertEqual(ledger.pending_for_source(7), [])
        self.assertFalse(ledger.destination_available(record["id"], (7.0, 2.0)))

        refreshed = ledger.observe(
            7,
            (4.05, 2.0),
            (1.0, 0.0),
            map_epoch=11,
            now=6.0,
        )
        self.assertEqual(
            [item["id"] for item in ledger.pending_for_source(7)],
            [record["id"]],
        )
        self.assertTrue(ledger.destination_available(refreshed["id"], (7.0, 2.0)))

    def test_observed_legacy_terminal_remains_supported_as_final_state(self):
        ledger = PortalProbeLedger(match_radius=0.35)
        record = ledger.observe(7, (4.0, 2.0), (1.0, 0.0), now=1.0)
        ledger.start(record["id"], now=2.0)
        observed = ledger.finish(record["id"], "observed", now=3.0)
        self.assertEqual(observed["state"], PROBE_OBSERVED)

    def test_route_failure_releases_obligation_for_another_viewpoint(self):
        ledger = PortalProbeLedger(match_radius=0.35)
        record = ledger.observe(7, (4.0, 2.0), (1.0, 0.0), now=1.0)
        ledger.start(record["id"], now=2.0, viewpoint_xy=(5.0, 2.0))
        failed = ledger.finish(record["id"], "failed", now=3.0, reason="stall")

        self.assertEqual(failed["state"], PROBE_PENDING)
        self.assertTrue(ledger.available(record["id"]))
        self.assertFalse(ledger.viewpoint_available(record["id"], (5.1, 2.0)))
        self.assertTrue(ledger.viewpoint_available(record["id"], (6.0, 2.0)))

    def test_destination_route_failure_reopens_same_epoch_for_new_viewpoint(self):
        ledger = PortalProbeLedger(match_radius=0.35)
        record = ledger.observe(
            7,
            (4.0, 2.0),
            (1.0, 0.0),
            map_epoch=10,
            now=1.0,
        )
        ledger.start(record["id"], now=2.0, viewpoint_xy=(3.0, 2.0))
        ledger.finish(record["id"], "source_arrived", now=3.0)
        ledger.start_destination(record["id"], now=4.0, viewpoint_xy=(5.0, 2.0))

        failed = ledger.finish(
            record["id"], "failed", now=5.0, reason="stall",
        )

        self.assertEqual(failed["state"], PROBE_SOURCE_ARRIVED)
        self.assertTrue(failed["destination_retry_allowed"])
        self.assertEqual(
            [item["id"] for item in ledger.pending_for_source(7)],
            [record["id"]],
        )
        self.assertTrue(ledger.destination_available(record["id"], (6.0, 2.0)))

    def test_activation_rejection_is_physical_and_epoch_independent(self):
        ledger = PortalProbeLedger(match_radius=0.35)
        record = ledger.observe(
            7, (4.0, 2.0), (1.0, 0.0), map_epoch=10, now=1.0,
        )
        ledger.start(record["id"], now=2.0, viewpoint_xy=(5.0, 2.0))
        ledger.finish(record["id"], "source_arrived", now=3.0)
        ledger.start_destination(
            record["id"], now=4.0, viewpoint_xy=(6.0, 2.0), map_epoch=10,
        )
        ledger.finish(record["id"], "failed", now=5.0, reason="stall")

        rejected = ledger.reject_viewpoint(
            record["id"], (6.0, 2.0), phase="destination",
            reason="probe_lease_unavailable", now=6.0,
        )

        self.assertEqual(rejected["state"], PROBE_SOURCE_ARRIVED)
        self.assertFalse(
            ledger.viewpoint_available(
                record["id"], (6.02, 2.0), phase="destination", map_epoch=11,
            )
        )
        self.assertTrue(
            ledger.viewpoint_available(
                record["id"], (7.0, 2.0), phase="destination", map_epoch=11,
            )
        )

    def test_gate_matching_radius_does_not_deduplicate_room_viewpoints(self):
        """A wide gate identity radius must not suppress active viewpoints."""
        ledger = PortalProbeLedger(match_radius=2.0)
        record = ledger.observe(
            7, (4.0, 2.0), (1.0, 0.0), map_epoch=1, now=1.0,
        )
        ledger.start(record["id"], now=2.0, viewpoint_xy=(3.0, 2.0))
        arrived = ledger.finish(record["id"], "source_arrived", now=3.0)

        self.assertEqual(arrived["viewpoint_match_radius"], 0.35)
        self.assertEqual(ledger.match_radius, 2.0)
        # The destination-side point is 1 m from the source stance. It is a
        # new observation action even though the doorway identity still uses a
        # 2 m association radius across SLAM projections.
        self.assertTrue(ledger.viewpoint_available(record["id"], (5.0, 2.0)))

    def test_crossed_portal_reverse_side_is_transit_not_a_new_probe(self):
        ledger = PortalHypothesisLedger(match_radius=2.0)
        edge, _created = ledger.certify(
            1, (0.0, 0.0), (1.0, 0.0), now=1.0,
        )
        ledger.crossed(edge["id"], now=2.0)
        ledger.bind_destination(edge["id"], 2, now=2.0)

        reverse = ledger.crossed_reverse_for_destination(
            2, (0.05, 0.02), (-1.0, 0.0), gate_radius=0.7,
        )

        self.assertIsNotNone(reverse)
        self.assertEqual(reverse["id"], edge["id"])
        self.assertIsNone(
            ledger.crossed_reverse_for_destination(
                2, (0.05, 0.02), (1.0, 0.0), gate_radius=0.7,
            )
        )

    def test_projection_gap_parks_probe_until_new_physical_evidence(self):
        ledger = PortalProbeLedger(match_radius=0.35)
        record = ledger.observe(
            7, (4.0, 2.0), (1.0, 0.0), map_epoch=10, now=1.0,
        )

        parked = ledger.mark_projection_unavailable(
            record["id"], map_epoch=10, reason="no_reachable_gate_side_cell",
        )

        self.assertEqual(parked["state"], PROBE_AWAITING_PROJECTION)
        self.assertFalse(ledger.available(record["id"]))
        self.assertEqual(ledger.pending_for_source(7), [])

        reopened = ledger.observe(
            7, (4.05, 2.0), (1.0, 0.0), map_epoch=11, now=2.0,
        )

        self.assertEqual(reopened["id"], record["id"])
        self.assertEqual(reopened["state"], PROBE_PENDING)
        self.assertTrue(ledger.available(record["id"]))

    def test_projection_gap_does_not_reopen_in_the_same_map_epoch(self):
        ledger = PortalProbeLedger(match_radius=0.35)
        record = ledger.observe(
            7, (4.0, 2.0), (1.0, 0.0), map_epoch=10, now=1.0,
        )

        parked = ledger.mark_projection_unavailable(
            record["id"], map_epoch=10, reason="no_reachable_gate_side_cell",
        )
        self.assertEqual(parked["state"], PROBE_AWAITING_PROJECTION)

        same_epoch = ledger.observe(
            7, (4.02, 2.0), (1.0, 0.0), map_epoch=10, now=2.0,
        )
        self.assertEqual(same_epoch["state"], PROBE_AWAITING_PROJECTION)
        self.assertFalse(ledger.available(record["id"]))
        self.assertEqual(ledger.pending_for_source(7, map_epoch=10), [])

        next_epoch = ledger.observe(
            7, (4.03, 2.0), (1.0, 0.0), map_epoch=11, now=3.0,
        )
        self.assertEqual(next_epoch["state"], PROBE_PENDING)
        self.assertTrue(ledger.available(record["id"]))

    def test_pending_for_source_excludes_observed_probe(self):
        ledger = PortalProbeLedger(match_radius=0.35)
        pending = ledger.observe(7, (4.0, 2.0), (1.0, 0.0), now=1.0)
        observed = ledger.observe(7, (8.0, 2.0), (0.0, 1.0), now=1.0)
        ledger.start(observed["id"], now=2.0)
        ledger.finish(observed["id"], "observed", now=3.0)

        self.assertEqual(
            [item["id"] for item in ledger.pending_for_source(7)],
            [pending["id"]],
        )

    def test_pending_probe_can_be_rehydrated_after_frontier_cell_moves(self):
        ledger = PortalProbeLedger(match_radius=0.35)
        record = ledger.observe(
            7, (3.0, 3.0), (1.0, 0.0),
            map_gate_xy=(3.0, 3.0), opening_cell=(3, 3), now=1.0,
        )
        ledger.bind_work_item(record["id"], 21)
        stub = ProbeCandidateStub(ledger)
        request = SimpleNamespace(
            message=SimpleNamespace(
                header=SimpleNamespace(frame_id="map"),
                info=SimpleNamespace(
                    resolution=1.0,
                    origin=SimpleNamespace(
                        position=SimpleNamespace(x=0.0, y=0.0)
                    ),
                ),
            ),
            steps=np.ones((8, 8), dtype=int),
            preferred_steps=None,
            preferred_mask=None,
            selection_tier="strict_clearance",
            validation=None,
            map_to_physical_xy=None,
            now=2.0,
        )
        frontier = np.zeros((8, 8), dtype=bool)
        frontier[3, 4] = True
        candidates = stub.pending_portal_probe_candidates(
            request,
            SimpleNamespace(source_place_id=7),
            frontier,
            approach_cells=1,
        )

        self.assertEqual(len(candidates), 1)
        _row, _col, candidate = candidates[0]
        self.assertEqual(candidate.portal_observation_probe.probe_id, record["id"])
        self.assertEqual(candidate.work_item_id, 21)

    def test_ros_boundary_assigns_identity_and_closes_successful_probe(self):
        stub = ProbeLifecycleStub()
        request = SimpleNamespace(
            message=object(),
            map_to_physical_xy=lambda x, y: (x + 10.0, y - 2.0),
            now=1.0,
        )
        probe = stub.register_portal_probe(
            request, PortalObservationProbe((3, 8), (1, 0)),
        )
        self.assertIsNotNone(probe)
        self.assertIsNotNone(probe.probe_id)
        # The candidate stores a grid (row, col) normal, while the durable
        # ledger must retain a physical/map (x, y) normal.
        self.assertEqual(
            stub.portal_probe_ledger.get(probe.probe_id)["normal_xy"],
            [0.0, 1.0],
        )
        self.assertEqual(stub.start_active_portal_probe(
            SimpleNamespace(
                portal_observation_probe=probe,
                x=8.0,
                y=3.0,
            ), 2.0,
        )["state"], "active")
        settled = stub.settle_active_portal_probe(
            "source_arrived", 3.0, "source_viewpoint_arrived",
        )
        self.assertEqual(settled["state"], PROBE_SOURCE_ARRIVED)
        self.assertEqual(stub.events[-1][0], "portal_probe_settled")
        self.assertIsNone(stub.register_portal_probe(request, probe))

    def test_ros_boundary_does_not_start_the_same_destination_attempt_twice(self):
        stub = ProbeLifecycleStub()
        request = SimpleNamespace(
            message=object(),
            map_to_physical_xy=lambda x, y: (x, y),
            now=1.0,
        )
        probe = stub.register_portal_probe(
            request, PortalObservationProbe((3, 8), (1, 0)),
        )
        stub.start_active_portal_probe(
            SimpleNamespace(portal_observation_probe=probe, x=8.0, y=3.0),
            2.0,
        )
        stub.settle_active_portal_probe(
            "source_arrived", 3.0, "source_viewpoint_arrived",
        )
        destination = PortalObservationProbe(
            probe.opening_cell,
            probe.normal,
            probe.probe_id,
            "destination",
            probe.observation_source,
        )
        stub.start_active_portal_probe(
            SimpleNamespace(portal_observation_probe=destination, x=9.0, y=3.0),
            4.0,
        )
        stub.settle_active_portal_probe("failed", 5.0, "stall")

        self.assertIsNone(stub.start_active_portal_probe(
            SimpleNamespace(portal_observation_probe=destination, x=9.0, y=3.0),
            6.0,
        ))
        self.assertEqual(
            stub.events[-1][0], "portal_probe_activation_rejected",
        )

    def test_reverse_portal_suppression_event_is_deduplicated(self):
        stub = ProbeLifecycleStub()
        stub.current_physical_place_id = 2
        stub.portal_hypothesis_ledger = PortalHypothesisLedger()
        edge, _created = stub.portal_hypothesis_ledger.certify(
            1, (3.0, 3.0), (4.0, 3.0), now=1.0,
        )
        stub.portal_hypothesis_ledger.crossed(edge["id"], now=2.0)
        stub.portal_hypothesis_ledger.bind_destination(edge["id"], 2, now=2.0)
        request = SimpleNamespace(
            message=SimpleNamespace(
                header=SimpleNamespace(frame_id="map"),
            ),
            map_to_physical_xy=lambda x, y: (x, y),
            components=SimpleNamespace(epoch=3),
            now=3.0,
        )
        probe = PortalObservationProbe((3, 3), (0, -1))

        self.assertIsNone(stub.register_portal_probe(request, probe))
        self.assertIsNone(stub.register_portal_probe(request, probe))
        self.assertEqual(
            [event for event, _fields in stub.events],
            ["portal_probe_suppressed"],
        )

    def test_wide_gate_suppression_survives_a_flipped_normal(self):
        stub = ProbeLifecycleStub()
        stub.current_physical_place_id = 2
        stub.region_memory = SimpleNamespace(
            portal_entry_merge_radius=0.35,
            portal_entry_match_radius=1.25,
        )
        stub.portal_hypothesis_ledger = PortalHypothesisLedger(
            match_radius=0.30,
        )
        edge, _created = stub.portal_hypothesis_ledger.certify(
            1, (3.0, 3.0), (3.0, 4.0), now=1.0,
        )
        stub.portal_hypothesis_ledger.crossed(edge["id"], now=2.0)
        stub.portal_hypothesis_ledger.bind_destination(edge["id"], 2, now=2.0)
        request = SimpleNamespace(
            message=SimpleNamespace(
                header=SimpleNamespace(frame_id="map"),
            ),
            map_to_physical_xy=lambda x, y: (x, y),
            components=SimpleNamespace(epoch=3),
            now=3.0,
        )
        # The candidate is one metre along the same opening and points through
        # the opposite half-plane.  A point-radius-only query would miss it.
        probe = PortalObservationProbe((3, 2), (0, -1))

        self.assertIsNone(stub.register_portal_probe(request, probe))
        self.assertEqual(
            stub.events[-1][1]["reason"],
            "same_wide_gate_as_crossed_portal_is_transit",
        )

    def test_destination_viewpoint_has_distinct_observation_terminal(self):
        stub = ProbeLifecycleStub()
        record = stub.portal_probe_ledger.observe(
            7, (4.0, 2.0), (1.0, 0.0), now=1.0,
        )
        stub.portal_probe_ledger.start(
            record["id"], now=2.0, viewpoint_xy=(3.0, 2.0),
        )
        stub.portal_probe_ledger.finish(
            record["id"], "source_arrived", now=3.0,
        )
        stub.portal_probe_ledger.start_destination(
            record["id"], now=4.0, viewpoint_xy=(5.0, 2.0),
        )
        stub.active_portal_probe_id = record["id"]

        settled = stub.settle_active_portal_probe(
            "observed", 5.0, "destination_viewpoint_observed",
        )

        self.assertEqual(settled["state"], PROBE_OBSERVED)
        self.assertEqual(settled["last_phase"], "destination")
        self.assertEqual(stub.events[-1][1]["reason"], "destination_viewpoint_observed")


if __name__ == "__main__":
    unittest.main()
