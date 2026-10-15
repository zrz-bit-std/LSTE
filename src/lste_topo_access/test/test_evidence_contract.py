"""Pure regression tests for ECAG evidence contracts."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_evidence_contract import (  # noqa: E402
    EVIDENCE_DESTINATION_VIEW,
    EVIDENCE_LOCAL_OBSERVATION,
    EVIDENCE_PORTAL_CROSSING,
    EVIDENCE_SOURCE_VIEW,
    EVIDENCE_PORTAL_CERTIFIED,
    EvidenceObservation,
    apply_observation,
    build_contract,
    next_evidence_gap,
)
from global_frontier_event_graph import EvidenceEventGraph  # noqa: E402
from global_frontier_graph_route_planner import GraphRoutePlan  # noqa: E402


class EvidenceContractTest(unittest.TestCase):
    def test_probe_phase_is_explicit_and_deterministic(self):
        snapshot = build_contract(
            work_items=[{"id": 3, "place_id": 1, "state": "unresolved"}],
            probes=[{"id": 7, "source_place_id": 1, "state": "source_arrived"}],
            portals=[{
                "id": 11,
                "source_place_id": 1,
                "destination_place_id": None,
                "state": "certified",
            }],
        )
        self.assertEqual(
            [(item.owner_id, item.kind) for item in snapshot.pending()],
            [(11, EVIDENCE_PORTAL_CROSSING),
             (7, EVIDENCE_DESTINATION_VIEW),
             (3, EVIDENCE_LOCAL_OBSERVATION)],
        )

    def test_destination_view_precedes_unrelated_local_work(self):
        gap = next_evidence_gap(
            1,
            work_items=[{"id": 3, "place_id": 1, "state": "unresolved"}],
            probes=[{"id": 7, "source_place_id": 1, "state": "source_arrived"}],
        )
        self.assertEqual(gap.owner_id, 7)
        self.assertEqual(gap.kind, EVIDENCE_DESTINATION_VIEW)

    def test_pending_probe_does_not_starve_local_work(self):
        gap = next_evidence_gap(
            1,
            work_items=[{"id": 3, "place_id": 1, "state": "unresolved"}],
            probes=[{"id": 7, "source_place_id": 1, "state": "pending"}],
        )
        self.assertEqual(gap.owner_id, 3)
        self.assertEqual(gap.kind, EVIDENCE_LOCAL_OBSERVATION)

    def test_observation_is_idempotent_and_unknown_events_are_noops(self):
        snapshot = build_contract(
            probes=[{"id": 7, "source_place_id": 1, "state": "pending"}],
        )
        observation = EvidenceObservation(
            "source_viewpoint_arrived", "portal_probe", 7, EVIDENCE_SOURCE_VIEW,
        )
        resolved = apply_observation(snapshot, observation)
        self.assertEqual(resolved.revision, 1)
        self.assertEqual(apply_observation(resolved, observation), resolved)
        unknown = EvidenceObservation(
            "late_event", "portal_probe", 99, EVIDENCE_SOURCE_VIEW,
        )
        self.assertEqual(apply_observation(resolved, unknown), resolved)

    def test_replay_order_does_not_change_the_contract(self):
        snapshot = build_contract(
            work_items=[{"id": 3, "place_id": 1, "state": "unresolved"}],
            portals=[{
                "id": 11,
                "source_place_id": 1,
                "destination_place_id": None,
                "state": "certified",
            }],
        )
        observations = (
            EvidenceObservation(
                "frontier_endpoint_observed",
                "work_item",
                3,
                EVIDENCE_LOCAL_OBSERVATION,
                route_id=1,
            ),
            EvidenceObservation(
                "portal_crossing_verified",
                "portal",
                11,
                EVIDENCE_PORTAL_CROSSING,
                route_id=2,
            ),
        )
        forward = apply_observation(
            apply_observation(snapshot, observations[0]), observations[1]
        )
        reverse = apply_observation(
            apply_observation(snapshot, observations[1]), observations[0]
        )

        self.assertEqual(forward.signature(), reverse.signature())
        self.assertEqual(len(forward.pending()), 0)

    def test_route_terminal_is_not_observation_proof(self):
        graph = EvidenceEventGraph()
        graph.ingest(
            "route_command",
            {
                "route_id": 4,
                "evidence_requirements": [{
                    "owner_kind": "work_item",
                    "owner_id": 3,
                    "kind": EVIDENCE_LOCAL_OBSERVATION,
                }],
            },
        )
        graph.ingest("route_terminal", {"route_id": 4, "state": "succeeded"})

        pending = graph.evidence_contract.pending()
        self.assertEqual([(item.owner_id, item.kind) for item in pending], [(3, EVIDENCE_LOCAL_OBSERVATION)])

    def test_nested_route_contract_is_reduced_and_owner_namespaces_stay_distinct(self):
        graph = EvidenceEventGraph()
        graph.ingest(
            "route_command",
            {
                "route_id": 7,
                "graph_route_plan": {
                    "required_evidence": [{
                        "owner_kind": "portal_probe",
                        "owner_id": 41,
                        "kind": EVIDENCE_DESTINATION_VIEW,
                    }],
                },
            },
        )
        graph.ingest(
            "portal_destination_view_observed",
            {"portal_probe_id": 41, "route_id": 7},
        )
        self.assertEqual(graph.evidence_contract.pending(), ())
        self.assertEqual(
            graph.evidence_contract.resolved[0].owner_kind,
            "portal_probe",
        )

    def test_portal_certification_is_not_a_crossing(self):
        graph = EvidenceEventGraph()
        graph.ingest(
            "route_command",
            {
                "route_id": 8,
                "graph_route_plan": {
                    "required_evidence": [{
                        "owner_kind": "portal",
                        "owner_id": 13,
                        "kind": EVIDENCE_PORTAL_CERTIFIED,
                    }, {
                        "owner_kind": "portal",
                        "owner_id": 13,
                        "kind": EVIDENCE_PORTAL_CROSSING,
                    }],
                },
            },
        )
        graph.ingest("portal_hypothesis_certified", {"portal_id": 13})
        pending = graph.evidence_contract.pending()
        self.assertEqual(
            [(item.owner_id, item.kind) for item in pending],
            [(13, EVIDENCE_PORTAL_CROSSING)],
        )

    def test_graph_plan_publishes_the_action_contract(self):
        plan = GraphRoutePlan(
            "ready",
            "observe_local_work",
            current_place_id=1,
            target_place_id=1,
            obligation_kind="work_item",
            obligation_id=3,
        )

        payload = plan.as_dict()
        self.assertEqual(payload["required_evidence"][0]["kind"], "place_view")
        self.assertEqual(payload["produces_evidence"][0]["kind"], EVIDENCE_LOCAL_OBSERVATION)


if __name__ == "__main__":
    unittest.main()
