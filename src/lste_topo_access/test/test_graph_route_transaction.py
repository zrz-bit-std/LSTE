"""Regression tests for the two-phase graph-action transaction."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_graph_route_planner import (  # noqa: E402
    ACTION_BOOTSTRAP,
    ACTION_CROSS_PORTAL,
    ACTION_OBSERVE_LOCAL_WORK,
    ACTION_PROBE_PORTAL,
    ACTION_REINSPECT_TARGET,
    ACTION_RETRY_VIEWPOINT,
    GraphRoutePlan,
    PLAN_READY,
)
from global_frontier_graph_route_transaction import (  # noqa: E402
    GraphRouteActionTransaction,
    materialize_graph_route_action,
    with_graph_action,
)
from global_frontier_portal_probes import PortalObservationProbe  # noqa: E402
from global_frontier_frontier_decision import candidate_from_route  # noqa: E402
from global_frontier_selection_planner import (  # noqa: E402
    GlobalFrontierSelectionPlannerMixin,
)


def plan(action, **kwargs):
    values = dict(
        status=PLAN_READY,
        action=action,
        current_place_id=1,
        target_place_id=1,
        obligation_kind="work_item",
        obligation_id=7,
        reason="test_plan",
    )
    values.update(kwargs)
    return GraphRoutePlan(**values)


def route(
    action,
    *,
    work_item_id=7,
    hops=0,
    route_kind="frontier_endpoint",
    gate=None,
    portal_id=None,
    reason="test_candidate",
):
    values = [
        1, 2, 1.0, 2.0, 1.0, 10.0, 0.0, 0.0, None, hops,
        route_kind, gate, work_item_id, "matched", 4, None, False,
        action, reason,
    ]
    if portal_id is not None:
        values.append(portal_id)
    return tuple(values)


class GraphRouteTransactionTest(unittest.TestCase):
    def test_bootstrap_accepts_rehydrated_local_observation(self):
        """The first Place view may be projected by the WorkItem adapter."""
        result = materialize_graph_route_action(
            plan(ACTION_BOOTSTRAP),
            route(ACTION_OBSERVE_LOCAL_WORK),
        )

        self.assertTrue(result.accepted)
        self.assertTrue(result.reconciled)
        self.assertEqual(result.plan.action, ACTION_BOOTSTRAP)
        self.assertEqual(result.plan.obligation_id, 7)

    def test_bootstrap_rejects_rehydrated_work_from_another_place(self):
        candidate = SimpleNamespace(
            route=route(ACTION_OBSERVE_LOCAL_WORK, work_item_id=8),
            place_id=2,
        )
        result = materialize_graph_route_action(plan(ACTION_BOOTSTRAP), candidate)

        self.assertFalse(result.accepted)
        self.assertEqual(
            result.reason, "bootstrap_work_item_place_identity_missing"
        )

    def test_route_projection_preserves_committed_target_action(self):
        original = route(ACTION_OBSERVE_LOCAL_WORK)

        projected = with_graph_action(
            original,
            ACTION_REINSPECT_TARGET,
            "target_observation_required",
        )

        self.assertEqual(original[17], ACTION_OBSERVE_LOCAL_WORK)
        self.assertEqual(projected[17], ACTION_REINSPECT_TARGET)
        self.assertEqual(projected[18], "target_observation_required")
        self.assertEqual(projected[:17], original[:17])

    def test_selection_projects_target_action_in_legacy_scalar_mode(self):
        class Selector(GlobalFrontierSelectionPlannerMixin):
            frontier_action_policy = "legacy_scalar"
            last_frontier_region_tier = None
            last_selected_place_hops = None
            last_selected_viewpoint_retry = False

        prepared = plan(
            ACTION_REINSPECT_TARGET,
            target_place_id=1,
            obligation_kind="target_observation",
            obligation_id=None,
        )
        candidate = candidate_from_route(
            route(ACTION_OBSERVE_LOCAL_WORK),
            region_tier="revisit",
            place_id=1,
        )

        selected = Selector()._select_graph_plan_candidate(
            [candidate], prepared, SimpleNamespace()
        )

        self.assertEqual(selected[17], ACTION_REINSPECT_TARGET)
        self.assertEqual(selected[18], prepared.reason)

    def test_local_work_commits_the_actual_work_item_identity(self):
        prepared = plan(ACTION_OBSERVE_LOCAL_WORK, obligation_id=3)
        result = materialize_graph_route_action(
            prepared,
            route(ACTION_OBSERVE_LOCAL_WORK, work_item_id=3),
        )

        self.assertTrue(result.accepted)
        self.assertFalse(result.reconciled)
        self.assertEqual(result.plan.action, ACTION_OBSERVE_LOCAL_WORK)
        self.assertEqual(result.plan.obligation_id, 3)

    def test_target_reinspection_commits_only_a_local_viewpoint(self):
        prepared = plan(
            ACTION_REINSPECT_TARGET,
            target_place_id=1,
            obligation_kind="target_observation",
            obligation_id=None,
        )
        result = materialize_graph_route_action(
            prepared,
            route(ACTION_OBSERVE_LOCAL_WORK, work_item_id=7),
        )

        self.assertTrue(result.accepted)
        self.assertEqual(result.plan.action, ACTION_REINSPECT_TARGET)
        self.assertEqual(result.plan.obligation_kind, "target_observation")
        self.assertEqual(result.plan.target_place_id, 1)

    def test_target_reinspection_rejects_portal_probe(self):
        prepared = plan(
            ACTION_REINSPECT_TARGET,
            target_place_id=1,
            obligation_kind="target_observation",
            obligation_id=None,
        )
        probe = route(ACTION_PROBE_PORTAL, work_item_id=7)
        probe = probe[:15] + (type("Probe", (), {"probe_id": 11})(),) + probe[16:]
        result = materialize_graph_route_action(prepared, probe)

        self.assertFalse(result.accepted)
        self.assertEqual(
            result.reason, "target_reinspection_requires_local_viewpoint"
        )

    def test_target_reinspection_rejects_cross_place_route(self):
        prepared = plan(
            ACTION_REINSPECT_TARGET,
            target_place_id=1,
            obligation_kind="target_observation",
            obligation_id=None,
        )
        result = materialize_graph_route_action(
            prepared,
            route(
                "cross_portal",
                work_item_id=None,
                hops=1,
                route_kind="portal_transition",
                gate=(1.5, 2.0),
                portal_id=11,
            ),
        )

        self.assertFalse(result.accepted)
        self.assertEqual(
            result.reason, "target_reinspection_requires_local_viewpoint"
        )

    def test_target_reinspection_rejects_missing_same_place_proof(self):
        prepared = plan(
            ACTION_REINSPECT_TARGET,
            target_place_id=1,
            obligation_kind="target_observation",
            obligation_id=None,
        )
        malformed = route(ACTION_OBSERVE_LOCAL_WORK)
        malformed = malformed[:9] + (None,) + malformed[10:]

        result = materialize_graph_route_action(prepared, malformed)

        self.assertFalse(result.accepted)
        self.assertEqual(
            result.reason, "target_reinspection_requires_local_viewpoint"
        )

    def test_target_reinspection_rejects_named_candidate_from_another_place(self):
        prepared = plan(
            ACTION_REINSPECT_TARGET,
            target_place_id=1,
            obligation_kind="target_observation",
            obligation_id=None,
        )
        candidate = SimpleNamespace(
            route=route(ACTION_OBSERVE_LOCAL_WORK),
            place_id=2,
        )

        result = materialize_graph_route_action(prepared, candidate)

        self.assertFalse(result.accepted)
        self.assertEqual(
            result.reason, "target_reinspection_requires_local_viewpoint"
        )

    def test_local_work_reconciles_to_another_item_in_same_place(self):
        candidate = SimpleNamespace(
            route=route(ACTION_OBSERVE_LOCAL_WORK, work_item_id=7),
            place_id=1,
        )
        result = materialize_graph_route_action(
            plan(ACTION_OBSERVE_LOCAL_WORK, obligation_id=3),
            candidate,
        )

        self.assertTrue(result.accepted)
        self.assertTrue(result.reconciled)
        self.assertEqual(result.plan.obligation_id, 7)

    def test_local_work_reconciliation_rejects_another_place(self):
        candidate = SimpleNamespace(
            route=route(ACTION_OBSERVE_LOCAL_WORK, work_item_id=7),
            place_id=2,
        )
        result = materialize_graph_route_action(
            plan(ACTION_OBSERVE_LOCAL_WORK, obligation_id=3),
            candidate,
        )

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "local_work_item_place_changed")

    def test_local_work_reconciliation_rejects_missing_place_identity(self):
        result = materialize_graph_route_action(
            plan(ACTION_OBSERVE_LOCAL_WORK, obligation_id=3),
            route(ACTION_OBSERVE_LOCAL_WORK, work_item_id=7),
        )

        self.assertFalse(result.accepted)
        self.assertEqual(
            result.reason, "local_work_item_place_identity_missing"
        )

    def test_failed_viewpoint_is_an_explicit_same_work_item_refinement(self):
        result = materialize_graph_route_action(
            plan(ACTION_OBSERVE_LOCAL_WORK),
            route(ACTION_RETRY_VIEWPOINT),
        )

        self.assertTrue(result.accepted)
        self.assertTrue(result.reconciled)
        self.assertEqual(result.plan.action, ACTION_RETRY_VIEWPOINT)
        self.assertEqual(result.plan.obligation_id, 7)

    def test_local_intent_can_promote_to_probe_only_with_probe_identity(self):
        probe = route(ACTION_PROBE_PORTAL, work_item_id=None)
        # A source-side probe carries its durable ID in the probe object slot.
        probe = probe[:15] + (type("Probe", (), {"probe_id": 11})(),) + probe[16:]
        result = materialize_graph_route_action(plan(ACTION_OBSERVE_LOCAL_WORK), probe)

        self.assertTrue(result.accepted)
        self.assertTrue(result.reconciled)
        self.assertEqual(result.plan.action, ACTION_PROBE_PORTAL)
        self.assertEqual(result.plan.first_portal_id, 11)

    def test_destination_probe_rejects_a_source_phase_candidate(self):
        prepared = plan(
            ACTION_PROBE_PORTAL,
            target_place_id=None,
            obligation_kind="portal_probe",
            obligation_id=11,
            portal_path=(11,),
            first_portal_id=11,
            portal_probe_phase="destination",
        )
        candidate = route(ACTION_PROBE_PORTAL, work_item_id=None)
        candidate = candidate[:15] + (
            PortalObservationProbe((1, 1), (1, 0), 11, "source"),
        ) + candidate[16:]

        result = materialize_graph_route_action(prepared, candidate)

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "portal_probe_phase_changed")

    def test_crossing_promotion_requires_branch_first_and_portal_proof(self):
        candidate = route(
            ACTION_CROSS_PORTAL,
            work_item_id=None,
            hops=1,
            route_kind="portal_transition",
            gate=(1.5, 2.0),
            portal_id=11,
            reason="certified_portal_branch_to_unobserved_place",
        )
        blocked = materialize_graph_route_action(
            plan(ACTION_OBSERVE_LOCAL_WORK), candidate, branch_first=False,
        )
        accepted = materialize_graph_route_action(
            plan(ACTION_OBSERVE_LOCAL_WORK), candidate, branch_first=True,
        )

        self.assertFalse(blocked.accepted)
        self.assertEqual(blocked.reason, "crossing_promotion_requires_branch_first")
        self.assertTrue(accepted.accepted)
        self.assertTrue(accepted.reconciled)
        self.assertEqual(accepted.plan.action, ACTION_CROSS_PORTAL)
        self.assertEqual(accepted.plan.first_portal_id, 11)

    def test_crossing_promotion_without_physical_proof_is_rejected(self):
        candidate = route(ACTION_CROSS_PORTAL, hops=1, portal_id=11)
        result = materialize_graph_route_action(
            plan(ACTION_OBSERVE_LOCAL_WORK), candidate, branch_first=True,
        )

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "crossing_promotion_missing_portal_proof")

    def test_committed_crossing_cannot_change_portal_identity(self):
        prepared = plan(
            ACTION_CROSS_PORTAL,
            target_place_id=2,
            obligation_kind="unobserved_place",
            obligation_id=None,
            portal_path=(11,),
            first_portal_id=11,
        )
        result = materialize_graph_route_action(
            prepared,
            route(
                ACTION_CROSS_PORTAL,
                work_item_id=None,
                hops=1,
                route_kind="portal_transition",
                gate=(1.5, 2.0),
                portal_id=12,
            ),
        )

        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "portal_crossing_identity_changed")

    def test_transaction_commit_does_not_mutate_prepared_record(self):
        prepared_plan = plan(ACTION_OBSERVE_LOCAL_WORK)
        transaction = GraphRouteActionTransaction(4, prepared_plan, map_epoch=9)
        result = materialize_graph_route_action(
            prepared_plan, route(ACTION_RETRY_VIEWPOINT),
        )
        committed = transaction.commit(result)

        self.assertEqual(transaction.phase, "prepared")
        self.assertEqual(transaction.plan.action, ACTION_OBSERVE_LOCAL_WORK)
        self.assertEqual(committed.phase, "committed")
        self.assertEqual(committed.plan.action, ACTION_RETRY_VIEWPOINT)


if __name__ == "__main__":
    unittest.main()
