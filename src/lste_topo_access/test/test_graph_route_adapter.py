"""Regression tests for graph-plan to portal-action adaptation."""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

sys.modules.setdefault(
    "rospy",
    SimpleNamespace(
        loginfo=lambda *_args, **_kwargs: None,
        logwarn=lambda *_args, **_kwargs: None,
    ),
)

from global_frontier_graph_route_adapter import (  # noqa: E402
    GlobalFrontierGraphRouteAdapterMixin,
)
from global_frontier_graph_route_planner import (  # noqa: E402
    ACTION_CROSS_PORTAL,
    ACTION_PROBE_PORTAL,
    ACTION_REINSPECT_TARGET,
    GraphRoutePlan,
    PLAN_BLOCKED,
    PLAN_READY,
)
from global_frontier_graph_route_gate import (  # noqa: E402
    candidate_matches_graph_plan,
    candidate_refines_graph_plan,
    graph_plan_is_exclusive,
)
from global_frontier_graph_route_transaction import (  # noqa: E402
    materialize_graph_route_action,
)
from global_frontier_frontier_decision import candidate_from_route  # noqa: E402
from global_frontier_planning_selection import (  # noqa: E402
    GlobalFrontierPlanningSelectionMixin,
)


class Ledger:
    def __init__(self, records):
        self.records = list(records)

    def snapshot(self):
        return list(self.records)

    def get(self, record_id):
        return next(
            (item for item in self.records if int(item.get("id", -1)) == int(record_id)),
            None,
        )

    def mark_unbound_rejected(self, record_id, map_epoch, reason=""):
        record = self.get(record_id)
        if record is None:
            return None
        record["rejected_map_epoch"] = map_epoch
        record["last_rejection_reason"] = reason
        return record


class RegionMemory:
    def __init__(self, regions):
        self.regions = list(regions)


class AdapterFixture(GlobalFrontierGraphRouteAdapterMixin):
    def __init__(self, portals=(), probes=(), work_items=()):
        self.graph_route_planner_enabled = True
        self.graph_route_planner = __import__(
            "global_frontier_graph_route_planner",
            fromlist=["GraphRoutePlanner"],
        ).GraphRoutePlanner()
        self.current_physical_place_id = 1
        self.branch_first_enabled = True
        self.region_memory = RegionMemory([
            {"id": 1, "state": "open", "endpoint_observations": 1},
            {"id": 2, "state": "open", "endpoint_observations": 1},
            {"id": 3, "state": "open", "endpoint_observations": 1},
        ])
        self.portal_hypothesis_ledger = Ledger(portals)
        self.portal_probe_ledger = Ledger(probes)
        self.place_work_items = Ledger(work_items)
        self.last_graph_route_plan = None
        self.last_graph_route_plan_signature = None
        self.graph_route_portal_id = None
        self.graph_route_probe_id = None
        self.active_route_id = 17
        self.events = []

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


class GraphRouteAdapterTest(unittest.TestCase):
    def test_stage_only_plan_does_not_create_transaction(self):
        fixture = AdapterFixture(
            portals=[
                {
                    "id": 11,
                    "source_place_id": 1,
                    "destination_place_id": 2,
                    "state": "crossed",
                },
            ],
            work_items=[
                {
                    "id": 31,
                    "place_id": 2,
                    "state": "unresolved",
                    "active_attempt_id": None,
                },
            ],
        )

        plan, candidate = fixture._prepare_graph_route_action(
            None, stage_only=True,
        )

        self.assertIsNone(candidate)
        self.assertEqual(plan.first_portal_id, 11)
        self.assertIsNone(fixture.last_graph_route_plan)
        self.assertIsNone(
            getattr(fixture, "graph_route_action_transaction", None)
        )

    def test_parked_probe_invalidates_ready_plan_lease(self):
        fixture = AdapterFixture(
            probes=[
                {
                    "id": 7,
                    "source_place_id": 1,
                    "state": "awaiting_projection",
                    "portal_id": None,
                },
            ],
        )
        fixture.last_graph_route_plan = GraphRoutePlan(
            PLAN_READY,
            ACTION_PROBE_PORTAL,
            current_place_id=1,
            obligation_kind="portal_probe",
            obligation_id=7,
            first_portal_id=7,
        )
        fixture.graph_route_plan_lease_active = True
        fixture.graph_route_plan_lease_signature = (
            fixture.last_graph_route_plan.signature()
        )

        plan, candidate = fixture._prepare_graph_route_action(
            None, reuse_existing_plan=True,
        )

        self.assertIsNone(candidate)
        self.assertEqual(plan.status, PLAN_BLOCKED)
        self.assertFalse(fixture.graph_route_plan_lease_active)
        self.assertIsNone(fixture.graph_route_probe_id)

    def test_explicit_reinspection_event_reaches_graph_planner(self):
        fixture = AdapterFixture()
        fixture.target_reinspection_pending = True

        plan = fixture._plan_durable_graph_route()

        self.assertEqual(plan.action, ACTION_REINSPECT_TARGET)
        self.assertEqual(plan.target_place_id, 1)
        self.assertEqual(plan.obligation_kind, "target_observation")

    def test_forward_plan_reserves_only_first_portal(self):
        fixture = AdapterFixture(
            portals=[
                {"id": 11, "source_place_id": 1, "destination_place_id": 2, "state": "crossed"},
                {"id": 12, "source_place_id": 2, "destination_place_id": 3, "state": "crossed"},
            ],
            work_items=[
                {"id": 31, "place_id": 3, "state": "unresolved", "active_attempt_id": None},
            ],
        )

        plan, candidate = fixture._prepare_graph_route_action(SimpleNamespace())

        self.assertIsNone(candidate)
        self.assertEqual(plan.portal_path, (11, 12))
        self.assertEqual(fixture.graph_route_portal_id, 11)
        self.assertEqual(fixture.events[-1][0], "graph_route_plan_selected")
        self.assertEqual(fixture.events[-1][1]["first_portal_id"], 11)

    def test_probe_plan_is_materialized_without_inventing_destination(self):
        fixture = AdapterFixture(
            probes=[
                {
                    "id": 7,
                    "source_place_id": 1,
                    "state": "pending",
                    "work_item_id": None,
                },
            ],
        )
        candidate = (1, 2, 3.0, 4.0, 1.2, 0.0, 0.0, 0.0, None, 0, "frontier_endpoint")
        fixture.select_durable_portal_probe = lambda _snapshot: candidate

        plan, selected = fixture._prepare_graph_route_action(SimpleNamespace())

        self.assertEqual(plan.first_portal_id, 7)
        self.assertIs(selected, candidate)
        self.assertIsNone(plan.target_place_id)
        self.assertEqual(fixture.events[-1][0], "graph_route_edge_materialized")
        self.assertEqual(fixture.events[-1][1]["portal_path"], [7])

    def test_unbound_cross_plan_uses_durable_crossing_materializer(self):
        fixture = AdapterFixture(
            portals=[
                {
                    "id": 11,
                    "source_place_id": 1,
                    "destination_place_id": None,
                    "state": "certified",
                },
            ],
            probes=[
                {
                    "id": 21,
                    "source_place_id": 1,
                    "portal_id": 11,
                    "state": "observed",
                },
            ],
            work_items=[
                {"id": 31, "place_id": 1, "state": "unresolved", "active_attempt_id": None},
            ],
        )
        candidate = (
            1, 2, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0, None, 1,
            "portal_transition", (0.0, 0.0), None, None, 0, None,
            False, "cross_portal", "durable_portal_crossing", 11,
        )
        fixture.select_durable_portal_crossing = lambda _snapshot: candidate

        plan, selected = fixture._prepare_graph_route_action(SimpleNamespace())

        self.assertEqual(plan.action, ACTION_CROSS_PORTAL)
        self.assertIs(selected, candidate)
        self.assertFalse(fixture.graph_route_plan_lease_active)
        self.assertEqual(fixture.events[-1][0], "graph_route_edge_materialized")

    def test_unavailable_crossing_replans_to_local_work_in_same_map_epoch(self):
        """A rejected Portal must not retain a graph lease indefinitely."""
        fixture = AdapterFixture(
            portals=[{
                "id": 11,
                "source_place_id": 1,
                "destination_place_id": None,
                "state": "certified",
            }],
            probes=[{
                "id": 21,
                "source_place_id": 1,
                "portal_id": 11,
                "state": "observed",
            }],
            work_items=[{
                "id": 31,
                "place_id": 1,
                "state": "unresolved",
                "active_attempt_id": None,
            }],
        )
        fixture.select_durable_portal_crossing = lambda _snapshot: (
            setattr(
                fixture,
                "last_durable_portal_crossing_unavailable",
                {
                    "portal_id": 11,
                    "map_epoch": 7,
                    "reason": "destination_side_not_reachable",
                },
            )
            or None
        )
        snapshot = SimpleNamespace(
            map_context=SimpleNamespace(components=SimpleNamespace(epoch=7)),
            route_graph=SimpleNamespace(validation=None),
        )

        plan, candidate = fixture._prepare_graph_route_action(snapshot)

        self.assertIsNone(candidate)
        self.assertEqual(plan.action, "observe_local_work")
        self.assertEqual(plan.obligation_id, 31)
        self.assertFalse(fixture.graph_route_plan_lease_active)
        self.assertIsNone(fixture.graph_route_portal_id)
        self.assertEqual(
            fixture.portal_hypothesis_ledger.get(11)["rejected_map_epoch"],
            7,
        )
        self.assertEqual(
            fixture.events[-1][0], "graph_route_materialization_replanned"
        )

    def test_terminal_materialization_reuses_prepared_plan(self):
        fixture = AdapterFixture()
        prepared = GraphRoutePlan(
            PLAN_READY,
            ACTION_CROSS_PORTAL,
            current_place_id=1,
            target_place_id=2,
            obligation_kind="unobserved_place",
            portal_path=(11,),
            first_portal_id=11,
        )
        fixture.last_graph_route_plan = prepared
        fixture.graph_route_planner.plan = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("terminal must not replan the prepared action")
        )

        plan, selected = fixture._prepare_graph_route_action(
            SimpleNamespace(), reuse_existing_plan=True,
        )

        self.assertIs(plan, prepared)
        self.assertIsNone(selected)
        self.assertEqual(fixture.graph_route_portal_id, 11)

    def test_materialization_wait_lease_reuses_plan_across_snapshots(self):
        fixture = AdapterFixture(
            probes=[
                {
                    "id": 7,
                    "source_place_id": 1,
                    "state": "pending",
                    "work_item_id": None,
                },
            ],
        )
        prepared, _unused = fixture._prepare_graph_route_action(SimpleNamespace())
        fixture._retain_graph_route_plan_lease(prepared, "test_wait")

        def planner_must_not_replan(*_args, **_kwargs):
            raise AssertionError("a waiting graph lease must survive map snapshots")

        fixture.graph_route_planner.plan = planner_must_not_replan
        reused, selected = fixture._prepare_graph_route_action(
            SimpleNamespace(),
            reuse_existing_plan=True,
            visible_work_item_ids=(),
            visible_probe_ids=(),
        )

        self.assertIs(reused, prepared)
        self.assertIsNone(selected)
        self.assertEqual(fixture.graph_route_probe_id, 7)
        self.assertTrue(fixture.graph_route_plan_lease_active)

    def test_probe_phase_transition_replans_before_materialization(self):
        """A source completion must invalidate the old source plan."""
        fixture = AdapterFixture(
            probes=[
                {
                    "id": 7,
                    "source_place_id": 1,
                    "state": "source_arrived",
                    "portal_id": None,
                },
            ],
        )
        prepared = GraphRoutePlan(
            PLAN_READY,
            ACTION_PROBE_PORTAL,
            current_place_id=1,
            obligation_kind="portal_probe",
            obligation_id=7,
            portal_path=(7,),
            first_portal_id=7,
            portal_probe_phase="source",
        )
        fixture.last_graph_route_plan = prepared
        fixture._retain_graph_route_plan_lease(prepared, "test_wait")
        replacement = GraphRoutePlan(
            PLAN_READY,
            ACTION_PROBE_PORTAL,
            current_place_id=1,
            obligation_kind="portal_probe",
            obligation_id=7,
            portal_path=(7,),
            first_portal_id=7,
            portal_probe_phase="destination",
        )
        calls = []

        def replan(*_args, **_kwargs):
            calls.append((_args, _kwargs))
            return replacement

        fixture.graph_route_planner.plan = replan
        replanned, selected = fixture._prepare_graph_route_action(
            SimpleNamespace(), reuse_existing_plan=True,
        )

        self.assertIs(replanned, replacement)
        self.assertIsNone(selected)
        self.assertTrue(calls)
        self.assertEqual(replanned.portal_probe_phase, "destination")
        self.assertFalse(fixture.graph_route_plan_lease_active)

    def test_waiting_local_lease_replans_when_snapshot_materializes_another_item(self):
        """A candidate identity change is evidence to replace a stale lease."""
        fixture = AdapterFixture(
            work_items=[
                {
                    "id": 31,
                    "place_id": 1,
                    "state": "unresolved",
                    "active_attempt_id": None,
                },
            ],
        )
        prepared = GraphRoutePlan(
            PLAN_READY,
            "observe_local_work",
            current_place_id=1,
            target_place_id=1,
            obligation_kind="work_item",
            obligation_id=31,
        )
        fixture.last_graph_route_plan = prepared
        fixture._retain_graph_route_plan_lease(prepared, "test_wait")
        replacement = GraphRoutePlan(
            PLAN_READY,
            "observe_local_work",
            current_place_id=1,
            target_place_id=1,
            obligation_kind="work_item",
            obligation_id=32,
        )
        calls = []

        def replan(*_args, **_kwargs):
            calls.append((_args, _kwargs))
            return replacement

        fixture.graph_route_planner.plan = replan
        replanned, selected = fixture._prepare_graph_route_action(
            SimpleNamespace(),
            reuse_existing_plan=True,
            visible_work_item_ids=(32,),
            visible_probe_ids=(),
        )

        self.assertIs(replanned, replacement)
        self.assertIsNone(selected)
        self.assertTrue(calls)
        self.assertFalse(fixture.graph_route_plan_lease_active)

    def test_waiting_graph_lease_is_released_after_place_transition(self):
        fixture = AdapterFixture(
            probes=[
                {
                    "id": 7,
                    "source_place_id": 1,
                    "state": "pending",
                    "work_item_id": None,
                },
            ],
        )
        prepared, _unused = fixture._prepare_graph_route_action(SimpleNamespace())
        fixture._retain_graph_route_plan_lease(prepared, "test_wait")
        fixture.current_physical_place_id = 2

        replanned, selected = fixture._prepare_graph_route_action(
            SimpleNamespace(), reuse_existing_plan=True,
        )

        self.assertIsNot(replanned, prepared)
        self.assertIsNone(selected)
        self.assertFalse(fixture.graph_route_plan_lease_active)
        self.assertEqual(replanned.current_place_id, 2)

    def test_cleared_plan_is_not_reused_after_place_transition(self):
        """Physical Place identity invalidates a plan even without an active lease."""
        fixture = AdapterFixture(
            portals=[
                {
                    "id": 11,
                    "source_place_id": 1,
                    "destination_place_id": 2,
                    "state": "crossed",
                },
            ],
        )
        prepared = GraphRoutePlan(
            PLAN_READY,
            ACTION_CROSS_PORTAL,
            current_place_id=1,
            target_place_id=2,
            obligation_kind="unobserved_place",
            portal_path=(11,),
            first_portal_id=11,
        )
        fixture.last_graph_route_plan = prepared
        # Portal arrival clears the lease before successor planning.  The
        # physical identity still changed, so the old source-place plan is
        # invalid regardless of lease state.
        fixture.graph_route_plan_lease_active = False
        fixture.current_physical_place_id = 2

        replanned, selected = fixture._prepare_graph_route_action(
            SimpleNamespace(), reuse_existing_plan=True,
        )

        self.assertIsNot(replanned, prepared)
        self.assertEqual(replanned.current_place_id, 2)
        self.assertIsNone(selected)


    def test_unavailable_ready_plan_publishes_controller_release_identity(self):
        fixture = AdapterFixture()
        plan = GraphRoutePlan(
            PLAN_READY,
            ACTION_CROSS_PORTAL,
            current_place_id=1,
            target_place_id=2,
            obligation_kind="unobserved_place",
            portal_path=(11,),
            first_portal_id=11,
        )

        self.assertTrue(
            fixture._publish_graph_route_unavailable(
                plan, "durable_identity_not_in_current_frontier_snapshot"
            )
        )
        event, fields = fixture.events[-1]
        self.assertEqual(event, "frontier_route_unavailable")
        self.assertEqual(fields["route_id"], 17)
        self.assertEqual(fields["controller_lease"], "release")
        self.assertTrue(fields["route_unavailable"])

    def test_post_terminal_unavailable_plan_releases_stale_controller_lease(self):
        fixture = AdapterFixture()
        fixture.active_frontier = None
        plan = GraphRoutePlan(
            PLAN_READY,
            ACTION_CROSS_PORTAL,
            current_place_id=1,
            target_place_id=2,
            obligation_kind="unobserved_place",
            portal_path=(11,),
            first_portal_id=11,
        )

        self.assertTrue(
            fixture._publish_graph_route_unavailable(
                plan, "durable_identity_not_in_current_frontier_snapshot"
            )
        )
        event, fields = fixture.events[-1]
        self.assertEqual(event, "frontier_route_unavailable")
        self.assertEqual(fields["route_id"], 17)
        self.assertEqual(fields["controller_lease"], "release")

    def test_route_without_controller_identity_does_not_emit_release(self):
        fixture = AdapterFixture()
        fixture.active_frontier = None
        fixture.active_route_id = 0
        plan = GraphRoutePlan(
            PLAN_READY,
            ACTION_CROSS_PORTAL,
            current_place_id=1,
            target_place_id=2,
            obligation_kind="unobserved_place",
            portal_path=(11,),
            first_portal_id=11,
        )

        self.assertFalse(
            fixture._publish_graph_route_unavailable(
                plan, "durable_identity_not_in_current_frontier_snapshot"
            )
        )
        self.assertEqual(fixture.events, [])


class GraphRouteSelectionFixture(GlobalFrontierPlanningSelectionMixin):
    def __init__(self, plan):
        self.plan = plan
        self.graph_route_planner_enabled = True
        self.place_entry_rehydration_pending = None
        self.generic_fallback_called = False
        self.events = []
        self.portal_transaction = None
        self.pending_local_egress = None
        self.pending_portal_retry = None
        self.prefetched_frontier = None
        self.frontier_validation_budget_exhausted = False
        self.frontier_validation_pending = False
        self.target_region_claim_active = False

    def select_pending_local_egress(self, _snapshot):
        return None, None, False

    def select_pending_portal_retry(self, _snapshot):
        return None, None, False

    def terminal_prefetch_novelty_decision(self, _snapshot):
        return None, None, False

    def promote_prefetched_frontier(self, *_args, **_kwargs):
        return None

    def _prepare_graph_route_action(self, _snapshot, **_kwargs):
        return self.plan, None

    def _choose_frontier_for_new_action(self, *_args, **_kwargs):
        # The graph-aware selector returns no candidate while its exact edge
        # is not materializable.  A separate gate test covers rejection of a
        # deliberately unrelated route.
        return None

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


class GraphRoutePendingValidationFixture(GraphRouteSelectionFixture):
    """Selection fixture that records lease/release decisions during Navfn wait."""

    def __init__(self, plan):
        super().__init__(plan)
        self.frontier_validation_pending = False
        self.unavailable_calls = []
        self.retained_plans = []

    def _choose_frontier_for_new_action(self, *_args, **_kwargs):
        # Mirror choose_valid_frontier: the asynchronous Navfn request marks
        # the validation as pending only after the graph branch invokes it.
        self.frontier_validation_pending = True
        return None

    def _publish_graph_route_unavailable(self, plan, reason):
        self.unavailable_calls.append((plan, reason))
        return True

    def _retain_graph_route_plan_lease(self, plan, reason):
        self.retained_plans.append((plan, reason))
        return True


class GraphRouteMaterializationTest(unittest.TestCase):
    @staticmethod
    def snapshot():
        return SimpleNamespace(
            message=SimpleNamespace(info=SimpleNamespace(resolution=0.1)),
            map_context=SimpleNamespace(),
            route_graph=SimpleNamespace(route_steps=None, validation=None),
            robot_map=(0.0, 0.0),
            robot_yaw_map=0.0,
            now=0.0,
        )

    def test_ready_cross_plan_waits_instead_of_generic_frontier(self):
        plan = GraphRoutePlan(
            PLAN_READY,
            ACTION_CROSS_PORTAL,
            current_place_id=1,
            target_place_id=2,
            obligation_kind="unobserved_place",
            obligation_id=None,
            portal_path=(11,),
            first_portal_id=11,
            reason="transit_to_pending_graph_obligation",
        )
        fixture = GraphRouteSelectionFixture(plan)

        candidate, mode, wait = fixture.select_next_active_frontier(
            self.snapshot()
        )

        self.assertIsNone(candidate)
        self.assertEqual(mode, "graph_route_materialization_wait")
        self.assertTrue(wait)
        self.assertFalse(fixture.generic_fallback_called)
        self.assertEqual(
            fixture.events[-1][0], "graph_route_materialization_wait"
        )

    def test_ready_probe_plan_has_the_same_no_substitution_contract(self):
        plan = GraphRoutePlan(
            PLAN_READY,
            ACTION_PROBE_PORTAL,
            current_place_id=1,
            obligation_kind="portal_probe",
            obligation_id=7,
            portal_path=(7,),
            first_portal_id=7,
            reason="portal_probe_pending",
        )
        fixture = GraphRouteSelectionFixture(plan)

        candidate, mode, wait = fixture.select_next_active_frontier(
            self.snapshot()
        )

        self.assertIsNone(candidate)
        self.assertEqual(mode, "graph_route_materialization_wait")
        self.assertTrue(wait)
        self.assertFalse(fixture.generic_fallback_called)

    def test_navfn_pending_keeps_work_item_lease_without_releasing_route(self):
        """A pending exact endpoint is not a graph materialization failure."""
        plan = GraphRoutePlan(
            PLAN_READY,
            "observe_local_work",
            current_place_id=3,
            target_place_id=3,
            obligation_kind="work_item",
            obligation_id=67,
            reason="local_work_pending",
        )
        fixture = GraphRoutePendingValidationFixture(plan)
        fixture.last_graph_route_plan = plan

        candidate, mode, wait = fixture.select_next_active_frontier(
            self.snapshot()
        )

        self.assertIsNone(candidate)
        self.assertEqual(mode, "graph_route_materialization_wait")
        self.assertTrue(wait)
        self.assertEqual(fixture.retained_plans, [(plan, "navfn_validation_pending")])
        self.assertEqual(fixture.unavailable_calls, [])
        self.assertEqual(fixture.events[-1][0], "graph_route_materialization_wait")
        self.assertEqual(
            fixture.events[-1][1]["reason"], "navfn_validation_pending"
        )

    def test_work_item_67_candidate_passes_gate_and_transaction(self):
        """The route-5 WorkItem identity is valid before Navfn admission."""
        route = (
            1, 2, 6.75, 14.55, 4.0, 0.0, 0.0, 0.0, None, 0,
            "frontier_endpoint", None, 67, "rehydrated_work_item", 485,
            None, False, "observe_local_work",
            "durable_work_item_rehydrated_normal",
        )
        candidate = candidate_from_route(
            route,
            region_tier="revisit",
            ranking_category="local",
            map_epoch=56,
            place_id=3,
        )
        plan = GraphRoutePlan(
            PLAN_READY,
            "observe_local_work",
            current_place_id=3,
            target_place_id=3,
            obligation_kind="work_item",
            obligation_id=67,
            reason="local_work_pending",
        )

        self.assertTrue(candidate_refines_graph_plan(plan, candidate))
        materialization = materialize_graph_route_action(plan, candidate)
        self.assertTrue(materialization.accepted)
        self.assertEqual(materialization.candidate_work_item_id, 67)
        self.assertEqual(materialization.plan.obligation_id, 67)

    def test_place_entry_waits_for_rehydration_before_graph_transit(self):
        """A newly entered Place cannot use an empty pre-reconciliation ledger."""
        plan = GraphRoutePlan(
            PLAN_READY,
            ACTION_CROSS_PORTAL,
            current_place_id=2,
            target_place_id=1,
            obligation_kind="portal_edge",
            first_portal_id=11,
        )
        fixture = GraphRouteSelectionFixture(plan)
        fixture.place_entry_rehydration_pending = 2

        candidate, mode, wait = fixture.select_next_active_frontier(
            self.snapshot()
        )

        self.assertIsNone(candidate)
        self.assertEqual(mode, "place_entry_rehydration_wait")
        self.assertTrue(wait)
        self.assertEqual(
            fixture.events[-1][0], "place_entry_rehydration_wait"
        )


class GraphRouteGateTest(unittest.TestCase):
    def test_local_plan_matches_only_the_selected_work_item(self):
        plan = GraphRoutePlan(
            PLAN_READY,
            "observe_local_work",
            current_place_id=1,
            obligation_kind="work_item",
            obligation_id=17,
        )
        matching = (1, 2, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, None, 0,
                    "frontier_endpoint", None, 17)
        unrelated = matching[:12] + (18,)

        self.assertTrue(graph_plan_is_exclusive(plan))
        self.assertTrue(candidate_matches_graph_plan(plan, matching))
        self.assertFalse(candidate_matches_graph_plan(plan, unrelated))

    def test_probe_plan_matches_the_selected_probe_identity(self):
        plan = GraphRoutePlan(
            PLAN_READY,
            ACTION_PROBE_PORTAL,
            current_place_id=1,
            obligation_kind="portal_probe",
            obligation_id=7,
            first_portal_id=7,
        )
        probe = SimpleNamespace(probe_id=7)
        matching = (1, 2, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, None, 0,
                    "frontier_endpoint", None, None, None, 0, probe)
        unrelated = matching[:15] + (SimpleNamespace(probe_id=8),)

        self.assertTrue(candidate_matches_graph_plan(plan, matching))
        self.assertFalse(candidate_matches_graph_plan(plan, unrelated))

    def test_probe_plan_rejects_generic_endpoint_without_probe_identity(self):
        plan = GraphRoutePlan(
            PLAN_READY,
            ACTION_PROBE_PORTAL,
            current_place_id=1,
            obligation_kind="portal_probe",
            obligation_id=7,
            first_portal_id=7,
        )
        generic_endpoint = (
            1, 2, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, None, 0,
            "frontier_endpoint", None, None, None, 0, None,
        )

        self.assertFalse(
            candidate_matches_graph_plan(plan, generic_endpoint)
        )
        self.assertFalse(
            candidate_refines_graph_plan(plan, generic_endpoint)
        )

    def test_cross_plan_requires_portal_transition_route(self):
        plan = GraphRoutePlan(
            PLAN_READY,
            ACTION_CROSS_PORTAL,
            current_place_id=1,
            target_place_id=2,
            obligation_kind="unobserved_place",
            first_portal_id=11,
        )
        portal_route = (1, 2, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, None, 1,
                        "portal_transition")
        local_route = portal_route[:10] + ("frontier_endpoint",)

        self.assertTrue(candidate_matches_graph_plan(plan, portal_route))
        self.assertFalse(candidate_matches_graph_plan(plan, local_route))


if __name__ == "__main__":
    unittest.main()
