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


def graph_snapshot(epoch=1):
    return SimpleNamespace(
        map_context=SimpleNamespace(components=SimpleNamespace(epoch=epoch)),
        route_graph=SimpleNamespace(validation=None),
    )


class AdapterFixture(GlobalFrontierGraphRouteAdapterMixin):
    def __init__(self, portals=(), probes=(), work_items=()):
        self.graph_route_planner_enabled = True
        self.graph_route_planner = __import__(
            "global_frontier_graph_route_planner",
            fromlist=["GraphRoutePlanner"],
        ).GraphRoutePlanner()
        self.current_physical_place_id = 1
        self.last_graph_snapshot_projection_complete = True
        self.last_graph_snapshot_epoch = 1
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
    def test_probe_candidate_work_lineage_is_not_a_local_wi_projection(self):
        from global_frontier_selection_planner import GlobalFrontierSelectionPlannerMixin
        route=[None]*20;route[12]=18;route[17]="probe_portal"
        probe=SimpleNamespace(work_item_id=18,portal_id=22,route=tuple(route))
        local_route=list(route);local_route[17]="observe_local_work"
        local=SimpleNamespace(work_item_id=19,portal_id=None,route=tuple(local_route))
        work,probes,portals=GlobalFrontierSelectionPlannerMixin._graph_snapshot_identity_sets([probe,local])
        self.assertEqual(work,{19})
        self.assertEqual(probes,{22})

    def test_reverse_portal_retry_commits_monotonic_graph_identity(self):
        from global_frontier_portal_transaction import PortalTransaction
        fixture=AdapterFixture(portals=[dict(id=5,source_place_id=2,
            destination_place_id=3,state="crossed")])
        fixture.current_physical_place_id=3
        fixture.last_portal_hypothesis_id=5
        fixture.last_released_route_graph_transaction_id=109
        fixture.portal_transaction=PortalTransaction()
        fixture.portal_transaction.start(53,portal_id=5,source_place_id=3,
            gate_xy=(-2.,0.),destination_xy=(0.,0.),
            source_side_proven=True,source_signed_distance=-12.)
        candidate=(1,2,0.,0.,10.,0.,0.,0.,None,1,"portal_transition",(-2.,0.))
        result=fixture._commit_portal_recovery_retry(graph_snapshot(426),candidate)
        self.assertIsNotNone(result)
        self.assertEqual(result[19],5)
        transaction=fixture.graph_route_action_transaction
        self.assertEqual(transaction.transaction_id,110)
        self.assertEqual(transaction.phase,"committed")
        self.assertEqual(transaction.map_epoch,426)
        self.assertEqual(transaction.plan.target_place_id,2)
        self.assertEqual(transaction.plan.current_place_id,3)
        self.assertEqual(transaction.plan.first_portal_id,5)
        fixture.last_portal_hypothesis_id=6
        self.assertIsNone(fixture._commit_portal_recovery_retry(graph_snapshot(427),candidate))

    def test_invalid_scope_wait_is_bounded_across_committed_map_plans(self):
        from unittest.mock import patch
        from global_frontier_place_dfs import PlaceDfsTraversal
        fixture=AdapterFixture(portals=[dict(id=10,source_place_id=1,
            destination_place_id=2,state='crossed',gp_authentication_state='completed')])
        fixture.current_physical_place_id=2
        fixture.current_place_scope_id=2;fixture.current_place_scope_valid=False
        fixture.active_semantic_room=dict(source_place_id=1,place_id=2,portal_id=10)
        fixture.place_dfs=PlaceDfsTraversal();fixture.place_dfs.enter(1,2,10)
        with patch('global_frontier_graph_route_adapter.time.monotonic',side_effect=[0.,31.]):
            fixture._plan_durable_graph_route(map_epoch=1)
            plan=fixture._plan_durable_graph_route(map_epoch=2)
        self.assertEqual(plan.first_portal_id,10)
        self.assertEqual(plan.reason,'dfs_invalid_projection_recovery_return')
        self.assertEqual(fixture.place_dfs.nodes[2]['state'],'recovery_returning')
        self.assertIn('place_scope_recovery_started',[name for name,_ in fixture.events])

    def test_tsp_inputs_keep_physical_frame_despite_slam_reprojection(self):
        fixture = AdapterFixture(work_items=[
            {"id": 51, "place_id": 1, "state": "unresolved",
             "anchor_xy": (2.0, 3.0), "normal_xy": (1.0, 0.0)},
        ])
        fixture.pose_odom = SimpleNamespace(x=1.0, y=2.0, theta=0.0)
        fixture.map_msg = SimpleNamespace(
            header=SimpleNamespace(frame_id="map")
        )
        fixture.planar_xy_projector = lambda _target, _source: (
            lambda x, y: (10.0 - y, 20.0 + x)
        )

        items, robot_pose = fixture._graph_route_tsp_inputs(
            fixture.place_work_items
        )

        self.assertEqual(robot_pose[:2], (1.0, 2.0))
        self.assertAlmostEqual(robot_pose[2], 0.0)
        self.assertEqual(items[0]["anchor_xy"], (2.0, 3.0))
        self.assertEqual(items[0]["normal_xy"], (1.0, 0.0))
        self.assertEqual(fixture.place_work_items.records[0]["anchor_xy"], (2.0, 3.0))

    def test_slam_projection_cannot_move_sibling_work_into_room_tsp(self):
        from global_frontier_graph_route_planner import GraphRoutePlanner
        fixture = AdapterFixture(work_items=[
            {"id": 51, "place_id": 1, "state": "unresolved", "anchor_xy": (2.0, 3.0)},
            {"id": 52, "place_id": 1, "state": "unresolved", "anchor_xy": (-3.0, 2.0)},
        ])
        fixture.pose_odom = SimpleNamespace(x=1.0, y=2.0, theta=0.0)
        fixture.map_msg = SimpleNamespace(header=SimpleNamespace(frame_id="map"))
        fixture.planar_xy_projector = lambda _target, _source: (
            lambda x, y: (10.0 - y, 20.0 + x))
        items, pose = fixture._graph_route_tsp_inputs(fixture.place_work_items)
        plan = GraphRoutePlanner().plan(
            1, places=[{"id": 1, "state": "open", "endpoint_observations": 1}],
            portals=(), work_items=items, robot_pose=pose,
            room_work_item_only=True,
            room_branch_scope={"origin_xy": [1.0, 2.0], "selected_branch_id": 1,
                               "directions": [{"branch_id": 1, "direction_xy": [1.0, 0.0]},
                                              {"branch_id": 2, "direction_xy": [-1.0, 0.0]}]},
        )
        self.assertEqual(plan.obligation_id, 51)
        self.assertEqual(plan.work_item_tour, (51,))

    def test_only_certified_reverse_edge_uses_durable_egress(self):
        fixture = AdapterFixture(portals=[{
            "id": 11, "source_place_id": 1, "destination_place_id": 2,
            "state": "crossed",
        }])
        fixture.current_physical_place_id = 2
        plan = GraphRoutePlan(PLAN_READY, ACTION_CROSS_PORTAL,
                              current_place_id=2, target_place_id=1,
                              obligation_kind="portal_edge", obligation_id=11,
                              first_portal_id=11, portal_path=(11,))
        self.assertTrue(fixture._graph_plan_requires_durable_egress(plan))
        fixture.portal_hypothesis_ledger.records[0]["state"] = "certified"
        self.assertFalse(fixture._graph_plan_requires_durable_egress(plan))
        fixture.portal_hypothesis_ledger.records[0]["state"] = "crossed"
        fixture.current_physical_place_id = 1
        self.assertFalse(fixture._graph_plan_requires_durable_egress(plan))

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
            portal_probe_phase="source",
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

    def test_semantic_room_scope_covers_descendant_places_until_junction_return(self):
        fixture = AdapterFixture()
        scope = {
            "selected_branch_id": 8,
            "origin_xy": [0.0, 0.0],
            "directions": [
                {"branch_id": 8, "direction_xy": [0.0, 1.0]},
            ],
        }
        fixture.active_semantic_room = {
            "branch_id": 8,
            "place_id": 3,
            "source_place_id": 1,
            "branch_scope": scope,
        }
        fixture.current_physical_place_id = 6
        captured = {}

        def plan(*_args, **kwargs):
            captured.update(kwargs)
            return GraphRoutePlan(
                PLAN_BLOCKED, "hold", current_place_id=6
            )

        fixture.graph_route_planner.plan = plan
        fixture._plan_durable_graph_route(map_epoch=1, commit=False)

        self.assertTrue(captured["room_work_item_only"])
        self.assertIsNone(captured["room_branch_scope"])

        fixture.current_physical_place_id = 1
        captured.clear()
        fixture._plan_durable_graph_route(map_epoch=1, commit=False)
        self.assertFalse(captured["room_work_item_only"])

    def test_suspended_work_item_identity_reaches_planner_after_room_cycles(self):
        fixture = AdapterFixture(work_items=[{
            "id": 31, "place_id": 1, "state": "unresolved",
            "active_attempt_id": None,
        }])
        fixture.suspended_work_item_id = 31
        captured = {}
        fixture.graph_route_planner.plan = lambda *_args, **kwargs: (
            captured.update(kwargs)
            or GraphRoutePlan(PLAN_BLOCKED, "hold", current_place_id=1)
        )

        fixture._plan_durable_graph_route(map_epoch=1, commit=False)

        self.assertEqual(captured["resume_work_item_id"], 31)

    def test_suspended_parent_work_item_is_not_resumed_inside_active_room(self):
        fixture = AdapterFixture(work_items=[{
            "id": 31, "place_id": 1, "state": "unresolved",
            "active_attempt_id": None,
        }])
        fixture.suspended_work_item_id = 31
        fixture.current_physical_place_id = 2
        fixture.active_semantic_room = {
            "place_id": 2, "source_place_id": 1,
            "branch_scope": {"selected_branch_id": 8},
        }
        captured = {}
        fixture.graph_route_planner.plan = lambda *_args, **kwargs: (
            captured.update(kwargs)
            or GraphRoutePlan(PLAN_BLOCKED, "hold", current_place_id=2)
        )

        fixture._plan_durable_graph_route(map_epoch=1, commit=False)

        self.assertIsNone(captured["resume_work_item_id"])
        self.assertTrue(captured["room_work_item_only"])
        self.assertEqual(fixture.suspended_work_item_id, 31)

    def test_resolved_suspended_work_item_clears_stale_resume_marker(self):
        fixture = AdapterFixture(work_items=[{
            "id": 31, "place_id": 1, "state": "resolved",
            "active_attempt_id": None,
        }])
        fixture.suspended_work_item_id = 31
        captured = {}
        fixture.graph_route_planner.plan = lambda *_args, **kwargs: (
            captured.update(kwargs)
            or GraphRoutePlan(PLAN_BLOCKED, "hold", current_place_id=1)
        )

        fixture._plan_durable_graph_route(map_epoch=1, commit=False)

        self.assertIsNone(captured["resume_work_item_id"])
        self.assertIsNone(fixture.suspended_work_item_id)
        self.assertEqual(fixture.events[-1][0], "work_item_resume_cleared")

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

        plan, candidate = fixture._prepare_graph_route_action(graph_snapshot())

        self.assertIsNone(candidate)
        self.assertEqual(plan.portal_path, (11, 12))
        self.assertEqual(fixture.graph_route_portal_id, 11)
        self.assertEqual(fixture.events[-1][0], "graph_route_plan_selected")
        self.assertEqual(fixture.events[-1][1]["first_portal_id"], 11)

    def test_bound_forward_portal_uses_durable_crossing_materializer(self):
        fixture = AdapterFixture(
            portals=[
                {"id": 11, "source_place_id": 1, "destination_place_id": 2, "state": "crossed"},
                {"id": 12, "source_place_id": 2, "destination_place_id": 3, "state": "crossed"},
            ],
            work_items=[
                {"id": 31, "place_id": 3, "state": "unresolved", "active_attempt_id": None},
            ],
        )
        fixture._graph_snapshot_projection_is_current = lambda _epoch: True
        candidate = (1, 2, 3.0, 4.0, 1.2, 0.0, 0.0, 0.0, None, 0, "portal_transition")
        attempts = []
        fixture.select_durable_portal_crossing = lambda snapshot: (
            attempts.append(snapshot) or candidate
        )
        fixture._commit_graph_route_candidate = lambda *_args, **_kwargs: True
        fixture._publish_graph_route_materialized = lambda *_args, **_kwargs: None
        snapshot = SimpleNamespace(
            map_context=SimpleNamespace(components=SimpleNamespace(epoch=1)),
            route_graph=SimpleNamespace(validation=None),
        )

        plan, selected = fixture._prepare_graph_route_action(
            snapshot,
            visible_work_item_ids=(31,),
            visible_portal_ids=(11, 12),
        )

        self.assertEqual(plan.action, ACTION_CROSS_PORTAL)
        self.assertEqual(plan.first_portal_id, 11)
        self.assertIs(selected, candidate)
        self.assertEqual(attempts, [snapshot])
        self.assertIsNone(fixture.graph_route_portal_id)

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

        plan, selected = fixture._prepare_graph_route_action(graph_snapshot())

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

        plan, selected = fixture._prepare_graph_route_action(graph_snapshot())

        self.assertEqual(plan.action, ACTION_CROSS_PORTAL)
        self.assertIs(selected, candidate)
        self.assertTrue(fixture.graph_route_plan_lease_active)
        self.assertEqual(fixture.events[-1][0], "graph_route_edge_materialized")

    def test_unavailable_crossing_waits_for_a_new_map_epoch(self):
        """A same-epoch geometry failure preserves the durable Portal action."""
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
        fixture.last_graph_snapshot_epoch = 7
        snapshot = SimpleNamespace(
            map_context=SimpleNamespace(components=SimpleNamespace(epoch=7)),
            route_graph=SimpleNamespace(validation=None),
        )

        plan, candidate = fixture._prepare_graph_route_action(snapshot)

        self.assertIsNone(candidate)
        self.assertEqual(plan.action, ACTION_CROSS_PORTAL)
        self.assertEqual(plan.first_portal_id, 11)
        self.assertTrue(fixture.graph_route_plan_lease_active)
        self.assertEqual(fixture.graph_route_portal_id, 11)
        self.assertEqual(
            fixture.portal_hypothesis_ledger.get(11)["rejected_map_epoch"],
            7,
        )
        self.assertEqual(
            fixture.events[-1][0], "graph_route_materialization_retry_deferred"
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
        prepared = GraphRoutePlan(
            prepared.status,
            prepared.action,
            map_epoch=1,
            current_place_id=prepared.current_place_id,
            target_place_id=prepared.target_place_id,
            obligation_kind=prepared.obligation_kind,
            portal_path=prepared.portal_path,
            first_portal_id=prepared.first_portal_id,
        )
        fixture.last_graph_route_plan = prepared
        fixture.last_graph_snapshot_visible_portal_ids = (11,)
        fixture._retain_graph_route_plan_lease(prepared, "test_wait")
        fixture.graph_route_planner.plan = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("terminal must not replan the prepared action")
        )

        plan, selected = fixture._prepare_graph_route_action(
            graph_snapshot(), reuse_existing_plan=True,
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
        prepared, _unused = fixture._prepare_graph_route_action(graph_snapshot())
        fixture._retain_graph_route_plan_lease(prepared, "test_wait")

        def planner_must_not_replan(*_args, **_kwargs):
            raise AssertionError("a waiting graph lease must survive map snapshots")

        fixture.graph_route_planner.plan = planner_must_not_replan
        reused, selected = fixture._prepare_graph_route_action(
            graph_snapshot(),
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
            graph_snapshot(), reuse_existing_plan=True,
        )

        self.assertEqual(replanned.portal_probe_phase, "destination")
        self.assertEqual(replanned.map_epoch, 1)
        self.assertIsNone(selected)
        self.assertTrue(calls)
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
            graph_snapshot(),
            reuse_existing_plan=True,
            visible_work_item_ids=(32,),
            visible_probe_ids=(),
        )

        self.assertEqual(replanned.obligation_id, replacement.obligation_id)
        self.assertEqual(replanned.map_epoch, 1)
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
        prepared, _unused = fixture._prepare_graph_route_action(graph_snapshot())
        fixture._retain_graph_route_plan_lease(prepared, "test_wait")
        fixture.current_physical_place_id = 2

        replanned, selected = fixture._prepare_graph_route_action(
            graph_snapshot(), reuse_existing_plan=True,
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
            graph_snapshot(), reuse_existing_plan=True,
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

    def test_authentication_can_preempt_only_an_unstarted_portal_reservation(self):
        from global_frontier_portal_transaction import PortalTransaction
        for started, crossing_observed in ((False, False), (True, False), (False, True)):
            fixture = GraphRouteSelectionFixture(None)
            fixture.portal_authentication_owns_navigation = lambda: True
            fixture.active_portal_crossing_observed = crossing_observed
            tx = PortalTransaction()
            tx.start(22, portal_id=11, source_place_id=1,
                     gate_xy=(1, 0), destination_xy=(2, 0))
            if not started:
                tx.release_unstarted_route(22)
            fixture.portal_transaction = tx
            gp_candidate = ("observation_candidate",)
            fixture._choose_frontier_for_new_action = lambda *_args, **_kwargs: gp_candidate
            candidate, mode, wait = fixture.select_next_active_frontier(self.snapshot())
            if not started and not crossing_observed:
                self.assertEqual(candidate, gp_candidate)
                self.assertEqual(mode, "gp_portal_authentication")
                self.assertFalse(wait)
                self.assertFalse(tx.active)
                self.assertEqual(tx.state, "idle")
                self.assertEqual(tx.snapshot().portal_id, 11)
                self.assertFalse(fixture.portal_transaction_route_rebind_pending)
            else:
                self.assertIsNone(candidate)
                self.assertEqual(mode, "portal_transaction_wait")
                self.assertTrue(wait)
                self.assertTrue(tx.active)

    def test_normal_exploration_cannot_preempt_unstarted_portal_reservation(self):
        from global_frontier_portal_transaction import PortalTransaction
        fixture = GraphRouteSelectionFixture(None)
        fixture.portal_authentication_owns_navigation = lambda: False
        tx = PortalTransaction()
        tx.start(22, portal_id=11, source_place_id=1)
        tx.release_unstarted_route(22)
        fixture.portal_transaction = tx
        candidate, mode, wait = fixture.select_next_active_frontier(self.snapshot())
        self.assertIsNone(candidate)
        self.assertEqual(mode, "portal_transaction_wait")
        self.assertTrue(wait)
        self.assertTrue(tx.active)

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
        self.assertEqual(mode, "graph_route_rehydration_wait")
        self.assertTrue(wait)
        self.assertFalse(fixture.generic_fallback_called)
        self.assertEqual(
            fixture.events[-1][0], "graph_route_rehydration_wait"
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
        self.assertEqual(mode, "graph_route_rehydration_wait")
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
        fixture._graph_snapshot_projection_is_current = lambda _epoch: True
        fixture._graph_snapshot_projection_ids = lambda: ({67}, set(), set())

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

    def test_semantic_room_work_item_projection_failure_retries_on_next_epoch(self):
        fixture = AdapterFixture(work_items=[
            {"id": 20, "place_id": 6, "state": "unresolved"},
        ])
        fixture.active_semantic_room = {
            "place_id": 6, "source_place_id": 1, "branch_id": 7,
        }
        plan = GraphRoutePlan(
            PLAN_READY,
            "observe_local_work",
            current_place_id=6,
            target_place_id=6,
            obligation_kind="work_item",
            obligation_id=20,
        )

        self.assertIsNone(
            fixture._remember_graph_route_materialization_invalidation(plan)
        )
        negative = fixture._record_graph_route_negative_evidence(
            plan, map_epoch=370, reason="graph_route_materialization_expired",
            miss_count=3, epoch_span=2,
        )
        self.assertTrue(negative["work_item_negative_evidence"]["excluded_in_map_epoch"])
        self.assertEqual(fixture._graph_route_materialization_exclusions(370)[1], (20,))
        self.assertEqual(fixture._graph_route_materialization_exclusions(371)[1], ())
        self.assertEqual(
            getattr(fixture, "graph_route_materialization_invalidated_obligations", set()),
            set(),
        )

    def test_non_room_work_item_materialization_failure_remains_run_scoped(self):
        fixture = AdapterFixture(work_items=[
            {"id": 20, "place_id": 6, "state": "unresolved"},
        ])
        plan = GraphRoutePlan(
            PLAN_READY,
            "observe_local_work",
            current_place_id=6,
            target_place_id=6,
            obligation_kind="work_item",
            obligation_id=20,
        )

        key = fixture._remember_graph_route_materialization_invalidation(plan)

        self.assertEqual(key, ("obligation", "work_item", 20))
        self.assertEqual(fixture._graph_route_materialization_exclusions(371)[1], (20,))

    def test_fresh_projection_reopens_parked_wi_but_not_same_epoch_or_portal(self):
        fixture = AdapterFixture()
        work_plan = GraphRoutePlan(
            PLAN_READY, "observe_local_work", current_place_id=6,
            obligation_kind="work_item", obligation_id=20,
        )
        portal_plan = GraphRoutePlan(
            PLAN_READY, ACTION_CROSS_PORTAL, current_place_id=6,
            obligation_kind="portal_edge", obligation_id=11, first_portal_id=11,
        )
        fixture._remember_graph_route_materialization_invalidation(work_plan)
        fixture._remember_graph_route_materialization_invalidation(portal_plan)
        fixture._remember_graph_route_materialization_exclusion(370, "work_item_ids", 20)
        fixture._reopen_projected_work_items(370, (20,))
        self.assertEqual(fixture._graph_route_materialization_exclusions(370)[1], (20,))
        fixture._reopen_projected_work_items(371, ())
        self.assertEqual(fixture._graph_route_materialization_exclusions(371)[1], (20,))
        fixture._reopen_projected_work_items(371, (20,))
        excluded_portals, excluded_work, _ = fixture._graph_route_materialization_exclusions(371)
        self.assertEqual(excluded_work, ())
        self.assertIn(11, excluded_portals)

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


class PortalObligationCooldownTest(unittest.TestCase):
    def _portal_plan(self, portal_id=11):
        return GraphRoutePlan(
            PLAN_READY,
            ACTION_CROSS_PORTAL,
            current_place_id=6,
            obligation_kind="portal_edge",
            obligation_id=portal_id,
            first_portal_id=portal_id,
        )

    def test_exhausted_portal_is_paused_then_rearmed(self):
        fixture = AdapterFixture()
        plan = self._portal_plan()
        fixture._remember_graph_route_materialization_invalidation(
            plan, now=100.0, reason="graph_route_materialization_expired",
        )

        excluded, _, _ = fixture._graph_route_materialization_exclusions(
            371, now=120.0,
        )
        self.assertIn(11, excluded)
        self.assertEqual(
            fixture.events[-1][0],
            "graph_route_obligation_cooldown_started",
        )

        excluded, _, _ = fixture._graph_route_materialization_exclusions(
            371, now=146.0,
        )
        self.assertNotIn(11, excluded)
        self.assertEqual(
            fixture.events[-1][0],
            "graph_route_obligation_cooldown_expired",
        )
        self.assertNotIn(
            ("obligation", "portal_edge", 11),
            fixture.graph_route_materialization_invalidated_obligations,
        )

    def test_cooldown_escalates_and_caps(self):
        fixture = AdapterFixture()
        plan = self._portal_plan()
        key = ("obligation", "portal_edge", 11)
        expected = (45.0, 90.0, 180.0, 180.0)
        for index, seconds in enumerate(expected):
            fixture._remember_graph_route_materialization_invalidation(
                plan, now=float(index), reason="graph_route_materialization_expired",
            )
            self.assertEqual(
                fixture.graph_route_obligation_cooldowns[key]["seconds"],
                seconds,
            )

    def test_materialized_crossing_clears_cooldown(self):
        fixture = AdapterFixture()
        plan = self._portal_plan()
        fixture._remember_graph_route_materialization_invalidation(
            plan, now=10.0, reason="graph_route_materialization_expired",
        )
        self.assertIn(
            11, fixture._active_graph_route_obligation_cooldown_portals(now=20.0),
        )
        fixture._publish_graph_route_materialized(
            plan, (1, 2, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, None, 1,
                   "portal_transition"),
        )
        self.assertNotIn(
            11, fixture._active_graph_route_obligation_cooldown_portals(now=20.0),
        )
        excluded, _, _ = fixture._graph_route_materialization_exclusions(371, now=20.0)
        self.assertNotIn(11, excluded)

    def test_cooled_entry_item_is_hidden_from_the_planner_input(self):
        items = [
            {"id": 3, "kind": "portal_entry", "portal_id": 11},
            {"id": 4, "kind": "portal_entry", "portal_id": 12},
            {"id": 5, "kind": "frontier_observation", "portal_id": 11},
        ]
        kept = AdapterFixture._without_cooled_portal_entries(items, {11})
        self.assertEqual([item["id"] for item in kept], [4, 5])

    def test_cooled_entry_is_not_the_semantic_pending_obligation(self):
        # The planner input no longer contains the paused item; the durable
        # ledger still does, so a later plan can re-arm without recreating it.
        fixture = AdapterFixture(work_items=[
            {"id": 3, "kind": "portal_entry", "portal_id": 11,
             "place_id": 6, "state": "unresolved"},
        ])
        plan = self._portal_plan()
        fixture._remember_graph_route_materialization_invalidation(
            plan, now=1.0, reason="graph_route_materialization_expired",
        )
        kept = fixture._without_cooled_portal_entries(
            fixture.place_work_items.snapshot(),
            fixture._active_graph_route_obligation_cooldown_portals(now=20.0),
        )
        self.assertEqual(kept, [])
        self.assertEqual(
            fixture.place_work_items.get(3)["state"], "unresolved",
        )


class _BestEffortMemory:
    def __init__(self):
        self.region = {
            "id": 6, "state": "open", "endpoint_observations": 0,
        }

    def by_id(self, place_id):
        return self.region if int(place_id) == 6 else None

    def mark_observation_best_effort(self, place_id, now, reason=""):
        if int(place_id) != 6:
            return None
        self.region["endpoint_observations"] = 1
        self.region["covered_best_effort"] = True
        self.region["last_reason"] = reason
        return self.region


class _BestEffortLedger:
    def __init__(self, unresolved=0):
        self.unresolved = unresolved

    def unresolved_frontier_observation_count(self, place_id):
        return self.unresolved


class PlaceObservationBestEffortAdapterTest(unittest.TestCase):
    def _fixture(self, unresolved=0):
        fixture = AdapterFixture()
        fixture.region_memory = _BestEffortMemory()
        fixture.place_work_items = _BestEffortLedger(unresolved)
        fixture.last_graph_route_plan = GraphRoutePlan(
            PLAN_BLOCKED,
            "hold",
            current_place_id=6,
            reason="place_observation_required_before_graph_route",
        )
        fixture.place_observation_best_effort_seconds = 10.0
        return fixture

    def test_due_after_grace_then_marks_coverage(self):
        fixture = self._fixture()
        self.assertFalse(
            fixture._place_observation_best_effort_due(6, now=100.0)
        )
        self.assertTrue(
            fixture._place_observation_best_effort_due(6, now=111.0)
        )
        self.assertTrue(
            fixture._mark_place_observation_best_effort(6, 111.0)
        )
        self.assertEqual(
            fixture.region_memory.region["endpoint_observations"], 1
        )
        self.assertEqual(
            fixture.events[-1][0], "place_observation_best_effort"
        )

    def test_unresolved_observation_work_blocks_best_effort(self):
        fixture = self._fixture(unresolved=1)
        self.assertFalse(
            fixture._place_observation_best_effort_due(6, now=100.0)
        )
        self.assertFalse(
            fixture._place_observation_best_effort_due(6, now=200.0)
        )

    def test_covered_place_does_not_arm(self):
        fixture = self._fixture()
        fixture.region_memory.region["endpoint_observations"] = 1
        self.assertFalse(
            fixture._place_observation_best_effort_due(6, now=100.0)
        )
        self.assertFalse(
            fixture._place_observation_best_effort_due(6, now=200.0)
        )


if __name__ == "__main__":
    unittest.main()
