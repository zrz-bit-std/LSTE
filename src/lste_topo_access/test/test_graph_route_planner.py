"""Regression tests for durable graph-level route selection."""

from copy import deepcopy
from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_graph_route_planner import (  # noqa: E402
    ACTION_CROSS_PORTAL,
    ACTION_HOLD,
    ACTION_OBSERVE_LOCAL_WORK,
    ACTION_PROBE_PORTAL,
    ACTION_REINSPECT_TARGET,
    GraphRoutePlanner,
    PLAN_BLOCKED,
    PLAN_COMPLETE,
    PLAN_READY,
)
from global_frontier_directional_branch_coverage import (  # noqa: E402
    DirectionalBranchCoverage,
)


def place(place_id, *, covered=True, state="open"):
    return {
        "id": place_id,
        "state": state,
        "endpoint_observations": 1 if covered else 0,
    }


def portal(portal_id, source, destination, state="crossed"):
    return {
        "id": portal_id,
        "source_place_id": source,
        "destination_place_id": destination,
        "state": state,
    }


def work(item_id, place_id, active_attempt_id=None, normal_xy=None):
    item = {
        "id": item_id,
        "place_id": place_id,
        "state": "unresolved",
        "active_attempt_id": active_attempt_id,
    }
    if normal_xy is not None:
        item["normal_xy"] = normal_xy
    return item


class GraphRoutePlannerTest(unittest.TestCase):
    def setUp(self):
        self.planner = GraphRoutePlanner()

    def plan(self, current, places, portals=(), work_items=(), probes=(), **kwargs):
        return self.planner.plan(
            current,
            places=places,
            portals=portals,
            work_items=work_items,
            probes=probes,
            **kwargs,
        )

    def test_multihop_route_returns_complete_path_but_one_first_edge(self):
        result = self.plan(
            1,
            [place(1), place(2, covered=False), place(3)],
            [portal(11, 1, 2), portal(12, 2, 3)],
            work_items=[work(31, 3)],
        )

        self.assertEqual(result.status, PLAN_READY)
        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.target_place_id, 2)
        self.assertEqual(result.obligation_kind, "unobserved_place")
        self.assertEqual(result.portal_path, (11,))
        self.assertEqual(result.first_portal_id, 11)

    def test_failed_edge_is_not_used(self):
        result = self.plan(
            1,
            [place(1), place(2), place(3)],
            [portal(11, 1, 2, state="failed"), portal(12, 1, 3)],
            work_items=[work(31, 3)],
        )

        self.assertEqual(result.portal_path, (12,))
        self.assertEqual(result.target_place_id, 3)

    def test_self_loop_portal_never_enters_graph_adjacency(self):
        result = self.plan(
            1,
            [place(1), place(2)],
            [portal(99, 1, 1), portal(11, 1, 2)],
            work_items=[work(31, 2)],
        )

        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.portal_path, (11,))
        self.assertNotIn(99, result.portal_path)

    def test_unbound_portal_is_an_obligation_not_a_fake_place(self):
        result = self.plan(
            1,
            [place(1)],
            [{
                "id": 13,
                "source_place_id": 1,
                "destination_place_id": None,
                "state": "certified",
            }],
        )

        self.assertEqual(result.status, PLAN_READY)
        self.assertEqual(result.action, ACTION_PROBE_PORTAL)
        self.assertIsNone(result.target_place_id)
        self.assertEqual(result.first_portal_id, 13)

    def test_rejected_unbound_portal_retries_only_after_map_epoch_changes(self):
        """A projection rejection does not erase the physical doorway."""
        rejected_portal = {
            "id": 13,
            "source_place_id": 1,
            "destination_place_id": None,
            "state": "certified",
            "rejected_map_epoch": 7,
        }
        pending_work = work(31, 1, normal_xy=(1.0, 0.0))

        # The local WorkItem remains the executable obligation in the epoch
        # whose map projection rejected the Portal.
        local = self.plan(
            1,
            [place(1)],
            [rejected_portal],
            work_items=[pending_work],
            map_epoch=7,
        )
        self.assertEqual(local.action, ACTION_OBSERVE_LOCAL_WORK)
        self.assertEqual(local.obligation_id, 31)

        # Once the local obligation is resolved, the rejected Portal must not
        # be replayed on the same map snapshot.
        settled_same_epoch = self.plan(
            1,
            [place(1)],
            [rejected_portal],
            work_items=[dict(pending_work, state="resolved")],
            map_epoch=7,
        )
        self.assertEqual(settled_same_epoch.status, PLAN_COMPLETE)
        self.assertEqual(settled_same_epoch.action, ACTION_HOLD)

        # A newer map epoch reopens the durable Portal hypothesis for another
        # projection attempt without changing its physical identity.
        retry = self.plan(
            1,
            [place(1)],
            [rejected_portal],
            work_items=[dict(pending_work, state="resolved")],
            map_epoch=8,
        )
        self.assertEqual(retry.status, PLAN_READY)
        self.assertEqual(retry.action, ACTION_PROBE_PORTAL)
        self.assertEqual(retry.obligation_kind, "unbound_portal")
        self.assertEqual(retry.obligation_id, 13)
        self.assertEqual(retry.first_portal_id, 13)

    def test_destination_view_promotes_unbound_portal_to_crossing(self):
        """A completed two-view probe can leave through its physical edge."""
        result = self.plan(
            1,
            [place(1)],
            [{
                "id": 13,
                "source_place_id": 1,
                "destination_place_id": None,
                "state": "certified",
            }],
            work_items=[work(31, 1)],
            probes=[{
                "id": 41,
                "source_place_id": 1,
                "portal_id": 13,
                "state": "observed",
            }],
            branch_first=True,
        )

        self.assertEqual(result.status, PLAN_READY)
        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.first_portal_id, 13)
        self.assertEqual(result.obligation_kind, "portal_edge")
        self.assertEqual(result.reason, "destination_view_promoted_to_portal")

    def test_certified_portal_crossing_precedes_new_structural_probe(self):
        result = self.plan(
            1,
            [place(1)],
            [{
                "id": 13,
                "source_place_id": 1,
                "destination_place_id": None,
                "state": "certified",
            }],
            probes=[
                {
                    "id": 41,
                    "source_place_id": 1,
                    "portal_id": 13,
                    "state": "observed",
                },
                {
                    "id": 42,
                    "source_place_id": 1,
                    "state": "pending",
                    "observation_source": "structural_boundary",
                },
            ],
            branch_first=True,
            structural_boundary_first=True,
        )

        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.first_portal_id, 13)
        self.assertEqual(result.reason, "destination_view_promoted_to_portal")

    def test_bound_probe_promotes_unbound_portal_to_action_identity(self):
        """A Portal ID must never be sent to the source-side probe adapter."""
        result = self.plan(
            1,
            [place(1)],
            [{
                "id": 13,
                "source_place_id": 1,
                "destination_place_id": None,
                "state": "certified",
            }],
            probes=[{
                "id": 41,
                "source_place_id": 1,
                "portal_id": 13,
                "state": "pending",
            }],
            # The probe is intentionally absent from the current frontier
            # projection. Durable binding remains sufficient to plan it.
            visible_probe_ids=(),
        )

        self.assertEqual(result.action, ACTION_PROBE_PORTAL)
        self.assertEqual(result.obligation_kind, "portal_probe")
        self.assertEqual(result.obligation_id, 41)
        self.assertEqual(result.first_portal_id, 41)
        self.assertIn(
            result.reason,
            ("portal_probe_pending", "unbound_portal_promoted_to_source_probe"),
        )

    def test_projection_parked_probe_is_not_replayed_as_unbound_portal(self):
        """A parked physical probe blocks cleanly until new map evidence."""
        result = self.plan(
            1,
            [place(1)],
            [{
                "id": 13,
                "source_place_id": 1,
                "destination_place_id": None,
                "state": "certified",
            }],
            probes=[{
                "id": 41,
                "source_place_id": 1,
                "portal_id": 13,
                "state": "awaiting_projection",
            }],
        )

        self.assertEqual(result.status, PLAN_BLOCKED)
        self.assertEqual(result.action, ACTION_HOLD)
        self.assertEqual(
            result.reason,
            "durable_obligation_has_no_certified_graph_path",
        )

    def test_projection_parked_probe_does_not_starve_another_probe(self):
        """Only the projected doorway is skipped; other evidence can proceed."""
        result = self.plan(
            1,
            [place(1)],
            [
                {
                    "id": 13,
                    "source_place_id": 1,
                    "destination_place_id": None,
                    "state": "certified",
                },
                {
                    "id": 14,
                    "source_place_id": 1,
                    "destination_place_id": None,
                    "state": "certified",
                },
            ],
            probes=[
                {
                    "id": 41,
                    "source_place_id": 1,
                    "portal_id": 13,
                    "state": "awaiting_projection",
                },
                {
                    "id": 42,
                    "source_place_id": 1,
                    "portal_id": 14,
                    "state": "pending",
                },
            ],
        )

        self.assertEqual(result.action, ACTION_PROBE_PORTAL)
        self.assertEqual(result.obligation_id, 42)
        self.assertEqual(result.first_portal_id, 42)

    def test_branch_first_can_suspend_local_work_for_unobserved_place(self):
        data = dict(
            current_place_id=1,
            places=[place(1), place(2, covered=False)],
            portals=[portal(11, 1, 2)],
            work_items=[work(31, 1)],
        )
        branch = self.planner.plan(branch_first=True, **data)
        strict = self.planner.plan(branch_first=False, **data)

        self.assertEqual(branch.action, ACTION_CROSS_PORTAL)
        self.assertEqual(branch.reason, "branch_first_to_unobserved_place")
        self.assertEqual(strict.action, ACTION_OBSERVE_LOCAL_WORK)
        self.assertEqual(strict.obligation_id, 31)

    def test_branch_first_closes_rehydratable_local_work_before_crossing(self):
        """A durable viewpoint normal makes local WorkItem closure mandatory."""
        result = self.plan(
            1,
            [place(1), place(2, covered=False)],
            [portal(11, 1, 2)],
            work_items=[work(31, 1, normal_xy=(1.0, 0.0))],
            branch_first=True,
        )

        self.assertEqual(result.action, ACTION_OBSERVE_LOCAL_WORK)
        self.assertEqual(result.obligation_id, 31)
        self.assertEqual(result.reason, "local_work_pending")

    def test_hidden_rehydratable_local_work_blocks_branch_until_rehydrated(self):
        """A stable hidden WorkItem is held, rather than causing a later reentry."""
        result = self.plan(
            1,
            [place(1), place(2, covered=False)],
            [portal(11, 1, 2)],
            work_items=[work(31, 1, normal_xy=(1.0, 0.0))],
            branch_first=True,
            visible_work_item_ids=(99,),
        )

        self.assertEqual(result.status, PLAN_BLOCKED)
        self.assertEqual(result.action, ACTION_HOLD)
        self.assertEqual(result.reason, "current_place_obligation_not_materialized")

    def test_portal_crossing_becomes_legal_after_local_work_is_resolved(self):
        """Place closure changes the legal graph action, not a numeric score."""
        places = [place(1), place(2, covered=False)]
        portals = [portal(11, 1, 2)]
        pending = work(31, 1, normal_xy=(1.0, 0.0))

        local = self.plan(
            1, places, portals, work_items=[pending], branch_first=True,
        )
        resolved = dict(pending, state="resolved")
        crossing = self.plan(
            1, places, portals, work_items=[resolved], branch_first=True,
        )

        self.assertEqual(local.action, ACTION_OBSERVE_LOCAL_WORK)
        self.assertEqual(local.obligation_id, 31)
        self.assertEqual(crossing.action, ACTION_CROSS_PORTAL)
        self.assertEqual(crossing.portal_path, (11,))

    def test_target_place_work_cannot_be_abandoned_by_branch_first(self):
        result = self.plan(
            1,
            [place(1), place(2, covered=False)],
            [portal(11, 1, 2)],
            work_items=[work(31, 1)],
            branch_first=True,
            target_place_id=1,
        )

        self.assertEqual(result.action, ACTION_OBSERVE_LOCAL_WORK)
        self.assertEqual(result.obligation_id, 31)

    def test_target_observation_lease_blocks_graph_departure(self):
        result = self.plan(
            1,
            [place(1), place(2, covered=False)],
            [portal(11, 1, 2)],
            branch_first=True,
            target_observation_pending=True,
        )

        self.assertEqual(result.status, PLAN_READY)
        self.assertEqual(result.action, ACTION_REINSPECT_TARGET)
        self.assertEqual(result.target_place_id, 1)
        self.assertEqual(result.obligation_kind, "target_observation")

    def test_target_reinspection_precedes_local_work_and_portal_probe(self):
        result = self.plan(
            1,
            [place(1), place(2, covered=False)],
            [
                {
                    "id": 13,
                    "source_place_id": 1,
                    "destination_place_id": None,
                    "state": "certified",
                },
            ],
            work_items=[work(31, 1)],
            probes=[
                {
                    "id": 41,
                    "source_place_id": 1,
                    "state": "pending",
                },
            ],
            branch_first=True,
            target_observation_pending=True,
        )

        self.assertEqual(result.action, ACTION_REINSPECT_TARGET)
        self.assertEqual(result.target_place_id, 1)
        self.assertIsNone(result.first_portal_id)

    def test_covered_transit_reaches_remote_obligation(self):
        result = self.plan(
            1,
            [place(1), place(2), place(3)],
            [portal(11, 1, 2), portal(12, 2, 3)],
            work_items=[work(31, 3)],
        )

        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.portal_path, (11, 12))
        self.assertEqual(result.target_place_id, 3)
        self.assertEqual(result.obligation_kind, "work_item")
        self.assertEqual(result.obligation_id, 31)

    def test_all_covered_without_obligation_is_complete(self):
        result = self.plan(
            1,
            [place(1), place(2)],
            [portal(11, 1, 2)],
        )

        self.assertEqual(result.status, PLAN_COMPLETE)
        self.assertEqual(result.action, ACTION_HOLD)

    def test_disconnected_obligation_is_blocked_not_complete(self):
        result = self.plan(
            1,
            [place(1), place(2)],
            [],
            work_items=[work(31, 2)],
        )

        self.assertEqual(result.status, PLAN_BLOCKED)
        self.assertEqual(result.action, ACTION_HOLD)
        self.assertEqual(
            result.reason,
            "durable_obligation_has_no_certified_graph_path",
        )

    def test_active_local_attempt_blocks_a_second_graph_action(self):
        result = self.plan(
            1,
            [place(1), place(2, covered=False)],
            [portal(11, 1, 2)],
            work_items=[work(31, 1, active_attempt_id=41)],
            branch_first=True,
        )

        self.assertEqual(result.status, PLAN_BLOCKED)
        self.assertEqual(result.reason, "local_work_attempt_active")

    def test_hidden_current_work_item_does_not_freeze_visible_graph_progress(self):
        result = self.plan(
            1,
            [place(1), place(2, covered=False)],
            [portal(11, 1, 2)],
            work_items=[work(31, 1), work(32, 2)],
            branch_first=False,
            visible_work_item_ids=(),
        )

        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.target_place_id, 2)
        self.assertEqual(result.portal_path, (11,))

    def test_unmaterialized_current_work_item_does_not_block_visible_remote_progress(self):
        """A missing local viewpoint remains a ledger obligation, not a route lock."""
        result = self.plan(
            1,
            [place(1), place(2)],
            [portal(11, 1, 2)],
            work_items=[
                # No durable normal and absent from this snapshot's candidate
                # set: it cannot currently be rehydrated into a safe view.
                {
                    "id": 31,
                    "place_id": 1,
                    "state": "unresolved",
                    "active_attempt_id": None,
                    "normal_xy": None,
                },
                work(32, 2),
            ],
            branch_first=True,
            visible_work_item_ids=(32,),
        )

        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.target_place_id, 2)
        self.assertEqual(result.portal_path, (11,))

    def test_visible_same_place_item_replaces_stale_local_obligation(self):
        """The graph may execute an explicit same-place identity from this map."""
        result = self.plan(
            1,
            [place(1)],
            work_items=[
                {
                    "id": 31,
                    "place_id": 1,
                    "state": "unresolved",
                    "active_attempt_id": None,
                    "normal_xy": None,
                },
                {
                    "id": 32,
                    "place_id": 1,
                    "state": "unresolved",
                    "active_attempt_id": None,
                    "normal_xy": [1.0, 0.0],
                },
            ],
            branch_first=True,
            visible_work_item_ids=(32,),
        )

        self.assertEqual(result.action, ACTION_OBSERVE_LOCAL_WORK)
        self.assertEqual(result.obligation_id, 32)

    def test_branch_first_retains_hidden_local_intent_for_portal_promotion(self):
        """A hidden current WorkItem stays promotable during discovery."""
        result = self.plan(
            1,
            [place(1)],
            work_items=[work(31, 1)],
            branch_first=True,
            visible_work_item_ids=(),
        )

        self.assertEqual(result.status, PLAN_READY)
        self.assertEqual(result.action, ACTION_OBSERVE_LOCAL_WORK)
        self.assertEqual(result.obligation_id, 31)

    def test_rehydrated_current_work_item_remains_owned_by_graph_plan(self):
        result = self.plan(
            1,
            [place(1), place(2, covered=False)],
            [portal(11, 1, 2)],
            work_items=[work(31, 1), work(32, 2)],
            branch_first=False,
            visible_work_item_ids=(31,),
        )

        self.assertEqual(result.action, ACTION_OBSERVE_LOCAL_WORK)
        self.assertEqual(result.obligation_id, 31)

    def test_hidden_current_probe_does_not_starve_visible_local_work(self):
        result = self.plan(
            1,
            [place(1), place(2, covered=False)],
            [portal(11, 1, 2)],
            work_items=[work(31, 1)],
            probes=[
                {"id": 41, "source_place_id": 1, "state": "pending"},
            ],
            visible_work_item_ids=(31,),
            visible_probe_ids=(),
        )

        self.assertEqual(result.action, ACTION_OBSERVE_LOCAL_WORK)
        self.assertEqual(result.obligation_id, 31)

    def test_structural_boundary_probe_precedes_local_work(self):
        result = self.plan(
            1,
            [place(1), place(2, covered=False)],
            [portal(11, 1, 2)],
            work_items=[work(31, 1)],
            probes=[{
                "id": 41,
                "source_place_id": 1,
                "state": "pending",
                "observation_source": "structural_boundary",
            }],
            branch_first=True,
            structural_boundary_first=True,
        )

        self.assertEqual(result.action, ACTION_PROBE_PORTAL)
        self.assertEqual(result.obligation_id, 41)
        self.assertEqual(
            result.reason,
            "structural_boundary_requires_observation",
        )

    def test_hidden_probe_remains_a_durable_obligation_without_local_work(self):
        result = self.plan(
            1,
            [place(1)],
            probes=[
                {"id": 41, "source_place_id": 1, "state": "pending"},
            ],
            visible_probe_ids=(),
        )

        self.assertEqual(result.action, ACTION_PROBE_PORTAL)
        self.assertEqual(result.obligation_kind, "portal_probe")
        self.assertEqual(result.obligation_id, 41)

    def test_source_arrived_probe_selects_destination_phase(self):
        result = self.plan(
            1,
            [place(1)],
            probes=[
                {
                    "id": 41,
                    "source_place_id": 1,
                    "state": "source_arrived",
                },
            ],
        )

        self.assertEqual(result.action, ACTION_PROBE_PORTAL)
        self.assertEqual(result.obligation_kind, "portal_probe")
        self.assertEqual(result.obligation_id, 41)
        self.assertEqual(result.portal_probe_phase, "destination")
        self.assertEqual(
            result.produces_evidence[0].owner_kind,
            "portal_probe",
        )
        self.assertEqual(
            result.produces_evidence[0].kind,
            "portal_destination_view",
        )

    def test_inconclusive_destination_probe_is_not_planned_again_in_same_epoch(self):
        result = self.plan(
            1,
            [place(1)],
            probes=[
                {
                    "id": 41,
                    "source_place_id": 1,
                    "state": "source_arrived",
                    "last_phase": "destination",
                    "destination_attempt_epoch": 10,
                    "last_map_epoch": 10,
                    "destination_retry_allowed": False,
                },
            ],
        )

        self.assertEqual(result.status, PLAN_BLOCKED)
        self.assertEqual(result.action, ACTION_HOLD)
        self.assertEqual(
            result.reason,
            "durable_obligation_has_no_certified_graph_path",
        )

    def test_failed_destination_probe_remains_plannable_for_alternative_view(self):
        result = self.plan(
            1,
            [place(1)],
            probes=[
                {
                    "id": 41,
                    "source_place_id": 1,
                    "state": "source_arrived",
                    "last_phase": "destination",
                    "destination_attempt_epoch": 10,
                    "last_map_epoch": 10,
                    "destination_retry_allowed": True,
                },
            ],
        )

        self.assertEqual(result.status, PLAN_READY)
        self.assertEqual(result.action, ACTION_PROBE_PORTAL)
        self.assertEqual(result.obligation_id, 41)
        self.assertEqual(result.portal_probe_phase, "destination")

    def test_full_method_holds_instead_of_reentering_for_hidden_current_probe(self):
        result = self.plan(
            2,
            [place(1), place(2)],
            [portal(11, 2, 1)],
            probes=[
                {
                    "id": 41,
                    "source_place_id": 2,
                    "state": "source_arrived",
                    "last_phase": "destination",
                    "destination_attempt_epoch": 10,
                    "last_map_epoch": 10,
                    "destination_retry_allowed": False,
                },
            ],
            branch_first=True,
        )

        self.assertEqual(result.status, PLAN_BLOCKED)
        self.assertEqual(result.action, ACTION_HOLD)
        self.assertEqual(
            result.reason,
            "current_place_obligation_not_materialized",
        )

    def test_covered_directional_branch_is_not_replanned_as_observation(self):
        coverage = DirectionalBranchCoverage()
        key = (1, "portal:13")
        coverage.observe_frontier(key, (2.0, 3.0), direction_xy=(1.0, 0.0))
        coverage.complete_branch(key, event_id="destination-view-13")

        result = self.plan(
            1,
            [place(1)],
            [{
                "id": 13,
                "source_place_id": 1,
                "destination_place_id": None,
                "state": "certified",
            }],
            probes=[{
                "id": 41,
                "source_place_id": 1,
                "portal_id": 13,
                "state": "source_arrived",
            }],
            branch_coverage=coverage,
        )

        # Coverage suppresses a stale second observation attempt. The physical
        # Portal remains available to the graph as a crossing obligation.
        self.assertEqual(result.action, ACTION_PROBE_PORTAL)
        self.assertEqual(result.first_portal_id, 13)
        self.assertEqual(result.obligation_kind, "unbound_portal")

    def test_cycle_terminates_and_selects_stable_shortest_path(self):
        result = self.plan(
            1,
            [place(1), place(2), place(3)],
            [portal(20, 1, 2), portal(10, 2, 3), portal(30, 3, 1)],
            work_items=[work(31, 3)],
        )

        self.assertEqual(result.portal_path, (30,))
        self.assertEqual(result.target_place_id, 3)

    def test_input_order_does_not_change_result(self):
        places = [place(1), place(2), place(3)]
        portals = [portal(20, 1, 2), portal(10, 2, 3)]
        items = [work(31, 3)]
        first = self.plan(1, places, portals, items)
        second = self.plan(1, list(reversed(places)), list(reversed(portals)), list(reversed(items)))

        self.assertEqual(first.signature(), second.signature())

    def test_planner_does_not_mutate_inputs(self):
        places = [place(1), place(2)]
        portals = [portal(11, 1, 2)]
        items = [work(31, 2)]
        before = deepcopy((places, portals, items))

        self.plan(1, places, portals, items)

        self.assertEqual((places, portals, items), before)


if __name__ == "__main__":
    unittest.main()
