from pathlib import Path
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_graph_route_planner import (  # noqa: E402
    ACTION_CROSS_PORTAL, ACTION_HOLD, ACTION_OBSERVE_LOCAL_WORK,
    GraphRoutePlanner, PLAN_BLOCKED, PLAN_READY,
)
from global_frontier_room_scope import (
    best_branch_for_portal, build_branch_scope, work_item_belongs_to_branch,
)  # noqa: E402
from global_frontier_work_items import PlaceWorkItemLedger  # noqa: E402


class RoomBranchScopeTest(unittest.TestCase):
    def setUp(self):
        self.branches = [
            {"id": 1, "node_id": 7, "state": "pending",
             "map_geometry": {"direction": [1.0, 0.0]}},
            {"id": 2, "node_id": 7, "state": "pending",
             "map_geometry": {"direction": [0.0, 1.0]}},
            {"id": 3, "node_id": 7, "state": "pending",
             "map_geometry": {"direction": [0.0, -1.0]}},
        ]
        self.scope = build_branch_scope([0.0, 0.0], self.branches, 1)

    def test_portal_room_scope_uses_best_ranked_associated_branch(self):
        branches = [
            {"id": 4, "canonical_portal_id": 12, "room_priority": 5},
            {"id": 6, "canonical_portal_id": 12, "room_priority": 2},
            {"id": 7, "canonical_portal_id": 12, "room_priority": 4},
            {"id": 8, "canonical_portal_id": 13, "room_priority": 1},
        ]
        self.assertEqual(best_branch_for_portal(branches, 12)["id"], 6)

    def test_scope_assigns_connected_place_work_to_nearest_gp_arm(self):
        self.assertTrue(work_item_belongs_to_branch({"anchor_xy": [4.0, 0.5]}, self.scope))
        self.assertFalse(work_item_belongs_to_branch({"anchor_xy": [0.5, 4.0]}, self.scope))
        self.assertFalse(work_item_belongs_to_branch({"anchor_xy": [0.5, -4.0]}, self.scope))
        self.assertFalse(work_item_belongs_to_branch({"anchor_xy": None}, self.scope))

    def test_room_planner_services_selected_arm_then_returns_to_parent(self):
        planner = GraphRoutePlanner()
        places = [{"id": 1, "state": "open", "endpoint_observations": 1},
                  {"id": 2, "state": "open", "endpoint_observations": 1}]
        portals = [{"id": 10, "source_place_id": 1, "destination_place_id": 2,
                    "state": "crossed", "crossed_at": 1.0}]
        work = [
            {"id": 31, "place_id": 2, "state": "unresolved",
             "kind": "frontier_observation", "anchor_xy": [4.0, 0.5]},
            {"id": 32, "place_id": 2, "state": "unresolved",
             "kind": "frontier_observation", "anchor_xy": [0.5, 4.0]},
        ]
        result = planner.plan(
            2, places=places, portals=portals, work_items=work,
            room_work_item_only=True, room_branch_scope=self.scope,
        )
        self.assertEqual(result.action, ACTION_OBSERVE_LOCAL_WORK)
        self.assertEqual(result.obligation_id, 31)
        work[0]["state"] = "resolved"
        result = planner.plan(
            2, places=places, portals=portals, work_items=work,
            room_work_item_only=True, room_branch_scope=self.scope,
        )
        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.first_portal_id, 10)
        self.assertEqual(
            result.reason, "dfs_place_and_children_complete_return_to_parent",
        )
        self.assertEqual(work[1]["state"], "unresolved")

    def test_repeatedly_stalled_semantic_portal_yields_until_later_map_epoch(self):
        planner = GraphRoutePlanner()
        places = [{"id": 1, "state": "open", "endpoint_observations": 1}]
        portals = [{"id": 10, "source_place_id": 1, "destination_place_id": None,
                    "state": "certified", "semantic_priority": 1,
                    "navigation_deferred_until_epoch": 25}]
        work = [
            {"id": 50, "place_id": 1, "state": "unresolved", "kind": "portal_entry",
             "portal_id": 10, "priority": 1},
            {"id": 51, "place_id": 1, "state": "unresolved",
             "kind": "frontier_observation", "anchor_xy": [4.0, 0.5],
             "normal_xy": [1.0, 0.0]},
        ]
        deferred = planner.plan(
            1, places=places, portals=portals, work_items=work,
            branch_first=True, map_epoch=10, visible_portal_ids=(10,),
            visible_work_item_ids=(51,),
        )
        self.assertEqual(deferred.action, ACTION_OBSERVE_LOCAL_WORK)
        self.assertEqual(deferred.obligation_id, 51)

        retried = planner.plan(
            1, places=places, portals=portals, work_items=work,
            branch_first=True, map_epoch=25, visible_portal_ids=(10,),
            visible_work_item_ids=(51,),
        )
        self.assertEqual(retried.action, ACTION_CROSS_PORTAL)
        self.assertEqual(retried.first_portal_id, 10)
        self.assertEqual(retried.reason, "semantic_priority_portal_entry_work_item:50")

    def test_room_waits_when_any_place_work_is_unprojected(self):
        result = GraphRoutePlanner().plan(
            2,
            places=[{"id": 1, "state": "open", "endpoint_observations": 1},
                    {"id": 2, "state": "open", "endpoint_observations": 1}],
            portals=[{"id": 10, "source_place_id": 1, "destination_place_id": 2,
                      "state": "crossed"}],
            work_items=[{"id": 31, "place_id": 2, "state": "unresolved",
                         "anchor_xy": [2.0, 0.0]}],
            room_work_item_only=True,
            visible_work_item_ids=(),
        )
        self.assertEqual(result.status, PLAN_BLOCKED)
        self.assertEqual(result.action, ACTION_HOLD)
        self.assertEqual(result.obligation_id, 31)
        self.assertEqual(
            result.reason, "semantic_room_work_item_projection_pending"
        )

    def test_certified_child_precedes_remaining_parent_work(self):
        result = GraphRoutePlanner().plan(
            2,
            places=[{"id": 1, "state": "open", "endpoint_observations": 1},
                    {"id": 2, "state": "open", "endpoint_observations": 1}],
            portals=[{"id": 20, "source_place_id": 2, "destination_place_id": None,
                      "state": "certified", "semantic_priority": 1},
                     {"id": 10, "source_place_id": 1, "destination_place_id": 2,
                      "state": "crossed"}],
            work_items=[{"id": 51, "place_id": 2, "state": "unresolved",
                         "kind": "portal_entry", "portal_id": 20, "priority": 1},
                        {"id": 52, "place_id": 2, "state": "unresolved",
                         "kind": "frontier_observation", "anchor_xy": [4.0, 0.5]}],
            branch_first=True,
            room_work_item_only=True,
            room_branch_scope=self.scope,
            visible_work_item_ids=(52,),
            visible_portal_ids=(20,),
        )
        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.first_portal_id, 20)
        self.assertEqual(result.reason, "dfs_semantic_child_before_parent_work")

    def test_room_explores_pending_child_before_returning_to_parent(self):
        result = GraphRoutePlanner().plan(
            2,
            places=[{"id": 1, "state": "open", "endpoint_observations": 1},
                    {"id": 2, "state": "open", "endpoint_observations": 1}],
            portals=[{"id": 20, "source_place_id": 2, "destination_place_id": None,
                      "state": "certified", "semantic_priority": 1},
                     {"id": 10, "source_place_id": 1, "destination_place_id": 2,
                      "state": "crossed"}],
            work_items=[{"id": 51, "place_id": 2, "state": "unresolved",
                         "kind": "portal_entry", "portal_id": 20, "priority": 1}],
            branch_first=True,
            room_work_item_only=True,
            visible_portal_ids=(20,),
        )
        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.first_portal_id, 20)
        self.assertEqual(result.reason, "dfs_semantic_child_before_parent_work")

    def test_room_does_not_egress_while_local_work_item_is_unprojected(self):
        result = GraphRoutePlanner().plan(
            2,
            places=[{"id": 1, "state": "open", "endpoint_observations": 1},
                    {"id": 2, "state": "open", "endpoint_observations": 1}],
            portals=[{"id": 10, "source_place_id": 1, "destination_place_id": 2,
                      "state": "crossed"}],
            work_items=[{"id": 52, "place_id": 2, "state": "unresolved",
                         "kind": "frontier_observation", "anchor_xy": [4.0, 0.5]}],
            branch_first=True,
            room_work_item_only=True,
            room_branch_scope=self.scope,
            visible_work_item_ids=(),
        )
        self.assertEqual(result.status, PLAN_BLOCKED)
        self.assertEqual(result.obligation_id, 52)
        self.assertEqual(result.reason, "semantic_room_work_item_projection_pending")

    def test_frontier_waypoint_arrival_honors_controller_xy_tolerance(self):
        from global_frontier_execution_watchdogs import GlobalFrontierExecutionWatchdogMixin

        class Route(GlobalFrontierExecutionWatchdogMixin):
            active_last_waypoint_map = (1.0, 1.0)
            active_route_kind = "frontier_endpoint"
            waypoint_release_radius = 0.40
            teb_xy_goal_tolerance = 0.50

        route = Route()
        self.assertTrue(route.active_frontier_observation_ready(0.45))
        self.assertFalse(route.active_frontier_observation_ready(0.51))

    def test_room_return_waits_for_all_place_work_items(self):
        ledger = PlaceWorkItemLedger(merge_radius=0.5)
        selected = ledger._new_item(2, (), 0.0, anchor_xy=[4.0, 0.5])
        sibling = ledger._new_item(2, (), 0.0, anchor_xy=[0.5, 4.0])
        self.assertEqual(ledger.unresolved_ids(2), [selected["id"], sibling["id"]])
        ledger.resolve(selected["id"], 1.0, reason="observed")
        self.assertEqual(ledger.unresolved_ids(2), [sibling["id"]])
        ledger.resolve(sibling["id"], 1.5, reason="observed")
        self.assertEqual(ledger.unresolved_ids(2), [])

    def test_room_close_closes_every_work_item_in_the_place(self):
        ledger = PlaceWorkItemLedger(merge_radius=0.5)
        items = [ledger._new_item(2, (), 0.0, anchor_xy=xy)
                 for xy in ([4.0, 0.5], [0.5, 4.0], None)]
        closed = ledger.close_place(
            2, 1.0, reason="semantic_room_explored_returned_to_junction",
        )
        self.assertCountEqual(closed, [item["id"] for item in items])
        snapshot = {item["id"]: item for item in ledger.snapshot()}
        self.assertTrue(all(snapshot[item["id"]]["state"] == "resolved"
                            for item in items))
        self.assertTrue(all(snapshot[item["id"]]["place_id"] == 2
                            for item in items))
        self.assertTrue(ledger.place_is_closed(2))


if __name__ == "__main__":
    unittest.main()
