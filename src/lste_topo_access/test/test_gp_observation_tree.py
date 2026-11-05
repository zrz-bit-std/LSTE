"""Tests for junction observation scores and room-local graph scheduling."""
from pathlib import Path
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from gp_observation_tree import JunctionObservationTree
from global_frontier_semantic_belief import (
    semantic_room_score,
    task_intent_from_message,
)
from types import SimpleNamespace
from global_frontier_portal_authentication import (
    DoorwayAuthentication,
    GlobalFrontierPortalAuthenticationMixin,
    JunctionDoorwayAuthentication,
)
from global_frontier_work_items import PlaceWorkItemLedger
from global_frontier_graph_route_adapter import GlobalFrontierGraphRouteAdapterMixin
from global_frontier_graph_route_planner import (
    ACTION_CROSS_PORTAL,
    ACTION_OBSERVE_LOCAL_WORK,
    ACTION_PROBE_PORTAL,
    GraphRoutePlanner,
)


class JunctionObservationTreeTest(unittest.TestCase):
    def test_scores_middle_of_longest_stable_detection_segment(self):
        tree = JunctionObservationTree()
        room = tree.begin(7, 12, 1.25)
        for stamp, label, score in [
            (1, "chair", 0.2),
            (2, "cup", 0.4),
            (3, "cup", 0.6),
            (4, "cup", 0.9),
            (5, "cup", 0.7),
            (6, "cup", 0.5),
            (7, "cup", 0.95),
            (8, "desk", 0.1),
        ]:
            tree.observe(7, 12, [{"label": label, "score": score}], stamp)
        result = tree.finish_room(7, 12, valid=True, place_id=4, work_item_ids=[3, 3, 8])
        self.assertEqual(result["relative_heading"], 1.25)
        self.assertEqual(result["labels"], ["cup"])
        self.assertEqual(result["stable_frame_count"], 4)
        self.assertAlmostEqual(result["semantic_score"], (0.6 + 0.9 + 0.7 + 0.5) / 4)
        self.assertEqual(result["work_item_ids"], [3, 8])

    def test_unobserved_superseded_room_is_safely_pruned(self):
        tree = JunctionObservationTree()
        tree.begin(4, 20, 0.0)
        tree.begin(4, 21, 1.2)
        tree.observe(4, 21, [{"label": "desk", "score": 0.8}], 1.0)
        tree.finish_room(4, 21, valid=True)
        ranked = tree.finish_junction(4)
        unobserved = next(room for room in ranked if room["branch_id"] == 20)
        observed = next(room for room in ranked if room["branch_id"] == 21)
        self.assertEqual(unobserved["semantic_score"], -1.0)
        self.assertEqual(unobserved["state"], "pruned")
        self.assertTrue(unobserved["exploration_suppressed"])
        self.assertFalse(unobserved["invalidated"])
        self.assertEqual(observed["semantic_score"], 0.8)

    def test_superseded_room_is_pruned_and_cannot_accumulate_detections(self):
        tree = JunctionObservationTree()
        tree.begin(6, 30, 0.4)
        tree.observe(6, 30, [{"label": "chair", "score": 0.9}], 1.0)
        room = tree.prune_branch(6, 30)
        self.assertEqual(room["state"], "pruned")
        self.assertTrue(room["exploration_suppressed"])
        self.assertEqual(room["semantic_score"], -1.0)
        self.assertEqual(room["frames"], [])
        self.assertFalse(tree.observe(6, 30, [{"label": "desk", "score": 0.95}], 2.0))
        self.assertEqual(tree.snapshot()[6]["rooms"][0]["state"], "pruned")

    def test_invalid_and_negative_score_rooms_are_distinct(self):
        tree = JunctionObservationTree()
        tree.begin(2, 10, 0.0)
        tree.begin(2, 11, 1.0)
        tree.observe(2, 10, [{"label": "cup", "score": 0.8}], 1.0)
        tree.finish_room(2, 10, valid=True)
        tree.finish_room(2, 11, valid=False)
        ranked = tree.finish_junction(2)
        self.assertEqual([room["branch_id"] for room in ranked], [10, 11])
        self.assertFalse(ranked[0]["invalidated"])
        self.assertTrue(ranked[1]["invalidated"])
        self.assertTrue(ranked[1]["exploration_suppressed"])
        self.assertEqual(ranked[1]["semantic_score"], -1.0)

    def test_late_valid_certification_clears_earlier_no_score_pruning(self):
        tree=JunctionObservationTree()
        tree.begin(2,10,0.)
        tree.begin(2,11,1.)
        tree.observe(2,10,[{"label":"desk","score":.8}],1.)
        tree.finish_room(2,10,valid=True)
        tree.finish_junction(2)
        self.assertTrue(tree.snapshot()[2]["rooms"][1]["exploration_suppressed"])
        tree.finish_room(2,11,valid=True)
        ranked=tree.finish_junction(2)
        self.assertEqual([r["branch_id"] for r in ranked],[10,11])
        self.assertEqual(ranked[1]["semantic_score"],-1.)
        self.assertEqual(ranked[1]["state"],"observed")
        self.assertFalse(ranked[1]["exploration_suppressed"])


class RoomGraphPolicyTest(unittest.TestCase):
    def setUp(self):
        self.planner = GraphRoutePlanner()
        self.room_scope = {
            "origin_xy": [0.0, 0.0],
            "selected_branch_id": 1,
            "directions": [
                {"branch_id": 1, "direction_xy": [1.0, 0.0]},
                {"branch_id": 2, "direction_xy": [0.0, 1.0]},
            ],
        }

    def plan(self, **kwargs):
        return self.planner.plan(
            2,
            places=[
                {"id": 1, "state": "open", "endpoint_observations": 1},
                {"id": 2, "state": "open", "endpoint_observations": 1},
            ],
            portals=[
                {"id": 20, "source_place_id": 1, "destination_place_id": 2, "state": "crossed"},
                {"id": 21, "source_place_id": 2, "destination_place_id": None,
                 "state": "certified", "semantic_priority": 1},
            ],
            branch_first=True,
            room_work_item_only=True,
            room_branch_scope=self.room_scope,
            **kwargs,
        )

    def test_certified_child_portal_precedes_parent_local_work(self):
        result = self.plan(work_items=[{
            "id": 31, "place_id": 2, "state": "unresolved",
            "active_attempt_id": None, "normal_xy": [1.0, 0.0],
            "anchor_xy": [2.0, 0.0],
        }])
        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.first_portal_id, 21)

    def test_unfinished_child_prevents_return_even_without_parent_work(self):
        result = self.plan(work_items=[])
        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.first_portal_id, 21)
        self.assertEqual(result.reason, "dfs_certified_child_before_parent_work")

    def test_semantic_priority_orders_unbound_portals(self):
        result = self.planner.plan(
            1,
            places=[{"id": 1, "state": "open", "endpoint_observations": 1}],
            portals=[
                {"id": 50, "source_place_id": 1, "destination_place_id": None,
                 "state": "certified", "semantic_priority": 2},
                {"id": 51, "source_place_id": 1, "destination_place_id": None,
                 "state": "certified", "semantic_priority": 1},
            ],
            branch_first=True,
        )
        self.assertEqual(result.action, ACTION_PROBE_PORTAL)
        self.assertEqual(result.first_portal_id, 51)

    def test_ranked_portal_entry_precedes_unrelated_source_arrived_probe(self):
        result = self.planner.plan(
            1,
            places=[{"id": 1, "state": "open", "endpoint_observations": 1}],
            portals=[{
                "id": 21, "source_place_id": 1, "destination_place_id": None,
                "state": "certified", "semantic_priority": 1,
                "semantic_score": 0.94,
            }],
            work_items=[{
                "id": 70, "place_id": 1, "kind": "portal_entry",
                "portal_id": 21, "priority": 1, "state": "unresolved",
            }],
            probes=[{
                "id": 61, "source_place_id": 1, "state": "source_arrived",
                "physical_gate_xy": [5.0, 0.0], "last_phase": "source",
            }],
            branch_first=True,
        )
        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.first_portal_id, 21)
        self.assertEqual(result.reason, "semantic_priority_portal_entry_work_item:70")

    def test_semantic_priority_bypasses_linked_legacy_probe(self):
        result = self.planner.plan(
            1,
            places=[{"id": 1, "state": "open", "endpoint_observations": 1}],
            portals=[
                {"id": 50, "source_place_id": 1, "destination_place_id": None,
                 "state": "certified", "semantic_priority": 2},
                {"id": 51, "source_place_id": 1, "destination_place_id": None,
                 "state": "certified", "semantic_priority": 1},
            ],
            probes=[{
                "id": 60, "source_place_id": 1, "portal_id": 50,
                "physical_gate_xy": [2.0, 0.0], "normal_xy": [1.0, 0.0],
                "state": "pending",
            }],
            visible_portal_ids=[50],
            branch_first=True,
        )
        self.assertEqual(result.action, ACTION_PROBE_PORTAL)
        self.assertEqual(result.first_portal_id, 51)

    def test_portal_entry_work_item_is_idempotent_and_resolves_only_on_entry(self):
        ledger = PlaceWorkItemLedger(merge_radius=0.35)
        entry_id = ledger.ensure_portal_entry(51, 1, (2.5, 0.0), 1, 0.91, 1.0)
        self.assertEqual(
            ledger.ensure_portal_entry(51, 1, (2.5, 0.0), 1, 0.91, 2.0),
            entry_id,
        )
        entry = next(item for item in ledger.snapshot() if item["id"] == entry_id)
        self.assertEqual(entry["kind"], "portal_entry")
        self.assertEqual(entry["portal_id"], 51)
        self.assertEqual(entry["state"], "unresolved")
        support = type("Support", (), {
            "support_cells": frozenset({(2, 3)}),
            "frontier_cells": frozenset({(3, 3)}),
            "anchor_xy": (2.5, 0.0), "normal_xy": (1.0, 0.0),
        })()
        ledger.reconcile(1, [support], now=3.0)
        self.assertEqual(
            next(item for item in ledger.snapshot() if item["id"] == entry_id)["kind"],
            "portal_entry",
        )
        resolved = ledger.resolve_portal_entry(51, 4.0)
        self.assertEqual(resolved["state"], "resolved")
        self.assertEqual(resolved["last_reason"], "portal_crossing_created_place")

    def test_highest_ranked_portal_entry_work_item_precedes_regular_work(self):
        result = self.planner.plan(
            1,
            places=[{"id": 1, "state": "open", "endpoint_observations": 1}],
            portals=[
                {"id": 50, "source_place_id": 1, "destination_place_id": None,
                 "state": "certified", "semantic_priority": 2},
                {"id": 51, "source_place_id": 1, "destination_place_id": None,
                 "state": "certified", "semantic_priority": 1},
            ],
            work_items=[
                {"id": 60, "place_id": 1, "state": "unresolved", "kind": "frontier_observation"},
                {"id": 70, "place_id": 1, "state": "unresolved", "kind": "portal_entry", "portal_id": 50, "priority": 2},
                {"id": 71, "place_id": 1, "state": "unresolved", "kind": "portal_entry", "portal_id": 51, "priority": 1},
            ],
            branch_first=True,
        )
        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.first_portal_id, 51)
        self.assertEqual(result.reason, "semantic_priority_portal_entry_work_item:71")

    def test_closed_room_dismisses_existing_and_blocks_regenerated_work(self):
        ledger = PlaceWorkItemLedger(merge_radius=0.35)
        item = ledger.dispatch(8, (1.0, 2.0), now=1.0, map_epoch=3)
        ledger.close_place(8, now=2.0, reason="semantic_room_returned")
        self.assertTrue(ledger.place_is_closed(8))
        closed = next(record for record in ledger.snapshot() if record["id"] == item)
        self.assertEqual(closed["state"], "resolved")
        support = type("Support", (), {
            "support_cells": frozenset({(2, 3)}),
            "frontier_cells": frozenset({(3, 3)}),
            "anchor_xy": (1.0, 2.0), "normal_xy": (1.0, 0.0),
        })()
        self.assertEqual(ledger.reconcile(8, [support], now=3.0), ({}, {}))

    def test_suppressed_portal_is_not_reentered(self):
        result = self.planner.plan(
            1,
            places=[{"id": 1, "state": "open", "endpoint_observations": 1}],
            portals=[{"id": 50, "source_place_id": 1, "destination_place_id": None,
                      "state": "certified", "exploration_suppressed": True}],
        )
        self.assertNotEqual(result.action, ACTION_PROBE_PORTAL)


class GraphRouteOwnershipTest(unittest.TestCase):
    def test_portal_watch_owns_navigation_until_all_branches_finish(self):
        fixture = type("Fixture", (GlobalFrontierGraphRouteAdapterMixin,), {})()
        fixture.gp_portal_assistance_enabled = True
        fixture.active_portal_authentication = lambda: None
        fixture.junction_portal_authentication = type("Task", (), {
            "branches": {7: object()}, "phase": "watching",
            "all_finished": lambda self: False,
        })()
        self.assertTrue(fixture.portal_authentication_owns_navigation())

        fixture.junction_portal_authentication.phase = "complete"
        fixture.junction_portal_authentication.all_finished = lambda: True
        self.assertFalse(fixture.portal_authentication_owns_navigation())

    def test_active_authentication_owns_navigation_even_without_watch_task(self):
        fixture = type("Fixture", (GlobalFrontierGraphRouteAdapterMixin,), {})()
        fixture.gp_portal_assistance_enabled = True
        fixture.active_portal_authentication = lambda: object()
        fixture.junction_portal_authentication = None
        self.assertTrue(fixture.portal_authentication_owns_navigation())


if __name__ == "__main__":
    unittest.main()


class NegativeSemanticPortalLifecycleTest(unittest.TestCase):
    def test_valid_unknown_semantics_preserves_entry_and_unfinished_child_work(self):
        class Tree:
            def __init__(self, branch_id, node_id, portal_id):
                self.nodes = {node_id: {"physical_xy": [0.0, 0.0]}}
                self.branches = {branch_id: {
                    "canonical_portal_id": portal_id,
                    "destination_space_id": 2,
                }}

            def reject_branch_authentication(self, branch_id, reason):
                self.branches[branch_id]["authentication_failure_reason"] = reason

        class PortalLedger:
            def __init__(self, portal_id, record):
                self.portal_id = portal_id
                self.record = record

            def get(self, portal_id):
                return self.record if int(portal_id) == self.portal_id else None

        class Runtime(GlobalFrontierPortalAuthenticationMixin):
            def __init__(self):
                self.status_events = []

            def publish_status(self, event, **payload):
                self.status_events.append((event, payload))

        node_id, branch_id, portal_id = 9, 14, 23
        work_items = PlaceWorkItemLedger(merge_radius=0.2)
        entry_id = work_items.ensure_portal_entry(
            portal_id, 1, (2.0, 0.0), 1, 0.8, 1.0,
        )
        room_item_id = work_items.dispatch(2, (3.0, 1.0), 1.0)
        self.assertIsNotNone(entry_id)
        self.assertIsNotNone(room_item_id)

        branch = DoorwayAuthentication(branch_id, node_id, (1.0, 0.0), (2.0, 0.0))
        task = JunctionDoorwayAuthentication(node_id, settle_seconds=0.0)
        self.assertTrue(task.add(branch, now=0.0))
        self.assertIs(task.start_next((0.0, 0.0), now=1.0), branch)
        branch.state = "completed"

        runtime = Runtime()
        runtime.initialize_portal_authentication()
        runtime.junction_portal_authentication = task
        runtime.gp_exploration_tree = Tree(branch_id, node_id, portal_id)
        runtime.gp_observation_tree = JunctionObservationTree()
        runtime.gp_observation_tree.begin(node_id, branch_id, 0.0)
        runtime.portal_hypothesis_ledger = PortalLedger(portal_id, {
            "id": portal_id,
            "source_place_id": 1,
            "destination_place_id": 2,
            "physical_destination": [2.0, 0.0],
            "semantic_entry_work_item_id": entry_id,
        })
        runtime.place_work_items = work_items
        runtime.portal_probe_ledger = None

        self.assertIsNone(runtime.finish_portal_authentication_branch(now=2.0))

        room = runtime.gp_observation_tree.snapshot()[node_id]["rooms"][0]
        self.assertEqual(room["semantic_score"], -1.0)
        self.assertFalse(room["exploration_suppressed"])
        self.assertEqual(room["work_item_ids"], [entry_id])
        self.assertFalse(runtime.portal_hypothesis_ledger.record["exploration_suppressed"])
        states = {item["id"]: item for item in work_items.snapshot()}
        self.assertEqual(states[entry_id]["state"], "unresolved")
        self.assertNotEqual(states[room_item_id]["state"], "resolved")
        self.assertNotIn("room_work_item_ids_dismissed", runtime.gp_exploration_tree.branches[branch_id])
        self.assertGreater(work_items.unresolved_count(2), 0)


class TaskSemanticRoomRankingTest(unittest.TestCase):
    def test_environment_room_outranks_high_confidence_fire_hydrant(self):
        task = SimpleNamespace(
            task_id="yellow_cup",
            target_name="yellow cup",
            target_attributes=["yellow", "ceramic"],
            env_related_structures=["office desk", "monitor", "chair"],
            env_type_prior=["office workspace"],
            obj_key_objects=["computer monitor", "other mugs"],
            obj_negative_clues=[
                "door", "fire hydrant", "fire extinguisher",
            ],
            ctx_left="monitor",
            ctx_right="monitor",
            raw_json="",
        )
        intent = task_intent_from_message(task, "version-a")
        tree = JunctionObservationTree(
            semantic_scorer=lambda labels: semantic_room_score(intent, labels),
        )
        for branch_id, detections in (
            (10, [{"label": "fire hydrant", "score": 0.96}]),
            (11, [{"label": "fire hydrant", "score": 0.91}]),
            (12, [{"label": "chair", "score": 0.78},
                  {"label": "monitor", "score": 0.78}]),
        ):
            tree.begin(2, branch_id, 0.0)
            tree.observe(2, branch_id, detections, 1.0)
            tree.finish_room(2, branch_id, valid=True, place_id=branch_id)

        ranked = tree.finish_junction(2)

        self.assertEqual([room["branch_id"] for room in ranked], [12, 10, 11])
        self.assertEqual(ranked[0]["semantic_score_source"], "task_environment")
        self.assertGreater(ranked[0]["semantic_score"], 0.0)
        self.assertLess(ranked[1]["semantic_score"], 0.0)
        self.assertLess(ranked[2]["semantic_score"], 0.0)
        # A negative score lowers priority; it does not invalidate a room
        # that was physically observed and successfully authenticated.
        self.assertTrue(all(room["state"] == "observed" for room in ranked))
        self.assertTrue(all(not room["exploration_suppressed"] for room in ranked))
