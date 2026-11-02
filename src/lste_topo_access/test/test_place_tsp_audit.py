"""TSP runtime acceptance must inspect execution, not only intended plans."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    "audit_place_tsp", ROOT / "scripts/tests/targeted_navigation/audit_place_tsp.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PlaceTspAuditTest(unittest.TestCase):
    def test_observation_freeze_and_deferred_place_creation_are_checked(self):
        spec = importlib.util.spec_from_file_location(
            "room_audit", ROOT / "scripts/tests/targeted_navigation/audit_semantic_room_lifecycle.py")
        room_audit = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(room_audit)
        root = {"tree": {"nodes": [{"id": 10, "junction_confirmed": True, "physical_xy": [0, 0]}]}}
        watching = dict(root,
            frontier_event="portal_authentication_observation_period",
            portal_authentication={"node_id": 10, "phase": "watching", "all_finished": False},
            observation_tree={"10": {"rooms": [{"branch_id": 1, "place_id": None}]}})
        complete = dict(root, portal_authentication={"node_id": 10, "phase": "complete", "all_finished": True})
        entry = dict(complete, frontier_event="semantic_room_exploration_started",
                     active_semantic_room={"node_id": 10, "place_id": 2, "branch_id": 1})
        dispatch = dict(entry, frontier_event="work_item_dispatched", goal_context={"owner_place_id": 2})
        events = [dict(root, frontier_event="work_item_dispatched"), watching, complete, entry, dispatch]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "metrics.log"
            def checks(data):
                path.write_text("".join("event=global_frontier_event data=" + json.dumps(e) + "\n"
                                        for e in data))
                return room_audit.audit(path, [0, 0], 2.0, 1)["checks"]
            valid = checks(events)
            for key in ("normal_wi_used_before_junction", "normal_wi_frozen_during_observation",
                        "place_creation_deferred_during_observation", "room_exploration_after_observation_completed",
                        "room_work_items_keep_current_place_owner"):
                self.assertTrue(valid[key], key)
            existing = {"10": {"rooms": [{"branch_id": 1, "place_id": 2}]}}
            second_watch = dict(watching, observation_tree=existing)
            known = dict(dispatch, observation_tree=existing)
            self.assertTrue(checks(events + [known, second_watch])[
                "place_creation_deferred_during_observation"])
            bad = copy.deepcopy(events)
            bad[1]["frontier_event"] = "work_item_dispatched"
            self.assertFalse(checks(bad)["normal_wi_frozen_during_observation"])
            bad = copy.deepcopy(events)
            bad[1]["observation_tree"]["10"]["rooms"][0]["place_id"] = 2
            self.assertFalse(checks(bad)["place_creation_deferred_during_observation"])
            bad = copy.deepcopy(events)
            bad[4]["goal_context"]["owner_place_id"] = 99
            self.assertFalse(checks(bad)["room_work_items_keep_current_place_owner"])
            early_entry = dict(entry, portal_authentication=watching["portal_authentication"])
            self.assertFalse(checks(events[:2] + [early_entry, complete, dispatch])[
                "room_exploration_after_observation_completed"])

    def test_semantic_priority_must_match_actual_room_entry_order(self):
        spec = importlib.util.spec_from_file_location(
            "room_audit", ROOT / "scripts/tests/targeted_navigation/audit_semantic_room_lifecycle.py")
        room_audit = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(room_audit)
        rooms = [
            {"branch_id": 1, "place_id": 4, "semantic_score": 0.5, "priority": 2},
            {"branch_id": 2, "place_id": 5, "semantic_score": 0.9, "priority": 1},
        ]
        root = {
            "tree": {"nodes": [{"id": 10, "junction_confirmed": True, "physical_xy": [0, 0]}]},
            "observation_tree": {"10": {"rooms": rooms}},
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "metrics.log"
            for order, expected in (([2, 1], True), ([1, 2], False), ([2, 1, 2], False)):
                entries = [dict(root, frontier_event="semantic_room_exploration_started",
                                active_semantic_room={"node_id": 10, "branch_id": branch})
                           for branch in order]
                path.write_text("".join("event=global_frontier_event data=" + json.dumps(e) + "\n"
                                        for e in entries))
                result = room_audit.audit(path, [0, 0], 2.0, 2)
                self.assertEqual(result["checks"]["actual_room_exploration_follows_priority"], expected)

    def test_invalidated_same_place_candidate_needs_no_priority_or_child_portal(self):
        spec = importlib.util.spec_from_file_location(
            "room_audit", ROOT / "scripts/tests/targeted_navigation/audit_semantic_room_lifecycle.py")
        room_audit = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(room_audit)
        rooms = [
            {"branch_id": 1, "place_id": 4, "portal_id": 3, "priority": 1,
             "semantic_score": 0.8, "stable_frame_count": 2},
            {"branch_id": 2, "place_id": 5, "portal_id": 4, "priority": 2,
             "semantic_score": 0.6, "stable_frame_count": 2},
            {"branch_id": 3, "place_id": None, "portal_id": None, "priority": None,
             "semantic_score": -1.0, "stable_frame_count": 0, "invalidated": True,
             "exploration_suppressed": True},
        ]
        branches = [dict(id=1, canonical_portal_id=3), dict(id=2, canonical_portal_id=4),
                    dict(id=3, canonical_portal_id=None)]
        base = {"tree": {"nodes": [{"id": 10, "junction_confirmed": True,
                                      "physical_xy": [0, 0]}], "branches": branches},
                "observation_tree": {"10": {"rooms": rooms}}}
        events = [dict(base, frontier_event="work_item_dispatched"),
                  dict(base, frontier_event="portal_authentication_observation_period",
                       portal_authentication={"node_id": 10, "phase": "watching", "all_finished": False}),
                  dict(base, frontier_event="portal_authentication_observation_period",
                       portal_authentication={"node_id": 10, "phase": "complete", "all_finished": True}),
                  dict(base, frontier_event="semantic_room_exploration_started",
                       active_semantic_room={"node_id": 10, "branch_id": 1, "place_id": 4}),
                  dict(base, frontier_event="work_item_dispatched",
                       active_semantic_room={"node_id": 10, "branch_id": 1, "place_id": 4},
                       goal_context={"owner_place_id": 4}),
                  dict(base, frontier_event="semantic_room_exploration_started",
                       active_semantic_room={"node_id": 10, "branch_id": 2, "place_id": 5}),
                  dict(base, frontier_event="work_item_dispatched",
                       active_semantic_room={"node_id": 10, "branch_id": 2, "place_id": 5},
                       goal_context={"owner_place_id": 5})]
        for portal, branch, place in ((3, 1, 4), (4, 2, 5)):
            events.append(dict(base, transition={"event": "semantic_room_closed", "portal_id": portal},
                active_semantic_rooms=[{"branch_id": branch, "portal_id": portal, "place_id": place}],
                work_items=[], reentry_suppressed=True, portal_id=portal))
        events.append(dict(base, frontier_event="work_item_resume_target_selected", work_item_id=99,
            work_items=[{"id": 99, "state": "resolved", "attempts":[
                {"state":"suspended", "reason":"portal_junction_observation_period"},
                {"state":"succeeded"}]}]))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "metrics.log"
            path.write_text("".join("event=global_frontier_event data=" + json.dumps(e) + "\n"
                                    for e in events))
            result = room_audit.audit(path, [0, 0], 2.0, 3)
        self.assertTrue(result["checks"]["priority_follows_semantic_score"])
        self.assertTrue(result["checks"]["each_target_portal_closed"])
        self.assertTrue(result["checks"]["actual_room_exploration_follows_priority"])

    def test_correct_plan_cannot_hide_a_sibling_work_item_dispatch(self):
        events = []
        for item_id, tour in ((1, [1, 2]), (2, [2])):
            events.append({
                "frontier_event": "work_item_dispatched",
                "work_item_id": item_id,
                "goal_context": {"work_item_id": item_id, "owner_place_id": 3},
                "graph_route_plan": {
                    "action": "observe_local_work", "obligation_kind": "work_item",
                    "current_place_id": 3, "obligation_id": item_id,
                    "work_item_tour": tour,
                },
                "work_items": [{"id": 1, "place_id": 3}, {"id": 2, "place_id": 3}],
            })
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "metrics.log"
            def run():
                path.write_text("".join(
                    "event=global_frontier_event data=" + json.dumps(event) + "\n"
                    for event in events))
                return module.audit(path)
            self.assertTrue(run()["passed"])
            events[0]["work_item_id"] = 2
            result = run()
            self.assertTrue(result["checks"]["selected_item_is_tour_head"])
            self.assertFalse(result["checks"]["dispatched_item_is_tour_head"])
            self.assertFalse(result["passed"])


if __name__ == "__main__":
    unittest.main()
