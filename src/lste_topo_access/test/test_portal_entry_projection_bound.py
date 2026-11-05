#!/usr/bin/env python3
"""Bounded semantic Portal-entry projection.

A top-ranked room can be unprojectable for a while.  The planner must hold it
only for a bounded number of ticks, then let lower-ranked entries run, and
retry it after the epoch cooldown.  It must never become a permanent hold.
"""

from pathlib import Path
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_planning_selection import (  # noqa: E402
    portal_gate_approach_candidate,
)
from global_frontier_graph_route_planner import (  # noqa: E402
    ACTION_CROSS_PORTAL,
    ACTION_HOLD,
    ACTION_OBSERVE_LOCAL_WORK,
    GraphRoutePlanner,
    PLAN_BLOCKED,
    PLAN_READY,
)


def place(place_id):
    return {"id": place_id, "state": "open", "endpoint_observations": 1}


def work(item_id, place_id, **extra):
    item = {
        "id": item_id,
        "place_id": place_id,
        "state": "unresolved",
        "active_attempt_id": None,
    }
    item.update(extra)
    return item


class BoundedSemanticEntryProjectionTest(unittest.TestCase):
    def setUp(self):
        self.planner = GraphRoutePlanner()
        self.portals = [
            {"id": 13, "source_place_id": 1, "destination_place_id": None,
             "state": "certified"},
            {"id": 14, "source_place_id": 1, "destination_place_id": None,
             "state": "certified"},
        ]
        self.work_items = [
            dict(work(71, 1), kind="portal_entry", portal_id=13, priority=1),
            dict(work(70, 1), kind="portal_entry", portal_id=14, priority=2),
            work(31, 1),
        ]

    def plan(self, epoch, visible=(14,)):
        return self.planner.plan(
            1,
            places=[place(1)],
            portals=self.portals,
            work_items=self.work_items,
            probes=[{"id": 7, "source_place_id": 1, "state": "pending"}],
            branch_first=True,
            map_epoch=epoch,
            visible_portal_ids=visible,
            visible_probe_ids=(7,),
        )

    def test_top_entry_is_held_for_a_bounded_window_then_yields(self):
        for epoch in range(100, 104):
            result = self.plan(epoch)
            self.assertEqual(result.status, PLAN_BLOCKED)
            self.assertEqual(result.action, ACTION_HOLD)
            self.assertEqual(result.obligation_id, 71)
            self.assertEqual(
                result.reason,
                "semantic_priority_portal_entry_projection_pending",
            )

        # Fifth miss defers the unprojectable top entry in this epoch window.
        result = self.plan(104)
        self.assertEqual(result.status, PLAN_READY)
        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.first_portal_id, 14)
        self.assertEqual(result.reason, "semantic_priority_portal_entry_work_item:70")

    def test_top_entry_is_retried_after_the_cooldown(self):
        for epoch in range(100, 104):
            self.plan(epoch)
        self.plan(104)  # defers 13 until epoch 114

        result = self.plan(114)
        self.assertEqual(result.status, PLAN_BLOCKED)
        self.assertEqual(result.obligation_id, 71)
        self.assertEqual(
            result.reason,
            "semantic_priority_portal_entry_projection_pending",
        )

    def test_frozen_source_work_item_is_not_resumed_while_semantic_entry_pending(self):
        self.work_items = [
            dict(work(71, 1), kind="portal_entry", portal_id=13, priority=1),
            work(32, 1),
        ]
        for epoch in range(100, 104):
            self.plan(epoch)
        # Fifth miss defers the unprojectable Portal; the planner must pick
        # the ordinary same-Place work item (32), not resume the frozen 31.
        result = self.plan(104)
        self.assertEqual(result.status, PLAN_READY)
        self.assertEqual(result.obligation_id, 32)
        self.assertNotEqual(
            result.reason, "resume_suspended_work_item_after_portal_observation"
        )

    def test_visible_top_entry_is_selected_immediately(self):
        result = self.plan(100, visible=(13, 14))
        self.assertEqual(result.status, PLAN_READY)
        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.first_portal_id, 13)
        self.assertEqual(result.reason, "semantic_priority_portal_entry_work_item:71")


if __name__ == "__main__":
    unittest.main()


class PortalGateApproachCandidateTest(unittest.TestCase):
    def test_source_probe_becomes_a_plain_gate_approach(self):
        candidate = [None] * 23
        candidate[2], candidate[3] = 7.25, 0.05
        candidate[4] = 2.1
        candidate[10] = "frontier_endpoint"
        candidate[11] = (8.15, -0.05)
        candidate[15] = object()
        candidate[17] = "probe_portal"
        candidate[18] = "durable_probe"
        candidate[22] = "source"
        approach = portal_gate_approach_candidate(candidate)
        self.assertIsNotNone(approach)
        self.assertEqual(approach[10], "frontier_endpoint")
        self.assertIsNone(approach[15])
        self.assertEqual(approach[17], "portal_gate_approach")
        self.assertEqual(approach[18], "durable_probe_source_approach")
        self.assertIsNone(approach[22])
        self.assertEqual((approach[2], approach[3]), (7.25, 0.05))
        self.assertEqual(approach[11], (8.15, -0.05))

    def test_non_candidate_is_rejected(self):
        self.assertIsNone(portal_gate_approach_candidate(None))
        self.assertIsNone(portal_gate_approach_candidate((1, 2)))


class GateGoalLocalPlanTest(unittest.TestCase):
    def test_unprojected_top_entry_returns_local_gate_plan(self):
        planner = GraphRoutePlanner()
        result = planner.plan(
            1,
            places=[place(1)],
            portals=[],
            work_items=[
                dict(
                    work(71, 1),
                    kind="portal_entry",
                    portal_id=13,
                    priority=1,
                    x=8.05,
                    y=-0.05,
                ),
                work(31, 1),
            ],
            branch_first=True,
            map_epoch=100,
        )
        self.assertEqual(result.status, PLAN_READY)
        self.assertEqual(result.action, ACTION_OBSERVE_LOCAL_WORK)
        self.assertEqual(result.obligation_id, 71)
        self.assertEqual(result.reason, "semantic_portal_entry_gate_approach")


class BoundSemanticEntryCrossingTest(unittest.TestCase):
    """A pre-created destination must not filter its own doorway out."""

    def entry_plan(self, **portal_extra):
        portal = {
            "id": 13,
            "source_place_id": 1,
            "destination_place_id": 2,
            "state": "certified",
            "map_gate": [8.05, -0.05],
            "map_destination": [9.85, -0.35],
            "physical_gate": [8.03, -0.02],
            "physical_destination": [9.80, -0.28],
            "gp_authentication_state": "completed",
            "exploration_suppressed": False,
        }
        portal.update(portal_extra)
        return GraphRoutePlanner().plan(
            1,
            places=[place(1), place(2)],
            portals=[portal],
            work_items=[
                dict(
                    work(71, 1),
                    kind="portal_entry",
                    portal_id=13,
                    priority=1,
                    x=8.03,
                    y=-0.02,
                ),
                work(31, 1),
            ],
            branch_first=True,
            map_epoch=100,
        )

    def test_bound_certified_portal_is_crossed_by_its_entry(self):
        result = self.entry_plan()
        self.assertEqual(result.status, PLAN_READY)
        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.first_portal_id, 13)
        self.assertEqual(
            result.reason, "semantic_priority_portal_entry_work_item:71",
        )

    def test_suppressed_bound_portal_returns_the_gate_local_plan(self):
        result = self.entry_plan(exploration_suppressed=True)
        self.assertEqual(result.status, PLAN_READY)
        self.assertEqual(result.action, ACTION_OBSERVE_LOCAL_WORK)
        self.assertEqual(result.obligation_id, 71)
        self.assertEqual(result.reason, "semantic_portal_entry_gate_approach")


class BoundRoomChildEntryTest(unittest.TestCase):
    def test_bound_child_portal_is_crossed_inside_a_room_visit(self):
        planner = GraphRoutePlanner()
        result = planner.plan(
            2,
            places=[place(1), place(2), place(4)],
            portals=[{
                "id": 3,
                "source_place_id": 2,
                "destination_place_id": 4,
                "state": "certified",
                "map_gate": [11.5, 0.0],
                "map_destination": [11.8, 0.5],
                "physical_gate": [11.5, 0.0],
                "physical_destination": [11.8, 0.5],
                "exploration_suppressed": False,
            }],
            work_items=[
                dict(
                    work(16, 2),
                    kind="portal_entry",
                    portal_id=3,
                    priority=1,
                    x=11.5,
                    y=0.0,
                ),
            ],
            branch_first=True,
            room_work_item_only=True,
            dfs_parent_portal_id=1,
            dfs_ancestor_place_ids=(1,),
            dfs_completed_place_ids=(),
            map_epoch=100,
        )
        self.assertEqual(result.status, PLAN_READY)
        self.assertEqual(result.action, ACTION_CROSS_PORTAL)
        self.assertEqual(result.first_portal_id, 3)
        self.assertEqual(
            result.reason, "dfs_semantic_child_before_parent_work",
        )
