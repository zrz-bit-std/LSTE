"""Regression tests for durable local WorkItem viewpoint rehydration."""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_candidate_work_items import (  # noqa: E402
    GlobalFrontierCandidateWorkItemMixin,
)
from global_frontier_portal_probe_ledger import PortalProbeLedger  # noqa: E402
from gp_branch_frontier import GPBranchFrontierMixin  # noqa: E402
from global_frontier_selection_planner import (  # noqa: E402
    GlobalFrontierSelectionPlannerMixin,
)
from global_frontier_work_items import (  # noqa: E402
    ATTEMPT_SUSPENDED,
    GlobalFrontierWorkItemLifecycleMixin,
    PlaceWorkItemLedger,
)


class WorkItemRehydrationFixture(
    GlobalFrontierWorkItemLifecycleMixin,
    GlobalFrontierCandidateWorkItemMixin,
    GPBranchFrontierMixin,
    GlobalFrontierSelectionPlannerMixin,
):
    def __init__(self):
        self.place_work_items = PlaceWorkItemLedger(merge_radius=0.35)
        self.portal_probe_ledger = PortalProbeLedger(match_radius=0.35)
        self.frontier_approach_distance = 1.0
        self.completed_radius = 1.25
        self.events = []
        self.disable_inverse_tf = False
        self.work_item_memory_enabled = True
        self.active_work_item_id = None
        self.active_work_item_attempt_id = None
        self.active_work_item_place_id = None
        self.active_work_item_goal = None
        self.active_work_item_route_kind = None
        self.active_work_item_dispatch_announced = False
        self.suspended_work_item_id = None
        self.current_physical_place_id = 1
        self.endpoint_terminal_wait_radius = 0.45
        self.active_frontier = None
        self.portal_authentication_by_node = {}
        self.junction_portal_authentication = None
        self.gp_portal_assistance_enabled = False

    @staticmethod
    def planar_xy_projector(target_frame, source_frame):
        del target_frame, source_frame
        return lambda x, y: (float(x), float(y))

    def _inverse_projector(self, target_frame, source_frame):
        if self.disable_inverse_tf:
            return None
        if (target_frame, source_frame) == ("map", "odom"):
            return lambda x, y: (float(x), float(y))
        return None

    @staticmethod
    def nearest_reachable_cell(_message, _steps, _x, _y):
        return 2, 2

    @staticmethod
    def cell_xy(_message, row, col):
        return float(col), float(row)

    @staticmethod
    def candidate_costmap_distance(_validation, _x, _y):
        return None

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


def request():
    return SimpleNamespace(
        message=SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            info=SimpleNamespace(resolution=1.0),
        ),
        steps=np.ones((6, 6), dtype=np.int32),
        validation=None,
        map_to_physical_xy=lambda x, y: (float(x), float(y)),
    )


class WorkItemRehydrationTest(unittest.TestCase):
    def test_fallback_can_reobserve_from_newly_known_opposite_side_in_same_place(self):
        fixture = WorkItemRehydrationFixture()
        item = fixture.place_work_items._new_item(
            1, frozenset({(8, 8)}), 1.0,
            anchor_xy=(8.0, 8.0), normal_xy=(1.0, 0.0),
        )
        req = request()
        req.message.info.origin = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0)
        )
        req.robot_map = (11.0, 8.0)
        req.steps = np.full((20, 20), -1, dtype=np.int32)
        req.steps[8, 11] = 3
        req.unknown = np.zeros((20, 20), dtype=bool)
        fixture.nearest_reachable_cell = (
            lambda _message, _steps, _x, _y: None
        )
        candidates = fixture.pending_local_work_item_candidates(
            req, SimpleNamespace(source_place_id=1),
            np.zeros((20, 20), dtype=bool), 1,
        )
        self.assertEqual(len(candidates), 1)
        _row, _col, candidate = candidates[0]
        self.assertEqual(candidate.work_item_id, item["id"])
        self.assertEqual(
            candidate.graph_action_reason,
            "durable_work_item_rehydrated_opposite_side_same_place",
        )

    def test_fallback_search_expands_to_remote_work_item_inside_current_place(self):
        fixture = WorkItemRehydrationFixture()
        item = fixture.place_work_items._new_item(
            1, frozenset({(8, 8)}), 1.0,
            anchor_xy=(8.0, 8.0), normal_xy=(1.0, 0.0),
        )
        req = request()
        req.message.info.origin = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0)
        )
        req.robot_map = (1.0, 8.0)
        req.steps = np.full((20, 20), -1, dtype=np.int32)
        req.steps[8, 1] = 7
        req.unknown = np.zeros((20, 20), dtype=bool)
        fixture.nearest_reachable_cell = (
            lambda _message, _steps, _x, _y: None
        )
        candidates = fixture.pending_local_work_item_candidates(
            req, SimpleNamespace(source_place_id=1),
            np.zeros((20, 20), dtype=bool), 1,
        )
        self.assertEqual(len(candidates), 1)
        _row, _col, candidate = candidates[0]
        self.assertEqual(candidate.work_item_id, item["id"])
        self.assertGreater(((8.0 - candidate.x) ** 2
                           + (8.0 - candidate.y) ** 2) ** 0.5, 5.0)

    def test_durable_normal_is_rotated_into_current_map_frame(self):
        fixture = WorkItemRehydrationFixture()
        fixture.planar_xy_projector = lambda _target, _source: (
            lambda x, y: (-float(y), float(x))
        )
        normal = fixture._work_item_map_normal(
            request(), {"anchor_xy": (2.0, 3.0), "normal_xy": (1.0, 0.0)},
        )
        self.assertAlmostEqual(normal[0], 0.0)
        self.assertAlmostEqual(normal[1], 1.0)

    def test_portal_observation_suspends_and_resumes_same_work_item(self):
        fixture = WorkItemRehydrationFixture()
        item = fixture.place_work_items.dispatch(
            1, (1.0, 2.0), now=1.0, map_epoch=1,
        )
        fixture.active_work_item_id = item
        fixture.active_work_item_attempt_id = (
            fixture.place_work_items.active_attempt_id(item)
        )

        branch = {
            "stable": True, "passability": "open",
            "physical_gate": (1.0, 0.0), "heading": 0.0,
            "map_geometry": {"width": 2.0,
                             "physical_bounds": [(1.0, -1.0), (1.0, 1.0)]},
        }
        tree = SimpleNamespace(
            current_node_id=1,
            nodes={1: {
                "junction_confirmed": True, "branch_ids": [7],
                "physical_xy": (0.0, 0.0),
            }},
            branches={7: branch},
        )
        fixture._sync_portal_authentication(
            tree, None, None, now=2.0,
        )

        suspended = next(
            record for record in fixture.place_work_items.snapshot()
            if record["id"] == item
        )
        self.assertEqual(suspended["state"], "unresolved")
        self.assertEqual(suspended["attempts"][-1]["state"], ATTEMPT_SUSPENDED)
        self.assertEqual(fixture.suspended_work_item_id, item)
        self.assertIn("work_item_suspended", [event for event, _ in fixture.events])

        resumed = fixture.dispatch_active_work_item(
            request().message,
            SimpleNamespace(
                route_kind="frontier_endpoint", place_hops=0,
                work_item_id=item, x=1.0, y=2.0, component=None,
            ),
            {"id": 1},
            3.0,
        )

        self.assertEqual(resumed, item)
        self.assertIsNone(fixture.suspended_work_item_id)
        self.assertIn("work_item_resumed", [event for event, _ in fixture.events])
        resumed_record = next(
            record for record in fixture.place_work_items.snapshot()
            if record["id"] == item
        )
        self.assertEqual(resumed_record["state"], "unresolved")

    def test_portal_observation_preserves_next_pending_work_item_when_none_active(self):
        fixture = WorkItemRehydrationFixture()
        pending = fixture.place_work_items._new_item(
            1, frozenset({(3, 3)}), 1.0, anchor_xy=(2.0, 2.0),
            normal_xy=(1.0, 0.0),
        )
        pending_id = pending["id"]

        selected = fixture.suspend_active_work_item(
            2.0, "portal_junction_observation_period",
        )

        self.assertEqual(selected["id"], pending_id)
        self.assertEqual(fixture.suspended_work_item_id, pending_id)
        self.assertIn(
            "work_item_resume_target_selected",
            [event for event, _ in fixture.events],
        )
        record = next(item for item in fixture.place_work_items.snapshot()
                      if item["id"] == pending_id)
        self.assertEqual(record["state"], "unresolved")
        self.assertIsNone(record["active_attempt_id"])

        resumed = fixture.dispatch_active_work_item(
            request().message,
            SimpleNamespace(
                route_kind="frontier_endpoint", place_hops=0,
                work_item_id=pending_id, x=1.0, y=2.0, component=None,
            ),
            {"id": 1}, 3.0,
        )
        self.assertEqual(resumed, pending_id)
        self.assertIsNone(fixture.suspended_work_item_id)
        self.assertIn("work_item_resumed", [event for event, _ in fixture.events])

    def test_watching_period_blocks_new_work_item_selection(self):
        fixture = WorkItemRehydrationFixture()
        fixture.gp_portal_assistance_enabled = True
        fixture.junction_portal_authentication = SimpleNamespace(
            branches={7: object()}, phase="watching", all_finished=lambda: False,
        )
        fixture._reset_frontier_selection_metrics = lambda: None

        selected = fixture.choose_frontier(
            None, None, None, None, None, now=2.0,
        )

        self.assertIsNone(selected)

    def test_unresolved_item_gets_a_new_current_map_viewpoint(self):
        fixture = WorkItemRehydrationFixture()
        item = fixture.place_work_items._new_item(
            1,
            frozenset({(3, 3)}),
            1.0,
            anchor_xy=(2.0, 2.0),
            normal_xy=(1.0, 0.0),
        )
        context = SimpleNamespace(source_place_id=1)

        candidates = fixture.pending_local_work_item_candidates(
            request(), context, frontier=np.zeros((6, 6), dtype=bool), approach_cells=1,
        )

        self.assertEqual(len(candidates), 1)
        _row, _col, candidate = candidates[0]
        self.assertEqual(candidate.work_item_id, item["id"])
        self.assertEqual(candidate.work_item_match, "rehydrated_work_item")
        self.assertFalse(candidate.viewpoint_retry)

    def test_lifecycle_rejected_normal_view_tries_same_item_lateral_view(self):
        fixture = WorkItemRehydrationFixture()
        item_id = fixture.place_work_items._new_item(
            1, frozenset({(3, 3)}), 1.0,
            anchor_xy=(2.0, 2.0), normal_xy=(1.0, 0.0))["id"]
        fixture.frontier_information = lambda *args: 0.0
        tried = []
        def admit(_request, candidate, _context, _information):
            tried.append(candidate.graph_action_reason)
            return None if len(tried) == 1 else ("revisit", {})
        fixture._candidate_region_pool = admit
        req = request()
        req.unknown = np.zeros((6, 6), dtype=bool)
        result = fixture.pending_local_work_item_candidates(
            req, SimpleNamespace(source_place_id=1),
            np.zeros((6, 6), dtype=bool), 1)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0][2].work_item_id, item_id)
        self.assertEqual(tried, ["durable_work_item_rehydrated_normal",
                                 "durable_work_item_rehydrated_lateral_left"])

    def test_all_lifecycle_rejected_views_leave_work_unresolved(self):
        fixture = WorkItemRehydrationFixture()
        item_id = fixture.place_work_items._new_item(
            1, frozenset({(3, 3)}), 1.0,
            anchor_xy=(2.0, 2.0), normal_xy=(1.0, 0.0))["id"]
        fixture.frontier_information = lambda *args: 0.0
        fixture._candidate_region_pool = lambda *args: None
        req = request()
        req.unknown = np.zeros((6, 6), dtype=bool)
        self.assertEqual(fixture.pending_local_work_item_candidates(
            req, SimpleNamespace(source_place_id=1),
            np.zeros((6, 6), dtype=bool), 1), [])
        self.assertEqual(fixture.place_work_items.unresolved_count(1), 1)

    def test_physical_anchor_is_not_used_without_inverse_tf(self):
        fixture = WorkItemRehydrationFixture()
        fixture.planar_xy_projector = fixture._inverse_projector
        fixture.disable_inverse_tf = True
        fixture.place_work_items._new_item(
            1,
            frozenset({(3, 3)}),
            1.0,
            anchor_xy=(2.0, 2.0),
            normal_xy=(1.0, 0.0),
        )

        candidates = fixture.pending_local_work_item_candidates(
            request(), SimpleNamespace(source_place_id=1),
            frontier=np.zeros((6, 6), dtype=bool), approach_cells=1,
        )

        self.assertEqual(candidates, [])
        self.assertEqual(
            fixture.events[-1][1]["rejection_reasons"],
            {"physical_anchor_projection_unavailable": 1},
        )

    def test_viewpoint_ladder_keeps_far_side_views_after_local_failures(self):
        ladder = WorkItemRehydrationFixture()._viewpoint_ladder(
            (2.0, 2.0), (1.0, 0.0), 1.0
        )
        self.assertEqual(
            [name for name, _point in ladder],
            ["normal", "lateral_left", "lateral_right",
             "lateral_left_far", "lateral_right_far", "normal_far"],
        )
        points = dict(ladder)
        self.assertEqual(points["lateral_left_far"], (1.0, 4.0))
        self.assertEqual(points["lateral_right_far"], (1.0, 0.0))
        self.assertAlmostEqual(points["normal_far"][0], 0.3)
        self.assertAlmostEqual(points["normal_far"][1], 2.0)

    def test_failed_viewpoint_uses_a_lateral_view_of_the_same_item(self):
        fixture = WorkItemRehydrationFixture()
        fixture.nearest_reachable_cell = (
            lambda _message, _steps, x, y: (int(round(y)), int(round(x)))
        )
        item = fixture.place_work_items._new_item(
            1,
            frozenset({(3, 3)}),
            1.0,
            anchor_xy=(2.0, 2.0),
            normal_xy=(1.0, 0.0),
        )
        fixture.place_work_items.dispatch_existing(
            item["id"],
            now=2.0,
            viewpoint_xy=(1.0, 2.0),
            map_goal=(1.0, 2.0),
            route_kind="frontier_endpoint",
        )
        fixture.place_work_items.fail_attempt(
            item["id"], now=3.0, reason="stall",
        )

        candidates = fixture.pending_local_work_item_candidates(
            request(), SimpleNamespace(source_place_id=1),
            frontier=np.zeros((6, 6), dtype=bool), approach_cells=1,
        )

        self.assertEqual(len(candidates), 1)
        _row, _col, candidate = candidates[0]
        self.assertTrue(candidate.viewpoint_retry)
        self.assertEqual(
            candidate.graph_action_reason,
            "durable_work_item_rehydrated_lateral_left",
        )
        self.assertNotEqual((candidate.x, candidate.y), (1.0, 2.0))

    def test_navfn_route_rejection_uses_a_lateral_view_without_an_attempt(self):
        fixture = WorkItemRehydrationFixture()
        fixture.nearest_reachable_cell = (
            lambda _message, _steps, x, y: (int(round(y)), int(round(x)))
        )
        item = fixture.place_work_items._new_item(
            1,
            frozenset({(3, 3)}),
            1.0,
            anchor_xy=(2.0, 2.0),
            normal_xy=(1.0, 0.0),
        )
        fixture.place_work_items.record_route_rejection(
            item["id"],
            now=2.0,
            viewpoint_xy=(1.0, 2.0),
            map_goal=(1.0, 2.0),
            reason="navfn_endpoint_offset",
        )

        candidates = fixture.pending_local_work_item_candidates(
            request(), SimpleNamespace(source_place_id=1),
            frontier=np.zeros((6, 6), dtype=bool), approach_cells=1,
        )

        self.assertEqual(len(candidates), 1)
        _row, _col, candidate = candidates[0]
        self.assertTrue(candidate.viewpoint_retry)
        self.assertEqual(
            candidate.graph_action_reason,
            "durable_work_item_rehydrated_lateral_left",
        )
        self.assertNotEqual((candidate.x, candidate.y), (1.0, 2.0))
        rejected = next(
            entry
            for entry in fixture.events[-1][1]["item_audit"]
            if entry.get("strategy") == "normal"
        )
        self.assertEqual(rejected["reason"], "route_rejected_in_map_epoch")

    def test_room_visit_rehydrates_probe_bound_work_item_as_local_work(self):
        fixture = WorkItemRehydrationFixture()
        item = fixture.place_work_items._new_item(
            2,
            frozenset({(3, 3)}),
            1.0,
            anchor_xy=(2.0, 2.0),
            normal_xy=(1.0, 0.0),
        )
        probe = fixture.portal_probe_ledger.observe(
            2, (2.0, 2.0), (1.0, 0.0), now=1.0,
        )
        fixture.portal_probe_ledger.bind_work_item(probe["id"], item["id"])
        fixture.active_semantic_room = {
            "place_id": 2, "source_place_id": 1,
        }

        candidates = fixture.pending_local_work_item_candidates(
            request(), SimpleNamespace(source_place_id=2),
            frontier=np.zeros((6, 6), dtype=bool), approach_cells=1,
        )

        self.assertEqual(len(candidates), 1)
        _row, _col, candidate = candidates[0]
        self.assertEqual(candidate.work_item_id, item["id"])
        self.assertEqual(candidate.work_item_match, "rehydrated_work_item")
        self.assertEqual(fixture.events[-1][1]["candidate_count"], 1)

    def test_portal_owned_item_is_left_to_probe_ledger(self):
        fixture = WorkItemRehydrationFixture()
        item = fixture.place_work_items._new_item(
            1,
            frozenset({(3, 3)}),
            1.0,
            anchor_xy=(2.0, 2.0),
            normal_xy=(1.0, 0.0),
        )
        probe = fixture.portal_probe_ledger.observe(
            1, (2.0, 2.0), (1.0, 0.0), now=1.0,
        )
        fixture.portal_probe_ledger.bind_work_item(probe["id"], item["id"])

        candidates = fixture.pending_local_work_item_candidates(
            request(), SimpleNamespace(source_place_id=1),
            frontier=np.zeros((6, 6), dtype=bool), approach_cells=1,
        )

        self.assertEqual(candidates, [])
        self.assertEqual(fixture.events[-1][1]["rejection_reasons"], {
            "owned_by_portal_probe": 1,
        })


    def test_collapsed_failed_ladder_finds_a_distinct_reachable_view_in_place(self):
        fixture = WorkItemRehydrationFixture()
        item = fixture.place_work_items._new_item(
            1, frozenset({(4, 4)}), 1.0,
            anchor_xy=(3.0, 3.0), normal_xy=(1.0, 0.0),
        )
        fixture.place_work_items.dispatch_existing(
            item["id"], now=2.0, viewpoint_xy=(2.0, 2.0),
            map_goal=(2.0, 2.0), route_kind="frontier_endpoint",
        )
        fixture.place_work_items.fail_attempt(
            item["id"], now=3.0, reason="controller_stall_no_physical_progress",
        )
        req = request()
        req.message.info.origin = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0)
        )
        req.steps = np.ones((8, 8), dtype=np.int32)
        fixture.nearest_reachable_cell = (
            lambda _message, _steps, _x, _y: (2, 2)
        )
        req.unknown = np.zeros((8, 8), dtype=bool)

        candidates = fixture.pending_local_work_item_candidates(
            req, SimpleNamespace(source_place_id=1),
            frontier=np.zeros((8, 8), dtype=bool), approach_cells=1,
        )

        self.assertEqual(len(candidates), 1)
        _row, _col, candidate = candidates[0]
        self.assertEqual(candidate.work_item_id, item["id"])
        self.assertTrue(candidate.viewpoint_retry)
        self.assertEqual(
            candidate.graph_action_reason,
            "durable_work_item_rehydrated_reachable_same_place",
        )
        self.assertNotEqual((candidate.x, candidate.y), (2.0, 2.0))
        self.assertLess(candidate.x, 3.0)


    def test_exhausted_nearby_views_expand_to_an_untried_view_in_same_place(self):
        fixture = WorkItemRehydrationFixture()
        item = fixture.place_work_items._new_item(
            1, frozenset({(6, 6)}), 1.0,
            anchor_xy=(6.0, 6.0), normal_xy=(1.0, 0.0),
        )
        item_id = item["id"]
        attempt = fixture.place_work_items.dispatch_existing(
            item_id, now=2.0, viewpoint_xy=(6.0, 5.0),
            map_goal=(6.0, 5.0), route_kind="frontier_endpoint",
            map_epoch=1,
        )
        self.assertEqual(attempt, item_id)
        fixture.place_work_items.fail_attempt(
            item_id, now=3.0,
            reason="controller_stall_no_physical_progress",
        )
        original_available = fixture.place_work_items.viewpoint_is_available
        def nearby_views_blocked(work_id, viewpoint_xy, map_epoch=None):
            distance = ((6.0 - viewpoint_xy[0]) ** 2
                        + (6.0 - viewpoint_xy[1]) ** 2) ** 0.5
            return distance > 3.2 and original_available(
                work_id, viewpoint_xy, map_epoch=map_epoch,
            )
        fixture.place_work_items.viewpoint_is_available = nearby_views_blocked

        fixture.nearest_reachable_cell = lambda _message, _steps, _x, _y: (5, 6)
        req = request()
        req.message.info.origin = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0)
        )
        req.steps = np.ones((12, 12), dtype=np.int32)
        req.unknown = np.zeros((12, 12), dtype=bool)
        candidates = fixture.pending_local_work_item_candidates(
            req, SimpleNamespace(source_place_id=1),
            np.zeros((12, 12), dtype=bool), 1,
        )

        self.assertEqual(len(candidates), 1)
        _row, _col, candidate = candidates[0]
        self.assertEqual(candidate.work_item_id, item_id)
        self.assertTrue(candidate.viewpoint_retry)
        self.assertEqual(
            candidate.graph_action_reason,
            "durable_work_item_rehydrated_reachable_same_place",
        )
        self.assertGreater(
            ((6.0 - candidate.x) ** 2 + (6.0 - candidate.y) ** 2) ** 0.5,
            3.2,
        )
        self.assertLess(
            ((6.0 - candidate.x) ** 2 + (6.0 - candidate.y) ** 2) ** 0.5,
            5.01,
        )


if __name__ == "__main__":
    unittest.main()
