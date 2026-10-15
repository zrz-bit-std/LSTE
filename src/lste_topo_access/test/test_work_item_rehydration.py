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
from global_frontier_work_items import PlaceWorkItemLedger  # noqa: E402


class WorkItemRehydrationFixture(GlobalFrontierCandidateWorkItemMixin):
    def __init__(self):
        self.place_work_items = PlaceWorkItemLedger(merge_radius=0.35)
        self.portal_probe_ledger = PortalProbeLedger(match_radius=0.35)
        self.frontier_approach_distance = 1.0
        self.completed_radius = 1.25
        self.events = []
        self.disable_inverse_tf = False

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


if __name__ == "__main__":
    unittest.main()
