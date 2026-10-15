"""Regression tests for physical unknown-boundary work identity."""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

sys.modules.setdefault(
    "rospy",
    SimpleNamespace(logwarn=lambda *args, **kwargs: None, logwarn_throttle=lambda *args, **kwargs: None),
)

from global_frontier_work_item_components import (
    ObservationSupport,
    observation_supports,
)
from global_frontier_work_items import (
    ATTEMPT_FAILED,
    GlobalFrontierWorkItemLifecycleMixin,
    PlaceWorkItemLedger,
    WORK_DEFERRED,
    WORK_RESOLVED,
)
from global_frontier_execution_routes import GlobalFrontierExecutionRouteMixin


def cell_xy(origin_x=0.0):
    return lambda row, col: (origin_x + float(col) * 0.5, float(row) * 0.5)


class FrontierWorkItemComponentsTest(unittest.TestCase):
    def test_observation_support_derives_unknown_side_anchor_and_normal(self):
        frontier = np.zeros((5, 12), dtype=bool)
        unknown = np.zeros_like(frontier)
        frontier[2, 2] = True
        unknown[2, 3:8] = True

        supports = observation_supports(
            frontier,
            unknown,
            cell_xy=cell_xy(),
            resolution=0.5,
            sensor_horizon_m=2.5,
        )

        self.assertEqual(len(supports), 1)
        self.assertAlmostEqual(supports[0].anchor_xy[0], 1.0)
        self.assertAlmostEqual(supports[0].anchor_xy[1], 1.0)
        self.assertAlmostEqual(supports[0].normal_xy[0], 1.0)
        self.assertAlmostEqual(supports[0].normal_xy[1], 0.0)

    def test_reconcile_keeps_opposite_unknown_directions_distinct(self):
        ledger = PlaceWorkItemLedger(merge_radius=0.5)
        support_cells = frozenset({(10, 10), (10, 11)})
        forward = ObservationSupport(
            frozenset({(1, 1)}),
            support_cells,
            anchor_xy=(1.0, 1.0),
            normal_xy=(1.0, 0.0),
        )
        reverse = ObservationSupport(
            frozenset({(1, 4)}),
            support_cells,
            anchor_xy=(2.0, 1.0),
            normal_xy=(-1.0, 0.0),
        )

        first_cells, _ = ledger.reconcile(4, (forward,), now=1.0)
        first_id = first_cells[(1, 1)]
        second_cells, _ = ledger.reconcile(4, (reverse,), now=2.0)

        self.assertNotEqual(second_cells[(1, 4)], first_id)
        self.assertEqual(len(ledger._items), 2)

    def test_reconcile_inherits_same_direction_after_support_moves(self):
        ledger = PlaceWorkItemLedger(merge_radius=0.5)
        first = ObservationSupport(
            frozenset({(1, 1)}),
            frozenset({(10, 10), (10, 11)}),
            anchor_xy=(1.0, 1.0),
            normal_xy=(1.0, 0.0),
        )
        descendant = ObservationSupport(
            frozenset({(1, 4)}),
            frozenset({(10, 11), (10, 12)}),
            anchor_xy=(1.4, 1.0),
            normal_xy=(0.98, 0.2),
        )

        first_cells, _ = ledger.reconcile(4, (first,), now=1.0)
        item_id = first_cells[(1, 1)]
        second_cells, details = ledger.reconcile(4, (descendant,), now=2.0)

        self.assertEqual(second_cells[(1, 4)], item_id)
        self.assertEqual(details[item_id]["match"], "inherited")
        normal = ledger._items[item_id]["normal_xy"]
        self.assertAlmostEqual(normal[0] ** 2 + normal[1] ** 2, 1.0)
        self.assertGreater(normal[0] * 0.98 + normal[1] * 0.2, 0.0)

    def test_reconcile_uses_physical_anchor_when_support_cells_are_disjoint(self):
        ledger = PlaceWorkItemLedger(merge_radius=0.5)
        first = ObservationSupport(
            frozenset({(1, 1)}),
            frozenset({(10, 10)}),
            anchor_xy=(1.0, 1.0),
            normal_xy=(1.0, 0.0),
        )
        shifted = ObservationSupport(
            frozenset({(8, 8)}),
            frozenset({(30, 30)}),
            anchor_xy=(1.2, 1.0),
            normal_xy=(0.99, 0.1),
        )
        first_cells, _details = ledger.reconcile(4, (first,), now=1.0)
        item_id = first_cells[(1, 1)]
        shifted_cells, details = ledger.reconcile(4, (shifted,), now=2.0)

        self.assertEqual(shifted_cells[(8, 8)], item_id)
        self.assertEqual(details[item_id]["match"], "inherited")

    def test_physical_anchor_keeps_distinct_nearby_support_from_merging(self):
        ledger = PlaceWorkItemLedger(merge_radius=0.5)
        first = ObservationSupport(
            frozenset({(1, 1)}),
            frozenset({(10, 10)}),
            anchor_xy=(1.0, 1.0),
            normal_xy=(1.0, 0.0),
        )
        distinct = ObservationSupport(
            frozenset({(2, 2)}),
            frozenset({(10, 10)}),
            anchor_xy=(2.0, 1.0),
            normal_xy=(1.0, 0.0),
        )
        first_cells, _details = ledger.reconcile(4, (first,), now=1.0)
        first_id = first_cells[(1, 1)]
        distinct_cells, _details = ledger.reconcile(4, (distinct,), now=2.0)

        self.assertNotEqual(distinct_cells[(2, 2)], first_id)
        self.assertEqual(len(ledger._items), 2)

    def test_stale_terminal_without_attempt_id_cannot_resolve_replacement(self):
        ledger = PlaceWorkItemLedger(merge_radius=0.5)
        item = ledger._new_item(
            4, frozenset({(1, 1)}), now=1.0,
            anchor_xy=(1.0, 1.0), normal_xy=(1.0, 0.0),
        )
        first_id = ledger.active_attempt_id(
            ledger.dispatch_existing(item["id"], now=1.1, viewpoint_xy=(0.0, 1.0))
        )
        _item, first_attempt = ledger.fail_attempt(
            item["id"], now=2.0, reason="stall", attempt_id=first_id,
        )
        replacement = ledger.dispatch_existing(
            item["id"], now=2.1, viewpoint_xy=(0.0, 2.0),
        )
        replacement_id = ledger.active_attempt_id(replacement)

        self.assertIsNone(
            ledger.resolve(item["id"], now=2.2, reason="stale_terminal")
        )
        self.assertEqual(ledger.active_attempt_id(item["id"]), replacement_id)
        self.assertIsNone(
            ledger.resolve(
                item["id"], now=2.3, reason="old_terminal",
                attempt_id=first_attempt["id"],
            )
        )
        self.assertIsNotNone(
            ledger.resolve(
                item["id"], now=2.4, reason="current_terminal",
                attempt_id=replacement_id,
            )
        )

    def test_active_lifecycle_emits_one_dispatch_and_one_terminal_event(self):
        class Explorer(GlobalFrontierWorkItemLifecycleMixin):
            def __init__(self):
                self.place_work_items = PlaceWorkItemLedger(merge_radius=0.5)
                self.active_work_item_id = None
                self.active_work_item_attempt_id = None
                self.active_work_item_place_id = None
                self.active_work_item_goal = None
                self.active_work_item_route_kind = None
                self.active_work_item_dispatch_announced = False
                self.active_route_id = 17
                self.events = []

            def publish_status(self, event, **fields):
                self.events.append((event, fields))

        class Selection:
            route_kind = "frontier_endpoint"
            place_hops = 0
            work_item_id = None
            x = 2.0
            y = 3.0

        explorer = Explorer()
        item_id = explorer.dispatch_active_work_item(
            message=None,
            selection=Selection(),
            region={"id": 4},
            now=1.0,
        )
        self.assertIsNotNone(item_id)
        # Selection locks the ledger, but the public lifecycle begins only
        # after the execution boundary has been published.
        explorer.announce_active_work_item_dispatch()
        explorer.settle_active_work_item(4.5, WORK_RESOLVED, "observed")
        # A duplicate terminal callback is ignored after the active lease is
        # cleared, so experiment duration is not recorded twice.
        explorer.settle_active_work_item(4.6, WORK_DEFERRED, "stale_terminal")

        self.assertEqual([event for event, _fields in explorer.events], [
            "work_item_dispatched", "work_item_settled",
        ])
        self.assertEqual(explorer.events[0][1]["place_id"], 4)
        self.assertEqual(explorer.events[1][1]["work_item_state"], "resolved")

    def test_prefetched_reservation_respects_the_method_capability_gate(self):
        class Explorer(GlobalFrontierWorkItemLifecycleMixin):
            def __init__(self, enabled):
                self.work_item_memory_enabled = enabled
                self.place_work_items = PlaceWorkItemLedger(merge_radius=0.5)
                self.prefetched_work_item_id = None
                self.active_work_item_id = None
                self.active_work_item_attempt_id = None
                self.active_work_item_place_id = None
                self.active_work_item_goal = None
                self.active_work_item_route_kind = None
                self.active_work_item_dispatch_announced = False
                self.active_route_id = 3
                self.events = []

            def publish_status(self, event, **fields):
                self.events.append((event, fields))

        baseline = Explorer(enabled=False)
        self.assertIsNone(baseline.reserve_prefetched_work_item(
            None, 2.0, 3.0, {"id": 4}, now=1.0,
        ))
        self.assertEqual(baseline.events, [])

        full_method = Explorer(enabled=True)
        item_id = full_method.reserve_prefetched_work_item(
            None, 2.0, 3.0, {"id": 4}, now=1.0,
        )
        self.assertIsNotNone(item_id)
        full_method.announce_active_work_item_dispatch()
        self.assertEqual(full_method.events[0][0], "work_item_dispatched")

    def test_resolved_unknown_patch_descendant_survives_slam_map_shift(self):
        """A farther frontier of one physical unknown patch cannot reopen work."""
        ledger = PlaceWorkItemLedger(merge_radius=0.5)
        first_frontier = np.zeros((5, 18), dtype=bool)
        first_unknown = np.zeros_like(first_frontier)
        first_frontier[2, 2] = True
        first_unknown[2, 3:14] = True
        first = observation_supports(
            first_frontier,
            first_unknown,
            cell_xy=cell_xy(),
            resolution=0.5,
            sensor_horizon_m=5.0,
        )
        first_cells, _details = ledger.reconcile(4, first, now=1.0)
        item_id = first_cells[(2, 2)]
        ledger.dispatch_existing(item_id, now=1.1)
        ledger.settle(item_id, WORK_RESOLVED, now=2.0, reason="observed")

        # GMapping moved the map frame by +0.4 m. The new map frontier is
        # farther along the same unknown side, while its odom support remains
        # physically continuous with the completed parent.
        next_frontier = np.zeros_like(first_frontier)
        next_unknown = np.zeros_like(first_frontier)
        next_frontier[2, 7] = True
        next_unknown[2, 8:18] = True
        shifted = observation_supports(
            next_frontier,
            next_unknown,
            cell_xy=cell_xy(origin_x=0.4),
            map_to_physical_xy=lambda x, y: (x - 0.4, y),
            resolution=0.5,
            sensor_horizon_m=5.0,
        )
        second_cells, details = ledger.reconcile(4, shifted, now=3.0)

        self.assertEqual(second_cells[(2, 7)], item_id)
        self.assertEqual(details[item_id]["match"], "resolved_descendant")
        self.assertFalse(ledger.work_item_is_available(item_id))

    def test_failed_attempt_does_not_retire_support_descendant(self):
        ledger = PlaceWorkItemLedger(merge_radius=0.5)
        parent = ObservationSupport(
            frontier_cells=frozenset({(1, 1)}),
            support_cells=frozenset({(10, 10), (10, 11)}),
        )
        cells, _details = ledger.reconcile(4, (parent,), now=1.0)
        item_id = cells[(1, 1)]
        ledger.dispatch_existing(
            item_id, now=1.1, viewpoint_xy=(2.0, 1.0), map_goal=(2.0, 1.0),
        )
        _item, attempt = ledger.fail_attempt(item_id, 2.0, "stall")
        self.assertEqual(attempt["state"], ATTEMPT_FAILED)

        shifted = ObservationSupport(
            frontier_cells=frozenset({(1, 4)}),
            support_cells=frozenset({(10, 10), (10, 12)}),
        )
        descendant_cells, details = ledger.reconcile(4, (shifted,), now=3.0)
        self.assertEqual(descendant_cells[(1, 4)], item_id)
        self.assertEqual(details[item_id]["match"], "inherited")
        self.assertTrue(ledger.viewpoint_is_available(item_id, (3.0, 1.0)))
        self.assertFalse(ledger.viewpoint_is_available(item_id, (2.1, 1.0)))

    def test_identity_radius_does_not_collapse_alternate_recovery_viewpoints(self):
        ledger = PlaceWorkItemLedger(
            merge_radius=1.25,
            viewpoint_merge_radius=0.25,
        )
        cells, _details = ledger.reconcile(
            4,
            (ObservationSupport(
                frontier_cells=frozenset({(1, 1)}),
                support_cells=frozenset({(10, 10)}),
                anchor_xy=(2.0, 2.0),
                normal_xy=(1.0, 0.0),
            ),),
            now=1.0,
        )
        item_id = cells[(1, 1)]
        ledger.dispatch_existing(
            item_id,
            now=2.0,
            viewpoint_xy=(1.45, 1.55),
            map_goal=(1.45, 1.55),
            route_kind="frontier_endpoint",
        )
        ledger.fail_attempt(item_id, now=3.0, reason="stall")

        # The two views belong to one physical WorkItem (1.25 m identity
        # radius), but the second is a genuinely new Attempt (0.25 m radius).
        self.assertTrue(ledger.viewpoint_is_available(item_id, (1.95, 1.55)))
        self.assertFalse(ledger.viewpoint_is_available(item_id, (1.60, 1.55)))

    def test_map_disconnection_finishes_attempt_before_route_release(self):
        class Explorer(GlobalFrontierExecutionRouteMixin):
            def __init__(self):
                self.active_unreachable_since = 1.0
                self.active_last_waypoint_map = None
                self.unreachable_grace = 0.5
                self.active_best_path_distance = 3.0
                self.active_best_goal_distance = 2.0
                self.active_last_progress_signal = "none"
                self.active_route_kind = "frontier_endpoint"
                self.events = []
                self.released = False

            def active_waypoint_distance(self, _robot_map):
                return 0.0

            def publish_status(self, event, **fields):
                self.events.append((event, fields))

            def settle_active_work_item(self, now, state, reason):
                self.events.append(("attempt_settle", {
                    "now": now, "state": state, "reason": reason,
                }))

            def release_active_frontier(self, **_kwargs):
                self.released = True

        explorer = Explorer()
        result = explorer.handle_disconnected_active_frontier(
            1, 1, 2.0, 3.0, (0.0, 0.0), now=2.0,
        )

        self.assertEqual(result, (None, None))
        self.assertEqual(explorer.events[1], (
            "attempt_settle",
            {"now": 2.0, "state": "deferred", "reason": "disconnected"},
        ))
        self.assertTrue(explorer.released)

    def test_resolved_parent_covers_split_descendants(self):
        ledger = PlaceWorkItemLedger(merge_radius=0.5)
        parent = ObservationSupport(
            frontier_cells=frozenset({(1, 1)}),
            support_cells=frozenset({(10, 10), (10, 11), (11, 10), (11, 11)}),
        )
        parent_cells, _details = ledger.reconcile(9, (parent,), now=1.0)
        item_id = parent_cells[(1, 1)]
        ledger.dispatch_existing(item_id, now=1.1)
        ledger.settle(item_id, WORK_RESOLVED, now=2.0, reason="observed")

        children = (
            ObservationSupport(frozenset({(2, 2)}), frozenset({(10, 10), (10, 9)})),
            ObservationSupport(frozenset({(3, 3)}), frozenset({(11, 11), (12, 11)})),
        )
        cells, details = ledger.reconcile(9, children, now=3.0)

        self.assertEqual(cells[(2, 2)], item_id)
        self.assertEqual(cells[(3, 3)], item_id)
        self.assertEqual(details[item_id]["match"], "resolved_descendant")
        self.assertFalse(ledger.work_item_is_available(item_id))

    def test_wall_separated_unknown_sides_remain_two_work_items(self):
        ledger = PlaceWorkItemLedger(merge_radius=0.5)
        supports = (
            ObservationSupport(frozenset({(1, 1)}), frozenset({(4, 4), (4, 5)})),
            ObservationSupport(frozenset({(1, 8)}), frozenset({(4, 8), (4, 9)})),
        )
        cells, details = ledger.reconcile(5, supports, now=1.0)

        self.assertNotEqual(cells[(1, 1)], cells[(1, 8)])
        self.assertEqual({detail["match"] for detail in details.values()}, {"created"})

    def test_two_doorway_arcs_of_one_remote_unknown_blob_stay_distinct(self):
        frontier = np.zeros((7, 11), dtype=bool)
        unknown = np.zeros_like(frontier)
        frontier[3, 1] = True
        frontier[3, 9] = True
        unknown[3, 2:9] = True

        supports = observation_supports(
            frontier,
            unknown,
            cell_xy=cell_xy(),
            resolution=0.5,
            sensor_horizon_m=0.5,
        )

        self.assertEqual(len(supports), 2)
        self.assertFalse(supports[0].support_cells & supports[1].support_cells)


if __name__ == "__main__":
    unittest.main()
