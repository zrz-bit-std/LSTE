#!/usr/bin/env python3
"""Unit tests for the map-only frontier observation-region memory."""

import ast
import collections
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from global_frontier_place_memory import FrontierRegionMemory
from global_frontier_grid import xy_to_grid_cell as grid_xy_to_grid_cell
from global_frontier_topology import (
    adjacent_place_transition_goals,
    TopologicalFreeSpaceComponents,
    first_route_place_portal,
    first_route_place_transition_goal,
    frontier_score_bucket_prefixes,
    grid_frontier_observed_from_viewpoint,
    grid_line_is_known_free,
    grid_reachable_without_closed_places,
    grid_route_avoids_closed_places,
    grid_visible_free_footprint,
    place_action_tier,
    route_place_hops,
    same_topology_component,
    semantic_hint_is_forward,
)


def load_visibility_methods():
    main_source = SCRIPTS_DIR / "lste_global_frontier_node.py"
    observation_source = SCRIPTS_DIR / "global_frontier_observation_viewpoints.py"
    main_tree = ast.parse(
        main_source.read_text(encoding="utf-8"), filename=str(main_source)
    )
    observation_tree = ast.parse(
        observation_source.read_text(encoding="utf-8"),
        filename=str(observation_source),
    )
    explorer = next(
        node for node in main_tree.body
        if isinstance(node, ast.ClassDef) and node.name == "GlobalFrontierExplorer"
    )
    observation = next(
        node for node in observation_tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierObservationViewpointMixin"
    )
    main_methods = {
        node.name: node
        for node in explorer.body
        if isinstance(node, ast.FunctionDef) and node.name == "xy_to_grid_cell"
    }
    observation_methods = {
        node.name: node
        for node in observation.body
        if isinstance(node, ast.FunctionDef)
        and node.name in {
            "candidate_covered_by_ready_place_viewpoint",
            "candidate_covered_by_covered_place_viewpoint",
            "completed_viewpoint_component",
            "candidate_covered_by_completed_viewpoint",
        }
    }
    selected = list(main_methods.values()) + list(observation_methods.values())
    namespace = {
        "math": math,
        "grid_line_is_known_free": grid_line_is_known_free,
        "grid_xy_to_grid_cell": grid_xy_to_grid_cell,
    }
    exec(
        compile(
            ast.Module(body=selected, type_ignores=[]),
            str(observation_source),
            "exec",
        ),
        namespace,
    )
    return namespace


VISIBILITY_METHODS = load_visibility_methods()


class FrontierRegionMemoryTest(unittest.TestCase):
    def make_memory(self):
        return FrontierRegionMemory(
            radius=2.5,
            information_delta=8,
            stagnation_timeout=12,
            failure_limit=2,
            limit=16,
        )

    def test_target_pursuit_prefers_only_forward_progress_when_available(self):
        origin = (4.0, 7.0)
        hint = (4.0, 12.0)
        self.assertTrue(semantic_hint_is_forward(origin, hint, (4.2, 8.0)))
        self.assertFalse(semantic_hint_is_forward(origin, hint, (4.0, 6.5)))
        # A recovery frontier still needs an ordinary fallback when the hint is
        # degenerate or the map contains no branch in that direction.
        self.assertFalse(semantic_hint_is_forward(origin, origin, (4.2, 8.0)))

    def test_place_action_tier_prioritizes_local_coverage(self):
        self.assertEqual(place_action_tier(0, True), "local")
        self.assertEqual(place_action_tier(1, True), "adjacent")
        self.assertIsNone(place_action_tier(2, True))
        self.assertEqual(place_action_tier(None, False), "unconstrained")

    def test_target_room_claim_requires_the_exact_current_map_component(self):
        # A target-room claim is stronger than ordinary region-memory
        # association. Across one map snapshot it may not bridge a doorway by
        # centroid proximity: only one exact topology label keeps the vehicle
        # in the room where direct target evidence was observed.
        claimed = self.component(30, 4, center_x=10.0, center_y=8.0)
        same_room = self.component(30, 4, center_x=11.5, center_y=8.2)
        corridor = self.component(30, 5, center_x=9.0, center_y=8.0)
        old_snapshot = self.component(29, 4, center_x=10.0, center_y=8.0)

        self.assertTrue(same_topology_component(claimed, same_room))
        self.assertFalse(same_topology_component(claimed, corridor))
        self.assertFalse(same_topology_component(claimed, old_snapshot))
        self.assertFalse(same_topology_component(claimed, None))

    def test_closed_place_is_removed_from_global_route_graph(self):
        # The metric path reaches a far frontier only by passing through a
        # previously completed side room. It must be rejected at the graph
        # level rather than merely receiving a weaker score.
        steps = np.tile(np.arange(7, dtype=np.int32), (5, 1))
        labels = np.zeros((5, 7), dtype=np.int32)
        labels[:, 3] = 9

        self.assertFalse(
            grid_route_avoids_closed_places(
                labels, steps, 2, 6, source_label=None, closed_labels={9},
            )
        )

    def test_source_place_remains_available_so_the_robot_can_leave_it(self):
        steps = np.tile(np.arange(7, dtype=np.int32), (5, 1))
        labels = np.zeros((5, 7), dtype=np.int32)
        labels[:, :3] = 9

        self.assertTrue(
            grid_route_avoids_closed_places(
                labels, steps, 2, 6, source_label=9, closed_labels={9},
            )
        )

    def test_sealed_portal_ledger_rejects_reentry_but_allows_departure(self):
        # The entry direction is part of a portal fact. A dormant room must
        # reject a later route entering from the corridor, but its own route
        # remains free to leave through that same doorway in reverse.
        memory = self.make_memory()
        region, _tier = memory.enter(
            3.0,
            0.0,
            now=1.0,
            entry_portal=(2.0, 0.0),
        )
        memory.dormant(region, now=2.0, reason="place_observation_complete")

        inbound = memory.sealed_portal_entry((2.1, 0.0), (3.1, 0.0))
        outbound = memory.sealed_portal_entry((2.1, 0.0), (1.1, 0.0))
        reverse_exit = memory.portal_exit_from_entry(
            (2.1, 0.0), (3.1, 0.0), (1.1, 0.0)
        )
        false_exit = memory.portal_exit_from_entry(
            (2.1, 0.0), (1.1, 0.0), (3.1, 0.0)
        )

        self.assertIsNotNone(inbound)
        self.assertEqual(inbound[0]["id"], region["id"])
        self.assertIsNone(outbound)
        self.assertIsNotNone(reverse_exit)
        self.assertEqual(reverse_exit[0]["id"], region["id"])
        self.assertIsNone(false_exit)

    def test_reverse_egress_resolves_the_recorded_source_place(self):
        memory = self.make_memory()
        source, _tier = memory.enter(0.0, 0.0, now=0.0)
        destination, _tier = memory.enter(
            3.0,
            0.0,
            now=1.0,
            entry_portal=(2.0, 0.0),
            source_place_id=source["id"],
        )
        memory.dormant(destination, now=2.0, reason="place_observation_complete")

        reverse_exit = memory.portal_exit_from_entry(
            (2.0, 0.0),
            (3.0, 0.0),
            (1.0, 0.0),
        )

        self.assertIsNotNone(reverse_exit)
        self.assertEqual(reverse_exit[0]["id"], source["id"])
        self.assertEqual(
            reverse_exit[1].get("source_place_id"),
            source["id"],
        )

    def test_physical_portal_ledger_survives_a_map_frame_shift(self):
        """A corrected map must not make an old doorway look unfamiliar."""
        memory = self.make_memory()
        region, _tier = memory.enter(
            18.0,
            3.0,
            now=1.0,
            entry_portal=(17.0, 3.0),
            physical_xy=(5.0, 4.0),
            physical_entry_portal=(4.0, 4.0),
        )
        memory.dormant(region, now=2.0, reason="place_observation_complete")

        # In the next SLAM epoch this doorway has wholly different map
        # coordinates. Its odom direction remains the same physical fact.
        inbound = memory.sealed_portal_entry(
            (-9.0, 16.0),
            (-8.0, 16.0),
            physical_gate_xy=(4.05, 4.0),
            physical_destination_xy=(5.05, 4.0),
        )
        outbound = memory.sealed_portal_entry(
            (-9.0, 16.0),
            (-10.0, 16.0),
            physical_gate_xy=(4.05, 4.0),
            physical_destination_xy=(3.05, 4.0),
        )

        self.assertIsNotNone(inbound)
        self.assertEqual(inbound[0]["id"], region["id"])
        self.assertIsNone(outbound)

    def test_same_door_destination_reuses_place_before_component_matching(self):
        """A noisy reverse route cannot mint a second room at one doorway."""
        memory = self.make_memory()
        first_component = self.component(
            10, 4, center_x=20.4, center_y=8.8, width=8.0, height=0.6,
        )
        region, tier = memory.enter(
            20.45,
            8.85,
            now=1.0,
            component=first_component,
            entry_portal=(19.65, 9.25),
            physical_xy=(20.42, 8.82),
            physical_entry_portal=(19.64, 9.42),
        )
        self.assertEqual(tier, "new")

        # A later SLAM snapshot shifts the doorway and reports a different
        # structural core on the same destination side.  The physical gate,
        # not the component label, is the durable graph identity.
        second_component = self.component(
            11, 9, center_x=18.6, center_y=8.8, width=8.0, height=0.6,
        )
        same_place, second_tier = memory.enter(
            18.55,
            8.75,
            now=2.0,
            component=second_component,
            entry_portal=(19.25, 9.15),
            physical_xy=(18.56, 8.76),
            physical_entry_portal=(19.33, 9.37),
        )

        self.assertIs(same_place, region)
        self.assertEqual(second_tier, "portal_revisit")
        self.assertEqual(len(memory.regions), 1)

        opposite_component = {
            "epoch": 12,
            "label": 10,
            "cells": 80,
            "center_x": 30.0,
            "center_y": 30.0,
            "min_x": 15.5,
            "max_x": 23.5,
            "min_y": 9.9,
            "max_y": 10.5,
        }
        opposite, opposite_tier = memory.enter(
            19.5,
            10.2,
            now=3.0,
            component=opposite_component,
            entry_portal=(19.25, 9.15),
            physical_xy=(19.5, 13.0),
            physical_entry_portal=(19.33, 9.37),
        )

        self.assertIsNot(opposite, region)
        self.assertEqual(opposite_tier, "new")
        self.assertEqual(len(memory.regions), 2)

    def test_physical_viewpoint_matches_across_slam_epochs(self):
        """Odom evidence identifies a closed room after map correction."""
        memory = self.make_memory()
        before = self.component(7, 3, center_x=19.0, center_y=3.0)
        after = self.component(8, 11, center_x=9.0, center_y=11.0)
        region, _tier = memory.enter(
            19.0,
            3.0,
            now=1.0,
            component=before,
            physical_xy=(5.0, 4.0),
        )
        memory.observe(
            19.0,
            3.0,
            40.0,
            robot_x=19.0,
            robot_y=3.0,
            now=2.0,
            component=before,
            region_id=region["id"],
            observation_ready=True,
            physical_robot_xy=(5.0, 4.0),
        )
        memory.dormant(region, now=3.0, reason="place_observation_complete")

        # The component lookup deliberately returns no bridge: the component
        # moved too far in map coordinates for the old signature heuristic.
        memory.refresh_components(
            lambda _x, _y: None,
            map_from_physical_xy=lambda x, y: (x + 4.0, y + 7.0),
        )
        tier, matched = memory.candidate_tier(
            9.1,
            11.1,
            20.0,
            component=after,
            physical_xy=(5.1, 4.1),
        )

        self.assertEqual(region["viewpoints"], [(9.0, 11.0)])
        self.assertEqual(tier, "dormant")
        self.assertEqual(matched["id"], region["id"])

    def test_physical_coverage_refuses_a_portal_endpoint_without_merging_places(self):
        """Coverage admission may use odom evidence without changing identity."""
        memory = self.make_memory()
        source = self.component(21, 3, center_x=5.0, center_y=5.0)
        split = self.component(21, 4, center_x=6.0, center_y=5.0)
        region, _tier = memory.enter(
            5.0,
            5.0,
            now=1.0,
            component=source,
            physical_xy=(5.0, 5.0),
        )
        memory.observe(
            5.0,
            5.0,
            20.0,
            robot_x=5.0,
            robot_y=5.0,
            now=2.0,
            component=source,
            region_id=region["id"],
            observation_ready=True,
            physical_robot_xy=(5.0, 5.0),
        )
        memory.endpoint_observed(
            5.0,
            5.0,
            now=3.0,
            component=source,
            region_id=region["id"],
        )

        # Same-epoch labels remain distinct for identity association.
        self.assertEqual(
            memory.candidate_tier(
                5.2, 5.0, 20.0, component=split, physical_xy=(5.2, 5.0),
            )[0],
            "new",
        )
        # Portal admission can still see that the physical endpoint was
        # already observed, without changing either region label.
        self.assertEqual(
            memory.physical_coverage_region((5.2, 5.0), 1.25)["id"],
            region["id"],
        )

    def test_physical_match_never_crosses_distinct_current_components(self):
        """A same-epoch wall label takes precedence over nearby odom points."""
        memory = self.make_memory()
        room_a = self.component(9, 3, center_x=5.0, center_y=4.0)
        room_b = self.component(9, 4, center_x=7.0, center_y=4.0)
        region, _tier = memory.enter(
            5.0,
            4.0,
            now=1.0,
            component=room_a,
            physical_xy=(5.0, 4.0),
        )
        memory.observe(
            5.0,
            4.0,
            20.0,
            robot_x=5.0,
            robot_y=4.0,
            now=2.0,
            component=room_a,
            region_id=region["id"],
            observation_ready=True,
            physical_robot_xy=(5.0, 4.0),
        )
        memory.dormant(region, now=3.0, reason="place_observation_complete")

        tier, matched = memory.candidate_tier(
            50.0,
            50.0,
            20.0,
            component=room_b,
            physical_xy=(5.1, 4.0),
        )

        self.assertEqual(tier, "new")
        self.assertIsNone(matched)

    def test_route_place_hops_counts_only_structural_portal_transitions(self):
        # Labels name high-clearance interiors; a zero-labelled doorway throat
        # is transparent. The path therefore represents one transition from
        # the source place to its adjacent neighbour, followed by another
        # transition into the next room.
        steps = np.arange(9, dtype=np.int32).reshape(1, 9)
        labels = np.array([[1, 1, 0, 2, 2, 0, 3, 3, 3]], dtype=np.int32)

        self.assertEqual(route_place_hops(labels, steps, 0, 1, 1), 0)
        self.assertEqual(route_place_hops(labels, steps, 0, 4, 1), 1)
        self.assertEqual(route_place_hops(labels, steps, 0, 8, 1), 2)

    def test_route_place_hops_refuses_an_unanchored_route(self):
        # Without a source label, seeing B -> C is insufficient evidence that
        # the route did not already enter B from an earlier place. Reporting
        # a smaller hop count here would let a remote global frontier bypass
        # the one-adjacent-place exploration contract.
        steps = np.arange(7, dtype=np.int32).reshape(1, 7)
        labels = np.array([[0, 2, 2, 0, 3, 3, 3]], dtype=np.int32)

        self.assertIsNone(route_place_hops(labels, steps, 0, 6, None))

    def test_first_route_place_portal_splits_a_multi_place_route_at_the_door(self):
        # The global boundary is two rooms away. The graph must not publish
        # that remote endpoint; it should first send a collision-safe action
        # to the doorway throat immediately after leaving place 1.
        steps = np.arange(9, dtype=np.int32).reshape(1, 9)
        labels = np.array([[1, 1, 0, 2, 2, 0, 3, 3, 3]], dtype=np.int32)

        self.assertEqual(first_route_place_portal(labels, steps, 0, 8, 1), (0, 2))
        self.assertIsNone(first_route_place_portal(labels, steps, 0, 8, None))

    def test_portal_transition_goal_lands_inside_the_adjacent_place(self):
        # The doorway throat at index 2 is topologically useful but too near
        # to be a stable move_base goal. A graph-edge action must finish at
        # the first safe core cell of place 2, then let the next planning
        # cycle decide whether to cover it or cross another doorway.
        steps = np.arange(9, dtype=np.int32).reshape(1, 9)
        labels = np.array([[1, 1, 0, 2, 2, 0, 3, 3, 3]], dtype=np.int32)

        self.assertEqual(
            first_route_place_transition_goal(labels, steps, 0, 8, 1),
            (0, 3),
        )
        self.assertIsNone(
            first_route_place_transition_goal(labels, steps, 0, 8, None),
        )

    def test_portal_helpers_recover_when_source_pose_is_in_an_unlabelled_throat(self):
        # During online SLAM the robot may be at a doorway cell that has not
        # yet acquired the higher-clearance source-place label. The validated
        # route still proves the first labelled core is the next graph edge.
        steps = np.arange(7, dtype=np.int32).reshape(1, 7)
        labels = np.array([[0, 0, 0, 2, 2, 0, 3]], dtype=np.int32)

        self.assertEqual(
            first_route_place_portal(labels, steps, 0, 6, 1),
            (0, 3),
        )
        self.assertEqual(
            first_route_place_transition_goal(labels, steps, 0, 6, 1),
            (0, 3),
        )

    def test_structural_graph_can_generate_a_next_place_without_frontier_points(self):
        steps = np.arange(9, dtype=np.int32).reshape(1, 9)
        labels = np.array([[1, 1, 0, 2, 2, 0, 3, 3, 3]], dtype=np.int32)

        self.assertEqual(
            adjacent_place_transition_goals(labels, steps, 1),
            [(0, 3, 2)],
        )
        self.assertEqual(
            adjacent_place_transition_goals(labels, steps, 1, closed_labels={2}),
            [],
        )

    def test_closed_observation_submap_stops_at_walls(self):
        # A persistent place stores viewpoints, then rebuilds their observed
        # footprint from the latest map. The footprint may cover a room but
        # cannot leak through the wall into the neighbouring room.
        known_free = np.ones((11, 13), dtype=bool)
        known_free[:, 6] = False

        footprint = grid_visible_free_footprint(
            known_free, [(5, 3)], max_range_cells=10,
        )

        self.assertTrue(footprint[5, 3])
        self.assertTrue(footprint[2, 5])
        self.assertFalse(footprint[5, 6])
        self.assertFalse(footprint[5, 9])

    def test_closed_submap_blocks_transit_when_component_labels_change(self):
        # This regression models a SLAM snapshot whose structural component
        # labels have changed. The closed room's visibility footprint remains
        # a durable submap barrier even with no matching label.
        steps = np.tile(np.arange(7, dtype=np.int32), (5, 1))
        labels = np.zeros((5, 7), dtype=np.int32)
        footprint = np.zeros((5, 7), dtype=bool)
        footprint[:, 3] = True

        reachable = grid_reachable_without_closed_places(
            labels,
            steps,
            source_label=None,
            closed_labels=set(),
            closed_footprint=footprint,
        )

        self.assertFalse(reachable[2, 6])

    def test_target_pursuit_pool_names_match_the_allocated_score_keys(self):
        # A target-route recovery reads pools named
        # ``pursuit_structured_best`` and ``pursuit_fallback_best``. This
        # regression guards the path that previously built
        # ``pursuitstructured_best`` and stalled the frontier timer.
        self.assertEqual(frontier_score_bucket_prefixes(True), ("pursuit_", ""))
        self.assertEqual(frontier_score_bucket_prefixes(False), ("",))

    def test_current_map_rehydration_merges_an_early_legacy_room_region(self):
        # At SLAM startup an entered room can lack a core label. Once the map
        # exposes that core, persistent region memory must bind the old
        # endpoint to the *current* epoch before scoring a new frontier from
        # the same room; otherwise the later endpoint is incorrectly novel.
        memory = self.make_memory()
        region, tier = memory.activate(3.55, 16.15, 120, now=0.0)
        self.assertEqual(tier, "new")
        memory.dormant(region, now=1.0, reason="endpoint_completed")
        conference = self.component(24, 7, center_x=4.8, center_y=17.5)

        refreshed = memory.refresh_components(lambda _x, _y: conference)

        self.assertEqual(refreshed, [region["id"]])
        self.assertEqual(
            memory.candidate_tier(6.25, 19.85, 174, component=conference)[0],
            "dormant",
        )

    def test_rehydration_uses_a_reached_viewpoint_before_a_frontier_endpoint(self):
        # The endpoint can be an unlabelled doorway/unknown-boundary cell in
        # the next SLAM snapshot. The robot's reached viewpoint is a durable
        # physical fact inside the place and must preserve the covered-room
        # identity used to reject a later re-entry.
        memory = self.make_memory()
        region, _ = memory.activate(3.55, 16.15, 120, now=0.0)
        memory.observe(
            3.55,
            16.15,
            120,
            robot_x=4.80,
            robot_y=17.50,
            now=1.0,
            observation_ready=True,
        )
        memory.endpoint_observed(3.55, 16.15, now=2.0, region_id=region["id"])
        memory.dormant(region, now=3.0, reason="place_observation_complete")
        conference = self.component(24, 7, center_x=4.8, center_y=17.5)

        refreshed = memory.refresh_components(
            lambda x, y: conference
            if math.hypot(x - 4.80, y - 17.50) < 0.1
            else None
        )

        self.assertEqual(refreshed, [region["id"]])
        self.assertEqual(region["component"]["label"], 7)
        self.assertEqual(
            memory.candidate_tier(6.25, 19.85, 174, component=conference)[0],
            "dormant",
        )

    def test_rehydration_keeps_a_region_unlabeled_when_current_map_has_no_safe_core(self):
        memory = self.make_memory()
        region, _ = memory.activate(3.55, 16.15, 120, now=0.0)

        self.assertEqual(memory.refresh_components(lambda _x, _y: None), [])
        self.assertIsNone(region["component"])

    def test_local_egress_rebind_cannot_mint_a_second_place(self):
        # A stalled viewpoint may retreat through known free space while SLAM
        # changes the structural core label. That is still the same room: the
        # new core must attach to the original durable place node.
        memory = self.make_memory()
        first_component = self.component(40, 3, center_x=16.0, center_y=16.0)
        recovered_component = self.component(41, 8, center_x=14.0, center_y=14.0)
        region, tier = memory.activate(
            16.0, 16.0, 80, now=0.0, component=first_component
        )
        self.assertEqual(tier, "new")

        rebound = memory.rebind_after_local_egress(
            region["id"], 14.0, 14.0, now=5.0, component=recovered_component
        )

        self.assertIs(rebound, region)
        self.assertEqual(region["last_association"], "local_egress_rebind")
        self.assertEqual(
            memory.candidate_tier(13.5, 14.5, 50, component=recovered_component)[0],
            "revisit",
        )
        selected, tier = memory.activate(
            13.5, 14.5, 50, now=6.0, component=recovered_component
        )
        self.assertIs(selected, region)
        self.assertEqual(tier, "revisit")
        self.assertEqual(len(memory.regions), 1)

    @staticmethod
    def component(
        epoch, label, center_x, center_y, cells=80, width=2.0, height=2.0,
    ):
        return {
            "epoch": epoch,
            "label": label,
            "cells": cells,
            "center_x": center_x,
            "center_y": center_y,
            "min_x": center_x - width / 2.0,
            "max_x": center_x + width / 2.0,
            "min_y": center_y - height / 2.0,
            "max_y": center_y + height / 2.0,
        }

    def test_observed_region_cannot_be_reentered_from_another_current_place(self):
        memory = self.make_memory()
        source_room = self.component(31, 4, center_x=4.0, center_y=4.0)
        adjacent_room = self.component(31, 5, center_x=8.0, center_y=4.0)
        region, _ = memory.activate(
            4.0, 4.0, 60, now=0.0, component=adjacent_room
        )
        memory.observe(
            4.0,
            4.0,
            60,
            robot_x=4.0,
            robot_y=4.0,
            now=1.0,
            component=adjacent_room,
            observation_ready=True,
        )
        memory.endpoint_observed(
            4.0, 4.0, now=2.0, component=adjacent_room, region_id=region["id"]
        )

        self.assertTrue(
            memory.is_observed_region_reentry(
                region, source_room, adjacent_room
            )
        )
        self.assertFalse(
            memory.is_observed_region_reentry(
                region, adjacent_room, adjacent_room
            )
        )
        old_snapshot = self.component(30, 5, center_x=8.0, center_y=4.0)
        self.assertFalse(
            memory.is_observed_region_reentry(region, source_room, old_snapshot)
        )

    def test_new_region_is_distinct_from_revisit(self):
        memory = self.make_memory()
        self.assertEqual(memory.candidate_tier(1.0, 1.0, 60)[0], "new")
        region, tier = memory.activate(1.0, 1.0, 60, now=0.0)
        self.assertEqual(tier, "new")
        self.assertEqual(region["visits"], 1)
        memory.activate(1.0, 1.0, 60, now=1.0)
        self.assertEqual(region["visits"], 1)
        self.assertEqual(region["route_dispatches"], 2)
        self.assertEqual(memory.candidate_tier(2.0, 1.0, 60)[0], "revisit")
        self.assertEqual(memory.candidate_tier(5.0, 1.0, 60)[0], "new")

    def test_portal_arrival_creates_an_entered_but_unobserved_place(self):
        memory = self.make_memory()
        destination = self.component(41, 7, center_x=8.0, center_y=4.0)

        region, tier = memory.enter(
            8.0, 4.0, now=5.0, component=destination
        )

        self.assertEqual(tier, "new")
        self.assertEqual(region["entered_at"], 5.0)
        self.assertIsNone(region["observation_started_at"])
        self.assertEqual(region["portal_arrivals"], 1)
        self.assertEqual(
            memory.candidate_tier(8.2, 4.0, 10.0, component=destination)[0],
            "revisit",
        )
        # Passing through a doorway is not a no-information observation
        # session, even when another action starts much later.
        self.assertIsNone(
            memory.stagnant(8.0, 4.0, 8.0, 4.0, now=120.0, component=destination)
        )
        # It is still a true physical place entry, so a later verified exit
        # may close this transit-only room and keep the graph from re-entering.
        memory.close(
            8.0,
            4.0,
            now=121.0,
            reason="crossed_to_next_place",
            component=destination,
            region_id=region["id"],
        )
        self.assertEqual(region["state"], "dormant")

    def test_dormant_region_stays_excluded_despite_unknown_count_noise(self):
        memory = self.make_memory()
        region, _ = memory.activate(1.0, 1.0, 60, now=0.0)
        self.assertTrue(memory.dormant(region, now=1.0, reason="no_information_progress"))
        self.assertEqual(memory.candidate_tier(1.5, 1.0, 67)[0], "dormant")
        self.assertEqual(memory.candidate_tier(1.5, 1.0, 200)[0], "dormant")
        selected, tier = memory.activate(1.5, 1.0, 200, now=2.0)
        self.assertIsNone(selected)
        self.assertEqual(tier, "dormant")

    def test_information_progress_prevents_false_dwell_expiry(self):
        memory = self.make_memory()
        memory.activate(1.0, 1.0, 60, now=0.0)
        memory.observe(
            1.0, 1.0, 60, robot_x=1.0, robot_y=1.0, now=0.0,
            observation_ready=True,
        )
        memory.observe(
            1.0, 1.0, 50, robot_x=1.0, robot_y=1.0, now=10.0,
            observation_ready=True,
        )
        self.assertIsNone(memory.stagnant(1.0, 1.0, 1.0, 1.0, now=21.0))
        self.assertIsNotNone(memory.stagnant(1.0, 1.0, 1.0, 1.0, now=22.1))

    def test_near_route_endpoint_does_not_start_dwell_before_observation(self):
        # Region identity radius is intentionally broad enough to group room
        # frontiers. A route that starts within that radius has not thereby
        # entered the room or observed it, so it must not become dormant while
        # the controller is still approaching its actual viewpoint.
        memory = self.make_memory()
        region, _ = memory.activate(3.05, 12.55, 135, now=0.0)
        memory.observe(
            3.05,
            12.55,
            135,
            robot_x=2.80,
            robot_y=11.09,
            now=0.1,
            observation_ready=False,
        )

        self.assertIsNone(region["entered_at"])
        self.assertIsNone(
            memory.stagnant(3.05, 12.55, 2.80, 11.09, now=60.0)
        )

        memory.observe(
            3.05,
            12.55,
            135,
            robot_x=3.05,
            robot_y=12.55,
            now=61.0,
            observation_ready=True,
        )
        self.assertEqual(region["entered_at"], 61.0)
        self.assertIsNone(
            memory.stagnant(3.05, 12.55, 3.05, 12.55, now=72.9)
        )
        self.assertIsNotNone(
            memory.stagnant(3.05, 12.55, 3.05, 12.55, now=73.1)
        )

    def test_new_endpoint_session_does_not_reset_place_no_gain_epoch(self):
        # Several endpoint actions can cover one room. A new controller action
        # must not restart the place lifecycle: otherwise endpoint churn keeps
        # a room open forever without observing any new information.
        memory = self.make_memory()
        memory.activate(10.0, 6.0, 80, now=0.0)
        memory.observe(
            10.0,
            6.0,
            80,
            robot_x=10.0,
            robot_y=6.0,
            now=1.0,
            observation_ready=True,
        )
        self.assertIsNotNone(
            memory.stagnant(10.0, 6.0, 10.0, 6.0, now=13.1)
        )

        memory.observe(
            12.0,
            6.0,
            75,
            robot_x=12.0,
            robot_y=6.0,
            now=20.0,
            observation_ready=True,
            observation_session_started=True,
        )
        self.assertIsNotNone(
            memory.stagnant(12.0, 6.0, 12.0, 6.0, now=20.0)
        )

    def test_route_failures_do_not_close_an_unobserved_physical_place(self):
        memory = self.make_memory()
        memory.activate(1.0, 1.0, 60, now=0.0)
        first = memory.fail(1.0, 1.0, now=1.0, reason="stall")
        self.assertEqual(first["state"], "open")
        second = memory.fail(1.0, 1.0, now=2.0, reason="stall")
        self.assertEqual(second["state"], "open")
        self.assertEqual(second["failures"], 2)
        self.assertEqual(second["last_reason"], "viewpoint_attempt_failed:stall")
        self.assertEqual(memory.candidate_tier(1.0, 1.0, 60)[0], "revisit")

    def test_far_frontiers_with_one_topology_component_are_one_region(self):
        memory = self.make_memory()
        # The endpoints are much farther apart than the 2.5 m geometric
        # radius. Exact same-snapshot topology-component identity must still
        # make them one room-scale observation region.
        first = self.component(7, 11, center_x=8.0, center_y=8.0)
        same_room = self.component(7, 11, center_x=8.0, center_y=8.0)
        region, tier = memory.activate(1.0, 1.0, 60, now=0.0, component=first)
        self.assertEqual(tier, "new")
        self.assertEqual(
            memory.candidate_tier(7.0, 6.0, 60, component=same_room)[0],
            "revisit",
        )
        revisited, tier = memory.activate(
            7.0, 6.0, 60, now=1.0, component=same_room,
        )
        self.assertEqual(tier, "revisit")
        self.assertEqual(revisited["id"], region["id"])
        self.assertEqual(revisited["last_association"], "component_exact")

    def test_near_frontiers_behind_distinct_components_are_not_merged(self):
        memory = self.make_memory()
        # A doorway can put two safe approach cells inside the old 2.5 m
        # radius. Distinct same-snapshot topology components prove that these
        # are different exploration branches, so geometry must not merge them.
        lobby = self.component(8, 3, center_x=1.0, center_y=5.0)
        conference = self.component(8, 4, center_x=3.0, center_y=5.0)
        memory.activate(1.0, 1.0, 60, now=0.0, component=lobby)
        self.assertEqual(
            memory.candidate_tier(2.0, 1.0, 60, component=conference)[0],
            "new",
        )

    def test_completed_room_envelope_rejects_a_near_furniture_split_core(self):
        # A table can split one physical room into two high-clearance cores.
        # Once the first core has actually been inspected and retired, a
        # nearby second core must not make the vehicle leave and later reenter
        # that same room. This fallback does not apply while the first region
        # is still open (covered by the previous test).
        memory = self.make_memory()
        first_core = self.component(15, 2, center_x=4.0, center_y=16.0)
        table_split_core = self.component(15, 7, center_x=4.5, center_y=17.8)
        region, _ = memory.activate(3.55, 16.15, 100, now=0.0, component=first_core)
        memory.dormant(region, now=1.0, reason="endpoint_completed")

        tier, matched = memory.candidate_tier(
            4.45, 17.85, 60, component=table_split_core,
        )

        self.assertEqual(tier, "dormant")
        self.assertEqual(matched["id"], region["id"])
        self.assertEqual(matched["last_association"], "new")

    def test_route_bound_completion_cannot_retire_nearby_region(self):
        # Before room cores form, two route endpoints can be inside the legacy
        # geometric radius. Completion must close the region selected for that
        # route, not whichever endpoint happens to be nearest at callback time.
        memory = self.make_memory()
        first, _ = memory.activate(3.55, 16.15, 80, now=0.0)
        second, _ = memory.activate(6.55, 16.15, 80, now=1.0)
        self.assertNotEqual(first["id"], second["id"])
        memory.observe(
            6.55, 16.15, 70, robot_x=6.55, robot_y=16.15, now=2.0,
            region_id=second["id"], observation_ready=True,
        )
        completed = memory.complete(
            6.55, 16.15, now=3.0, region_id=second["id"],
        )
        self.assertEqual(completed["id"], second["id"])
        self.assertEqual(first["completions"], 0)
        self.assertEqual(second["completions"], 1)

    def test_component_signature_bridges_a_slam_update(self):
        memory = self.make_memory()
        before = self.component(3, 6, center_x=10.0, center_y=10.0, cells=100)
        after = self.component(4, 2, center_x=10.6, center_y=9.8, cells=82)
        memory.activate(1.0, 1.0, 60, now=0.0, component=before)
        self.assertEqual(
            memory.candidate_tier(6.0, 5.0, 60, component=after)[0],
            "revisit",
        )

    def test_dormant_component_excludes_far_same_room_endpoint(self):
        memory = self.make_memory()
        component = self.component(11, 9, center_x=12.0, center_y=6.0)
        region, _ = memory.activate(1.0, 1.0, 60, now=0.0, component=component)
        memory.dormant(region, now=1.0, reason="no_information_progress")
        self.assertEqual(
            memory.candidate_tier(7.5, 5.0, 120, component=component)[0],
            "dormant",
        )

    def test_completed_entered_room_becomes_dormant_without_new_boundary(self):
        memory = self.make_memory()
        room = self.component(17, 5, center_x=12.0, center_y=6.0)
        region, _ = memory.activate(10.0, 6.0, 80, now=0.0, component=room)
        # The dwell must start from physical presence, rather than from the
        # earlier route assignment, before terminal completion may retire it.
        memory.observe(
            10.0, 6.0, 72, robot_x=10.0, robot_y=6.0, now=4.0,
            component=room, observation_ready=True,
        )
        completed = memory.complete(10.0, 6.0, now=5.0, component=room)
        self.assertEqual(completed["id"], region["id"])
        self.assertEqual(completed["state"], "dormant")
        self.assertEqual(
            memory.candidate_tier(15.0, 6.0, 90, component=room)[0],
            "dormant",
        )

    def test_endpoint_observation_keeps_local_coverage_open_until_departure(self):
        room = self.component(17, 5, center_x=12.0, center_y=6.0)
        memory = self.make_memory()
        region, _ = memory.activate(10.0, 6.0, 80, now=0.0, component=room)
        memory.observe(
            10.0, 6.0, 72, robot_x=10.0, robot_y=6.0, now=1.0,
            component=room, observation_ready=True,
        )

        observed = memory.endpoint_observed(
            10.0, 6.0, now=2.0, component=room, region_id=region["id"],
        )

        self.assertEqual(observed["state"], "open")
        self.assertEqual(observed["endpoint_observations"], 1)

    def test_suspended_place_reopens_only_when_local_work_is_selected(self):
        """Novel branch departure preserves a resumable Place identity."""
        room = self.component(31, 6, center_x=12.0, center_y=6.0)
        memory = self.make_memory()
        region, _tier = memory.activate(
            10.0, 6.0, 80, now=0.0, component=room,
        )
        memory.endpoint_observed(
            10.0, 6.0, now=1.0, component=room, region_id=region["id"],
        )

        self.assertTrue(
            memory.suspend(region, now=2.0, reason="novel_portal_branch_committed")
        )
        self.assertEqual(region["state"], "suspended")
        self.assertEqual(
            memory.candidate_tier(11.0, 6.0, 70, component=room)[0],
            "revisit",
        )

        reopened, tier = memory.activate(
            11.0, 6.0, 70, now=3.0, component=room,
        )
        self.assertIs(reopened, region)
        self.assertEqual(tier, "revisit")
        self.assertEqual(reopened["state"], "open")

    def test_resume_is_the_only_named_suspended_to_open_transition(self):
        memory = self.make_memory()
        room = self.component(33, 6, center_x=12.0, center_y=6.0)
        region, _tier = memory.activate(
            10.0, 6.0, 80, now=0.0, component=room,
        )
        memory.suspend(region, now=1.0)

        self.assertTrue(memory.resume(region, now=2.0))
        self.assertEqual(region["state"], "open")
        self.assertEqual(region["last_reason"], "suspended_place_reopened")
        self.assertFalse(memory.resume(region, now=3.0))

    def test_terminal_observation_establishes_unentered_place_evidence(self):
        """A matching controller terminal must not wait for a planner tick."""
        room = self.component(28, 6, center_x=8.0, center_y=4.0)
        memory = self.make_memory()
        region, _tier = memory.activate(8.0, 4.0, 80, now=1.0, component=room)

        observed = memory.endpoint_observed(
            8.2,
            4.1,
            now=5.0,
            component=room,
            region_id=region["id"],
            physical_xy=(3.2, -1.1),
        )

        self.assertEqual(observed["entered_at"], 5.0)
        self.assertEqual(observed["observation_started_at"], 5.0)
        self.assertEqual(observed["endpoint_observations"], 1)
        self.assertEqual(observed["viewpoints"], [(8.2, 4.1)])
        self.assertEqual(observed["physical_viewpoints"], [(3.2, -1.1)])
        self.assertEqual(
            memory.candidate_tier(8.1, 4.0, 20, component=room)[0],
            "revisit",
        )
        self.assertEqual(
            memory.candidate_tier(15.0, 6.0, 90, component=room)[0],
            "revisit",
        )
        selected, tier = memory.activate(
            15.0, 6.0, 90, now=2.5, component=room,
        )
        self.assertIs(selected, region)
        self.assertEqual(tier, "revisit")
        closed = memory.close(
            10.0,
            6.0,
            now=3.0,
            reason="no_local_executable_frontier",
            component=room,
            region_id=region["id"],
        )
        self.assertEqual(closed["state"], "dormant")
        self.assertEqual(closed["last_reason"], "no_local_executable_frontier")

    def test_covered_place_visibility_blocks_a_furniture_split_but_not_a_wall(self):
        class CoveredVisibilityStub:
            candidate_covered_by_covered_place_viewpoint = VISIBILITY_METHODS[
                "candidate_covered_by_covered_place_viewpoint"
            ]

            def __init__(self):
                self.scan_range_max = 10.0
                self.region_memory = SimpleNamespace(
                    covered_viewpoints=lambda: [(9, [(1.5, 3.5)])]
                )

            @staticmethod
            def xy_to_grid_cell(_message, x, y):
                return int(y), int(x)

        message = SimpleNamespace(
            info=SimpleNamespace(
                resolution=1.0,
                width=9,
                height=7,
                origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
            )
        )
        manager = CoveredVisibilityStub()
        known_free = np.ones((7, 9), dtype=bool)
        self.assertEqual(
            manager.candidate_covered_by_covered_place_viewpoint(
                message, known_free, 3, 7, 7.5, 3.5,
            ),
            (9, (1.5, 3.5)),
        )
        # A physical wall must prevent a covered-place footprint from suppressing
        # an actually distinct room on the other side.
        known_free[:, 4] = False
        self.assertIsNone(
            manager.candidate_covered_by_covered_place_viewpoint(
                message, known_free, 3, 7, 7.5, 3.5,
            )
        )

    def test_completed_place_closes_despite_local_unknown_expansion(self):
        memory = self.make_memory()
        room = self.component(18, 4, center_x=12.0, center_y=6.0)
        memory.activate(10.0, 6.0, 60, now=0.0, component=room)
        memory.observe(
            10.0, 6.0, 60, robot_x=10.0, robot_y=6.0, now=1.0,
            component=room, observation_ready=True,
        )
        # Online SLAM can expose more boundary pixels around furniture after
        # the endpoint was already selected. That is diagnostic progress, not
        # a second room session: otherwise a static room reopens forever as
        # scan matching changes its local frontier.
        memory.observe(
            10.0, 6.0, 70, robot_x=10.0, robot_y=6.0, now=2.0,
            component=room, observation_ready=True,
        )
        first = memory.complete(10.0, 6.0, now=3.0, component=room)
        self.assertEqual(first["state"], "dormant")
        self.assertEqual(first["last_reason"], "place_observation_complete")
        self.assertEqual(
            memory.candidate_tier(14.0, 6.0, 70, component=room)[0],
            "dormant",
        )

    def test_target_claim_explicitly_retains_an_observed_place(self):
        memory = self.make_memory()
        room = self.component(18, 4, center_x=12.0, center_y=6.0)
        memory.activate(10.0, 6.0, 60, now=0.0, component=room)
        memory.observe(
            10.0, 6.0, 55, robot_x=10.0, robot_y=6.0, now=1.0,
            component=room, observation_ready=True,
        )

        retained = memory.complete(
            10.0,
            6.0,
            now=2.0,
            component=room,
            retain_for_target=True,
        )

        self.assertEqual(retained["state"], "open")
        self.assertEqual(
            retained["last_reason"], "target_claim_requires_more_observation"
        )

    def test_terminal_without_physical_entry_does_not_blacklist_room(self):
        memory = self.make_memory()
        room = self.component(19, 2, center_x=12.0, center_y=6.0)
        memory.activate(10.0, 6.0, 60, now=0.0, component=room)
        region = memory.complete(10.0, 6.0, now=1.0, component=room)
        self.assertEqual(region["state"], "open")
        self.assertEqual(region["last_reason"], "route_terminal_without_region_entry")

    def test_new_terminal_branch_outranks_a_prefetched_revisit(self):
        # The cached successor is selected before the robot's final scan at
        # the current endpoint.  When that scan exposes another room, its
        # component must remain "new" while the cached same-room successor
        # stays a revisit. The explorer uses precisely these tiers to replace
        # the cache at the terminal transition.
        memory = self.make_memory()
        inspected_room = self.component(15, 4, center_x=4.0, center_y=4.0)
        newly_visible_room = self.component(16, 7, center_x=9.0, center_y=4.0)
        memory.activate(3.0, 3.0, 80, now=0.0, component=inspected_room)
        self.assertEqual(
            memory.candidate_tier(
                5.0, 5.0, 70, component=inspected_room,
            )[0],
            "revisit",
        )
        self.assertEqual(
            memory.candidate_tier(
                9.0, 4.0, 90, component=newly_visible_room,
            )[0],
            "new",
        )

    def test_topology_component_index_keeps_diagonal_rooms_separate(self):
        # Diagonal cells must not connect through a wall corner. This is the
        # key distinction between indoor topology and an 8-connected image.
        core = np.zeros((5, 5), dtype=bool)
        core[1, 1] = True
        core[2, 2] = True
        core[1, 3] = True
        core[2, 3] = True
        index = TopologicalFreeSpaceComponents(core, epoch=1)
        self.assertNotEqual(index.labels[1, 1], index.labels[2, 2])
        self.assertEqual(index.labels[1, 3], index.labels[2, 3])

    def test_route_evidence_inherits_the_first_validated_topology_core(self):
        # A frontier endpoint at the edge of recently scanned free space can
        # have no core cell locally. Its shortest known-free route still
        # reaches the conference-room core before crossing any doorway.
        core = np.zeros((3, 6), dtype=bool)
        core[1, 1] = True
        index = TopologicalFreeSpaceComponents(core, epoch=2)
        steps = np.full((3, 6), -1, dtype=np.int32)
        steps[1, :] = np.arange(6, dtype=np.int32)
        evidence = index.route_evidence(steps, row=1, col=5, max_steps=5)
        self.assertIsNotNone(evidence)
        self.assertEqual(evidence["label"], int(index.labels[1, 1]))
        self.assertIsNone(index.route_evidence(steps, row=1, col=5, max_steps=2))

    def test_known_free_visibility_ray_rejects_unknown_and_occupied_occlusion(self):
        known_free = np.ones((7, 7), dtype=bool)
        self.assertTrue(grid_line_is_known_free(known_free, 1, 1, 5, 5))
        # Unknown map cells are not a visibility proof. A future frontier
        # behind this gap must remain eligible for inspection.
        known_free[3, 3] = False
        self.assertFalse(grid_line_is_known_free(known_free, 1, 1, 5, 5))
        self.assertFalse(grid_line_is_known_free(known_free, -1, 1, 5, 5))

    def test_observation_certificate_completes_a_resolved_endpoint_at_standoff(self):
        # A frontier's map cell can be beside a wall. The base need not touch
        # that geometric endpoint once a current lidar viewpoint has a clear
        # free-space ray to it and the surrounding boundary is fully mapped.
        known_free = np.ones((9, 9), dtype=bool)
        unknown = np.zeros((9, 9), dtype=bool)

        self.assertTrue(
            grid_frontier_observed_from_viewpoint(
                known_free, unknown, 4, 2, 4, 6, unknown_halo_cells=2,
            )
        )

    def test_observation_certificate_keeps_doorway_and_occluded_endpoint_open(self):
        known_free = np.ones((9, 9), dtype=bool)
        unknown = np.zeros((9, 9), dtype=bool)
        # Unknown beyond the endpoint is an unobserved continuation, not a
        # dead end. It must remain an active observation task.
        unknown[4, 8] = True
        self.assertFalse(
            grid_frontier_observed_from_viewpoint(
                known_free, unknown, 4, 2, 4, 6, unknown_halo_cells=2,
            )
        )
        unknown[:, :] = False
        known_free[4, 4] = False
        self.assertFalse(
            grid_frontier_observed_from_viewpoint(
                known_free, unknown, 4, 2, 4, 6, unknown_halo_cells=2,
            )
        )

    def test_visibility_certificate_requires_same_topology_and_clear_map_ray(self):
        class Components:
            def __init__(self, labels):
                self.labels = labels

            def nearby_evidence(self, row, col, _radius_cells):
                label = self.labels.get((int(row), int(col)))
                if label is None:
                    return None
                return {"epoch": 7, "label": label}

        class VisibilityStub:
            xy_to_grid_cell = VISIBILITY_METHODS["xy_to_grid_cell"]
            completed_viewpoint_component = VISIBILITY_METHODS[
                "completed_viewpoint_component"
            ]
            candidate_covered_by_completed_viewpoint = VISIBILITY_METHODS[
                "candidate_covered_by_completed_viewpoint"
            ]

            def __init__(self):
                self.scan_range_max = 10.0
                self.completed_frontiers = collections.deque([(1.5, 1.5)])
                self.region_topology_association_radius = 1.0
                self.completed_viewpoint_component_cache = {}

            @staticmethod
            def component_evidence(_message, _components, raw):
                return raw

        message = SimpleNamespace(
            info=SimpleNamespace(
                resolution=1.0,
                width=7,
                height=7,
                origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
            )
        )
        known_free = np.ones((7, 7), dtype=bool)
        components = Components({(1, 1): 4, (5, 5): 4})
        manager = VisibilityStub()
        self.assertEqual(
            manager.candidate_covered_by_completed_viewpoint(
                message, known_free, components, 5, 5, 5.5, 5.5,
            ),
            (1.5, 1.5),
        )
        known_free[3, 3] = False
        self.assertIsNone(
            manager.candidate_covered_by_completed_viewpoint(
                message, known_free, components, 5, 5, 5.5, 5.5,
            )
        )
        components = Components({(1, 1): 4, (5, 5): 8})
        self.assertIsNone(
            manager.candidate_covered_by_completed_viewpoint(
                message, np.ones((7, 7), dtype=bool), components, 5, 5, 5.5, 5.5,
            )
        )


if __name__ == "__main__":
    unittest.main()
