#!/usr/bin/env python3
"""Regression tests for physical portal certification.

The fixtures encode only grid-observable geometry.  They deliberately avoid
Gazebo room names or object annotations: the same raw-free bypass criterion
must distinguish an interior table from a wall with one doorway.
"""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from global_frontier_grid import bfs
from global_frontier_candidate_portals import GlobalFrontierCandidatePortalMixin
from global_frontier_topology_paths import route_predecessor_path
from global_frontier_portal_certification import (
    RoutePlaceTransition,
    certified_route_place_hops,
    certified_route_place_transitions,
    portal_transition_exit_cell,
    portal_transition_verified_exit_cell,
)


class PortalCertificationTest(unittest.TestCase):
    def test_large_table_with_two_raw_free_bypasses_is_not_a_portal(self):
        """A transient high-clearance split must remain one physical room."""
        known_free = np.zeros((25, 30), dtype=bool)
        known_free[1:24, 1:29] = True
        # The table almost spans the room vertically. A route may choose its
        # upper side, but a second known-free passage exists below it.
        known_free[6:19, 12:17] = False
        labels = np.zeros_like(known_free, dtype=np.int32)
        labels[4:21, 2:10] = 1
        labels[4:21, 19:28] = 2
        steps = bfs(known_free, (12, 5))

        transitions = certified_route_place_transitions(
            labels,
            steps,
            known_free,
            12,
            24,
            source_label=1,
            throat_radius_cells=2,
            window_margin_cells=12,
        )

        self.assertEqual(transitions, [])
        self.assertEqual(
            certified_route_place_hops(
                labels,
                steps,
                known_free,
                12,
                24,
                source_label=1,
                throat_radius_cells=2,
                window_margin_cells=12,
            ),
            0,
        )

    def test_wall_with_one_doorway_is_a_certified_portal(self):
        """Removing the local throat of a real door leaves no raw-free path."""
        known_free = np.zeros((25, 30), dtype=bool)
        known_free[1:24, 1:29] = True
        known_free[1:24, 15] = False
        # The 3-cell opening is narrower than the 2-cell structural-clearance
        # disk on both sides, so it is exactly the kind of throat that splits
        # high-clearance place cores while remaining raw-free navigable.
        known_free[11:14, 15] = True
        labels = np.zeros_like(known_free, dtype=np.int32)
        labels[3:22, 2:12] = 1
        labels[3:22, 18:28] = 2
        steps = bfs(known_free, (6, 5))

        transitions = certified_route_place_transitions(
            labels,
            steps,
            known_free,
            6,
            24,
            source_label=1,
            throat_radius_cells=2,
            window_margin_cells=12,
        )

        self.assertEqual(len(transitions), 1)
        self.assertEqual(transitions[0].source_label, 1)
        self.assertEqual(transitions[0].destination_label, 2)
        self.assertEqual(
            certified_route_place_hops(
                labels,
                steps,
                known_free,
                6,
                24,
                source_label=1,
                throat_radius_cells=2,
                window_margin_cells=12,
            ),
            1,
        )

    def test_doorway_requires_long_structural_wall_runs(self):
        """A doorway is an opening in a wall, not merely an aisle cut."""
        known_free = np.zeros((25, 30), dtype=bool)
        known_free[1:24, 1:29] = True
        known_free[1:24, 15] = False
        known_free[11:14, 15] = True
        labels = np.zeros_like(known_free, dtype=np.int32)
        labels[3:22, 2:12] = 1
        labels[3:22, 18:28] = 2
        steps = bfs(known_free, (6, 5))
        structural_occupied = ~known_free

        transitions = certified_route_place_transitions(
            labels,
            steps,
            known_free,
            6,
            24,
            source_label=1,
            throat_radius_cells=2,
            window_margin_cells=12,
            structural_occupied=structural_occupied,
            minimum_wall_span_cells=4,
        )

        self.assertEqual(len(transitions), 1)
        # The route first enters a low-clearance approach at (6, 12), but the
        # actual doorway is the wall opening at (11, 15).  Navigation must
        # retain the latter as the durable physical gate.
        self.assertEqual(transitions[0].portal_cell, (11, 15))

    def test_runtime_wall_support_uses_clearance_not_furniture_span(self):
        """A valid short jamb must survive the runtime evidence adapter."""
        known_free = np.zeros((25, 30), dtype=bool)
        known_free[1:24, 1:29] = True
        known_free[1:24, 15] = False
        known_free[11:14, 15] = True
        labels = np.zeros_like(known_free, dtype=np.int32)
        labels[3:22, 2:12] = 1
        labels[3:22, 18:28] = 2
        steps = bfs(known_free, (6, 5))
        structural_occupied = ~known_free
        selector = GlobalFrontierCandidatePortalMixin()
        selector.region_topology_clearance = 0.52
        selector.place_furniture_max_span_m = 2.50
        request = SimpleNamespace(
            message=SimpleNamespace(info=SimpleNamespace(resolution=0.10)),
            components=SimpleNamespace(structural_occupied=structural_occupied),
        )

        support_kwargs = selector._portal_wall_support_kwargs(
            request.message, request.components,
        )

        self.assertEqual(support_kwargs["minimum_wall_span_cells"], 6)
        transitions = certified_route_place_transitions(
            labels,
            steps,
            known_free,
            6,
            24,
            source_label=1,
            throat_radius_cells=2,
            window_margin_cells=12,
            **support_kwargs,
        )

        self.assertEqual(len(transitions), 1)
        self.assertEqual(transitions[0].portal_cell, (11, 15))

    def test_portal_execution_exit_is_shortly_beyond_the_doorway(self):
        """A portal action ends after one graph edge, not at a remote core."""
        known_free = np.zeros((25, 30), dtype=bool)
        known_free[1:24, 1:29] = True
        known_free[1:24, 15] = False
        known_free[11:14, 15] = True
        labels = np.zeros_like(known_free, dtype=np.int32)
        labels[3:22, 2:12] = 1
        # Make the next core intentionally far from the door. This models a
        # low-clearance entry zone that has not received a structural label.
        labels[3:22, 21:28] = 2
        steps = bfs(known_free, (12, 5))
        transition = certified_route_place_transitions(
            labels,
            steps,
            known_free,
            12,
            24,
            source_label=1,
            throat_radius_cells=2,
            window_margin_cells=12,
        )[0]

        exit_cell = portal_transition_exit_cell(
            steps, transition, minimum_distance_cells=3,
        )
        path = route_predecessor_path(steps, *transition.destination_cell)

        self.assertLess(path.index(exit_cell), path.index(transition.destination_cell))
        self.assertEqual(path.index(exit_cell) - path.index(transition.portal_cell), 3)

    def test_portal_execution_uses_route_continuation_for_goal_tolerance_depth(self):
        """A shallow structural core must not place TEB's success ball at a gate."""
        known_free = np.zeros((25, 30), dtype=bool)
        known_free[1:24, 1:29] = True
        known_free[1:24, 15] = False
        known_free[11:14, 15] = True
        labels = np.zeros_like(known_free, dtype=np.int32)
        labels[3:22, 2:12] = 1
        labels[3:22, 18:28] = 2
        steps = bfs(known_free, (12, 5))
        transition = certified_route_place_transitions(
            labels,
            steps,
            known_free,
            12,
            24,
            source_label=1,
            throat_radius_cells=2,
            window_margin_cells=12,
        )[0]

        exit_cell = portal_transition_exit_cell(
            steps,
            transition,
            minimum_distance_cells=10,
            continuation_cell=(12, 24),
        )
        path = route_predecessor_path(steps, 12, 24)

        self.assertEqual(
            path.index(exit_cell) - path.index(transition.portal_cell), 10,
        )
        self.assertGreater(
            path.index(exit_cell), path.index(transition.destination_cell),
        )

    def test_verified_portal_exit_accounts_for_teb_goal_tolerance(self):
        """The endpoint must be beyond crossing depth plus TEB's goal ball."""
        known_free = np.ones((7, 20), dtype=bool)
        labels = np.zeros_like(known_free, dtype=np.int32)
        labels[:, :6] = 1
        labels[:, 9:] = 2
        steps = bfs(known_free, (3, 2))
        transition = RoutePlaceTransition(
            source_label=1,
            destination_label=2,
            source_cell=(3, 5),
            portal_cell=(3, 6),
            destination_cell=(3, 9),
            throat_cells=((3, 7), (3, 8)),
        )

        # 0.31 m crossing depth + 0.50 m TEB success circle at 0.10 m/cell.
        exit_cell = portal_transition_verified_exit_cell(
            steps,
            transition,
            minimum_signed_depth_cells=8.1,
            continuation_cell=(3, 18),
        )

        self.assertEqual(exit_cell, (3, 15))

    def test_verified_portal_exit_refuses_to_cross_a_second_place(self):
        """A deeper target must never turn one portal action into two edges."""
        known_free = np.ones((5, 16), dtype=bool)
        labels = np.zeros_like(known_free, dtype=np.int32)
        labels[:, :4] = 1
        labels[:, 6:10] = 2
        labels[:, 12:] = 3
        steps = bfs(known_free, (2, 1))
        transition = RoutePlaceTransition(
            source_label=1,
            destination_label=2,
            source_cell=(2, 3),
            portal_cell=(2, 4),
            destination_cell=(2, 6),
            throat_cells=((2, 5),),
        )
        next_transition = RoutePlaceTransition(
            source_label=2,
            destination_label=3,
            source_cell=(2, 9),
            portal_cell=(2, 10),
            destination_cell=(2, 12),
            throat_cells=((2, 11),),
        )

        exit_cell = portal_transition_verified_exit_cell(
            steps,
            transition,
            minimum_signed_depth_cells=8.1,
            continuation_cell=(2, 15),
            next_transition=next_transition,
        )

        self.assertIsNone(exit_cell)

    def test_portal_source_rejects_an_unprovable_endpoint_without_crashing(self):
        """A shallow adjacent place is withheld until mapping exposes depth."""
        known_free = np.ones((5, 12), dtype=bool)
        labels = np.zeros_like(known_free, dtype=np.int32)
        labels[:, :4] = 1
        labels[:, 6:] = 2
        steps = bfs(known_free, (2, 1))
        transition = RoutePlaceTransition(
            source_label=1,
            destination_label=2,
            source_cell=(2, 3),
            portal_cell=(2, 4),
            destination_cell=(2, 6),
            throat_cells=((2, 5),),
        )
        request = SimpleNamespace(
            message=SimpleNamespace(info=SimpleNamespace(resolution=0.10)),
            steps=steps,
            components=SimpleNamespace(labels=labels),
        )
        selector = GlobalFrontierCandidatePortalMixin()
        selector.clearance = 0.52
        selector.teb_xy_goal_tolerance = 0.50

        source = selector._portal_source_for_transition(
            request, transition, 2, 11,
        )

        self.assertIsNone(source)

    def test_short_opposing_obstacles_cannot_certify_a_portal(self):
        """Two furniture-sized supports around an aisle are not a doorway."""
        known_free = np.zeros((25, 30), dtype=bool)
        known_free[1:24, 1:29] = True
        known_free[1:24, 15] = False
        known_free[11:14, 15] = True
        labels = np.zeros_like(known_free, dtype=np.int32)
        labels[3:22, 2:12] = 1
        labels[3:22, 18:28] = 2
        steps = bfs(known_free, (6, 5))
        structural_occupied = np.zeros_like(known_free, dtype=bool)
        structural_occupied[8:11, 15] = True
        structural_occupied[14:17, 15] = True

        transitions = certified_route_place_transitions(
            labels,
            steps,
            known_free,
            6,
            24,
            source_label=1,
            throat_radius_cells=2,
            window_margin_cells=12,
            structural_occupied=structural_occupied,
            minimum_wall_span_cells=4,
        )

        self.assertEqual(transitions, [])

    def test_explicit_doorway_chain_keeps_its_existing_hop_contract(self):
        """The simple three-place topology still exposes two real crossings."""
        labels = np.array([[1, 1, 0, 2, 2, 0, 3, 3, 3]], dtype=np.int32)
        known_free = np.ones_like(labels, dtype=bool)
        steps = np.arange(labels.size, dtype=np.int32).reshape(labels.shape)

        transitions = certified_route_place_transitions(
            labels,
            steps,
            known_free,
            0,
            8,
            source_label=1,
            throat_radius_cells=1,
            window_margin_cells=2,
        )

        self.assertEqual(
            [transition.destination_label for transition in transitions],
            [2, 3],
        )

    def test_unlabelled_source_throat_waits_for_more_map_evidence(self):
        """A source without a visible core cannot certify a portal cut."""
        labels = np.array([[0, 0, 0, 2, 2]], dtype=np.int32)
        known_free = np.ones_like(labels, dtype=bool)
        steps = np.arange(labels.size, dtype=np.int32).reshape(labels.shape)

        transitions = certified_route_place_transitions(
            labels,
            steps,
            known_free,
            0,
            4,
            source_label=1,
            throat_radius_cells=1,
        )

        self.assertEqual(transitions, [])


if __name__ == "__main__":
    unittest.main()
