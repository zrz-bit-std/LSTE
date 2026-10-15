#!/usr/bin/env python3
"""Selection-level tests for the place-graph action contract."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from global_frontier_place_memory import FrontierRegionMemory
from global_frontier_portal_belief import PortalHypothesisLedger
from global_frontier_work_item_components import ObservationSupport
from global_frontier_work_items import PlaceWorkItemLedger, WORK_RESOLVED
from global_frontier_target_observation_work import TargetObservationWorkLedger
sys.modules.setdefault("rospy", SimpleNamespace())

from global_frontier_selection import GlobalFrontierSelectionMixin


class RegionMemory:
    def dormant_component_labels(self, _epoch):
        return set()

    def observed_open_component_labels(self, _epoch):
        return set()

    def candidate_tier(self, _x, _y, _information, component=None):
        return "new", None

    @staticmethod
    def is_observed_region_reentry(_region, _source_component, _candidate_component):
        return False


class ObservedRegionMemory(RegionMemory):
    """Minimal observed-place fixture exercising selector graph blocking."""

    def __init__(self, labels):
        self.labels = set(labels)
        self.region = {
            "id": 8,
            "state": "open",
            "endpoint_observations": 1,
        }

    def observed_open_component_labels(self, _epoch):
        return set(self.labels)

    def candidate_tier(self, _x, _y, _information, component=None):
        return "revisit", self.region

    @staticmethod
    def is_observed_region_reentry(region, source_component, candidate_component):
        return FrontierRegionMemory.is_observed_region_reentry(
            region, source_component, candidate_component
        )


class ObservedSourceMemory(RegionMemory):
    """One source-room observation has completed; adjacent space is new."""

    def __init__(self, source_label):
        self.source_label = int(source_label)
        self.region = {
            "id": 31,
            "state": "open",
            "entered_at": 1.0,
            "endpoint_observations": 1,
        }

    def observed_open_component_labels(self, _epoch):
        return {self.source_label}

    def candidate_tier(self, _x, _y, _information, component=None):
        if component is not None and int(component["label"]) == self.source_label:
            return "revisit", self.region
        return "new", None


class OwnedUnobservedSourceMemory(ObservedSourceMemory):
    """A portal arrival owns the room before an endpoint view is reached."""

    def __init__(self, source_label):
        super().__init__(source_label)
        self.region["endpoint_observations"] = 0

    def by_id(self, region_id):
        return self.region if int(region_id) == int(self.region["id"]) else None


class OwnedObservedSourceMemory(ObservedSourceMemory):
    """A normal observed source place with a durable owner lookup."""

    def by_id(self, region_id):
        return self.region if int(region_id) == int(self.region["id"]) else None


class DormantTransitSourceMemory(RegionMemory):
    """An observed room remains a graph node but never reopens for work."""

    def __init__(self, source_label):
        self.source_label = int(source_label)
        self.region = {
            "id": 37,
            "state": "dormant",
            "entered_at": None,
            "endpoint_observations": 2,
        }

    def by_id(self, region_id):
        return self.region if int(region_id) == int(self.region["id"]) else None

    def dormant_component_labels(self, _epoch):
        # This is the exact regression: the current place also appears in the
        # completed-place mask after a covered portal arrival.
        return {self.source_label}

    def candidate_tier(self, _x, _y, _information, component=None):
        if component is not None and int(component["label"]) == self.source_label:
            return "dormant", self.region
        return "new", None


class ReadyToExitRegionMemory(RegionMemory):
    """Fixture for an endpoint-complete source place awaiting an egress."""

    def __init__(self, source_label):
        self.source_label = int(source_label)
        self.region = {
            "id": 12,
            "state": "ready_to_exit",
            "endpoint_observations": 1,
        }

    def candidate_tier(self, _x, _y, _information, component=None):
        if component is not None and int(component["label"]) == self.source_label:
            return "ready_to_exit", self.region
        return "new", None


class DormantDestinationMemory(RegionMemory):
    """A known-covered adjacent core must not receive a portal action."""

    def __init__(self, destination_label):
        self.destination_label = int(destination_label)
        self.region = {
            "id": 13,
            "state": "dormant",
            "endpoint_observations": 1,
        }

    def candidate_tier(self, _x, _y, _information, component=None):
        if component is not None and int(component["label"]) == self.destination_label:
            return "dormant", self.region
        return "new", None


class ObservedPortalDestinationMemory(RegionMemory):
    """An open region with a reached viewpoint is covered for portal admission."""

    def __init__(self, destination_label):
        self.destination_label = int(destination_label)
        self.region = {
            "id": 14,
            "state": "open",
            "endpoint_observations": 1,
        }

    def candidate_tier(self, _x, _y, _information, component=None):
        if component is not None and int(component["label"]) == self.destination_label:
            return "revisit", self.region
        return "new", None


class ReverseExitDormantDestinationMemory(DormantDestinationMemory):
    """A covered destination reached only by leaving the current entry gate."""

    def __init__(self, destination_label):
        super().__init__(destination_label)
        self.exit_region = {"id": 21, "state": "open"}

    def portal_exit_from_entry(self, _gate_xy, _source_xy, _destination_xy):
        return self.exit_region, {"gate": [2.0, 0.0], "inside": [1.0, 0.0]}


class SelectorStub(GlobalFrontierSelectionMixin):
    def __init__(self, labels):
        self.labels = labels
        self.region_memory = RegionMemory()
        self.candidate_limit = 100
        self.min_path_distance = 0.0
        self.frontier_approach_distance = 0.0
        self.completed_radius = 0.25
        self.heading_weight = 0.0
        self.structure_weight = 0.0
        self.min_structure_cells = 0
        self.semantic_hint_weight = 0.0
        self.semantic_hint_max_distance = 10.0
        self.place_graph_hop_limit = 1
        self.target_region_claim_active = False
        self.completed_frontiers = []
        self.last_frontier_region_tier = None
        self.last_selected_place_hops = None
        self.unanchored_topology = False
        self.current_physical_place_id = None
        self.current_task_version = ""
        self.target_observation_work = None
        self.semantic_place_action = None
        self.task_semantic_value_enabled = False
        self.published = []

    @staticmethod
    def nearest_safe_approach(_steps, row, col, _approach_cells, **_kwargs):
        return row, col

    @staticmethod
    def cell_xy(_message, row, col):
        return float(col), float(row)

    @staticmethod
    def candidate_costmap_distance(_validation, _x, _y):
        return None

    @staticmethod
    def frontier_is_completed(_x, _y):
        return False

    @staticmethod
    def frontier_is_rejected(_x, _y, _now):
        return False

    @staticmethod
    def frontier_structure(_occupied, _row, _col):
        return 1.0

    @staticmethod
    def heading_delta(_x, _y, _anchor, _heading):
        return None

    @staticmethod
    def candidate_covered_by_completed_viewpoint(*_args):
        return None

    @staticmethod
    def covered_place_observation_footprint(
        _message, known_free, _components, _anchor, _source_component,
    ):
        return np.zeros_like(known_free), 0

    @staticmethod
    def component_at_map_position(_message, components, _known_free, _x, _y):
        return {
            "epoch": components.epoch,
            "label": 1,
            "cells": 2,
            "center_x": 0.5,
            "center_y": 0.0,
            "min_x": 0.0,
            "max_x": 1.0,
            "min_y": 0.0,
            "max_y": 0.0,
        }

    def topology_component_evidence(self, _message, components, row, col, steps=None):
        if self.unanchored_topology:
            return None
        label = int(components.labels[row, col])
        return None if label <= 0 else {
            "epoch": components.epoch,
            "label": label,
            "cells": 2,
            "center_x": float(col),
            "center_y": float(row),
            "min_x": float(col),
            "max_x": float(col),
            "min_y": float(row),
            "max_y": float(row),
        }

    @staticmethod
    def frontier_information(_unknown, _row, col):
        # The adjacent candidate receives a much higher metric score. The
        # state machine, rather than this score, must decide the place order.
        return float(col * 100)

    def publish_status(self, event, **fields):
        self.published.append((event, fields))

def message():
    return SimpleNamespace(
        info=SimpleNamespace(
            resolution=1.0,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
        )
    )


class PlaceActionSelectionTest(unittest.TestCase):
    def select(self, labels, frontier, region_memory=None, **selection_options):
        labels = np.asarray(labels, dtype=np.int32)
        steps = selection_options.pop("steps", None)
        if steps is None:
            steps = np.arange(labels.size, dtype=np.int32).reshape(labels.shape)
        else:
            steps = np.asarray(steps, dtype=np.int32)
        unknown = np.asarray(
            selection_options.pop("unknown", np.zeros_like(labels, dtype=bool)),
            dtype=bool,
        )
        occupied = np.zeros_like(labels, dtype=bool)
        components = SimpleNamespace(labels=labels, epoch=1)
        selector = SelectorStub(labels)
        if region_memory is not None:
            selector.region_memory = region_memory
        current_place_id = selection_options.pop("current_physical_place_id", None)
        selector.current_physical_place_id = current_place_id
        selector.place_work_items = selection_options.pop("place_work_items", None)
        selector.current_task_version = selection_options.pop(
            "current_task_version", ""
        )
        selector.target_observation_work = selection_options.pop(
            "target_observation_work", None
        )
        selected = selector.choose_frontier(
            message(),
            steps,
            np.asarray(frontier, dtype=bool),
            unknown,
            occupied,
            now=0.0,
            robot_map=(0.0, 0.0),
            components=components,
            allow_portal_transitions=True,
            **selection_options,
        )
        return selected, selector

    def test_local_place_beats_higher_scored_adjacent_frontier(self):
        labels = [[1, 1, 0, 2, 2]]
        frontier = [[False, True, False, False, True]]

        selected, _selector = self.select(labels, frontier)

        self.assertIsNotNone(selected)
        self.assertEqual(selected[1], 1)
        self.assertEqual(selected[9], 0)
        self.assertEqual(selected[10], "frontier_endpoint")

    def test_covered_labels_with_live_unknown_space_remain_transit_candidates(self):
        labels = np.asarray([[1, 1, 0, 2, 2, 2]], dtype=np.int32)
        unknown = np.asarray([[False, False, False, False, False, True]])
        selector = SelectorStub(labels)
        self.assertEqual(
            selector._labels_with_unknown_boundary(labels, unknown), {2}
        )

    def test_observed_source_converts_adjacent_endpoint_to_portal(self):
        # The normal initial policy may score an adjacent endpoint directly.
        # Once a real endpoint was reached in place 1, however, movement
        # across its doorway must be an explicit portal action. The remote
        # frontier may rank that action but never becomes its direct goal.
        selected, selector = self.select(
            [[1, 1, 0, 2, 2]],
            [[False, True, False, False, True]],
            region_memory=OwnedObservedSourceMemory(1),
            current_physical_place_id=31,
        )

        self.assertIsNotNone(selected)
        self.assertEqual(selected[1], 3)
        self.assertEqual(selected[9], 1)
        self.assertEqual(selected[10], "portal_transition")
        self.assertEqual(selector.last_action_tier_order, ("adjacent", "probe", "local"))
        self.assertGreater(selector.last_cross_place_endpoint_deferrals, 0)

    def test_physical_owner_forces_portal_before_first_endpoint_view(self):
        """A room entered through a door owns its first outward transition."""
        memory = OwnedUnobservedSourceMemory(1)
        selected, selector = self.select(
            [[1, 1, 0, 2, 2]],
            [[False, False, False, False, True]],
            region_memory=memory,
            current_physical_place_id=31,
        )

        self.assertIsNotNone(selected)
        self.assertEqual(selected[9], 1)
        self.assertEqual(selected[10], "portal_transition")
        self.assertFalse(selector.last_source_place_observed)

    def test_empty_post_portal_snapshot_rehydrates_the_current_place(self):
        """An empty frontier must not strand the first post-arrival snapshot.

        Portal arrival installs a one-shot rehydration barrier so the next map
        projection can reconcile local WorkItems before graph transit.  There
        may be no ordinary frontier cells in that projection; the durable
        Place handoff must still complete instead of waiting for another map
        event forever.
        """
        labels = np.asarray([[1, 1, 1]], dtype=np.int32)
        selector = SelectorStub(labels)
        selector.region_memory = OwnedUnobservedSourceMemory(1)
        selector.current_physical_place_id = 31
        selector.place_entry_rehydration_pending = 31

        selected = selector.choose_frontier(
            message(),
            np.arange(labels.size, dtype=np.int32).reshape(labels.shape),
            np.zeros_like(labels, dtype=bool),
            np.zeros_like(labels, dtype=bool),
            np.zeros_like(labels, dtype=bool),
            now=1.0,
            robot_map=(0.0, 0.0),
            components=SimpleNamespace(labels=labels, epoch=2),
            allow_portal_transitions=True,
        )

        self.assertIsNone(selected)
        self.assertIsNone(selector.place_entry_rehydration_pending)
        self.assertEqual(selector.last_selection_context.source_place_id, 31)
        self.assertEqual(selector.published[-1][0], "place_entry_rehydrated")
        self.assertEqual(
            selector.published[-1][1]["reason"],
            "first_snapshot_after_portal_arrival",
        )

    def test_completed_current_place_is_transit_only_and_egresses_by_portal(self):
        """A return through a known room cannot restart local exploration."""
        selected, selector = self.select(
            [[1, 1, 0, 2, 2]],
            [[False, True, False, False, True]],
            region_memory=DormantTransitSourceMemory(1),
            current_physical_place_id=37,
        )

        self.assertIsNotNone(selected)
        self.assertEqual(selected[9], 1)
        self.assertEqual(selected[10], "portal_transition")
        self.assertTrue(selector.last_source_place_observed)
        self.assertEqual(
            selector.last_action_tier_order, ("adjacent", "probe", "local"),
        )

    def test_resolved_owner_work_item_cannot_be_selected_again(self):
        ledger = PlaceWorkItemLedger(merge_radius=0.5)
        item = ledger.dispatch(31, (1.0, 0.0), now=1.0)
        ledger.settle(item, WORK_RESOLVED, now=2.0, reason="observed")

        selected, _selector = self.select(
            [[1, 1, 1]],
            [[False, True, False]],
            region_memory=OwnedObservedSourceMemory(1),
            current_physical_place_id=31,
            place_work_items=ledger,
        )

        self.assertIsNone(selected)

    def test_failed_viewpoint_prefers_an_alternative_of_the_same_work_item(self):
        """A WorkItem retry outranks a more informative unrelated frontier."""
        ledger = PlaceWorkItemLedger(merge_radius=0.5)
        support = ObservationSupport(
            frontier_cells=frozenset({(0, 1), (1, 1)}),
            support_cells=frozenset({(2, 0), (2, 1)}),
        )
        cells, _details = ledger.reconcile(31, (support,), now=1.0)
        item_id = cells[(0, 1)]
        ledger.dispatch_existing(
            item_id,
            now=1.1,
            viewpoint_xy=(1.0, 0.0),
            map_goal=(1.0, 0.0),
        )
        ledger.fail_attempt(item_id, now=2.0, reason="stall")

        selected, selector = self.select(
            [[1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1]],
            [[False, True, False, False, False, True, False],
             [False, True, False, False, False, False, False]],
            steps=[[0, 1, 2, 3, 4, 5, 6],
                   [1, 2, 3, 4, 5, 6, 7]],
            unknown=[[False, False, True, False, False, False, True],
                     [False, False, True, False, False, False, False]],
            region_memory=OwnedObservedSourceMemory(1),
            current_physical_place_id=31,
            place_work_items=ledger,
        )

        self.assertIsNotNone(selected)
        self.assertEqual((selected[0], selected[1]), (1, 1))
        self.assertEqual(selected[12], item_id)
        self.assertTrue(selected[16])
        self.assertTrue(selector.last_selected_viewpoint_retry)

    def test_observed_source_prioritizes_portal_before_local_revisit(self):
        # The only new frontier is two rooms away, so the executable adjacent
        # action is the first certified portal. It must beat the still-valid
        # local endpoint after place 1 has an observation anchor.
        selected, selector = self.select(
            [[1, 1, 0, 2, 2, 0, 3, 3, 3]],
            [[False, True, False, False, False, False, False, False, True]],
            region_memory=ObservedSourceMemory(1),
        )

        self.assertIsNotNone(selected)
        self.assertEqual(selected[10], "portal_transition")
        self.assertEqual(selected[9], 1)
        self.assertEqual(selector.last_action_tier_order, ("adjacent", "probe", "local"))

    def test_task_belief_expands_through_portal_before_unrelated_local_work(self):
        # This is the architectural branch: once the current Place has been
        # observed but has no task evidence, a certified new Place is more
        # useful than another generic local frontier. The portal checks still
        # run before the action is returned.
        selected, selector = self.select(
            [[1, 1, 0, 2, 2]],
            [[False, True, False, False, True]],
            region_memory=OwnedObservedSourceMemory(1),
            current_physical_place_id=31,
        )
        selector.semantic_place_action = lambda _place_id, _observed: (
            "expand_unobserved_portal"
        )
        selector.task_semantic_value_enabled = True
        # Re-run through the actual selector with the policy installed.
        selected = selector.choose_frontier(
            message(),
            np.arange(5, dtype=np.int32).reshape(1, 5),
            np.asarray([[False, True, False, False, True]], dtype=bool),
            np.zeros((1, 5), dtype=bool),
            np.zeros((1, 5), dtype=bool),
            now=0.0,
            robot_map=(0.0, 0.0),
            components=SimpleNamespace(
                labels=np.asarray([[1, 1, 0, 2, 2]], dtype=np.int32), epoch=1
            ),
            allow_portal_transitions=True,
        )
        self.assertIsNotNone(selected)
        self.assertEqual(selected[10], "portal_transition")
        self.assertTrue(selector.published)
        self.assertEqual(selector.published[-1][0], "semantic_topology_expansion_selected")

    def test_target_observation_work_blocks_branch_first_crossing(self):
        target_work = TargetObservationWorkLedger("task-v1")
        target_work.observe_target(
            "task-v1", 31, labels=("yellow cup",), now=1.0,
        )
        selected, selector = self.select(
            [[1, 1, 0, 2, 2]],
            [[False, True, False, False, True]],
            region_memory=OwnedObservedSourceMemory(1),
            current_physical_place_id=31,
            current_task_version="task-v1",
            target_observation_work=target_work,
        )

        self.assertIsNotNone(selected)
        self.assertNotEqual(selected[10], "portal_transition")
        self.assertNotEqual(selected[17], "cross_portal")

    def test_portal_selection_keeps_a_physical_hypothesis_identity(self):
        selected, selector = self.select(
            [[1, 1, 0, 2, 2]],
            [[False, False, False, False, True]],
            region_memory=OwnedObservedSourceMemory(1),
            current_physical_place_id=31,
        )
        selector.portal_hypothesis_ledger = PortalHypothesisLedger()
        selector.current_task_version = "version-a"
        selected = selector.choose_frontier(
            message(),
            np.arange(5, dtype=np.int32).reshape(1, 5),
            np.asarray([[False, False, False, False, True]], dtype=bool),
            np.zeros((1, 5), dtype=bool),
            np.zeros((1, 5), dtype=bool),
            now=3.0,
            robot_map=(0.0, 0.0),
            components=SimpleNamespace(
                labels=np.asarray([[1, 1, 0, 2, 2]], dtype=np.int32), epoch=1
            ),
            allow_portal_transitions=True,
        )
        self.assertIsNotNone(selected)
        self.assertEqual(selected[10], "portal_transition")
        self.assertEqual(len(selector.portal_hypothesis_ledger.snapshot()), 1)
        hypothesis = selector.portal_hypothesis_ledger.snapshot()[0]
        self.assertEqual(hypothesis["state"], "selected")
        self.assertEqual(hypothesis["task_versions"], ["version-a"])

    def test_raw_label_split_with_a_bypass_keeps_source_place_identity(self):
        # The direct top-row route crosses a zero label between 1 and 2, but
        # raw free cells below it provide a local bypass. The selector must
        # keep both the route and its region-memory identity in place 1.
        labels = [
            [1, 0, 2, 2, 2],
            [1, 0, 0, 0, 2],
            [1, 0, 2, 2, 2],
        ]
        frontier = [
            [False, False, False, False, True],
            [False, False, False, False, False],
            [False, False, False, False, False],
        ]

        selected, selector = self.select(labels, frontier)

        self.assertIsNotNone(selected)
        self.assertEqual(selected[9], 0)
        self.assertEqual(selected[8]["label"], 1)
        self.assertGreater(selector.last_uncertified_place_transition_skips, 0)

    def test_remote_frontier_emits_adjacent_core_portal_action(self):
        labels = [[1, 1, 0, 2, 2, 0, 3, 3, 3]]
        frontier = [[False, False, False, False, False, False, False, False, True]]

        selected, _selector = self.select(labels, frontier)

        self.assertIsNotNone(selected)
        self.assertEqual((selected[0], selected[1]), (0, 3))
        self.assertEqual((selected[2], selected[3]), (3.0, 0.0))
        self.assertEqual(selected[9], 1)
        self.assertEqual(selected[10], "portal_transition")
        self.assertIsNotNone(selected[8])
        self.assertEqual(selected[8]["label"], 2)
        self.assertEqual(selected[11], (2.0, 0.0))

    def test_adjacent_frontier_endpoint_retains_the_certified_portal_gate(self):
        # An adjacent endpoint can be selected before the remote portal
        # fallback. It still physically crosses a doorway, so terminal memory
        # must receive the same gate fact as a portal_transition action.
        selected, _selector = self.select(
            [[1, 1, 0, 2, 2]],
            [[False, False, False, False, True]],
        )

        self.assertIsNotNone(selected)
        self.assertEqual(selected[9], 1)
        self.assertEqual(selected[10], "frontier_endpoint")
        self.assertEqual(selected[11], (2.0, 0.0))

    def test_sealed_portal_reentry_beats_remote_frontier_score(self):
        # The remembered room has an old/unrelated component label, modelling
        # an online-SLAM split. Geometry association cannot recognize the far
        # core, but the certified first doorway is the same recorded entrance.
        memory = FrontierRegionMemory(
            radius=2.5,
            information_delta=8,
            stagnation_timeout=12,
            failure_limit=2,
            limit=16,
        )
        region, _tier = memory.enter(
            3.0,
            0.0,
            now=0.0,
            component={
                "epoch": 99,
                "label": 42,
                "cells": 40,
                "center_x": 3.0,
                "center_y": 0.0,
                "min_x": 2.5,
                "max_x": 3.5,
                "min_y": -0.5,
                "max_y": 0.5,
            },
            entry_portal=(2.0, 0.0),
        )
        memory.dormant(region, now=1.0, reason="place_observation_complete")

        selected, selector = self.select(
            [[1, 1, 0, 2, 2, 0, 3, 3, 3]],
            [[False, False, False, False, False, False, False, False, True]],
            region_memory=memory,
        )

        self.assertIsNone(selected)
        self.assertGreater(selector.last_sealed_portal_reentry_skips, 0)
        self.assertEqual(selector.last_sealed_portal_reentry_region_id, region["id"])
        self.assertEqual(selector.last_sealed_portal_reentry_gate, [2.0, 0.0])

    def test_semantic_pursuit_resolves_the_forward_hint_predicate(self):
        selected, _selector = self.select(
            [[1, 1]],
            [[False, True]],
            semantic_hint=(10.0, 0.0),
            semantic_pursuit=True,
        )

        self.assertIsNotNone(selected)
        self.assertEqual((selected[2], selected[3]), (1.0, 0.0))

    def test_observed_adjacent_place_is_not_reselected_as_a_portal(self):
        # The sole frontier lies in a previously observed adjacent room. It
        # must be blocked both as a normal revisit and as a portal fallback;
        # otherwise the graph bypasses the lifecycle invariant after scoring.
        selected, selector = self.select(
            [[1, 0, 2]],
            [[False, False, True]],
            region_memory=ObservedRegionMemory({2}),
        )

        self.assertIsNone(selected)
        self.assertGreater(selector.last_observed_place_reentry_skips, 0)

    def test_covered_portal_destination_is_rejected_before_dispatch(self):
        # The ordinary scoring pool is empty.  The portal fallback must still
        # inspect the destination's durable topology identity before emitting
        # a cross-door action, otherwise the controller can physically enter
        # a covered room and only learn that it was invalid at terminal time.
        selected, selector = self.select(
            [[1, 1, 0, 2, 2, 0, 3]],
            [[False, False, False, False, False, False, True]],
            region_memory=DormantDestinationMemory(2),
        )

        self.assertIsNone(selected)
        self.assertGreater(selector.last_portal_covered_destination_skips, 0)

    def test_observed_open_portal_destination_is_rejected_before_dispatch(self):
        # A place remains open until the robot crosses a different doorway,
        # but a reached viewpoint already makes it ineligible as a portal
        # destination. Otherwise a furniture-induced component split can
        # send the robot back into the same physical room.
        selected, selector = self.select(
            [[1, 1, 0, 2, 2, 0, 3]],
            [[False, False, False, False, False, False, True]],
            region_memory=ObservedPortalDestinationMemory(2),
        )

        self.assertIsNone(selected)
        self.assertGreater(selector.last_portal_covered_destination_skips, 0)

    def test_reverse_entry_exit_can_cross_a_covered_destination(self):
        # A room can only leave through the same doorway through which it was
        # entered. The destination side may be a covered corridor, but this
        # is an outward graph edge rather than a prohibited room revisit.
        selected, selector = self.select(
            [[1, 1, 0, 2, 2, 0, 3]],
            [[False, False, False, False, False, False, True]],
            region_memory=ReverseExitDormantDestinationMemory(2),
        )

        self.assertIsNotNone(selected)
        self.assertEqual(selected[10], "portal_transition")
        self.assertEqual(selector.last_portal_reverse_egress_count, 1)
        self.assertEqual(selector.last_portal_reverse_egress_region_id, 21)

    def test_observed_long_corridor_stays_available_in_its_current_place(self):
        # A long corridor may publish multiple endpoints. The source label is
        # exempt from the observed-place block, so its route can continue
        # without being mistaken for a room re-entry.
        selected, selector = self.select(
            [[1, 1, 1]],
            [[False, False, True]],
            region_memory=ObservedRegionMemory({1}),
        )

        self.assertIsNotNone(selected)
        self.assertEqual(selected[9], 0)
        self.assertEqual(selector.last_observed_place_reentry_skips, 0)

    def test_ready_place_selects_an_egress_instead_of_another_local_endpoint(self):
        # A terminal endpoint in place 1 has already been observed. The only
        # raw frontier is still in that place, but the graph knows a doorway to
        # place 2. The selector must create the explicit portal action rather
        # than reactivate place 1 because its score happens to be high.
        selected, selector = self.select(
            [[1, 1, 0, 2, 2]],
            [[False, True, False, False, False]],
            region_memory=ReadyToExitRegionMemory(1),
        )

        self.assertIsNotNone(selected)
        self.assertEqual(selected[10], "portal_transition")
        self.assertEqual(selected[9], 1)
        self.assertGreater(selector.last_ready_to_exit_reentry_skips, 0)

    def test_recovery_cannot_mint_an_unanchored_adjacent_room(self):
        """Weak recovery needs a durable place label before it may score."""
        labels = np.asarray([[1, 0, 2]], dtype=np.int32)
        steps = np.arange(labels.size, dtype=np.int32).reshape(labels.shape)
        unknown = np.zeros_like(labels, dtype=bool)
        occupied = np.zeros_like(labels, dtype=bool)
        selector = SelectorStub(labels)
        selector.unanchored_topology = True
        components = SimpleNamespace(labels=labels, epoch=1)

        selected = selector.choose_frontier(
            message(),
            steps,
            np.asarray([[False, False, True]], dtype=bool),
            unknown,
            occupied,
            now=0.0,
            robot_map=(0.0, 0.0),
            components=components,
            selection_tier="navfn_observation_recovery",
            allow_portal_transitions=False,
        )

        self.assertIsNone(selected)
        self.assertGreater(selector.last_place_graph_unresolved_candidates, 0)

    def test_strict_selection_cannot_mint_an_unanchored_room(self):
        """After topology is available, coordinates cannot create a place."""
        labels = np.asarray([[1, 0, 2]], dtype=np.int32)
        steps = np.arange(labels.size, dtype=np.int32).reshape(labels.shape)
        unknown = np.zeros_like(labels, dtype=bool)
        occupied = np.zeros_like(labels, dtype=bool)
        selector = SelectorStub(labels)
        selector.unanchored_topology = True
        components = SimpleNamespace(labels=labels, epoch=1)

        selected = selector.choose_frontier(
            message(),
            steps,
            np.asarray([[False, False, True]], dtype=bool),
            unknown,
            occupied,
            now=0.0,
            robot_map=(0.0, 0.0),
            components=components,
            selection_tier="strict_clearance",
            allow_portal_transitions=False,
        )

        self.assertIsNone(selected)
        self.assertGreater(selector.last_place_graph_unresolved_candidates, 0)


if __name__ == "__main__":
    unittest.main()
