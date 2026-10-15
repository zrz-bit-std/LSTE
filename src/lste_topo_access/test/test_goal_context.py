#!/usr/bin/env python3
"""Regression tests for durable mission ownership across SLAM replans."""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from goal_context import (
    default_goal_context,
    goal_context_identity,
    normalize_goal_context,
    task_version_from_task,
)


class GoalContextTest(unittest.TestCase):
    @staticmethod
    def _task(**overrides):
        fields = {
            "task_id": "yellow_cup",
            "target_name": "cup",
            "target_attributes": ["yellow"],
            "env_related_structures": ["table"],
            "env_type_prior": ["office"],
            "obj_key_objects": ["monitor"],
            "obj_negative_clues": ["sink"],
            "ctx_left": "table",
            "ctx_right": "monitor",
            "raw_json": '{"target":{"name":"cup","attributes":["yellow"]}}',
        }
        fields.update(overrides)
        return SimpleNamespace(**fields)

    def test_task_version_is_stable_for_json_formatting(self):
        first = self._task(raw_json='{"b": 2, "a": 1}')
        equivalent = self._task(raw_json=' { "a":1,"b":2 } ')
        self.assertEqual(task_version_from_task(first), task_version_from_task(equivalent))

    def test_task_version_changes_when_semantic_content_changes(self):
        first = self._task()
        changed = self._task(target_attributes=["blue"])
        self.assertNotEqual(task_version_from_task(first), task_version_from_task(changed))

    def test_mission_version_is_part_of_goal_identity(self):
        first = default_goal_context(
            "place_portal_workitem", "yellow_cup", "yellow_cup", "v1"
        )
        next_version = dict(first, task_version="v2")
        self.assertNotEqual(
            goal_context_identity(first), goal_context_identity(next_version)
        )

    def test_same_work_item_keeps_identity_when_its_map_endpoint_moves(self):
        first = {
            "exploration_method": "place_portal_workitem",
            "task_id": "yellow_cup",
            "goal_role": "place_observation_work",
            "owner_place_id": 4,
            "source_place_id": 4,
            "work_item_id": 11,
        }
        replanned = {
            **first,
            # Endpoint coordinates intentionally do not belong here. A SLAM
            # correction must remain the same ObservationWorkItem action.
        }
        self.assertEqual(goal_context_identity(first), goal_context_identity(replanned))

    def test_same_pose_with_a_new_work_item_is_a_new_mission_action(self):
        first = {
            "goal_role": "place_observation_work",
            "owner_place_id": 4,
            "source_place_id": 4,
            "work_item_id": 11,
        }
        next_item = {**first, "work_item_id": 12}
        self.assertNotEqual(
            goal_context_identity(first), goal_context_identity(next_item)
        )

    def test_task_id_changes_the_mission_without_changing_place_memory(self):
        first = default_goal_context("place_portal_workitem", "yellow_cup")
        next_task = default_goal_context("place_portal_workitem", "blue_mug")
        self.assertNotEqual(
            goal_context_identity(first), goal_context_identity(next_task)
        )

    def test_invalid_wire_values_fall_back_to_a_safe_geometry_context(self):
        context = normalize_goal_context({
            "goal_role": "untrusted_role",
            "owner_place_id": "nope",
        })
        self.assertEqual(context["goal_role"], "geometry_frontier")
        self.assertIsNone(context["owner_place_id"])

    def test_probe_phase_is_part_of_route_identity(self):
        source = {
            "goal_role": "portal_probe",
            "source_place_id": 3,
            "portal_probe_id": 9,
            "portal_probe_phase": "source",
        }
        destination = {
            **source,
            "portal_probe_phase": "destination",
        }

        self.assertNotEqual(
            goal_context_identity(source), goal_context_identity(destination)
        )


if __name__ == "__main__":
    unittest.main()
