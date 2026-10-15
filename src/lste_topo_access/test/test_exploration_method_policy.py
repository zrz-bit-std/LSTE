#!/usr/bin/env python3
"""Regression tests for named, comparable exploration method contracts."""

from pathlib import Path
import sys
import unittest
from unittest.mock import patch


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_config import GlobalFrontierConfigurationMixin
from global_frontier_experiment_policy import (
    method_names,
    resolve_exploration_method,
)


class ExplorationMethodPolicyTest(unittest.TestCase):
    def test_contracts_are_named_and_distinguishable(self):
        self.assertEqual(
            method_names(),
            (
                "frontier",
                "frontier_distance_dedup",
                "place_portal",
                "place_portal_workitem",
                "place_portal_workitem_legacy_rank",
                "place_portal_workitem_strict",
            ),
        )
        ordinary = resolve_exploration_method("frontier")
        dedup = resolve_exploration_method("frontier-distance-dedup")
        full = resolve_exploration_method("place_portal_workitem")
        self.assertFalse(ordinary.place_memory)
        self.assertFalse(ordinary.distance_deduplication)
        self.assertTrue(dedup.distance_deduplication)
        self.assertFalse(dedup.place_memory)
        self.assertTrue(full.place_memory)
        self.assertTrue(full.certified_portals)
        self.assertTrue(full.work_items)
        self.assertTrue(full.task_semantic_value)
        self.assertTrue(full.branch_first)
        self.assertTrue(full.graph_route_planner)
        self.assertTrue(full.event_driven_deliberation)
        self.assertFalse(dedup.branch_first)
        strict = resolve_exploration_method("place_portal_workitem_strict")
        self.assertTrue(strict.work_items)
        self.assertTrue(strict.task_semantic_value)
        self.assertFalse(strict.branch_first)
        self.assertTrue(strict.graph_route_planner)
        self.assertTrue(strict.event_driven_deliberation)
        self.assertFalse(dedup.task_semantic_value)
        legacy_rank = resolve_exploration_method("place_portal_workitem_legacy_rank")
        self.assertTrue(legacy_rank.work_items)
        self.assertTrue(legacy_rank.branch_first)
        self.assertEqual(legacy_rank.frontier_action_policy, "legacy_scalar")
        self.assertFalse(legacy_rank.event_driven_deliberation)

    def test_unknown_method_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "unknown exploration_method"):
            resolve_exploration_method("whatever")

    def test_configuration_does_not_silently_replace_an_invalid_method(self):
        configuration = GlobalFrontierConfigurationMixin()
        with patch(
            "global_frontier_config_exploration.rospy.logerr",
            return_value=None,
        ):
            with self.assertRaisesRegex(ValueError, "unknown exploration_method"):
                configuration._load_exploration_parameters(
                    lambda name, default: (
                        "invalid_method"
                        if name == "~exploration_method" else default
                    )
                )

    def test_configuration_exposes_the_resolved_method_capabilities(self):
        configuration = GlobalFrontierConfigurationMixin()
        with patch(
            "global_frontier_config_navigation.rospy.get_param",
            return_value=0.40,
        ):
            configuration._load_parameters(
                lambda name, default: (
                    "frontier_distance_dedup"
                    if name == "~exploration_method" else default
                )
            )
        self.assertEqual(configuration.exploration_method, "frontier_distance_dedup")
        self.assertTrue(configuration.distance_deduplication_enabled)
        self.assertFalse(configuration.place_memory_enabled)
        self.assertFalse(configuration.work_item_memory_enabled)
        self.assertFalse(configuration.certified_portals_enabled)
        self.assertFalse(configuration.task_semantic_value_enabled)
        self.assertFalse(configuration.graph_route_planner_enabled)


if __name__ == "__main__":
    unittest.main()
