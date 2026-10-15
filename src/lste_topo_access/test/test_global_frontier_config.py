#!/usr/bin/env python3
"""Behavioral regression tests for decomposed frontier configuration."""

from pathlib import Path
import sys
import unittest
from unittest.mock import patch


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_config import GlobalFrontierConfigurationMixin


class GlobalFrontierConfigurationTest(unittest.TestCase):
    def test_groups_preserve_defaults_and_cross_domain_dependencies(self):
        """The facade must still yield the same resolved runtime contract."""
        overrides = {
            "~clearance": 0.30,
            "~fallback_clearance": 0.80,
            "~waypoint_release_radius": 0.60,
            "~lookahead_distance": 1.00,
            "~prefetch_distance": 0.50,
            "~early_handoff_distance": 3.00,
            "~mission_endpoint_only": "false",
            "~persistent_execution": "yes",
            "~region_memory_radius": 0.10,
            "~turn_execution_mode": "unsupported",
        }

        def get_parameter(name, default):
            return overrides.get(name, default)

        configuration = GlobalFrontierConfigurationMixin()
        with patch(
            "global_frontier_config_navigation.rospy.get_param",
            return_value=0.40,
        ), patch("global_frontier_config_exploration.rospy.logwarn") as warn:
            configuration._load_parameters(get_parameter)

        self.assertEqual(configuration.map_topic, "/map")
        self.assertEqual(
            configuration.command_topic, "/lste/global_frontier/route_command"
        )
        self.assertEqual(configuration.scan_topic, "/pro3/rlscan")
        self.assertEqual(configuration.costmap_max_age, 3.0)
        self.assertEqual(configuration.frontier_clearance, 0.30)
        self.assertEqual(configuration.route_segment_distance, 0.80)
        self.assertFalse(configuration.mission_endpoint_only)
        self.assertTrue(configuration.persistent_execution)
        self.assertEqual(configuration.prefetch_distance, 1.20)
        self.assertEqual(configuration.early_handoff_distance, 1.20)
        self.assertEqual(configuration.endpoint_terminal_wait_radius, 0.90)
        self.assertEqual(configuration.region_memory_radius, 2.50)
        self.assertEqual(configuration.turn_execution_mode, "native_teb")
        self.assertEqual(configuration.planning_period, 1.5)
        warn.assert_called_once()


if __name__ == "__main__":
    unittest.main()
