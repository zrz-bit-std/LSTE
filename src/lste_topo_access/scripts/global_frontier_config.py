#!/usr/bin/env python3

"""Configuration facade for the online frontier explorer.

The public mixin preserves the old configuration entry point while sibling
modules own each independent configuration domain. Keeping the order here
makes the small number of cross-domain dependencies explicit.
"""

import rospy

from global_frontier_config_exploration import (
    GlobalFrontierExplorationConfigurationMixin,
)
from global_frontier_config_navigation import (
    GlobalFrontierNavigationConfigurationMixin,
)
from global_frontier_config_topics import GlobalFrontierTopicConfigurationMixin


class GlobalFrontierConfigurationMixin(
    GlobalFrontierExplorationConfigurationMixin,
    GlobalFrontierNavigationConfigurationMixin,
    GlobalFrontierTopicConfigurationMixin,
):
    """Resolve static configuration without mixing unrelated policy domains."""

    def _load_parameters(self, gp):
        """Load groups in dependency order while preserving ROS parameter names."""
        target_done_policy = str(
            gp("~target_done_policy", "visual_target")
        ).strip().lower()
        if target_done_policy not in (
            "visual_target",
            "target_room_entry",
            "target_context",
        ):
            rospy.logwarn(
                "Global frontier received unsupported target_done_policy=%r; "
                "using visual_target",
                target_done_policy,
            )
            target_done_policy = "visual_target"
        # This is a mission ownership contract, not a navigation tuning
        # parameter. In target-room-entry mode the explorer must not turn a
        # detector candidate into a target reinspection route.
        self.target_done_policy = target_done_policy
        self._load_topic_parameters(gp)
        self._load_navigation_parameters(gp)
        self._load_exploration_parameters(gp)
