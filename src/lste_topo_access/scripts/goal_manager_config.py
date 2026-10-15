"""Small configuration facade for the Goal Manager ROS node.

The sibling modules own independent parameter domains.  This facade preserves
the historical entry points so launch files and callers do not need to know
about the internal layout.
"""

import math

from goal_manager_config_navigation import GoalManagerNavigationConfigMixin
from goal_manager_config_target_completion import (
    GoalManagerTargetCompletionConfigMixin,
)
from goal_manager_config_target_routing import GoalManagerTargetRoutingConfigMixin
from goal_manager_config_target_tracking import GoalManagerTargetTrackingConfigMixin


class GoalManagerConfigMixin(
    GoalManagerNavigationConfigMixin,
    GoalManagerTargetCompletionConfigMixin,
    GoalManagerTargetRoutingConfigMixin,
    GoalManagerTargetTrackingConfigMixin,
):
    """Load configuration groups in dependency order."""

    def _load_parameters(self, gp):
        """Load all configuration without changing the ROS parameter API."""
        self._load_target_parameters(gp)
        self._load_navigation_parameters(gp)
        self._load_scan_parameters(gp)

    def _load_target_parameters(self, gp):
        """Load visual tracking, route admission, and completion policy."""
        self._load_target_tracking_parameters(gp)
        self._load_target_routing_parameters(gp)
        self._load_target_completion_parameters(gp)

    def _load_navigation_parameters(self, gp):
        """Load global-goal source, frontier integration, and controller API."""
        self._load_goal_source_parameters(gp)
        self._load_frontier_integration_parameters(gp)
        self._load_mission_interface_parameters(gp)

    def _load_scan_parameters(self, gp):
        """Configure scan-based goal shaping and diagnostics."""
        self.scan_topic = gp("~scan_topic", "/pro3/rl_scan")
        self.scan_frame = gp("~scan_frame", "")
        self.safety_margin = float(gp("~safety_margin", 0.6))
        self.scan_window_bins = int(gp("~scan_window_bins", 2))
        self.front_sigma = math.radians(float(gp("~front_sigma_deg", 25.0)))
        self.area_power = float(gp("~area_power", 1.0))
        self.v_min = float(gp("~v_min", 0.05))
        self.v_scale = float(gp("~v_scale", 0.3))
        self.front_deg = math.radians(float(gp("~front_deg", 60.0)))
        self.back_deg = math.radians(float(gp("~back_deg", 60.0)))
        self.w_front_max = float(gp("~w_front_max", 1.5))
        self.w_back_min = float(gp("~w_back_min", 0.2))
        self.forward_only = bool(gp("~forward_only", True))
        self.v_gate = float(gp("~v_gate", 0.08))
        self.min_allowed_score = float(gp("~min_allowed_score", 0.05))
        self.switch_margin = float(gp("~switch_margin", 0.15))
        debug_goal_log = gp("~debug_goal_log", False)
        self.debug_goal_log = str(debug_goal_log).strip().lower() in (
            "1", "true", "yes", "on",
        )
