#!/usr/bin/env python3
"""Online-map frontier explorer for the normal LSTE search pipeline.

The node never knows an object's coordinates and never publishes
``/lste/final_goal``.  It builds an online SLAM map, finds an unknown-space
boundary reachable through known free cells, and publishes stable route points
in the ``map`` frame.  Goal Manager remains the sole final-goal owner. Keeping
the route in the SLAM frame is important: converting each point to ``odom``
would make an otherwise fixed map point move whenever gmapping updates the
``map -> odom`` transform. Navfn/TEB then follows the complete known-free path
in one action instead of stopping at a sequence of drifting setpoints.
"""

from pathlib import Path
import sys

import rospy


# ``rosrun`` executes a relay under ``devel/lib``.  Resolve sibling modules
# from this source script instead of depending on a catkin Python package.
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from global_frontier_execution import GlobalFrontierExecutionMixin
from global_frontier_event_callbacks import GlobalFrontierEventCallbacksMixin
from global_frontier_lifecycle import GlobalFrontierLifecycleMixin
from global_frontier_durable_lease_lifecycle import (
    GlobalFrontierDurableLeaseLifecycleMixin,
)
from global_frontier_geometry import GlobalFrontierGeometryMixin
from global_frontier_observation import GlobalFrontierObservationMixin
from global_frontier_planning_cycle import GlobalFrontierPlanningCycleMixin
from global_frontier_activation import GlobalFrontierActivationMixin
from global_frontier_config import GlobalFrontierConfigurationMixin
from global_frontier_planning import GlobalFrontierPlanningMixin
from global_frontier_portal_lifecycle import GlobalFrontierPortalLifecycleMixin
from global_frontier_portal_probe_lifecycle import (
    GlobalFrontierPortalProbeLifecycleMixin,
)
from global_frontier_reporting import GlobalFrontierReportingMixin
from global_frontier_ros_interfaces import GlobalFrontierRosInterfacesMixin
from global_frontier_route_state import GlobalFrontierRouteStateMixin
from global_frontier_runtime_state import GlobalFrontierRuntimeStateMixin
from global_frontier_work_items import GlobalFrontierWorkItemLifecycleMixin
from global_frontier_grid import (
    bfs as grid_bfs,
    cell_xy as grid_cell_xy,
    frontier_information as grid_frontier_information,
    frontier_mask as grid_frontier_mask,
    inflate as grid_inflate,
    nearest_seed as grid_nearest_seed,
    route_path as grid_route_path,
    waypoint_on_path as grid_waypoint_on_path,
    xy_to_grid_cell as grid_xy_to_grid_cell,
)
from global_frontier_route_commands import GlobalFrontierRouteCommandMixin
from global_frontier_selection import GlobalFrontierSelectionMixin
from global_frontier_terminal_lifecycle import GlobalFrontierTerminalLifecycleMixin
from global_frontier_terminal_observation import GlobalFrontierTerminalObservationMixin


class GlobalFrontierExplorer(
    GlobalFrontierLifecycleMixin,
    GlobalFrontierTerminalLifecycleMixin,
    GlobalFrontierTerminalObservationMixin,
    GlobalFrontierPortalProbeLifecycleMixin,
    GlobalFrontierPortalLifecycleMixin,
    GlobalFrontierPlanningCycleMixin,
    GlobalFrontierActivationMixin,
    GlobalFrontierObservationMixin,
    GlobalFrontierSelectionMixin,
    GlobalFrontierRouteCommandMixin,
    GlobalFrontierPlanningMixin,
    GlobalFrontierExecutionMixin,
    GlobalFrontierEventCallbacksMixin,
    GlobalFrontierDurableLeaseLifecycleMixin,
    GlobalFrontierReportingMixin,
    GlobalFrontierWorkItemLifecycleMixin,
    GlobalFrontierRouteStateMixin,
    GlobalFrontierGeometryMixin,
    GlobalFrontierRosInterfacesMixin,
    GlobalFrontierRuntimeStateMixin,
    GlobalFrontierConfigurationMixin,
):
    def __init__(self):
        rospy.init_node("lste_global_frontier")
        self._load_parameters(rospy.get_param)
        self._initialize_runtime_state()
        self._initialize_lifecycle_manager()
        self._setup_ros_interfaces()

    @staticmethod
    def inflate(occupied, cells):
        return grid_inflate(occupied, cells)

    @staticmethod
    def nearest_seed(free, row, col, limit):
        return grid_nearest_seed(free, row, col, limit)

    @staticmethod
    def bfs(free, seed, should_abort=None):
        return grid_bfs(free, seed, should_abort)

    @staticmethod
    def is_frontier(free, unknown):
        return grid_frontier_mask(free, unknown)

    @staticmethod
    def cell_xy(message, row, col):
        return grid_cell_xy(message, row, col)

    def frontier_information(self, unknown, row, col):
        return grid_frontier_information(unknown, row, col, self.info_radius)

    @staticmethod
    def xy_to_grid_cell(message, x, y):
        return grid_xy_to_grid_cell(message, x, y)

    @staticmethod
    def world_cell(message, x, y):
        return grid_xy_to_grid_cell(message, x, y)

    @staticmethod
    def waypoint_on_path(steps, row, col, lookahead_steps):
        return grid_waypoint_on_path(steps, row, col, lookahead_steps)

    @staticmethod
    def route_path(steps, seed, target):
        return grid_route_path(steps, seed, target)

    def resolve_active_route_terminal_or_failure(self, *args, **kwargs):
        """Compatibility entry point for the extracted execution mixin."""
        return GlobalFrontierExecutionMixin.resolve_active_route_terminal_or_failure(
            self, *args, **kwargs
        )


if __name__ == "__main__":
    GlobalFrontierExplorer()
    rospy.spin()
