#!/usr/bin/env python3
"""Compose the TEB move_base action bridge.

The executable intentionally contains only ROS node construction and the
module composition.  Geometry, mission intent, frontier lifecycle, TEB
feedback, action dispatch, and persistent execution each live in focused
sibling modules so they can be edited and tested independently.
"""

from pathlib import Path as FilePath
import sys

import rospy


# ``rosrun`` may execute a catkin-generated relay from ``devel/lib``.  Import
# sibling modules from the source/install script directory in either layout.
SCRIPT_DIR = FilePath(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from teb_goal_bridge_action_health import TebGoalBridgeActionHealthMixin
from teb_goal_bridge_action_callbacks import TebGoalBridgeActionCallbacksMixin
from teb_goal_bridge_action_dispatch import TebGoalBridgeActionDispatchMixin
from teb_goal_bridge_frontier_status import TebGoalBridgeFrontierStatusMixin
from teb_goal_bridge_geometry import TebGoalBridgeGeometryMixin
from teb_goal_bridge_handoff import TebGoalBridgeHandoffMixin
from teb_goal_bridge_intent import TebGoalBridgeIntentMixin
from teb_goal_bridge_lifecycle import TebGoalBridgeLifecycleMixin
from teb_goal_bridge_mission_lifecycle import TebGoalBridgeMissionLifecycleMixin
from teb_goal_bridge_parameters import configure_bridge_parameters
from teb_goal_bridge_persistent_frontier_endpoint import (
    TebGoalBridgePersistentFrontierEndpointMixin,
)
from teb_goal_bridge_persistent_frontier_handoff import (
    TebGoalBridgePersistentFrontierHandoffMixin,
)
from teb_goal_bridge_persistent_execution import TebGoalBridgePersistentExecutionMixin
from teb_goal_bridge_prefetch_admission import TebGoalBridgePrefetchAdmissionMixin
from teb_goal_bridge_persistent_target import TebGoalBridgePersistentTargetMixin
from teb_goal_bridge_persistent_target_approach import (
    TebGoalBridgePersistentTargetApproachMixin,
)
from teb_goal_bridge_ros import connect_bridge_ros
from teb_goal_bridge_route_monitoring import TebGoalBridgeRouteMonitoringMixin
from teb_goal_bridge_state import initialize_bridge_state
from teb_goal_bridge_status import TebGoalBridgeStatusMixin
from teb_goal_bridge_teb_runtime import TebGoalBridgeTebRuntimeMixin


class TebGoalBridge(
    TebGoalBridgeLifecycleMixin,
    TebGoalBridgeTebRuntimeMixin,
    TebGoalBridgeFrontierStatusMixin,
    TebGoalBridgeIntentMixin,
    TebGoalBridgeGeometryMixin,
    TebGoalBridgeMissionLifecycleMixin,
    TebGoalBridgePersistentFrontierEndpointMixin,
    TebGoalBridgePersistentTargetApproachMixin,
    TebGoalBridgePersistentFrontierHandoffMixin,
    TebGoalBridgePersistentExecutionMixin,
    TebGoalBridgeHandoffMixin,
    TebGoalBridgeActionDispatchMixin,
    TebGoalBridgePrefetchAdmissionMixin,
    TebGoalBridgeActionCallbacksMixin,
    TebGoalBridgeStatusMixin,
    TebGoalBridgePersistentTargetMixin,
    TebGoalBridgeActionHealthMixin,
    TebGoalBridgeRouteMonitoringMixin,
):
    def __init__(self):
        rospy.init_node("lste_teb_goal_bridge")
        configure_bridge_parameters(self)
        initialize_bridge_state(self)
        self._initialize_lifecycle_manager()
        connect_bridge_ros(self)
        rospy.loginfo(
            "TEB goal bridge ready: action=move_base mode=%s active_mode=%s "
            "goal=%s command=%s atomic=%s global_frame=%s persistent_mission=%s",
            self.mode,
            self.active_mode,
            self.goal_topic,
            self.goal_command_topic,
            self.use_goal_command,
            self.global_frame,
            self.persistent_mission_goal_topic,
        )


if __name__ == "__main__":
    TebGoalBridge()
    rospy.spin()
