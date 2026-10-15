#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Goal Manager
- 唯一对外发布 /lste/final_goal (PoseStamped; normal local goals use odom,
  online SLAM frontier goals retain the map frame)
- 依据 state/subtype + frontier/检测/激光裁剪生成全局目标
- 各状态有独立更新周期；state 变化可立即打断更新
"""

from pathlib import Path
import sys

import rospy

# ``rosrun`` may execute a relay from ``devel/lib``.  Keep the extracted
# GoalManager modules resolvable in both devel-space and install-space runs.
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from goal_manager_config import GoalManagerConfigMixin
from goal_manager_lifecycle import GoalManagerLifecycleMixin
from goal_manager_goal_arbitration import GoalManagerGoalArbitrationMixin
from goal_manager_goal_output import GoalManagerGoalOutputMixin
from goal_manager_legacy_goals import GoalManagerLegacyGoalsMixin
from goal_manager_modes import (
    CATCH_CTX_MODE,
    CATCH_TARGET_MODE,
    EXPLORE_PASS_MODE,
    EXPLORE_SUS_C_MODE,
    STATE_LOCKED,
    STATE_PASS,
    STATE_SUSPICIOUS,
)
from goal_manager_ros_callbacks import GoalManagerRosCallbacksMixin
from goal_manager_ros_interfaces import GoalManagerRosInterfacesMixin
from goal_manager_runtime_state import GoalManagerRuntimeStateMixin
from goal_manager_scheduling import GoalManagerSchedulingMixin
from goal_manager_target_completion import GoalManagerTargetCompletionMixin
from goal_manager_target_segments import GoalManagerTargetSegmentsMixin
from goal_manager_frontier import GoalManagerFrontierMixin
from goal_manager_projection import GoalManagerProjectionMixin
from goal_manager_target_follow import GoalManagerTargetFollowMixin
from goal_manager_viewpoint import select_target_viewpoint
from goal_manager_target_utils import GoalManagerTargetUtilsMixin, wrap_angle



class GoalManager(
    GoalManagerLifecycleMixin,
    GoalManagerRosCallbacksMixin,
    GoalManagerSchedulingMixin,
    GoalManagerTargetSegmentsMixin,
    GoalManagerGoalArbitrationMixin,
    GoalManagerTargetCompletionMixin,
    GoalManagerLegacyGoalsMixin,
    GoalManagerGoalOutputMixin,
    GoalManagerRuntimeStateMixin,
    GoalManagerRosInterfacesMixin,
    GoalManagerConfigMixin,
    GoalManagerFrontierMixin,
    GoalManagerProjectionMixin,
    GoalManagerTargetFollowMixin,
    GoalManagerTargetUtilsMixin,
):
    def __init__(self):
        rospy.init_node("lste_goal_manager")

        # Parameters are kept in a dedicated method so mission setup remains
        # readable and parameter-only changes stay localized.
        gp = rospy.get_param
        self._load_parameters(gp)
        self._initialize_runtime_state(gp)
        self._initialize_lifecycle_manager()
        self._setup_ros_interfaces()
        rospy.loginfo(
            "Goal Manager started: source=%s publishes /lste/final_goal "
            "target_route_validation=%s service=%s target_done_policy=%s",
            self.global_goal_source,
            self.target_route_validation,
            self.target_route_validation_service,
            self.target_done_policy,
        )
        if self.global_goal_source == "fixed":
            rospy.loginfo(
                "Fixed global goal configured: (%.2f, %.2f, %.2f), publish_period=%.2fs",
                *self.fixed_goal, self.fixed_goal_publish_period,
            )


    def mode_from_state(self, state: int, subtype: str) -> str:
        """冻结映射表：state/subtype -> 基础 goal 模式。"""
        if state == STATE_LOCKED:
            return CATCH_TARGET_MODE
        if state == STATE_SUSPICIOUS:
            if subtype == "Sus-A":
                return CATCH_TARGET_MODE
            if subtype == "Sus-B":
                return CATCH_CTX_MODE
            if subtype == "Sus-C":
                return EXPLORE_SUS_C_MODE
        # 默认 PASS
        return EXPLORE_PASS_MODE

def main():
    GoalManager()
    rospy.spin()


if __name__ == "__main__":
    main()
