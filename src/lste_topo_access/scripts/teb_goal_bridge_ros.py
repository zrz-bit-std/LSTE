#!/usr/bin/env python3
"""ROS publishers, subscribers, and timers owned by the TEB goal bridge."""

import rospy
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from lste_topo_access.msg import (
    FrontierExecutionTerminal,
    PersistentGoalCommand,
    PlannerCommandContract,
)
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool, String
from teb_local_planner.msg import FeedbackMsg

from experiment_reset_contract import HARD_RESET_ACK_TOPIC, HARD_RESET_TOPIC


def connect_bridge_ros(bridge):
    """Connect the already initialized bridge to ROS exactly once."""
    _create_publishers(bridge)
    # A hot restart must not leave a speculative target request latched.
    bridge._clear_persistent_target_request_locked("bridge_startup", force=True)
    _create_subscribers(bridge)
    rospy.Timer(
        rospy.Duration(float(getattr(bridge, "lifecycle_tick_period", 0.2))),
        bridge.on_timer,
    )


def _create_publishers(bridge):
    bridge.terminal_pub = rospy.Publisher(bridge.terminal_topic, PoseStamped, queue_size=1)
    bridge.terminal_contract_pub = rospy.Publisher(
        bridge.terminal_contract_topic,
        FrontierExecutionTerminal,
        queue_size=10,
    )
    bridge.target_failure_pub = rospy.Publisher(
        bridge.target_failure_topic, String, queue_size=10
    )
    bridge.persistent_target_goal_pub = rospy.Publisher(
        bridge.persistent_target_goal_topic, PoseStamped, queue_size=1, latch=True
    )
    bridge.persistent_target_command_pub = rospy.Publisher(
        bridge.persistent_target_command_topic,
        PersistentGoalCommand,
        queue_size=1,
        latch=True,
    )
    bridge.persistent_mission_goal_pub = rospy.Publisher(
        bridge.persistent_mission_goal_topic, PoseStamped, queue_size=1, latch=True
    )
    bridge.persistent_mission_command_pub = rospy.Publisher(
        bridge.persistent_mission_command_topic,
        PersistentGoalCommand,
        queue_size=1,
        latch=True,
    )
    bridge.bridge_status_pub = rospy.Publisher(
        bridge.bridge_status_topic, String, queue_size=10, latch=True
    )
    bridge.hard_reset_ack_pub = rospy.Publisher(
        HARD_RESET_ACK_TOPIC, String, queue_size=20
    )
    bridge.planner_command_invalidation_pub = rospy.Publisher(
        bridge.planner_command_contract_topic,
        PlannerCommandContract,
        queue_size=20,
    )


def _create_subscribers(bridge):
    if bridge.use_goal_command:
        # Atomic mission commands are the sole bridge ownership boundary.
        # Keeping the legacy pose and intent streams in this lifecycle queue
        # lets their independently ordered transaction ids advance the bridge
        # ahead of the command that carries the matching goal.
        rospy.Subscriber(
            bridge.goal_command_topic, String, bridge.on_goal_command, queue_size=1
        )
    else:
        rospy.Subscriber(bridge.goal_topic, PoseStamped, bridge.on_goal, queue_size=1)
        rospy.Subscriber(bridge.intent_topic, String, bridge.on_intent, queue_size=1)
    rospy.Subscriber(
        bridge.frontier_status_topic, String, bridge.on_frontier_status, queue_size=10
    )
    rospy.Subscriber(
        bridge.turn_supervisor_status_topic,
        String,
        bridge.on_turn_supervisor_status,
        queue_size=10,
    )
    rospy.Subscriber(
        bridge.teb_feedback_topic, FeedbackMsg, bridge.on_teb_feedback, queue_size=10
    )
    rospy.Subscriber(
        bridge.teb_planner_cmd_topic,
        Twist,
        bridge.on_teb_planner_command,
        queue_size=20,
    )
    rospy.Subscriber(
        bridge.planner_command_contract_topic,
        PlannerCommandContract,
        bridge.on_planner_command_contract,
        queue_size=20,
    )
    rospy.Subscriber(bridge.navfn_plan_topic, Path, bridge.on_navfn_plan, queue_size=2)
    rospy.Subscriber(
        bridge.local_costmap_topic,
        OccupancyGrid,
        bridge.on_local_costmap,
        queue_size=1,
    )
    rospy.Subscriber(bridge.pose_topic, Pose2D, bridge.on_pose, queue_size=10)
    rospy.Subscriber(bridge.controller_topic, String, bridge.on_mode, queue_size=1)
    rospy.Subscriber(bridge.task_done_topic, Bool, bridge.on_task_done, queue_size=1)
    rospy.Subscriber(
        bridge.navigation_hold_topic,
        Bool,
        bridge.on_navigation_hold,
        queue_size=1,
    )
    rospy.Subscriber(
        bridge.persistent_target_plan_result_topic,
        String,
        bridge.on_persistent_target_plan_result,
        queue_size=10,
    )
    rospy.Subscriber(
        bridge.persistent_frontier_endpoint_topic,
        PoseStamped,
        bridge.on_persistent_frontier_endpoint_reached,
        queue_size=10,
    )
    rospy.Subscriber(
        HARD_RESET_TOPIC, String, bridge.on_hard_reset, queue_size=5
    )
