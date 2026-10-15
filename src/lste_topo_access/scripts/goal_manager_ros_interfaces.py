#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS publishers, subscribers, and timer wiring for GoalManager."""

import rospy
from geometry_msgs.msg import Pose2D, PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, String, UInt8

from lste_msgs.msg import LsteDetections, LsteFrontiers, LsteScores, LsteState, LsteTask

from experiment_reset_contract import HARD_RESET_ACK_TOPIC, HARD_RESET_TOPIC

class GoalManagerRosInterfacesMixin:
    def _setup_ros_interfaces(self):
        """Create Goal Manager publishers, subscribers, and its timer."""
        # 发布
        # Controllers can start after the first frontier decision.  Latching
        # the current goal makes startup order irrelevant and prevents a
        # wait_for_goal controller from remaining stopped until the next map
        # update happens to produce a different coordinate.
        self.pub_goal = rospy.Publisher(
            "/lste/final_goal", PoseStamped, queue_size=1, latch=True
        )
        # Machine-readable decision trace consumed by navigation metrics. The
        # human-readable GOAL_DIAG line remains for rosout/tmux inspection.
        self.pub_goal_diagnostic = rospy.Publisher(
            "/lste/goal_diagnostic", String, queue_size=10
        )
        self.pub_goal_intent = rospy.Publisher(
            self.goal_intent_topic, String, queue_size=1, latch=True
        )
        self.pub_goal_command = rospy.Publisher(
            self.goal_command_topic, String, queue_size=1, latch=True
        )
        self.goal_arbitration_topic = rospy.get_param(
            "~goal_arbitration_topic", "/lste/goal_arbitration"
        )
        self.pub_goal_arbitration = rospy.Publisher(
            self.goal_arbitration_topic, String, queue_size=10
        )
        self.pub_global_frontier_replan = rospy.Publisher(
            self.global_frontier_replan_request_topic, String, queue_size=10
        )
        self.pub_access_mode = rospy.Publisher("/lste/access_topo/active_mode", String, queue_size=1, latch=True)
        self.pub_task_done = rospy.Publisher("/lste/task_done", Bool, queue_size=1, latch=True)
        self.pub_navigation_hold = rospy.Publisher(
            self.navigation_hold_topic, Bool, queue_size=1, latch=True
        )
        self.pub_hard_reset_ack = rospy.Publisher(
            HARD_RESET_ACK_TOPIC, String, queue_size=20
        )
        self.pub_navigation_hold.publish(Bool(data=False))

        # 订阅
        self.sub_state = rospy.Subscriber("/lste/state", LsteState, self.on_state, queue_size=1)
        self.sub_dets = rospy.Subscriber("/lste/detections", LsteDetections, self.on_dets, queue_size=1)
        self.sub_scores = rospy.Subscriber("/lste/scores", LsteScores, self.on_scores, queue_size=1)
        self.sub_task = rospy.Subscriber("/lste/task", LsteTask, self.on_task, queue_size=1)
        self.sub_hard_reset = rospy.Subscriber(
            HARD_RESET_TOPIC, String, self.on_hard_reset, queue_size=5
        )
        self.sub_map = rospy.Subscriber(
            "/map", OccupancyGrid, self.on_map, queue_size=1
        )
        self.sub_pose = rospy.Subscriber("/rbt_pose", Pose2D, self.on_pose, queue_size=1)
        self.sub_cam_info = rospy.Subscriber("/kinect/hd/camera_info", CameraInfo, self.on_cam_info, queue_size=1)
        if self.use_depth:
            self.sub_depth = rospy.Subscriber(self.depth_topic, Image, self.on_depth, queue_size=1)
        self.sub_frontier = rospy.Subscriber(self.frontier_topic, Vector3Stamped, self.on_frontier, queue_size=1)
        self.sub_frontiers = rospy.Subscriber(self.frontiers_topic, LsteFrontiers, self.on_frontiers, queue_size=1)
        if self.global_frontier_enabled:
            self.sub_global_frontier = rospy.Subscriber(
                self.global_frontier_topic, PoseStamped, self.on_global_frontier, queue_size=1
            )
            if self.global_frontier_command_enabled:
                self.sub_global_frontier_command = rospy.Subscriber(
                    self.global_frontier_command_topic,
                    String,
                    self.on_global_frontier_command,
                    queue_size=10,
                )
            self.sub_global_frontier_status = rospy.Subscriber(
                self.global_frontier_status_topic,
                String,
                self.on_global_frontier_status,
                queue_size=10,
            )
        self.sub_teb_goal_terminal = rospy.Subscriber(
            self.teb_goal_terminal_topic, PoseStamped,
            self.on_teb_goal_terminal, queue_size=1,
        )
        self.sub_teb_goal_failure = rospy.Subscriber(
            self.teb_goal_failure_topic,
            String,
            self.on_teb_goal_failure,
            queue_size=10,
        )
        self.sub_teb_goal_bridge_status = rospy.Subscriber(
            self.teb_goal_bridge_status_topic,
            String,
            self.on_teb_goal_bridge_status,
            queue_size=10,
        )
        self.sub_controller_mode = rospy.Subscriber(
            self.controller_mode_topic, String, self.on_controller_mode,
            queue_size=1,
        )
        self.sub_scan = rospy.Subscriber(self.scan_topic, LaserScan, self.on_scan, queue_size=1)
        self.sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self.on_cmd_vel, queue_size=1)
        self.sub_access_mode = rospy.Subscriber("/lste/access_topo/mode", UInt8, self.on_access_mode, queue_size=1)
        self.sub_access_goal = rospy.Subscriber("/lste/access_topo/backtrack_goal", PoseStamped, self.on_access_goal, queue_size=1)
        if self.global_goal_source == "fixed" and self.fixed_goal_allow_click_override:
            self.sub_fixed_goal_click = rospy.Subscriber(
                self.fixed_goal_click_topic, PoseStamped, self.on_fixed_goal_click, queue_size=1,
            )

        self.timer = rospy.Timer(rospy.Duration(0.2), self.on_timer)  # 5Hz
