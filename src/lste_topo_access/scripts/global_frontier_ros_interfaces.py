#!/usr/bin/env python3

"""ROS publishers, subscribers, and timers for global-frontier exploration."""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import Pose2D, PoseStamped
from lste_topo_access.msg import FrontierExecutionTerminal
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import RecoveryStatus
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from lste_msgs.msg import LsteDetections, LsteTask

from experiment_reset_contract import HARD_RESET_ACK_TOPIC, HARD_RESET_TOPIC


class GlobalFrontierRosInterfacesMixin:
    def _setup_ros_interfaces(self):
        """Create transport objects only after configuration and state exist."""
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)
        self.command_publisher = rospy.Publisher(
            self.command_topic, String, queue_size=10, latch=True
        )
        self.status_publisher = rospy.Publisher(
            self.status_topic, String, queue_size=10, latch=True
        )
        self.hard_reset_ack_publisher = rospy.Publisher(
            HARD_RESET_ACK_TOPIC, String, queue_size=20
        )
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.on_map, queue_size=1)
        rospy.Subscriber(self.costmap_topic, OccupancyGrid, self.on_costmap, queue_size=1)
        rospy.Subscriber(
            self.costmap_updates_topic,
            OccupancyGridUpdate,
            self.on_costmap_update,
            queue_size=10,
        )
        rospy.Subscriber(self.pose_topic, Pose2D, self.on_pose, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber(self.task_topic, LsteTask, self.on_task, queue_size=1)
        rospy.Subscriber(
            self.detections_topic,
            LsteDetections,
            self.on_detections,
            queue_size=1,
        )
        rospy.Subscriber(
            self.goal_arbitration_topic,
            String,
            self.on_goal_arbitration,
            queue_size=20,
        )
        rospy.Subscriber(
            self.replan_request_topic, String, self.on_replan_request, queue_size=10
        )
        rospy.Subscriber(self.task_done_topic, Bool, self.on_task_done, queue_size=1)
        rospy.Subscriber(
            self.recovery_topic,
            RecoveryStatus,
            self.on_move_base_recovery,
            queue_size=10,
        )
        rospy.Subscriber(
            self.bridge_status_topic,
            String,
            self.on_bridge_status,
            queue_size=20,
        )
        rospy.Subscriber(
            self.turn_status_topic,
            String,
            self.on_turn_status,
            queue_size=10,
        )
        rospy.Subscriber(
            self.terminal_contract_topic,
            FrontierExecutionTerminal,
            self.on_execution_terminal,
            queue_size=10,
        )
        rospy.Subscriber(
            HARD_RESET_TOPIC, String, self.on_hard_reset, queue_size=5
        )
        rospy.Timer(
            rospy.Duration(
                float(getattr(self, "lifecycle_tick_period", 1.0))
            ),
            self.on_timer,
        )
        rospy.loginfo(
            "Global frontier explorer started: map=%s costmap=%s scan=%s task=%s detections=%s arbitration=%s recovery=%s bridge=%s goal=%s "
            "route_tangent_weight=%.2f heading_hard_limit=%.1fdeg "
            "successor_envelopes=[%.1f,%.1f]deg "
            "semantic_hint_weight=%.2f task_semantic_value=%s branch_first=%s "
            "event_driven_deliberation=%s route_segment=%.2fm "
            "mission_endpoint_only=%s persistent_execution=%s turn_execution=%s planning_period=%.2fs "
            "navfn_startup_probe=%.2fm place_ledger=[radius=%.2fm delta=%.0f dwell=%.1fs failures=%d] "
            "structural_place=[furniture_span=%.2fm]",
            self.map_topic,
            self.costmap_topic,
            self.scan_topic,
            self.task_topic,
            self.detections_topic,
            self.goal_arbitration_topic,
            self.recovery_topic,
            self.bridge_status_topic,
            self.goal_topic,
            self.heading_weight,
            math.degrees(self.heading_hard_limit),
            math.degrees(self.successor_smooth_heading_limit),
            math.degrees(self.successor_curve_heading_limit),
            self.semantic_hint_weight,
            self.task_semantic_value_enabled,
            self.branch_first_enabled,
            self.event_driven_deliberation_enabled,
            self.route_segment_distance,
            self.mission_endpoint_only,
            self.persistent_execution,
            self.turn_execution_mode,
            self.planning_period,
            self.navfn_startup_probe_distance,
            self.region_memory_radius,
            self.region_information_delta,
            self.region_stagnation_timeout,
            self.region_failure_limit,
            self.place_furniture_max_span_m,
        )
