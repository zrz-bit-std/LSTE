"""ROS publishers, subscribers, and timer wiring for the turn supervisor."""

import rospy
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from lste_topo_access.msg import PlannerCommandContract
from move_base_msgs.msg import RecoveryStatus
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from teb_local_planner.msg import FeedbackMsg

from experiment_reset_contract import HARD_RESET_ACK_TOPIC, HARD_RESET_TOPIC


def connect_turn_supervisor_ros(supervisor):
    """Connect all runtime endpoints after parameters and state are ready."""
    supervisor.output_pub = rospy.Publisher(
        supervisor.output_cmd_topic, Twist, queue_size=10
    )
    supervisor.command_contract_pub = rospy.Publisher(
        supervisor.command_contract_topic, String, queue_size=10
    )
    supervisor.status_pub = rospy.Publisher(
        supervisor.status_topic, String, queue_size=10, latch=True
    )
    supervisor.hard_reset_ack_pub = rospy.Publisher(
        HARD_RESET_ACK_TOPIC, String, queue_size=20
    )
    rospy.Subscriber(
        supervisor.planner_cmd_topic, Twist, supervisor.on_planner_command,
        queue_size=1,
    )
    rospy.Subscriber(
        supervisor.planner_command_contract_topic,
        PlannerCommandContract,
        supervisor.on_planner_command_contract,
        queue_size=20,
    )
    rospy.Subscriber(
        supervisor.recovery_status_topic,
        RecoveryStatus,
        supervisor.on_recovery_status,
        queue_size=10,
    )
    rospy.Subscriber(
        "/move_base/TebLocalPlannerROS/teb_feedback", FeedbackMsg,
        supervisor.on_teb_feedback, queue_size=1,
    )
    rospy.Subscriber(
        supervisor.navfn_plan_topic, Path, supervisor.on_navfn_plan, queue_size=1
    )
    rospy.Subscriber(supervisor.goal_topic, PoseStamped, supervisor.on_goal, queue_size=1)
    rospy.Subscriber(supervisor.intent_topic, String, supervisor.on_intent, queue_size=1)
    rospy.Subscriber(
        supervisor.bridge_status_topic, String, supervisor.on_bridge_status,
        queue_size=10,
    )
    rospy.Subscriber(supervisor.pose_topic, Pose2D, supervisor.on_pose, queue_size=1)
    rospy.Subscriber(supervisor.scan_topic, LaserScan, supervisor.on_scan, queue_size=1)
    rospy.Subscriber(supervisor.mode_topic, String, supervisor.on_mode, queue_size=1)
    rospy.Subscriber(
        supervisor.task_done_topic, Bool, supervisor.on_task_done, queue_size=1
    )
    rospy.Subscriber(
        supervisor.navigation_hold_topic, Bool, supervisor.on_navigation_hold,
        queue_size=1,
    )
    rospy.Subscriber(
        supervisor.mux_status_topic, String, supervisor.on_cmd_vel_mux_status,
        queue_size=10,
    )
    rospy.Subscriber(
        HARD_RESET_TOPIC, String, supervisor.on_hard_reset, queue_size=5
    )
    supervisor.timer = rospy.Timer(
        rospy.Duration(1.0 / supervisor.command_frequency), supervisor.on_timer
    )
    rospy.on_shutdown(supervisor.on_shutdown)
