"""ROS topic bindings and immutable run-start telemetry."""

from actionlib_msgs.msg import GoalStatusArray
from gazebo_msgs.msg import ContactsState, ModelStates
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from lste_topo_access.msg import FrontierExecutionTerminal, PlannerCommandContract
from lste_msgs.msg import LsteDetections, LsteScores, LsteState
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseActionGoal, RecoveryStatus
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
from std_msgs.msg import Bool, String
from teb_local_planner.msg import FeedbackMsg

import rospy

from experiment_reset_contract import HARD_RESET_TOPIC


class NavigationMetricsRosInterfacesMixin:
    """Declare the passive ROS subscriptions for the metrics observer."""

    def _write_run_start(self):
        # ``NavigationMetricsSetupMixin`` captures this before subscriptions
        # start.  Keep a lazy fallback for ROS-free adapters and older launch
        # compositions that call this method directly.
        run_context = getattr(self, "run_context", None)
        if not isinstance(run_context, dict):
            run_context = self._run_start_context()
        resolved_params = run_context.get("resolved_params")
        if not isinstance(resolved_params, dict):
            resolved_params = self._resolved_startup_params()
        self._write(
            "INFO",
            "run_start",
            run_timestamp=self.run_timestamp,
            log_path=str(self.log_path),
            retention_days=self.retention_days,
            ros_time=self.start_ros,
            topics={
                "pose": "/pro3/wheel_odom",
                "goal": "/lste/final_goal",
                "dispatch": "/move_base_simple/goal",
                "cmd_vel": "/cmd_vel",
                "scan": "/pro3/rlscan",
                "status": "/move_base/status",
                "teb_feedback": "/move_base/TebLocalPlannerROS/teb_feedback",
                "teb_planner_cmd": "/lste/cmd_vel/teb_planner",
                "teb_turn_supervisor_status": "/lste/teb_turn_supervisor/status",
                "teb_bridge_status": "/lste/teb_goal_bridge/status",
                "persistent_terminal": "/lste/teb_goal_terminal",
                "global_frontier_status": "/lste/global_frontier/status",
                "teb_goal_failure": "/lste/teb_goal_failure",
                "persistent_plan_event": "/lste/persistent_execution/plan_event",
                "goal_arbitration": "/lste/goal_arbitration",
                "perception_decision": "/lste/perception_decision",
                "navigation_hold": "/lste/navigation_hold",
                "benchmark_coverage": (
                    "manifest floor truth projected into /map"
                    if self._benchmark_truth is not None else None
                ),
                "benchmark_contacts": (
                    "/lste/benchmark/contact_states"
                    if self.benchmark_collision_truth_enabled else None
                ),
                "global_costmap": "/move_base/global_costmap/costmap",
                "local_costmap": "/move_base/local_costmap/costmap",
                "gazebo_model_states": "/gazebo/model_states" if self.target_eval_enabled else None,
                "camera_info": self.target_eval_camera_info_topic if self.target_eval_enabled else None,
                "depth": (
                    self.target_eval_depth_topic
                    if self.target_eval_enabled and self.target_eval_depth_enabled
                    else None
                ),
                "hard_reset": HARD_RESET_TOPIC,
                "experiment_boundary": "/lste/experiment/trial_boundary",
            },
            resolved_params=resolved_params,
            experiment=run_context["experiment"],
            task_definition=run_context["task_definition"],
            detector=run_context["detector"],
            target_thresholds=run_context["target_thresholds"],
            target_geometric_evaluation=self._target_eval_configuration(),
            failure_evidence_log=(
                None
                if self.failure_log_path is None
                else str(self.failure_log_path)
            ),
            failure_artifact_pattern=(
                None
                if self.failure_log_path is None
                else str(
                    self.failure_log_path.parent
                    / (self.run_timestamp + "_failure_<sequence>.json")
                )
            ),
        )
        rospy.loginfo("Navigation metrics log: %s", self.log_path)

    def _setup_ros_interfaces(self):
        rospy.Subscriber("/pro3/wheel_odom", Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber("/rbt_pose", Pose2D, self.on_pose2d, queue_size=1)
        rospy.Subscriber("/lste/final_goal", PoseStamped, self.on_goal, queue_size=1)
        # The action bridge executes this atomic GoalManager contract. Observe
        # it directly so a goal-change record has the correct source/route id
        # even before the human-readable diagnostic callback arrives.
        rospy.Subscriber("/lste/mission_goal", String, self.on_mission_goal, queue_size=10)
        rospy.Subscriber("/lste/goal_diagnostic", String, self.on_goal_diagnostic, queue_size=10)
        rospy.Subscriber("/lste/goal_arbitration", String, self.on_goal_arbitration, queue_size=10)
        # The TEB bridge uses the typed move_base action. Keep the legacy
        # simple-goal observer for older comparison launches, but count either
        # transport through the same dispatch recorder.
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.on_action_dispatch, queue_size=1)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.on_dispatch, queue_size=1)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.on_status, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self.on_cmd, queue_size=1)
        rospy.Subscriber(
            "/lste/cmd_vel_mux/status", String, self.on_cmd_vel_mux_status,
            queue_size=50,
        )
        rospy.Subscriber("/lste/cmd_vel/teb", Twist, self.on_teb_cmd, queue_size=1)
        rospy.Subscriber(
            "/lste/cmd_vel/teb_planner", Twist, self.on_teb_planner_cmd, queue_size=1
        )
        rospy.Subscriber("/pro3/rlscan", LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber("/lste/controller_mode", String, self.on_controller_mode, queue_size=1)
        rospy.Subscriber(
            "/lste/teb_goal_bridge/status", String, self.on_bridge_status, queue_size=10
        )
        rospy.Subscriber(
            "/lste/teb_goal_terminal",
            PoseStamped,
            self.on_persistent_execution_terminal,
            queue_size=10,
        )
        rospy.Subscriber(
            "/lste/global_frontier/status",
            String,
            self.on_global_frontier_status,
            queue_size=20,
        )
        rospy.Subscriber(
            "/lste/teb_turn_supervisor/status",
            String,
            self.on_turn_supervisor_status,
            queue_size=10,
        )
        rospy.Subscriber("/lste/sappo_controller_status", String, self.on_controller_status, queue_size=1)
        rospy.Subscriber(
            "/move_base/TebLocalPlannerROS/teb_feedback",
            FeedbackMsg,
            self.on_teb_feedback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/lste/persistent_execution/planner_command_contract",
            PlannerCommandContract,
            self.on_planner_command_contract,
            queue_size=20,
        )
        rospy.Subscriber(
            "/lste/teb_goal_terminal_contract",
            FrontierExecutionTerminal,
            self.on_frontier_execution_terminal,
            queue_size=10,
        )
        rospy.Subscriber(
            "/lste/persistent_execution/plan_event",
            String,
            self.on_persistent_plan_event,
            queue_size=20,
        )
        rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.on_move_base_feedback, queue_size=1)
        rospy.Subscriber("/move_base/recovery_status", RecoveryStatus, self.on_recovery, queue_size=1)
        rospy.Subscriber("/lste/state", LsteState, self.on_state, queue_size=1)
        rospy.Subscriber(
            self.target_eval_detections_topic,
            LsteDetections,
            self.on_detections,
            queue_size=1,
        )
        if self.target_eval_enabled:
            rospy.Subscriber(
                "/gazebo/model_states", ModelStates, self.on_gazebo_model_states, queue_size=5
            )
            rospy.Subscriber(
                self.target_eval_camera_info_topic,
                CameraInfo,
                self.on_target_eval_camera_info,
                queue_size=1,
            )
            if self.target_eval_depth_enabled and self.target_eval_depth_topic:
                rospy.Subscriber(
                    self.target_eval_depth_topic,
                    CompressedImage,
                    self.on_target_eval_depth,
                    queue_size=2,
                    buff_size=8 * 1024 * 1024,
                )
        rospy.Subscriber(
            "/lste/perception_decision", String, self.on_perception_decision, queue_size=20
        )
        rospy.Subscriber("/lste/scores", LsteScores, self.on_scores, queue_size=1)
        rospy.Subscriber("/lste/task_done", Bool, self.on_task_done, queue_size=1)
        rospy.Subscriber("/lste/navigation_hold", Bool, self.on_navigation_hold, queue_size=1)
        rospy.Subscriber(
            "/lste/teb_goal_failure", String, self.on_teb_goal_failure, queue_size=20
        )
        rospy.Subscriber(
            HARD_RESET_TOPIC, String, self.on_hard_reset, queue_size=20
        )
        rospy.Subscriber(
            "/lste/experiment/trial_boundary",
            String,
            self.on_experiment_boundary,
            queue_size=10,
        )
        rospy.Subscriber("/map", OccupancyGrid, self.on_map, queue_size=1)
        if self.benchmark_collision_truth_enabled:
            rospy.Subscriber(
                "/lste/benchmark/contact_states", ContactsState,
                self.on_benchmark_contacts, queue_size=20,
            )
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.on_global_costmap, queue_size=1)
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.on_local_costmap, queue_size=1)
        rospy.Subscriber("/move_base/NavfnROS/plan", NavPath, self.on_navfn_plan, queue_size=1)
        rospy.Subscriber("/move_base/GlobalPlanner/plan", NavPath, self.on_global_planner_plan, queue_size=1)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", NavPath, self.on_teb_global_plan, queue_size=1)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", NavPath, self.on_teb_local_plan, queue_size=1)
        rospy.Timer(rospy.Duration(0.5), self.on_sample)
        # A faster bounded ring is cheap (only summaries, never raw images or
        # full costmaps) and gives every failure a pre-event context window.
        rospy.Timer(
            rospy.Duration(self.failure_evidence_period),
            self.on_failure_evidence_sample,
        )
        rospy.on_shutdown(self.close)
