"""Parameter loading for the TEB turn supervisor.

All values here either select an existing ROS interface or bound the adapter
around TEB.  The adapter does not introduce a second translational controller.
"""

import math

import rospy


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def configure_turn_supervisor_parameters(supervisor):
    """Load topics and execution bounds into ``supervisor``."""
    gp = rospy.get_param
    supervisor.planner_cmd_topic = gp(
        "~planner_cmd_topic", "/lste/cmd_vel/teb_planner"
    )
    supervisor.command_contract_topic = gp(
        "~command_contract_topic", "/lste/cmd_vel/teb_contract"
    )
    supervisor.navfn_plan_topic = gp(
        "~navfn_plan_topic", "/move_base/NavfnROS/plan"
    )
    supervisor.output_cmd_topic = gp("~output_cmd_topic", "/lste/cmd_vel/teb")
    supervisor.goal_topic = gp("~goal_topic", "/lste/final_goal")
    supervisor.intent_topic = gp("~intent_topic", "/lste/goal_intent")
    supervisor.bridge_status_topic = gp(
        "~bridge_status_topic", "/lste/teb_goal_bridge/status"
    )
    supervisor.planner_command_contract_topic = gp(
        "~planner_command_contract_topic",
        "/lste/persistent_execution/planner_command_contract",
    )
    supervisor.recovery_status_topic = gp(
        "~recovery_status_topic", "/move_base/recovery_status"
    )
    supervisor.require_planner_command_contract = _as_bool(
        gp("~require_planner_command_contract", False)
    )
    supervisor.pose_topic = gp("~pose_topic", "/rbt_pose")
    supervisor.pose_frame = (
        str(gp("~pose_frame", "odom")).strip().lstrip("/") or "odom"
    )
    supervisor.scan_topic = gp("~scan_topic", "/pro3/rlscan")
    supervisor.mode_topic = gp("~mode_topic", "/lste/controller_mode")
    supervisor.task_done_topic = gp("~task_done_topic", "/lste/task_done")
    supervisor.navigation_hold_topic = gp(
        "~navigation_hold_topic", "/lste/navigation_hold"
    )
    supervisor.mux_status_topic = gp(
        "~mux_status_topic", "/lste/cmd_vel_mux/status"
    )
    supervisor.status_topic = gp(
        "~status_topic", "/lste/teb_turn_supervisor/status"
    )
    supervisor.active_mode = str(gp("~active_mode", "teb")).strip().lower()
    supervisor.mode = supervisor.active_mode

    supervisor.command_frequency = max(
        5.0,
        float(gp(
            "~command_frequency",
            rospy.get_param("/move_base/controller_frequency", 20.0),
        )),
    )
    # Keep angular limits in one source of truth: TEB's own configuration.
    supervisor.max_vel_theta = max(
        0.05,
        float(gp(
            "~max_vel_theta",
            rospy.get_param("/move_base/TebLocalPlannerROS/max_vel_theta", 0.65),
        )),
    )
    supervisor.acc_lim_theta = max(
        0.05,
        float(gp(
            "~acc_lim_theta",
            rospy.get_param("/move_base/TebLocalPlannerROS/acc_lim_theta", 0.90),
        )),
    )
    supervisor.yaw_goal_tolerance = max(
        0.02,
        float(gp(
            "~yaw_goal_tolerance",
            rospy.get_param(
                "/move_base/TebLocalPlannerROS/yaw_goal_tolerance", 0.35
            ),
        )),
    )
    supervisor.planner_command_timeout = max(
        0.10, float(gp("~planner_command_timeout", 0.30))
    )
    supervisor.recovery_command_timeout = max(
        0.10, float(gp("~recovery_command_timeout", 0.50))
    )

    supervisor.trajectory_continuity_enabled = _as_bool(
        gp("~trajectory_continuity_enabled", True)
    )
    supervisor.trajectory_feedback_timeout = max(
        0.02, float(gp("~trajectory_feedback_timeout", 0.35))
    )
    supervisor.trajectory_feedback_timeout_cap = max(
        supervisor.trajectory_feedback_timeout,
        float(gp("~trajectory_feedback_timeout_cap", 0.60)),
    )
    supervisor.trajectory_feedback_period_scale = max(
        1.0, float(gp("~trajectory_feedback_period_scale", 1.50))
    )
    supervisor.trajectory_continuity_min_forward = max(
        0.01, float(gp("~trajectory_continuity_min_forward", 0.05))
    )
    supervisor.trajectory_continuity_max_hold = max(
        0.02, float(gp("~trajectory_continuity_max_hold", 0.10))
    )
    supervisor.trajectory_continuity_terminal_radius = max(
        0.05, float(gp("~trajectory_continuity_terminal_radius", 0.75))
    )

    supervisor.turn_start_confirm_duration = max(
        0.0, float(gp("~turn_start_confirm_duration", 0.50))
    )
    supervisor.turn_rotation_cap_factor = max(
        1.1, float(gp("~turn_rotation_cap_factor", 1.6))
    )
    supervisor.turn_settle_duration = max(
        0.0, float(gp("~turn_settle_duration", 0.35))
    )
    supervisor.pre_route_turn_threshold = math.radians(max(
        30.0, min(90.0, float(gp("~pre_route_turn_threshold_deg", 75.0)))
    ))
    supervisor.endpoint_alignment_enabled = _as_bool(
        gp("~endpoint_alignment_enabled", False)
    )
    supervisor.stalled_route_reorientation_enabled = _as_bool(
        gp("~stalled_route_reorientation_enabled", True)
    )
    supervisor.stalled_route_reorientation_delay = max(
        0.10, float(gp("~stalled_route_reorientation_delay", 0.80))
    )
    supervisor.stalled_route_reorientation_heading = math.radians(max(
        20.0,
        min(90.0, float(gp("~stalled_route_reorientation_heading_deg", 45.0))),
    ))
    supervisor.stalled_route_reorientation_linear = max(
        0.001, float(gp("~stalled_route_reorientation_linear", 0.03))
    )
    supervisor.stalled_route_reorientation_angular = max(
        0.01, float(gp("~stalled_route_reorientation_angular", 0.15))
    )
    supervisor.stalled_route_reorientation_min_distance = max(
        0.20, float(gp("~stalled_route_reorientation_min_distance", 1.00))
    )
    # A circular base cannot safely rotate when lidar already reports an
    # obstacle inside its physical footprint.
    supervisor.turn_min_clearance = max(
        0.05,
        float(rospy.get_param(
            "/move_base/TebLocalPlannerROS/footprint_model/radius", 0.30
        )) + 0.02,
    )
    supervisor.turn_scan_timeout = 0.50
