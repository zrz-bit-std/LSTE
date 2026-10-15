#!/usr/bin/env python3
"""Parameter loading for :mod:`lste_teb_goal_bridge`.

The bridge has a large execution contract, but its parameter handling should
not obscure that contract.  This module groups parameters by the subsystem
they affect and keeps all derived limits next to the source values that define
them.  It intentionally writes the resolved compatibility values back to the
private ROS namespace, matching the former in-class implementation.
"""

import math

import rospy


_TOPIC_DEFAULTS = (
    ("goal_topic", "~goal_topic", "/lste/final_goal"),
    ("simple_goal_topic", "~simple_goal_topic", "/move_base_simple/goal"),
    ("cancel_topic", "~cancel_topic", "/move_base/cancel"),
    ("terminal_topic", "~terminal_topic", "/lste/teb_goal_terminal"),
    (
        "terminal_contract_topic",
        "~terminal_contract_topic",
        "/lste/teb_goal_terminal_contract",
    ),
    ("target_failure_topic", "~target_failure_topic", "/lste/teb_goal_failure"),
    ("intent_topic", "~intent_topic", "/lste/goal_intent"),
    ("goal_command_topic", "~goal_command_topic", "/lste/mission_goal"),
    ("frontier_status_topic", "~frontier_status_topic", "/lste/global_frontier/status"),
    ("controller_topic", "~controller_topic", "/lste/controller_mode"),
    ("task_done_topic", "~task_done_topic", "/lste/task_done"),
    ("navigation_hold_topic", "~navigation_hold_topic", "/lste/navigation_hold"),
    (
        "persistent_target_goal_topic",
        "~persistent_target_goal_topic",
        "/lste/persistent_execution/target_goal",
    ),
    (
        "persistent_mission_goal_topic",
        "~persistent_mission_goal_topic",
        "/lste/persistent_execution/mission_goal",
    ),
    (
        "persistent_target_command_topic",
        "~persistent_target_command_topic",
        "/lste/persistent_execution/target_command",
    ),
    (
        "persistent_mission_command_topic",
        "~persistent_mission_command_topic",
        "/lste/persistent_execution/mission_command",
    ),
    (
        "persistent_installed_target_goal_topic",
        "~persistent_installed_target_goal_topic",
        "/lste/persistent_execution/installed_target_goal",
    ),
    (
        "persistent_installed_target_command_topic",
        "~persistent_installed_target_command_topic",
        "/lste/persistent_execution/installed_target_command",
    ),
    (
        "persistent_target_approach_topic",
        "~persistent_target_approach_topic",
        "/lste/persistent_execution/target_approach",
    ),
    (
        "persistent_frontier_endpoint_topic",
        "~persistent_frontier_endpoint_topic",
        "/lste/persistent_execution/frontier_endpoint_reached",
    ),
    (
        "persistent_target_plan_result_topic",
        "~persistent_target_plan_result_topic",
        "/lste/persistent_execution/target_plan_result",
    ),
    ("bridge_status_topic", "~bridge_status_topic", "/lste/teb_goal_bridge/status"),
    (
        "turn_supervisor_status_topic",
        "~turn_supervisor_status_topic",
        "/lste/teb_turn_supervisor/status",
    ),
    (
        "teb_feedback_topic",
        "~teb_feedback_topic",
        "/move_base/TebLocalPlannerROS/teb_feedback",
    ),
    ("teb_planner_cmd_topic", "~teb_planner_cmd_topic", "/lste/cmd_vel/teb_planner"),
    (
        "planner_command_contract_topic",
        "~planner_command_contract_topic",
        "/lste/persistent_execution/planner_command_contract",
    ),
    ("navfn_plan_topic", "~navfn_plan_topic", "/move_base/NavfnROS/plan"),
    (
        "local_costmap_topic",
        "~local_costmap_topic",
        "/move_base/local_costmap/costmap",
    ),
    (
        "navfn_make_plan_service",
        "~navfn_make_plan_service",
        "/move_base/NavfnROS/make_plan",
    ),
    ("pose_topic", "~pose_topic", "/rbt_pose"),
)


def configure_bridge_parameters(bridge):
    """Load all bridge parameters and publish derived compatibility values."""
    gp = rospy.get_param
    _configure_topics(bridge, gp)
    _configure_execution_mode(bridge, gp)
    _configure_action_health(bridge, gp)
    _configure_teb_execution(bridge, gp)
    _configure_route_handoff(bridge, gp)
    _publish_derived_parameters(bridge)


def _configure_topics(bridge, gp):
    """Resolve every ROS API endpoint in one inspectable table."""
    for attribute, parameter, default in _TOPIC_DEFAULTS:
        setattr(bridge, attribute, gp(parameter, default))

    bridge.use_goal_command = bridge._as_bool(gp("~use_goal_command", True))
    bridge.require_intent = bridge._as_bool(gp("~require_intent", True))
    bridge.global_frame = str(gp("~global_frame", "map")).strip().lstrip("/") or "map"


def _configure_execution_mode(bridge, gp):
    """Configure the move_base action contract and persistent executor."""
    bridge.persistent_frontier_lookahead_handoff_enabled = bridge._as_bool(
        gp("~persistent_frontier_lookahead_handoff_enabled", False)
    )
    bridge.persistent_frontier_lookahead_trigger_distance = max(
        0.20, float(gp("~persistent_frontier_lookahead_trigger_distance", 1.0))
    )
    bridge.persistent_frontier_admission_horizon = max(
        0.20, float(gp("~persistent_frontier_admission_horizon", 2.0))
    )
    bridge.persistent_frontier_admission_max_costmap_age = max(
        0.05, float(gp("~persistent_frontier_admission_max_costmap_age", 0.50))
    )
    bridge.persistent_frontier_admission_blocked_cost = min(
        100, max(1, int(gp("~persistent_frontier_admission_blocked_cost", 100)))
    )
    bridge.persistent_frontier_admission_service_timeout = max(
        0.01, float(gp("~persistent_frontier_admission_service_timeout", 0.10))
    )
    bridge.persistent_frontier_admission_endpoint_epsilon = max(
        0.05, float(gp("~persistent_frontier_admission_endpoint_epsilon", 0.20))
    )

    bridge.active_mode = str(gp("~active_mode", "teb")).strip().lower()
    bridge.mode = str(gp("~initial_mode", "teb")).strip().lower()
    bridge.persistent_execution = bridge._as_bool(gp("~persistent_execution", False))
    bridge.require_canonical_identity = bridge._as_bool(
        gp("~require_canonical_identity", bridge.persistent_execution)
    )
    bridge.allow_in_place_replacement = bridge._as_bool(
        gp("~allow_in_place_replacement", False)
    )
    bridge.allow_route_continuation_replacement = bridge._as_bool(
        gp("~allow_route_continuation_replacement", True)
    )
    bridge.position_epsilon = max(0.001, float(gp("~position_epsilon", 0.05)))
    bridge.yaw_epsilon = max(0.001, float(gp("~yaw_epsilon", 0.08)))
    bridge.compare_goal_yaw = bridge._as_bool(gp("~compare_goal_yaw", False))
    bridge.min_update_interval = max(0.0, float(gp("~min_update_interval", 1.5)))
    bridge.goal_retry_interval = max(
        bridge.min_update_interval, float(gp("~goal_retry_interval", 2.0))
    )


def _configure_action_health(bridge, gp):
    """Configure bounded action health and observation completion checks."""
    bridge.handoff_distance = max(
        bridge.position_epsilon * 2.0, float(gp("~handoff_distance", 1.0))
    )
    # Retained only for launch-file compatibility; health uses progress timeout.
    bridge.handoff_radius = max(
        bridge.position_epsilon * 2.0, float(gp("~handoff_radius", 0.85))
    )
    bridge.progress_timeout = max(2.0, float(gp("~progress_timeout", 12.0)))
    bridge.progress_epsilon = max(0.01, float(gp("~progress_epsilon", 0.12)))
    bridge.handoff_min_interval = max(
        bridge.min_update_interval, float(gp("~handoff_min_interval", 4.0))
    )
    bridge.frontier_observation_completion_radius = max(
        0.0, float(gp("~frontier_observation_completion_radius", 0.90))
    )
    bridge.frontier_observation_completion_max_linear_speed = max(
        0.0,
        float(gp("~frontier_observation_completion_max_linear_speed", 0.15)),
    )
    bridge.frontier_observation_stationary_hold = max(
        0.0, float(gp("~frontier_observation_stationary_hold", 0.10))
    )
    bridge.teb_planner_command_timeout = max(
        0.05, float(gp("~teb_planner_command_timeout", 0.30))
    )
    bridge.frontier_continuous_prefetch_handoff_enabled = bridge._as_bool(
        gp("~frontier_continuous_prefetch_handoff_enabled", False)
    )
    bridge.frontier_continuous_prefetch_handoff_timeout = max(
        0.20, float(gp("~frontier_continuous_prefetch_handoff_timeout", 1.50))
    )


def _configure_teb_execution(bridge, gp):
    """Configure the bounded exemption for genuine in-place TEB turns."""
    bridge.teb_reorientation_enabled = bridge._as_bool(
        gp("~teb_reorientation_enabled", True)
    )
    bridge.teb_reorientation_linear_threshold = max(
        0.001, float(gp("~teb_reorientation_linear_threshold", 0.03))
    )
    bridge.teb_reorientation_angular_threshold = max(
        0.01, float(gp("~teb_reorientation_angular_threshold", 0.15))
    )
    bridge.teb_reorientation_feedback_timeout = max(
        0.05, float(gp("~teb_reorientation_feedback_timeout", 0.50))
    )
    bridge.teb_reorientation_yaw_progress = max(
        0.01, float(gp("~teb_reorientation_yaw_progress", 0.10))
    )
    bridge.teb_reorientation_stagnation_timeout = max(
        0.1, float(gp("~teb_reorientation_stagnation_timeout", 1.50))
    )
    bridge.teb_reorientation_max_extension = max(
        bridge.teb_reorientation_stagnation_timeout,
        float(gp("~teb_reorientation_max_extension", 8.0)),
    )


def _configure_route_handoff(bridge, gp):
    """Derive safe route-continuation envelopes from TEB's own tolerances."""
    bridge.target_early_handoff_distance = max(
        bridge.position_epsilon * 2.0,
        float(gp("~target_early_handoff_distance", 0.70)),
    )
    bridge.target_early_handoff_min_delta = max(
        bridge.position_epsilon * 2.0,
        float(gp("~target_early_handoff_min_delta", 0.40)),
    )
    bridge.in_place_replacement_max_delta = max(
        bridge.target_early_handoff_min_delta,
        float(
            rospy.get_param(
                "/move_base/TebLocalPlannerROS/force_reinit_new_goal_dist", 4.0
            )
        ),
    )
    teb_xy_goal_tolerance = max(
        bridge.position_epsilon,
        float(rospy.get_param("/move_base/TebLocalPlannerROS/xy_goal_tolerance", 0.35)),
    )
    bridge.in_place_replacement_min_distance = max(
        bridge.position_epsilon * 2.0, teb_xy_goal_tolerance * 1.8
    )
    bridge.frontier_replacement_min_delta = max(bridge.position_epsilon * 2.0, 0.10)
    bridge.frontier_replacement_max_delta = max(
        bridge.frontier_replacement_min_delta,
        float(gp("~frontier_replacement_max_delta", 1.0)),
    )
    bridge.frontier_early_handoff_max_delta = max(
        bridge.frontier_replacement_max_delta, bridge.in_place_replacement_max_delta
    )
    bridge.frontier_early_handoff_max_heading_delta = math.radians(
        max(1.0, float(gp("~frontier_early_handoff_max_heading_deg", 45.0)))
    )
    bridge.persistent_frontier_curve_handoff_max_heading_delta = math.radians(
        min(
            89.0,
            max(
                math.degrees(bridge.frontier_early_handoff_max_heading_delta),
                float(gp("~persistent_frontier_curve_handoff_max_heading_deg", 65.0)),
            ),
        )
    )
    bridge.persistent_frontier_entry_tangent_distance = max(
        0.20, float(gp("~persistent_frontier_entry_tangent_distance", 0.50))
    )
    bridge.in_place_replacement_max_distance = max(
        bridge.target_early_handoff_distance, teb_xy_goal_tolerance * 3.0
    )
    bridge.frontier_early_handoff_min_distance = max(
        bridge.in_place_replacement_min_distance, 0.85
    )
    bridge.frontier_early_handoff_max_distance = max(
        bridge.in_place_replacement_max_distance,
        bridge.frontier_early_handoff_min_distance + 0.45,
    )
    bridge.frontier_sharp_replacement_min_distance = max(
        bridge.position_epsilon * 2.0, teb_xy_goal_tolerance * 1.10
    )
    bridge.frontier_stale_recovery_grace = max(
        0.0, float(gp("~frontier_stale_recovery_grace", 4.0))
    )
    bridge.frontier_stale_recovery_max_distance = max(
        bridge.frontier_early_handoff_max_distance,
        float(gp("~frontier_stale_recovery_max_distance", 1.50)),
    )


def _publish_derived_parameters(bridge):
    """Expose derived handoff limits for launch-time diagnostics."""
    resolved = {
        "~in_place_replacement_max_delta": bridge.in_place_replacement_max_delta,
        "~frontier_replacement_max_delta": bridge.frontier_replacement_max_delta,
        "~in_place_replacement_min_distance": bridge.in_place_replacement_min_distance,
        "~in_place_replacement_max_distance": bridge.in_place_replacement_max_distance,
        "~frontier_replacement_min_delta": bridge.frontier_replacement_min_delta,
        "~frontier_early_handoff_max_delta": bridge.frontier_early_handoff_max_delta,
        "~frontier_early_handoff_max_heading_deg": math.degrees(
            bridge.frontier_early_handoff_max_heading_delta
        ),
        "~persistent_frontier_curve_handoff_max_heading_deg": math.degrees(
            bridge.persistent_frontier_curve_handoff_max_heading_delta
        ),
        "~persistent_frontier_entry_tangent_distance": (
            bridge.persistent_frontier_entry_tangent_distance
        ),
        "~frontier_stale_recovery_grace": bridge.frontier_stale_recovery_grace,
        "~frontier_stale_recovery_max_distance": bridge.frontier_stale_recovery_max_distance,
        "~frontier_sharp_replacement_min_distance": (
            bridge.frontier_sharp_replacement_min_distance
        ),
    }
    for parameter, value in resolved.items():
        rospy.set_param(parameter, value)
