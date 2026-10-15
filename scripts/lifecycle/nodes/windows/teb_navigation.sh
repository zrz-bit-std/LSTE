#!/usr/bin/env bash
# TEB navigation and command arbitration tmux windows (12-16).

start_teb_navigation_windows() {
  tmux_new_window 12 "$WS" "teb_nav" \
    "$WAIT_ROSCORE; until rostopic list | grep -qx /pro3/rlscan; do sleep 1; done; \\
     roslaunch lste_topo_access teb_navigation.launch \\
       scan_topic:=/pro3/rlscan odom_topic:=/pro3/wheel_odom \\
       terminal_topic:=$TEB_GOAL_TERMINAL_TOPIC target_failure_topic:=$TEB_GOAL_FAILURE_TOPIC \\
       intent_topic:=$GOAL_INTENT_TOPIC goal_command_topic:=$GOAL_COMMAND_TOPIC \\
       max_linear_speed:=$TEB_MAX_LINEAR_SPEED controller_mode:=$LSTE_CONTROLLER \\
       teb_xy_goal_tolerance:=$TEB_XY_GOAL_TOLERANCE \\
       persistent_execution:=$TEB_PERSISTENT_EXECUTION \\
       base_global_planner:=$TEB_BASE_GLOBAL_PLANNER base_local_planner:=$TEB_BASE_LOCAL_PLANNER \\
       planner_frequency:=$TEB_PLANNER_FREQUENCY \\
       persistent_plan_equivalence_distance:=$TEB_PERSISTENT_CORRIDOR_RADIUS \\
       persistent_frontier_lookahead_handoff_enabled:=$TEB_PERSISTENT_FRONTIER_LOOKAHEAD_HANDOFF_ENABLED \\
       persistent_frontier_lookahead_trigger_distance:=$TEB_PERSISTENT_FRONTIER_LOOKAHEAD_TRIGGER_DISTANCE \\
       persistent_frontier_admission_horizon:=$TEB_PERSISTENT_FRONTIER_ADMISSION_HORIZON \\
       persistent_frontier_admission_max_costmap_age:=$TEB_PERSISTENT_FRONTIER_ADMISSION_MAX_COSTMAP_AGE \\
       persistent_frontier_admission_blocked_cost:=$TEB_PERSISTENT_FRONTIER_ADMISSION_BLOCKED_COST \\
       persistent_frontier_admission_service_timeout:=$TEB_PERSISTENT_FRONTIER_ADMISSION_SERVICE_TIMEOUT \\
       persistent_frontier_admission_endpoint_epsilon:=$TEB_PERSISTENT_FRONTIER_ADMISSION_ENDPOINT_EPSILON \\
       persistent_frontier_curve_handoff_max_heading_deg:=$TEB_PERSISTENT_FRONTIER_CURVE_HANDOFF_MAX_HEADING_DEG \\
       teb_homotopy_class_planning:=$TEB_HOMOTOPY_CLASS_PLANNING \\
       teb_costmap_converter_plugin:=$TEB_COSTMAP_CONVERTER_PLUGIN \\
       teb_costmap_converter_spin_thread:=$TEB_COSTMAP_CONVERTER_SPIN_THREAD \\
       teb_costmap_converter_rate:=$TEB_COSTMAP_CONVERTER_RATE \\
       handoff_distance:=$TEB_ACTION_HANDOFF_DISTANCE handoff_radius:=$TEB_ACTION_HANDOFF_RADIUS \\
       progress_timeout:=$TEB_ACTION_PROGRESS_TIMEOUT progress_epsilon:=$TEB_ACTION_PROGRESS_EPSILON \\
       handoff_min_interval:=$TEB_ACTION_HANDOFF_MIN_INTERVAL \\
       frontier_observation_completion_radius:=$TEB_FRONTIER_OBSERVATION_COMPLETION_RADIUS \\
       frontier_observation_completion_max_linear_speed:=$TEB_FRONTIER_OBSERVATION_COMPLETION_MAX_LINEAR_SPEED \\
       frontier_observation_stationary_hold:=$TEB_FRONTIER_OBSERVATION_STATIONARY_HOLD \\
       frontier_continuous_prefetch_handoff_enabled:=$TEB_FRONTIER_CONTINUOUS_PREFETCH_HANDOFF_ENABLED \\
       frontier_continuous_prefetch_handoff_timeout:=$TEB_FRONTIER_CONTINUOUS_PREFETCH_HANDOFF_TIMEOUT \\
       teb_reorientation_enabled:=$TEB_NATIVE_REORIENTATION_ENABLED \\
       teb_reorientation_linear_threshold:=$TEB_NATIVE_REORIENTATION_LINEAR_THRESHOLD \\
       teb_reorientation_angular_threshold:=$TEB_NATIVE_REORIENTATION_ANGULAR_THRESHOLD \\
       teb_reorientation_yaw_progress:=$TEB_NATIVE_REORIENTATION_YAW_PROGRESS \\
       teb_reorientation_stagnation_timeout:=$TEB_NATIVE_REORIENTATION_STAGNATION_TIMEOUT \\
       teb_reorientation_max_extension:=$TEB_NATIVE_REORIENTATION_MAX_EXTENSION \\
       teb_reorientation_feedback_timeout:=$TEB_NATIVE_REORIENTATION_FEEDBACK_TIMEOUT \\
       teb_frontier_pre_route_alignment:=$TEB_FRONTIER_PRE_ROUTE_ALIGNMENT \\
       navfn_allow_unknown:=$TEB_NAVFN_ALLOW_UNKNOWN \\
       target_early_handoff_distance:=$TEB_TARGET_EARLY_HANDOFF_DISTANCE \\
       target_early_handoff_min_delta:=$TEB_TARGET_EARLY_HANDOFF_MIN_DELTA \\
       frontier_replacement_max_delta:=$TEB_FRONTIER_REPLACEMENT_MAX_DELTA \\
       allow_route_continuation_replacement:=$TEB_ALLOW_ROUTE_CONTINUATION_REPLACEMENT \\
       frontier_early_handoff_max_heading_deg:=$TEB_FRONTIER_EARLY_HANDOFF_MAX_HEADING_DEG \\
       frontier_stale_recovery_grace:=$TEB_FRONTIER_STALE_RECOVERY_GRACE \\
       frontier_stale_recovery_max_distance:=$TEB_FRONTIER_STALE_RECOVERY_MAX_DISTANCE"

  if [[ "${LEGACY_GP_FRONTIER_ENABLED,,}" == "true" || "$LEGACY_GP_FRONTIER_ENABLED" == "1" ]]; then
    tmux_new_window 13 "$WS" "rviz_frontier" \
      "$WAIT_ROSCORE; rviz -d $GP_FRONTIER_RVIZ"
  fi

  tmux_new_window 14 "$WS" "controller_switch" \
    "$WAIT_ROSCORE; rosrun lste_core lste_controller_switch_node.py _initial_mode:=$LSTE_CONTROLLER"
  tmux_new_window 15 "$WS" "cmd_vel_mux" \
    "$WAIT_ROSCORE; rosrun lste_core lste_cmd_vel_mux_node.py \\
      _initial_mode:=$LSTE_CONTROLLER _teb_forward_only:=$TEB_FORWARD_ONLY \\
      _teb_angular_sign_switch_threshold:=$TEB_ANGULAR_SIGN_SWITCH_THRESHOLD \\
      _teb_angular_deadband:=$TEB_ANGULAR_DEADBAND _scan_topic:=/pro3/rlscan \\
      _teb_command_contract_topic:=/lste/cmd_vel/teb_contract \\
      _require_teb_command_contract:=true \\
      _governor_decel:=$MUX_GOVERNOR_DECEL _governor_min_gap:=$MUX_GOVERNOR_MIN_GAP \\
      _governor_max_speed:=$MUX_GOVERNOR_MAX_SPEED"
  tmux_new_window 16 "$WS" "health" \
    "$WAIT_ROSCORE; python3 $WS/scripts/tools/lste_health_audit.py \\
      _controller_mode:=$LSTE_CONTROLLER \\
      _legacy_gp_frontier_enabled:=$LEGACY_GP_FRONTIER_ENABLED \\
      _detector_node:=$DETECTOR_ROS_NODE"
}
