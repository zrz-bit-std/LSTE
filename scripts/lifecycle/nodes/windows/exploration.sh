#!/usr/bin/env bash
# Legacy GP (optional) and online-SLAM global-frontier tmux windows (10-11).

start_exploration_windows() {
  if [[ "${LEGACY_GP_FRONTIER_ENABLED,,}" == "true" || "$LEGACY_GP_FRONTIER_ENABLED" == "1" ]]; then
    tmux_new_window 10 "$WS" "gp_frontier" \
      "$WAIT_ROSCORE; conda activate vsgp; roslaunch lste_topo_access gp_frontier.launch \\
        access_topo_config:=$ACCESS_TOPO_CONFIG \\
        access_topo_config_pass:=$ACCESS_TOPO_CONFIG_PASS \\
        access_topo_config_sus_c:=$ACCESS_TOPO_CONFIG_SUS_C \\
        access_topo_test_name:=$ACCESS_TOPO_TEST_NAME access_topo_run_name:=$ACCESS_TOPO_RUN_NAME \\
        follow_locked_done_time:=$FOLLOW_LOCKED_DONE_TIME frontier_log:=$FRONTIER_LOG \\
        launch_goal_manager:=false"
  else
    echo "[nodes] Legacy GP frontier disabled; online SLAM map remains active."
  fi

  if [[ "${ONLINE_SLAM_ENABLED,,}" != "true" && "$ONLINE_SLAM_ENABLED" != "1" ]]; then
    echo "[nodes] Online SLAM disabled; Navfn visual-target validation is unavailable."
    return
  fi

  if [[ "${GLOBAL_FRONTIER_ENABLED,,}" != "true" && "$GLOBAL_FRONTIER_ENABLED" != "1" ]]; then
    echo "[nodes] Global frontier disabled; starting online SLAM without the exploration executive."
  fi

  tmux_new_window 11 "$WS" "global_frontier" \
    "$WAIT_ROSCORE; roslaunch lste_topo_access online_slam_frontier.launch \\
      goal_topic:=$GLOBAL_FRONTIER_TOPIC command_topic:=/lste/global_frontier/route_command \\
      terminal_topic:=$TEB_GOAL_TERMINAL_TOPIC clearance:=$GLOBAL_FRONTIER_CLEARANCE \\
      navfn_observation_recovery_enabled:=$GLOBAL_FRONTIER_NAVFN_OBSERVATION_RECOVERY_ENABLED \\
      fallback_clearance:=$GLOBAL_FRONTIER_FALLBACK_CLEARANCE \\
      frontier_approach_distance:=$GLOBAL_FRONTIER_APPROACH_DISTANCE \\
      min_path_distance:=$GLOBAL_FRONTIER_MIN_PATH_DISTANCE \\
      mission_endpoint_only:=$GLOBAL_FRONTIER_MISSION_ENDPOINT_ONLY \\
      teb_xy_goal_tolerance:=$TEB_XY_GOAL_TOLERANCE \\
      persistent_execution:=$TEB_PERSISTENT_EXECUTION \\
      lookahead_distance:=$GLOBAL_FRONTIER_LOOKAHEAD_DISTANCE \\
      waypoint_release_radius:=$GLOBAL_FRONTIER_WAYPOINT_RELEASE_RADIUS \\
      early_handoff_distance:=$GLOBAL_FRONTIER_EARLY_HANDOFF_RADIUS \\
      active_timeout:=$GLOBAL_FRONTIER_ACTIVE_TIMEOUT stall_timeout:=$GLOBAL_FRONTIER_STALL_TIMEOUT \\
      post_turn_stall_timeout:=$GLOBAL_FRONTIER_POST_TURN_STALL_TIMEOUT \\
      unreachable_grace:=$GLOBAL_FRONTIER_UNREACHABLE_GRACE rejected_timeout:=$GLOBAL_FRONTIER_REJECTED_TIMEOUT \\
      completed_radius:=$GLOBAL_FRONTIER_COMPLETED_RADIUS \\
      exploration_method:=$GLOBAL_FRONTIER_METHOD \\
      target_done_policy:=$TARGET_DONE_POLICY \\
      region_memory_radius:=$GLOBAL_FRONTIER_REGION_MEMORY_RADIUS \\
      region_information_delta:=$GLOBAL_FRONTIER_REGION_INFORMATION_DELTA \\
      region_stagnation_timeout:=$GLOBAL_FRONTIER_REGION_STAGNATION_TIMEOUT \\
      region_failure_limit:=$GLOBAL_FRONTIER_REGION_FAILURE_LIMIT \\
      structure_radius_cells:=$GLOBAL_FRONTIER_STRUCTURE_RADIUS_CELLS \\
      structure_weight:=$GLOBAL_FRONTIER_STRUCTURE_WEIGHT min_structure_cells:=$GLOBAL_FRONTIER_MIN_STRUCTURE_CELLS \\
      heading_weight:=$GLOBAL_FRONTIER_HEADING_WEIGHT \\
      heading_hard_limit_deg:=$GLOBAL_FRONTIER_HEADING_HARD_LIMIT_DEG \\
      successor_smooth_heading_limit_deg:=$TEB_FRONTIER_EARLY_HANDOFF_MAX_HEADING_DEG \\
      successor_curve_heading_limit_deg:=$TEB_PERSISTENT_FRONTIER_CURVE_HANDOFF_MAX_HEADING_DEG \\
      turn_execution_mode:=$GLOBAL_FRONTIER_TURN_EXECUTION_MODE \\
      explicit_turn_connector_threshold_deg:=$GLOBAL_FRONTIER_EXPLICIT_TURN_CONNECTOR_THRESHOLD_DEG \\
      turn_connector_distance:=$GLOBAL_FRONTIER_TURN_CONNECTOR_DISTANCE \\
      semantic_hint_weight:=$GLOBAL_FRONTIER_SEMANTIC_HINT_WEIGHT \\
      semantic_hint_max_distance:=$GLOBAL_FRONTIER_SEMANTIC_HINT_MAX_DISTANCE \\
      navfn_empty_warmup_seconds:=$GLOBAL_FRONTIER_NAVFN_EMPTY_WARMUP_SECONDS \\
      navfn_startup_probe_distance:=$GLOBAL_FRONTIER_NAVFN_STARTUP_PROBE_DISTANCE \\
      start_frontier:=$GLOBAL_FRONTIER_ENABLED \\
      planning_period:=$GLOBAL_FRONTIER_PLANNING_PERIOD"
}
