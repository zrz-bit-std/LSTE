#!/usr/bin/env bash
# Navigation telemetry and final controller activation (window 17 and exit).

# Failure evidence is normally sampled with a conservative pre/post window.
# Diagnostic runners may override these environment values for a short,
# targeted reproduction without changing the formal benchmark defaults.
NAVIGATION_FAILURE_EVIDENCE_ENABLED=${NAVIGATION_FAILURE_EVIDENCE_ENABLED:-${CFG_NAVIGATION_FAILURE_EVIDENCE_ENABLED:-true}}
NAVIGATION_FAILURE_EVIDENCE_SAMPLE_PERIOD=${NAVIGATION_FAILURE_EVIDENCE_SAMPLE_PERIOD:-${CFG_NAVIGATION_FAILURE_EVIDENCE_SAMPLE_PERIOD:-0.20}}
NAVIGATION_FAILURE_EVIDENCE_PRE_WINDOW=${NAVIGATION_FAILURE_EVIDENCE_PRE_WINDOW:-${CFG_NAVIGATION_FAILURE_EVIDENCE_PRE_WINDOW:-12.0}}
NAVIGATION_FAILURE_EVIDENCE_POST_WINDOW=${NAVIGATION_FAILURE_EVIDENCE_POST_WINDOW:-${CFG_NAVIGATION_FAILURE_EVIDENCE_POST_WINDOW:-12.0}}
NAVIGATION_FAILURE_EVIDENCE_ZERO_VELOCITY_SECONDS=${NAVIGATION_FAILURE_EVIDENCE_ZERO_VELOCITY_SECONDS:-${CFG_NAVIGATION_FAILURE_EVIDENCE_ZERO_VELOCITY_SECONDS:-5.0}}
NAVIGATION_FAILURE_EVIDENCE_NO_PROGRESS_SECONDS=${NAVIGATION_FAILURE_EVIDENCE_NO_PROGRESS_SECONDS:-${CFG_NAVIGATION_FAILURE_EVIDENCE_NO_PROGRESS_SECONDS:-8.0}}

start_navigation_metrics_window() {
  local metrics_command
  local -a metrics_arguments=(
    "_log_dir:=$NAVIGATION_LOG_DIR"
    "_run_directory:=$NAVIGATION_RUN_DIRECTORY"
    "_retention_days:=15"
    "_failure_evidence_enabled:=$NAVIGATION_FAILURE_EVIDENCE_ENABLED"
    "_failure_evidence_sample_period:=$NAVIGATION_FAILURE_EVIDENCE_SAMPLE_PERIOD"
    "_failure_evidence_pre_window:=$NAVIGATION_FAILURE_EVIDENCE_PRE_WINDOW"
    "_failure_evidence_post_window:=$NAVIGATION_FAILURE_EVIDENCE_POST_WINDOW"
    "_failure_evidence_zero_velocity_seconds:=$NAVIGATION_FAILURE_EVIDENCE_ZERO_VELOCITY_SECONDS"
    "_failure_evidence_no_progress_seconds:=$NAVIGATION_FAILURE_EVIDENCE_NO_PROGRESS_SECONDS"
    "_execution_architecture:=$TEB_EXECUTION_ARCHITECTURE"
    "_teb_strong_angular_threshold:=$TEB_ANGULAR_SIGN_SWITCH_THRESHOLD"
    "_task_json:=$TASK_JSON"
    "_task_id:=$TASK_ID"
    "_detector:=$DETECTOR"
    "_detector_backend:=$DETECTOR_BACKEND"
    "_detector_variant:=$WDETECT_VARIANT"
    "_detector_runtime:=$WDETECT_RUNTIME"
    "_detector_score_threshold:=$WDETECT_SCORE_THRESHOLD"
    "_detector_nms_iou:=$WDETECT_NMS_IOU"
    "_detector_min_inference_interval:=$DETECTOR_MIN_INTERVAL"
    "_target_follow_min_score:=$TARGET_FOLLOW_MIN_SCORE"
    "_target_follow_min_box_size:=$TARGET_FOLLOW_MIN_BOX_SIZE"
    "_target_follow_confirm_hits:=$TARGET_FOLLOW_CONFIRM_HITS"
    "_target_follow_confirm_window:=$TARGET_FOLLOW_CONFIRM_WINDOW"
    "_target_done_policy:=$TARGET_DONE_POLICY"
    "_target_room_entry_min_x:=$TARGET_ROOM_ENTRY_MIN_X"
    "_target_room_entry_min_y:=$TARGET_ROOM_ENTRY_MIN_Y"
    "_target_room_entry_max_x:=$TARGET_ROOM_ENTRY_MAX_X"
    "_target_room_entry_max_y:=$TARGET_ROOM_ENTRY_MAX_Y"
    "_target_room_entry_margin:=$TARGET_ROOM_ENTRY_MARGIN"
    "_target_room_entry_min_hold_time:=$TARGET_ROOM_ENTRY_MIN_HOLD_TIME"
    "_target_room_entry_require_crossing:=$TARGET_ROOM_ENTRY_REQUIRE_CROSSING"
    "_target_done_min_score:=$TARGET_DONE_MIN_SCORE"
    "_target_done_min_box_width:=$TARGET_DONE_MIN_BOX_WIDTH"
    "_target_done_min_box_height:=$TARGET_DONE_MIN_BOX_HEIGHT"
    "_target_done_min_fresh_hits:=$TARGET_DONE_MIN_FRESH_HITS"
    "_target_done_min_hold_time:=$TARGET_DONE_MIN_HOLD_TIME"
    "_target_done_require_close_evidence:=$TARGET_DONE_REQUIRE_CLOSE_EVIDENCE"
    "_target_done_require_confirmed_track:=$TARGET_DONE_REQUIRE_CONFIRMED_TRACK"
    "_target_done_require_approach_terminal:=$TARGET_DONE_REQUIRE_APPROACH_TERMINAL"
    "_target_eval_enabled:=$TARGET_EVAL_ENABLED"
    "_target_eval_gazebo_target_model:=$TARGET_EVAL_GAZEBO_TARGET_MODEL"
    "_target_eval_gazebo_robot_model:=$TARGET_EVAL_GAZEBO_ROBOT_MODEL"
    "_target_eval_world_frame:=$TARGET_EVAL_WORLD_FRAME"
    "_target_eval_odom_frame:=$TARGET_EVAL_ODOM_FRAME"
    "_target_eval_camera_frame:=$TARGET_EVAL_CAMERA_FRAME"
    "_target_eval_camera_info_topic:=$TARGET_EVAL_CAMERA_INFO_TOPIC"
    "_target_eval_depth_enabled:=$TARGET_EVAL_DEPTH_ENABLED"
    "_target_eval_depth_topic:=$TARGET_EVAL_DEPTH_TOPIC"
    "_target_eval_depth_max_source_age_s:=$TARGET_EVAL_DEPTH_MAX_SOURCE_AGE_S"
    "_target_eval_depth_sample_grid:=$TARGET_EVAL_DEPTH_SAMPLE_GRID"
    "_target_eval_depth_min_valid_samples:=$TARGET_EVAL_DEPTH_MIN_VALID_SAMPLES"
    "_target_eval_depth_min_matching_samples:=$TARGET_EVAL_DEPTH_MIN_MATCHING_SAMPLES"
    "_target_eval_depth_abs_tolerance_m:=$TARGET_EVAL_DEPTH_ABS_TOLERANCE_M"
    "_target_eval_depth_relative_tolerance:=$TARGET_EVAL_DEPTH_RELATIVE_TOLERANCE"
    "_target_eval_detections_topic:=$TARGET_EVAL_DETECTIONS_TOPIC"
    "_target_eval_target_labels:=$TARGET_EVAL_TARGET_LABELS"
    "_target_eval_target_center_m:=$TARGET_EVAL_TARGET_CENTER_M"
    "_target_eval_target_size_m:=$TARGET_EVAL_TARGET_SIZE_M"
    "_target_eval_min_depth_m:=$TARGET_EVAL_MIN_DEPTH_M"
    "_target_eval_max_depth_m:=$TARGET_EVAL_MAX_DEPTH_M"
    "_target_eval_edge_margin_px:=$TARGET_EVAL_EDGE_MARGIN_PX"
    "_target_eval_max_gazebo_state_age_s:=$TARGET_EVAL_MAX_GAZEBO_STATE_AGE_S"
    "_target_eval_exposure_hold_s:=$TARGET_EVAL_EXPOSURE_HOLD_S"
    "_target_eval_episode_gap_s:=$TARGET_EVAL_EPISODE_GAP_S"
    "_target_eval_min_match_iou:=$TARGET_EVAL_MIN_MATCH_IOU"
    "_target_eval_min_score:=$TARGET_EVAL_MIN_SCORE"
    "_pipeline_config:=$PIPELINE_CONFIG"
    "_pipeline_config_sha256:=$PIPELINE_CONFIG_SHA256"
    "_world:=$WORLD"
    "_benchmark_manifest:=$BENCHMARK_MANIFEST"
    "_benchmark_level:=$BENCHMARK_LEVEL"
    "_benchmark_collision_truth_enabled:=$BENCHMARK_COLLISION_TRUTH_ENABLED"
    "_pro3_spawn_x:=$PRO3_SPAWN_X"
    "_pro3_spawn_y:=$PRO3_SPAWN_Y"
    "_pro3_spawn_z:=$PRO3_SPAWN_Z"
    "_pro3_spawn_yaw:=$PRO3_SPAWN_YAW"
    "_online_slam_enabled:=$ONLINE_SLAM_ENABLED"
    "_global_frontier_enabled:=$GLOBAL_FRONTIER_ENABLED"
    "_startup_forward_enabled:=$STARTUP_FORWARD_ENABLED"
    "_global_goal_source:=$GLOBAL_GOAL_SOURCE"
    "_git_revision:=$GIT_REVISION"
    "_git_dirty:=$GIT_DIRTY"
  )
  printf -v metrics_command '%q ' \
    rosrun lste_topo_access lste_navigation_metrics.py "${metrics_arguments[@]}"
  tmux_new_window 17 "$WS/runtime/navigation" "metrics" \
    "$WAIT_ROSCORE; $metrics_command"
}

start_observability_and_controller() {
  mkdir -p "$WS/runtime/navigation"
  start_navigation_metrics_window

  # Keep controller configuration as data. This avoids a fragile shell command
  # with dozens of inline assignments and makes each forwarded setting explicit.
  local -a sappo_environment=(
    "SAPPO_SPEED=$SAPPO_SPEED"
    "SAPPO_PYTHON=$SAPPO_PYTHON"
    "SAPPO_CONTROLLER_MODE=$SAPPO_CONTROLLER_MODE"
    "SAPPO_GOAL_TOLERANCE=$SAPPO_GOAL_TOLERANCE"
    "SAPPO_INTERMEDIATE_GOAL_TOLERANCE=$SAPPO_INTERMEDIATE_GOAL_TOLERANCE"
    "SAPPO_ANGULAR_SIGN_SWITCH_THRESHOLD=$SAPPO_ANGULAR_SIGN_SWITCH_THRESHOLD"
    "SAPPO_MAX_LINEAR_ACTION_STEP=$SAPPO_MAX_LINEAR_ACTION_STEP"
    "SAPPO_MAX_LINEAR_ACTION_DECEL=$SAPPO_MAX_LINEAR_ACTION_DECEL"
    "SAPPO_MAX_ANGULAR_ACTION_STEP=$SAPPO_MAX_ANGULAR_ACTION_STEP"
    "SAPPO_IN_PLACE_TURN_ANGULAR_THRESHOLD=$SAPPO_IN_PLACE_TURN_ANGULAR_THRESHOLD"
    "SAPPO_BOUNDARY_TURN_LOCK_TIME=$SAPPO_BOUNDARY_TURN_LOCK_TIME"
    "SAPPO_BOUNDARY_TURN_MAX_DURATION=$SAPPO_BOUNDARY_TURN_MAX_DURATION"
    "SAPPO_BOUNDARY_TURN_MAX_ANGLE_DEG=$SAPPO_BOUNDARY_TURN_MAX_ANGLE_DEG"
    "SAPPO_BOUNDARY_TURN_MAX_ATTEMPTS=$SAPPO_BOUNDARY_TURN_MAX_ATTEMPTS"
    "SAPPO_BOUNDARY_REENTRY_COOLDOWN=$SAPPO_BOUNDARY_REENTRY_COOLDOWN"
    "SAPPO_BOUNDARY_PROGRESS_TIMEOUT=$SAPPO_BOUNDARY_PROGRESS_TIMEOUT"
    "SAPPO_BOUNDARY_PROGRESS_MARGIN=$SAPPO_BOUNDARY_PROGRESS_MARGIN"
    "SAPPO_BOUNDARY_WALL_DISTANCE=$SAPPO_BOUNDARY_WALL_DISTANCE"
    "SAPPO_BOUNDARY_WALL_KP=$SAPPO_BOUNDARY_WALL_KP"
    "SAPPO_BOUNDARY_WALL_HEADING_KP=$SAPPO_BOUNDARY_WALL_HEADING_KP"
    "SAPPO_BOUNDARY_WALL_MAX_LINEAR=$SAPPO_BOUNDARY_WALL_MAX_LINEAR"
    "SAPPO_BOUNDARY_WALL_MAX_RANGE=$SAPPO_BOUNDARY_WALL_MAX_RANGE"
    "SAPPO_BOUNDARY_WALL_FILTER_ALPHA=$SAPPO_BOUNDARY_WALL_FILTER_ALPHA"
    "SAPPO_BOUNDARY_WALL_ANGULAR_STEP=$SAPPO_BOUNDARY_WALL_ANGULAR_STEP"
    "SAPPO_BOUNDARY_WALL_ANGULAR_DEADBAND=$SAPPO_BOUNDARY_WALL_ANGULAR_DEADBAND"
    "SAPPO_BOUNDARY_GOAL_CANCEL_ANGLE_DEG=$SAPPO_BOUNDARY_GOAL_CANCEL_ANGLE_DEG"
    "SAPPO_BOUNDARY_GOAL_CANCEL_DISTANCE=$SAPPO_BOUNDARY_GOAL_CANCEL_DISTANCE"
    "SAPPO_WAYPOINT_HOLD_RADIUS=$SAPPO_WAYPOINT_HOLD_RADIUS"
    "SAPPO_WAYPOINT_SWITCH_DISTANCE=$SAPPO_WAYPOINT_SWITCH_DISTANCE"
    "SAPPO_GRID_OBSTACLE_RADIUS=$SAPPO_GRID_OBSTACLE_RADIUS"
    "SAPPO_GRID_EDGE_CLEARANCE=$SAPPO_GRID_EDGE_CLEARANCE"
    "SAPPO_REORIENT_OBSTACLE_CLEARANCE=$SAPPO_REORIENT_OBSTACLE_CLEARANCE"
  )
  env "${sappo_environment[@]}" \
    "$WS/scripts/lifecycle/switch_controller.sh" "$LSTE_CONTROLLER"
}

show_node_summary() {
  local window_summary
  if [[ "${LEGACY_GP_FRONTIER_ENABLED,,}" == "true" || "$LEGACY_GP_FRONTIER_ENABLED" == "1" ]]; then
    window_summary="20 lste windows (legacy GP, TEB, and metrics) + persistent lste-teleop"
  else
    window_summary="18 lste windows (TEB, online SLAM, and metrics) + persistent lste-teleop"
  fi
  echo
  echo "============================================"
  echo "  Node startup complete ($window_summary)"
  echo "  Controller: $LSTE_CONTROLLER"
  echo "  Environment: tmux attach -t lste-env"
  echo "  Nodes:       tmux attach -t lste"
  echo "  Teleop:      teleop"
  echo "============================================"
}
