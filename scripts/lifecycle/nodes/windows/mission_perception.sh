#!/usr/bin/env bash
# Task, prompt, detector, state, and visual-display tmux windows (1-9).

start_mission_and_perception_windows() {
  tmux_new_window 1 "$WS" "task" \
    "$WAIT_ROSCORE; rosrun lste_core lste_task_node.py _json_path:=$TASK_JSON _task_id:=$TASK_ID"

  tmux_new_window 2 "$WS/model/MiniCPM/test" "vllm" \
    "$WAIT_ROSCORE; echo \"Waiting for prompt cache decision...\"; \\
     while true; do \\
       decision=\$(rosparam get /lste_prompt_node/needs_vllm 2>/dev/null || true); \\
       case \"\$decision\" in \\
         true) echo \"Prompt cache miss; starting MiniCPM.\"; set +u; conda activate minicpm; set -u; exec bash start.sh ;; \\
         false) echo \"Prompt cache hit; MiniCPM was not started.\"; exit 0 ;; \\
       esac; \\
       sleep 0.5; \\
     done"

  start_goal_manager_window
  start_prompt_and_detector_windows
  start_state_and_visualization_windows
}

start_goal_manager_window() {
  tmux_new_window 3 "$WS" "goal" \
    "$WAIT_ROSCORE; rosrun lste_topo_access lste_goal_manager.py \\
      _follow_locked_done_time:=$FOLLOW_LOCKED_DONE_TIME _debug_goal_log:=$GOAL_DEBUG_LOG \\
      _target_done_policy:=$TARGET_DONE_POLICY \\
      _target_room_entry_min_x:=$TARGET_ROOM_ENTRY_MIN_X \\
      _target_room_entry_min_y:=$TARGET_ROOM_ENTRY_MIN_Y \\
      _target_room_entry_max_x:=$TARGET_ROOM_ENTRY_MAX_X \\
      _target_room_entry_max_y:=$TARGET_ROOM_ENTRY_MAX_Y \\
      _target_room_entry_margin:=$TARGET_ROOM_ENTRY_MARGIN \\
      _target_room_entry_min_hold_time:=$TARGET_ROOM_ENTRY_MIN_HOLD_TIME \\
      _target_room_entry_require_crossing:=$TARGET_ROOM_ENTRY_REQUIRE_CROSSING \\
      _target_done_min_box_width:=$TARGET_DONE_MIN_BOX_WIDTH \\
      _target_done_min_box_height:=$TARGET_DONE_MIN_BOX_HEIGHT \\
      _target_done_min_score:=$TARGET_DONE_MIN_SCORE \\
      _target_done_min_hold_time:=$TARGET_DONE_MIN_HOLD_TIME \\
      _target_done_min_fresh_hits:=$TARGET_DONE_MIN_FRESH_HITS \\
      _target_done_require_close_evidence:=$TARGET_DONE_REQUIRE_CLOSE_EVIDENCE \\
      _target_done_require_confirmed_track:=$TARGET_DONE_REQUIRE_CONFIRMED_TRACK \\
      _target_done_max_detection_age:=$TARGET_DONE_MAX_DETECTION_AGE \\
      _target_done_require_approach_terminal:=$TARGET_DONE_REQUIRE_APPROACH_TERMINAL \\
      _target_follow_min_score:=$TARGET_FOLLOW_MIN_SCORE \\
      _target_follow_min_box_size:=$TARGET_FOLLOW_MIN_BOX_SIZE \\
      _target_follow_candidate_timeout:=$TARGET_FOLLOW_CANDIDATE_TIMEOUT \\
      _target_follow_confirm_hits:=$TARGET_FOLLOW_CONFIRM_HITS \\
      _target_follow_confirm_window:=$TARGET_FOLLOW_CONFIRM_WINDOW \\
      _target_follow_weak_confirm_hits:=$TARGET_FOLLOW_WEAK_CONFIRM_HITS \\
      _target_follow_weak_min_average_score:=$TARGET_FOLLOW_WEAK_MIN_AVERAGE_SCORE \\
      _target_follow_spatial_tolerance:=$TARGET_FOLLOW_SPATIAL_TOLERANCE \\
      _target_observation_hold:=$TARGET_OBSERVATION_HOLD \\
      _target_done_require_locked:=$TARGET_DONE_REQUIRE_LOCKED \\
      _follow_goal_publish_period:=$FOLLOW_GOAL_PUBLISH_PERIOD \\
      _follow_target_step_distance:=$FOLLOW_TARGET_STEP_DISTANCE \\
      _follow_target_use_scan_clip:=$FOLLOW_TARGET_USE_SCAN_CLIP \\
      _target_approach_strategy:=$TARGET_APPROACH_STRATEGY \\
      _target_bbox_alpha:=$TARGET_BBOX_ALPHA _target_heading_alpha:=$TARGET_HEADING_ALPHA \\
      _target_max_heading_step_deg:=$TARGET_MAX_HEADING_STEP_DEG \\
      _target_goal_reached_radius:=$TARGET_GOAL_REACHED_RADIUS \\
      _target_goal_heading_update_threshold_deg:=$TARGET_GOAL_HEADING_UPDATE_THRESHOLD_DEG \\
      _target_cache_max_advances:=$TARGET_CACHE_MAX_ADVANCES \\
      _target_continuous_handoff_enabled:=$TARGET_CONTINUOUS_HANDOFF_ENABLED \\
      _target_continuous_handoff_distance:=$TARGET_CONTINUOUS_HANDOFF_DISTANCE \\
      _target_continuous_handoff_min_new_frames:=$TARGET_CONTINUOUS_HANDOFF_MIN_NEW_FRAMES \\
      _follow_target_lost_timeout:=$FOLLOW_TARGET_LOST_TIMEOUT \\
      _target_reacquire_duration:=$TARGET_REACQUIRE_DURATION \\
      _target_reacquire_distance:=$TARGET_REACQUIRE_DISTANCE \\
      _target_reacquire_max_attempts:=$TARGET_REACQUIRE_MAX_ATTEMPTS \\
      _follow_context_step_distance:=$FOLLOW_CONTEXT_STEP_DISTANCE \\
      _context_goal_reached_radius:=$CONTEXT_GOAL_REACHED_RADIUS \\
      _context_goal_heading_update_threshold_deg:=$CONTEXT_GOAL_HEADING_UPDATE_THRESHOLD_DEG \\
      _context_follow_cooldown:=$CONTEXT_FOLLOW_COOLDOWN \\
      _context_state_hold_time:=$CONTEXT_STATE_HOLD_TIME \\
      _global_frontier_enabled:=$GLOBAL_FRONTIER_ENABLED \\
      _startup_forward_enabled:=$STARTUP_FORWARD_ENABLED \\
      _global_frontier_topic:=$GLOBAL_FRONTIER_TOPIC \\
      _global_frontier_replan_request_topic:=$GLOBAL_FRONTIER_REPLAN_REQUEST_TOPIC \\
      _teb_goal_terminal_topic:=$TEB_GOAL_TERMINAL_TOPIC \\
      _teb_goal_failure_topic:=$TEB_GOAL_FAILURE_TOPIC \\
      _goal_intent_topic:=$GOAL_INTENT_TOPIC _goal_command_topic:=$GOAL_COMMAND_TOPIC \\
      _global_frontier_max_age:=$GLOBAL_FRONTIER_MAX_AGE \\
      _global_frontier_period:=$GLOBAL_FRONTIER_PERIOD \\
      _global_frontier_update_radius:=$GLOBAL_FRONTIER_UPDATE_RADIUS \\
      _global_frontier_early_handoff_radius:=$GLOBAL_FRONTIER_EARLY_HANDOFF_RADIUS \\
      _global_frontier_min_hold_time:=$GLOBAL_FRONTIER_MIN_HOLD_TIME \\
      _global_frontier_jump_distance:=$GLOBAL_FRONTIER_JUMP_DISTANCE \\
      _global_frontier_jump_release_radius:=$GLOBAL_FRONTIER_JUMP_RELEASE_RADIUS \\
      _global_frontier_preempt_context:=$GLOBAL_FRONTIER_PREEMPT_CONTEXT \\
      _global_frontier_terminal_hold_timeout:=$GLOBAL_FRONTIER_TERMINAL_HOLD_TIMEOUT \\
      _target_route_validation:=$TARGET_ROUTE_VALIDATION \\
      _target_route_validation_timeout:=$TARGET_ROUTE_VALIDATION_TIMEOUT \\
      _target_route_validation_tolerance:=$TARGET_ROUTE_VALIDATION_TOLERANCE \\
      _target_route_validation_period:=$TARGET_ROUTE_VALIDATION_PERIOD \\
      _target_route_continuity_enabled:=$TARGET_ROUTE_CONTINUITY_ENABLED \\
      _target_route_continuity_entry_lookahead:=$TARGET_ROUTE_CONTINUITY_ENTRY_LOOKAHEAD \\
      _target_route_continuity_smooth_heading_deg:=$TARGET_ROUTE_CONTINUITY_SMOOTH_HEADING_DEG \\
      _target_route_continuity_takeover_heading_deg:=$TARGET_ROUTE_CONTINUITY_TAKEOVER_HEADING_DEG \\
      _target_route_continuity_defer_sharp_takeover:=$TARGET_ROUTE_CONTINUITY_DEFER_SHARP_TAKEOVER \\
      _target_route_continuity_distance_tradeoff_deg:=$TARGET_ROUTE_CONTINUITY_DISTANCE_TRADEOFF_DEG \\
      _controller_mode:=$LSTE_CONTROLLER _global_goal_source:=$GLOBAL_GOAL_SOURCE \\
      _fixed_goal_x:=$FIXED_GLOBAL_GOAL_X _fixed_goal_y:=$FIXED_GLOBAL_GOAL_Y \\
      _fixed_goal_yaw:=$FIXED_GLOBAL_GOAL_YAW \\
      _fixed_goal_publish_period:=$FIXED_GLOBAL_GOAL_PUBLISH_PERIOD \\
      _fixed_goal_allow_click_override:=$FIXED_GLOBAL_GOAL_ALLOW_CLICK_OVERRIDE"
}

start_prompt_and_detector_windows() {
  tmux_new_window 4 "$WS" "prompt" \
    "$WAIT_ROSCORE; set +u; conda activate minicpm; set -u; \\
     rosparam set /lste_prompt_node/vllm_stop_command \"tmux kill-window -t =$SESSION:vllm\"; \\
     rosrun lste_core lste_prompt_node.py \\
       _vllm_base_url:=$VLLM_URL _vllm_model_name:=$VLLM_MODEL \\
       _cache_enabled:=$PROMPT_CACHE_ENABLED _cache_dir:=$PROMPT_CACHE_DIR \\
       _vllm_start_timeout:=$VLLM_START_TIMEOUT"

  # Run the source through the activated detector interpreter. rosrun's relay
  # has the base-conda shebang and would otherwise load incompatible CUDA libs.
  tmux_new_window 5 "$WS" "detector" \
    "$WAIT_ROSCORE; set +u; conda activate dino; set -u; export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libffi.so.7; \\
     export LD_LIBRARY_PATH=\"\$CONDA_PREFIX/lib/python3.9/site-packages/tensorrt_libs:\$CONDA_PREFIX/lib/python3.9/site-packages/nvidia/cudnn/lib:\$CONDA_PREFIX/lib/python3.9/site-packages/nvidia/cublas/lib:\$CONDA_PREFIX/lib/python3.9/site-packages/nvidia/cufft/lib:\$CONDA_PREFIX/lib/python3.9/site-packages/nvidia/curand/lib:\$CONDA_PREFIX/lib/python3.9/site-packages/nvidia/cusolver/lib:\$CONDA_PREFIX/lib/python3.9/site-packages/nvidia/cusparse/lib:\$CONDA_PREFIX/lib/python3.9/site-packages/nvidia/cuda_runtime/lib:\$CONDA_PREFIX/lib/python3.9/site-packages/nvidia/cuda_nvrtc/lib:\$CONDA_PREFIX/lib/python3.9/site-packages/nvidia/nvjitlink/lib:\$CONDA_PREFIX/lib/python3.9/site-packages/torch/lib:\${LD_LIBRARY_PATH:-}\"; \\
     python \"$WS/src/lste_core/scripts/$DETECTOR_NODE\" \\
       _min_inference_interval:=$DETECTOR_MIN_INTERVAL \\
       _max_source_image_age:=$DETECTION_MAX_SOURCE_IMAGE_AGE \\
       _interval_pass:=$DETECTOR_INTERVAL_PASS _interval_suspicious:=$DETECTOR_INTERVAL_SUSPICIOUS \\
       _interval_locked:=$DETECTOR_INTERVAL_LOCKED _interval_exhausted:=$DETECTOR_INTERVAL_EXHAUSTED \\
       $DETECTOR_MODEL_ARGS _spin_hz:=$WDETECT_SPIN_HZ"
}

start_state_and_visualization_windows() {
  tmux_new_window 6 "$WS" "score" \
    "$WAIT_ROSCORE; rosrun lste_core lste_score_node.py \\
      _w_target:=0.2 _w_env:=0.3 _w_ctx:=0.5 _lambda_neg:=0.7 \\
      _pos_midpoint:=0.25 _pos_steepness:=6 _neg_midpoint:=0.15 _neg_steepness:=12"
  tmux_new_window 7 "$WS" "state" \
    "$WAIT_ROSCORE; rosrun lste_core lste_state_node.py \\
      _suspicious_window:=$SUSPICIOUS_WINDOW _subtype_switch_min_hits:=$STATE_SUBTYPE_SWITCH_MIN_HITS \\
      _lock_total_enter:=$TARGET_LOCK_TOTAL_MIN _lock_target_enter:=$TARGET_LOCK_SCORE_MIN \\
      _lock_enter_frames:=$TARGET_LOCK_WINDOW _lock_enter_min_hits:=$TARGET_LOCK_MIN_HITS \\
      _lock_total_exit:=$TARGET_LOCK_EXIT_TOTAL_MIN _lock_target_exit:=$TARGET_LOCK_EXIT_SCORE_MIN \\
      _lock_exit_unstable_frames:=$TARGET_LOCK_EXIT_UNSTABLE_FRAMES"
  tmux_new_window 8 "$WS" "vis" \
    "$WAIT_ROSCORE; roslaunch lste_core lste_det_vis.launch \\
      tracking_enabled:=$DETECTION_VISUAL_TRACKING_ENABLED \\
      display_sync_mode:=$DETECTION_DISPLAY_SYNC_MODE \\
      display_history_size:=$DETECTION_DISPLAY_HISTORY_SIZE \\
      tracking_mode:=$DETECTION_TRACKING_MODE detector_backend:=$DETECTOR_BACKEND"
  tmux_new_window 9 "$WS" "oc_srfc" \
    "$WAIT_ROSCORE; roslaunch lste_oc_srfc oc_srfc_proj.launch"
}
