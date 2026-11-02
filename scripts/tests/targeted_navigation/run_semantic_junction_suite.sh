#!/usr/bin/env bash
set -o pipefail
WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
source /opt/ros/noetic/setup.bash
# Gazebo camera sensors need a rendering context even when the GUI is hidden.
if [[ -z "${DISPLAY:-}" ]] && command -v xdpyinfo >/dev/null 2>&1 \
    && xdpyinfo -display :0 >/dev/null 2>&1; then
  export DISPLAY=:0
fi
cd "$WS"
export OPENBLAS_NUM_THREADS=1
export GAZEBO_MODEL_PATH="$WS/worlds/targeted_navigation/models:$HOME/.gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export T_JUNCTION_EXPLORATION_METHOD=place_portal_workitem
export T_JUNCTION_STOP_ON_FAILURE=false
export T_JUNCTION_SEMANTIC_PERCEPTION=true
export T_JUNCTION_TARGET_DONE_POLICY=graph_exploration
RESULT_ROOT="$WS/runtime/gp_branch_development/semantic_junction_suite_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RESULT_ROOT"
echo "$RESULT_ROOT"
SUITE_STATUS=0
for CASE in ${SEMANTIC_CASES:-semantic_t_junction semantic_l_junction semantic_cross_junction semantic_t_junction_rotated}; do
  export T_JUNCTION_WORLD="$WS/worlds/targeted_navigation/$CASE.world"
  export T_JUNCTION_X=-3.5 T_JUNCTION_Y=0 T_JUNCTION_YAW=0
  HEADINGS=(0 1.57079632679 3.14159265359)
  CENTER=(0 0)
  HEADING_TOLERANCE=0.45
  EXPECTED_ROOMS=${#HEADINGS[@]}
  if [[ "$CASE" == semantic_t_junction_rotated ]]; then
    CENTER=(0 0)
    export T_JUNCTION_WORLD="$WS/worlds/targeted_navigation/semantic_t_junction_focused_rotated.world"
    export T_JUNCTION_X=-2.474873734 T_JUNCTION_Y=-2.474873734 T_JUNCTION_YAW=0.78539816339
    HEADINGS=(0.78539816339 2.35619449019 3.92699081699)
    EXPECTED_ROOMS=3
  elif [[ "$CASE" == t_junction_rotated ]]; then
    CENTER=(1.3 1.6)
    export T_JUNCTION_WORLD="$WS/worlds/targeted_navigation/t_junction_rotated.world"
    export T_JUNCTION_X=-2.474873734 T_JUNCTION_Y=-2.474873734 T_JUNCTION_YAW=0.78539816339
    HEADINGS=(0.78539816339 2.35619449019 3.92699081699)
  elif [[ "$CASE" == semantic_l_junction ]]; then
    HEADINGS=(1.57079632679 3.14159265359)
  elif [[ "$CASE" == semantic_cross_junction ]]; then
    HEADINGS=(0 1.57079632679 3.14159265359 -1.57079632679)
    HEADING_TOLERANCE=0.70
  fi
  # Each physical outgoing arm must complete its own room lifecycle.
  EXPECTED_ROOMS=${#HEADINGS[@]}
  export T_JUNCTION_GOAL_HEADINGS="${HEADINGS[*]}"
  export T_JUNCTION_GOAL_CENTER="${CENTER[*]}"
  export T_JUNCTION_GOAL_HEADING_TOLERANCE="$HEADING_TOLERANCE"
  RUNNER_LOG="$RESULT_ROOT/${CASE}_runner.log"
  LSTE_WS="$WS" bash scripts/tests/targeted_navigation/run_t_junction.sh --persistent \
      --max-seconds "${GP_SUITE_SECONDS:-900}" --startup-seconds 90 --no-stop-on-failure >"$RUNNER_LOG" 2>&1
  LAUNCH_STATUS=$?
  RUN_DIR=$(sed -n 's/^run_directory=//p' "$RUNNER_LOG" | tail -1)
  if [[ -z "$RUN_DIR" ]]; then
    printf '%s launch_failed %s\n' "$CASE" "$LAUNCH_STATUS" >>"$RESULT_ROOT/summary.log"
    SUITE_STATUS=1
    continue
  fi
  STAMP=$(basename "$RUN_DIR")
  METRICS="$RUN_DIR/${STAMP}_navigation_metrics.log"
  python3 scripts/tests/targeted_navigation/check_gp_tree.py \
      "$METRICS" --headings "${HEADINGS[@]}" --center "${CENTER[@]}" \
      --heading-tolerance "$HEADING_TOLERANCE" --output "$RESULT_ROOT/${CASE}_tree_result.json" >"$RESULT_ROOT/${CASE}_tree_check.log" 2>&1
  TREE_STATUS=$?
  python3 scripts/tests/targeted_navigation/check_portal_authentication.py \
      "$METRICS" --junction-center "${CENTER[@]}" --junction-radius 2.0 \
      --output "$RESULT_ROOT/${CASE}_authentication_result.json" \
      >"$RESULT_ROOT/${CASE}_authentication_check.log" 2>&1
  AUTH_STATUS=$?
  python3 scripts/tests/targeted_navigation/audit_semantic_room_lifecycle.py \
      "$METRICS" --center "${CENTER[@]}" --expected-rooms "$EXPECTED_ROOMS" \
      --output "$RESULT_ROOT/${CASE}_lifecycle_audit.json" \
      >"$RESULT_ROOT/${CASE}_lifecycle_check.log" 2>&1
  LIFECYCLE_STATUS=$?
  python3 scripts/tests/targeted_navigation/audit_place_tsp.py \
      "$METRICS" --output "$RESULT_ROOT/${CASE}_tsp_audit.json" \
      >"$RESULT_ROOT/${CASE}_tsp_check.log" 2>&1
  TSP_STATUS=$?
  PLOT_ARGS=(python3 scripts/tools/plot_portal_authentication.py "$METRICS"
      --world "$T_JUNCTION_WORLD" --route-zoom
      --junction-center "${CENTER[@]}" --junction-radius 2.0
      --output "$RESULT_ROOT/${CASE}_portal_authentication.png")
  "${PLOT_ARGS[@]}" >"$RESULT_ROOT/${CASE}_visualization.log" 2>&1
  VIS_STATUS=$?
  printf '%s launch=%s tree=%s auth=%s lifecycle=%s tsp=%s visual=%s run=%s\n' \
      "$CASE" "$LAUNCH_STATUS" "$TREE_STATUS" "$AUTH_STATUS" "$LIFECYCLE_STATUS" "$TSP_STATUS" "$VIS_STATUS" "$RUN_DIR" \
      >>"$RESULT_ROOT/summary.log"
  if ((LAUNCH_STATUS || TREE_STATUS || AUTH_STATUS || LIFECYCLE_STATUS || TSP_STATUS || VIS_STATUS)); then
    SUITE_STATUS=1
  fi
done
exit "$SUITE_STATUS"
