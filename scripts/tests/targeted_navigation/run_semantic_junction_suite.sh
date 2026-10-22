#!/usr/bin/env bash
set -o pipefail
WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
source /opt/ros/noetic/setup.bash
cd "$WS"
export OPENBLAS_NUM_THREADS=1
export GAZEBO_MODEL_PATH="$WS/worlds/targeted_navigation/models:$HOME/.gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export T_JUNCTION_EXPLORATION_METHOD=gp_branch_tree
export T_JUNCTION_STOP_ON_FAILURE=false
RESULT_ROOT="$WS/runtime/gp_branch_development/semantic_junction_suite_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RESULT_ROOT"
echo "$RESULT_ROOT"
for CASE in ${SEMANTIC_CASES:-semantic_t_junction semantic_l_junction semantic_cross_junction}; do
  export T_JUNCTION_WORLD="$WS/worlds/targeted_navigation/$CASE.world"
  export T_JUNCTION_X=-3.5 T_JUNCTION_Y=0 T_JUNCTION_YAW=0
  HEADINGS=(0 1.57079632679 3.14159265359)
  CENTER=(0 0)
  if [[ "$CASE" == semantic_l_junction ]]; then
    HEADINGS=(1.57079632679 3.14159265359)
  elif [[ "$CASE" == semantic_cross_junction ]]; then
    HEADINGS=(0 1.57079632679 3.14159265359 -1.57079632679)
  fi
  RUNNER_LOG="$RESULT_ROOT/${CASE}_runner.log"
  LSTE_WS="$WS" bash scripts/tests/targeted_navigation/run_t_junction.sh --persistent \
      --max-seconds "${GP_SUITE_SECONDS:-900}" --startup-seconds 90 --no-stop-on-failure >"$RUNNER_LOG" 2>&1
  LAUNCH_STATUS=$?
  RUN_DIR=$(sed -n 's/^run_directory=//p' "$RUNNER_LOG" | tail -1)
  if [[ -z "$RUN_DIR" ]]; then
    printf '%s launch_failed %s\n' "$CASE" "$LAUNCH_STATUS" >>"$RESULT_ROOT/summary.log"
    continue
  fi
  STAMP=$(basename "$RUN_DIR")
  METRICS="$RUN_DIR/${STAMP}_navigation_metrics.log"
  python3 scripts/tests/targeted_navigation/check_gp_tree.py \
      "$METRICS" --headings "${HEADINGS[@]}" --center "${CENTER[@]}" \
      --output "$RESULT_ROOT/${CASE}_tree_result.json" >"$RESULT_ROOT/${CASE}_tree_check.log" 2>&1
  TREE_STATUS=$?
  python3 scripts/tests/targeted_navigation/check_portal_authentication.py \
      "$METRICS" --output "$RESULT_ROOT/${CASE}_authentication_result.json" \
      >"$RESULT_ROOT/${CASE}_authentication_check.log" 2>&1
  AUTH_STATUS=$?
  python3 scripts/tools/plot_portal_authentication.py "$METRICS" \
      --world "$T_JUNCTION_WORLD" --route-zoom --output "$RESULT_ROOT/${CASE}_portal_authentication.png" \
      >"$RESULT_ROOT/${CASE}_visualization.log" 2>&1
  VIS_STATUS=$?
  printf '%s launch=%s tree=%s auth=%s visual=%s run=%s\n' \
      "$CASE" "$LAUNCH_STATUS" "$TREE_STATUS" "$AUTH_STATUS" "$VIS_STATUS" "$RUN_DIR" \
      >>"$RESULT_ROOT/summary.log"
done
