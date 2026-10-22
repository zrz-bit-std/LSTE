#!/usr/bin/env bash
set -o pipefail
WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
source /opt/ros/noetic/setup.bash
cd "$WS"
export OPENBLAS_NUM_THREADS=1
export T_JUNCTION_EXPLORATION_METHOD=gp_branch_tree
export T_JUNCTION_STOP_ON_FAILURE=false
RESULT_ROOT="$WS/runtime/gp_branch_development/junction_suite_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RESULT_ROOT"
printf '%s\n' "$RESULT_ROOT"
if [[ -n "${GP_WAIT_PID:-}" ]]; then
  while kill -0 "$GP_WAIT_PID" 2>/dev/null; do sleep 2; done
fi
for CASE in t_junction_small l_junction_small cross_junction_small t_junction_rotated; do
  export T_JUNCTION_WORLD="$WS/worlds/targeted_navigation/$CASE.world"
  export T_JUNCTION_X=-2.5 T_JUNCTION_Y=0 T_JUNCTION_YAW=0
  HEADINGS=(0 1.57079632679 3.14159265359)
  CENTER=(2 0)
  if [[ "$CASE" == l_junction_small ]]; then
    HEADINGS=(1.57079632679 3.14159265359)
  elif [[ "$CASE" == cross_junction_small ]]; then
    HEADINGS=(0 1.57079632679 3.14159265359 -1.57079632679)
  elif [[ "$CASE" == t_junction_rotated ]]; then
    export T_JUNCTION_X=-1.76776695 T_JUNCTION_Y=-1.76776695 T_JUNCTION_YAW=0.78539816339
    HEADINGS=(0.78539816339 2.35619449019 -2.35619449019)
    CENTER=(1.41421356 1.41421356)
  fi
  RUNNER_LOG="$RESULT_ROOT/${CASE}_runner.log"
  cp scripts/tests/targeted_navigation/run_t_junction.sh "$RESULT_ROOT/${CASE}_runner.sh"
  LSTE_WS="$WS" bash "$RESULT_ROOT/${CASE}_runner.sh" --persistent \
      --max-seconds "${GP_SUITE_SECONDS:-360}" --startup-seconds 90 --no-stop-on-failure >"$RUNNER_LOG" 2>&1
  LAUNCH_STATUS=$?
  RUN_DIR=$(sed -n 's/^run_directory=//p' "$RUNNER_LOG" | tail -1)
  if [[ -z "$RUN_DIR" ]]; then
    printf '%s launch_failed %s\n' "$CASE" "$LAUNCH_STATUS" >>"$RESULT_ROOT/summary.log"
    continue
  fi
  STAMP=$(basename "$RUN_DIR")
  METRICS="$RUN_DIR/${STAMP}_navigation_metrics.log"
  python3 scripts/tests/targeted_navigation/check_gp_tree.py \
      "$METRICS" --headings "${HEADINGS[@]}" \
      --center "${CENTER[@]}" --output "$RESULT_ROOT/${CASE}_tree_result.json" \
      >"$RESULT_ROOT/${CASE}_tree_check.log" 2>&1
  TREE_STATUS=$?
  python3 scripts/tests/targeted_navigation/check_portal_authentication.py \
      "$METRICS" --output "$RESULT_ROOT/${CASE}_authentication_result.json" \
      >"$RESULT_ROOT/${CASE}_authentication_check.log" 2>&1
  AUTH_STATUS=$?
  python3 scripts/tools/plot_portal_authentication.py "$METRICS" \
      --world "$T_JUNCTION_WORLD" --output "$RESULT_ROOT/${CASE}_portal_authentication.png" \
      >"$RESULT_ROOT/${CASE}_visualization.log" 2>&1
  VIS_STATUS=$?
  printf '%s launch=%s tree=%s auth=%s visual=%s run=%s\n' \
      "$CASE" "$LAUNCH_STATUS" "$TREE_STATUS" "$AUTH_STATUS" "$VIS_STATUS" "$RUN_DIR" \
      >>"$RESULT_ROOT/summary.log"
done
python3 scripts/tests/targeted_navigation/portal_retry_single_door.py run \
    --log-root "$RESULT_ROOT/portal_retry_single_door/logs" \
    >"$RESULT_ROOT/portal_retry_single_door_result.json"

