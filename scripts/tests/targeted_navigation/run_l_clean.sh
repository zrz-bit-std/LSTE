#!/usr/bin/env bash
# Clean-environment launcher for the L scenario.
#
# Drive cap: 3000 s wall (~45 min of simulated time at the measured 0.74x
# ratio).  The driver stops on its own at semantic_junction_goal_complete, so
# this is only a ceiling: a healthy L finished in 27-44 minutes historically.
# Any uninterrupted loss of progress longer than LSTE_STALL_SECONDS (default
# 180 s) is treated as failure by l_stall_watchdog.py, which terminates the
# suite immediately instead of waiting for the ceiling.
#
# Preflight refuses to start when the host is not clean, because a stale
# rosmaster or a leftover Gazebo silently contaminates the run (a previous
# attempt attached to a five-day-old master on the suite port).
set -o pipefail

WS="${LSTE_WS:-/home/zrz/work/lste_ws}"
CASE="${SEMANTIC_CASES:-semantic_l_junction}"
SECONDS_TO_RUN="${GP_SUITE_SECONDS:-3000}"
STALL_SECONDS="${LSTE_STALL_SECONDS:-180}"
MASTER_PORT="${PLACE_DFS_MASTER_PORT:-11673}"

fail() { echo "PREFLIGHT FAIL: $*" >&2; exit 2; }

echo "== preflight =="

leftovers=$(ps -eo pid,cmd | grep -E "gzserver|roslaunch .*t_junction_nav|rosbag record" | grep -v grep | wc -l)
[[ "$leftovers" -eq 0 ]] || fail "$leftovers leftover ros/gazebo processes"

if ss -ltn 2>/dev/null | grep -q ":$MASTER_PORT "; then
  fail "port $MASTER_PORT already bound"
fi

newest_src=$(find "$WS/src/lste_topo_access/src" "$WS/src/lste_topo_access/include" \
  -name '*.cpp' -o -name '*.h' | xargs stat -c '%Y' | sort -n | tail -1)
lib="$WS/devel/lib/liblste_persistent_navigation_plugins.so"
[[ -f "$lib" ]] || fail "plugin library not built"
[[ "$(stat -c '%Y' "$lib")" -ge "$newest_src" ]] || fail "plugin library older than sources"

cd "$WS/src/lste_topo_access/test" || fail "test directory missing"
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
source "$WS/devel/setup.bash" >/dev/null 2>&1
python3 test_zero_command_diagnosis.py >/tmp/preflight_zero.log 2>&1 \
  || fail "zero-command regression failed (see /tmp/preflight_zero.log)"
python3 test_global_plan_retention_contract.py >/tmp/preflight_retention.log 2>&1 \
  || fail "plan-retention contract failed (see /tmp/preflight_retention.log)"
echo "regressions OK"

echo "== launch =="
cd "$WS" || fail "workspace missing"
STAMP=$(date +%Y%m%d_%H%M%S)
LOG="/tmp/l_suite_${STAMP}.log"
SUITE_STARTED=$(date +%s)
setsid nohup env SEMANTIC_CASES="$CASE" GP_SUITE_SECONDS="$SECONDS_TO_RUN" \
  bash ./scripts/tests/targeted_navigation/run_place_dfs_suite.sh \
  >"$LOG" 2>&1 </dev/null &
SUITE_PID=$!
echo "suite_log=$LOG"
sleep 25
ROOT=$(head -1 "$LOG")
echo "result_root=$ROOT"
WATCHDOG_LOG="/tmp/l_watchdog_${STAMP}.log"
setsid nohup python3 scripts/tests/targeted_navigation/l_stall_watchdog.py \
  --suite-root "$ROOT" --wrapper-pid "$SUITE_PID" \
  --stall-seconds "$STALL_SECONDS" --since-wall "$((SUITE_STARTED - 2))" \
  --max-wall-seconds "$((SECONDS_TO_RUN + 600))" \
  >"$WATCHDOG_LOG" 2>&1 </dev/null &
echo "watchdog_log=$WATCHDOG_LOG stall_seconds=$STALL_SECONDS"
echo
echo "watch the stall gate with:"
echo "  python3 $WS/scripts/tests/targeted_navigation/l_stall_gate.py $ROOT"
