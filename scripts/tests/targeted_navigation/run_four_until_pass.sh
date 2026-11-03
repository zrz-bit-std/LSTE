#!/usr/bin/env bash
# Unattended four-scenario run with up to three attempts per scenario.
#
# Rules (agreed with the user):
#   * drive ceiling is per scenario (see DRIVE_BY_CASE below); the driver stops
#     itself on goal completion, so the ceiling is only a safety net,
#   * any uninterrupted 180 s without progress terminates the attempt,
#   * a scenario passes only when every audit status is 0,
#   * each scenario gets at most 3 attempts, then the next scenario starts,
#   * failure snapshots are collected for every failed attempt,
#   * the loop never stops early.
set -uo pipefail

WS="${LSTE_WS:-/home/zrz/work/lste_ws}"
CASES=(semantic_l_junction semantic_t_junction semantic_cross_junction semantic_t_junction_rotated)
# Run a subset (e.g. LSTE_CASES=semantic_l_junction) without editing the script.
if [[ -n "${LSTE_CASES:-}" ]]; then
  read -ra CASES <<<"${LSTE_CASES}"
fi
MAX_ATTEMPTS="${LSTE_MAX_ATTEMPTS:-3}"
# Measured sim-time cost of a full mission (sim/wall ratio 0.65-0.74):
#   L     2 doorways, completed at 1168 s sim  -> 2400 s wall is ample
#   T     3 doorways, still incomplete at 2042 s sim -> 4500 s wall (~3100 sim)
#   Cross 4 doorways, still on place 2-3 at 1866 s sim -> 5400 s wall (~3700 sim)
#   Rot   3 doorways, still incomplete at 2062 s sim -> 4500 s wall
# A single 3000 s ceiling could never let T/Cross/Rot finish; that, not a
# planning defect, was why every completed run failed auth_all + dfs.
declare -A DRIVE_BY_CASE=(
  [semantic_l_junction]=2400
  [semantic_t_junction]=4500
  [semantic_cross_junction]=5400
  [semantic_t_junction_rotated]=4500
)
DRIVE_OVERRIDE="${GP_SUITE_SECONDS:-}"
case_drive_seconds() {
  local case="$1"
  if [[ -n "$DRIVE_OVERRIDE" ]]; then printf '%s' "$DRIVE_OVERRIDE"; return; fi
  printf '%s' "${DRIVE_BY_CASE[$case]:-3000}"
}
STALL_SECONDS="${LSTE_STALL_SECONDS:-180}"
# 1 = stop a scenario at its first PASS (the default four-scenario loop).
# 0 = run every attempt even after a PASS and require all of them (stability run).
STOP_ON_PASS="${LSTE_STOP_ON_PASS:-1}"
LOG_ROOT="$WS/runtime/diagnostics/20261031_four_scenarios"
ATTEMPTS_TSV="$LOG_ROOT/attempts.tsv"
mkdir -p "$LOG_ROOT"
[[ -f "$ATTEMPTS_TSV" ]] || printf 'case\tattempt\tresult\tfailing\troot\tstart\tend\n' > "$ATTEMPTS_TSV"

log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }

cleanup_env() {
  # Terminate anything the previous attempt left behind, then free the master.
  local pids
  pids=$(pgrep -f "roslaunch .*t_junction_nav.launch" || true)
  [[ -z "$pids" ]] || kill -INT $pids 2>/dev/null
  pids=$(pgrep -f "gzserver" || true)
  [[ -z "$pids" ]] || kill -TERM $pids 2>/dev/null
  pids=$(pgrep -f "rosbag record" || true)
  [[ -z "$pids" ]] || kill -INT $pids 2>/dev/null
  sleep 5
  pids=$(pgrep -f "run_place_dfs_suite.sh" || true)
  [[ -z "$pids" ]] || kill -TERM $pids 2>/dev/null
  # A watchdog that outlives its attempt can match (and kill) the next
  # roslaunch; it is keyed only by its suite root, so sweep all of them.
  pids=$(pgrep -f "l_stall_watchdog.py" || true)
  [[ -z "$pids" ]] || kill -TERM $pids 2>/dev/null
  local holder
  holder=$(ss -ltnp 2>/dev/null | grep 11673 | grep -o "pid=[0-9]*" | head -1 | cut -d= -f2)
  [[ -z "$holder" ]] || kill -TERM "$holder" 2>/dev/null
  sleep 3
}

run_attempt() {
  local case="$1" attempt="$2"
  local stamp log root suite_pid watchdog_pid
  stamp=$(date +%Y%m%d_%H%M%S)
  log="/tmp/four_${case}_a${attempt}_${stamp}.log"
  cleanup_env
  log "launch $case attempt $attempt"
  local case_seconds
  case_seconds="$(case_drive_seconds "$case")"
  log "$case drive ceiling ${case_seconds}s"
  setsid nohup env SEMANTIC_CASES="$case" GP_SUITE_SECONDS="$case_seconds" \
    bash "$WS/scripts/tests/targeted_navigation/run_place_dfs_suite.sh" \
    >"$log" 2>&1 </dev/null &
  suite_pid=$!
  local waited=0
  while (( waited < 60 )); do
    root=$(head -1 "$log" 2>/dev/null)
    [[ -n "$root" && -d "$root" ]] && break
    sleep 2; waited=$((waited + 2))
  done
  if [[ -z "${root:-}" || ! -d "${root:-}" ]]; then
    log "suite root not found for $case attempt $attempt"
    return 1
  fi
  log "root=$root"
  setsid nohup python3 "$WS/scripts/tests/targeted_navigation/l_stall_watchdog.py" \
    --suite-root "$root" --wrapper-pid 0 \
    --stall-seconds "$STALL_SECONDS" \
    --since-wall "$(date +%s)" \
    --max-wall-seconds "$((case_seconds + 900))" \
    >"/tmp/four_watchdog_${case}_a${attempt}_${stamp}.log" 2>&1 </dev/null &
  watchdog_pid=$!
  wait "$suite_pid" 2>/dev/null
  local suite_status=$?
  sleep 3
  kill -TERM "$watchdog_pid" 2>/dev/null
  pkill -TERM -f "l_stall_watchdog.py --suite-root $root" 2>/dev/null
  bash "$WS/scripts/tests/targeted_navigation/collect_failure_snapshots.sh" \
    "$root" "$LOG_ROOT/snapshots" >/dev/null 2>&1
  # The suite writes "launch_failed <status>" after a watchdog kill, but that
  # line means no audit ran.  Treat it like a missing summary.
  if [[ ! -f "$root/summary.log" ]] || ! grep -q "launch=" "$root/summary.log"; then
    # The stall watchdog killed this attempt before the suite reached its
    # audit.  The bag and metrics still exist, so score them: a killed run is
    # data, not a missing result (L attempt 1 of 20261102_214754 was a near
    # pass that got recorded as launch_failed).
    log "$case attempt $attempt: killed before the audit; scoring the recording"
    rm -f "$root/summary.log"
    env SEMANTIC_CASES="$case" LSTE_AUDIT_ONLY_ROOT="$root" \
      bash "$WS/scripts/tests/targeted_navigation/run_place_dfs_suite.sh" \
      >>"$root/audit_only.log" 2>&1
  fi
  local result failing
  result=$(python3 "$WS/scripts/tests/targeted_navigation/scenario_audit_result.py" \
    "$root/summary.log" "$case" 2>/dev/null)
  failing="${result#FAIL }"
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$case" "$attempt" "${result%% *}" "$failing" "$root" \
    "$(date +%H:%M:%S)" "$(date +%H:%M:%S)" >> "$ATTEMPTS_TSV"
  log "$case attempt $attempt -> $result (driver status $suite_status)"
  [[ "$result" == "PASS" ]]
}

overall=0
for case in "${CASES[@]}"; do
  passed=0
  passes=0
  for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
    if run_attempt "$case" "$attempt"; then
      log "$case PASSED on attempt $attempt"
      passed=1
      passes=$((passes + 1))
      if [[ "$STOP_ON_PASS" == "1" ]]; then
        break
      fi
    else
      log "$case attempt $attempt failed"
    fi
  done
  if [[ "$STOP_ON_PASS" == "1" ]]; then
    if (( ! passed )); then
      log "$case EXHAUSTED $MAX_ATTEMPTS attempts"
      overall=1
    fi
  else
    log "$case completed $passes/$MAX_ATTEMPTS attempts"
    if (( passes < MAX_ATTEMPTS )); then
      overall=1
    fi
  fi
done

log "all scenarios processed"
exit "$overall"
