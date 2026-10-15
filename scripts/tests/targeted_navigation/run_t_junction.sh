#!/usr/bin/env bash
set -euo pipefail

# Bounded physical diagnostic for the compact T-junction world. This is a
# development loop only; the office benchmark runner owns formal comparisons.
WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"

MAX_SECONDS="${T_JUNCTION_MAX_SECONDS:-60}"
STARTUP_SECONDS="${T_JUNCTION_STARTUP_SECONDS:-45}"
STOP_ON_FAILURE="${T_JUNCTION_STOP_ON_FAILURE:-true}"
GUI="${T_JUNCTION_GUI:-false}"
ALIGNMENT="${T_JUNCTION_PRE_ROUTE_ALIGNMENT:-true}"
PERSISTENT_EXECUTION="${T_JUNCTION_PERSISTENT_EXECUTION:-false}"
PLANNER_FREQUENCY="${T_JUNCTION_PLANNER_FREQUENCY:-0.0}"
CONVERTER="${T_JUNCTION_TEB_CONVERTER:-costmap_converter::CostmapToPolygonsDBSMCCH}"
INCLUDE_COSTMAP_OBSTACLES="${T_JUNCTION_TEB_INCLUDE_COSTMAP_OBSTACLES:-true}"
TARGET_PROBE=false
TARGET_PROBE_TIMEOUT="${T_JUNCTION_TARGET_PROBE_TIMEOUT:-30}"
TARGET_PROBE_X="${T_JUNCTION_TARGET_PROBE_X:-0.0}"
TARGET_PROBE_Y="${T_JUNCTION_TARGET_PROBE_Y:-1.25}"
TARGET_PROBE_TRANSACTION_ID="${T_JUNCTION_TARGET_PROBE_TRANSACTION_ID:-900}"
TARGET_PROBE_EPOCH="${T_JUNCTION_TARGET_PROBE_EPOCH:-1}"
TARGET_PROBE_TRACK_ID="${T_JUNCTION_TARGET_PROBE_TRACK_ID:-probe:unreachable-wall}"
# The named persistent-target runner is an ownership-chain test. Require the
# replacement frontier by default; set T_JUNCTION_TARGET_PROBE_REQUIRE_FRONTIER=false
# only when isolating target release without exercising graph takeover.
TARGET_PROBE_REQUIRE_FRONTIER="${T_JUNCTION_TARGET_PROBE_REQUIRE_FRONTIER:-true}"

usage() {
  cat <<'EOF'
Usage: run_t_junction.sh [options]

Options:
  --max-seconds N       Bounded navigation window (default: 60)
  --startup-seconds N   Readiness wait (default: 45)
  --no-stop-on-failure  Keep the window until --max-seconds
  --gui                 Start Gazebo GUI
  --persistent, --persistent-execution
                        Use the persistent Navfn/TEB stream (default: endpoint action)
  --probe-unreachable-target
                        Inject a wall target and verify frontier takeover
  --target-transaction-id N
                        Transaction identity for the target probe (default: 900)
  --target-goal X Y     Target probe endpoint (default: 0.0 1.25)
  --require-frontier-takeover
                        Also require a fresh frontier route after release
  --no-pre-route-turn   Disable TEB route-entry alignment for an A/B run
  --raw-costmap         Disable the DBSMCCH obstacle-converter A/B path
  --polygon-only        Use converter polygons without raw costmap cells
  -h, --help            Show this help

Every run writes a failure report (`<timestamp>_failure_report.md/.json`) beside
the metrics and failure artifacts.  Use `scripts/bin/navdiag <run-directory>`
to read it without starting ROS.
EOF
}

while (($#)); do
  case "$1" in
    --max-seconds)
      MAX_SECONDS="$2"
      shift 2
      ;;
    --startup-seconds)
      STARTUP_SECONDS="$2"
      shift 2
      ;;
    --no-stop-on-failure)
      STOP_ON_FAILURE=false
      shift
      ;;
    --gui)
      GUI=true
      shift
      ;;
    --persistent|--persistent-execution)
      PERSISTENT_EXECUTION=true
      shift
      ;;
    --probe-unreachable-target)
      TARGET_PROBE=true
      PERSISTENT_EXECUTION=true
      shift
      ;;
    --target-transaction-id)
      TARGET_PROBE_TRANSACTION_ID="$2"
      shift 2
      ;;
    --target-goal)
      TARGET_PROBE_X="$2"
      TARGET_PROBE_Y="$3"
      shift 3
      ;;
    --require-frontier-takeover)
      TARGET_PROBE_REQUIRE_FRONTIER=true
      shift
      ;;
    --no-pre-route-turn)
      ALIGNMENT=false
      shift
      ;;
    --raw-costmap)
      CONVERTER=disabled
      shift
      ;;
    --polygon-only)
      INCLUDE_COSTMAP_OBSTACLES=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[error] unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "$MAX_SECONDS" in
  ''|*[!0-9]*) echo "[error] --max-seconds must be an integer" >&2; exit 2 ;;
esac
case "$STARTUP_SECONDS" in
  ''|*[!0-9]*) echo "[error] --startup-seconds must be an integer" >&2; exit 2 ;;
esac
if ((MAX_SECONDS <= 0 || STARTUP_SECONDS <= 0)); then
  echo "[error] time limits must be positive" >&2
  exit 2
fi

case "$TARGET_PROBE_TIMEOUT" in
  ''|*[!0-9]*) echo "[error] target probe timeout must be an integer" >&2; exit 2 ;;
esac
if ((TARGET_PROBE_TIMEOUT <= 0)); then
  echo "[error] target probe timeout must be positive" >&2
  exit 2
fi
case "$TARGET_PROBE_TRANSACTION_ID" in
  ''|*[!0-9]*) echo "[error] target transaction id must be an integer" >&2; exit 2 ;;
esac
if ((TARGET_PROBE_TRANSACTION_ID <= 0)); then
  echo "[error] target transaction id must be positive" >&2
  exit 2
fi
case "$TARGET_PROBE_EPOCH" in
  ''|*[!0-9]*) echo "[error] target epoch must be an integer" >&2; exit 2 ;;
esac
if ((TARGET_PROBE_EPOCH < 0)); then
  echo "[error] target epoch must be non-negative" >&2
  exit 2
fi
if [[ "$TARGET_PROBE_REQUIRE_FRONTIER" != "true" \
  && "$TARGET_PROBE_REQUIRE_FRONTIER" != "false" ]]; then
  echo "[error] target frontier requirement must be true or false" >&2
  exit 2
fi

if [[ "$PERSISTENT_EXECUTION" != "true" && "$PERSISTENT_EXECUTION" != "false" ]]; then
  echo "[error] persistent execution must be true or false" >&2
  exit 2
fi

# Persistent execution is a paired Navfn/TEB contract. Selecting only one
# plugin would make the run look persistent while move_base still follows the
# ordinary endpoint-action lifecycle.
BASE_GLOBAL_PLANNER="navfn/NavfnROS"
BASE_LOCAL_PLANNER="teb_local_planner/TebLocalPlannerROS"
if [[ "$PERSISTENT_EXECUTION" == "true" ]]; then
  BASE_GLOBAL_PLANNER="lste_topo_access/StreamingNavfnPlanner"
  BASE_LOCAL_PLANNER="lste_topo_access/PersistentTebLocalPlanner"
  # The persistent action is intentionally kept alive, so move_base's planner
  # thread must periodically call StreamingNavfnPlanner after a target command
  # arrives. This is a wake-up contract, not a TEB tuning parameter.
  PLANNER_FREQUENCY="${T_JUNCTION_PLANNER_FREQUENCY:-20.0}"
fi

# Source catkin only after consuming script arguments.  A sourced setup file
# inherits "$@"; passing ``--help`` or an option into catkin's setup util makes
# it interpret the option as its own and can produce a misleading startup
# error before this runner has done any work.
source "$WS/scripts/config/pipeline_env.sh"
source "$WS/scripts/tests/targeted_navigation/readiness.sh"

PIPELINE_CONFIG_PATH="${PIPELINE_CONFIG:-$WS/scripts/config/pipeline_defaults.yaml}"
if [[ -f "$PIPELINE_CONFIG_PATH" ]]; then
  PIPELINE_CONFIG_SHA256="$(sha256sum "$PIPELINE_CONFIG_PATH" | awk '{print $1}')"
else
  PIPELINE_CONFIG_SHA256=missing
fi
GIT_REVISION="$(git -C "$WS" rev-parse HEAD 2>/dev/null || echo unknown)"
if git -C "$WS" diff --quiet && git -C "$WS" diff --cached --quiet; then
  GIT_DIRTY=false
else
  GIT_DIRTY=true
fi

LOG_ROOT="$WS/runtime/targeted_navigation/t_junction_small/logs"
mkdir -p "$LOG_ROOT"
while :; do
  RUN_TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
  RUN_DIRECTORY="$LOG_ROOT/$RUN_TIMESTAMP"
  if mkdir "$RUN_DIRECTORY" 2>/dev/null; then
    break
  fi
  sleep 1
done

LAUNCHER_LOG="$RUN_DIRECTORY/${RUN_TIMESTAMP}_launcher.log"
METRICS_LOG="$RUN_DIRECTORY/${RUN_TIMESTAMP}_navigation_metrics.log"
SUMMARY_LOG="$RUN_DIRECTORY/${RUN_TIMESTAMP}_summary.log"
LIFECYCLE_LOG="$RUN_DIRECTORY/${RUN_TIMESTAMP}_lifecycle.log"
STARTUP_ARTIFACT="$RUN_DIRECTORY/${RUN_TIMESTAMP}_startup_failure.json"
TRIAL_END_ARTIFACT="$RUN_DIRECTORY/${RUN_TIMESTAMP}_trial_end_diagnostic.json"
REPORT_JSON="$RUN_DIRECTORY/${RUN_TIMESTAMP}_failure_report.json"
REPORT_MARKDOWN="$RUN_DIRECTORY/${RUN_TIMESTAMP}_failure_report.md"
WORLD="$WS/worlds/targeted_navigation/t_junction_small.world"
RUN_CONFIG="world=$WORLD initial_pose=[-2.5,0.0,0.1,0.0] max_seconds=$MAX_SECONDS startup_seconds=$STARTUP_SECONDS stop_on_failure=$STOP_ON_FAILURE persistent_execution=$PERSISTENT_EXECUTION planner_frequency=$PLANNER_FREQUENCY base_global_planner=$BASE_GLOBAL_PLANNER base_local_planner=$BASE_LOCAL_PLANNER target_probe=$TARGET_PROBE target_probe_goal=[$TARGET_PROBE_X,$TARGET_PROBE_Y] target_probe_transaction_id=$TARGET_PROBE_TRANSACTION_ID target_probe_epoch=$TARGET_PROBE_EPOCH target_probe_track_id=$TARGET_PROBE_TRACK_ID target_probe_require_frontier=$TARGET_PROBE_REQUIRE_FRONTIER pre_route_alignment=$ALIGNMENT converter=$CONVERTER include_costmap_obstacles=$INCLUDE_COSTMAP_OBSTACLES"

log_lifecycle() {
  local level="$1"
  local event="$2"
  shift 2
  printf '%s [%s] process=t_junction_runner event=%s run_timestamp=%s %s\n' \
    "$(date --iso-8601=seconds)" "$level" "$event" "$RUN_TIMESTAMP" "$*" \
    >>"$LIFECYCLE_LOG"
}

log_lifecycle INFO run_start "$RUN_CONFIG"

LAUNCH_PID=""
TARGET_PROBE_PID=""
TARGET_PROBE_RESULT=""
TARGET_PROBE_STDERR=""
LAUNCH_GROUP=false
STOP_REASON="window_timeout"
STARTUP_STARTED_SECONDS=0
CLEANUP_DONE=false

report_outcome() {
  [[ -f "$REPORT_JSON" ]] || return 0
  python3 - "$REPORT_JSON" <<'PY'
import json
import sys

try:
    payload = json.loads(open(sys.argv[1], encoding="utf-8").read())
except (OSError, TypeError, ValueError, json.JSONDecodeError):
    print("unknown")
else:
    print(str(payload.get("outcome") or "unknown"))
PY
}

record_startup_failure() {
  local reason="$1"
  local elapsed_seconds=$((SECONDS - STARTUP_STARTED_SECONDS))
  # Refresh the inventory at the failure boundary; this is the state that
  # explains whether a process, topic, or metrics writer was missing.
  readiness_check t_junction "$METRICS_LOG" || true
  STOP_REASON="$reason"
  log_lifecycle ERROR startup_failure \
    reason="$reason" elapsed_wall_seconds="$elapsed_seconds" \
    missing="$(readiness_missing_summary)"
  readiness_write_failure_artifact \
    "$STARTUP_ARTIFACT" t_junction_runner "$reason" "$RUN_TIMESTAMP" \
    "$LAUNCHER_LOG" "$METRICS_LOG" "$LIFECYCLE_LOG" "$LAUNCH_PID" \
    "$elapsed_seconds" "$RUN_CONFIG"
}

cleanup() {
  local status=$?
  if [[ "$CLEANUP_DONE" == "true" ]]; then
    return
  fi
  CLEANUP_DONE=true
  set +e
  log_lifecycle INFO run_stop_requested reason="$STOP_REASON"
  if [[ -n "$TARGET_PROBE_PID" ]]; then
    if kill -0 "$TARGET_PROBE_PID" 2>/dev/null; then
      kill -TERM "$TARGET_PROBE_PID" 2>/dev/null || true
    fi
    wait "$TARGET_PROBE_PID" 2>/dev/null || true
  fi
  if [[ -n "$LAUNCH_PID" ]]; then
    if [[ "$LAUNCH_GROUP" == "true" ]]; then
      kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
    elif kill -0 "$LAUNCH_PID" 2>/dev/null; then
      kill -INT "$LAUNCH_PID" 2>/dev/null || true
    fi
    sleep 1
    if [[ "$LAUNCH_GROUP" == "true" ]]; then
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
    elif kill -0 "$LAUNCH_PID" 2>/dev/null; then
      kill -TERM "$LAUNCH_PID" 2>/dev/null || true
    fi
    sleep 1
    if [[ "$LAUNCH_GROUP" == "true" ]]; then
      kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
    elif kill -0 "$LAUNCH_PID" 2>/dev/null; then
      kill -KILL "$LAUNCH_PID" 2>/dev/null || true
    fi
  fi
  if [[ -n "$LAUNCH_PID" ]]; then
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  log_lifecycle INFO run_stop reason="$STOP_REASON" status="$status"
  if [[ -f "$METRICS_LOG" ]]; then
    python3 "$WS/scripts/tools/analyze_navigation_metrics.py" \
      "$METRICS_LOG" >"$SUMMARY_LOG" 2>&1 || true
  fi
  if [[ ! -f "$STARTUP_ARTIFACT" ]] \
    && [[ -f "$METRICS_LOG" ]] \
    && [[ "$STOP_REASON" == "window_timeout" || "$STOP_REASON" == "launch_process_exit" ]]; then
    if readiness_write_trial_end_artifact \
      "$TRIAL_END_ARTIFACT" "$METRICS_LOG" "$RUN_TIMESTAMP" \
      "$RUN_CONFIG" "$WS" "$STOP_REASON"; then
      log_lifecycle WARN trial_end_diagnostic \
        outcome="$STOP_REASON" artifact="$TRIAL_END_ARTIFACT"
    fi
  fi
  if [[ -f "$METRICS_LOG" || -f "$STARTUP_ARTIFACT" ]]; then
    if python3 "$WS/scripts/tools/navigation_failure_report.py" \
      "$RUN_DIRECTORY" \
      --output-json "$REPORT_JSON" \
      --output-markdown "$REPORT_MARKDOWN" \
      >/dev/null 2>&1; then
      log_lifecycle INFO failure_report_ready \
        json="$REPORT_JSON" markdown="$REPORT_MARKDOWN"
    else
      log_lifecycle WARN failure_report_failed
    fi
  fi
  if [[ -f "$REPORT_JSON" ]]; then
    log_lifecycle INFO run_outcome \
      outcome="$(report_outcome)" stop_reason="$STOP_REASON" \
      report_json="$REPORT_JSON"
  fi
  if [[ -f "$STARTUP_ARTIFACT" ]]; then
    printf 'startup failure artifact: %s\n' "$STARTUP_ARTIFACT" >>"$SUMMARY_LOG"
  fi
  if [[ -f "$TRIAL_END_ARTIFACT" ]]; then
    printf 'trial end diagnostic: %s\n' "$TRIAL_END_ARTIFACT" >>"$SUMMARY_LOG"
  fi
  if [[ -f "$REPORT_JSON" ]]; then
    printf 'failure report json: %s\n' "$REPORT_JSON" >>"$SUMMARY_LOG"
    printf 'failure report markdown: %s\n' "$REPORT_MARKDOWN" >>"$SUMMARY_LOG"
  fi
  printf 'run_directory=%s\n' "$RUN_DIRECTORY"
  printf 'summary=%s\n' "$SUMMARY_LOG"
  if [[ -f "$STARTUP_ARTIFACT" ]]; then
    printf 'startup_failure=%s\n' "$STARTUP_ARTIFACT"
  fi
  if [[ -f "$REPORT_MARKDOWN" ]]; then
    printf 'failure_report=%s\n' "$REPORT_MARKDOWN"
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

LAUNCH_ARGS=(
  "$WS/scripts/tests/targeted_navigation/t_junction_nav.launch"
  "gui:=$GUI"
  start_metrics:=true
  "metrics_log_dir:=$LOG_ROOT"
  "metrics_run_directory:=$RUN_DIRECTORY"
  "metrics_run_timestamp:=$RUN_TIMESTAMP"
  "metrics_pipeline_config:=$PIPELINE_CONFIG_PATH"
  "metrics_pipeline_config_sha256:=$PIPELINE_CONFIG_SHA256"
  "metrics_git_revision:=$GIT_REVISION"
  "metrics_git_dirty:=$GIT_DIRTY"
  metrics_failure_sample_period:=0.20
  metrics_failure_pre_window:=4.0
  metrics_failure_post_window:=1.5
  metrics_failure_zero_velocity_seconds:=3.0
  metrics_failure_no_progress_seconds:=4.0
  "teb_frontier_pre_route_alignment:=$ALIGNMENT"
  "teb_costmap_converter_plugin:=$CONVERTER"
  "teb_include_costmap_obstacles:=$INCLUDE_COSTMAP_OBSTACLES"
  "persistent_execution:=$PERSISTENT_EXECUTION"
  "base_global_planner:=$BASE_GLOBAL_PLANNER"
  "base_local_planner:=$BASE_LOCAL_PLANNER"
  "planner_frequency:=$PLANNER_FREQUENCY"
)
if command -v setsid >/dev/null 2>&1; then
  setsid roslaunch "${LAUNCH_ARGS[@]}" >"$LAUNCHER_LOG" 2>&1 &
  LAUNCH_GROUP=true
else
  roslaunch "${LAUNCH_ARGS[@]}" >"$LAUNCHER_LOG" 2>&1 &
fi
LAUNCH_PID=$!
STARTUP_STARTED_SECONDS=$SECONDS

ready_deadline=$((SECONDS + STARTUP_SECONDS))
last_missing=""
navigation_ready=false
while ((SECONDS < ready_deadline)); do
  if readiness_check t_junction "$METRICS_LOG"; then
    navigation_ready=true
    break
  fi
  missing="$(readiness_missing_summary)"
  if [[ "$missing" != "$last_missing" ]]; then
    log_lifecycle INFO startup_readiness_wait missing="$missing"
    last_missing="$missing"
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    record_startup_failure startup_process_exit
    exit 1
  fi
  sleep 0.25
done
if [[ "$navigation_ready" != "true" ]]; then
  record_startup_failure startup_readiness_timeout
  exit 1
fi
log_lifecycle INFO navigation_ready \
  elapsed_wall_seconds="$((SECONDS - STARTUP_STARTED_SECONDS))" missing=none

if [[ "$TARGET_PROBE" == "true" ]]; then
  TARGET_PROBE_RESULT="$RUN_DIRECTORY/${RUN_TIMESTAMP}_target_probe_result.json"
  TARGET_PROBE_STDERR="$RUN_DIRECTORY/${RUN_TIMESTAMP}_target_probe_stdout.log"
  TARGET_PROBE_ARGS=(
    --log-directory "$RUN_DIRECTORY"
    --run-timestamp "$RUN_TIMESTAMP"
    --result-file "$TARGET_PROBE_RESULT"
    --timeout "$TARGET_PROBE_TIMEOUT"
    --goal-x "$TARGET_PROBE_X"
    --goal-y "$TARGET_PROBE_Y"
    --transaction-id "$TARGET_PROBE_TRANSACTION_ID"
    --target-epoch "$TARGET_PROBE_EPOCH"
    --target-track-id "$TARGET_PROBE_TRACK_ID"
  )
  if [[ "$TARGET_PROBE_REQUIRE_FRONTIER" == "true" ]]; then
    TARGET_PROBE_ARGS+=(--require-frontier-takeover)
  fi
  python3 "$WS/scripts/tests/targeted_navigation/publish_target_mission.py" \
    "${TARGET_PROBE_ARGS[@]}" \
    >"$TARGET_PROBE_STDERR" 2>&1 &
  TARGET_PROBE_PID=$!
  log_lifecycle INFO target_probe_started \
    pid="$TARGET_PROBE_PID" goal="[$TARGET_PROBE_X,$TARGET_PROBE_Y]" \
    transaction_id="$TARGET_PROBE_TRANSACTION_ID" \
    target_epoch="$TARGET_PROBE_EPOCH" target_track_id="$TARGET_PROBE_TRACK_ID" \
    require_frontier_takeover="$TARGET_PROBE_REQUIRE_FRONTIER" \
    timeout_seconds="$TARGET_PROBE_TIMEOUT" result="$TARGET_PROBE_RESULT" \
    stderr="$TARGET_PROBE_STDERR"
fi

navigation_deadline=$((SECONDS + MAX_SECONDS))
while ((SECONDS < navigation_deadline)); do
  if [[ "$TARGET_PROBE" == "true" && -f "$TARGET_PROBE_RESULT" ]]; then
    if rg -q '"passed": true' "$TARGET_PROBE_RESULT"; then
      STOP_REASON="target_probe_complete"
      log_lifecycle INFO target_probe_complete result="$TARGET_PROBE_RESULT"
      break
    fi
    STOP_REASON="target_probe_failed"
    log_lifecycle ERROR target_probe_failed result="$TARGET_PROBE_RESULT"
    exit 1
  fi
  # The target-failure probe deliberately creates a metrics failure episode;
  # let its three-stage ownership result decide when this diagnostic ends.
  if [[ "$TARGET_PROBE" != "true" ]] \
    && [[ "$STOP_ON_FAILURE" == "true" ]] \
    && rg -q 'event=failure_snapshot_ready' "$METRICS_LOG"; then
    STOP_REASON="failure_snapshot_ready"
    log_lifecycle WARN failure_stop_requested
    break
  fi
  if [[ "$TARGET_PROBE" == "true" ]] \
    && [[ -n "$TARGET_PROBE_PID" ]] \
    && ! kill -0 "$TARGET_PROBE_PID" 2>/dev/null \
    && [[ ! -f "$TARGET_PROBE_RESULT" ]]; then
    wait "$TARGET_PROBE_PID" 2>/dev/null
    probe_status=$?
    STOP_REASON="target_probe_process_exit"
    log_lifecycle ERROR target_probe_process_exit \
      status="$probe_status" stderr="$TARGET_PROBE_STDERR"
    exit 1
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    STOP_REASON="launch_process_exit"
    log_lifecycle ERROR run_aborted reason="$STOP_REASON"
    exit 1
  fi
  sleep 0.25
done
