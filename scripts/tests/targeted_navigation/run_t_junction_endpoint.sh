#!/usr/bin/env bash
set -euo pipefail

# Reproduce one controller/endpoint failure from a nearby pose. This is the
# fast inner loop for endpoint-admission work; run_t_junction.sh remains the
# topology integration experiment.
WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
MAX_SECONDS="${T_JUNCTION_ENDPOINT_MAX_SECONDS:-30}"
STARTUP_SECONDS="${T_JUNCTION_ENDPOINT_STARTUP_SECONDS:-35}"
GUI="${T_JUNCTION_ENDPOINT_GUI:-false}"
START_X="${T_JUNCTION_ENDPOINT_START_X:-3.0}"
START_Y="${T_JUNCTION_ENDPOINT_START_Y:-0.2}"
GOAL_X="${T_JUNCTION_ENDPOINT_GOAL_X:-4.45}"
GOAL_Y="${T_JUNCTION_ENDPOINT_GOAL_Y:-0.15}"
FORWARD_ONLY="${T_JUNCTION_ENDPOINT_FORWARD_ONLY:-true}"
STOP_ON_FAILURE="${T_JUNCTION_ENDPOINT_STOP_ON_FAILURE:-true}"
STOP_ON_SUCCESS="${T_JUNCTION_ENDPOINT_STOP_ON_SUCCESS:-true}"
CONVERTER="${T_JUNCTION_ENDPOINT_CONVERTER:-costmap_converter::CostmapToPolygonsDBSMCCH}"
INCLUDE_COSTMAP_OBSTACLES="${T_JUNCTION_ENDPOINT_INCLUDE_COSTMAP_OBSTACLES:-true}"

usage() {
  cat <<'EOF'
Usage: run_t_junction_endpoint.sh [options]

Options:
  --start X Y           Nearby reproducible start pose (default: 3.0 -0.2)
  --goal X Y            Endpoint to test (default: 4.45 -0.15)
  --max-seconds N       Bounded controller window (default: 30)
  --startup-seconds N   Readiness wait (default: 35)
  --gui                 Start Gazebo GUI
  --allow-reverse       Let raw TEB reverse commands reach the base
  --raw-costmap         Disable the DBSMCCH obstacle-converter A/B path
  --polygon-only        Use converter polygons without raw costmap cells
  --no-stop-on-failure  Keep running until --max-seconds
  --no-stop-on-success  Keep the bounded window after a successful goal
  -h, --help            Show this help

Every run writes a failure report (`<timestamp>_failure_report.md/.json`) beside
the metrics and failure artifacts.  Use `scripts/bin/navdiag <run-directory>`
to read it without starting ROS.
EOF
}

while (($#)); do
  case "$1" in
    --start) START_X="$2"; START_Y="$3"; shift 3 ;;
    --goal) GOAL_X="$2"; GOAL_Y="$3"; shift 3 ;;
    --max-seconds) MAX_SECONDS="$2"; shift 2 ;;
    --startup-seconds) STARTUP_SECONDS="$2"; shift 2 ;;
    --gui) GUI=true; shift ;;
    --allow-reverse) FORWARD_ONLY=false; shift ;;
    --raw-costmap) CONVERTER=disabled; shift ;;
    --polygon-only) INCLUDE_COSTMAP_OBSTACLES=false; shift ;;
    --no-stop-on-failure) STOP_ON_FAILURE=false; shift ;;
    --no-stop-on-success) STOP_ON_SUCCESS=false; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[error] unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

for value in "$MAX_SECONDS" "$STARTUP_SECONDS"; do
  case "$value" in ''|*[!0-9]*) echo "[error] time limits must be integers" >&2; exit 2 ;; esac
done
if ((MAX_SECONDS <= 0 || STARTUP_SECONDS <= 0)); then
  echo "[error] time limits must be positive" >&2
  exit 2
fi
if [[ "$STOP_ON_FAILURE" != "true" && "$STOP_ON_FAILURE" != "false" ]]; then
  echo "[error] stop-on-failure must be true or false" >&2
  exit 2
fi
if [[ "$STOP_ON_SUCCESS" != "true" && "$STOP_ON_SUCCESS" != "false" ]]; then
  echo "[error] stop-on-success must be true or false" >&2
  exit 2
fi

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
LOG_ROOT="$WS/runtime/targeted_navigation/t_junction_endpoint/logs"
mkdir -p "$LOG_ROOT"
while :; do
  RUN_TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
  RUN_DIRECTORY="$LOG_ROOT/$RUN_TIMESTAMP"
  if mkdir "$RUN_DIRECTORY" 2>/dev/null; then break; fi
  sleep 1
done

LAUNCHER_LOG="$RUN_DIRECTORY/${RUN_TIMESTAMP}_launcher.log"
METRICS_LOG="$RUN_DIRECTORY/${RUN_TIMESTAMP}_navigation_metrics.log"
GOAL_LOG="$RUN_DIRECTORY/${RUN_TIMESTAMP}_goal_publisher.log"
SUMMARY_LOG="$RUN_DIRECTORY/${RUN_TIMESTAMP}_summary.log"
LIFECYCLE_LOG="$RUN_DIRECTORY/${RUN_TIMESTAMP}_lifecycle.log"
STARTUP_ARTIFACT="$RUN_DIRECTORY/${RUN_TIMESTAMP}_startup_failure.json"
TRIAL_END_ARTIFACT="$RUN_DIRECTORY/${RUN_TIMESTAMP}_trial_end_diagnostic.json"
REPORT_JSON="$RUN_DIRECTORY/${RUN_TIMESTAMP}_failure_report.json"
REPORT_MARKDOWN="$RUN_DIRECTORY/${RUN_TIMESTAMP}_failure_report.md"
RUN_CONFIG="world=$WS/worlds/targeted_navigation/t_junction_small.world initial_pose=[$START_X,$START_Y,0.1,0.0] goal=[$GOAL_X,$GOAL_Y] max_seconds=$MAX_SECONDS startup_seconds=$STARTUP_SECONDS forward_only=$FORWARD_ONLY stop_on_failure=$STOP_ON_FAILURE stop_on_success=$STOP_ON_SUCCESS converter=$CONVERTER include_costmap_obstacles=$INCLUDE_COSTMAP_OBSTACLES"

log_lifecycle() {
  local level="$1" event="$2"
  shift 2
  printf '%s [%s] process=t_junction_endpoint_runner event=%s run_timestamp=%s %s\n' \
    "$(date --iso-8601=seconds)" "$level" "$event" "$RUN_TIMESTAMP" "$*" \
    >>"$LIFECYCLE_LOG"
}

log_lifecycle INFO run_start "$RUN_CONFIG"

LAUNCH_PID=""
LAUNCH_GROUP=false
GOAL_PID=""
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
  readiness_check endpoint "$METRICS_LOG" || true
  STOP_REASON="$reason"
  log_lifecycle ERROR startup_failure \
    reason="$reason" elapsed_wall_seconds="$elapsed_seconds" \
    missing="$(readiness_missing_summary)"
  readiness_write_failure_artifact \
    "$STARTUP_ARTIFACT" t_junction_endpoint_runner "$reason" "$RUN_TIMESTAMP" \
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
  if [[ -n "$GOAL_PID" ]] && kill -0 "$GOAL_PID" 2>/dev/null; then
    kill -TERM "$GOAL_PID" 2>/dev/null || true
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
  wait "$GOAL_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
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

goal_succeeded() {
  # Metrics observes the action status directly. The run file is fresh for
  # every invocation, so a matching SUCCEEDED status belongs to this goal and
  # is stronger than a distance estimate sampled on a slower timer.
  [[ -f "$METRICS_LOG" ]] \
    && rg -q 'event=move_base_status .*"status_name":"SUCCEEDED"' "$METRICS_LOG"
}

ENDPOINT_LAUNCH_ARGS=(
  "$WS/scripts/tests/targeted_navigation/t_junction_endpoint_nav.launch"
  "gui:=$GUI"
  "x:=$START_X"
  "y:=$START_Y"
  "teb_forward_only:=$FORWARD_ONLY"
  start_metrics:=true
  "teb_costmap_converter_plugin:=$CONVERTER"
  "teb_include_costmap_obstacles:=$INCLUDE_COSTMAP_OBSTACLES"
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
)
if command -v setsid >/dev/null 2>&1; then
  setsid roslaunch "${ENDPOINT_LAUNCH_ARGS[@]}" >"$LAUNCHER_LOG" 2>&1 &
  LAUNCH_GROUP=true
else
  roslaunch "${ENDPOINT_LAUNCH_ARGS[@]}" >"$LAUNCHER_LOG" 2>&1 &
fi
LAUNCH_PID=$!
STARTUP_STARTED_SECONDS=$SECONDS

ready_deadline=$((SECONDS + STARTUP_SECONDS))
last_missing=""
navigation_ready=false
while ((SECONDS < ready_deadline)); do
  if readiness_check endpoint "$METRICS_LOG"; then
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
log_lifecycle INFO controller_ready \
  elapsed_wall_seconds="$((SECONDS - STARTUP_STARTED_SECONDS))" missing=none

python3 "$WS/scripts/tests/rl_fixed_goal/publish_goal.py" \
  _topic:=/lste/final_goal _goal_x:="$GOAL_X" _goal_y:="$GOAL_Y" \
  _frame_id:=map >"$GOAL_LOG" 2>&1 &
GOAL_PID=$!
log_lifecycle INFO goal_published goal="[$GOAL_X,$GOAL_Y]" frame=map

deadline=$((SECONDS + MAX_SECONDS))
while ((SECONDS < deadline)); do
  if [[ "$STOP_ON_FAILURE" == "true" ]] \
    && rg -q 'event=failure_snapshot_ready' "$METRICS_LOG"; then
    STOP_REASON="failure_snapshot_ready"
    log_lifecycle WARN failure_stop_requested
    break
  fi
  if [[ "$STOP_ON_SUCCESS" == "true" ]] && goal_succeeded; then
    STOP_REASON="goal_succeeded"
    log_lifecycle INFO goal_succeeded
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    STOP_REASON="launch_process_exit"
    log_lifecycle ERROR run_aborted reason="$STOP_REASON"
    exit 1
  fi
  sleep 0.25
done
