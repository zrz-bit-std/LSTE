#!/usr/bin/env bash
set -euo pipefail

# One-command entry point for the single complex office-building benchmark.
# It keeps all four deterministic world artifacts and never edits production
# defaults or enters tmux.

WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
WORLD=""
MANIFEST="$WS/worlds/benchmark/office_building_v1_manifest.yaml"
CONFIG="$WS/scripts/tests/office_building/office_building_config.yaml"
GENERATOR="$WS/scripts/tests/office_building/generate_world.py"
VALIDATOR="$WS/scripts/tests/office_building/validate_office_building.py"
SUMMARY="$WS/scripts/tests/office_building/summarize_run.py"
TOPOLOGY_VERIFIER="$WS/scripts/tests/office_building/verify_topology_exploration.py"
PREFIX_LOGGER="$WS/scripts/tests/office_building/prefix_log.py"
PRUNE_LOGS="$WS/scripts/tools/prune_runtime_logs.py"
LOG_ROOT="$WS/runtime/office_building_benchmark/logs"
CURRENT_RUN_LINK="$WS/runtime/office_building_benchmark/current"
LOG_MAX_BYTES_LIMIT=1000000000
LEVEL="${OFFICE_BUILDING_LEVEL:-level_2}"
PROFILE="${OFFICE_BUILDING_PROFILE:-primary}"

RUN_TIMESTAMP=""
RUN_DIR=""
LIFECYCLE_LOG=""

usage() {
  cat <<'EOF'
Usage: run_office_building.sh <command> [options]

Commands:
  generate [level]         Regenerate one deterministic level without touching others.
  validate [level]         Validate the world, manifest, and local models.
  start [level]            Stop an old LSTE run, then start the benchmark in background.
  stop                     Stop the complete LSTE system.
  status                   Show the benchmark tmux sessions and ROS nodes.
  profile [name]           Print the selected robot profile pose without starting ROS.
  summary [log]            Summarize a navigation metrics log (latest by default).
  verify [log]             Assert topology exploration behavior (latest by default).

Levels: level_1, level_2, level_3, level_4.
Set OFFICE_BUILDING_PROFILE=target_corridor_entry for the short target-context
perception profile; it starts in the corridor closest to the target room.
The launcher does not attach to tmux. Use `tmux attach -t lste` only for diagnosis.
EOF
}

select_profile() {
  local profile="$1"
  python3 - "$MANIFEST" "$profile" <<'PY'
import sys
import yaml

manifest = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
profile = (manifest.get("robot_profiles", {}) or {}).get(sys.argv[2])
if not profile or len(profile.get("pose", [])) != 3:
    raise SystemExit("unknown robot profile: %s" % sys.argv[2])
for value in profile["pose"]:
    print(value)
PY
}

world_for_level() {
  local level="$1"
  python3 - "$MANIFEST" "$level" <<'PY'
import sys
import yaml

manifest = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
level = (manifest.get("levels", {}) or {}).get(sys.argv[2], {}) or {}
world = str(level.get("world") or "").strip()
if not world:
    raise SystemExit("manifest has no world for level: %s" % sys.argv[2])
print(world)
PY
}

validate_world() {
  local level="$1"
  python3 "$VALIDATOR" --manifest "$MANIFEST" --level "$level"
}

generate_world() {
  local level="${1:-$LEVEL}"
  WORLD="$WS/$(world_for_level "$level")"
  python3 "$GENERATOR" --output "$WORLD" --level "$level"
}

config_scalar() {
  local key="$1"
  python3 - "$CONFIG" "$key" <<'PY'
import sys
import yaml

data = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
value = data.get(sys.argv[2], "")
print(value if value is not None else "")
PY
}

LOG_MAX_BYTES="${OFFICE_BUILDING_LOG_MAX_BYTES:-$(config_scalar LOG_MAX_BYTES)}"
LOG_MAX_BYTES="${LOG_MAX_BYTES:-1000000000}"
if ! [[ "$LOG_MAX_BYTES" =~ ^[1-9][0-9]*$ ]] || (( LOG_MAX_BYTES > LOG_MAX_BYTES_LIMIT )); then
  echo "[error] LOG_MAX_BYTES must be between 1 and $LOG_MAX_BYTES_LIMIT bytes" >&2
  exit 2
fi

resolve_task_json() {
  local task_json="$1"
  if [[ -z "$task_json" ]]; then
    echo "[error] task JSON is empty" >&2
    return 1
  fi
  if [[ "$task_json" != /* ]]; then
    task_json="$WS/$task_json"
  fi
  if [[ ! -f "$task_json" ]]; then
    echo "[error] task JSON does not exist: $task_json" >&2
    return 1
  fi
  printf '%s\n' "$task_json"
}

target_xy() {
  python3 - "$MANIFEST" <<'PY'
import sys
import yaml

data = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
target = (data.get("targets", {}) or {}).get("primary", {}) or {}
xy = target.get("pose_xy", [])
if len(xy) != 2:
    raise SystemExit("manifest primary target pose_xy must contain two values")
print("%s,%s" % (xy[0], xy[1]))
PY
}

prepare_run_logs() {
  mkdir -p "$LOG_ROOT"
  while true; do
    RUN_TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
    RUN_DIR="$LOG_ROOT/$RUN_TIMESTAMP"
    if mkdir "$RUN_DIR" 2>/dev/null; then
      break
    fi
    sleep 1
  done
  LIFECYCLE_LOG="$RUN_DIR/${RUN_TIMESTAMP}_lifecycle.log"
  ln -sfn "$RUN_DIR" "$CURRENT_RUN_LINK"
  LOG_RETENTION_RESULT=applied
  if ! python3 "$PRUNE_LOGS" \
      --runtime-root "$WS/runtime" \
      --max-bytes "$LOG_MAX_BYTES" \
      --keep "$RUN_DIR"; then
    LOG_RETENTION_RESULT=failed
    echo "[warning] runtime log budget cleanup failed" >&2
  fi
  export OFFICE_BUILDING_RUN_TIMESTAMP="$RUN_TIMESTAMP"
  export OFFICE_BUILDING_LOG_DIR="$RUN_DIR"
  export NAVIGATION_LOG_DIR="$RUN_DIR"
  export NAVIGATION_RUN_DIRECTORY="$RUN_DIR"
  export NAVIGATION_RUN_TIMESTAMP="$RUN_TIMESTAMP"
}

lifecycle_log() {
  local event="$1"
  shift || true
  printf '%s [INFO] process=office_building_launcher event=%s run_timestamp=%s %s\n' \
    "$(date '+%Y-%m-%dT%H:%M:%S%z')" "$event" "${RUN_TIMESTAMP:-unknown}" "$*" \
    >> "$LIFECYCLE_LOG"
}

notify_metrics_stop_reason() {
  # ``lste_navigation_metrics`` owns the bounded failure ring.  Pass the
  # runner's terminal boundary through the parameter server immediately before
  # rosnode kill so its shutdown callback can promote a final timeout state
  # into the same formal failure artifact as an in-run watchdog.
  if command -v rosnode >/dev/null 2>&1 && \
     rosnode list 2>/dev/null | grep -qx /lste_navigation_metrics; then
    rosparam set /lste_navigation_metrics/termination_reason "$STOP_REASON" \
      >/dev/null 2>&1 || true
  fi
}

prefix_captured_lines() {
  local process="$1"
  local log_file="$2"
  awk -v process="$process" '{printf "%s [INFO] process=%s event=stdout line=%s\n", strftime("%Y-%m-%dT%H:%M:%S%z"), process, $0; fflush()}' \
    >> "$log_file"
}

attach_pane_log() {
  local session="$1"
  local window="$2"
  local process="$3"
  local log_file="$RUN_DIR/${RUN_TIMESTAMP}_${process}.log"
  : > "$log_file"
  tmux capture-pane -pJ -t "=$session:$window" -S -1000 2>/dev/null \
    | prefix_captured_lines "$process" "$log_file" || true
  tmux pipe-pane -o -t "=$session:$window" \
    "python3 '$PREFIX_LOGGER' --process '$process' --output '$log_file'"
}

attach_process_logs() {
  local session window process
  while IFS='|' read -r session window process; do
    [[ -z "$session" ]] && continue
    if tmux has-session -t "=$session" 2>/dev/null && \
       tmux list-windows -t "=$session" -F '#{window_name}' 2>/dev/null | grep -qx "$window"; then
      attach_pane_log "$session" "$window" "$process"
    fi
  done <<'EOF'
lste-env|roscore|roscore
lste-env|world|gazebo_world
lste-env|gazebo_gui|gazebo_gui
lste|pro3|pro3_support
lste|task|task
lste|vllm|vllm
lste|goal|goal_manager
lste|prompt|prompt
lste|detector|detector
lste|score|score
lste|state|state
lste|vis|detector_visualization
lste|oc_srfc|oc_srfc
lste|global_frontier|global_frontier
lste|teb_nav|teb_navigation
lste|controller_switch|controller_switch
lste|cmd_vel_mux|cmd_vel_mux
lste|health|health_audit
lste|teleop|teleop
EOF
}

record_resolved_start() {
  local level="$1"
  local world_hash git_revision git_dirty target
  world_hash="$(sha256sum "$WORLD" | awk '{print $1}')"
  git_revision="$(git -C "$WS" rev-parse HEAD 2>/dev/null || echo unknown)"
  git_dirty=false
  if ! git -C "$WS" diff --quiet || ! git -C "$WS" diff --cached --quiet; then
    git_dirty=true
  fi
  target="$(target_xy)"
  lifecycle_log "run_start" \
    "scenario_id=office_building_v1 level=$level profile=$PROFILE trial_id=${OFFICE_BUILDING_TRIAL_ID:-none} gazebo_seed=${GAZEBO_RANDOM_SEED:-none} exploration_method=${GLOBAL_FRONTIER_METHOD:-$(config_scalar GLOBAL_FRONTIER_METHOD)} global_frontier_enabled=${GLOBAL_FRONTIER_ENABLED:-$(config_scalar GLOBAL_FRONTIER_ENABLED)} controller=$(config_scalar LSTE_CONTROLLER) detector=$(config_scalar DETECTOR) speed=$(config_scalar TEB_MAX_LINEAR_SPEED) target_done_policy=${TARGET_DONE_POLICY:-$(config_scalar TARGET_DONE_POLICY)} target_room_entry_bounds=${TARGET_ROOM_ENTRY_MIN_X:-$(config_scalar TARGET_ROOM_ENTRY_MIN_X)},${TARGET_ROOM_ENTRY_MIN_Y:-$(config_scalar TARGET_ROOM_ENTRY_MIN_Y)},${TARGET_ROOM_ENTRY_MAX_X:-$(config_scalar TARGET_ROOM_ENTRY_MAX_X)},${TARGET_ROOM_ENTRY_MAX_Y:-$(config_scalar TARGET_ROOM_ENTRY_MAX_Y)} log_max_bytes=$LOG_MAX_BYTES initial_pose=${pose[*]} target=$target task_id=${TASK_ID:-unknown} task_json=${TASK_JSON:-unknown} world=$WORLD world_sha256=$world_hash git_revision=$git_revision git_dirty=$git_dirty"
}

start_benchmark() {
  local level="$1"
  local task_json task_id
  generate_world "$level"
  validate_world "$level"

  mapfile -t pose < <(select_profile "$PROFILE")
  if (( ${#pose[@]} != 3 )); then
    echo "[error] invalid pose profile: $PROFILE" >&2
    exit 1
  fi
  task_json="$(resolve_task_json "${OFFICE_BUILDING_TASK_JSON:-$(config_scalar TASK_JSON)}")"
  task_id="${OFFICE_BUILDING_TASK_ID:-$(config_scalar TASK_ID)}"
  if [[ -z "$task_id" ]]; then
    echo "[error] task ID is empty" >&2
    exit 1
  fi

  if [[ "${OFFICE_BUILDING_PRESERVE_RUNTIME:-0}" == "1" ||
        "${OFFICE_BUILDING_PRESERVE_RUNTIME:-}" == "true" ]]; then
    echo "[office] preserving lste-env; rebuilding only business nodes"
    if tmux has-session -t "=lste" 2>/dev/null; then
      tmux kill-session -t "=lste"
    fi
  else
    echo "[office] stopping any previous LSTE run"
    "$WS/scripts/bin/stopall" || true
  fi

  export TASK_JSON="$task_json"
  export TASK_ID="$task_id"
  prepare_run_logs

  # ``target_entry`` remains a doorway-approach diagnostic profile. The newer
  # ``target_corridor_entry`` deliberately keeps frontier exploration enabled
  # so this short run tests perception from the nearest corridor start.
  if [[ "$PROFILE" == "target_entry" ]]; then
    export GLOBAL_FRONTIER_ENABLED=false
    export LEGACY_GP_FRONTIER_ENABLED=false
    lifecycle_log "diagnostic_profile" \
      "profile=target_entry global_frontier_enabled=false reason=bypasses_exploration"
  fi
  record_resolved_start "$level"

  export PIPELINE_CONFIG="$CONFIG"
  export WORLD
  export PRO3_SPAWN_X="${pose[0]}"
  export PRO3_SPAWN_Y="${pose[1]}"
  export PRO3_SPAWN_Z=0.1
  export PRO3_SPAWN_YAW="${pose[2]}"
  export LSTE_NO_ATTACH=1
  export OFFICE_BUILDING_LEVEL="$level"
  export OFFICE_BUILDING_MANIFEST="$MANIFEST"
  export OFFICE_BUILDING_COLLISION_TRUTH_ENABLED=true
  export OFFICE_BUILDING_CONTACT_SENSOR=true

  lifecycle_log "log_retention" \
    "result=$LOG_RETENTION_RESULT max_bytes=$LOG_MAX_BYTES runtime_root=$WS/runtime"
  echo "[office] starting scenario=office_building_v1 level=$level profile=$PROFILE"
  local launcher_log="$RUN_DIR/${RUN_TIMESTAMP}_launcher.log"
  if ! "$WS/scripts/lifecycle/run_all_tmux.sh" 2>&1 \
      | awk -v process=office_building_launcher '{printf "%s [INFO] process=%s event=stdout line=%s\\n", strftime("%Y-%m-%dT%H:%M:%S%z"), process, $0; fflush()}' \
      >> "$launcher_log"; then
    lifecycle_log "startup_failed" "level=$level profile=$PROFILE"
    attach_process_logs
    return 1
  fi

  attach_process_logs
  lifecycle_log "process_ready" "level=$level profile=$PROFILE controller=$(config_scalar LSTE_CONTROLLER) log_dir=$RUN_DIR"
  echo "[office] benchmark started in background"
}

case "${1:-}" in
  generate)
    generate_world "${2:-$LEVEL}"
    ;;
  validate)
    validate_world "${2:-$LEVEL}"
    ;;
  start)
    start_benchmark "${2:-$LEVEL}"
    ;;
  stop)
    STOP_REASON="${OFFICE_BUILDING_STOP_REASON:-operator_request}"
    if [[ -L "$CURRENT_RUN_LINK" || -d "$CURRENT_RUN_LINK" ]]; then
      RUN_DIR="$(readlink -f "$CURRENT_RUN_LINK" 2>/dev/null || true)"
      if [[ -n "$RUN_DIR" && -d "$RUN_DIR" ]]; then
        RUN_TIMESTAMP="$(basename "$RUN_DIR")"
        LIFECYCLE_LOG="$RUN_DIR/${RUN_TIMESTAMP}_lifecycle.log"
        lifecycle_log "stop_requested" "reason=$STOP_REASON"
      fi
    fi
    notify_metrics_stop_reason
    "$WS/scripts/bin/stopall"
    if [[ -n "${LIFECYCLE_LOG:-}" && -f "$LIFECYCLE_LOG" ]]; then
      lifecycle_log "stop_completed" "reason=$STOP_REASON"
    fi
    ;;
  status)
    tmux list-sessions 2>/dev/null || true
    rosnode list 2>/dev/null | rg '/(lste_goal_manager|lste_global_frontier|lste_navigation_metrics|move_base)$' || true
    ;;
  profile)
    if [[ $# -ge 2 ]]; then
      PROFILE="$2"
    fi
    select_profile "$PROFILE"
    ;;
  summary)
    if [[ $# -ge 2 ]]; then
      exec python3 "$SUMMARY" "$2"
    fi
    exec python3 "$SUMMARY"
    ;;
  verify)
    if [[ $# -ge 2 ]]; then
      exec python3 "$TOPOLOGY_VERIFIER" "${@:2}"
    fi
    exec python3 "$TOPOLOGY_VERIFIER"
    ;;
  -h|--help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
