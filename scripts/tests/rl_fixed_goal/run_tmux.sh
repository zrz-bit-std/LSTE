#!/usr/bin/env bash
set -euo pipefail

# Start a completely separate Gazebo + SA-PPO fixed-goal validation. It has its
# own ROS/Gazebo masters and never launches LSTE brain nodes.
WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
TEST_DIR="$WS/scripts/tests/rl_fixed_goal"
CONFIG="${RL_FIXED_GOAL_CONFIG:-$TEST_DIR/config.yaml}"
SESSION="${RL_FIXED_GOAL_SESSION:-rl-fixed-goal-test}"
RUNTIME_DIR="$WS/runtime/rl_fixed_goal_test"
LOG_ROOT="$RUNTIME_DIR/logs"
PRUNE_LOGS="$WS/scripts/tools/prune_runtime_logs.py"
LOG_MAX_BYTES_LIMIT=1000000000
RL_DIR="$WS/rl_navigation"
POLICY_DIR="$RL_DIR/policy"
PYTHON_BIN="${SAPPO_PYTHON:-$HOME/miniconda3/envs/rlenvs/bin/python}"

if [[ ! -f "$CONFIG" ]]; then
  echo "[error] Test config not found: $CONFIG" >&2
  exit 1
fi

eval "$(
  python3 - "$CONFIG" <<'PY'
import sys
import shlex
import yaml

with open(sys.argv[1], encoding="utf-8") as stream:
    data = yaml.safe_load(stream) or {}
for key, value in data.items():
    if not isinstance(value, (str, int, float, bool)):
        raise SystemExit(f"Only scalar config values are supported: {key}")
    print(f"CFG_{key}={shlex.quote(str(value))}")
PY
)"

WORLD="$WS/${CFG_WORLD:?WORLD is required}"
GUI="${CFG_GUI:-true}"
INITIAL_X="${CFG_INITIAL_X:?INITIAL_X is required}"
INITIAL_Y="${CFG_INITIAL_Y:?INITIAL_Y is required}"
INITIAL_Z="${CFG_INITIAL_Z:?INITIAL_Z is required}"
INITIAL_YAW="${CFG_INITIAL_YAW:?INITIAL_YAW is required}"
GOAL_X="${CFG_GOAL_X:?GOAL_X is required}"
GOAL_Y="${CFG_GOAL_Y:?GOAL_Y is required}"
ROS_PORT="${CFG_ROS_PORT:-11312}"
GAZEBO_PORT="${CFG_GAZEBO_PORT:-11346}"
SAPPO_SPEED="${CFG_SAPPO_SPEED:-0.50}"
NAVIGATION_MAX_LINEAR_SPEED="${CFG_NAVIGATION_MAX_LINEAR_SPEED:-0.50}"
RECOVERY_MODE="${CFG_RECOVERY_MODE:-policy_only}"
CONTROLLER_MODE="${CFG_CONTROLLER_MODE:-$RECOVERY_MODE}"
ALLOW_GAZEBO_CLICK_GOAL="${CFG_ALLOW_GAZEBO_CLICK_GOAL:-false}"
FINAL_GOAL_MAX_FAILURES="${CFG_FINAL_GOAL_MAX_FAILURES:-3}"
LOCAL_PLANNER_PLUGIN="${CFG_LOCAL_PLANNER_PLUGIN:-teb_local_planner/TebLocalPlannerROS}"
CONTROLLER_METHOD="${CFG_CONTROLLER_METHOD:-}"
LIVE_RVIZ="${CFG_LIVE_RVIZ:-true}"
LOG_RETENTION_DAYS="${CFG_LOG_RETENTION_DAYS:-15}"
LOG_MAX_BYTES="${CFG_LOG_MAX_BYTES:-1000000000}"
SCREEN_RECORD="${CFG_SCREEN_RECORD:-false}"
SCREEN_RECORD_START_DELAY_SECONDS="${CFG_SCREEN_RECORD_START_DELAY_SECONDS:-18}"
SCREEN_RECORD_DURATION_SECONDS="${CFG_SCREEN_RECORD_DURATION_SECONDS:-75}"
SCREEN_RECORD_FPS="${CFG_SCREEN_RECORD_FPS:-30}"
SCREEN_RECORD_WINDOW="${CFG_SCREEN_RECORD_WINDOW:-RViz}"

if ! [[ "$LOG_MAX_BYTES" =~ ^[1-9][0-9]*$ ]] || (( LOG_MAX_BYTES > LOG_MAX_BYTES_LIMIT )); then
  echo "[error] LOG_MAX_BYTES must be between 1 and $LOG_MAX_BYTES_LIMIT bytes." >&2
  exit 2
fi

# A single comparison config can select the controller while retaining one
# shared world, spawn pose, goal publisher, and lifecycle. Older configs that
# set CONTROLLER_MODE directly are intentionally still supported.
if [[ -n "$CONTROLLER_METHOD" ]]; then
  case "${CONTROLLER_METHOD,,}" in
    teb)
      CONTROLLER_MODE="ros_navigation"
      LOCAL_PLANNER_PLUGIN="teb_local_planner/TebLocalPlannerROS"
      ;;
    dwa)
      CONTROLLER_MODE="ros_navigation"
      LOCAL_PLANNER_PLUGIN="dwa_local_planner/DWAPlannerROS"
      ;;
    rl)
      CONTROLLER_MODE="${CFG_RL_CONTROLLER_MODE:-policy_only}"
      ;;
    *)
      echo "[error] CONTROLLER_METHOD must be one of: teb, dwa, rl" >&2
      exit 1
      ;;
  esac
fi
ROS_URI="http://localhost:${ROS_PORT}"
GAZEBO_URI="http://localhost:${GAZEBO_PORT}"

if [[ ! -f "$WORLD" ]]; then
  echo "[error] World not found: $WORLD" >&2
  exit 1
fi
if [[ "$CONTROLLER_MODE" != "ros_navigation" && ( ! -x "$PYTHON_BIN" || ! -f "$RL_DIR/sappo_pure.py" || ! -f "$POLICY_DIR/sa_peppo_1650.pth" ) ]]; then
  echo "[error] SA-PPO runtime is incomplete." >&2
  exit 1
fi
if ! command -v tmux >/dev/null; then
  echo "[error] tmux is required." >&2
  exit 1
fi

# Two SA-PPO instances would share the GPU and invalidate a performance test.
# Do not stop the normal system automatically; the operator decides when to.
if tmux has-session -t "=lste" 2>/dev/null; then
  echo "[error] Existing LSTE session detected. Stop it before this isolated RL test." >&2
  exit 1
fi

if tmux has-session -t "=$SESSION" 2>/dev/null; then
  tmux kill-session -t "=$SESSION"
fi

mkdir -p "$RUNTIME_DIR" "$LOG_ROOT"
find "$LOG_ROOT" -mindepth 1 -maxdepth 1 -type d -mtime "+$LOG_RETENTION_DAYS" -exec rm -rf {} +

# The directory name is intentionally timestamp-only: it is the parent of all
# formal .log files for exactly one rltest invocation.
RUN_TIMESTAMP="${RL_FIXED_GOAL_RUN_TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="$LOG_ROOT/$RUN_TIMESTAMP"
while [[ -e "$LOG_DIR" ]]; do
  sleep 1
  RUN_TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
  LOG_DIR="$LOG_ROOT/$RUN_TIMESTAMP"
done
mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$RUNTIME_DIR/current_log"
LIFECYCLE_LOG="$LOG_DIR/${RUN_TIMESTAMP}_lifecycle.log"

VIDEO_ROOT="$RUNTIME_DIR/videos"
VIDEO_DIR="$VIDEO_ROOT/$RUN_TIMESTAMP"

lifecycle_log() {
  printf '%s [INFO] [launcher] run_timestamp=%s event=%s\n' \
    "$(date '+%Y-%m-%d %H:%M:%S.%3N')" "$RUN_TIMESTAMP" "$1" >> "$LIFECYCLE_LOG"
}

{
  lifecycle_log "run_start"
  printf '%s [INFO] [launcher] config_path=%s config_begin\n' \
    "$(date '+%Y-%m-%d %H:%M:%S.%3N')" "$CONFIG"
  sed 's/^/[CONFIG] /' "$CONFIG"
  printf '%s [INFO] [launcher] config_end\n' "$(date '+%Y-%m-%d %H:%M:%S.%3N')"
  printf '%s [INFO] [launcher] world=%s controller_method=%s controller_mode=%s local_planner=%s max_linear_speed=%s sappo_speed=%s initial_pose=(%s,%s,%s,%s) goal=(%s,%s) ros_port=%s gazebo_port=%s screen_record=%s git_revision=%s\n' \
    "$(date '+%Y-%m-%d %H:%M:%S.%3N')" "$WORLD" "${CONTROLLER_METHOD:-legacy}" "$CONTROLLER_MODE" "$LOCAL_PLANNER_PLUGIN" "$NAVIGATION_MAX_LINEAR_SPEED" "$SAPPO_SPEED" "$INITIAL_X" "$INITIAL_Y" "$INITIAL_Z" "$INITIAL_YAW" "$GOAL_X" "$GOAL_Y" "$ROS_PORT" "$GAZEBO_PORT" "$SCREEN_RECORD" "$(git -C "$WS" rev-parse --short HEAD 2>/dev/null || echo unknown)"
  printf '%s [INFO] [launcher] log_max_bytes=%s log_root=%s\n' \
    "$(date '+%Y-%m-%d %H:%M:%S.%3N')" "$LOG_MAX_BYTES" "$LOG_ROOT"
} >> "$LIFECYCLE_LOG"

LOG_RETENTION_RESULT=applied
if ! python3 "$PRUNE_LOGS" \
    --runtime-root "$WS/runtime" \
    --max-bytes "$LOG_MAX_BYTES" \
    --keep "$LOG_DIR"; then
  LOG_RETENTION_RESULT=failed
  echo "[warning] runtime log budget cleanup failed" >&2
fi
lifecycle_log "log_retention result=$LOG_RETENTION_RESULT max_bytes=$LOG_MAX_BYTES runtime_root=$WS/runtime"

if [[ "${SCREEN_RECORD,,}" == "true" ]]; then
  if ! [[ "$SCREEN_RECORD_START_DELAY_SECONDS" =~ ^[0-9]+$ && "$SCREEN_RECORD_DURATION_SECONDS" =~ ^[1-9][0-9]*$ && "$SCREEN_RECORD_FPS" =~ ^[1-9][0-9]*$ ]]; then
    echo "[error] Screen-recording delay, duration, and FPS must be integers." >&2
    exit 1
  fi
  mkdir -p "$VIDEO_DIR"
  find "$VIDEO_ROOT" -mindepth 1 -maxdepth 1 -type d -mtime "+$LOG_RETENTION_DAYS" -exec rm -rf {} +
  printf 'run_timestamp: %s\ncontroller_method: %s\ncontroller_mode: %s\nworld: %s\ninitial_pose: [%s, %s, %s, %s]\ngoal: [%s, %s]\nrecording_window: %s\nrecording_start_delay_seconds: %s\nrecording_duration_seconds: %s\nrecording_fps: %s\ngit_revision: %s\n' \
    "$RUN_TIMESTAMP" "${CONTROLLER_METHOD:-legacy}" "$CONTROLLER_MODE" "$WORLD" "$INITIAL_X" "$INITIAL_Y" "$INITIAL_Z" "$INITIAL_YAW" "$GOAL_X" "$GOAL_Y" "$SCREEN_RECORD_WINDOW" "$SCREEN_RECORD_START_DELAY_SECONDS" "$SCREEN_RECORD_DURATION_SECONDS" "$SCREEN_RECORD_FPS" "$(git -C "$WS" rev-parse --short HEAD 2>/dev/null || echo unknown)" \
    > "$VIDEO_DIR/${RUN_TIMESTAMP}_recording_manifest.yaml"
fi

ln -sfn "$POLICY_DIR" "$RUNTIME_DIR/policy"

tmux new-session -d -s "$SESSION" -n roscore -c "$WS" \
  "bash -lc 'export ROS_MASTER_URI=$ROS_URI GAZEBO_MASTER_URI=$GAZEBO_URI; WS=\"$WS\"; exec > >(tee -a \"$LOG_DIR/${RUN_TIMESTAMP}_roscore.log\") 2>&1; source \"$WS/scripts/config/pipeline_env.sh\"; printf \"%s [INFO] [roscore] event=node_start run_timestamp=$RUN_TIMESTAMP\\n\" \"\$(date \"+%Y-%m-%d %H:%M:%S.%3N\")\"; roscore -p $ROS_PORT; status=\$?; printf \"%s [INFO] [roscore] event=node_exit status=%s\\n\" \"\$(date \"+%Y-%m-%d %H:%M:%S.%3N\")\" \"\$status\"; exec bash'"
tmux set-option -t "$SESSION" remain-on-exit on

new_window() {
  local name="$1"
  local directory="$2"
  local command="$3"
  local log_name="${4:-$name}"
  local log_file="$LOG_DIR/${RUN_TIMESTAMP}_${log_name}.log"
  tmux new-window -d -t "$SESSION" -n "$name" -c "$directory" \
    "bash -lc 'export ROS_MASTER_URI=$ROS_URI GAZEBO_MASTER_URI=$GAZEBO_URI; WS=\"$WS\"; exec > >(tee -a \"$log_file\") 2>&1; source \"$WS/scripts/config/pipeline_env.sh\"; printf \"%s [INFO] [$log_name] event=node_start run_timestamp=$RUN_TIMESTAMP\\n\" \"\$(date \"+%Y-%m-%d %H:%M:%S.%3N\")\"; $command; status=\$?; printf \"%s [INFO] [$log_name] event=node_exit status=%s\\n\" \"\$(date \"+%Y-%m-%d %H:%M:%S.%3N\")\" \"\$status\"; echo; echo \"[EXIT] $name\"; exec bash'"
}

WAIT_ROS='until rostopic list >/dev/null 2>&1; do sleep 1; done'
new_window world "$WS" \
  "$WAIT_ROS; export LIBGL_ALWAYS_SOFTWARE=1; roslaunch lste_core lab_with_pro3.launch world_name:=$WORLD gui:=$GUI spawn_pro3:=true x:=$INITIAL_X y:=$INITIAL_Y z:=$INITIAL_Z yaw:=$INITIAL_YAW"
new_window scan "$WS" \
  "$WAIT_ROS; until rostopic list | grep -qx /pro3/wheel_odom; do sleep 1; done; roslaunch pointcloud_to_laserscan lste_pro3_to_scan.launch"
new_window fixed_goal "$WS" \
  "$WAIT_ROS; rosrun lste_topo_access lste_goal_manager.py _global_goal_source:=fixed _fixed_goal_x:=$GOAL_X _fixed_goal_y:=$GOAL_Y _fixed_goal_yaw:=0.0 _fixed_goal_publish_period:=1.0 _fixed_goal_allow_click_override:=$ALLOW_GAZEBO_CLICK_GOAL _debug_goal_log:=true"
new_window goal_sphere "$WS" \
  "$WAIT_ROS; until rostopic list | grep -qx /lste/final_goal; do sleep 1; done; python3 $TEST_DIR/gazebo_goal_sphere.py _goal_topic:=/lste/final_goal"

PYTHON_SITE="$($PYTHON_BIN -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
if [[ "$CONTROLLER_MODE" == "ros_navigation" ]]; then
  new_window navigation_goal "$WS" \
    "$WAIT_ROS; until rostopic list | grep -qx /pro3/wheel_odom && rostopic list | grep -qx /lste/final_goal; do sleep 1; done; python3 -u $TEST_DIR/frontier_goal_manager.py _goal_x:=$GOAL_X _goal_y:=$GOAL_Y _goal_topic:=/lste/final_goal _max_final_goal_failures:=$FINAL_GOAL_MAX_FAILURES" \
    "frontier_manager"
  new_window navigation "$WS" \
    "$WAIT_ROS; until rostopic list | grep -qx /pro3/wheel_odom && rostopic list | grep -qx /pro3/rlscan; do sleep 1; done; roslaunch $TEST_DIR/ros_navigation.launch local_planner_plugin:=$LOCAL_PLANNER_PLUGIN max_linear_speed:=$NAVIGATION_MAX_LINEAR_SPEED"
  if [[ "${LIVE_RVIZ,,}" == "true" ]]; then
    new_window live_rviz "$WS" \
      "$WAIT_ROS; until rostopic list | grep -qx /map; do sleep 1; done; rviz -d $TEST_DIR/live_navigation.rviz"
  fi
else
  new_window sappo "$RUNTIME_DIR" \
    "$WAIT_ROS; until rostopic list | grep -qx /pro3/wheel_odom && rostopic list | grep -qx /pro3/rlscan && rostopic list | grep -qx /lste/final_goal; do sleep 1; done; PYTHONPATH=$PYTHON_SITE:$WS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages $PYTHON_BIN $TEST_DIR/sappo_test.py _linear_speed_scale:=$SAPPO_SPEED _goal_x:=$GOAL_X _goal_y:=$GOAL_Y _goal_topic:=/lste/final_goal _cmd_vel_topic:=/cmd_vel _wait_for_goal:=true _subscribe_gp_subgoal:=false _recovery_mode:=$RECOVERY_MODE _controller_mode:=$CONTROLLER_MODE"
  if [[ "${LIVE_RVIZ,,}" == "true" ]]; then
    new_window live_rviz "$WS" \
      "$WAIT_ROS; until rostopic list | grep -qx /pro3/wheel_odom && rostopic list | grep -qx /pro3/rlscan; do sleep 1; done; rviz -d $TEST_DIR/live_rl.rviz"
  fi
fi
new_window monitor "$WS" \
  "$WAIT_ROS; until rostopic list | grep -qx /pro3/wheel_odom; do sleep 1; done; python3 $TEST_DIR/monitor.py _log_dir:=$RUNTIME_DIR/traces _goal_topic:=/lste/final_goal"

if [[ "${SCREEN_RECORD,,}" == "true" ]]; then
  if [[ "$CONTROLLER_MODE" == "ros_navigation" ]]; then
    VIDEO_SPEED="$NAVIGATION_MAX_LINEAR_SPEED"
  else
    VIDEO_SPEED="$SAPPO_SPEED"
  fi
  VIDEO_METHOD="${CONTROLLER_METHOD:-legacy}"
  if [[ "$CONTROLLER_MODE" != "ros_navigation" ]]; then
    VIDEO_METHOD="${VIDEO_METHOD}_${CONTROLLER_MODE}"
  fi
  VIDEO_VIEW="$(tr '[:upper:]' '[:lower:]' <<<"$SCREEN_RECORD_WINDOW" | sed 's/[^a-z0-9_-]/_/g')"
  VIDEO_FILE="$VIDEO_DIR/${RUN_TIMESTAMP}_${VIDEO_METHOD}_${VIDEO_VIEW}.mp4"
  # Keep this argument shell-safe because new_window embeds the command in a
  # tmux shell string. Parentheses and spaces are not safe unquoted here.
  VIDEO_LABEL="method=${VIDEO_METHOD}_start=${INITIAL_X},${INITIAL_Y}_goal=${GOAL_X},${GOAL_Y}_speed=${VIDEO_SPEED}mps"
  lifecycle_log "screen_record_scheduled output=$VIDEO_FILE window=$SCREEN_RECORD_WINDOW delay_seconds=$SCREEN_RECORD_START_DELAY_SECONDS duration_seconds=$SCREEN_RECORD_DURATION_SECONDS fps=$SCREEN_RECORD_FPS"
  new_window screen_record "$WS" \
    "sleep $SCREEN_RECORD_START_DELAY_SECONDS; $TEST_DIR/record_desktop.sh --output $VIDEO_FILE --duration $SCREEN_RECORD_DURATION_SECONDS --fps $SCREEN_RECORD_FPS --label $VIDEO_LABEL --window-name $SCREEN_RECORD_WINDOW"
fi

echo "[rl-fixed-goal-test] Started session '$SESSION' detached."
echo "  robot: ($INITIAL_X, $INITIAL_Y, $INITIAL_Z), yaw=$INITIAL_YAW"
echo "  goal:  ($GOAL_X, $GOAL_Y)"
echo "  controller: ${CONTROLLER_METHOD:-legacy} -> $CONTROLLER_MODE"
if [[ "$CONTROLLER_MODE" == "ros_navigation" ]]; then
  echo "  local planner: $LOCAL_PLANNER_PLUGIN, max speed: ${NAVIGATION_MAX_LINEAR_SPEED} m/s"
fi
echo "  ROS master: $ROS_URI, Gazebo master: $GAZEBO_URI"
echo "  monitor: tmux capture-pane -pJ -t $SESSION:monitor -S -80"
echo "  logs: $LOG_DIR"
if [[ "${SCREEN_RECORD,,}" == "true" ]]; then
  echo "  video: $VIDEO_FILE"
fi
lifecycle_log "run_started"
