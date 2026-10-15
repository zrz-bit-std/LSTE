#!/usr/bin/env bash
set -euo pipefail

WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
SESSION="lste"
TELEOP_SESSION="lste-teleop"
MODE="${1:-}"
# Hot controller switches use the same YAML values as a full pipeline start.
# Explicit environment overrides still win, while the fallback literals keep
# this script usable when the config file is unavailable.
source "$WS/scripts/config/pipeline_env.sh"
PIPELINE_CONFIG="${PIPELINE_CONFIG:-$WS/scripts/config/pipeline_defaults.yaml}"
if [[ -f "$PIPELINE_CONFIG" ]]; then
  eval "$("$WS/scripts/config/load_pipeline_config.sh" "$PIPELINE_CONFIG")"
fi
# Keep one canonical source of controller defaults. Environment variables set
# by a hot switch still take precedence over YAML-derived CFG_* values.
source "$WS/scripts/lifecycle/nodes/config/controllers.sh"
RL_DIR="$WS/rl_navigation"
RUNTIME_DIR="$WS/runtime/sappo"
POLICY_DIR="$RL_DIR/policy"

if [[ "$MODE" != "sappo" && "$MODE" != "teleop" && "$MODE" != "teb" ]]; then
  echo "Usage: $0 {sappo|teleop|teb}" >&2
  exit 2
fi

if ! tmux has-session -t "=$SESSION" 2>/dev/null; then
  echo "[error] LSTE session '$SESSION' is not running." >&2
  exit 1
fi

ros_node_exists() {
  bash -lc "source \"$WS/scripts/config/pipeline_env.sh\"; rosnode list 2>/dev/null | grep -qx \"$1\""
}

wait_node_ready() {
  local node="$1"
  local attempts="${2:-50}"
  local i
  for i in $(seq 1 "$attempts"); do
    if ros_node_exists "$node"; then
      return 0
    fi
    sleep 0.1
  done
  echo "[error] ROS node did not start: $node" >&2
  return 1
}

wait_sappo_stable() {
  local i
  # A process can register its ROS name and then fail while loading the policy.
  # Do not let the lifecycle script report success until it has survived that
  # short startup window and the tmux pane is still alive.
  for i in $(seq 1 20); do
    if ! window_alive sappo || ! ros_node_exists /StageEnv_0; then
      echo "[error] SA-PPO exited during startup; inspect: tmux capture-pane -pt $SESSION:sappo" >&2
      return 1
    fi
    sleep 0.1
  done
}

wait_teb_ready() {
  local i
  for i in $(seq 1 100); do
    if ros_node_exists /move_base && ros_node_exists /lste_teb_goal_bridge; then
      return 0
    fi
    sleep 0.1
  done
  echo "[error] TEB navigation stack did not start; inspect: tmux capture-pane -pt $SESSION:teb_nav" >&2
  return 1
}

window_alive() {
  tmux list-panes -t "=$SESSION:$1" -F '#{pane_dead}' 2>/dev/null | grep -qx 0
}

start_rl_scan() {
  if window_alive rl_scan; then
    return
  fi
  tmux kill-window -t "=$SESSION:rl_scan" 2>/dev/null || true
  tmux new-window -t "=$SESSION:" -n rl_scan -c "$WS" \
    "bash -lc 'source \"$WS/scripts/config/pipeline_env.sh\"; \
roslaunch pointcloud_to_laserscan lste_pro3_to_scan.launch; exec bash'"
}

start_sappo() {
  if window_alive sappo && ros_node_exists /StageEnv_0; then
    return
  fi
  if [[ ! -x "$SAPPO_PYTHON" || ! -f "$RL_DIR/sappo_pure.py" || \
        ! -f "$POLICY_DIR/sa_peppo_1650.pth" ]]; then
    echo "[error] SA-PPO runtime is incomplete." >&2
    exit 1
  fi
  local python_site sappo_entry sappo_command
  local -a sappo_arguments=(
    "_linear_speed_scale:=$SAPPO_SPEED"
    "_goal_size:=$SAPPO_GOAL_TOLERANCE"
    "_goal_topic:=/lste/final_goal"
    "_cmd_vel_topic:=/lste/cmd_vel/sappo"
    "_wait_for_goal:=true"
    "_allow_intermediate_goals:=true"
  )
  case "$SAPPO_CONTROLLER_MODE" in
    policy_only)
      sappo_entry="$RL_DIR/sappo_pure.py"
      ;;
    stabilized_recovery|rl_dwa_guard|rl_mppi_guard|rl_grid_guard)
      sappo_entry="$WS/scripts/tests/rl_fixed_goal/sappo_test.py"
      sappo_arguments+=(
        "_controller_mode:=$SAPPO_CONTROLLER_MODE"
        "_in_place_turn_angular_threshold:=$SAPPO_IN_PLACE_TURN_ANGULAR_THRESHOLD"
        "_boundary_turn_lock_time:=$SAPPO_BOUNDARY_TURN_LOCK_TIME"
        "_boundary_turn_max_duration:=$SAPPO_BOUNDARY_TURN_MAX_DURATION"
        "_boundary_turn_max_angle_deg:=$SAPPO_BOUNDARY_TURN_MAX_ANGLE_DEG"
        "_boundary_turn_max_attempts:=$SAPPO_BOUNDARY_TURN_MAX_ATTEMPTS"
        "_boundary_reentry_cooldown:=$SAPPO_BOUNDARY_REENTRY_COOLDOWN"
        "_boundary_progress_timeout:=$SAPPO_BOUNDARY_PROGRESS_TIMEOUT"
        "_boundary_progress_margin:=$SAPPO_BOUNDARY_PROGRESS_MARGIN"
        "_boundary_wall_distance:=$SAPPO_BOUNDARY_WALL_DISTANCE"
        "_boundary_wall_kp:=$SAPPO_BOUNDARY_WALL_KP"
        "_boundary_wall_heading_kp:=$SAPPO_BOUNDARY_WALL_HEADING_KP"
        "_boundary_wall_max_linear:=$SAPPO_BOUNDARY_WALL_MAX_LINEAR"
        "_boundary_wall_max_range:=$SAPPO_BOUNDARY_WALL_MAX_RANGE"
        "_boundary_wall_filter_alpha:=$SAPPO_BOUNDARY_WALL_FILTER_ALPHA"
        "_boundary_wall_angular_step:=$SAPPO_BOUNDARY_WALL_ANGULAR_STEP"
        "_boundary_wall_angular_deadband:=$SAPPO_BOUNDARY_WALL_ANGULAR_DEADBAND"
        "_boundary_goal_cancel_angle_deg:=$SAPPO_BOUNDARY_GOAL_CANCEL_ANGLE_DEG"
        "_boundary_goal_cancel_distance:=$SAPPO_BOUNDARY_GOAL_CANCEL_DISTANCE"
        "_waypoint_hold_radius:=$SAPPO_WAYPOINT_HOLD_RADIUS"
        "_waypoint_switch_distance:=$SAPPO_WAYPOINT_SWITCH_DISTANCE"
        "_grid_obstacle_radius:=$SAPPO_GRID_OBSTACLE_RADIUS"
        "_grid_edge_clearance:=$SAPPO_GRID_EDGE_CLEARANCE"
        "_reorient_obstacle_clearance:=$SAPPO_REORIENT_OBSTACLE_CLEARANCE"
        "_max_linear_action_step:=$SAPPO_MAX_LINEAR_ACTION_STEP"
        "_max_linear_action_decel:=$SAPPO_MAX_LINEAR_ACTION_DECEL"
        "_max_angular_action_step:=$SAPPO_MAX_ANGULAR_ACTION_STEP"
        "_angular_sign_switch_threshold:=$SAPPO_ANGULAR_SIGN_SWITCH_THRESHOLD"
        "_intermediate_goal_size:=$SAPPO_INTERMEDIATE_GOAL_TOLERANCE"
      )
      ;;
    *)
      echo "[error] invalid SAPPO_CONTROLLER_MODE: '$SAPPO_CONTROLLER_MODE'" >&2
      exit 2
      ;;
  esac
  if [[ ! -f "$sappo_entry" ]]; then
    echo "[error] SA-PPO controller entrypoint is missing: $sappo_entry" >&2
    exit 1
  fi
  python_site="$($SAPPO_PYTHON -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
  mkdir -p "$RUNTIME_DIR"
  ln -sfn "$POLICY_DIR" "$RUNTIME_DIR/policy"
  printf -v sappo_command '%q ' "$SAPPO_PYTHON" "$sappo_entry" "${sappo_arguments[@]}"
  tmux kill-window -t "=$SESSION:sappo" 2>/dev/null || true
  tmux new-window -t "=$SESSION:" -n sappo -c "$RUNTIME_DIR" \
    "bash -lc 'source \"$WS/scripts/config/pipeline_env.sh\"; \
until rostopic list | grep -qx /pro3/wheel_odom && rostopic list | grep -qx /pro3/rlscan; do sleep 1; done; \
PYTHONPATH=\"$python_site:$WS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages\" \
$sappo_command; exec bash'"
  # Point-cloud conversion and the policy process can take tens of seconds to
  # register after Gazebo has spawned the robot. Keep startup bounded, but do
  # not mistake normal initialization for a controller crash.
  wait_node_ready /StageEnv_0 600
  wait_sappo_stable
}

stop_sappo() {
  # TEB and teleop do not need a live policy process. Removing it when the
  # controller is deselected releases its CPU/GPU allocation and prevents a
  # background /cmd_vel publisher from being mistaken for the active path.
  if ros_node_exists /StageEnv_0; then
    rosnode kill /StageEnv_0 >/dev/null 2>&1 || true
  fi
  tmux kill-window -t "=$SESSION:sappo" 2>/dev/null || true
}

start_teleop() {
  if tmux has-session -t "=$TELEOP_SESSION" 2>/dev/null && \
     ros_node_exists /teleop_twist_keyboard_reset; then
    return
  fi
  local command
  command="bash -lc 'source \"$WS/scripts/config/pipeline_env.sh\"; \
python3 \"$WS/scripts/tools/teleop_with_reset.py\" cmd_vel:=/lste/cmd_vel/teleop; exec bash'"
  if tmux has-session -t "=$TELEOP_SESSION" 2>/dev/null; then
    tmux respawn-pane -k -t "=$TELEOP_SESSION:teleop" -c "$WS" "$command"
  else
    tmux new-session -d -s "$TELEOP_SESSION" -n teleop -c "$WS" "$command"
    tmux set-option -t "=$TELEOP_SESSION:" remain-on-exit on
  fi
  wait_node_ready /teleop_twist_keyboard_reset 200
}

select_controller() {
  source "$WS/scripts/config/pipeline_env.sh"
  wait_node_ready /lste_controller_switch
  python3 - "$MODE" <<'PY'
import sys
import time

import rospy
from std_msgs.msg import String

mode = sys.argv[1]
rospy.init_node("lste_controller_select_cli", anonymous=True, disable_signals=True)
publisher = rospy.Publisher("/lste/controller_select", String, queue_size=1)
seen_mode = {"value": None}

def on_mode(message):
    seen_mode["value"] = message.data.strip().lower()

rospy.Subscriber("/lste/controller_mode", String, on_mode, queue_size=1)
deadline = time.time() + 10.0
while publisher.get_num_connections() == 0 and time.time() < deadline:
    time.sleep(0.05)
if publisher.get_num_connections() == 0:
    raise SystemExit("controller switch node is not available")
while time.time() < deadline:
    publisher.publish(String(data=mode))
    if seen_mode["value"] == mode:
        break
    time.sleep(0.10)
if seen_mode["value"] != mode:
    raise SystemExit(
        "controller switch did not acknowledge requested mode %r (last=%r)"
        % (mode, seen_mode["value"])
    )
PY
}

start_rl_scan
if [[ "$MODE" == "sappo" || "${SAPPO_BACKGROUND_ENABLED,,}" == "true" || "$SAPPO_BACKGROUND_ENABLED" == "1" ]]; then
  start_sappo
else
  stop_sappo
fi
start_teleop
if [[ "$MODE" == "teb" ]]; then
  wait_teb_ready
fi
select_controller
if [[ "$MODE" == "sappo" || "${SAPPO_BACKGROUND_ENABLED,,}" == "true" || "$SAPPO_BACKGROUND_ENABLED" == "1" ]]; then
  echo "[controller] Selected $MODE; SA-PPO remains available for hot switching."
else
  echo "[controller] Selected $MODE; SA-PPO is lazy and not running."
fi
