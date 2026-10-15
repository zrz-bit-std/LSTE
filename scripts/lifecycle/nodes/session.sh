#!/usr/bin/env bash
# tmux session lifecycle for LSTE business nodes.
#
# Requires WS, SESSION, ENV_SESSION, and the local tmux() wrapper from the
# public launcher. It deliberately contains no node-specific ROS command.

assert_node_prerequisites() {
  if ! command -v tmux >/dev/null 2>&1; then
    echo "[error] tmux is not installed" >&2
    exit 1
  fi
  if ! tmux has-session -t "=$ENV_SESSION" 2>/dev/null; then
    echo "[error] Environment session '$ENV_SESSION' is missing; run scripts/lifecycle/run_env_tmux.sh first." >&2
    exit 1
  fi
  if tmux has-session -t "=sappo-lste" 2>/dev/null; then
    echo "[error] Standalone SA-PPO session 'sappo-lste' is still running; stop it before starting LSTE nodes." >&2
    exit 1
  fi
  echo "[nodes] Detected live $ENV_SESSION session"

  echo "[nodes] Waiting for Gazebo spawn service..."
  local gazebo_ready=0
  for _ in $(seq 1 120); do
    if rosservice list 2>/dev/null | grep -qx /gazebo/spawn_urdf_model; then
      gazebo_ready=1
      break
    fi
    sleep 1
  done
  if [[ "$gazebo_ready" != "1" ]]; then
    echo "[error] Gazebo spawn service did not become ready; leave lste-env running and retry." >&2
    exit 1
  fi
}

remove_previous_node_session() {
  if tmux has-session -t "=$SESSION" 2>/dev/null; then
    echo "[nodes] Rebuilding existing '$SESSION' session..."
    tmux kill-session -t "=$SESSION"
  fi

  # A tmux pane may outlive its session briefly. Bound every unregister call
  # so hot restart does not become proportional to stale ROS registrations.
  local stale_nodes=()
  local node
  while IFS= read -r node; do
    [[ -n "$node" ]] && stale_nodes+=("$node")
  done < <(
    rosnode list 2>/dev/null | grep -E '^/(lste_|move_base$|rviz_lste_det_vis$|pro3/)' || true
  )
  if (( ${#stale_nodes[@]} > 0 )); then
    echo "[nodes] Requesting stale ROS nodes to exit: ${stale_nodes[*]}"
    for node in "${stale_nodes[@]}"; do
      (timeout 1 rosnode kill "$node" >/dev/null 2>&1 || true) &
    done
    wait || true
  fi

  for _ in $(seq 1 30); do
    if ! rosnode list 2>/dev/null | grep -Eq '^/(lste_|move_base$|rviz_lste_det_vis$|pro3/)'; then
      break
    fi
    sleep 0.2
  done
  if rosnode list 2>/dev/null | grep -Eq '^/(lste_|move_base$|rviz_lste_det_vis$|pro3/)'; then
    echo "[nodes] Cleaning stale ROS node registrations..." >&2
    timeout -k 1 3 rosnode cleanup </dev/null >/dev/null 2>&1 || true
    for _ in $(seq 1 20); do
      if ! rosnode list 2>/dev/null | grep -Eq '^/(lste_|move_base$|rviz_lste_det_vis$|pro3/)'; then
        break
      fi
      sleep 0.2
    done
  fi
  if rosnode list 2>/dev/null | grep -Eq '^/(lste_|move_base$|rviz_lste_det_vis$|pro3/)'; then
    echo "[nodes] Warning: a stale ROS registration remains; proceeding after the bounded cleanup." >&2
  fi
  echo "[nodes] Previous node session cleanup complete"

  # lste-env keeps roscore across a node hot restart. Clear a prior cache
  # decision so the new prompt node owns it again.
  bash -lc "WS=\"$WS\"; source \"$WS/scripts/config/pipeline_env.sh\"; rosparam delete /lste_prompt_node/needs_vllm >/dev/null 2>&1 || true"
}

create_pro3_window() {
  echo "[nodes] Creating '$SESSION' session..."
  tmux new-session -d -s "$SESSION" -c "$WS" -n "pro3" \
    "bash -lc 'WS=\"$WS\"; source \"$WS/scripts/config/pipeline_env.sh\"; set -e; \\
until rostopic list >/dev/null 2>&1; do echo \"waiting for roscore...\"; sleep 1; done; \\
if rosservice call /gazebo/get_world_properties 2>/dev/null | grep -q \"pro3\"; then \\
  echo \"[pro3] Existing Gazebo model found; starting TF support nodes.\"; \\
  roslaunch lste_core spawn_pro3.launch spawn_model:=false start_support_nodes:=true \\
    x:=$PRO3_SPAWN_X y:=$PRO3_SPAWN_Y z:=$PRO3_SPAWN_Z yaw:=$PRO3_SPAWN_YAW benchmark_contact_sensor:=$BENCHMARK_CONTACT_SENSOR; \\
else \\
  roslaunch lste_core spawn_pro3.launch spawn_model:=true start_support_nodes:=true \\
    x:=$PRO3_SPAWN_X y:=$PRO3_SPAWN_Y z:=$PRO3_SPAWN_Z yaw:=$PRO3_SPAWN_YAW benchmark_contact_sensor:=$BENCHMARK_CONTACT_SENSOR; \\
fi; \\
echo; echo \"[EXIT] pro3 spawn\"; exec bash'"
  tmux set-option -t "=$SESSION:" remain-on-exit on

  echo "[nodes] Waiting for Pro3 odometry..."
  local pro3_ready=0
  for _ in $(seq 1 120); do
    if rostopic list 2>/dev/null | grep -qx /pro3/wheel_odom; then
      pro3_ready=1
      break
    fi
    sleep 1
  done
  if [[ "$pro3_ready" != "1" ]]; then
    echo "[error] Pro3 did not publish /pro3/wheel_odom; stopping incomplete node session." >&2
    tmux capture-pane -pJ -t "=$SESSION:pro3" -S -80 >&2 || true
    tmux kill-session -t "=$SESSION" || true
    exit 1
  fi
}

tmux_new_window() {
  local index="$1"
  local dir="$2"
  local title="$3"
  local cmd="$4"
  tmux new-window -t "=$SESSION:$index" -n "$title" -c "$dir" \
    "bash -lc 'WS=\"$WS\"; source \"$WS/scripts/config/pipeline_env.sh\"; set -e; $cmd'; echo; echo '[EXIT] $title'; exec bash"
  echo "[nodes] Started window $index ($title)"
}

WAIT_ROSCORE='until rostopic list >/dev/null 2>&1; do echo \"waiting for roscore...\"; sleep 1; done'
