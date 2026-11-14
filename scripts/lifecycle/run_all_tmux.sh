#!/usr/bin/env bash
set -euo pipefail

WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
export LSTE_NO_ATTACH=1
PIPELINE_CONFIG="${PIPELINE_CONFIG:-$WS/scripts/config/pipeline_defaults.yaml}"
if [[ -f "$PIPELINE_CONFIG" ]]; then
  eval "$(LSTE_WS="$WS" "$WS/scripts/config/load_pipeline_config.sh" "$PIPELINE_CONFIG")"
fi

# A complete startup spans the environment and node sessions.  Serialize that
# sequence so two quick `runall` invocations cannot both observe a missing
# session and race while creating the same tmux session.
if [[ "${LSTE_LIFECYCLE_LOCK_HELD:-0}" != "1" ]]; then
  if ! command -v flock >/dev/null 2>&1; then
    echo "[error] flock is required to serialize LSTE startup." >&2
    exit 1
  fi
  LOCK_DIR="$WS/runtime/lifecycle"
  mkdir -p "$LOCK_DIR"
  exec 9>"$LOCK_DIR/lifecycle.lock"
  if ! flock -n 9; then
    echo "[error] Another LSTE lifecycle command is already starting or stopping the system." >&2
    exit 75
  fi
  export LSTE_LIFECYCLE_LOCK_HELD=1
fi

# Do not let the tmux server inherit the lifecycle lock descriptor.  A tmux
# server outlives this shell; inheriting fd 9 would keep the flock held until
# the entire session is killed and would make a later `stopall` fail.
tmux() { command tmux "$@" 9>&-; }

if tmux has-session -t "=sappo-lste" 2>/dev/null; then
  echo "[all] Stopping the conflicting standalone SA-PPO session."
  tmux kill-session -t "=sappo-lste"
fi

"$WS/scripts/lifecycle/run_env_tmux.sh"

# `rosnode list` can segfault under a freshly launched ROS graph (observed as
# a core dump on batch runs, which then failed the 300 s readiness gate).
# Take a single snapshot per readiness poll and retry a transient crash, so a
# segfault cannot sink the whole launch and the per-node greps stop spawning a
# dozen `rosnode list` processes every second.
rosnode_list_snapshot() {
  local nodes attempt
  for attempt in 1 2 3 4 5; do
    nodes="$(rosnode list 2>/dev/null || true)"
    if [[ -n "$nodes" ]]; then
      printf '%s\n' "$nodes"
      return 0
    fi
    sleep 1
  done
  printf '%s\n' "$nodes"
}

nodes_session_ready() {
  local window node global_frontier_enabled
  global_frontier_enabled="${GLOBAL_FRONTIER_ENABLED:-${CFG_GLOBAL_FRONTIER_ENABLED:-true}}"
  for window in goal teb_nav controller_switch cmd_vel_mux health; do
    tmux list-windows -t "=lste" -F '#{window_name}' 2>/dev/null | grep -qx "$window" || return 1
  done
  local node_nodes
  node_nodes="$(rosnode_list_snapshot)"
  for node in /lste_goal_manager /lste_teb_goal_bridge /lste_controller_switch /lste_cmd_vel_mux /move_base; do
    grep -qx "$node" <<< "$node_nodes" || return 1
  done
  if [[ "${global_frontier_enabled,,}" == "true" \
      || "$global_frontier_enabled" == "1" ]]; then
    rosnode list 2>/dev/null | grep -qx /lste_global_frontier || return 1
  fi
}

all_nodes_ready() {
  local node global_frontier_enabled
  global_frontier_enabled="${GLOBAL_FRONTIER_ENABLED:-${CFG_GLOBAL_FRONTIER_ENABLED:-true}}"
  local required_nodes=(
    /lste_task_node /lste_prompt_node /lste_det_node \
    /lste_score_node /lste_state_node /lste_det_vis_node /oc_srfc_proj \
    /lste_teb_goal_bridge /lste_teb_turn_supervisor \
    /lste_controller_switch /lste_cmd_vel_mux /lste_health_audit \
    /lste_navigation_metrics /move_base /teleop_twist_keyboard_reset
  )
  # Diagnostic target-entry runs keep SLAM and the controller but deliberately
  # omit the exploration executive. Do not wait 300 s for a node that the
  # selected profile explicitly disabled.
  if [[ "${global_frontier_enabled,,}" == "true" \
      || "$global_frontier_enabled" == "1" ]]; then
    required_nodes+=(/lste_global_frontier)
  fi
  local node_nodes
  node_nodes="$(rosnode_list_snapshot)"
  for node in "${required_nodes[@]}"; do
    grep -qx "$node" <<< "$node_nodes" || return 1
  done
}

wait_for_all_nodes() {
  local attempt
  for attempt in $(seq 1 300); do
    if all_nodes_ready; then
      echo "[all] All benchmark nodes are registered."
      return 0
    fi
    sleep 1
  done
  echo "[error] Required LSTE nodes did not become ready within 300 seconds." >&2
  echo "[error] Registered nodes:" >&2
  rosnode list 2>/dev/null >&2 || true
  return 1
}

if ! tmux has-session -t "=lste" 2>/dev/null; then
  source "$WS/scripts/config/pipeline_env.sh"
  echo "[all] Waiting for Gazebo services..."
  for _ in $(seq 1 120); do
    if rosservice list 2>/dev/null | grep -qx /gazebo/spawn_urdf_model; then
      break
    fi
    sleep 1
  done
  if ! rosservice list 2>/dev/null | grep -qx /gazebo/spawn_urdf_model; then
    echo "[error] Gazebo spawn service did not become ready." >&2
    exit 1
  fi
  "$WS/scripts/lifecycle/run_nodes_tmux.sh"
  wait_for_all_nodes
elif nodes_session_ready; then
  echo "[all] Reusing the existing 'lste' session."
  source "$WS/scripts/config/pipeline_env.sh"
  if rosnode list 2>/dev/null | grep -qx /StageEnv_0; then
    "$WS/scripts/lifecycle/switch_controller.sh" teb
  elif rosnode list 2>/dev/null | grep -qx /teleop_twist_keyboard_reset; then
    "$WS/scripts/lifecycle/switch_controller.sh" teleop
  else
    "$WS/scripts/lifecycle/switch_controller.sh" teb
  fi
  wait_for_all_nodes
else
  echo "[all] Existing 'lste' session is incomplete; rebuilding the node session."
  tmux kill-session -t "=lste"
  source "$WS/scripts/config/pipeline_env.sh"
  "$WS/scripts/lifecycle/run_nodes_tmux.sh"
  wait_for_all_nodes
fi

for session in lste-env lste lste-teleop; do
  if ! tmux has-session -t "=$session" 2>/dev/null; then
    echo "[error] Required tmux session was not created: $session" >&2
    exit 1
  fi
done

echo ""
echo "============================================"
echo "  LSTE complete system is ready"
echo "  Environment:  tmux attach -t lste-env"
echo "  Brain/TEB:    tmux attach -t lste"
echo "  Keyboard:     teleop"
echo "============================================"
