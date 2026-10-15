#!/usr/bin/env bash
set -euo pipefail

SESSION="${RL_FIXED_GOAL_SESSION:-rl-fixed-goal-test}"
ROS_PORT="${RL_FIXED_GOAL_ROS_PORT:-11312}"
GAZEBO_PORT="${RL_FIXED_GOAL_GAZEBO_PORT:-11346}"
WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
CURRENT_LOG_LINK="$WS/runtime/rl_fixed_goal_test/current_log"

lifecycle_log() {
  local current_dir lifecycle_file
  if [[ -L "$CURRENT_LOG_LINK" ]]; then
    current_dir="$(readlink -f "$CURRENT_LOG_LINK")"
    lifecycle_file="$current_dir/$(basename "$current_dir")_lifecycle.log"
  fi
  if [[ -n "${lifecycle_file:-}" && -f "$lifecycle_file" ]]; then
    printf '%s [INFO] [launcher] event=%s\n' \
      "$(date '+%Y-%m-%d %H:%M:%S.%3N')" "$1" >> "$lifecycle_file"
  fi
}

lifecycle_log "stop_requested"

# Gazebo can outlive a tmux pane after a bare SIGHUP. Ask every node on this
# test-only master to shut down first, so the next isolated run owns its ports.
ROS_MASTER_URI="http://localhost:${ROS_PORT}" rosnode kill -a >/dev/null 2>&1 || true
sleep 2

if tmux has-session -t "=$SESSION" 2>/dev/null; then
  tmux kill-session -t "=$SESSION"
  echo "[rl-fixed-goal-test] Stopped session '$SESSION'."
else
  echo "[rl-fixed-goal-test] Session '$SESSION' is not running."
fi

# A server that lost its ROS master cannot answer rosnode shutdown. Match its
# inherited Gazebo master URI before signalling it, preserving every other
# Gazebo instance on this machine.
for pid in $(pgrep -x gzserver 2>/dev/null || true); do
  environment="$(tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null || true)"
  if grep -qx "GAZEBO_MASTER_URI=http://localhost:${GAZEBO_PORT}" <<<"$environment"; then
    kill -TERM "$pid" 2>/dev/null || true
  fi
done
sleep 2
for pid in $(pgrep -x gzserver 2>/dev/null || true); do
  environment="$(tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null || true)"
  if grep -qx "GAZEBO_MASTER_URI=http://localhost:${GAZEBO_PORT}" <<<"$environment"; then
    kill -KILL "$pid" 2>/dev/null || true
  fi
done

lifecycle_log "stop_completed"
