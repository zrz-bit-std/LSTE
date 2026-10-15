#!/usr/bin/env bash
set -euo pipefail

WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
if [[ "${LSTE_LIFECYCLE_LOCK_HELD:-0}" != "1" ]]; then
  if ! command -v flock >/dev/null 2>&1; then
    echo "[error] flock is required to serialize LSTE lifecycle commands." >&2
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

# Never pass the stop command's lock fd into a surviving tmux server.
tmux() { command tmux "$@" 9>&-; }

SESSIONS=(lste-teleop sappo-lste lste lste-env)

graceful_shutdown_metrics() {
  # The metrics node owns the failure ring and its on_shutdown hook writes the
  # final sample/run_stop record.  Killing the tmux session first can send a
  # SIGHUP before rospy gets that callback, leaving an active failure episode
  # open and making a manually stopped run impossible to audit.
  if ! command -v rosnode >/dev/null 2>&1; then
    return 0
  fi
  if ! rosnode list 2>/dev/null | grep -qx /lste_navigation_metrics; then
    return 0
  fi
  echo "[stop] Requesting a graceful shutdown of /lste_navigation_metrics"
  timeout -k 1 3 rosnode kill /lste_navigation_metrics >/dev/null 2>&1 || true
  for _ in $(seq 1 30); do
    if ! rosnode list 2>/dev/null | grep -qx /lste_navigation_metrics; then
      return 0
    fi
    sleep 0.1
  done
  echo "[stop] Warning: metrics node did not unregister within the bounded grace period" >&2
}

collect_residual_pids() {
  {
    pgrep -x gzclient || true
    pgrep -x gzserver || true
    pgrep -f '/opt/ros/noetic/bin/roscore' || true
    pgrep -f '/opt/ros/noetic/bin/rosmaster --core' || true
    # Killing a tmux session sends SIGHUP to its shell, but Python children
    # can outlive that shell.  Leaving one behind is especially dangerous for
    # the fixed ROS name /StageEnv_0: the next run then starts with a duplicate
    # controller and one of the two processes exits immediately.
    pgrep -f "$WS/rl_navigation/sappo_pure.py" || true
    pgrep -f "$WS/scripts/tests/rl_fixed_goal/sappo_test.py" || true
    pgrep -f "$WS/scripts/tools/teleop_with_reset.py" || true
    pgrep -f "$WS/scripts/lifecycle/switch_controller.sh" || true
  } | sort -nu
}

# Flush correlated failure evidence before killing the business-node session.
graceful_shutdown_metrics

for session in "${SESSIONS[@]}"; do
  if tmux has-session -t "=$session" 2>/dev/null; then
    echo "[stop] Stopping tmux session: $session"
    tmux kill-session -t "=$session"
  else
    echo "[stop] Session already stopped: $session"
  fi
done

remaining=()
for session in "${SESSIONS[@]}"; do
  if tmux has-session -t "=$session" 2>/dev/null; then
    remaining+=("$session")
  fi
done

if (( ${#remaining[@]} > 0 )); then
  echo "[error] Failed to stop sessions: ${remaining[*]}" >&2
  exit 1
fi

for _ in $(seq 1 30); do
  mapfile -t residual_pids < <(collect_residual_pids)
  if (( ${#residual_pids[@]} == 0 )); then
    break
  fi
  sleep 0.1
done

mapfile -t residual_pids < <(collect_residual_pids)
if (( ${#residual_pids[@]} > 0 )); then
  echo "[stop] Terminating residual ROS/Gazebo processes: ${residual_pids[*]}"
  kill -TERM "${residual_pids[@]}" 2>/dev/null || true
  sleep 1
fi

mapfile -t residual_pids < <(collect_residual_pids)
if (( ${#residual_pids[@]} > 0 )); then
  echo "[stop] Forcing residual processes to exit: ${residual_pids[*]}"
  kill -KILL "${residual_pids[@]}" 2>/dev/null || true
fi

echo "[stop] LSTE complete system is stopped."
