#!/usr/bin/env bash
set -euo pipefail
export PYTHONDONTWRITEBYTECODE=1

# Public entry point for the business-node tmux session. Configuration,
# session ownership, and individual node groups deliberately live in focused
# source files under scripts/lifecycle/nodes/.
if [[ -z "${WS:-}" ]]; then
  WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
fi
LSTE_WS="$WS"
ENV_SESSION=lste-env
SESSION=lste
NODES_DIR="$WS/scripts/lifecycle/nodes"

source "$WS/scripts/config/pipeline_env.sh"

if [[ "${LSTE_LIFECYCLE_LOCK_HELD:-0}" != "1" ]]; then
  if ! command -v flock >/dev/null 2>&1; then
    echo "[error] flock is required to serialize LSTE lifecycle commands." >&2
    exit 1
  fi
  mkdir -p "$WS/runtime/lifecycle"
  exec 9>"$WS/runtime/lifecycle/lifecycle.lock"
  if ! flock -n 9; then
    echo "[error] Another LSTE lifecycle command is already starting or stopping the system." >&2
    exit 75
  fi
  export LSTE_LIFECYCLE_LOCK_HELD=1
fi

# Do not let long-lived tmux windows inherit the lifecycle file descriptor.
tmux() { command tmux "$@" 9>&-; }

PIPELINE_CONFIG=${PIPELINE_CONFIG:-$WS/scripts/config/pipeline_defaults.yaml}
if [[ -f "$PIPELINE_CONFIG" ]]; then
  eval "$("$WS/scripts/config/load_pipeline_config.sh" "$PIPELINE_CONFIG")"
fi

_resolve() {
  local value="$1"
  if [[ -z "$value" || "$value" == http://* || "$value" == https://* || "$value" == /* ]]; then
    echo "$value"
  else
    echo "$WS/$value"
  fi
}

# Provenance is resolved once before tmux panes begin. Navigation metrics uses
# it to make two runs comparable without depending on shell history.
WORLD=$(_resolve "${WORLD:-${CFG_WORLD:-worlds/place1.world}}")
NAVIGATION_LOG_DIR=${NAVIGATION_LOG_DIR:-${CFG_NAVIGATION_LOG_DIR:-$WS/runtime/navigation/logs}}
NAVIGATION_RUN_DIRECTORY=${NAVIGATION_RUN_DIRECTORY:-${CFG_NAVIGATION_RUN_DIRECTORY:-}}
NAVIGATION_RUN_TIMESTAMP=${NAVIGATION_RUN_TIMESTAMP:-${CFG_NAVIGATION_RUN_TIMESTAMP:-}}
GIT_REVISION=$(git -C "$WS" rev-parse HEAD 2>/dev/null || echo unknown)
if git -C "$WS" diff --quiet && git -C "$WS" diff --cached --quiet; then
  GIT_DIRTY=false
else
  GIT_DIRTY=true
fi
if [[ -f "$PIPELINE_CONFIG" ]]; then
  PIPELINE_CONFIG_SHA256=$(sha256sum "$PIPELINE_CONFIG" | awk '{print $1}')
else
  PIPELINE_CONFIG_SHA256=missing
fi

# Environment values override YAML; each config module provides its own
# fallback so focused experiment overlays remain valid.
source "$NODES_DIR/config/mission.sh"
source "$NODES_DIR/config/exploration.sh"
source "$NODES_DIR/config/detector.sh"
source "$NODES_DIR/config/controllers.sh"

source "$NODES_DIR/session.sh"
source "$NODES_DIR/windows/mission_perception.sh"
source "$NODES_DIR/windows/exploration.sh"
source "$NODES_DIR/windows/teb_navigation.sh"
source "$NODES_DIR/windows/observability.sh"

assert_node_prerequisites
remove_previous_node_session
create_pro3_window
start_mission_and_perception_windows
start_exploration_windows
start_teb_navigation_windows
start_observability_and_controller
show_node_summary

if [[ -t 0 && "${LSTE_NO_ATTACH:-0}" != "1" ]]; then
  tmux attach -t "$SESSION"
fi
