#!/usr/bin/env bash
set -euo pipefail

# Shared environment setup for tmux pipeline windows.
# - Loads ROS Noetic
# - Loads this workspace overlay (devel)
# - Initializes conda (without requiring `conda init`)

# Auto-detect workspace root (can override with LSTE_WS env var)
if [ -z "${WS:-}" ]; then
  WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
fi

# Auto-detect the supporting workspace (robot model and Gazebo plugins).
# Keep an explicit LSTE_PRE_WS override for installations in other locations.
if [ -z "${LSTE_PRE_WS:-}" ]; then
  for candidate in "$(dirname "$WS")/pre_work" \
                   "$(dirname "$WS")/pre_ws" \
                   "$(dirname "$WS")/lste-previous/pre_ws"; do
    if [ -f "$candidate/devel/setup.bash" ] && [ -d "$candidate/devel/lib" ]; then
      LSTE_PRE_WS="$candidate"
      break
    fi
  done
fi
export LSTE_PRE_WS="${LSTE_PRE_WS:-}"
export LSTE_WS="$WS"

# Catkin's generated Python relays use the base interpreter's shebang, while
# the LSTE helper modules live in src/lste_core/scripts/utils.  Export the
# source script directory explicitly so every ROS Python node can resolve the
# shared ``utils`` package regardless of which conda interpreter launches it.
export PYTHONPATH="$WS/src/lste_core/scripts:$WS/devel/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"

# Keep startup deterministic: every model used by the simulation is expected to
# be installed locally. Gazebo Classic otherwise blocks on its retired public
# model database before the world can begin publishing /clock.
export GAZEBO_MODEL_DATABASE_URI="${GAZEBO_MODEL_DATABASE_URI:-file://$WS/worlds/.gazebo_model_database}"

if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi

if [ -n "$LSTE_PRE_WS" ] && [ -f "$LSTE_PRE_WS/devel/setup.bash" ]; then
  # pja (Pro3 URDF) lives in the supporting workspace on this machine.
  # Source it before this workspace so both ROS packages and Gazebo plugins
  # remain discoverable from tmux windows started in a fresh shell.
  # shellcheck disable=SC1090
  source "$LSTE_PRE_WS/devel/setup.bash"
fi

if [ -f "$WS/devel/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$WS/devel/setup.bash"
fi

if command -v conda >/dev/null 2>&1; then
  CONDA_BASE="$(conda info --base 2>/dev/null || true)"
  if [ -n "${CONDA_BASE:-}" ] && [ -f "$CONDA_BASE/etc/profile.d/conda.sh" ]; then
    # shellcheck disable=SC1090
    source "$CONDA_BASE/etc/profile.d/conda.sh"
  else
    # fallback: try the shell hook
    eval "$(conda shell.bash hook 2>/dev/null || true)"
  fi
  # fix conda libffi.so.8 shadowing system libffi.so.7 (needed by apt opencv/PyKDL)
  export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libffi.so.7${LD_PRELOAD:+:$LD_PRELOAD}
fi

# Gazebo Classic loads ROS camera support through libgazebo_ros_camera.so,
# which in turn links the system CameraPlugin.  The latter lives in Gazebo's
# plugin directory rather than in /opt/ros/noetic/lib.  Keep both directories
# visible to Gazebo and the dynamic linker; otherwise the sensor exists in
# Gazebo transport but never advertises its ROS image topic.
GAZEBO_SYSTEM_PLUGIN_DIR="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
if [ -d "$GAZEBO_SYSTEM_PLUGIN_DIR" ]; then
  case ":${GAZEBO_PLUGIN_PATH:-}:" in
    *":$GAZEBO_SYSTEM_PLUGIN_DIR:"*) ;;
    *) GAZEBO_PLUGIN_PATH="$GAZEBO_SYSTEM_PLUGIN_DIR${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}" ;;
  esac
  case ":${LD_LIBRARY_PATH:-}:" in
    *":$GAZEBO_SYSTEM_PLUGIN_DIR:"*) ;;
    *) LD_LIBRARY_PATH="$GAZEBO_SYSTEM_PLUGIN_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
  esac
fi
export GAZEBO_PLUGIN_PATH
export LD_LIBRARY_PATH

# Run a Gazebo command with a usable render context.  A remote shell often
# inherits DISPLAY=:0 without the desktop user's Xauthority; Gazebo then starts
# but disables CameraSensor rendering, leaving ROS image topics with no
# publisher.  Prefer a real display when it is reachable and fall back to a
# short-lived Xvfb owned by xvfb-run for reproducible headless runs.
lste_gazebo_exec() {
  if [ -n "${DISPLAY:-}" ] && command -v xdpyinfo >/dev/null 2>&1 \
      && xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
    "$@"
    return $?
  fi
  if ! command -v xvfb-run >/dev/null 2>&1; then
    echo "[gazebo] no usable DISPLAY and xvfb-run is unavailable; install xvfb" >&2
    return 127
  fi
  echo "[gazebo] DISPLAY=${DISPLAY:-unset} is unavailable; using Xvfb render context" >&2
  xvfb-run -a -s "-screen 0 1920x1080x24" "$@"
}

# 统一 ROS master URI，避免 lste-env 和 lste session 各连不同的 roscore
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311/}
