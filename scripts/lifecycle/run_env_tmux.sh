#!/usr/bin/env bash
set -euo pipefail
export PYTHONDONTWRITEBYTECODE=1

# ============================================
# 环境启动脚本：只启动 roscore + Gazebo 仿真
# 仿真加载慢且几乎不变，单独跑，可以长期保持不动
# ============================================

# ---- 路径解析（与 run_nodes_tmux.sh 完全一致） ----
if [ -z "${WS:-}" ]; then
  WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
fi
LSTE_WS="$WS"
SESSION=lste-env

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

# The tmux server is long-lived, so it must not retain the startup lock fd.
tmux() { command tmux "$@" 9>&-; }
PIPELINE_CONFIG=${PIPELINE_CONFIG:-$WS/scripts/config/pipeline_defaults.yaml}
if [[ -f "$PIPELINE_CONFIG" ]]; then
  eval "$("$WS/scripts/config/load_pipeline_config.sh" "$PIPELINE_CONFIG")"
fi
_resolve() {
  local val="$1"
  if [[ -z "$val" || "$val" == http://* || "$val" == https://* || "$val" == /* ]]; then
    echo "$val"
  else
    echo "$WS/$val"
  fi
}

WORLD=${WORLD:-$(_resolve "${CFG_WORLD:-worlds/place1.world}")}
GAZEBO_RANDOM_SEED=${GAZEBO_RANDOM_SEED:-${CFG_GAZEBO_RANDOM_SEED:-}}
GAZEBO_EXTRA_ARGS=""
if [[ -n "$GAZEBO_RANDOM_SEED" ]]; then
  if [[ ! "$GAZEBO_RANDOM_SEED" =~ ^[0-9]+$ ]]; then
    echo "[error] GAZEBO_RANDOM_SEED must be a non-negative integer." >&2
    exit 1
  fi
  GAZEBO_EXTRA_ARGS="--seed $GAZEBO_RANDOM_SEED"
fi
GUI=true
GUI_PLUGIN="$WS/devel/lib/libgazebo_click_point.so"

if [[ ! -f "$WORLD" ]]; then
  echo "[error] WORLD file not found: $WORLD" >&2
  exit 1
fi

if [[ ! -f "$GUI_PLUGIN" ]]; then
  echo "[error] Gazebo click plugin not found: $GUI_PLUGIN" >&2
  echo "        Run: source /opt/ros/noetic/setup.bash && catkin_make --pkg gazebo_click_point" >&2
  exit 1
fi

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux 未安装，请先安装 tmux。" >&2
  exit 1
fi

gazebo_gui_ready() {
  # The bracket prevents pgrep from matching its own command line.
  pgrep -f '[g]zclient.*--gui-client-plugin .*libgazebo_click_point\.so' >/dev/null
}

start_gazebo_gui() {
  local window_name
  for window_name in gui gazebo_gui; do
    if tmux list-windows -t "=$SESSION" -F '#{window_name}' 2>/dev/null | grep -qx "$window_name"; then
      tmux kill-window -t "$SESSION:$window_name"
    fi
  done
  tmux new-window -d -t "=$SESSION" -n "gazebo_gui" -c "$WS" \
    "bash -lc 'source \"$WS/scripts/config/pipeline_env.sh\"; \
until rostopic list >/dev/null 2>&1; do sleep 1; done; \
LSTE_WORLD=\"$WORLD\" exec rosrun gazebo_ros gzclient --gui-client-plugin \"$GUI_PLUGIN\" __name:=gazebo_gui'"
}

# ---- Session 管理 ----
if tmux has-session -t "=$SESSION" 2>/dev/null; then
  EXISTING_WORLD="$(tmux show-environment -t "=$SESSION" 2>/dev/null | awk -F= '/^LSTE_WORLD=/{print substr($0,length("LSTE_WORLD=")+1)}')"
  EXISTING_SEED="$(tmux show-environment -t "=$SESSION" 2>/dev/null | awk -F= '/^LSTE_GAZEBO_SEED=/{print substr($0,length("LSTE_GAZEBO_SEED=")+1)}')"
  if [[ -n "$EXISTING_WORLD" && ( "$EXISTING_WORLD" != "$WORLD" || "$EXISTING_SEED" != "$GAZEBO_RANDOM_SEED" ) ]]; then
    echo "[info] WORLD or Gazebo seed changed, restarting session."
    tmux kill-session -t "=$SESSION"
  else
    if ! gazebo_gui_ready; then
      echo "[info] Gazebo GUI/plugin missing; restarting the GUI client."
      start_gazebo_gui
    fi
    echo "[info] Session '$SESSION' 已存在 (WORLD=$WORLD), GUI 插件已就绪。" >&2
    echo "  提示：按 Ctrl+B 再按 D 可分离 tmux，Ctrl+C 退出。"
    if [[ -t 0 && "${LSTE_NO_ATTACH:-0}" != "1" ]]; then
      tmux attach -t "=$SESSION"
    fi
    exit 0
  fi
fi

# ---- 新建 session ----
tmux new-session -d -s "$SESSION" -c "$WS" -n "roscore" \
  "bash -lc 'WS=\"$WS\"; source \"$WS/scripts/config/pipeline_env.sh\"; roscore'; echo; echo '[EXIT] roscore'; exec bash"
tmux set-environment -t "=$SESSION" LSTE_WORLD "$WORLD"
tmux set-environment -t "=$SESSION" LSTE_GAZEBO_SEED "$GAZEBO_RANDOM_SEED"
tmux set-option -t "=$SESSION:" remain-on-exit on
echo "[env] roscore started (window 0)"

WAIT_ROSCORE='until rostopic list >/dev/null 2>&1; do echo \"waiting for roscore...\"; sleep 1; done'

# Window 1: Gazebo 场景（不含 pro3，用 _no_pro3.world）
# Keep each launch argument as a single shell word.  In particular,
# ``extra_gazebo_args`` may be "--seed N"; interpolating it into the tmux
# command used to split it into two roslaunch arguments and leave runall
# waiting forever for Gazebo's spawn service.
printf -v WORLD_QUOTED '%q' "$WORLD"
printf -v PIPELINE_ENV_QUOTED '%q' "$WS/scripts/config/pipeline_env.sh"
printf -v GAZEBO_ARGS_QUOTED '%q' "$GAZEBO_EXTRA_ARGS"
WORLD_COMMAND="WS=$(printf '%q' "$WS"); source $PIPELINE_ENV_QUOTED; $WAIT_ROSCORE; export LIBGL_ALWAYS_SOFTWARE=1; lste_gazebo_exec roslaunch lste_core lab_with_pro3.launch world_name:=$WORLD_QUOTED spawn_pro3:=false gui:=$GUI extra_gazebo_args:=$GAZEBO_ARGS_QUOTED"
printf -v WORLD_COMMAND_QUOTED '%q' "$WORLD_COMMAND"
tmux new-window -d -t "=$SESSION" -n "world" -c "$WS" \
  "bash -lc $WORLD_COMMAND_QUOTED"
echo "[env] Gazebo world started (window 1)"

echo ""
echo "============================================"
echo "  环境已启动（场景无小车）。运行以下命令："
echo "  ./scripts/lifecycle/run_nodes_tmux.sh                         （大脑 + TEB）"
echo "  LSTE_CONTROLLER=teleop ./scripts/lifecycle/run_nodes_tmux.sh  （大脑 + 键盘遥控）"
echo "============================================"

if [[ -t 0 && "${LSTE_NO_ATTACH:-0}" != "1" ]]; then
  tmux attach -t "=$SESSION"
fi
