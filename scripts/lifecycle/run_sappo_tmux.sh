#!/usr/bin/env bash
set -euo pipefail

WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
RL_DIR="$WS/rl_navigation"
ENV_SESSION="lste-env"
SESSION="sappo-lste"
PIPELINE_CONFIG=${PIPELINE_CONFIG:-$WS/scripts/config/pipeline_defaults.yaml}
if [[ -f "$PIPELINE_CONFIG" ]]; then
  eval "$("$WS/scripts/config/load_pipeline_config.sh" "$PIPELINE_CONFIG")"
fi
PRO3_SPAWN_X=${PRO3_SPAWN_X:-${CFG_PRO3_SPAWN_X:-0.0}}
PRO3_SPAWN_Y=${PRO3_SPAWN_Y:-${CFG_PRO3_SPAWN_Y:-0.0}}
PRO3_SPAWN_Z=${PRO3_SPAWN_Z:-${CFG_PRO3_SPAWN_Z:-0.0}}
PRO3_SPAWN_YAW=${PRO3_SPAWN_YAW:-${CFG_PRO3_SPAWN_YAW:-0.0}}
SPEED="${SAPPO_SPEED:-0.35}"
GOAL_X="${SAPPO_GOAL_X:-10.5}"
GOAL_Y="${SAPPO_GOAL_Y:-6.0}"
RUNTIME_DIR="$WS/runtime/sappo"
POLICY_DIR="$RL_DIR/policy"
PYTHON_BIN="${SAPPO_PYTHON:-$HOME/miniconda3/envs/rlenvs/bin/python}"
PYTHON_ENV="$(cd "$(dirname "$PYTHON_BIN")/.." && pwd)"
PYTHON_SITE="$PYTHON_ENV/lib/python3.9/site-packages"

if ! tmux has-session -t "=$ENV_SESSION" 2>/dev/null; then
  WS="$WS" "$WS/scripts/lifecycle/run_env_tmux.sh"
fi

if tmux has-session -t "=lste" 2>/dev/null; then
  echo "LSTE node session 'lste' is running; stop it before standalone SA-PPO validation." >&2
  exit 1
fi

if [[ ! -f "$RL_DIR/sappo_pure.py" || ! -f "$POLICY_DIR/sa_peppo_1650.pth" ]]; then
  echo "SA-PPO inference files are missing under $RL_DIR" >&2
  exit 1
fi

if tmux has-session -t "=$SESSION" 2>/dev/null; then
  tmux kill-session -t "=$SESSION"
fi

mkdir -p "$RUNTIME_DIR"
if [[ -d "$RUNTIME_DIR/policy" && ! -L "$RUNTIME_DIR/policy" ]]; then
  rmdir "$RUNTIME_DIR/policy"
fi
ln -sfn "$POLICY_DIR" "$RUNTIME_DIR/policy"
tmux new-session -d -s "$SESSION" -c "$WS" -n "pro3" \
  "bash -lc 'source \"$WS/scripts/config/pipeline_env.sh\"; until rosservice list | grep -qx /gazebo/spawn_urdf_model; do sleep 1; done; roslaunch lste_core spawn_pro3.launch spawn_model:=true x:=$PRO3_SPAWN_X y:=$PRO3_SPAWN_Y z:=$PRO3_SPAWN_Z yaw:=$PRO3_SPAWN_YAW || true; until rostopic list | grep -qx /pro3/wheel_odom; do sleep 1; done; exec bash'"
tmux new-window -t "=$SESSION:1" -n "scan" -c "$WS" \
  "bash -lc 'source \"$WS/scripts/config/pipeline_env.sh\"; until rostopic list >/dev/null 2>&1; do sleep 1; done; roslaunch pointcloud_to_laserscan lste_pro3_to_scan.launch; exec bash'"
tmux new-window -t "=$SESSION:2" -n "sappo" -c "$RUNTIME_DIR" \
  "bash -lc 'until rostopic list | grep -qx /pro3/wheel_odom && rostopic list | grep -qx /pro3/rlscan; do sleep 1; done; PYTHONPATH=\"$PYTHON_SITE:$WS/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages\" \"$PYTHON_BIN\" \"$RL_DIR/sappo_pure.py\" _linear_speed_scale:=$SPEED _goal_x:=$GOAL_X _goal_y:=$GOAL_Y; exec bash'"

echo "SA-PPO started in tmux session '$SESSION' (speed=$SPEED, goal=($GOAL_X, $GOAL_Y))."
