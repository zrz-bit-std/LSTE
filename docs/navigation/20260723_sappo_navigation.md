# SA-PPO Navigation

This workspace contains a self-contained SA-PPO inference integration for the
LSTE Gazebo environment. It runs the validated policy directly and does not
start training.

For the Chinese operational startup guide, see
[`20260723_startup_guide.md`](../operations/20260723_startup_guide.md).

## Integrated Start

The integrated mode starts the original LSTE brain and SA-PPO in one `lste`
tmux session. LSTE selects the target on `/lste/final_goal`; SA-PPO consumes
that goal and drives the vehicle through `/cmd_vel`.

Start it after the Gazebo environment:

```bash
./scripts/lifecycle/run_all_tmux.sh
```

Stop the complete system with:

```bash
./scripts/lifecycle/stop_all_tmux.sh
```

This mode spawns Pro3 once, starts point-cloud conversion, subscribes SA-PPO
directly to `/lste/final_goal`, and does not start the keyboard teleop
publisher. SA-PPO and a speed scale of `0.50` are the defaults, so no launch
arguments are required.

Hot-switch controllers with the Gazebo `RL / KB` button or press `Ctrl+Shift+K`.
Keyboard control runs continuously in the peer tmux session `lste-teleop`,
separate from `lste` and `lste-env`. SA-PPO also remains running; switching only
changes which input the command-velocity mux forwards to `/cmd_vel`.
The command-line fallback is:

```bash
./scripts/lifecycle/switch_controller.sh teleop
./scripts/lifecycle/switch_controller.sh sappo
```

## Standalone Start

```bash
cd .
source /opt/ros/noetic/setup.bash
catkin_make --pkg gazebo_click_point pointcloud_to_laserscan
SAPPO_SPEED=0.50 ./scripts/lifecycle/run_sappo_tmux.sh
```

The script starts `lste-env` for ROS and Gazebo, then starts `sappo-lste` with
three windows: Pro3, point cloud to laser conversion, and SA-PPO inference.
It uses `rl_navigation/policy/sa_peppo_1650.pth` and publishes velocity on
`/cmd_vel`.

Do not run this standalone launcher together with `run_nodes_tmux.sh`; both
would attempt to spawn Pro3.

## Goal Selection

Hold the left mouse button over Gazebo to see the ground coordinate. Releasing
the button hides the hint but retains the point. Use `Shift + left click` to
publish the retained point to `/move_base/current_goal`; right click remains
available for Gazebo's context menu. A new goal re-enables motion after the
previous goal has been reached.

The Pro3 spawn defaults are `x = 11.38` and `y = -5.97`. Override the initial
goal and speed when launching if required:

```bash
SAPPO_SPEED=0.50 SAPPO_GOAL_X=10.5 SAPPO_GOAL_Y=6.0 ./scripts/lifecycle/run_sappo_tmux.sh
```

## Included Files

- `rl_navigation/`: SA-PPO inference code, model definition, and policy
- `src/gazebo_click_point/`: Gazebo GUI coordinate and goal plugin
- `src/tools/pointcloud_to_laserscan/`: Pro3 point cloud to 512-beam scan bridge
- `scripts/lifecycle/run_sappo_tmux.sh`: standalone SA-PPO launcher
