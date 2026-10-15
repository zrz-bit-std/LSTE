# Fixed-Goal Controller Video Evidence (2026-07-25)

This page indexes the real simulator recordings used to explain the current
fixed-goal navigation work. The videos are runtime artifacts and intentionally
remain outside Git. Their matching formal ROS logs are retained for 15 days.

## Shared Scenario

All three recordings use the same isolated Gazebo world and fixed task:

| Item | Value |
| --- | --- |
| World | `worlds/topo3.0_catch_mode/session_test/env04_no_pro3.world` |
| Initial pose | `(15.79, -8.45, 0.10, -pi/2)` |
| Final goal | `(12.95, -6.51)` |
| ROS / Gazebo master | `11312` / `11346` |
| Screen source | Actual RViz X11 window, 1208x656, 30 FPS |

The videos are an explanatory comparison, not a statistically valid benchmark:
TEB and DWA have a `1.0 m/s` navigation limit, while the original SA-PPO policy
uses its existing `0.5 m/s` scale. No video is sped up and failure footage is
not removed.

## Raw Recordings

| Method | Video | Formal logs | Observed result |
| --- | --- | --- | --- |
| SA-PPO `policy_only` | `runtime/rl_fixed_goal_test/videos/20260725_190115/20260725_190115_rl_policy_only_rviz.mp4` | `runtime/rl_fixed_goal_test/logs/20260725_190115/` | Drove into a close-obstacle state, then policy output settled near zero at `(14.69, -10.40)`, `4.26 m` from the goal. |
| DWA | `runtime/rl_fixed_goal_test/videos/20260725_185731/20260725_185731_dwa_rviz.mp4` | `runtime/rl_fixed_goal_test/logs/20260725_185731/` | Stopped near `(15.96, -7.86)`, `3.30 m` from the goal; no successful move_base completion was logged. |
| TEB | `runtime/rl_fixed_goal_test/videos/20260725_185540/20260725_185540_teb_rviz.mp4` | `runtime/rl_fixed_goal_test/logs/20260725_185540/` | Reached the fixed goal. `frontier_manager.log` records `move_base reports the fixed final goal reached`. |

`policy_only` is deliberately the raw SA-PPO baseline. It does not use the
test-only `rl_grid_guard`, SLAM, Navfn, DWA, or TEB. Its RViz view contains only
the actual robot model, 2D lidar, odom frame, and final-goal topic; it does not
claim to show a map it does not maintain.

## Side-by-Side Clip

The synchronized first 45 seconds of the raw clips are in:

```text
runtime/rl_fixed_goal_test/videos/comparisons/20260725_190115/
20260725_190115_fixed_goal_rl_dwa_teb_comparison.mp4
```

The columns, from left to right, are `SA-PPO policy-only`, `DWA`, and `TEB`.
The top labels are added only during composition; all navigation imagery comes
from the original recordings above.

## Re-recording

The recording helper starts detached and does not modify `test_config.yaml`:

```bash
rlrecord teb 45
rlrecord dwa 75
RL_RECORD_RL_MODE=policy_only rlrecord rl 75
```

It creates a temporary runtime configuration, writes each process log to the
same invocation's timestamped log directory, and records the selected X11
window. The default `SCREEN_RECORD_WINDOW: RViz` is appropriate for the
navigation explanation; `Gazebo` and `desktop` are available when a physical
scene view is needed.
