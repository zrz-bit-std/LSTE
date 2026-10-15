# Telemetry and TEB Bridge Code Layout

The runtime nodes remain the composition roots. Large, independently testable
policies are implemented as sibling mixins so a change to one concern does not
require editing a multi-thousand-line node file.

## Navigation metrics

`src/lste_topo_access/scripts/lste_navigation_metrics.py` owns ROS subscribers,
run initialization, shared counters, lifecycle logging, and the final snapshot.

| Module | Responsibility |
| --- | --- |
| `navigation_metrics_command_events.py` | Raw TEB command events plus the evidence-only explanation for each command discontinuity. |
| `navigation_metrics_command_stream.py` | Final `/cmd_vel` quality metrics and mux-intervention accounting. |
| `navigation_metrics_target_eval.py` | Gazebo ground-truth target projection, frustum exposure, compressed-depth decoding, and exposure-episode summaries. |
| `navigation_metrics_detection.py` | Detector-message matching, IoU scoring, target acquisition evidence, and frame-level detection records. |

Both mixins are observer-only. They never publish motion commands or alter goal
selection. The host class provides the lock, `_write`, pose transforms, and
configured counters. This keeps simulator truth evaluation separate from the
detector transport and from navigation telemetry.

`navigation_metrics_command_quality.py` is deliberately only a compatibility
facade.  It composes the two command modules above so existing imports keep
working.  Edit a command-gap explanation in `command_events`; edit a counter
or a `/cmd_vel`/mux record in `command_stream`.

## TEB goal bridge

`src/lste_topo_access/scripts/lste_teb_goal_bridge.py` owns ROS wiring, action
dispatch, and route handoff orchestration.

| Module | Responsibility |
| --- | --- |
| `teb_goal_bridge_action_health.py` | Action cancellation/reset, pending-goal geometry, handoff heading checks, and one-shot target-failure latching. |
| `teb_goal_bridge_persistent_target.py` | Optional persistent Navfn target transaction: request, install, approve, and clear a target path. |

The mixin intentionally calls host lifecycle helpers such as
`publish_bridge_status`, `_goal_in_global_frame`, and
`_target_progress_state_locked`; it does not create a second action client or
publisher. The behavior therefore remains identical while the health policy is
editable in isolation.

## Installation and verification

The sibling modules are listed in `src/lste_topo_access/CMakeLists.txt` and are
installed beside their executable nodes. `test_goal_manager_module_layout.py`
checks that the split remains visible and that the composition roots no longer
contain the extracted method bodies.

Run the fast checks with:

```bash
python3 -m py_compile src/lste_topo_access/scripts/lste_navigation_metrics.py \
  src/lste_topo_access/scripts/navigation_metrics_detection.py \
  src/lste_topo_access/scripts/navigation_metrics_target_eval.py \
  src/lste_topo_access/scripts/lste_teb_goal_bridge.py \
  src/lste_topo_access/scripts/teb_goal_bridge_action_health.py
python3 -m unittest discover -s src/lste_topo_access/test -p 'test_*.py'
```
