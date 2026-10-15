# Workspace Rules

## Code Organization

- Keep ROS entrypoints thin: initialize the node, compose focused modules, and
  start the runtime. Do not accumulate callbacks, parameter parsing, mutable
  state setup, geometry, and control policy in one script.
- When a module has multiple independent responsibilities or is becoming hard
  to navigate, split it by responsibility with explicit names (for example
  `*_parameters.py`, `*_state.py`, `*_ros.py`, `*_routes.py`, or `*_control.py`).
  Preserve the existing public ROS interfaces unless the task explicitly
  changes them.
- Do not split merely to reduce line count. Keep closely related logic together
  and avoid very small wrapper files or deep mixin chains that force readers to
  jump across many modules.
- Build commands from named arrays or helper functions instead of maintaining
  long, shell-escaped command strings, so each argument remains easy to edit
  and review.
- Add a focused structural or behavioral regression test whenever a running
  node is split, and install every sibling Python module needed by an
  install-space or devel-space launch.

## Documentation Naming

- Except for the fixed index `docs/introduction.md`, files under `docs/` use
  the `YYYYMMDD_topic.extension` format; Markdown documents use
  `YYYYMMDD_topic.md`.
- The date prefix is written as year, month, and day in that order. Keep the
  fixed index path unchanged when updating its links.

## Fixed-Goal Test Logs

- Every `rltest` invocation must create one timestamped parent directory under
  `runtime/rl_fixed_goal_test/logs/` using the format `YYYYMMDD_HHMMSS`.
- All program logs for that invocation must be `.log` files inside that one
  timestamped directory. Do not write the formal experiment logs directly to
  `~/.ros/log` or mix files from multiple runs in one directory.
- The timestamped directory must contain one separate log for every launched
  process. Every log filename must repeat the same timestamp as its parent,
  using `YYYYMMDD_HHMMSS_<process>.log` (for example
  `20260723_191530_lifecycle.log`, `20260723_191530_navigation.log`, or
  `20260723_191530_frontier_manager.log`).
- The lifecycle file is named `YYYYMMDD_HHMMSS_lifecycle.log` and must record
  the resolved startup configuration, controller, speed, initial pose, target,
  world, Git revision, and start/stop events.
- Each process writes only its own log file. Log lines must include a timestamp,
  log level, process name, event, and relevant parameters.
- Retain test log directories for 15 days unless a test-specific configuration
  explicitly sets a different retention period.

## Failure Evidence

- `lste_navigation_metrics` must keep a bounded in-memory evidence ring and
  write failure episodes to the current run directory, never to a shared
  global log.
- Every episode uses one `<run_timestamp>-F####` failure ID in
  `failure_started`, related events, and `failure_snapshot_ready` records.
- A failure snapshot must retain the pre/post context needed to distinguish
  planner no-path, local obstacle, controller stall, TF/transform failure,
  ownership race, and exhausted recovery. It must contain summarized pose,
  goal/route transaction, command/mux, scan, Navfn/TEB, costmap, feedback,
  and recovery state; raw images and full costmap arrays are not required.
- Each closed episode must also be written as one atomic
  `<run_timestamp>_failure_<sequence>.json` artifact in the same run directory;
  the path must be included in `failure_snapshot_ready` and the experiment
  record so an investigator can open one failure without parsing a large log.
- Transient candidate/planning waits are context events, not failures. Only a
  terminal failure, explicit route failure, unexpected preemption, or a
  sustained no-progress/lease wait may start a failure episode.
