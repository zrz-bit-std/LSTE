# Script Layout

## Short Commands

The workspace `.envrc` adds `scripts/bin/` to `PATH`, providing these
directory-local commands:

```bash
runall
stopall
lste-env
teleop
health
```

Run `direnv allow` once in the workspace root. The commands and `LSTE_WS`
environment variable are automatically removed after leaving the workspace.
`health` reads the latest sidecar status from `runtime/health/status.json`;
`health --json` prints the full machine-readable audit result.

## Lifecycle

Public startup, shutdown, and controller-management entry points are under
`lifecycle/`.

```bash
# Start the complete LSTE system
./scripts/lifecycle/run_all_tmux.sh

# Stop the complete LSTE system
./scripts/lifecycle/stop_all_tmux.sh
```

The remaining lifecycle scripts support split startup, standalone SA-PPO,
legacy pipeline startup, and controller hot switching.

## Config

`config/` contains the shared shell environment and pipeline defaults used by
the lifecycle scripts.

## Tools

`tools/` contains operator and debugging utilities such as keyboard teleop.

## Archive

`archive/` contains historical command notes that are not active entry points.

## Bin

`bin/` contains the short command wrappers exposed by `.envrc`. `lste-env`
starts only `lste-env` (roscore + Gazebo + GUI) in the background. `teleop`
attaches directly to the `lste-teleop` keyboard-control session.
