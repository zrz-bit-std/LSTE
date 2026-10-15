#!/usr/bin/env bash
set -euo pipefail

# Short entrypoint for the fast ownership-failure experiment. The underlying
# runner still owns timestamps, cleanup, reports, and all launch arguments.
WS="${LSTE_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
# Keep the one-command diagnostic deterministic: a point outside the compact
# map must exercise Navfn's no-path result. Any explicit --target-goal passed
# by the caller appears later and overrides these defaults in the runner.
TARGET_X="${T_JUNCTION_TARGET_PROBE_X:-100.0}"
TARGET_Y="${T_JUNCTION_TARGET_PROBE_Y:-100.0}"
# The targeted runner owns an exclusive ROS/Gazebo instance. Standard LSTE
# sessions are safe to stop here, and the command is idempotent when nothing
# is running; the runner then records the new run in its own timestamp folder.
"$WS/scripts/bin/stopall" >/dev/null 2>&1 || true
exec "$WS/scripts/tests/targeted_navigation/run_t_junction.sh" \
  --persistent --probe-unreachable-target \
  --target-goal "$TARGET_X" "$TARGET_Y" "$@"
