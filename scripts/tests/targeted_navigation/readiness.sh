#!/usr/bin/env bash

# Shared startup probes for the short navigation experiments. This file is
# sourced by the runners; it deliberately uses ROS graph state instead of
# roslaunch's buffered console output as the readiness contract.

READINESS_NODES=""
READINESS_TOPICS=""
READINESS_MISSING_NODES=()
READINESS_MISSING_TOPICS=()
READINESS_MISSING_FILES=()
READINESS_METRICS_STARTED=false

readiness_has_line() {
  local needle="$1"
  local haystack="$2"
  grep -Fqx -- "$needle" <<<"$haystack"
}

readiness_refresh_inventory() {
  READINESS_NODES="$(rosnode list 2>/dev/null || true)"
  READINESS_TOPICS="$(rostopic list 2>/dev/null || true)"
}

readiness_metrics_started() {
  local metrics_log="$1"
  [[ -f "$metrics_log" ]] && rg -q 'event=run_start' "$metrics_log"
}

readiness_check() {
  local profile="$1"
  local metrics_log="$2"
  local node
  local topic
  local -a required_nodes=()
  local -a required_topics=()

  READINESS_MISSING_NODES=()
  READINESS_MISSING_TOPICS=()
  READINESS_MISSING_FILES=()
  READINESS_METRICS_STARTED=false
  readiness_refresh_inventory

  case "$profile" in
    t_junction)
      required_nodes=(
        /move_base
        /lste_global_frontier
        /lste_navigation_metrics
      )
      required_topics=(
        /map
        /move_base/global_costmap/costmap
        /move_base/local_costmap/costmap
        /pro3/wheel_odom
        /pro3/rlscan
      )
      ;;
    endpoint)
      required_nodes=(/move_base /lste_navigation_metrics)
      required_topics=(
        /map
        /move_base/global_costmap/costmap
        /move_base/local_costmap/costmap
        /pro3/wheel_odom
        /pro3/rlscan
      )
      ;;
    *)
      echo "[readiness] unknown profile: $profile" >&2
      return 2
      ;;
  esac

  for node in "${required_nodes[@]}"; do
    if ! readiness_has_line "$node" "$READINESS_NODES"; then
      READINESS_MISSING_NODES+=("$node")
    fi
  done
  for topic in "${required_topics[@]}"; do
    if ! readiness_has_line "$topic" "$READINESS_TOPICS"; then
      READINESS_MISSING_TOPICS+=("$topic")
    fi
  done
  if readiness_metrics_started "$metrics_log"; then
    READINESS_METRICS_STARTED=true
  else
    READINESS_MISSING_FILES+=("metrics_run_start")
  fi

  ((${#READINESS_MISSING_NODES[@]} == 0)) \
    && ((${#READINESS_MISSING_TOPICS[@]} == 0)) \
    && ((${#READINESS_MISSING_FILES[@]} == 0))
}

readiness_missing_summary() {
  local -a values=()
  local value
  for value in "${READINESS_MISSING_NODES[@]}"; do values+=("node:$value"); done
  for value in "${READINESS_MISSING_TOPICS[@]}"; do values+=("topic:$value"); done
  for value in "${READINESS_MISSING_FILES[@]}"; do values+=("file:$value"); done
  if ((${#values[@]} == 0)); then
    printf '%s\n' none
  else
    local joined
    joined="$(IFS=,; printf '%s' "${values[*]}")"
    printf '%s\n' "$joined"
  fi
}

readiness_write_failure_artifact() {
  local artifact_path="$1"
  local process_name="$2"
  local reason="$3"
  local run_timestamp="$4"
  local launcher_log="$5"
  local metrics_log="$6"
  local lifecycle_log="$7"
  local launch_pid="$8"
  local elapsed_seconds="$9"
  local startup_config="${10:-}"
  local missing_nodes missing_topics missing_files
  local process_alive=false
  missing_nodes="$(IFS=,; printf '%s' "${READINESS_MISSING_NODES[*]}")"
  missing_topics="$(IFS=,; printf '%s' "${READINESS_MISSING_TOPICS[*]}")"
  missing_files="$(IFS=,; printf '%s' "${READINESS_MISSING_FILES[*]}")"
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    process_alive=true
  fi

  python3 - "$artifact_path" "$process_name" "$reason" "$run_timestamp" \
    "$launcher_log" "$metrics_log" "$lifecycle_log" "$launch_pid" \
    "$elapsed_seconds" "$startup_config" "$missing_nodes" "$missing_topics" \
    "$missing_files" "$READINESS_NODES" "$READINESS_TOPICS" \
    "$READINESS_METRICS_STARTED" "$process_alive" <<'PY'
import datetime
import json
import os
import sys
from pathlib import Path


(
    artifact_path,
    process_name,
    reason,
    run_timestamp,
    launcher_log,
    metrics_log,
    lifecycle_log,
    launch_pid,
    elapsed_seconds,
    startup_config,
    missing_nodes,
    missing_topics,
    missing_files,
    observed_nodes,
    observed_topics,
    metrics_started,
    process_alive,
) = sys.argv[1:]


def tail(path, limit=80):
    try:
        return Path(path).read_text(encoding="utf-8", errors="replace").splitlines()[-limit:]
    except OSError:
        return []


def split(value):
    return [item for item in value.split(",") if item]


payload = {
    "schema_version": 1,
    "failure_kind": "startup_failure",
    "artifact_kind": "targeted_navigation_startup_failure",
    "artifact_path": str(Path(artifact_path)),
    "failure_id": "%s-S0001" % run_timestamp,
    "created_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    "run_timestamp": run_timestamp,
    "process": process_name,
    "reason": reason,
    "elapsed_wall_seconds": float(elapsed_seconds),
    "startup_config": startup_config,
    "readiness": {
        "status": "timeout",
        "ready": False,
        "reason": reason,
        "elapsed_wall_seconds": float(elapsed_seconds),
    },
    "checks": {
        "missing_nodes": split(missing_nodes),
        "missing_topics": split(missing_topics),
        "missing_files": split(missing_files),
        "observed_nodes": observed_nodes.splitlines(),
        "observed_topics": observed_topics.splitlines(),
        "metrics_run_start": metrics_started == "true",
        "launch_process_alive": process_alive == "true",
        "launch_pid": int(launch_pid) if launch_pid.isdigit() else None,
    },
    "logs": {
        "launcher": launcher_log,
        "metrics": metrics_log,
        "lifecycle": lifecycle_log,
        "launcher_tail": tail(launcher_log),
        "metrics_tail": tail(metrics_log, 32),
        "lifecycle_tail": tail(lifecycle_log, 32),
    },
}
path = Path(artifact_path)
path.parent.mkdir(parents=True, exist_ok=True)
temporary = path.with_suffix(path.suffix + ".tmp")
temporary.write_text(
    json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
)
os.replace(temporary, path)
PY
}

readiness_write_trial_end_artifact() {
  local artifact_path="$1"
  local metrics_path="$2"
  local run_timestamp="$3"
  local startup_config="$4"
  local workspace_root="$5"
  local outcome="${6:-timeout}"

  # Reuse the benchmark's bounded final-state reducer so a timeout without a
  # formal failure episode still has one consistent, readable evidence file.
  python3 - "$artifact_path" "$metrics_path" "$run_timestamp" \
    "$startup_config" "$workspace_root" "$outcome" <<'PY'
import datetime
import json
import os
import sys
from pathlib import Path


artifact_path, metrics_path, run_timestamp, startup_config, workspace_root, outcome = sys.argv[1:]
scripts = Path(workspace_root) / "scripts/tests/office_building"
sys.path.insert(0, str(scripts))
from run_experiment_suite import build_trial_end_diagnostic  # noqa: E402

diagnostic = build_trial_end_diagnostic(Path(metrics_path))
payload = {
    "schema_version": 1,
    "artifact_kind": "trial_end_diagnostic",
    "created_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    "run_timestamp": run_timestamp,
    "outcome": outcome,
    "stop_reason": outcome,
    "run_config": startup_config,
    "metrics_log": metrics_path,
    "diagnostic": diagnostic if isinstance(diagnostic, dict) else {},
    "artifact_path": artifact_path,
}
path = Path(artifact_path)
path.parent.mkdir(parents=True, exist_ok=True)
temporary = path.with_name(path.name + ".tmp")
temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
os.replace(temporary, path)
PY
}
