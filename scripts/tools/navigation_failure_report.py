#!/usr/bin/env python3
"""Build one concise, read-only report for a navigation run.

The metrics stream remains the source of truth.  This command only joins the
already-written artifacts so an investigator can open one report instead of
manually correlating metrics, failure evidence, and launcher logs.
"""

from __future__ import annotations

import argparse
import datetime as _datetime
import json
import re
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from analyze_navigation_metrics import (  # noqa: E402
    failure_episode_summary,
    load_failure_snapshots,
    load_records,
    load_startup_failure,
)


_LIFECYCLE_RE = re.compile(
    r"^(?P<stamp>\S+) \[(?P<level>[A-Z]+)\] "
    r"process=(?P<process>\S+) event=(?P<event>\S+)"
    r"(?: run_timestamp=(?P<run>\S+))?(?: (?P<fields>.*))?$"
)
_FIELD_RE = re.compile(r"(?:^| )(?P<key>[A-Za-z0-9_]+)=(?P<value>\S+)")
_TRUNCATION = "<max-depth>"
_METRICS_READ_LIMIT_BYTES = 8 * 1024 * 1024
_METRICS_HEAD_BYTES = 128 * 1024
_PARAMETER_LOG_LIMIT_BYTES = 512 * 1024
_FAILURE_ID_RE = re.compile(r"^(?P<run>\d{8}_\d{6})-F(?P<sequence>\d{4})$")
_FAILURE_ARTIFACT_RE = re.compile(
    r"^(?P<run>\d{8}_\d{6})_failure_(?P<sequence>\d{4})\.json$"
)
_FAILURE_LIFECYCLE_EVENTS = frozenset(
    ("failure_started", "failure_related_event", "failure_snapshot_ready")
)
_PARAMETER_KEYS = {
    "/move_base/TebLocalPlannerROS/max_vel_x": "teb_max_vel_x",
    "/move_base/TebLocalPlannerROS/max_vel_x_backwards": "teb_max_vel_x_backwards",
    "/move_base/TebLocalPlannerROS/max_vel_theta": "teb_max_vel_theta",
    "/move_base/TebLocalPlannerROS/xy_goal_tolerance": "teb_xy_goal_tolerance",
    "/move_base/TebLocalPlannerROS/min_obstacle_dist": "teb_min_obstacle_dist",
    "/move_base/TebLocalPlannerROS/inflation_dist": "teb_inflation_dist",
    "/move_base/controller_frequency": "teb_controller_frequency",
    "/lste_teb_goal_bridge/progress_timeout": "teb_progress_timeout",
    "/lste_teb_goal_bridge/min_update_interval": "teb_goal_min_update_interval",
    "/lste_global_frontier/planning_period": "frontier_planning_period",
    "/lste_global_frontier/stall_timeout": "frontier_stall_timeout",
    "/lste_global_frontier/active_timeout": "frontier_active_timeout",
}


def _resolve_run(path: Path | None):
    """Resolve a metrics log, artifact, or run directory to one directory."""
    if path is None:
        candidates = []
        for root in (
            ROOT / "runtime/targeted_navigation",
            ROOT / "runtime/office_building_benchmark/logs",
            ROOT / "runtime/navigation/logs",
        ):
            if root.is_dir():
                candidates.extend(
                    item for item in root.rglob("*_navigation_metrics.log")
                    if item.is_file()
                )
        if not candidates:
            return None, None
        metrics = max(candidates, key=lambda item: item.stat().st_mtime)
        return metrics.parent, metrics

    path = Path(path).expanduser()
    if path.is_dir():
        run_dir = path.resolve()
        exact = run_dir / (run_dir.name + "_navigation_metrics.log")
        if exact.is_file():
            return run_dir, exact
        matches = sorted(run_dir.glob("*_navigation_metrics.log"))
        return run_dir, matches[-1] if matches else None
    if path.is_file():
        if path.name.endswith("_navigation_metrics.log"):
            return path.parent.resolve(), path.resolve()
        return path.parent.resolve(), next(
            iter(sorted(path.parent.glob("*_navigation_metrics.log"))), None
        )
    return None, None


def _parse_lifecycle(path: Path | None):
    events = []
    if path is None or not path.is_file():
        return events
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return events
    for line in lines:
        match = _LIFECYCLE_RE.match(line.strip())
        if match is None:
            continue
        stamp = match.group("stamp")
        try:
            # ``date --iso-8601=seconds`` emits ``+08:00`` while the Python
            # runner's ``%z`` formatter emits ``+0800``. Python 3.8 accepts
            # the former but not the latter, so normalize the numeric offset
            # before parsing instead of dropping all phase timings.
            normalized_stamp = re.sub(
                r"([+-]\d{2})(\d{2})$", r"\1:\2", stamp
            )
            wall = _datetime.datetime.fromisoformat(normalized_stamp)
            wall_seconds = wall.timestamp()
        except (TypeError, ValueError, OverflowError):
            wall_seconds = None
        fields = {
            item.group("key"): item.group("value")
            for item in _FIELD_RE.finditer(match.group("fields") or "")
        }
        events.append(
            {
                "event": match.group("event"),
                "level": match.group("level"),
                "process": match.group("process"),
                "stamp": stamp,
                "wall_seconds": wall_seconds,
                "fields": fields,
            }
        )
    return events


def _records_from_text(text):
    """Decode metrics records from a bounded text window."""
    records = []
    for line in text.splitlines():
        marker = " data="
        if marker not in line:
            continue
        try:
            record = json.loads(line.split(marker, 1)[1])
        except (TypeError, ValueError, json.JSONDecodeError):
            continue
        if not isinstance(record, dict):
            continue
        event = (
            line.split(" event=", 1)[1].split(marker, 1)[0]
            if " event=" in line
            else ""
        )
        record["_event"] = event
        record["_line"] = line.rstrip()
        records.append(record)
    return records


def _load_metrics_records(metrics_path: Path | None):
    """Read all small logs, or only head+tail for a large run."""
    if metrics_path is None or not Path(metrics_path).is_file():
        return [], False
    metrics_path = Path(metrics_path)
    try:
        size = metrics_path.stat().st_size
    except OSError:
        return [], False
    if size <= _METRICS_READ_LIMIT_BYTES:
        return load_records(metrics_path), False
    try:
        with metrics_path.open("rb") as stream:
            head = stream.read(_METRICS_HEAD_BYTES)
            stream.seek(max(0, size - _METRICS_READ_LIMIT_BYTES), 0)
            tail = stream.read(_METRICS_READ_LIMIT_BYTES)
    except OSError:
        return [], True
    # The first tail line may be partial; dropping it is safer than accepting
    # malformed JSON. Keep the head's run_start record when available.
    text = head.decode("utf-8", errors="replace") + "\n" + tail.decode(
        "utf-8", errors="replace"
    )
    records = _records_from_text(text)
    unique = []
    seen = set()
    for record in records:
        line = record.get("_line")
        if line in seen:
            continue
        seen.add(line)
        unique.append(record)
    return unique, True


def _parameter_value(text):
    """Parse the scalar value printed by roslaunch's parameter dump."""
    value = str(text).strip()
    if value.lower() in {"true", "false"}:
        return value.lower() == "true"
    try:
        number = float(value)
        return int(number) if number.is_integer() else number
    except (TypeError, ValueError):
        return value


def _load_parameter_evidence(run_dir: Path):
    """Recover runtime parameters from process logs when metrics raced startup."""
    values = {}
    sources = []
    for path in sorted(Path(run_dir).glob("*_teb_navigation.log")) + sorted(
        Path(run_dir).glob("*_global_frontier.log")
    ):
        try:
            size = path.stat().st_size
            with path.open("rb") as stream:
                stream.seek(max(0, size - _PARAMETER_LOG_LIMIT_BYTES), 0)
                text = stream.read(_PARAMETER_LOG_LIMIT_BYTES).decode(
                    "utf-8", errors="replace"
                )
        except OSError:
            continue
        found = False
        for line in text.splitlines():
            # Captured tmux lines use ``event=stdout line=...``; direct
            # roslaunch output may omit that wrapper. Match the stable ROS
            # parameter path and stop at the end of the scalar token.
            for parameter, key in _PARAMETER_KEYS.items():
                match = re.search(
                    re.escape(parameter) + r"\s*:\s*([^\s]+)", line
                )
                if match is None:
                    continue
                values[key] = _parameter_value(match.group(1))
                found = True
        if found:
            sources.append(str(path))
    return {
        "values": values,
        "sources": sources,
        "missing": sorted(set(_PARAMETER_KEYS.values()) - set(values)),
    }


def _merge_startup_parameter_evidence(parameter_evidence, records, snapshots):
    """Use the metrics run-start contract when rosparam output is unavailable."""
    evidence = dict(parameter_evidence or {})
    values = dict(evidence.get("values") or {})
    found = False
    for record in records or []:
        if not isinstance(record, dict) or record.get("_event") != "run_start":
            continue
        resolved = record.get("resolved_params")
        if not isinstance(resolved, dict):
            continue
        for key, value in resolved.items():
            if value is not None and (key not in values or values[key] is None):
                values[str(key)] = value
                found = True
    for snapshot in (snapshots or {}).values():
        if not isinstance(snapshot, dict):
            continue
        context = snapshot.get("run_context")
        resolved = context.get("resolved_params") if isinstance(context, dict) else None
        if not isinstance(resolved, dict):
            continue
        for key, value in resolved.items():
            if value is not None and (key not in values or values[key] is None):
                values[str(key)] = value
                found = True
    if found:
        sources = list(evidence.get("sources") or [])
        marker = "metrics_run_start_or_failure_artifact"
        if marker not in sources:
            sources.append(marker)
        evidence["sources"] = sources
    evidence["values"] = values
    evidence["missing"] = sorted(
        set(_PARAMETER_KEYS.values()) - set(values)
    )
    return evidence


def _event_wall(events, name, last=False):
    matches = [item for item in events if item.get("event") == name]
    if not matches:
        return None
    item = matches[-1] if last else matches[0]
    return item.get("wall_seconds")


def _delta(start, end):
    if start is None or end is None:
        return None
    return round(max(0.0, float(end) - float(start)), 3)


def _number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _phase_timings(run_dir, records, lifecycle):
    """Compute wall-clock phases without relying on a shell's exit timing."""
    run_start = _event_wall(lifecycle, "run_start")
    ready = _event_wall(lifecycle, "navigation_ready")
    if ready is None:
        ready = _event_wall(lifecycle, "controller_ready")
    stop_requested = _event_wall(lifecycle, "failure_stop_requested", last=True)
    if stop_requested is None:
        stop_requested = _event_wall(lifecycle, "runner_stop_requested", last=True)
    if stop_requested is None:
        stop_requested = _event_wall(lifecycle, "run_stop_requested", last=True)
    run_stop = _event_wall(lifecycle, "run_stop", last=True)
    if run_stop is None:
        run_stop = _event_wall(lifecycle, "stop_completed", last=True)

    metrics_start = next(
        (item for item in records if item.get("_event") == "run_start"), None
    )
    metrics_stop = next(
        (item for item in reversed(records) if item.get("_event") == "run_stop"),
        None,
    )
    total = _delta(run_start, run_stop)
    if total is None and isinstance(metrics_stop, dict):
        total = _number(metrics_stop.get("wall_duration_seconds"))
        if total is not None:
            total = round(total, 3)

    result = {
        "launcher_to_ready_seconds": _delta(run_start, ready),
        "navigation_window_seconds": _delta(ready, stop_requested or run_stop),
        "cleanup_seconds": _delta(stop_requested, run_stop),
        "total_wall_seconds": total,
        "source": "lifecycle_log" if lifecycle else "metrics_log",
    }
    if metrics_start and result["launcher_to_ready_seconds"] is None:
        result["launcher_to_ready_seconds"] = None
    if isinstance(metrics_stop, dict):
        result["metrics_reported_wall_seconds"] = _number(
            metrics_stop.get("wall_duration_seconds")
        )
    else:
        result["metrics_reported_wall_seconds"] = None
    return result


def _walk_truncated(value, path=""):
    paths = []
    if value == _TRUNCATION:
        paths.append(path or "root")
    elif isinstance(value, dict):
        for key, item in value.items():
            paths.extend(_walk_truncated(item, "%s.%s" % (path, key)))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            paths.extend(_walk_truncated(item, "%s[%d]" % (path, index)))
    return paths


def _is_pre_dispatch_planner_failure(snapshot, trigger_sample, diagnosis):
    """Recognize a planner rejection that happened before controller dispatch."""
    if not isinstance(snapshot, dict) or not isinstance(trigger_sample, dict):
        return False
    diagnosis = diagnosis if isinstance(diagnosis, dict) else {}
    classification = snapshot.get("classification")
    classification = classification if isinstance(classification, dict) else {}
    cause = str(
        diagnosis.get("primary_cause")
        or classification.get("label")
        or ""
    ).strip().lower()
    if cause != "planner_no_path":
        return False
    route_id = None
    causal = snapshot.get("causal_route")
    if isinstance(causal, dict):
        route_id = causal.get("route_id") or causal.get("active_route_id")
    route_id = route_id or diagnosis.get("route_id")
    if route_id not in (None, "", 0, "0"):
        return False
    boundary = trigger_sample.get("failure_boundary")
    boundary = boundary if isinstance(boundary, dict) else {}
    details = boundary.get("details")
    details = details if isinstance(details, dict) else {}
    snapshot_details = snapshot.get("details")
    snapshot_details = snapshot_details if isinstance(snapshot_details, dict) else {}
    reason = str(
        boundary.get("reason")
        or details.get("reason")
        or snapshot_details.get("reason")
        or ""
    ).strip().lower()
    status = str(
        boundary.get("status")
        or details.get("status")
        or snapshot_details.get("status")
        or ""
    ).strip().lower()
    no_path = any(
        token in reason or token in status
        for token in ("no_path", "no path", "unreachable", "empty_plan")
    )
    navfn = trigger_sample.get("navfn_plan")
    navfn_empty = isinstance(navfn, dict) and not bool(navfn.get("poses"))
    return bool(trigger_sample.get("goal") is not None and (no_path or navfn_empty))


def _sample_missing(sample, allow_pre_dispatch=False):
    if not isinstance(sample, dict):
        return ["sample"]
    required = [
        "pose",
        "goal",
        "cmd_vel",
        "navfn_plan",
        "teb_status",
        "scan",
        "move_base_status",
        # These channels form the minimum causal boundary.  A null value is
        # still useful in the producer log, but the report must call the
        # episode incomplete instead of silently claiming that the channel
        # was captured.
        "cmd_vel_mux",
        "global_costmap",
        "local_costmap",
        "channel_health",
    ]
    if not allow_pre_dispatch:
        required.extend(("route_identity", "teb_feedback", "move_base_feedback"))
    missing = [key for key in required if key not in sample or sample.get(key) is None]
    # ``recovery`` is allowed to be null: null means no recovery action was
    # active at the boundary.  Omitting the key is a schema violation.
    if "recovery" not in sample:
        missing.append("recovery")
    return missing


def _channel_health_gaps(sample):
    """Return channels unavailable at one captured failure boundary.

    Direct sample fields tell us what was serialized; ``channel_health`` tells
    us whether a missing value was an actual sensor/planner gap. Keeping those
    facts separate makes a degraded snapshot actionable instead of opaque.
    """
    if not isinstance(sample, dict):
        return ["channel_health"]
    health = sample.get("channel_health")
    if not isinstance(health, dict):
        return ["channel_health"]
    required = (
        "pose", "goal", "cmd_vel", "scan", "navfn_plan", "teb_feedback",
        "global_costmap", "local_costmap", "move_base_feedback",
        "route_identity",
    )
    gaps = []
    for name in required:
        state = health.get(name)
        if not isinstance(state, dict):
            gaps.append(name)
            continue
        if bool(state.get("available")):
            continue
        # Older metrics versions marked an invalidated TEB payload as
        # unavailable even though the retained trajectory summary was
        # serialized in the sample.  A captured invalid payload is useful
        # causal evidence for a feedback-loss stall; only a genuinely absent
        # payload is a channel gap.
        if name == "teb_feedback" and isinstance(
            sample.get("teb_feedback"), dict
        ):
            continue
        gaps.append(name)
    return gaps


def _critical_truncations(snapshot):
    """Keep serializer truncation warnings focused on causal fields.

    Event-graph payloads intentionally contain bounded lists. Their
    ``<max-depth>`` markers are expected and do not mean that pose or planner
    evidence was lost. Only a marker under a direct causal channel should
    downgrade the evidence contract.
    """
    critical_fields = {
        "pose", "goal", "cmd_vel", "cmd_vel_mux", "teb_feedback",
        "teb_status", "scan", "navfn_plan", "global_costmap",
        "local_costmap", "move_base_feedback", "move_base_status",
        "route_identity", "recovery", "goal_transaction_id",
    }
    paths = _walk_truncated(snapshot)
    critical = []
    for path in paths:
        parts = path.split(".")
        if len(parts) >= 2 and parts[0] in {"trigger_sample", "end_sample"}:
            if parts[1] in critical_fields:
                critical.append(path)
        elif parts[0] in {"causal_route", "diagnosis", "diagnosis_at_trigger"}:
            # These structures are compact projections; a marker here can
            # hide the route identity or primary cause itself.
            critical.append(path)
    return critical


def _failure_event_contract(records, failure_id):
    """Check that one episode has a complete, ID-correlated lifecycle.

    This is intentionally read-only. It catches the post-mortem trap where a
    snapshot exists but the compact metrics log lost its start/ready marker.
    """
    failure_id = str(failure_id or "").strip()
    counts = {event: 0 for event in _FAILURE_LIFECYCLE_EVENTS}
    for record in records or ():
        if not isinstance(record, dict) or record.get("failure_id") != failure_id:
            continue
        event = record.get("_event")
        if event in counts:
            counts[event] += 1
    missing = [
        event
        for event in ("failure_started", "failure_snapshot_ready")
        if counts[event] == 0
    ]
    return {
        "complete": not missing,
        "missing": missing,
        "counts": counts,
        "failure_id": failure_id,
        "source": "metrics_log",
    }


def _success_evidence(records):
    """Return the first successful action boundary in a metrics stream."""
    for record in records or ():
        if not isinstance(record, dict):
            continue
        if record.get("_event") != "move_base_status":
            continue
        status_name = str(record.get("status_name", "")).strip().upper()
        if status_name != "SUCCEEDED":
            continue
        return {
            "observed": True,
            "event": "move_base_status",
            "ros_time": record.get("ros_time"),
            "goal_id": record.get("goal_id"),
            "status": record.get("status"),
            "status_name": status_name,
            "text": record.get("text"),
        }
    # A compact run_stop summary can outlive the individual status line in a
    # bounded metrics read. Keep this fallback explicit rather than guessing
    # success from a zero command or a small final distance.
    for record in reversed(list(records or ())):
        if not isinstance(record, dict) or record.get("_event") != "run_stop":
            continue
        summary = record.get("summary")
        if not isinstance(summary, dict):
            continue
        try:
            successes = int(summary.get("move_base_successes", 0) or 0)
        except (TypeError, ValueError):
            successes = 0
        if successes <= 0:
            continue
        return {
            "observed": True,
            "event": "run_stop.summary",
            "ros_time": record.get("ros_time"),
            "goal_id": None,
            "status": 3,
            "status_name": "SUCCEEDED",
            "text": "move_base_successes=%d" % successes,
        }
    return None


def _evidence_completeness(snapshot, run_dir=None, run_timestamp=None):
    """Check whether a failure artifact is actionable without reading the log.

    ``complete`` covers both field-level evidence and the durable artifact
    contract. ``unavailable_channels`` remains explicit because a real TF or
    planner outage is valuable diagnosis even though it makes the episode
    incomplete.
    """
    if not isinstance(snapshot, dict):
        return {
            "complete": False,
            "missing": ["failure_artifact"],
            "unavailable_channels": [],
            "truncated": [],
        }
    missing = []
    contract = {}
    failure_id = str(snapshot.get("failure_id", "")).strip()
    expected_run = str(run_timestamp or "").strip()
    failure_id_match = _FAILURE_ID_RE.fullmatch(failure_id)
    if failure_id_match is None:
        missing.append("failure_id.format")
    elif expected_run and failure_id_match.group("run") != expected_run:
        missing.append("failure_id.run_timestamp")

    artifact_path = snapshot.get("artifact_path")
    artifact = Path(str(artifact_path)).expanduser() if artifact_path else None
    contract["artifact_path_present"] = artifact is not None
    contract["artifact_exists"] = bool(artifact and artifact.is_file())
    contract["artifact_name_valid"] = bool(
        artifact and _FAILURE_ARTIFACT_RE.fullmatch(artifact.name)
    )
    contract["artifact_in_run_directory"] = True
    if artifact is None:
        missing.append("artifact_path")
    else:
        if not contract["artifact_exists"]:
            missing.append("artifact_path.exists")
        if not contract["artifact_name_valid"]:
            missing.append("artifact_path.name")
        if run_dir is not None:
            try:
                contract["artifact_in_run_directory"] = (
                    artifact.resolve().parent == Path(run_dir).resolve()
                )
            except OSError:
                contract["artifact_in_run_directory"] = False
            if not contract["artifact_in_run_directory"]:
                missing.append("artifact_path.run_directory")

    if not isinstance(snapshot.get("run_context"), dict):
        missing.append("run_context")
    else:
        for key in ("experiment", "resolved_params"):
            if not isinstance(snapshot["run_context"].get(key), dict):
                missing.append("run_context.%s" % key)
    causal_route = snapshot.get("causal_route")
    diagnosis = snapshot.get("diagnosis") or snapshot.get("diagnosis_at_trigger")
    route_id = None
    if isinstance(causal_route, dict):
        route_id = causal_route.get("route_id") or causal_route.get("active_route_id")
    if route_id in (None, "", 0, "0") and isinstance(diagnosis, dict):
        route_id = diagnosis.get("route_id")
    trigger_sample = snapshot.get("trigger_sample")
    end_sample = snapshot.get("end_sample")
    trigger_sample = trigger_sample if isinstance(trigger_sample, dict) else {}
    controller_goal_id = (
        (trigger_sample.get("move_base_feedback") or {}).get("goal_id")
        if isinstance(trigger_sample.get("move_base_feedback"), dict)
        else None
    )
    controller_identity = bool(
        route_id in (None, "", 0, "0")
        and trigger_sample.get("goal") is not None
        and (
            controller_goal_id
            or trigger_sample.get("goal_transaction_id") not in (None, "", 0, "0")
            or _is_pre_dispatch_planner_failure(
                snapshot, trigger_sample, diagnosis
            )
        )
    )
    pre_dispatch = _is_pre_dispatch_planner_failure(
        snapshot, trigger_sample, diagnosis
    )
    if route_id in (None, "", 0, "0") and not controller_identity:
        missing.append("causal_route.route_id")
    missing.extend(
        "trigger_sample.%s" % key
        for key in _sample_missing(trigger_sample, allow_pre_dispatch=pre_dispatch)
    )
    missing.extend(
        "end_sample.%s" % key
        for key in _sample_missing(end_sample, allow_pre_dispatch=pre_dispatch)
    )

    # A route ID alone cannot distinguish a stale action after a handoff. The
    # snapshot needs at least one route/goal transaction identity.
    transaction_values = []
    for container in (causal_route, diagnosis, trigger_sample):
        if not isinstance(container, dict):
            continue
        for key in (
            "transaction_id", "active_goal_transaction_id",
            "latest_goal_transaction_id", "goal_transaction_id",
            "target_transaction_id",
        ):
            value = container.get(key)
            if value not in (None, "", 0, "0"):
                transaction_values.append(value)
    if not transaction_values and not pre_dispatch:
        missing.append("goal_or_route_transaction")

    unavailable_channels = sorted(
        set(_channel_health_gaps(trigger_sample) + _channel_health_gaps(end_sample))
    )
    if pre_dispatch:
        # These channels are created by move_base/TEB only after a validated
        # goal is dispatched. Their absence is the expected evidence of a
        # planner-boundary rejection, not a damaged snapshot.
        unavailable_channels = [
            name
            for name in unavailable_channels
            if name
            not in {"goal", "route_identity", "teb_feedback", "move_base_feedback"}
        ]
    truncated = _walk_truncated(snapshot)
    critical_truncated = _critical_truncations(snapshot)
    contract["failure_id"] = failure_id
    contract["failure_id_valid"] = (
        failure_id_match is not None
        and (
            not expected_run
            or failure_id_match.group("run") == expected_run
        )
    )
    contract["required_channels"] = [
        "pose", "goal", "cmd_vel", "scan", "navfn_plan", "teb_feedback",
        "global_costmap", "local_costmap", "move_base_feedback",
        "route_identity",
    ]
    contract["execution_boundary"] = (
        "pre_dispatch_planner_rejection" if pre_dispatch else "controller_execution"
    )
    return {
        "complete": not missing and not unavailable_channels and not critical_truncated,
        "missing": sorted(set(missing)),
        "unavailable_channels": unavailable_channels,
        "contract": contract,
        "identity_mode": "controller_goal" if controller_identity else "graph_route",
        "truncated_count": len(truncated),
        "truncated_paths": truncated[:12],
        "critical_truncated_count": len(critical_truncated),
        "critical_truncated_paths": critical_truncated[:12],
    }


def _compact_episode(
    episode, snapshots, run_dir=None, run_timestamp=None, event_contract=None
):
    failure_id = episode.get("failure_id")
    snapshot = snapshots.get(failure_id) if isinstance(snapshots, dict) else None
    diagnosis = episode.get("technical_diagnosis") or {}
    if not isinstance(diagnosis, dict):
        diagnosis = {}
    causal = snapshot.get("causal_route") if isinstance(snapshot, dict) else {}
    causal = causal if isinstance(causal, dict) else {}
    route_id = (
        diagnosis.get("route_id")
        or causal.get("route_id")
        or causal.get("active_route_id")
    )
    route_kind = (
        diagnosis.get("route_kind")
        or causal.get("route_kind")
        or causal.get("active_route_kind")
    )
    trigger_sample = snapshot.get("trigger_sample") if isinstance(snapshot, dict) else {}
    trigger_sample = trigger_sample if isinstance(trigger_sample, dict) else {}
    move_base_feedback = trigger_sample.get("move_base_feedback")
    move_base_feedback = (
        move_base_feedback if isinstance(move_base_feedback, dict) else {}
    )
    identity_mode = "graph_route"
    identity = route_id
    if route_id in (None, "", 0, "0") and trigger_sample.get("goal") is not None:
        identity_mode = "controller_goal"
        identity = {
            "goal": trigger_sample.get("goal"),
            "move_base_goal_id": move_base_feedback.get("goal_id"),
        }
    completeness = _evidence_completeness(
        snapshot, run_dir=run_dir, run_timestamp=run_timestamp
    )
    if event_contract is not None:
        completeness["event_contract"] = event_contract
        completeness["complete"] = bool(
            completeness.get("complete") and event_contract.get("complete")
        )
    return {
        "failure_id": failure_id,
        "status": episode.get("status"),
        "trigger": episode.get("trigger"),
        "source": episode.get("source"),
        "classification": episode.get("classification"),
        "confidence": episode.get("confidence"),
        "route_id": route_id,
        "route_kind": route_kind,
        "identity_mode": identity_mode,
        "identity": identity,
        "primary_cause": diagnosis.get("primary_cause"),
        "layer": diagnosis.get("layer"),
        "explanation": diagnosis.get("explanation"),
        "command_chain": diagnosis.get("command_chain"),
        "execution_boundary": (
            completeness.get("contract", {}).get("execution_boundary")
        ),
        "started_wall_elapsed_seconds": episode.get("started_wall_elapsed_seconds"),
        "duration_seconds": episode.get("duration_seconds"),
        "artifact_path": episode.get("artifact_path")
        or (snapshot or {}).get("artifact_path"),
        "trigger_pose": diagnosis.get("spatial", {}).get("pose")
        if isinstance(diagnosis.get("spatial"), dict)
        else None,
        "trigger_goal": diagnosis.get("spatial", {}).get("goal")
        if isinstance(diagnosis.get("spatial"), dict)
        else None,
        "evidence": completeness,
    }


def _effective_trial_end_diagnostic(diagnostic, parameter_evidence):
    """Correct a legacy timeout label when a terminal callback followed its sample."""
    diagnostic = diagnostic if isinstance(diagnostic, dict) else {}
    result = dict(diagnostic)
    sample = diagnostic.get("latest_sample")
    sample = sample if isinstance(sample, dict) else {}
    tolerance = diagnostic.get("goal_tolerance_m")
    if tolerance is None:
        tolerance = (parameter_evidence.get("values") or {}).get(
            "teb_xy_goal_tolerance"
        )
    try:
        distance = float(sample.get("distance_to_goal"))
        tolerance = float(tolerance)
    except (TypeError, ValueError):
        distance = None
        tolerance = None
    terminal_boundary = bool(diagnostic.get("terminal_boundary_observed"))
    if not terminal_boundary:
        for event in diagnostic.get("recent_events") or []:
            if not isinstance(event, dict):
                continue
            name = str(event.get("event") or "").lower()
            reason = str(event.get("reason") or "").lower()
            bridge_event = str(event.get("bridge_event") or "").lower()
            if (
                name in {"persistent_execution_terminal", "task_done"}
                or "terminal" in bridge_event
                or reason in {
                    "source_viewpoint_arrived",
                    "endpoint_coverage_recorded",
                    "frontier_observed_at_standoff",
                }
            ):
                terminal_boundary = True
                break
    if (
        str(result.get("state") or "") == "stall_candidate"
        and terminal_boundary
        and (
            distance is None
            or tolerance is None
            or distance <= tolerance
        )
    ):
        result["reclassified_from"] = result.get("state")
        result["state"] = "terminal_settle"
        result["reason"] = "terminal_boundary_observed_after_last_sample"
    result["terminal_boundary_observed"] = terminal_boundary
    if result.get("goal_tolerance_m") is None and tolerance is not None:
        result["goal_tolerance_m"] = tolerance
    return result


def _outcome_summary(outcome, episodes, startup, trial_end, success_evidence):
    """Return one machine-readable explanation of the run boundary.

    ``failure_summary.episodes`` is intentionally detailed and may contain
    several episodes.  Runners and experiment aggregators need one stable,
    bounded record for the first actionable outcome without reopening the
    artifact or reimplementing episode selection.
    """
    if startup is not None:
        return {
            "kind": "startup_failure",
            "failure_id": startup.get("failure_id"),
            "cause": startup.get("reason"),
            "layer": "startup_readiness",
            "confidence": "high",
            "route_id": None,
            "route_kind": None,
            "evidence_complete": True,
            "explanation": "navigation did not reach the declared startup readiness boundary",
        }
    if episodes:
        primary = episodes[0] if isinstance(episodes[0], dict) else {}
        evidence = primary.get("evidence") or {}
        return {
            "kind": "failure",
            "failure_id": primary.get("failure_id"),
            "cause": primary.get("primary_cause") or primary.get("classification"),
            "layer": primary.get("layer"),
            "confidence": primary.get("confidence"),
            "route_id": primary.get("route_id"),
            "route_kind": primary.get("route_kind"),
            "evidence_complete": bool(evidence.get("complete")),
            "explanation": primary.get("explanation"),
        }
    if trial_end is not None:
        diagnostic = trial_end.get("diagnostic")
        diagnostic = diagnostic if isinstance(diagnostic, dict) else {}
        return {
            "kind": "trial_end_diagnostic",
            "failure_id": trial_end.get("termination_id") or diagnostic.get("termination_id"),
            "cause": diagnostic.get("reason") or trial_end.get("reason"),
            "layer": "trial_boundary",
            "confidence": "medium",
            "route_id": diagnostic.get("active_route_id"),
            "route_kind": diagnostic.get("active_route_kind"),
            "evidence_complete": bool((diagnostic.get("evidence") or {}).get("complete")),
            "explanation": diagnostic.get("interpretation") or diagnostic.get("reason"),
        }
    if success_evidence is not None:
        return {
            "kind": "success",
            "failure_id": None,
            "cause": "goal_succeeded",
            "layer": "move_base",
            "confidence": "high",
            "route_id": None,
            "route_kind": None,
            "evidence_complete": True,
            "explanation": "a successful action terminal was observed",
        }
    return {
        "kind": str(outcome or "completed_or_in_progress"),
        "failure_id": None,
        "cause": None,
        "layer": None,
        "confidence": "low",
        "route_id": None,
        "route_kind": None,
        "evidence_complete": False,
        "explanation": "no terminal failure, startup boundary, or successful action was observed",
    }


def build_report(path: Path | None):
    run_dir, metrics_path = _resolve_run(path)
    if run_dir is None:
        raise FileNotFoundError("navigation run directory not found")
    lifecycle_path = run_dir / (run_dir.name + "_lifecycle.log")
    lifecycle = _parse_lifecycle(lifecycle_path)
    records, metrics_bounded = _load_metrics_records(metrics_path)
    snapshots = (
        load_failure_snapshots(metrics_path) if metrics_path is not None else {}
    )
    episodes = failure_episode_summary(records, snapshots)
    event_contracts = {
        item.get("failure_id"): _failure_event_contract(
            records, item.get("failure_id")
        )
        for item in episodes.get("episodes", [])
        if item.get("failure_id")
    }
    compact_failures = [
        _compact_episode(
            item,
            snapshots,
            run_dir=run_dir,
            run_timestamp=run_dir.name,
            event_contract=event_contracts.get(item.get("failure_id")),
        )
        for item in episodes.get("episodes", [])
    ]
    # A bounded metrics read may begin after ``failure_started`` or
    # ``failure_snapshot_ready``. The standalone artifact is authoritative in
    # that case; synthesize the missing compact episode instead of reporting a
    # clean run merely because its log grew beyond the read window.
    known_failure_ids = {item.get("failure_id") for item in compact_failures}
    for failure_id, snapshot in snapshots.items():
        if failure_id in known_failure_ids or not isinstance(snapshot, dict):
            continue
        classification = snapshot.get("classification") or {}
        if not isinstance(classification, dict):
            classification = {}
        pseudo_episode = {
            "failure_id": failure_id,
            "status": "closed",
            "trigger": snapshot.get("trigger"),
            "source": snapshot.get("source"),
            "classification": classification.get("label", "unknown"),
            "confidence": classification.get("confidence", "low"),
            "started_wall_elapsed_seconds": snapshot.get(
                "started_wall_elapsed_seconds"
            ),
            "duration_seconds": snapshot.get("post_window_seconds"),
            "artifact_path": snapshot.get("artifact_path"),
            "technical_diagnosis": snapshot.get("diagnosis"),
        }
        compact_failures.append(
            _compact_episode(
                pseudo_episode,
                snapshots,
                run_dir=run_dir,
                run_timestamp=run_dir.name,
                event_contract=event_contracts.get(failure_id),
            )
        )
    compact_failures.sort(
        key=lambda item: (
            item.get("started_wall_elapsed_seconds") is None,
            item.get("started_wall_elapsed_seconds") or float("inf"),
        )
    )
    startup_path = run_dir / (run_dir.name + "_startup_failure.json")
    trial_end_path = run_dir / (run_dir.name + "_trial_end_diagnostic.json")
    startup = load_startup_failure(metrics_path) if metrics_path else None
    if startup is None and startup_path.is_file():
        try:
            payload = json.loads(startup_path.read_text(encoding="utf-8"))
            startup = payload if isinstance(payload, dict) else None
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            startup = {
                "failure_kind": "startup_failure",
                "failure_id": run_dir.name + "-S0001",
                "reason": "invalid_startup_failure_artifact",
                "artifact_path": str(startup_path),
            }
    trial_end = None
    if trial_end_path.is_file():
        try:
            payload = json.loads(trial_end_path.read_text(encoding="utf-8"))
            trial_end = payload if isinstance(payload, dict) else None
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            trial_end = {"artifact_path": str(trial_end_path), "read_error": True}

    trial_end_diagnostic = (trial_end or {}).get("diagnostic")
    trial_end_diagnostic = (
        trial_end_diagnostic if isinstance(trial_end_diagnostic, dict) else {}
    )
    parameter_evidence = _merge_startup_parameter_evidence(
        _load_parameter_evidence(run_dir), records, snapshots
    )
    trial_end_diagnostic = _effective_trial_end_diagnostic(
        trial_end_diagnostic, parameter_evidence
    )
    trial_end_sample = trial_end_diagnostic.get("latest_sample")
    trial_end_sample = trial_end_sample if isinstance(trial_end_sample, dict) else {}
    success_evidence = _success_evidence(records)

    logs = []
    for item in sorted(run_dir.glob("*.log")):
        try:
            size = item.stat().st_size
        except OSError:
            size = 0
        logs.append({"name": item.name, "path": str(item), "bytes": size})

    if startup is not None:
        outcome = "startup_failure"
    elif compact_failures or snapshots:
        outcome = "failure"
    elif trial_end is not None:
        # A long exploration can contain many successful sub-route actions
        # before its final timeout.  ``move_base SUCCEEDED`` is therefore not
        # a run-level success boundary when a trial-end artifact exists; the
        # artifact's active-route/progress state is the authoritative outcome.
        outcome = "trial_end_diagnostic"
    elif success_evidence is not None:
        outcome = "success"
    else:
        outcome = "completed_or_in_progress"

    outcome_summary = _outcome_summary(
        outcome,
        compact_failures,
        startup,
        trial_end,
        success_evidence,
    )

    return {
        "schema_version": 1,
        "report_kind": "navigation_failure_report",
        "created_at": _datetime.datetime.now(_datetime.timezone.utc).isoformat(),
        "run_timestamp": run_dir.name,
        "run_directory": str(run_dir),
        "metrics_log": None if metrics_path is None else str(metrics_path),
        "metrics_records_loaded": len(records),
        "metrics_read_bounded": metrics_bounded,
        "outcome": outcome,
        "outcome_summary": outcome_summary,
        "success_evidence": success_evidence,
        "phase_timings": _phase_timings(run_dir, records, lifecycle),
        "parameter_evidence": parameter_evidence,
        "failure_summary": {
            "episode_count": len(compact_failures),
            "closed_count": sum(
                item.get("status") == "closed" for item in compact_failures
            ),
            "open_count": sum(
                item.get("status") == "open_at_log_end" for item in compact_failures
            ),
            "episodes": compact_failures,
        },
        "startup_failure": (
            None
            if startup is None
            else {
                "failure_id": startup.get("failure_id"),
                "reason": startup.get("reason"),
                "artifact_path": startup.get("artifact_path") or str(startup_path),
            }
        ),
        "trial_end_diagnostic": (
            None
            if trial_end is None
            else {
                "artifact_path": trial_end.get("artifact_path") or str(trial_end_path),
                "termination_id": trial_end_diagnostic.get("termination_id")
                or trial_end.get("termination_id"),
                "classification": trial_end_diagnostic.get("classification")
                or trial_end.get("classification"),
                "state": trial_end_diagnostic.get("state"),
                "reason": trial_end_diagnostic.get("reason"),
                "route_id": trial_end_diagnostic.get("active_route_id"),
                "route_kind": trial_end_diagnostic.get("active_route_kind"),
                # Controller-only endpoint trials have no graph route ID. Keep
                # their physical pose, goal, and residual distance directly in
                # the report so the isolation experiment remains actionable.
                "pose": trial_end_sample.get("pose"),
                "goal": trial_end_sample.get("goal"),
                "distance_to_goal": trial_end_sample.get("distance_to_goal"),
                "plan_available": trial_end_diagnostic.get("plan_available"),
                "goal_tolerance_m": (
                    trial_end_diagnostic.get("goal_tolerance_m")
                    or parameter_evidence.get("values", {}).get(
                        "teb_xy_goal_tolerance"
                    )
                ),
                "progress": trial_end_diagnostic.get("progress") or {},
                "recent_samples": trial_end_diagnostic.get("recent_samples") or [],
                "route_transitions": (
                    (trial_end_diagnostic.get("progress") or {}).get(
                        "route_transitions", []
                    )
                ),
                "evidence": trial_end_diagnostic.get("evidence") or {},
            }
        ),
        "logs": logs,
        "lifecycle_event_count": len(lifecycle),
        "next_commands": {
            "full_metrics": (
                None
                if metrics_path is None
                else "python3 scripts/tools/analyze_navigation_metrics.py %s --failures"
                % metrics_path
            ),
            "event_trace": (
                None
                if metrics_path is None
                else "python3 scripts/tools/analyze_navigation_metrics.py %s --diagnose"
                % metrics_path
            ),
        },
    }


def _format_seconds(value):
    if value is None:
        return "n/a"
    return "%.3f s" % float(value)


def render_markdown(report):
    timing = report.get("phase_timings") or {}
    lines = [
        "# Navigation Failure Report",
        "",
        "- run: `%s`" % report.get("run_timestamp"),
        "- outcome: `%s`" % report.get("outcome"),
        "- directory: `%s`" % report.get("run_directory"),
        "",
        "## Outcome Summary",
        "",
    ]
    outcome_summary = report.get("outcome_summary") or {}
    lines.extend(
        [
            "- kind: `%s`" % (outcome_summary.get("kind") or "unknown"),
            "- failure/termination ID: `%s`"
            % (outcome_summary.get("failure_id") or "n/a"),
            "- cause: `%s`" % (outcome_summary.get("cause") or "n/a"),
            "- layer: `%s`" % (outcome_summary.get("layer") or "n/a"),
            "- confidence: `%s`"
            % (outcome_summary.get("confidence") or "n/a"),
            "- evidence: `%s`"
            % ("complete" if outcome_summary.get("evidence_complete") else "incomplete"),
            "- explanation: %s"
            % (outcome_summary.get("explanation") or "n/a"),
            "",
            "## Timing",
            "",
        ]
    )
    lines.extend(
        [
        "- launcher to ready: %s" % _format_seconds(timing.get("launcher_to_ready_seconds")),
        "- navigation window: %s" % _format_seconds(timing.get("navigation_window_seconds")),
        "- cleanup: %s" % _format_seconds(timing.get("cleanup_seconds")),
        "- total wall time: %s" % _format_seconds(timing.get("total_wall_seconds")),
        "",
        ]
    )
    success = report.get("success_evidence")
    if success:
        lines.extend(
            [
                "## Success",
                "",
                "- action terminal: `%s` at ROS time `%s` (goal `%s`)"
                % (
                    success.get("status_name") or "SUCCEEDED",
                    success.get("ros_time") if success.get("ros_time") is not None else "n/a",
                    success.get("goal_id") or "n/a",
                ),
                "",
            ]
        )
    lines.extend(
        [
        "## Runtime Parameters",
        "",
        ]
    )
    parameter_evidence = report.get("parameter_evidence") or {}
    parameter_values = parameter_evidence.get("values") or {}
    if parameter_values:
        for key in sorted(parameter_values):
            lines.append("- `%s`: `%s`" % (key, parameter_values[key]))
    else:
        lines.append("No runtime parameter dump was found.")
    lines.extend(["", "## Failures", ""])
    episodes = (report.get("failure_summary") or {}).get("episodes") or []
    if not episodes:
        lines.append("No closed or open failure episode was recorded.")
    else:
        for item in episodes:
            evidence = item.get("evidence") or {}
            lines.append(
                "- `%s`: `%s` / `%s`, route `%s` (%s), identity `%s`=%s, "
                "cause `%s`, layer `%s`; "
                "evidence `%s`"
                % (
                    item.get("failure_id"),
                    item.get("classification"),
                    item.get("confidence"),
                    item.get("route_id") or "n/a",
                    item.get("route_kind") or "n/a",
                    item.get("identity_mode") or "graph_route",
                    item.get("identity") or "n/a",
                    item.get("primary_cause") or "n/a",
                    item.get("layer") or "n/a",
                    "complete" if evidence.get("complete") else "INCOMPLETE",
                )
            )
            if item.get("artifact_path"):
                lines.append("  artifact: `%s`" % item["artifact_path"])
            if item.get("execution_boundary"):
                lines.append(
                    "  execution boundary: `%s`"
                    % item["execution_boundary"]
                )
            if item.get("trigger") or item.get("source"):
                lines.append(
                    "  trigger: `%s` from `%s`"
                    % (item.get("trigger") or "n/a", item.get("source") or "n/a")
                )
            if item.get("explanation"):
                lines.append("  boundary explanation: %s" % item["explanation"])
            if item.get("trigger_pose") or item.get("trigger_goal"):
                lines.append(
                    "  trigger pose: `%s`, goal: `%s`"
                    % (item.get("trigger_pose") or "n/a", item.get("trigger_goal") or "n/a")
                )
            if item.get("command_chain"):
                lines.append(
                    "  command chain: `%s`"
                    % json.dumps(
                        item["command_chain"],
                        ensure_ascii=False,
                        sort_keys=True,
                        separators=(",", ":"),
                    )
                )
            trigger_sample = {}
            evidence_payload = item.get("evidence") or {}
            if isinstance(evidence_payload, dict):
                trigger_sample = evidence_payload.get("trigger_sample") or {}
            channel_health = (
                trigger_sample.get("channel_health")
                if isinstance(trigger_sample, dict)
                else None
            )
            if isinstance(channel_health, dict):
                missing_channels = sorted(
                    name for name, state in channel_health.items()
                    if isinstance(state, dict) and not state.get("available")
                )
                lines.append(
                    "  trigger channels: `%s`"
                    % (
                        "all-present"
                        if not missing_channels
                        else "missing=" + ",".join(missing_channels)
                    )
                )
            missing = evidence.get("missing") or []
            if missing:
                lines.append("  missing: `%s`" % ", ".join(missing))
            unavailable = evidence.get("unavailable_channels") or []
            if unavailable:
                lines.append(
                    "  unavailable channels at boundary: `%s`"
                    % ", ".join(unavailable)
                )
            event_contract = evidence.get("event_contract") or {}
            event_missing = event_contract.get("missing") or []
            if event_missing:
                lines.append(
                    "  lifecycle evidence missing: `%s`"
                    % ", ".join(event_missing)
                )
            if evidence.get("truncated_count"):
                lines.append(
                    "  truncated fields: %d (first: `%s`)"
                    % (
                        evidence["truncated_count"],
                        (evidence.get("truncated_paths") or ["n/a"])[0],
                    )
                )
            if evidence.get("critical_truncated_count"):
                lines.append(
                    "  critical fields truncated: %d (first: `%s`)"
                    % (
                        evidence["critical_truncated_count"],
                        (evidence.get("critical_truncated_paths") or ["n/a"])[0],
                    )
                )

    startup = report.get("startup_failure")
    if startup:
        lines.extend(
            [
                "",
                "## Startup Failure",
                "",
                "- `%s`: `%s`" % (startup.get("failure_id"), startup.get("reason")),
                "- artifact: `%s`" % startup.get("artifact_path"),
            ]
        )
    trial_end = report.get("trial_end_diagnostic")
    if trial_end:
        lines.extend(
            [
                "",
                "## Trial-End Diagnostic",
                "",
                "- termination: `%s` / `%s`"
                % (
                    trial_end.get("termination_id") or "n/a",
                    trial_end.get("classification") or "n/a",
                ),
                "- state: `%s`" % trial_end.get("state"),
                "- reason: `%s`" % trial_end.get("reason"),
                "- route: `%s` (%s)"
                % (trial_end.get("route_id") or "n/a", trial_end.get("route_kind") or "n/a"),
                "- pose: `%s`, goal: `%s`, distance: `%s`"
                % (
                    trial_end.get("pose") or "n/a",
                    trial_end.get("goal") or "n/a",
                    trial_end.get("distance_to_goal")
                    if trial_end.get("distance_to_goal") is not None
                    else "n/a",
                ),
                "- goal tolerance: `%s`" % (
                    trial_end.get("goal_tolerance_m")
                    if trial_end.get("goal_tolerance_m") is not None
                    else "n/a"
                ),
                "- artifact: `%s`" % trial_end.get("artifact_path"),
            ]
        )
        progress = trial_end.get("progress") or {}
        distances = progress.get("distance_to_goal_m") or {}
        lines.extend(
            [
                "- aggregate progress: `%s` -> `%s` m (toward goal `%s` m), "
                "pose displacement `%s` m; interpretation `%s`"
                % (
                    distances.get("first", "n/a"),
                    distances.get("last", "n/a"),
                    distances.get("progress_toward_goal", "n/a"),
                    progress.get("pose_displacement_m", "n/a"),
                    progress.get("interpretation", "n/a"),
                ),
                "- route transitions in final window: `%s`"
                % progress.get("route_transition_count", 0),
                "- evidence: `%s` (samples `%s`)"
                % (
                    "complete"
                    if (trial_end.get("evidence") or {}).get("complete")
                    else "INCOMPLETE",
                    len(trial_end.get("recent_samples") or []),
                ),
            ]
        )
        active_route = progress.get("active_route")
        if isinstance(active_route, dict):
            active_distances = active_route.get("distance_to_goal_m") or {}
            lines.append(
                "- active route `%s` (%s, transaction `%s`): `%s` -> `%s` m "
                "(toward goal `%s` m), pose displacement `%s` m; interpretation `%s`"
                % (
                    active_route.get("route_id") or "n/a",
                    active_route.get("route_kind") or "n/a",
                    active_route.get("goal_transaction_id") or "n/a",
                    active_distances.get("first", "n/a"),
                    active_distances.get("last", "n/a"),
                    active_distances.get("progress_toward_goal", "n/a"),
                    active_route.get("pose_displacement_m", "n/a"),
                    active_route.get("interpretation", "n/a"),
                )
            )
            segments = progress.get("route_segments") or []
            if len(segments) > 1:
                lines.append(
                    "- route-local segments: `%s` (the aggregate window spans "
                    "multiple goals)" % len(segments)
                )

    lines.extend(["", "## Next Commands", ""])
    for label, command in (report.get("next_commands") or {}).items():
        if command:
            lines.append("- %s: `%s`" % (label, command))
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", nargs="?", type=Path, help="run directory or metrics log")
    parser.add_argument("--json", action="store_true", help="print report JSON")
    parser.add_argument("--output-json", type=Path, help="write report JSON")
    parser.add_argument("--output-markdown", type=Path, help="write report Markdown")
    args = parser.parse_args(argv)
    try:
        report = build_report(args.run)
    except (FileNotFoundError, OSError, ValueError) as exc:
        parser.error(str(exc))
    rendered_json = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    rendered_markdown = render_markdown(report)
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(rendered_json, encoding="utf-8")
    if args.output_markdown:
        args.output_markdown.parent.mkdir(parents=True, exist_ok=True)
        args.output_markdown.write_text(rendered_markdown, encoding="utf-8")
    if args.json:
        print(rendered_json, end="")
    else:
        print(rendered_markdown, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
