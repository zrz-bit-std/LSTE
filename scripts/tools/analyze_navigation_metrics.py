#!/usr/bin/env python3
"""Summarize one or more structured navigation metric logs.

The live metrics node writes one JSON object after ``data=`` on every log
line.  This tool deliberately consumes those logs instead of scraping tmux or
ROS console output, so comparisons remain reproducible after a run ends.
"""

import argparse
import datetime
import json
import math
from pathlib import Path
import re


def latest_log(root):
    candidates = sorted(root.glob("*/*_navigation_metrics.log"))
    return candidates[-1] if candidates else None


def load_records(path):
    records = []
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            marker = " data="
            if marker not in line:
                continue
            try:
                record = json.loads(line.split(marker, 1)[1])
            except (TypeError, ValueError):
                continue
            event = line.split(" event=", 1)[1].split(" data=", 1)[0] if " event=" in line else ""
            record["_event"] = event
            record["_line"] = line.rstrip()
            records.append(record)
    return records


def load_failure_snapshots(metrics_path):
    """Load failure artifacts, with the append-only evidence log as fallback.

    A closed episode is written to ``<run>_failure_NNNN.json`` before the
    corresponding log event.  Reading those bounded files first avoids
    scanning a large high-rate metrics stream and, importantly, preserves the
    direct artifact path in reports.  Older runs may only have the evidence
    log, so its ``failure_snapshot`` record remains a compatibility source.
    """
    metrics_path = Path(metrics_path)
    snapshots = {}
    artifact_pattern = metrics_path.parent.name + "_failure_*.json"
    artifact_name = re.compile(
        r"^%s_failure_[0-9]{4}\.json$" % re.escape(metrics_path.parent.name)
    )
    for path in sorted(metrics_path.parent.glob(artifact_pattern)):
        # The report generator writes ``*_failure_report.json`` beside the
        # episode snapshots.  Only numbered episode artifacts are valid input
        # here; otherwise a regenerated report is scanned as a fake episode.
        if artifact_name.fullmatch(path.name) is None:
            continue
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            continue
        if not isinstance(payload, dict):
            continue
        failure_id = str(payload.get("failure_id", "")).strip()
        if failure_id:
            snapshots[failure_id] = payload

    evidence_path = metrics_path.parent / (
        metrics_path.parent.name + "_failure_evidence.log"
    )
    if evidence_path.is_file():
        try:
            stream = evidence_path.open(
                "r", encoding="utf-8", errors="replace"
            )
        except OSError:
            stream = None
        if stream is not None:
            with stream:
                for line in stream:
                    if (
                        " event=failure_snapshot " not in line
                        or " data=" not in line
                    ):
                        continue
                    try:
                        payload = json.loads(line.split(" data=", 1)[1])
                    except (TypeError, ValueError, json.JSONDecodeError):
                        continue
                    failure_id = str(payload.get("failure_id", "")).strip()
                    # Keep the full artifact when available.  ``setdefault``
                    # makes the log a fallback for pre-artifact runs only.
                    if failure_id:
                        snapshots.setdefault(failure_id, payload)
    return snapshots


def load_startup_failure(metrics_path):
    """Load the sibling startup-gate artifact, if this run has one."""
    metrics_path = Path(metrics_path)
    path = metrics_path.parent / (metrics_path.parent.name + "_startup_failure.json")
    if not path.is_file():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, TypeError, ValueError, json.JSONDecodeError):
        return {
            "failure_kind": "startup_failure",
            "artifact_path": str(path),
            "read_error": "invalid_json",
        }
    return payload if isinstance(payload, dict) else {
        "failure_kind": "startup_failure",
        "artifact_path": str(path),
        "read_error": "invalid_payload",
    }


def _compact_failure_sample(sample):
    """Return investigator-facing fields without duplicating the full ring."""
    if not isinstance(sample, dict):
        return None
    route_context = sample.get("route_context")
    if isinstance(route_context, dict):
        route_context = route_context.get("route", route_context)
    result = {
        key: sample.get(key)
        for key in (
            "ros_time", "wall_elapsed_seconds", "pose", "pose_frame", "goal",
            "goal_frame", "pose_goal_frame", "pose_transform_available",
            "distance_to_goal", "goal_source",
            "goal_transaction_id", "cmd_vel", "teb_cmd", "teb_planner_cmd",
            "teb_status", "teb_feedback_age_seconds", "mux_status_age_seconds",
            "move_base_status", "scan",
            "obstacle_clearance_threshold", "navfn_plan",
            "navfn_path_remaining_m", "navfn_path_endpoint", "teb_feedback",
            "channel_health",
            "teb_turn_supervisor", "teb_turn_supervisor_last_event",
            "teb_turn_supervisor_events",
            "global_costmap", "local_costmap", "global_costmap_window",
            "local_costmap_window", "move_base_feedback", "recovery",
            "route_identity",
            "controller", "bridge_active", "navigation_hold", "state",
            "frontier_route_unavailable_count",
            "frontier_route_unavailable_identity",
            "frontier_route_unavailable_latch",
            "frontier_route_unavailable_duration_seconds",
        )
        if key in sample
    }
    if route_context:
        result["route"] = route_context
    return result


def _compact_route_context(context):
    if not isinstance(context, dict):
        return context
    route = context.get("route")
    return route if isinstance(route, dict) else context


def _failure_snapshot_evidence(snapshot):
    if not isinstance(snapshot, dict):
        return None
    return {
        "artifact_path": snapshot.get("artifact_path"),
        "run_context": snapshot.get("run_context"),
        "trigger": snapshot.get("trigger"),
        "source": snapshot.get("source"),
        "resolution": snapshot.get("resolution"),
        "details": snapshot.get("details"),
        "causal_route": snapshot.get("causal_route"),
        "diagnosis": snapshot.get("diagnosis"),
        "diagnosis_at_trigger": snapshot.get("diagnosis_at_trigger"),
        "diagnosis_at_end": snapshot.get("diagnosis_at_end"),
        "route_context_at_trigger": _compact_route_context(
            snapshot.get("route_context_at_trigger")
        ),
        "route_context_at_end": _compact_route_context(
            snapshot.get("route_context_at_end")
        ),
        "trigger_sample": _compact_failure_sample(snapshot.get("trigger_sample")),
        "end_sample": _compact_failure_sample(snapshot.get("end_sample")),
        "classification_initial": snapshot.get("classification_initial"),
        "classification_at_end": snapshot.get("classification_at_end"),
        "sample_count": snapshot.get("sample_count", 0),
    }


def _technical_diagnosis(snapshot):
    """Expose the causal reducer separately from the stored class label.

    Failure artifacts are immutable.  Their historical ``classification`` may
    have been produced by an older reducer, while ``diagnosis`` is the direct
    command/route evidence captured at the trigger boundary.  Keeping this
    field beside the label makes that distinction explicit to scripts and
    humans reading a summary.
    """
    if not isinstance(snapshot, dict):
        return None
    value = snapshot.get("diagnosis") or snapshot.get("diagnosis_at_trigger")
    if not isinstance(value, dict):
        return None
    # Older artifacts could sample after GlobalFrontier cleared its active
    # lease, leaving diagnosis.route_id=0 even though the trigger classifier
    # retained the causal route in classification.evidence.route. Normalize
    # that compatibility shape for human and machine summaries.
    normalized = dict(value)
    causal_route = snapshot.get("causal_route")
    if not isinstance(causal_route, dict):
        classification = snapshot.get("classification")
        classification = classification if isinstance(classification, dict) else {}
        evidence = classification.get("evidence")
        causal_route = evidence.get("route") if isinstance(evidence, dict) else None
    if not isinstance(causal_route, dict):
        causal_route = {}
    route_id = normalized.get("route_id")
    try:
        route_id_missing = route_id is None or float(route_id) <= 0.0
    except (TypeError, ValueError):
        route_id_missing = route_id in (None, "")
    if route_id_missing:
        for key in ("route_id", "active_route_id", "released_route_id"):
            if causal_route.get(key) not in (None, "", 0, "0"):
                normalized["route_id"] = causal_route[key]
                break
    if not normalized.get("route_kind"):
        normalized["route_kind"] = (
            causal_route.get("route_kind")
            or causal_route.get("active_route_kind")
            or causal_route.get("released_route_kind")
            or (normalized.get("execution_phase") or {}).get(
                "turn_supervisor_route_kind"
            )
        )
    if not normalized.get("graph_action"):
        normalized["graph_action"] = (
            causal_route.get("graph_action")
            or causal_route.get("action")
        )
    if not normalized.get("graph_action"):
        details = snapshot.get("details")
        details = details if isinstance(details, dict) else {}
        graph_plan = details.get("graph_route_plan")
        graph_plan = graph_plan if isinstance(graph_plan, dict) else {}
        transaction = details.get("graph_route_action_transaction")
        transaction = transaction if isinstance(transaction, dict) else {}
        transaction_plan = transaction.get("plan")
        transaction_plan = (
            transaction_plan if isinstance(transaction_plan, dict) else {}
        )
        normalized["graph_action"] = (
            graph_plan.get("action") or transaction_plan.get("action")
        )
    return normalized


def finite_min(values):
    values = [float(value) for value in values if value is not None]
    values = [value for value in values if math.isfinite(value)]
    return min(values) if values else None


def terminal_dispatch_latencies(records):
    """Pair each successful bridge terminal with its next action dispatch.

    This measures the execution-layer handoff gap directly from the same
    structured log used for the other metrics. Initial dispatches have no
    preceding terminal and are deliberately excluded.
    """
    latencies = []
    pending_terminal = None
    for record in records:
        event = record.get("_event", "")
        if (
            event == "teb_bridge_event"
            and record.get("bridge_event") == "terminal"
            and int(record.get("status", -1)) == 3
        ):
            pending_terminal = record.get("ros_time")
            continue
        if (
            pending_terminal is not None
            and event == "teb_bridge_event"
            and record.get("bridge_event") == "dispatch"
        ):
            dispatch_time = record.get("ros_time")
            if dispatch_time is not None:
                latency = float(dispatch_time) - float(pending_terminal)
                if 0.0 <= latency <= 30.0:
                    latencies.append(latency)
            pending_terminal = None
    return latencies


TARGET_LIFECYCLE_FIELDS = {
    "target_track_started": "first_seen_seconds",
    "target_follow_confirmed": "follow_confirmed_seconds",
    "target_approach_terminal": "approach_terminal_seconds",
    "target_close_confirmation_started": "close_started_seconds",
    "target_close_confirmed": "close_confirmed_seconds",
    "target_task_completed": "task_completed_seconds",
}


def target_lifecycle_sessions(records):
    """Rebuild target evidence by ``(task_id, target_track_id)``.

    The older sample counters are intentionally not used here: they retained
    the first target seen during a run, which can belong to a discarded track.
    Only the identity-bearing ``target_lifecycle`` records are allowed to
    produce end-to-end target timing.
    """
    sessions = {}
    unattributed = 0
    for record in records:
        if record.get("_event") != "target_lifecycle":
            continue
        lifecycle_event = str(record.get("lifecycle_event", ""))
        timestamp_field = TARGET_LIFECYCLE_FIELDS.get(lifecycle_event)
        if timestamp_field is None:
            continue
        task_id = str(record.get("task_id", "")).strip()
        track_id = str(record.get("target_track_id", "")).strip()
        if not task_id or not track_id:
            unattributed += 1
            continue
        try:
            event_seconds = float(record.get("event_seconds"))
        except (TypeError, ValueError):
            unattributed += 1
            continue
        key = (task_id, track_id)
        session = sessions.setdefault(
            key,
            {
                "task_id": task_id,
                "target_track_id": track_id,
                "first_seen_seconds": None,
                "follow_confirmed_seconds": None,
                "approach_terminal_seconds": None,
                "close_started_seconds": None,
                "close_confirmed_seconds": None,
                "task_completed_seconds": None,
            },
        )
        if session[timestamp_field] is None:
            session[timestamp_field] = event_seconds

    ordered = sorted(
        sessions.values(),
        key=lambda item: (
            item["task_completed_seconds"] is None,
            item["task_completed_seconds"]
            if item["task_completed_seconds"] is not None
            else item["first_seen_seconds"] or float("inf"),
        ),
    )
    for session in ordered:
        first_seen = session["first_seen_seconds"]
        follow = session["follow_confirmed_seconds"]
        approach = session["approach_terminal_seconds"]
        close = session["close_confirmed_seconds"]
        done = session["task_completed_seconds"]

        def interval(start, end):
            if start is None or end is None:
                return None
            return round(max(0.0, float(end) - float(start)), 3)

        session["first_seen_to_follow_seconds"] = interval(first_seen, follow)
        session["follow_to_approach_terminal_seconds"] = interval(follow, approach)
        session["approach_terminal_to_close_confirmed_seconds"] = interval(approach, close)
        session["close_confirmed_to_done_seconds"] = interval(close, done)
        session["first_seen_to_done_seconds"] = interval(first_seen, done)
        session["follow_to_done_seconds"] = interval(follow, done)
    return ordered, unattributed


def completed_target_session(sessions):
    completed = [item for item in sessions if item["task_completed_seconds"] is not None]
    return completed[-1] if completed else None


def active_target_session(sessions):
    """Return the newest identity-bound lifecycle that is not complete."""
    active = [item for item in sessions if item["task_completed_seconds"] is None]
    if not active:
        return None
    return max(active, key=lambda item: item["first_seen_seconds"] or float("-inf"))


def perception_tile_summary(records):
    """Summarize detector-side small-object fallback from formal run logs.

    The camera messages themselves are intentionally not written to the
    experiment log. ``perception_decision`` carries the source timestamp and
    model outcome needed to evaluate whether tiled inference found a target
    which the fast full-frame pass missed.
    """
    decisions = [
        record for record in records
        if record.get("_event") == "perception_decision"
    ]
    tile_runs = [
        record for record in decisions
        if bool(record.get("small_object_tile_search_ran"))
    ]
    tile_target_runs = [
        record for record in tile_runs
        if int(record.get("small_object_tile_search_target_candidates", 0) or 0) > 0
    ]
    tile_only_discoveries = [
        record for record in tile_target_runs
        if int(record.get("small_object_tile_search_primary_target_candidates", 0) or 0) == 0
    ]
    elapsed = [record.get("inference_elapsed_seconds") for record in decisions]
    tile_elapsed = [
        record.get("small_object_tile_search_elapsed_seconds") for record in tile_runs
    ]
    source_age = [record.get("source_image_age_seconds") for record in decisions]
    layouts = {}
    for record in tile_runs:
        layout = str(record.get("small_object_tile_search_layout", "uniform_grid"))
        summary = layouts.setdefault(
            layout,
            {
                "attempts": 0,
                "target_candidate_runs": 0,
                "tile_only_discovery_count": 0,
                "tile_elapsed_seconds": [],
            },
        )
        summary["attempts"] += 1
        target_candidates = int(
            record.get("small_object_tile_search_target_candidates", 0) or 0
        )
        if target_candidates > 0:
            summary["target_candidate_runs"] += 1
            if int(record.get("small_object_tile_search_primary_target_candidates", 0) or 0) == 0:
                summary["tile_only_discovery_count"] += 1
        elapsed_seconds = record.get("small_object_tile_search_elapsed_seconds")
        if elapsed_seconds is not None:
            summary["tile_elapsed_seconds"].append(elapsed_seconds)
    for summary in layouts.values():
        elapsed_seconds = summary.pop("tile_elapsed_seconds")
        summary["tile_elapsed_mean_seconds"] = mean_finite(elapsed_seconds)
        summary["tile_elapsed_max_seconds"] = max_finite(elapsed_seconds)

    exposure_stamps = []
    candidate_stamps = []
    for record in records:
        if record.get("_event") == "target_eval_detection_frame":
            if bool(record.get("frustum_exposed")):
                stamp = record.get("source_stamp")
                if stamp is not None:
                    exposure_stamps.append(float(stamp))
        elif record.get("_event") == "perception_decision":
            if int(record.get("model_target_candidates", 0) or 0) > 0:
                stamp = record.get("source_image_stamp")
                if stamp is not None:
                    candidate_stamps.append(float(stamp))
    first_exposure = min(exposure_stamps) if exposure_stamps else None
    first_candidate_after_exposure = (
        min(stamp for stamp in candidate_stamps if stamp >= first_exposure)
        if first_exposure is not None
        and any(stamp >= first_exposure for stamp in candidate_stamps)
        else None
    )
    return {
        "detector_decisions": len(decisions),
        "tile_attempts": len(tile_runs),
        "tile_target_candidate_runs": len(tile_target_runs),
        "tile_only_discovery_count": len(tile_only_discoveries),
        "inference_elapsed_mean_seconds": mean_finite(elapsed),
        "inference_elapsed_max_seconds": max_finite(elapsed),
        "tile_elapsed_mean_seconds": mean_finite(tile_elapsed),
        "tile_elapsed_max_seconds": max_finite(tile_elapsed),
        "tile_layouts": layouts,
        "source_image_age_mean_seconds": mean_finite(source_age),
        "source_image_age_max_seconds": max_finite(source_age),
        "first_exposure_source_stamp": first_exposure,
        "first_candidate_source_stamp": first_candidate_after_exposure,
        "first_exposure_to_first_candidate_seconds": (
            None
            if first_exposure is None or first_candidate_after_exposure is None
            else round(first_candidate_after_exposure - first_exposure, 4)
        ),
    }


def mean_finite(values):
    values = [float(value) for value in values if value is not None]
    values = [value for value in values if math.isfinite(value)]
    return None if not values else round(sum(values) / len(values), 4)


def max_finite(values):
    values = [float(value) for value in values if value is not None]
    values = [value for value in values if math.isfinite(value)]
    return None if not values else round(max(values), 4)


def target_mission_transaction_summary(records):
    """Summarize committed target route changes, not raw detector frames.

    ``mission_goal_transaction`` is emitted only after GoalManager has
    accepted a target route as executable. This deliberately distinguishes
    planned approach steps from a detector box or a goal topic that jitters on
    every camera callback.
    """
    transactions = [
        record for record in records
        if record.get("_event") == "mission_goal_transaction"
        and str(record.get("source", "")).startswith("target_")
    ]
    sources = {}
    intervals = []
    deltas = []
    previous = None
    for record in transactions:
        source = str(record.get("source", "unknown"))
        sources[source] = sources.get(source, 0) + 1
        goal = record.get("goal")
        timestamp = record.get("ros_time")
        if (
            previous is not None
            and isinstance(goal, (list, tuple))
            and len(goal) >= 2
            and isinstance(previous["goal"], (list, tuple))
            and len(previous["goal"]) >= 2
        ):
            try:
                deltas.append(math.hypot(
                    float(goal[0]) - float(previous["goal"][0]),
                    float(goal[1]) - float(previous["goal"][1]),
                ))
            except (TypeError, ValueError):
                pass
        if previous is not None and timestamp is not None:
            try:
                interval = float(timestamp) - float(previous["ros_time"])
                if interval >= 0.0:
                    intervals.append(interval)
            except (TypeError, ValueError):
                pass
        previous = {"goal": goal, "ros_time": timestamp}
    interval_min = finite_min(intervals)
    return {
        "count": len(transactions),
        "sources": sources,
        "interval_mean_seconds": mean_finite(intervals),
        "interval_min_seconds": (
            None if interval_min is None else round(interval_min, 4)
        ),
        "interval_max_seconds": max_finite(intervals),
        "delta_mean_m": mean_finite(deltas),
        "delta_max_m": max_finite(deltas),
    }


def failure_episode_summary(records, snapshots=None):
    """Rebuild correlated failure episodes from the compact metrics stream.

    ``failure_started`` and ``failure_snapshot_ready`` are intentionally
    separate events: a process can be killed during the post-window, in which
    case the open episode is still useful evidence and is reported as
    ``open_at_log_end`` instead of disappearing from the analysis.
    """
    started = {}
    completed = {}
    related_counts = {}
    for record in records:
        event = record.get("_event", "")
        failure_id = str(record.get("failure_id", "")).strip()
        if not failure_id:
            continue
        if event == "failure_started":
            started[failure_id] = dict(record)
        elif event == "failure_related_event":
            related_counts[failure_id] = related_counts.get(failure_id, 0) + 1
        elif event == "failure_snapshot_ready":
            completed[failure_id] = dict(record)

    snapshots = snapshots if isinstance(snapshots, dict) else {}
    episodes = []
    for failure_id, begin in started.items():
        end = completed.get(failure_id)
        classification = (
            (end or {}).get("classification")
            or ((begin.get("classification") or {}).get("label"))
            or "unknown"
        )
        episode = {
            "failure_id": failure_id,
            "trigger": begin.get("trigger"),
            "source": begin.get("source"),
            "artifact_path": (
                (snapshots.get(failure_id) or {}).get("artifact_path")
                or (end or {}).get("artifact_path")
            ),
            "classification": classification,
            "confidence": (
                (end or {}).get("confidence")
                or ((begin.get("classification") or {}).get("confidence"))
                or "low"
            ),
            "started_wall_elapsed_seconds": begin.get(
                "started_wall_elapsed_seconds"
            ),
            "ended_wall_elapsed_seconds": (
                None if end is None else end.get("ended_wall_elapsed_seconds")
            ),
            "duration_seconds": (
                None if end is None else end.get("duration_seconds")
            ),
            "sample_count": 0 if end is None else end.get("sample_count", 0),
            "related_event_count": related_counts.get(failure_id, 0),
            "status": "open_at_log_end" if end is None else "closed",
        }
        evidence = _failure_snapshot_evidence(snapshots.get(failure_id))
        if evidence is not None:
            episode["evidence"] = evidence
            diagnosis = _technical_diagnosis(snapshots.get(failure_id))
            if diagnosis is not None:
                episode["technical_diagnosis"] = diagnosis
        elif isinstance((end or {}).get("diagnosis"), dict):
            episode["technical_diagnosis"] = dict(end["diagnosis"])
        episodes.append(episode)

    # A snapshot can survive even if the compact start event was lost during
    # startup ordering. Keep it visible rather than silently dropping it.
    for failure_id, end in completed.items():
        if failure_id in started:
            continue
        episode = {
            "failure_id": failure_id,
            "trigger": end.get("trigger"),
            "source": end.get("source"),
            "artifact_path": (
                (snapshots.get(failure_id) or {}).get("artifact_path")
                or end.get("artifact_path")
            ),
            "classification": end.get("classification", "unknown"),
            "confidence": end.get("confidence", "low"),
            "started_wall_elapsed_seconds": None,
            "ended_wall_elapsed_seconds": end.get("ended_wall_elapsed_seconds"),
            "duration_seconds": end.get("duration_seconds"),
            "sample_count": end.get("sample_count", 0),
            "related_event_count": related_counts.get(failure_id, 0),
            "status": "closed_without_start_event",
        }
        evidence = _failure_snapshot_evidence(snapshots.get(failure_id))
        if evidence is not None:
            episode["evidence"] = evidence
            diagnosis = _technical_diagnosis(snapshots.get(failure_id))
            if diagnosis is not None:
                episode["technical_diagnosis"] = diagnosis
        elif isinstance(end.get("diagnosis"), dict):
            episode["technical_diagnosis"] = dict(end["diagnosis"])
        episodes.append(episode)

    episodes.sort(
        key=lambda item: (
            item.get("started_wall_elapsed_seconds") is None,
            item.get("started_wall_elapsed_seconds") or float("inf"),
        )
    )
    by_classification = {}
    for episode in episodes:
        label = str(episode.get("classification", "unknown"))
        by_classification[label] = by_classification.get(label, 0) + 1
    return {
        "episode_count": len(episodes),
        "closed_count": sum(item["status"] == "closed" for item in episodes),
        "open_count": sum(item["status"] == "open_at_log_end" for item in episodes),
        "by_classification": dict(sorted(by_classification.items())),
        "episodes": episodes,
    }


def summarize(path):
    raw_records = load_records(path)
    completion_record = next(
        (
            record for record in reversed(raw_records)
            if record.get("_event") == "task_completed"
        ),
        None,
    )
    completion_envelope = (
        completion_record.get("completion_snapshot")
        if isinstance(completion_record, dict)
        and isinstance(completion_record.get("completion_snapshot"), dict)
        else None
    )
    completion_snapshot = (
        completion_envelope.get("metrics")
        if isinstance(completion_envelope, dict)
        and isinstance(completion_envelope.get("metrics"), dict)
        else None
    )
    completion_cutoff_ros = None
    if completion_envelope is not None:
        completion_cutoff_ros = completion_envelope.get("boundary_ros_time")
    if completion_cutoff_ros is None and completion_record is not None:
        completion_cutoff_ros = completion_record.get("ros_time")
    if completion_cutoff_ros is not None:
        try:
            completion_cutoff_ros = float(completion_cutoff_ros)
        except (TypeError, ValueError):
            completion_cutoff_ros = None
    # Keep raw logs complete, while performance aggregation ends exactly when
    # the task completes. Older logs without a snapshot still use the last
    # sample at or before their task_completed event.
    if completion_cutoff_ros is None:
        records = raw_records
        metric_snapshot_source = "latest_sample"
    else:
        records = [
            record for record in raw_records
            if record.get("ros_time") is None
            or float(record.get("ros_time")) <= completion_cutoff_ros + 1e-6
        ]
        metric_snapshot_source = (
            "task_completion_snapshot"
            if completion_snapshot is not None
            else "last_sample_before_task_completed"
        )
    # Target lifecycle is reconstructed from every raw record. Its identity
    # event can legitimately arrive one callback after task_done, while the
    # control-rate benchmark above must remain strictly task-completion bound.
    lifecycle_sessions, lifecycle_unattributed = target_lifecycle_sessions(
        raw_records
    )
    tile_summary = perception_tile_summary(records)
    completed_lifecycle_session = completed_target_session(lifecycle_sessions)
    active_lifecycle_session = active_target_session(lifecycle_sessions)
    events = {}
    samples = []
    goal_changes = []
    bridge_events = []
    route_invalidations = []
    for record in records:
        event = record.get("_event", "")
        events[event] = events.get(event, 0) + 1
        if event == "sample":
            samples.append(record)
        elif event == "goal_change":
            goal_changes.append(record)
        elif event == "teb_bridge_event":
            bridge_events.append(record)
            if record.get("bridge_event") == "frontier_route_invalidated":
                route_invalidations.append(record)

    raw_samples = [
        record for record in raw_records if record.get("_event") == "sample"
    ]

    last = completion_snapshot or (samples[-1] if samples else {})
    if completion_record is not None and completion_snapshot is None:
        # Compatibility path for logs written before the explicit snapshot.
        # The nearest sample predates the task_done callback by one timer tick,
        # so carry the terminal fact without claiming its counters include
        # events that arrived afterwards.
        last = dict(last)
        last["task_done"] = True
        last["task_done_seconds"] = completion_record.get(
            "latency_seconds", last.get("task_done_seconds")
        )
    first = samples[0] if samples else {}
    raw_first = raw_samples[0] if raw_samples else {}
    raw_last = raw_samples[-1] if raw_samples else {}
    run_start = next(
        (record for record in records if record.get("_event") == "run_start"),
        {},
    )
    clearances = [sample.get("min_scan_clearance") for sample in samples]
    minimum_clearance = finite_min(clearances)
    duration = None
    if first.get("ros_time") is not None and last.get("ros_time") is not None:
        duration = max(0.0, float(last["ros_time"]) - float(first["ros_time"]))
    raw_duration = None
    if (
        raw_first.get("ros_time") is not None
        and raw_last.get("ros_time") is not None
    ):
        raw_duration = max(
            0.0, float(raw_last["ros_time"]) - float(raw_first["ros_time"])
        )
    source_counts = {}
    for record in goal_changes:
        source = str(record.get("goal_source", "unknown"))
        source_counts[source] = source_counts.get(source, 0) + 1
    target_mission_transactions = target_mission_transaction_summary(records)
    failure_evidence = failure_episode_summary(
        raw_records, load_failure_snapshots(path)
    )
    startup_failure = load_startup_failure(path)

    replacement_counts = {}
    for record in bridge_events:
        if not record.get("replacement", False):
            continue
        kind = str(record.get("replacement_kind", "unknown"))
        replacement_counts[kind] = replacement_counts.get(kind, 0) + 1

    dispatch_reasons = {}
    frontier_prefetch_defer_reasons = {}
    persistent_lookahead_admission_reasons = {}
    for record in bridge_events:
        if record.get("bridge_event") == "dispatch":
            reason = str(record.get("reason", "unknown"))
            dispatch_reasons[reason] = dispatch_reasons.get(reason, 0) + 1
        elif record.get("bridge_event") == "frontier_continuous_prefetch_deferred":
            reason = str(record.get("reason", "unknown"))
            frontier_prefetch_defer_reasons[reason] = (
                frontier_prefetch_defer_reasons.get(reason, 0) + 1
            )
        elif record.get("bridge_event") == "persistent_frontier_prefetch_admission_deferred":
            reason = str(record.get("reason", "unknown"))
            persistent_lookahead_admission_reasons[reason] = (
                persistent_lookahead_admission_reasons.get(reason, 0) + 1
            )
    retry_dispatches = int(dispatch_reasons.get("retry_move_base_goal", 0))
    action_goal_ids = sorted(
        {
            str(record.get("goal_id", ""))
            for record in records
            if record.get("_event") == "move_base_status"
            and str(record.get("goal_id", ""))
        }
    )
    route_invalidation_reasons = {}
    for record in route_invalidations:
        reason = str(record.get("reason", "unknown"))
        route_invalidation_reasons[reason] = (
            route_invalidation_reasons.get(reason, 0) + 1
        )
    latest_route_invalidation = (
        route_invalidations[-1].get("source_status", {})
        if route_invalidations else None
    )
    handoff_latencies = terminal_dispatch_latencies(records)
    path_length = last.get("path_length") or 0.0
    forward_distance = last.get("forward_distance_m") or 0.0
    preemptions = last.get("move_base_unexpected_preemptions") or 0
    bridge_dispatches = last.get("move_base_dispatches") or 0
    brakes = last.get("linear_brake_events") or 0
    brake_clear = last.get("brake_events_clear") or 0
    brake_near = last.get("brake_events_near") or 0
    brake_reasons = last.get("brake_reason_counts") or {}
    endpoint_terminal_brakes = last.get(
        "endpoint_terminal_brake_events",
        brake_reasons.get("action_terminal", 0)
        + brake_reasons.get("endpoint_action_terminal", 0)
        + brake_reasons.get("persistent_endpoint_terminal", 0),
    )
    unexplained_clear_brakes = last.get(
        "unexplained_clear_path_brake_events",
        brake_reasons.get("unexplained_clear_path", 0),
    )

    result = {
        "log": str(path),
        "run_id": path.parent.name,
        "experiment": run_start.get("experiment"),
        "records": len(records),
        "raw_records": len(raw_records),
        "samples": len(samples),
        "post_completion_sample_count": max(0, len(raw_samples) - len(samples)),
        "metric_snapshot_source": metric_snapshot_source,
        "duration_seconds": None if duration is None else round(duration, 3),
        "raw_duration_seconds": (
            None if raw_duration is None else round(raw_duration, 3)
        ),
        "execution_architecture": last.get("execution_architecture", "unknown"),
        "persistent_execution": last.get("persistent_execution"),
        "goal_first_seen_seconds": last.get("target_first_seen_seconds"),
        "target_follow_confirmed_seconds": last.get(
            "target_follow_confirmed_seconds"
        ),
        "target_close_confirmation_started_seconds": last.get(
            "target_close_confirmation_started_seconds"
        ),
        "target_close_confirmed_seconds": last.get(
            "target_close_confirmed_seconds"
        ),
        "target_lock_seconds": last.get("target_lock_seconds"),
        "task_done_seconds": last.get("task_done_seconds"),
        # Simulator-only frustum/IoU evidence is kept separate from the
        # detector-driven target lifecycle above. It is never a control input.
        "target_geometric_evaluation": last.get("target_geometric_evaluation"),
        "perception_tile_search": tile_summary,
        "target_lifecycle_sessions": lifecycle_sessions,
        "target_lifecycle_unattributed_events": lifecycle_unattributed,
        "target_lifecycle_completed_session": completed_lifecycle_session,
        "target_lifecycle_active_session": active_lifecycle_session,
        "goal_changes": len(goal_changes),
        "goal_delta_max_m": last.get("goal_delta_max"),
        "goal_delta_mean_m": last.get("goal_delta_mean"),
        "goal_change_sources": source_counts,
        "target_mission_transactions": target_mission_transactions,
        "failure_evidence": failure_evidence,
        "startup_failure": startup_failure,
        "failure_evidence_log": (
            (last.get("failure_evidence") or {}).get("failure_log_path")
            or run_start.get("failure_evidence_log")
        ),
        "goal_transition_counts": last.get("goal_transition_counts", {}),
        "goal_transition_brake_events": last.get(
            "goal_transition_brake_events", {}
        ),
        "persistent_plan": {
            "received": last.get("persistent_plan_received", 0),
            "equivalent_retained": last.get(
                "persistent_plan_equivalent_retained", 0
            ),
            "installed": last.get("persistent_plan_installed", 0),
            "last_event": last.get("persistent_plan_last_event", "unknown"),
            "route_version": last.get("persistent_plan_route_version", 0),
            "geometry_hash": last.get("persistent_plan_geometry_hash"),
        },
        # The bridge can publish its first dispatch before the metrics window
        # starts.  Goal IDs observed on /move_base/status are therefore the
        # authoritative lower bound for actions that actually existed, while
        # the bridge counter diagnoses dispatch lifecycle behavior.
        "move_base_bridge_dispatches": last.get("move_base_dispatches"),
        "move_base_observed_actions": len(action_goal_ids),
        "move_base_goal_ids": action_goal_ids,
        # Kept for JSON consumers of older analysis output.
        "move_base_dispatches": last.get("move_base_dispatches"),
        "move_base_unique_goal_ids": len(action_goal_ids),
        "move_base_preemptions": last.get("move_base_preemptions"),
        "move_base_frontier_observation_preemptions": last.get(
            "move_base_frontier_observation_preemptions", 0
        ),
        "move_base_frontier_terminal_settle_preemptions": last.get(
            "move_base_frontier_terminal_settle_preemptions", 0
        ),
        "move_base_frontier_continuous_prefetch_preemptions": last.get(
            "move_base_frontier_continuous_prefetch_preemptions", 0
        ),
        "move_base_frontier_segment_preemptions": last.get(
            "move_base_frontier_segment_preemptions", 0
        ),
        "move_base_target_segment_preemptions": last.get(
            "move_base_target_segment_preemptions", 0
        ),
        "move_base_priority_preemptions": last.get(
            "move_base_priority_preemptions", 0
        ),
        "move_base_task_done_preemptions": last.get(
            "move_base_task_done_preemptions", 0
        ),
        "move_base_route_recovery_preemptions": last.get(
            "move_base_route_recovery_preemptions", 0
        ),
        "move_base_route_recovery_preemption_reasons": last.get(
            "move_base_route_recovery_preemption_reasons", {}
        ),
        "move_base_unexpected_preemptions": last.get(
            "move_base_unexpected_preemptions", 0
        ),
        "move_base_successes": last.get("move_base_successes"),
        "move_base_aborts": last.get("move_base_aborts"),
        "bridge_dispatches": last.get("teb_bridge_dispatches"),
        "bridge_terminals": last.get("teb_bridge_terminal_events"),
        "bridge_deferred_updates": last.get("teb_bridge_deferred_goal_updates"),
        "bridge_goal_replacements": last.get("teb_bridge_goal_replacements"),
        "bridge_priority_goal_replacements": last.get(
            "teb_bridge_priority_goal_replacements"
        ),
        "bridge_target_goal_replacements": last.get(
            "teb_bridge_target_goal_replacements"
        ),
        "bridge_frontier_goal_replacements": last.get(
            "teb_bridge_frontier_segment_handoffs"
        ),
        "bridge_frontier_observation_completions": last.get(
            "teb_bridge_frontier_observation_completions", 0
        ),
        "bridge_frontier_terminal_settle_completions": last.get(
            "teb_bridge_frontier_terminal_settle_completions", 0
        ),
        "bridge_frontier_continuous_prefetch_handoffs": last.get(
            "teb_bridge_frontier_continuous_prefetch_handoffs", 0
        ),
        "bridge_frontier_continuous_prefetch_fallbacks": last.get(
            "teb_bridge_frontier_continuous_prefetch_fallbacks", 0
        ),
        "bridge_frontier_prefetch_requires_turn": last.get(
            "teb_bridge_frontier_prefetch_requires_turn", 0
        ),
        "bridge_frontier_continuous_prefetch_defer_reasons": (
            frontier_prefetch_defer_reasons
        ),
        "bridge_persistent_lookahead_handoffs": last.get(
            "teb_bridge_persistent_lookahead_handoffs", 0
        ),
        "bridge_persistent_lookahead_admission_deferred": last.get(
            "teb_bridge_persistent_lookahead_admission_deferred", 0
        ),
        "bridge_persistent_lookahead_admission_reasons": (
            persistent_lookahead_admission_reasons
        ),
        "bridge_frontier_sharp_replacements": last.get(
            "teb_bridge_frontier_sharp_replacements"
        ),
        "bridge_replacement_kinds": replacement_counts,
        "target_route_accepts": last.get("target_route_accepts"),
        "target_route_rejections": last.get("target_route_rejections"),
        "target_route_deferrals": last.get("target_route_deferrals"),
        "target_route_holds": last.get("target_route_holds"),
        "target_route_semantic_replans": last.get("target_route_semantic_replans", 0),
        "target_route_failures": last.get("target_route_failures"),
        "target_route_releases": last.get("target_route_releases"),
        "target_approach_terminals": last.get("target_approach_terminals"),
        "target_segments_committed": last.get("target_segments_committed", 0),
        "target_continuous_handoffs_prepared": last.get(
            "target_continuous_handoffs_prepared", 0
        ),
        "linear_brake_events": last.get("linear_brake_events"),
        "linear_brake_rate_per_minute": last.get("linear_brake_rate_per_minute"),
        "speed_modulation_events": last.get("speed_modulation_events", 0),
        "brake_reason_counts": brake_reasons,
        "stop_reason_counts": last.get("stop_reason_counts") or {},
        "retry_dispatches": retry_dispatches,
        "dispatch_reasons": dispatch_reasons,
        "route_invalidations": len(route_invalidations),
        "route_invalidation_reasons": route_invalidation_reasons,
        "latest_route_invalidation": latest_route_invalidation,
        "preemption_ratio": round(
            preemptions / max(1, bridge_dispatches, len(action_goal_ids)), 3
        ),
        "forward_distance_m": last.get("forward_distance_m"),
        "forward_angular_energy_per_m": last.get("forward_angular_energy_per_m"),
        "straight_path_distance_m": last.get("straight_path_distance_m"),
        "straight_path_angular_energy_per_m": last.get(
            "straight_path_angular_energy_per_m"
        ),
        "straight_path_steering_sign_flips": last.get(
            "straight_path_steering_sign_flips"
        ),
        "straight_path_samples": last.get("straight_path_samples"),
        "brake_events_clear": brake_clear,
        "brake_events_near": brake_near,
        "brake_events_clear_fraction": (
            None if (brake_clear + brake_near) == 0
            else round(brake_clear / (brake_clear + brake_near), 3)
        ),
        "endpoint_terminal_brake_events": endpoint_terminal_brakes,
        "unexplained_clear_path_brake_events": unexplained_clear_brakes,
        "unexplained_clear_path_brake_fraction": (
            None if brakes == 0 else round(unexplained_clear_brakes / brakes, 3)
        ),
        "brakes_per_m": round(brakes / max(0.01, path_length), 4),
        "brakes_per_m_forward": round(brakes / max(0.01, forward_distance), 4),
        "teb_supervisor_linear_brake_events": last.get(
            "teb_supervisor_linear_brake_events",
            last.get("teb_linear_brake_events"),
        ),
        "teb_supervisor_speed_modulation_events": last.get(
            "teb_supervisor_speed_modulation_events",
            last.get("teb_speed_modulation_events", 0),
        ),
        "teb_planner_linear_brake_events": last.get(
            "teb_planner_linear_brake_events"
        ),
        "teb_planner_speed_modulation_events": last.get(
            "teb_planner_speed_modulation_events", 0
        ),
        "teb_trajectory_continuity_events": last.get(
            "teb_trajectory_continuity_events", 0
        ),
        "stop_events": last.get("stop_events"),
        "stop_rate_per_minute": last.get("stop_rate_per_minute"),
        "average_stop_duration_seconds": last.get("average_stop_duration_seconds"),
        "max_stop_duration_seconds": last.get("max_stop_duration_seconds"),
        "goal_change_rate_per_minute": last.get("goal_change_rate_per_minute"),
        "terminal_to_dispatch_samples": len(handoff_latencies),
        "terminal_to_dispatch_mean_seconds": (
            None if not handoff_latencies
            else round(sum(handoff_latencies) / len(handoff_latencies), 4)
        ),
        "terminal_to_dispatch_max_seconds": (
            None if not handoff_latencies else round(max(handoff_latencies), 4)
        ),
        "terminal_to_dispatch_latencies_seconds": [
            round(value, 4) for value in handoff_latencies
        ],
        "angular_sign_flips": last.get("angular_sign_flips"),
        "strong_angular_sign_flips": last.get("strong_angular_sign_flips"),
        "forward_steering_sign_flips": last.get("forward_steering_sign_flips"),
        "teb_angular_sign_flips": last.get("teb_angular_sign_flips"),
        "teb_strong_angular_sign_flips": last.get("teb_strong_angular_sign_flips"),
        "teb_forward_steering_sign_flips": last.get(
            "teb_forward_steering_sign_flips"
        ),
        "min_scan_clearance_m": None if minimum_clearance is None else round(minimum_clearance, 4),
        "path_length_m": last.get("path_length"),
        "controller_mode": last.get("controller_mode"),
        "goal_source": last.get("goal_source"),
        "task_done": last.get("task_done"),
        "bridge_event_counts": {
            str(record.get("bridge_event", "unknown")): sum(
                1 for item in bridge_events
                if item.get("bridge_event", "unknown") == record.get("bridge_event", "unknown")
            )
            for record in bridge_events
        },
        "log_event_counts": events,
    }
    # Do not combine the global first-seen/follow/done counters above: a run
    # can contain multiple visual tracks. End-to-end values are emitted only
    # from one completed identity-bound lifecycle session.
    selected = completed_lifecycle_session or {}
    first_seen = selected.get("first_seen_seconds")
    target_lock = selected.get("follow_confirmed_seconds")
    close_confirmed = selected.get("close_confirmed_seconds")
    task_done = selected.get("task_completed_seconds")
    result["target_first_seen_to_lock_seconds"] = (
        None
        if first_seen is None or target_lock is None
        else round(max(0.0, float(target_lock) - float(first_seen)), 3)
    )
    result["target_lock_to_done_seconds"] = (
        None
        if target_lock is None or task_done is None
        else round(max(0.0, float(task_done) - float(target_lock)), 3)
    )
    result["target_first_seen_to_done_seconds"] = (
        None
        if first_seen is None or task_done is None
        else round(max(0.0, float(task_done) - float(first_seen)), 3)
    )
    result["target_follow_to_close_confirmed_seconds"] = (
        None
        if target_lock is None or close_confirmed is None
        else round(max(0.0, float(close_confirmed) - float(target_lock)), 3)
    )
    result["target_close_confirmed_to_done_seconds"] = (
        None
        if close_confirmed is None or task_done is None
        else round(max(0.0, float(task_done) - float(close_confirmed)), 3)
    )
    return result


def print_human(result):
    print("Navigation run: %s (architecture=%s persistent=%s)" % (
        result["run_id"], result["execution_architecture"],
        result["persistent_execution"],
    ))
    print("  metric cutoff: %s" % result["metric_snapshot_source"])
    print("  log: %s" % result["log"])
    experiment = result.get("experiment") or {}
    if experiment:
        pose = experiment.get("initial_pose") or {}
        print("  experiment: world=%s spawn=(%s,%s,%s,yaw=%s) slam=%s frontier=%s source=%s git=%s%s" % (
            experiment.get("world"),
            pose.get("x"), pose.get("y"), pose.get("z"), pose.get("yaw"),
            experiment.get("online_slam_enabled"),
            experiment.get("global_frontier_enabled"),
            experiment.get("global_goal_source"),
            experiment.get("git_revision"),
            " (dirty)" if experiment.get("git_dirty") else "",
        ))
    print("  duration: %s s, samples: %s" % (result["duration_seconds"], result["samples"]))
    if result["post_completion_sample_count"]:
        print("  raw log after completion: %s samples, %s s total" % (
            result["post_completion_sample_count"],
            result["raw_duration_seconds"],
        ))
    print("  target observations (legacy run-wide): first_seen=%s s follow_confirmed=%s s close_confirmed=%s s done=%s s" % (
        result["goal_first_seen_seconds"],
        result["target_follow_confirmed_seconds"],
        result["target_close_confirmed_seconds"],
        result["task_done_seconds"],
    ))
    completed = result["target_lifecycle_completed_session"]
    active = result["target_lifecycle_active_session"]
    if completed is None:
        if active is None:
            print("  target lifecycle: sessions=%s, no identity-bound completion; end-to-end latency omitted" % (
                len(result["target_lifecycle_sessions"]),
            ))
        else:
            print("  target lifecycle active: task=%s track=%s first_seen=%s s follow=%s s approach_terminal=%s s; seen->follow=%s s follow->approach=%s s" % (
                active["task_id"],
                active["target_track_id"],
                active["first_seen_seconds"],
                active["follow_confirmed_seconds"],
                active["approach_terminal_seconds"],
                active["first_seen_to_follow_seconds"],
                active["follow_to_approach_terminal_seconds"],
            ))
    else:
        print("  target lifecycle: task=%s track=%s first_seen=%s s follow=%s s approach_terminal=%s s close=%s s done=%s s" % (
            completed["task_id"],
            completed["target_track_id"],
            completed["first_seen_seconds"],
            completed["follow_confirmed_seconds"],
            completed["approach_terminal_seconds"],
            completed["close_confirmed_seconds"],
            completed["task_completed_seconds"],
        ))
    print("  target latency: first_seen->follow=%s s follow->done=%s s first_seen->done=%s s follow->close=%s s close->done=%s s" % (
        result["target_first_seen_to_lock_seconds"],
        result["target_lock_to_done_seconds"],
        result["target_first_seen_to_done_seconds"],
        result["target_follow_to_close_confirmed_seconds"],
        result["target_close_confirmed_to_done_seconds"],
    ))
    geometric = result["target_geometric_evaluation"]
    if geometric is None:
        print("  target geometric evaluation: unavailable in this legacy log")
    elif not geometric.get("enabled", False):
        print("  target geometric evaluation: disabled")
    else:
        print("  target geometric evaluation (Gazebo observer only): episodes=%s matched_episodes=%s frame_recall=%s episode_recall=%s exposed_frames=%s matched_frames=%s unmatched_target_candidate_frames=%s unavailable_frames=%s first_match_after_exposure=%s s" % (
            geometric.get("exposure_episodes"),
            geometric.get("matched_exposure_episodes"),
            geometric.get("frame_geometric_recall"),
            geometric.get("episode_geometric_recall"),
            geometric.get("exposed_detector_frames"),
            geometric.get("matched_detector_frames"),
            geometric.get(
                "unmatched_target_candidate_frames",
                geometric.get("false_positive_frames"),
            ),
            geometric.get("unavailable_detector_frames"),
            geometric.get("first_match_after_exposure_seconds"),
        ))
        if not geometric.get("depth_validation_enabled", False):
            print("  target depth validation: disabled")
        else:
            print("  target depth validation (Gazebo observer only): frame_ready=%s decoded=%s/%s decode_failures=%s visible_episodes=%s matched_visible_episodes=%s visible_frames=%s matched_visible_frames=%s frame_recall=%s occluded_frames=%s inconsistent_frames=%s unavailable_frames=%s first_match_after_depth_visible=%s s" % (
                geometric.get("depth_frame_ready"),
                geometric.get("depth_frames_decoded"),
                geometric.get("depth_frames_received"),
                geometric.get("depth_decode_failures"),
                geometric.get("depth_visible_exposure_episodes"),
                geometric.get("depth_matched_visible_exposure_episodes"),
                geometric.get("depth_visible_detector_frames"),
                geometric.get("depth_visible_matched_detector_frames"),
                geometric.get("frame_depth_visible_recall"),
                geometric.get("depth_occluded_detector_frames"),
                geometric.get("depth_inconsistent_detector_frames"),
                geometric.get("depth_unavailable_detector_frames"),
                geometric.get("first_match_after_depth_visible_seconds"),
            ))
    tile = result["perception_tile_search"]
    print("  perception tiles: attempts=%s target_candidate_runs=%s tile_only_discoveries=%s full+tile_latency_mean=%s s max=%s s tile_latency_mean=%s s max=%s s source_age_mean=%s s max=%s s exposure->candidate=%s s layouts=%s" % (
        tile["tile_attempts"],
        tile["tile_target_candidate_runs"],
        tile["tile_only_discovery_count"],
        tile["inference_elapsed_mean_seconds"],
        tile["inference_elapsed_max_seconds"],
        tile["tile_elapsed_mean_seconds"],
        tile["tile_elapsed_max_seconds"],
        tile["source_image_age_mean_seconds"],
        tile["source_image_age_max_seconds"],
        tile["first_exposure_to_first_candidate_seconds"],
        json.dumps(tile["tile_layouts"], sort_keys=True),
    ))
    print("  goals: changes=%s max_delta=%s m mean_delta=%s m" % (
        result["goal_changes"], result["goal_delta_max_m"], result["goal_delta_mean_m"],
    ))
    target_transactions = result["target_mission_transactions"]
    print("  target mission updates: count=%s interval_mean=%s s min=%s s max=%s s delta_mean=%s m delta_max=%s m sources=%s" % (
        target_transactions["count"],
        target_transactions["interval_mean_seconds"],
        target_transactions["interval_min_seconds"],
        target_transactions["interval_max_seconds"],
        target_transactions["delta_mean_m"],
        target_transactions["delta_max_m"],
        json.dumps(target_transactions["sources"], sort_keys=True),
    ))
    print("  endpoint transitions: %s; brakes within 0.5s: %s" % (
        json.dumps(result["goal_transition_counts"], sort_keys=True),
        json.dumps(result["goal_transition_brake_events"], sort_keys=True),
    ))
    print("  persistent plans: %s" % json.dumps(
        result["persistent_plan"], sort_keys=True
    ))
    print("  actions: observed=%s bridge_dispatches=%s success=%s raw_preempt=%s route_recovery_preempt=%s observation_preempt=%s settle_preempt=%s continuous_preempt=%s segment_preempt=%s priority_preempt=%s done_preempt=%s unexpected_preempt=%s abort=%s bridge_deferred=%s" % (
        result["move_base_observed_actions"], result["move_base_bridge_dispatches"],
        result["move_base_successes"],
        result["move_base_preemptions"],
        result["move_base_route_recovery_preemptions"],
        result["move_base_frontier_observation_preemptions"],
        result["move_base_frontier_terminal_settle_preemptions"],
        result["move_base_frontier_continuous_prefetch_preemptions"],
        result["move_base_frontier_segment_preemptions"],
        result["move_base_priority_preemptions"],
        result["move_base_task_done_preemptions"],
        result["move_base_unexpected_preemptions"], result["move_base_aborts"],
        result["bridge_deferred_updates"],
    ))
    print("  continuous frontier: handoffs=%s fallbacks=%s requires_turn=%s deferred=%s" % (
        result["bridge_frontier_continuous_prefetch_handoffs"],
        result["bridge_frontier_continuous_prefetch_fallbacks"],
        result["bridge_frontier_prefetch_requires_turn"],
        json.dumps(
            result["bridge_frontier_continuous_prefetch_defer_reasons"],
            sort_keys=True,
        ),
    ))
    print("  persistent lookahead: handoffs=%s admission_deferred=%s reasons=%s" % (
        result["bridge_persistent_lookahead_handoffs"],
        result["bridge_persistent_lookahead_admission_deferred"],
        json.dumps(
            result["bridge_persistent_lookahead_admission_reasons"],
            sort_keys=True,
        ),
    ))
    print("  replacements: total=%s priority=%s frontier_segment=%s target_segment=%s kinds=%s" % (
        result["bridge_goal_replacements"],
        result["bridge_priority_goal_replacements"],
        result["bridge_frontier_goal_replacements"],
        result["bridge_target_goal_replacements"],
        json.dumps(result["bridge_replacement_kinds"], sort_keys=True),
    ))
    print("  target route: committed=%s continuous_prefetch=%s accepted=%s rejected=%s deferred=%s held=%s semantic_replans=%s terminals=%s failures=%s releases=%s" % (
        result["target_segments_committed"],
        result["target_continuous_handoffs_prepared"],
        result["target_route_accepts"],
        result["target_route_rejections"],
        result["target_route_deferrals"],
        result["target_route_holds"],
        result["target_route_semantic_replans"],
        result["target_approach_terminals"],
        result["target_route_failures"],
        result["target_route_releases"],
    ))
    print("  control: mux_zero_drops=%s (%.2f/min) mux_speed_modulation=%s planner_zero_drops=%s supervisor_zero_drops=%s continuity=%s stops=%s (%.2f/min) "
          "avg_stop=%.3fs max_stop=%.3fs angular_flips=%s strong_flips=%s forward_flips=%s" % (
        result["linear_brake_events"],
        result["linear_brake_rate_per_minute"] or 0.0,
        result["speed_modulation_events"],
        result["teb_planner_linear_brake_events"],
        result["teb_supervisor_linear_brake_events"],
        result["teb_trajectory_continuity_events"],
        result["stop_events"], result["stop_rate_per_minute"] or 0.0,
        result["average_stop_duration_seconds"] or 0.0,
        result["max_stop_duration_seconds"] or 0.0,
        result["angular_sign_flips"],
        result["strong_angular_sign_flips"],
        result["forward_steering_sign_flips"],
    ))
    print("  smoothness: forward=%.1fm steer_energy=%.3f rad/m forward_flips/m=%.3f "
          "straight=%.1fm straight_steer=%.3f rad/m straight_flips=%s "
          "brakes/m=%.3f raw_clear=%s unexplained_clear=%s endpoint_terminal=%s" % (
        result["forward_distance_m"] or 0.0,
        result["forward_angular_energy_per_m"] or 0.0,
        (result["forward_steering_sign_flips"] or 0) / max(0.01, result["forward_distance_m"] or 0.0),
        result["straight_path_distance_m"] or 0.0,
        result["straight_path_angular_energy_per_m"] or 0.0,
        result["straight_path_steering_sign_flips"],
        result["brakes_per_m"] or 0.0,
        result["brake_events_clear"],
        result["unexplained_clear_path_brake_events"],
        result["endpoint_terminal_brake_events"],
    ))
    print("  discontinuity causes: brakes=%s stops=%s" % (
        json.dumps(result["brake_reason_counts"], sort_keys=True),
        json.dumps(result["stop_reason_counts"], sort_keys=True),
    ))
    print("  goal churn: retries=%s preemption_ratio=%.2f reasons=%s" % (
        result["retry_dispatches"],
        result["preemption_ratio"] or 0.0,
        json.dumps(result["dispatch_reasons"], sort_keys=True),
    ))
    print("  terminal handoff: samples=%s mean=%s s max=%s s latencies=%s" % (
        result["terminal_to_dispatch_samples"],
        result["terminal_to_dispatch_mean_seconds"],
        result["terminal_to_dispatch_max_seconds"],
        result["terminal_to_dispatch_latencies_seconds"],
    ))
    print("  route recovery: invalidations=%s reasons=%s last=%s" % (
        result["route_invalidations"],
        json.dumps(result["route_invalidation_reasons"], sort_keys=True),
        json.dumps(result["latest_route_invalidation"], sort_keys=True),
    ))
    print("  safety: min_clearance=%s m path=%s m mode=%s source=%s done=%s" % (
        result["min_scan_clearance_m"], result["path_length_m"],
        result["controller_mode"], result["goal_source"], result["task_done"],
    ))
    print("  bridge events: %s" % json.dumps(result["bridge_event_counts"], sort_keys=True))
    failure_evidence = result.get("failure_evidence") or {}
    print("  failure evidence: episodes=%s closed=%s open=%s classes=%s" % (
        failure_evidence.get("episode_count", 0),
        failure_evidence.get("closed_count", 0),
        failure_evidence.get("open_count", 0),
        json.dumps(failure_evidence.get("by_classification", {}), sort_keys=True),
    ))
    for episode in failure_evidence.get("episodes", []):
        print(
            "    %s t=%s trigger=%s class=%s confidence=%s status=%s related=%s"
            % (
                episode.get("failure_id"),
                episode.get("started_wall_elapsed_seconds"),
                episode.get("trigger"),
                episode.get("classification"),
                episode.get("confidence"),
                episode.get("status"),
                episode.get("related_event_count", 0),
            )
        )
        diagnosis = episode.get("technical_diagnosis") or {}
        if diagnosis:
            print(
                "     technical_diagnosis: cause=%s layer=%s route=%s "
                "action=%s confidence=%s"
                % (
                    diagnosis.get("primary_cause"),
                    diagnosis.get("layer"),
                    diagnosis.get("route_id"),
                    diagnosis.get("graph_action"),
                    diagnosis.get("confidence"),
                )
            )
        evidence = episode.get("evidence") or {}
        trigger = evidence.get("trigger_sample") or {}
        if trigger:
            print(
                "     现场: pose=%s goal=%s dist=%s teb=%s navfn_remaining=%s "
                "scan_forward=%s cmd=%s route=%s artifact=%s"
                % (
                    trigger.get("pose"),
                    trigger.get("goal"),
                    trigger.get("distance_to_goal"),
                    trigger.get("teb_status"),
                    trigger.get("navfn_path_remaining_m"),
                    (trigger.get("scan") or {}).get("forward_min"),
                    trigger.get("cmd_vel"),
                    evidence.get("route_context_at_trigger"),
                    evidence.get("artifact_path"),
                )
            )
    startup_failure = result.get("startup_failure")
    if startup_failure:
        readiness = startup_failure.get("readiness") or {}
        print(
            "  startup failure: id=%s reason=%s readiness=%s elapsed=%s s "
            "artifact=%s"
            % (
                startup_failure.get("failure_id"),
                startup_failure.get("reason"),
                readiness.get("readiness_last_state") or readiness.get("reason"),
                readiness.get("elapsed_wall_seconds"),
                startup_failure.get("artifact_path"),
            )
        )


COMPARE_FIELDS = [
    ("duration", "duration_seconds"),
    ("goal changes", "goal_changes"),
    ("goal churn/min", "goal_change_rate_per_minute"),
    ("observed actions", "move_base_observed_actions"),
    ("bridge dispatches", "move_base_bridge_dispatches"),
    ("preemptions", "move_base_preemptions"),
    ("route recovery preemptions", "move_base_route_recovery_preemptions"),
    ("continuous handoffs", "bridge_frontier_continuous_prefetch_handoffs"),
    ("continuous fallbacks", "bridge_frontier_continuous_prefetch_fallbacks"),
    ("preemption ratio", "preemption_ratio"),
    ("retries", "retry_dispatches"),
    ("route invalidations", "route_invalidations"),
    ("successes", "move_base_successes"),
    ("hard brakes", "linear_brake_events"),
    ("speed modulation", "speed_modulation_events"),
    ("hard brakes/m", "brakes_per_m"),
    ("unexplained clear brakes", "unexplained_clear_path_brake_events"),
    ("stops", "stop_events"),
    ("avg stop (s)", "average_stop_duration_seconds"),
    ("max stop (s)", "max_stop_duration_seconds"),
    ("angular flips", "angular_sign_flips"),
    ("forward steering flips", "forward_steering_sign_flips"),
    ("steer energy rad/m", "forward_angular_energy_per_m"),
    ("straight path (m)", "straight_path_distance_m"),
    ("straight steer energy rad/m", "straight_path_angular_energy_per_m"),
    ("straight steering flips", "straight_path_steering_sign_flips"),
    ("min clearance (m)", "min_scan_clearance_m"),
    ("path (m)", "path_length_m"),
    ("target seen->lock (s)", "target_first_seen_to_lock_seconds"),
    ("target lock->done (s)", "target_lock_to_done_seconds"),
    ("target seen->done (s)", "target_first_seen_to_done_seconds"),
    ("task done (s)", "task_done_seconds"),
]


def _resolve_log(path):
    path = Path(path)
    if path.is_dir():
        matches = sorted(path.glob("*_navigation_metrics.log"))
        return matches[-1] if matches else None
    return path if path.is_file() else None


def compare_runs(base_a, base_b):
    base_a = _resolve_log(base_a)
    base_b = _resolve_log(base_b)
    if base_a is None or base_b is None:
        raise SystemExit("compare: could not find a navigation metrics log in both paths")
    result_a = summarize(base_a)
    result_b = summarize(base_b)
    print("A/B navigation smoothness comparison")
    print("  A: %s (id=%s)" % (result_a["log"], result_a["run_id"]))
    print("  B: %s (id=%s)" % (result_b["log"], result_b["run_id"]))
    print("%-20s %12s %12s %10s" % ("metric", "A", "B", "delta"))
    print("-" * 58)
    for label, key in COMPARE_FIELDS:
        value_a = result_a.get(key)
        value_b = result_b.get(key)
        text_a = "-" if value_a is None else str(value_a)
        text_b = "-" if value_b is None else str(value_b)
        delta = ""
        try:
            delta = "%.3g" % (float(value_b) - float(value_a))
        except (TypeError, ValueError):
            delta = ""
        print("%-20s %12s %12s %10s" % (label, text_a, text_b, delta))


def diagnose(path):
    """Print the concrete event sequences behind the smoothness metrics.

    Rather than dumping every record, extract the three problem patterns the
    navigation task cares about: goal churn, straight-line steering wobble, and
    obstacle/safety braking. Each printed line keeps enough context (clearance,
    speed, goal distance) to distinguish a legitimate obstacle response from a
    system-side jitter stop or a goal that moved on its own.
    """
    records = load_records(path)
    # A zero command is often observed just before the corresponding action
    # status callback. The metrics node later emits an explicit association;
    # apply it here so the offline diagnostic presents the final causal label,
    # not the provisional callback-order label.
    final_discontinuity_reason = {
        record.get("discontinuity_id"): record.get("reason")
        for record in records
        if record.get("_event") == "discontinuity_reclassified"
        and record.get("discontinuity_id") is not None
    }
    print("Diagnostic trace: %s (%d records)" % (path, len(records)))
    print("-" * 78)
    for record in records:
        event = record.get("_event", "")
        if event == "goal_change":
            print("GOAL   t=%.1f delta=%.2fm goal=%s prev=%s source=%s transition=%s cmd=(%.2f,%.2f)" % (
                record.get("ros_time") or 0.0,
                record.get("delta_m") or 0.0,
                record.get("goal"),
                record.get("previous_goal"),
                record.get("goal_source"),
                record.get("transition_kind", "unknown"),
                (record.get("command") or [0, 0])[0],
                (record.get("command") or [0, 0])[1],
            ))
        elif event == "linear_brake":
            clear = record.get("scan_forward_min")
            final_reason = final_discontinuity_reason.get(
                record.get("discontinuity_id"), record.get("reason", "legacy_unknown")
            )
            print("BRAKE  t=%.1f %.2f->%.2f m/s w=%.2f fwd_clear=%s reason=%s age=%s goal_dist=%s src=%s/%s" % (
                record.get("ros_time") or 0.0,
                record.get("previous_linear") or 0.0,
                record.get("current_linear") or 0.0,
                record.get("angular") or 0.0,
                clear,
                final_reason,
                record.get("lifecycle_age_seconds"),
                record.get("robot_goal_distance") if "robot_goal_distance" in record else record.get("goal"),
                record.get("controller_source"),
                record.get("controller_reason"),
            ))
        elif event == "command_stop":
            final_reason = final_discontinuity_reason.get(
                record.get("discontinuity_id"), record.get("reason", "legacy_unknown")
            )
            print("STOP   t=%.1f reason=%s age=%s mode=%s src=%s teb_status=%s mb=%s" % (
                record.get("ros_time") or 0.0,
                final_reason,
                record.get("lifecycle_age_seconds"),
                record.get("controller_mode"),
                record.get("controller_source"),
                record.get("teb_status"),
                record.get("move_base_status"),
            ))
        elif event == "teb_raw_angular_sign_flip":
            print("FLIP   t=%.1f %.3f->%.3f rad/s clear=%.2f status=%s" % (
                record.get("ros_time") or 0.0,
                record.get("previous_angular") or 0.0,
                record.get("angular") or 0.0,
                record.get("scan_forward_min") or float("nan"),
                record.get("teb_status"),
            ))
        elif event == "teb_bridge_event" and record.get("bridge_event") == "dispatch":
            print("DISP   t=%.1f %s goal=%s kind=%s reason=%s" % (
                record.get("ros_time") or 0.0,
                record.get("active_intent_source"),
                record.get("dispatched_goal"),
                record.get("route_kind"),
                record.get("reason"),
            ))
        elif event == "teb_bridge_event" and record.get("bridge_event") == "handoff_requested":
            print("HANDOFF t=%.1f reason=%s pending_delta=%s" % (
                record.get("ros_time") or 0.0,
                record.get("reason"),
                record.get("pending_delta"),
            ))
        elif event == "move_base_recovery":
            print("RECOVER t=%.1f %s" % (record.get("ros_time") or 0.0, record.get("behavior")))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path, help="metrics log or run directory")
    parser.add_argument(
        "--root",
        type=Path,
        default=Path("runtime/navigation/logs"),
        help="navigation log root when no log path is supplied",
    )
    parser.add_argument("--json", action="store_true", help="print machine-readable JSON")
    parser.add_argument(
        "--diagnose",
        action="store_true",
        help="print the raw goal/brake/flip/dispatch event trace for a log",
    )
    parser.add_argument(
        "--failures",
        action="store_true",
        help="print correlated failure IDs plus trigger/end evidence",
    )
    parser.add_argument(
        "--compare",
        nargs=2,
        type=Path,
        metavar=("A", "B"),
        help="print an A/B comparison of two metrics log paths/directories",
    )
    args = parser.parse_args()

    if args.compare:
        compare_runs(args.compare[0], args.compare[1])
        return

    path = args.log
    if path is None:
        path = latest_log(args.root)
    elif path.is_dir():
        matches = sorted(path.glob("*_navigation_metrics.log"))
        path = matches[-1] if matches else None
    if path is None or not path.is_file():
        parser.error("navigation metrics log not found")
    if args.diagnose:
        diagnose(path)
        return
    result = summarize(path)
    if args.failures:
        evidence = result.get("failure_evidence") or {}
        print(json.dumps({
            "startup_failure": result.get("startup_failure"),
            "episodes": evidence,
        }, indent=2, sort_keys=True))
        return
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print_human(result)


if __name__ == "__main__":
    main()
