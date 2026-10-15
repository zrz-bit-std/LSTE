#!/usr/bin/env python3
"""Summarize one LSTE navigation metrics log for the office benchmark."""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter
from pathlib import Path

import yaml

import sys


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from graph_event_reducer import replay_graph_events


ROOT = Path(__file__).resolve().parents[3]
LOG_ROOTS = (
    ROOT / "runtime/office_building_benchmark/logs",
    ROOT / "runtime/navigation/logs",
)
EVENT_RE = re.compile(r"event=(\S+) data=(\{.*\})$")
DEFAULT_MANIFEST = ROOT / "worlds/benchmark/office_building_v1_manifest.yaml"


def read_events(path: Path):
    """Yield the structured metrics events without relying on console output."""
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            match = EVENT_RE.search(line.rstrip())
            if not match:
                continue
            event, payload = match.groups()
            try:
                yield event, json.loads(payload)
            except json.JSONDecodeError:
                continue


def room_for_pose(manifest: dict, pose):
    """Classify one Gazebo/odom pose into a benchmark room or corridor.

    The world and robot odometry share the Gazebo origin in this benchmark.
    A small boundary margin intentionally leaves doorway samples unclassified;
    ``room_visit_summary`` bridges those samples so doorway jitter never looks
    like an artificial room re-entry.
    """
    if not pose or len(pose) < 2:
        return None
    x, y = float(pose[0]), float(pose[1])
    margin = 0.12
    for room in manifest.get("rooms", []) or []:
        x0, y0, x1, y1 = [float(value) for value in room.get("bounds_m", [])]
        if x0 + margin <= x <= x1 - margin and y0 + margin <= y <= y1 - margin:
            return str(room.get("id") or "") or None
    corridor = (manifest.get("main_corridor", {}) or {}).get("bounds_m")
    if corridor and len(corridor) == 4:
        x0, y0, x1, y1 = [float(value) for value in corridor]
        if x0 + margin <= x <= x1 - margin and y0 + margin <= y <= y1 - margin:
            return "main_corridor"
    return None


def room_visit_summary(samples, manifest: dict):
    """Return hysteresis-stable physical-room visits from metric samples."""
    visits = []
    active = None
    pending = None
    pending_count = 0
    # Two samples prevent an odometry point at a door threshold from creating
    # an enter/leave pair. Sample cadence is 2 Hz, so this is a one-second
    # spatial confirmation rather than an arbitrary dwell timeout.
    for sample in samples:
        now = sample.get("ros_time")
        if now is None:
            continue
        label = room_for_pose(manifest, sample.get("pose"))
        if label is None:
            continue
        if label == active:
            pending = None
            pending_count = 0
            if visits:
                visits[-1]["last_seen_seconds"] = float(now)
            continue
        if label != pending:
            pending = label
            pending_count = 1
            continue
        pending_count += 1
        if pending_count < 2:
            continue
        if visits:
            visits[-1]["exited_seconds"] = float(now)
            visits[-1]["duration_seconds"] = round(
                max(0.0, float(now) - visits[-1]["entered_seconds"]), 3
            )
        active = label
        visits.append(
            {
                "region": label,
                "entered_seconds": float(now),
                "last_seen_seconds": float(now),
                "exited_seconds": None,
                "duration_seconds": None,
            }
        )
        pending = None
        pending_count = 0
    if visits and visits[-1]["duration_seconds"] is None:
        last_seen = float(visits[-1]["last_seen_seconds"])
        visits[-1]["duration_seconds"] = round(
            max(0.0, last_seen - visits[-1]["entered_seconds"]), 3
        )
    counts = Counter(visit["region"] for visit in visits)
    rooms_only = {
        room: count for room, count in counts.items() if room != "main_corridor"
    }
    reentries = {
        room: count - 1 for room, count in sorted(rooms_only.items()) if count > 1
    }
    dwell_by_region = {
        room: round(
            max(
                float(visit["duration_seconds"] or 0.0)
                for visit in visits
                if visit["region"] == room
            ),
            3,
        )
        for room in sorted(counts)
    }
    return {
        "visit_sequence": visits,
        "entry_counts": dict(sorted(counts.items())),
        "room_reentries": reentries,
        "total_room_reentries": sum(reentries.values()),
        "max_dwell_seconds_by_region": dwell_by_region,
    }


def _run_world(events):
    """Return the immutable world path recorded by the run contract."""
    for event, data in events:
        if event != "run_start" or not isinstance(data, dict):
            continue
        experiment = data.get("experiment")
        if isinstance(experiment, dict):
            world = str(experiment.get("world") or "").strip()
            if world:
                return world
        world = str(data.get("world") or "").strip()
        if world:
            return world
    return ""


def scope_manifest_for_run(manifest: dict, events):
    """Scope physical-room truth to the level that actually ran.

    The benchmark manifest describes all four levels in one file. Applying
    those room rectangles to a smaller level labels empty floor as a room and
    can manufacture a room re-entry. The run-start world is the provenance
    boundary: use it to select the level's enabled rooms before classifying
    odometry samples. A missing or unmatched world keeps the historical
    all-room behavior so old logs remain readable rather than being silently
    discarded.
    """
    levels = manifest.get("levels") or {}
    if not isinstance(levels, dict) or not levels:
        return manifest, None
    world = _run_world(events)
    if not world:
        return manifest, None
    world_name = Path(world).name
    selected_level = None
    selected_profile = None
    for level_name, profile in levels.items():
        if not isinstance(profile, dict):
            continue
        declared_world = str(profile.get("world") or "").strip()
        if declared_world and Path(declared_world).name == world_name:
            selected_level = str(level_name)
            selected_profile = profile
            break
    if selected_level is None or selected_profile is None:
        return manifest, None
    enabled = {
        str(room).strip()
        for room in selected_profile.get("enabled_rooms") or ()
        if str(room).strip()
    }
    if not enabled:
        return manifest, selected_level
    scoped = dict(manifest)
    scoped["rooms"] = [
        room for room in manifest.get("rooms", []) or ()
        if str(room.get("id") or "").strip() in enabled
    ]
    if "main_corridor" not in enabled:
        scoped.pop("main_corridor", None)
    return scoped, selected_level


def work_item_execution_summary(events, run_end_seconds):
    """Measure semantic WorkItems and their individual viewpoint Attempts.

    A room dwell interval contains corridor transit, rotations, recovery, and
    pauses.  It is therefore not a measure of useful exploration.  The full
    method instead publishes a small transaction for each viewpoint Attempt.
    An Attempt can fail while its parent ObservationWorkItem remains
    unresolved and is tried later from a different safe viewpoint.  Pair by
    ``(work_item_id, attempt_id)`` so that correct retries are not reported
    as duplicate semantic work. A missing end remains censored rather than
    being silently converted into completed observation.
    """
    active = {}
    completed = []
    invalid = []
    for event, data in events:
        if event != "global_frontier_event":
            continue
        frontier_event = data.get("frontier_event")
        if frontier_event == "work_item_dispatched":
            item_id = data.get("work_item_id")
            started = data.get("ros_time")
            if item_id is None or started is None:
                invalid.append({"event": frontier_event, "data": data})
                continue
            item_id = int(item_id)
            attempt_id = data.get("attempt_id")
            try:
                attempt_id = None if attempt_id is None else int(attempt_id)
            except (TypeError, ValueError):
                invalid.append({"event": frontier_event, "data": data})
                continue
            key = (item_id, attempt_id)
            if key in active:
                invalid.append({
                    "event": "duplicate_dispatch",
                    "work_item_id": item_id,
                    "attempt_id": attempt_id,
                    "previous": active[key],
                    "data": data,
                })
            active[key] = {
                "work_item_id": item_id,
                "attempt_id": attempt_id,
                "place_id": data.get("place_id"),
                "route_id": data.get("route_id"),
                "started_seconds": float(started),
            }
        elif frontier_event in (
            "work_item_settled",
            "viewpoint_attempt_settled",
        ):
            item_id = data.get("work_item_id")
            ended = data.get("ros_time")
            if item_id is None or ended is None:
                invalid.append({"event": frontier_event, "data": data})
                continue
            item_id = int(item_id)
            attempt_id = data.get("attempt_id")
            try:
                attempt_id = None if attempt_id is None else int(attempt_id)
            except (TypeError, ValueError):
                invalid.append({"event": frontier_event, "data": data})
                continue
            key = (item_id, attempt_id)
            started = active.pop(key, None)
            if started is None:
                invalid.append({
                    "event": "unmatched_settlement",
                    "work_item_id": item_id,
                    "attempt_id": attempt_id,
                    "data": data,
                })
                continue
            elapsed = max(0.0, float(ended) - started["started_seconds"])
            completed.append({
                **started,
                "ended_seconds": float(ended),
                "duration_seconds": round(elapsed, 3),
                "state": str(data.get("work_item_state") or "unknown"),
                "attempt_state": str(
                    data.get("attempt_state")
                    or (
                        "succeeded"
                        if data.get("work_item_state") == "resolved"
                        else "unknown"
                    )
                ),
                "reason": str(data.get("reason") or "unknown"),
            })

    effective_by_place = {}
    attempt_by_place = {}
    resolved_by_place = {}
    for interval in completed:
        place_id = str(interval.get("place_id"))
        attempt_by_place[place_id] = (
            attempt_by_place.get(place_id, 0.0) + interval["duration_seconds"]
        )
        if interval["attempt_state"] == "succeeded":
            effective_by_place[place_id] = (
                effective_by_place.get(place_id, 0.0)
                + interval["duration_seconds"]
            )
        if interval["state"] == "resolved":
            resolved_by_place[place_id] = (
                resolved_by_place.get(place_id, 0.0) + interval["duration_seconds"]
            )
    censored = [
        {
            **interval,
            "censored_at_seconds": run_end_seconds,
            "elapsed_to_run_end_seconds": round(
                max(0.0, float(run_end_seconds) - interval["started_seconds"]), 3
            ),
        }
        for interval in active.values()
    ]
    duplicate_dispatch_count = sum(
        item.get("event") == "duplicate_dispatch" for item in invalid
    )
    unmatched_settlement_count = sum(
        item.get("event") == "unmatched_settlement" for item in invalid
    )
    return {
        "status": "measured" if completed or censored or invalid else "not_available",
        "completed_interval_count": len(completed),
        "attempt_interval_count": len(completed),
        "resolved_interval_count": sum(
            interval["state"] == "resolved" for interval in completed
        ),
        "succeeded_attempt_count": sum(
            interval["attempt_state"] == "succeeded" for interval in completed
        ),
        "failed_attempt_count": sum(
            interval["attempt_state"] == "failed" for interval in completed
        ),
        "blocked_attempt_count": sum(
            interval["attempt_state"] == "blocked" for interval in completed
        ),
        "effective_work_seconds_by_place": {
            place_id: round(value, 3)
            for place_id, value in sorted(effective_by_place.items())
        },
        "attempt_seconds_by_place": {
            place_id: round(value, 3)
            for place_id, value in sorted(attempt_by_place.items())
        },
        "resolved_work_seconds_by_place": {
            place_id: round(value, 3)
            for place_id, value in sorted(resolved_by_place.items())
        },
        "max_effective_work_seconds_in_one_place": (
            None
            if not effective_by_place
            else round(max(effective_by_place.values()), 3)
        ),
        "censored_active_intervals": censored,
        "invalid_lifecycle_events": invalid,
        "duplicate_dispatch_count": duplicate_dispatch_count,
        "unmatched_settlement_count": unmatched_settlement_count,
        "active_attempt_count": len(active),
        "completed_intervals": completed,
    }


def stall_evidence_summary(events):
    """Reduce route-stall signals without inferring them from map churn.

    ``route_stagnant_observed`` is an explicit watchdog fact.  A repeated
    ``frontier_route_unavailable`` sequence is a separate diagnostic: it
    means the durable graph obligation could not be materialized for the same
    route, but it is not itself a controller failure.  Keeping both records
    lets the verifier distinguish a physical execution stall from a planning
    materialization stall.
    """
    stagnant = []
    unavailable_by_route = {}
    for event, data in events:
        if event != "global_frontier_event" or not isinstance(data, dict):
            continue
        frontier_event = str(data.get("frontier_event") or "")
        try:
            ros_time = float(data.get("ros_time"))
        except (TypeError, ValueError):
            ros_time = None
        if frontier_event == "route_stagnant_observed":
            stagnant.append({
                "route_id": data.get("route_id"),
                "route_kind": data.get("route_kind"),
                "reason": data.get("reason"),
                "active_elapsed_seconds": data.get("active_elapsed"),
                "ros_time": ros_time,
            })
        elif frontier_event == "frontier_route_unavailable":
            route_id = data.get("route_id")
            try:
                route_id = int(route_id)
            except (TypeError, ValueError):
                route_id = 0
            unavailable_by_route.setdefault(route_id, []).append({
                "ros_time": ros_time,
                "reason": data.get("reason"),
            })

    episodes = []
    for route_id, records in sorted(unavailable_by_route.items()):
        timed = [record for record in records if record["ros_time"] is not None]
        if not timed:
            episodes.append({
                "route_id": route_id,
                "event_count": len(records),
                "start_seconds": None,
                "end_seconds": None,
                "span_seconds": None,
            })
            continue
        start = timed[0]["ros_time"]
        end = timed[-1]["ros_time"]
        episodes.append({
            "route_id": route_id,
            "event_count": len(records),
            "start_seconds": round(start, 3),
            "end_seconds": round(end, 3),
            "span_seconds": round(max(0.0, end - start), 3),
            "reasons": sorted({
                str(record["reason"])
                for record in records
                if record.get("reason")
            }),
        })
    spans = [
        episode["span_seconds"]
        for episode in episodes
        if episode.get("span_seconds") is not None
    ]
    return {
        "route_stagnant_observed_count": len(stagnant),
        "route_stagnant_observations": stagnant,
        "frontier_route_unavailable_event_count": sum(
            len(records) for records in unavailable_by_route.values()
        ),
        "frontier_route_unavailable_route_count": len(unavailable_by_route),
        "frontier_route_unavailable_episodes": episodes,
        "max_frontier_route_unavailable_span_seconds": (
            None if not spans else round(max(spans), 3)
        ),
    }


_ROUTE_TERMINAL_FRONTIER_EVENTS = frozenset(
    (
        "frontier_endpoint_observed",
        "portal_place_entered",
        "portal_place_covered_arrival",
        "frontier_route_failed",
        "route_invalidated",
        "persistent_execution_terminal",
    )
)


def _event_time(data):
    """Read a finite ROS timestamp from one structured event."""
    try:
        value = float(data.get("ros_time"))
    except (AttributeError, TypeError, ValueError):
        return None
    return value if value == value and abs(value) != float("inf") else None


def _positive_route_id(value):
    try:
        value = int(value)
    except (TypeError, ValueError):
        return None
    return value if value > 0 else None


def _xy_goal(value):
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return None
    try:
        x, y = float(value[0]), float(value[1])
    except (TypeError, ValueError):
        return None
    if not all(abs(item) != float("inf") and item == item for item in (x, y)):
        return None
    return (x, y)


def _goal_delta(first, second):
    first = _xy_goal(first)
    second = _xy_goal(second)
    if first is None or second is None:
        return None
    return round(((second[0] - first[0]) ** 2 + (second[1] - first[1]) ** 2) ** 0.5, 4)


def route_goal_lifecycle_summary(events):
    """Correlate accepted route identities with goals and graph lifecycles.

    ``goal_change`` is a low-level observer counter and does not represent all
    accepted exploration actions.  The mission transaction and graph route
    events do: use them to expose actual route churn, rather than confusing
    repeated map updates or retained TEB plans with new goals.
    """
    routes = {}
    route_order = []
    mission_goals = []
    raw_goal_changes = 0
    unavailable_by_route = {}
    lifecycle_counts = Counter()

    def route_for(route_id):
        if route_id not in routes:
            routes[route_id] = {
                "route_id": route_id,
                "start_seconds": None,
                "end_seconds": None,
                "end_event": None,
                "goal": None,
                "route_kind": None,
                "graph_action": None,
                "place_id": None,
                "work_item_id": None,
                "portal_id": None,
                "selection_count": 0,
                "unavailable_event_count": 0,
            }
            route_order.append(route_id)
        return routes[route_id]

    for event, data in events:
        if not isinstance(data, dict):
            continue
        timestamp = _event_time(data)
        if event == "goal_change":
            raw_goal_changes += 1
        if event == "mission_goal_transaction":
            goal = _xy_goal(data.get("goal"))
            if goal is not None:
                mission_goals.append({
                    "ros_time": timestamp,
                    "goal": [round(goal[0], 4), round(goal[1], 4)],
                    "route_id": _positive_route_id(data.get("route_id")),
                    "route_kind": data.get("route_kind"),
                    "source": data.get("source"),
                })
        if event != "global_frontier_event":
            continue
        frontier_event = str(data.get("frontier_event") or "")
        if frontier_event:
            if frontier_event.startswith("portal_"):
                lifecycle_counts["portal." + frontier_event] += 1
            elif frontier_event.startswith("work_item") or frontier_event.startswith("viewpoint_"):
                lifecycle_counts["work_item." + frontier_event] += 1
            elif frontier_event.startswith("frontier_place") or frontier_event in {
                "portal_place_entered", "portal_place_covered_arrival",
            }:
                lifecycle_counts["place." + frontier_event] += 1

        route_id = _positive_route_id(data.get("route_id"))
        if frontier_event == "route_selected" and route_id is not None:
            route = route_for(route_id)
            route["selection_count"] += 1
            if route["start_seconds"] is None:
                route["start_seconds"] = timestamp
            for key in ("goal", "route_kind", "place_id", "work_item_id", "portal_id"):
                if route.get(key) is None and data.get(key) is not None:
                    route[key] = data.get(key)
            route["graph_action"] = (
                data.get("graph_action") or route.get("graph_action")
            )
        elif frontier_event == "route_command" and route_id is not None:
            route = route_for(route_id)
            if route["start_seconds"] is None:
                route["start_seconds"] = timestamp
            for key in ("goal", "route_kind", "place_id", "work_item_id", "portal_id"):
                if route.get(key) is None and data.get(key) is not None:
                    route[key] = data.get(key)
        elif frontier_event == "frontier_route_unavailable":
            if route_id is not None:
                unavailable_by_route.setdefault(route_id, []).append({
                    "ros_time": timestamp,
                    "reason": data.get("reason"),
                })
                route_for(route_id)["unavailable_event_count"] += 1

        if frontier_event in _ROUTE_TERMINAL_FRONTIER_EVENTS and route_id is not None:
            route = route_for(route_id)
            if route["end_seconds"] is None or (
                timestamp is not None and timestamp >= route["end_seconds"]
            ):
                route["end_seconds"] = timestamp
                route["end_event"] = frontier_event

    deltas = []
    duplicate_transactions = 0
    for previous, current in zip(mission_goals, mission_goals[1:]):
        delta = _goal_delta(previous.get("goal"), current.get("goal"))
        if delta is None:
            continue
        deltas.append(delta)
        if delta <= 0.05:
            duplicate_transactions += 1

    route_records = []
    for route_id in route_order:
        route = dict(routes[route_id])
        if route["start_seconds"] is not None and route["end_seconds"] is not None:
            route["duration_seconds"] = round(
                max(0.0, route["end_seconds"] - route["start_seconds"]), 3
            )
        else:
            route["duration_seconds"] = None
        route_records.append(route)

    return {
        "accepted_route_count": len(route_records),
        "route_reselection_count": sum(
            max(0, route["selection_count"] - 1) for route in route_records
        ),
        "routes": route_records,
        "mission_goal_transaction_count": len(mission_goals),
        "mission_goal_transactions": mission_goals,
        "mission_goal_delta_mean_m": (
            None if not deltas else round(sum(deltas) / len(deltas), 4)
        ),
        "mission_goal_delta_max_m": None if not deltas else max(deltas),
        "near_duplicate_mission_goal_count": duplicate_transactions,
        "raw_goal_change_event_count": raw_goal_changes,
        "frontier_route_unavailable_total": sum(
            len(records) for records in unavailable_by_route.values()
        ),
        "frontier_route_unavailable_routes": sorted(unavailable_by_route),
        "lifecycle_event_counts": dict(sorted(lifecycle_counts.items())),
    }


def transition_topology_cache_summary(events):
    """Summarize endpoint-rooted BFS work by active route lease."""
    records = []
    for event, data in events:
        if event != "global_frontier_event" or not isinstance(data, dict):
            continue
        if data.get("frontier_event") != "transition_topology_cached":
            continue
        route_id = _positive_route_id(data.get("route_id"))
        if route_id is None:
            continue
        endpoint = data.get("endpoint")
        records.append({
            "route_id": route_id,
            "endpoint": endpoint,
            "grid_shape": data.get("grid_shape"),
            "map_resolution": data.get("map_resolution"),
            "map_origin": data.get("map_origin"),
            "structural_revision": data.get("structural_revision"),
            "ros_time": _event_time(data),
        })
    by_route = Counter(item["route_id"] for item in records)
    return {
        "build_count": len(records),
        "unique_route_count": len(by_route),
        "rebuild_count": sum(max(0, count - 1) for count in by_route.values()),
        "builds_by_route": dict(sorted(by_route.items())),
        "records": records,
    }


def termination_diagnosis(events, final_sample):
    """Explain why a trial ended without claiming a failure without evidence."""
    final_sample = final_sample if isinstance(final_sample, dict) else {}
    route_events = []
    unavailable = []
    route_kinds = {}
    terminal_failures = []
    for event, data in events:
        if not isinstance(data, dict):
            continue
        if event == "global_frontier_event":
            frontier_event = str(data.get("frontier_event") or "")
            route_id = _positive_route_id(data.get("route_id"))
            if route_id is not None:
                if data.get("route_kind"):
                    route_kinds[route_id] = data.get("route_kind")
                route_events.append((
                    _event_time(data), route_id, data.get("route_kind"),
                    frontier_event, data,
                ))
            if frontier_event == "frontier_route_unavailable":
                unavailable.append(data)
        elif event in {
            "failure_started", "failure_snapshot_ready", "route_failure",
        }:
            terminal_failures.append((event, data))

    latest_route = route_events[-1] if route_events else None
    latest_route_id = None if latest_route is None else latest_route[1]
    latest_route_kind = (
        None
        if latest_route is None
        else latest_route[2] or route_kinds.get(latest_route_id)
    )
    route_identity = final_sample.get("route_identity")
    if isinstance(route_identity, dict):
        latest_route_id = (
            _positive_route_id(route_identity.get("active_route_id"))
            or _positive_route_id(route_identity.get("route_id"))
            or latest_route_id
        )
        latest_route_kind = (
            route_identity.get("active_route_kind")
            or route_identity.get("route_kind")
            or latest_route_kind
        )
    feedback = final_sample.get("move_base_feedback")
    feedback = feedback if isinstance(feedback, dict) else {}
    status_name = str(
        final_sample.get("move_base_status") or feedback.get("status_name") or ""
    ).upper()
    goal_distance = final_sample.get("distance_to_goal")
    bridge_active = bool(final_sample.get("teb_bridge_active"))
    command = final_sample.get("cmd") or final_sample.get("cmd_vel") or [0.0, 0.0]
    try:
        command_active = abs(float(command[0])) > 0.05 or abs(float(command[1])) > 0.03
    except (TypeError, ValueError, IndexError):
        command_active = False
    navfn = final_sample.get("navfn_plan")
    navfn = navfn if isinstance(navfn, dict) else {}
    plan_available = bool(navfn.get("poses")) or str(
        final_sample.get("teb_status") or ""
    ) == "trajectory_valid"
    goal_tolerance = 0.50
    candidates = [final_sample.get("teb_xy_goal_tolerance")]
    for event, data in events:
        if event != "run_start" or not isinstance(data, dict):
            continue
        params = data.get("resolved_params")
        if isinstance(params, dict):
            candidates.append(params.get("teb_xy_goal_tolerance"))
        break
    for candidate in candidates:
        try:
            value = float(candidate)
        except (TypeError, ValueError):
            continue
        if value >= 0.05:
            goal_tolerance = value
            break
    try:
        goal_distance_value = float(goal_distance)
    except (TypeError, ValueError):
        goal_distance_value = None
    within_goal_tolerance = (
        goal_distance_value is not None
        and goal_distance_value >= 0.0
        and goal_distance_value <= goal_tolerance
    )

    if bool(final_sample.get("task_done")):
        state, reason = "completed", "task_done"
    elif status_name in {"ABORTED", "REJECTED", "LOST"}:
        state, reason = "failure", "terminal_failure_evidence"
    elif bridge_active and within_goal_tolerance:
        state, reason = "terminal_settle", "within_goal_tolerance_waiting_for_terminal"
    elif bridge_active and command_active and plan_available:
        state, reason = "in_progress", "active_route_not_finished_at_trial_end"
    elif terminal_failures:
        state, reason = "failure", "terminal_failure_evidence"
    elif bridge_active and unavailable:
        state, reason = "planning_wait", "frontier_materialization_wait_at_trial_end"
    elif bridge_active and not command_active and plan_available:
        state, reason = "stall_candidate", "active_route_without_effective_command"
    elif bridge_active:
        state, reason = "active_without_plan", "active_route_without_executable_plan"
    else:
        state, reason = "idle", "no_active_controller_route"

    result = {
        "state": state,
        "reason": reason,
        "trial_end_ros_time": final_sample.get("ros_time"),
        "pose": final_sample.get("pose"),
        "goal": final_sample.get("goal"),
        "distance_to_goal_m": goal_distance,
        "goal_tolerance_m": round(goal_tolerance, 4),
        "within_goal_tolerance": within_goal_tolerance,
        "move_base_status": status_name or None,
        "bridge_active": bridge_active,
        "command_active": command_active,
        "plan_available": plan_available,
        "active_route_id": latest_route_id,
        "active_route_kind": latest_route_kind,
        "last_frontier_event": None if latest_route is None else latest_route[3],
        "frontier_route_unavailable_count": sum(
            _positive_route_id(data.get("route_id")) == latest_route_id
            for data in unavailable
        ),
        "terminal_failure_event_count": len(terminal_failures),
    }
    return result


def frontier_region_summary(events):
    """Summarize the explicit place lifecycle independently of room labels."""
    selected = Counter()
    endpoint_observations = Counter()
    normal_closures = Counter()
    suspended_places = Counter()
    exceptional_retirements = Counter()
    prefetch_deferred = Counter()
    route_regions = {}
    route_region_conflicts = []
    sealed_portal_reentry_skips = 0
    sealed_portal_reentry_skips_by_region = Counter()
    sealed_portal_reentry_gates = Counter()
    selected_portal_routes = []
    reverse_egress_routes = []
    portal_entry_records = []
    selected_work_items = Counter()
    dispatched_work_items = Counter()
    resolved_work_items = set()
    repeated_resolved_work_item_dispatches = Counter()
    resolved_descendant_dispatches = []
    portal_observation_probes = []
    semantic_task_updates = 0
    semantic_place_evidence = 0
    semantic_topology_expansions = 0
    portal_hypothesis_events = Counter()
    portal_hypothesis_ids = set()
    portal_probe_events = Counter()
    portal_probe_ids = set()
    portal_probe_results = Counter()
    portal_probe_value_categories = Counter()
    portal_probe_value_rejection_reasons = Counter()
    portal_probe_value_candidate_total = 0
    portal_probe_value_feasible_total = 0
    portal_probe_value_pareto_total = 0
    portal_destination_classes = Counter()
    target_belief_updates = 0
    target_belief_geometry_bindings = 0
    target_direction_candidates = 0
    graph_action_counts = Counter()
    graph_action_reasons = Counter()
    graph_policy_rejections = 0
    frontier_decision_categories = Counter()
    frontier_decision_policies = Counter()
    frontier_decision_candidate_total = 0
    frontier_decision_feasible_total = 0
    frontier_decision_pareto_total = 0
    graph_route_plan_statuses = Counter()
    graph_route_plan_actions = Counter()
    graph_route_committed_actions = Counter()
    graph_route_plan_count = 0
    graph_route_edge_materialized = 0
    graph_route_multihop_plan_count = 0
    graph_route_max_hops = 0
    graph_route_plan_mismatch_count = 0
    graph_route_plan_reconciliation_count = 0
    route_plan_action_mismatch_count = 0
    for event, data in events:
        if event != "global_frontier_event":
            continue
        frontier_event = data.get("frontier_event")
        region_id = data.get("active_region_id")
        if frontier_event == "graph_route_plan_selected":
            plan = data.get("graph_route_plan")
            if not isinstance(plan, dict):
                plan = data
            status = str(plan.get("status") or "unknown")
            action = str(plan.get("action") or "unknown")
            graph_route_plan_statuses[status] += 1
            graph_route_plan_actions[action] += 1
            graph_route_plan_count += 1
            path = plan.get("portal_path") or ()
            try:
                hop_count = len(path)
            except TypeError:
                hop_count = 0
            graph_route_max_hops = max(graph_route_max_hops, hop_count)
            if hop_count > 1:
                graph_route_multihop_plan_count += 1
            continue
        if frontier_event == "graph_route_edge_materialized":
            graph_route_edge_materialized += 1
            continue
        if frontier_event == "graph_route_action_committed":
            action = data.get("action")
            if action:
                graph_route_committed_actions[str(action)] += 1
            continue
        if frontier_event == "graph_route_plan_mismatch":
            graph_route_plan_mismatch_count += 1
            continue
        if frontier_event == "graph_route_plan_reconciled":
            graph_route_plan_reconciliation_count += 1
            continue
        if frontier_event == "frontier_action_selected":
            category = data.get("category")
            if category:
                frontier_decision_categories[str(category)] += 1
            policy = data.get("policy") or data.get("frontier_action_policy")
            if policy:
                frontier_decision_policies[str(policy)] += 1
            frontier_decision_candidate_total += int(
                data.get("candidate_count") or 0
            )
            frontier_decision_feasible_total += int(
                data.get("feasible_count") or 0
            )
            frontier_decision_pareto_total += int(
                data.get("pareto_front_count") or 0
            )
            continue
        if frontier_event == "semantic_task_updated":
            semantic_task_updates += 1
            continue
        if frontier_event == "semantic_place_evidence":
            semantic_place_evidence += 1
            continue
        if frontier_event == "semantic_topology_expansion_selected":
            semantic_topology_expansions += 1
            continue
        if frontier_event == "target_belief_updated":
            target_belief_updates += 1
            continue
        if frontier_event == "target_belief_geometry_bound":
            target_belief_geometry_bindings += 1
            continue
        if str(frontier_event or "").startswith("portal_hypothesis_"):
            portal_hypothesis_events[frontier_event] += 1
            portal_id = data.get("portal_id")
            if portal_id is not None:
                portal_hypothesis_ids.add(int(portal_id))
            continue
        if str(frontier_event or "").startswith("portal_probe_"):
            portal_probe_events[frontier_event] += 1
            if frontier_event == "portal_probe_value_selected":
                category = data.get("action_category")
                if category:
                    portal_probe_value_categories[str(category)] += 1
                portal_probe_value_candidate_total += int(
                    data.get("candidate_count") or 0
                )
                portal_probe_value_feasible_total += int(
                    data.get("feasible_count") or 0
                )
                portal_probe_value_pareto_total += int(
                    data.get("pareto_front_count") or 0
                )
                for rejection in data.get("rejected") or ():
                    for reason in rejection.get("reasons") or ():
                        portal_probe_value_rejection_reasons[str(reason)] += 1
            if frontier_event == "portal_probe_settled":
                portal_probe_results[str(data.get("result") or "unknown")] += 1
            probe_id = data.get("probe_id")
            if probe_id is not None:
                portal_probe_ids.add(int(probe_id))
            continue
        if frontier_event == "work_item_dispatched":
            work_item_id = data.get("work_item_id")
            if work_item_id is not None:
                work_item_id = int(work_item_id)
                dispatched_work_items[work_item_id] += 1
                if work_item_id in resolved_work_items:
                    repeated_resolved_work_item_dispatches[work_item_id] += 1
            continue
        if (
            frontier_event == "work_item_settled"
            and data.get("work_item_state") == "resolved"
        ):
            work_item_id = data.get("work_item_id")
            if work_item_id is not None:
                resolved_work_items.add(int(work_item_id))
            continue
        if frontier_event in (
            "route_selected",
            "route_command",
            "terminal_prefetch_promoted",
        ):
            route_id = data.get("route_id")
            if route_id is not None and region_id is not None:
                route_id = int(route_id)
                region_id = int(region_id)
                previous = route_regions.get(route_id)
                if previous is not None and previous != region_id:
                    route_region_conflicts.append(
                        {"route_id": route_id, "first_region_id": previous,
                         "later_region_id": region_id}
                    )
                route_regions[route_id] = region_id
            if frontier_event == "route_selected":
                graph_action = data.get("graph_action")
                if graph_action:
                    graph_action_counts[str(graph_action)] += 1
                graph_plan = data.get("graph_route_plan")
                if (
                    isinstance(graph_plan, dict)
                    and graph_action
                    and graph_plan.get("action")
                    and str(graph_plan["action"]) != str(graph_action)
                ):
                    route_plan_action_mismatch_count += 1
                graph_reason = data.get("graph_action_reason")
                if graph_reason:
                    graph_action_reasons[str(graph_reason)] += 1
                graph_policy_rejections += int(
                    data.get("graph_policy_rejections") or 0
                )
                destination_class = data.get("portal_destination_class")
                if destination_class:
                    portal_destination_classes[str(destination_class)] += 1
                target_direction_candidates += int(
                    data.get("target_direction_candidates") or 0
                )
                work_item_id = data.get("work_item_id")
                if work_item_id is not None:
                    selected_work_items[int(work_item_id)] += 1
                    if (
                        data.get("work_item_match") == "resolved_descendant"
                        and data.get("graph_action") != "probe_portal"
                    ):
                        resolved_descendant_dispatches.append({
                            "route_id": data.get("route_id"),
                            "work_item_id": int(work_item_id),
                            "goal": data.get("goal"),
                        })
                probe = data.get("portal_observation_probe")
                if isinstance(probe, dict):
                    portal_observation_probes.append({
                        "route_id": data.get("route_id"),
                        "work_item_id": work_item_id,
                        "opening_cell": probe.get("opening_cell"),
                        "normal": probe.get("normal"),
                    })
            skip_count = int(data.get("sealed_portal_reentry_skips") or 0)
            sealed_portal_reentry_skips += skip_count
            skipped_region = data.get("sealed_portal_reentry_region_id")
            if skip_count and skipped_region is not None:
                sealed_portal_reentry_skips_by_region[int(skipped_region)] += skip_count
            gate = data.get("sealed_portal_reentry_gate")
            if skip_count and isinstance(gate, list) and len(gate) == 2:
                sealed_portal_reentry_gates[
                    "%0.3f,%0.3f" % (float(gate[0]), float(gate[1]))
                ] += skip_count
            selected_gate = data.get("portal_gate")
            if isinstance(selected_gate, list) and len(selected_gate) == 2:
                route = {
                    "route_id": None if route_id is None else int(route_id),
                    "route_kind": str(data.get("route_kind") or ""),
                    "goal": data.get("goal"),
                    "gate": [float(selected_gate[0]), float(selected_gate[1])],
                    "reverse_egress_count": int(
                        data.get("portal_reverse_egress_count") or 0
                    ),
                    "reverse_egress_region_id": data.get(
                        "portal_reverse_egress_region_id"
                    ),
                }
                selected_portal_routes.append(route)
                if route["reverse_egress_count"]:
                    reverse_egress_routes.append(route)
        elif frontier_event in (
            "portal_place_entered",
            "portal_place_covered_arrival",
        ):
            gate = data.get("portal_gate")
            inside = data.get("goal")
            if (
                isinstance(gate, list)
                and len(gate) == 2
                and isinstance(inside, list)
                and len(inside) == 2
            ):
                portal_entry_records.append(
                    {
                        "route_id": data.get("route_id"),
                        "region_id": data.get("region_id"),
                        "gate": [float(gate[0]), float(gate[1])],
                        "inside": [float(inside[0]), float(inside[1])],
                        "source": "portal_arrival",
                    }
                )
        elif frontier_event == "frontier_region_portal_entry_recorded":
            gate = data.get("gate")
            inside = data.get("inside")
            if (
                isinstance(gate, list)
                and len(gate) == 2
                and isinstance(inside, list)
                and len(inside) == 2
            ):
                portal_entry_records.append(
                    {
                        "route_id": data.get("route_id"),
                        "region_id": data.get("region_id"),
                        "gate": [float(gate[0]), float(gate[1])],
                        "inside": [float(inside[0]), float(inside[1])],
                        "source": "frontier_endpoint",
                    }
                )
        elif frontier_event == "frontier_endpoint_observed":
            region_id = data.get("region_id")
            if region_id is not None:
                endpoint_observations[int(region_id)] += 1
        elif frontier_event == "frontier_place_closed":
            region_id = data.get("region_id")
            if region_id is not None:
                normal_closures[int(region_id)] += 1
        elif frontier_event == "frontier_place_suspended":
            region_id = data.get("region_id")
            if region_id is not None:
                suspended_places[int(region_id)] += 1
        elif frontier_event == "frontier_region_dormant":
            region_id = data.get("region_id")
            if region_id is None:
                continue
            # Logs from the endpoint-driven implementation used this event
            # for normal room closure. Preserve that historical evidence,
            # while keeping genuine failures and stagnation separate from the
            # new place-completion transition.
            if data.get("reason") == "place_observation_complete":
                normal_closures[int(region_id)] += 1
            else:
                exceptional_retirements[int(region_id)] += 1
        elif frontier_event == "frontier_prefetch_deferred":
            reason = str(data.get("reason") or "unknown")
            prefetch_deferred[reason] += 1
    selected.update(route_regions.values())
    return {
        "route_region_sequence": [
            route_regions[route_id] for route_id in sorted(route_regions)
        ],
        "route_region_conflicts": route_region_conflicts,
        "route_count": len(route_regions),
        "region_activation_counts": dict(sorted(selected.items())),
        "repeated_region_activations": {
            str(region_id): count - 1
            for region_id, count in sorted(selected.items())
            if count > 1
        },
        "endpoint_observation_counts": dict(sorted(endpoint_observations.items())),
        "normal_place_closure_counts": dict(sorted(normal_closures.items())),
        "suspended_place_counts": dict(sorted(suspended_places.items())),
        "repeated_normal_place_closures": {
            str(region_id): count - 1
            for region_id, count in sorted(normal_closures.items())
            if count > 1
        },
        "exceptional_region_retirement_counts": dict(
            sorted(exceptional_retirements.items())
        ),
        "cross_place_prefetch_deferred_counts": dict(
            sorted(prefetch_deferred.items())
        ),
        "sealed_portal_reentry_skips": sealed_portal_reentry_skips,
        "sealed_portal_reentry_skips_by_region": dict(
            sorted(sealed_portal_reentry_skips_by_region.items())
        ),
        "sealed_portal_reentry_skips_by_gate": dict(
            sorted(sealed_portal_reentry_gates.items())
        ),
        "selected_portal_routes": selected_portal_routes,
        "reverse_egress_routes": reverse_egress_routes,
        "portal_entry_record_count": len(portal_entry_records),
        "portal_entry_records": portal_entry_records,
        "selected_work_item_counts": dict(sorted(selected_work_items.items())),
        "work_item_dispatch_counts": dict(sorted(dispatched_work_items.items())),
        "viewpoint_attempt_counts": dict(sorted(dispatched_work_items.items())),
        "repeated_work_item_dispatches": {
            str(item_id): count
            for item_id, count in sorted(
                repeated_resolved_work_item_dispatches.items()
            )
        },
        "resolved_work_item_descendant_dispatches": resolved_descendant_dispatches,
        "portal_observation_probe_routes": portal_observation_probes,
        "semantic_task_updates": semantic_task_updates,
        "semantic_place_evidence_events": semantic_place_evidence,
        "semantic_topology_expansion_events": semantic_topology_expansions,
        "portal_hypothesis_event_counts": dict(sorted(portal_hypothesis_events.items())),
        "portal_hypothesis_count": len(portal_hypothesis_ids),
        "portal_probe_event_counts": dict(sorted(portal_probe_events.items())),
        "portal_probe_count": len(portal_probe_ids),
        "portal_probe_result_counts": dict(sorted(portal_probe_results.items())),
        "portal_probe_value_selection_count": int(
            portal_probe_events.get("portal_probe_value_selected", 0)
        ),
        "portal_probe_value_action_categories": dict(
            sorted(portal_probe_value_categories.items())
        ),
        "portal_probe_value_candidate_total": portal_probe_value_candidate_total,
        "portal_probe_value_feasible_total": portal_probe_value_feasible_total,
        "portal_probe_value_pareto_total": portal_probe_value_pareto_total,
        "portal_probe_value_rejection_reasons": dict(
            sorted(portal_probe_value_rejection_reasons.items())
        ),
        "portal_destination_class_counts": dict(
            sorted(portal_destination_classes.items())
        ),
        "target_belief_updates": target_belief_updates,
        "target_belief_geometry_bindings": target_belief_geometry_bindings,
        "target_direction_candidates": target_direction_candidates,
        "graph_action_counts": dict(sorted(graph_action_counts.items())),
        "graph_action_reasons": dict(sorted(graph_action_reasons.items())),
        "graph_policy_rejections": graph_policy_rejections,
        "frontier_decision_category_counts": dict(
            sorted(frontier_decision_categories.items())
        ),
        "frontier_decision_policy_counts": dict(
            sorted(frontier_decision_policies.items())
        ),
        "frontier_decision_count": int(sum(frontier_decision_categories.values())),
        "frontier_decision_candidate_total": frontier_decision_candidate_total,
        "frontier_decision_feasible_total": frontier_decision_feasible_total,
        "frontier_decision_pareto_total": frontier_decision_pareto_total,
        "graph_route_plan_count": graph_route_plan_count,
        "graph_route_plan_status_counts": dict(
            sorted(graph_route_plan_statuses.items())
        ),
        "graph_route_plan_action_counts": dict(
            sorted(graph_route_plan_actions.items())
        ),
        "graph_route_committed_action_counts": dict(
            sorted(graph_route_committed_actions.items())
        ),
        "graph_route_edge_materialized": graph_route_edge_materialized,
        "graph_route_multihop_plan_count": graph_route_multihop_plan_count,
        "graph_route_max_hops": graph_route_max_hops,
        "graph_route_plan_mismatch_count": graph_route_plan_mismatch_count,
        "graph_route_plan_reconciliation_count": (
            graph_route_plan_reconciliation_count
        ),
        "route_plan_action_mismatch_count": route_plan_action_mismatch_count,
        "graph_invariants": replay_graph_events(events),
    }


def exploration_method_summary(events):
    """Read the method contract from route events, never from a filename."""
    methods = []
    policies = []
    for event, data in events:
        if event != "global_frontier_event":
            continue
        goal_context = data.get("goal_context")
        if not isinstance(goal_context, dict):
            goal_context = {}
        semantic_place = data.get("semantic_place")
        if not isinstance(semantic_place, dict):
            semantic_place = {}
        # New status events expose the contract at the top level.  The nested
        # fallbacks keep already-recorded benchmark runs comparable: older
        # logs carried the method in goal_context and the policy in
        # semantic_place instead.
        method = data.get("exploration_method") or goal_context.get(
            "exploration_method"
        )
        if method is not None and str(method).strip():
            methods.append(str(method).strip())
        policy = data.get("frontier_action_policy") or semantic_place.get(
            "frontier_action_policy"
        )
        if policy is not None and str(policy).strip():
            policies.append(str(policy).strip())
    unique = sorted(set(methods))
    unique_policies = sorted(set(policies))
    return {
        "selected_method": unique[0] if len(unique) == 1 else None,
        "methods_observed": unique,
        # Silence is not evidence of a contract match.  A run that never
        # emitted a frontier event cannot be compared to a declared method.
        "method_contract_consistent": bool(unique) and len(unique) == 1,
        "frontier_action_policies": unique_policies,
        "frontier_action_policy_consistent": bool(unique_policies)
        and len(unique_policies) == 1,
    }


def occupancy_coverage_summary(sample):
    """Keep raw map growth separate from manifest-defined coverage truth."""
    stats = (sample or {}).get("map") or {}
    known = int(stats.get("free") or 0) + int(stats.get("occupied") or 0)
    unknown = int(stats.get("unknown") or 0)
    total = known + unknown
    truth = (sample or {}).get("benchmark_coverage") or {}
    return {
        "building_truth_fraction": truth.get("building_truth_fraction"),
        "truth_definition": truth.get("definition"),
        "truth_status": truth.get("status"),
        "truth_sample_count": truth.get("truth_sample_count"),
        "truth_known_sample_count": truth.get("known_sample_count"),
        "truth_region_known_fraction": truth.get("region_known_fraction"),
        "map_known_fraction": (
            None if total <= 0 else round(float(known) / float(total), 6)
        ),
        "known_cells": known,
        "unknown_cells": unknown,
        "status": truth.get("status", "map_extent_diagnostic_only"),
    }


def motion_and_failure_summary(sample, task_done):
    """Select comparable observer-only motion and failure evidence."""
    sample = sample or {}
    collision_truth = sample.get("benchmark_collision_truth") or {}
    collision_events = (
        collision_truth.get("collision_events")
        if collision_truth.get("status") == "measured" else None
    )
    failure_counts = {
        "move_base_aborts": int(sample.get("move_base_aborts") or 0),
        "move_base_unexpected_preemptions": int(
            sample.get("move_base_unexpected_preemptions") or 0
        ),
        "target_route_failures": int(sample.get("target_route_failures") or 0),
        "safety_override_events": int(sample.get("safety_override_events") or 0),
    }
    failure_events = sum(failure_counts.values())
    target_truth = sample.get("target_geometric_evaluation") or {}
    target_truth_enabled = target_truth.get("enabled")
    target_truth_available = (
        None
        if target_truth_enabled is not True
        else bool(
            target_truth.get("camera_ready")
            and target_truth.get("model_state_ready")
        )
    )
    target_truth_match = (
        None
        if target_truth_enabled is not True
        else bool(
            int(target_truth.get("matched_exposure_episodes") or 0) > 0
            or int(target_truth.get("matched_detector_frames") or 0) > 0
        )
    )
    return {
        "motion_smoothness": {
            "straight_path_angular_energy_per_m": sample.get(
                "straight_path_angular_energy_per_m"
            ),
            "straight_path_steering_sign_flips": sample.get(
                "straight_path_steering_sign_flips"
            ),
            "forward_steering_sign_flips": sample.get(
                "forward_steering_sign_flips"
            ),
            "unexplained_clear_path_brake_events": sample.get(
                "unexplained_clear_path_brake_events"
            ),
            "angular_sign_flips": sample.get("angular_sign_flips"),
            "strong_angular_sign_flips": sample.get("strong_angular_sign_flips"),
            "forward_steering_sign_flips": sample.get(
                "forward_steering_sign_flips"
            ),
            "max_stop_duration_seconds": sample.get(
                "max_stop_duration_seconds"
            ),
            "average_stop_duration_seconds": sample.get(
                "average_stop_duration_seconds"
            ),
            "stop_rate_per_minute": sample.get("stop_rate_per_minute"),
            "zero_duration_seconds": sample.get("zero_duration_seconds"),
            "turn_only_duration_seconds": sample.get("turn_only_duration_seconds"),
            "turn_only_events": sample.get("turn_only_events"),
        },
        "failure_safety": {
            "task_success": bool(task_done),
            **failure_counts,
            "failure_events": failure_events,
            "failure_rate": float(failure_events > 0),
            "collision_events": collision_events,
            "collision_rate": (
                None
                if collision_events is None
                else float(collision_events > 0)
            ),
            "collision_truth_status": collision_truth.get("status"),
            "collision_truth_messages": collision_truth.get("contact_messages"),
            "min_scan_clearance_m": sample.get("min_scan_clearance"),
        },
        "target_truth": {
            "enabled": target_truth_enabled,
            "available": target_truth_available,
            "match": target_truth_match,
            "exposed_detector_frames": target_truth.get(
                "exposed_detector_frames"
            ),
            "matched_detector_frames": target_truth.get(
                "matched_detector_frames"
            ),
            "exposure_episodes": target_truth.get("exposure_episodes"),
            "matched_exposure_episodes": target_truth.get(
                "matched_exposure_episodes"
            ),
            "episode_geometric_recall": target_truth.get(
                "episode_geometric_recall"
            ),
            "first_exposure_source_stamp": target_truth.get(
                "first_exposure_source_stamp"
            ),
            "first_match_source_stamp": target_truth.get(
                "first_match_source_stamp"
            ),
            "truth_source": target_truth.get("truth_source"),
        },
    }


def load_failure_snapshots(metrics_path: Path):
    """Load independent failure artifacts, then use the evidence log fallback.

    New runs write one bounded JSON file per closed episode.  Loading those
    files first makes summaries cheap and keeps the artifact path available to
    downstream reports.  The append-only evidence log is still parsed for old
    runs and for an episode whose artifact write was interrupted.
    """
    snapshots = {}
    pattern = metrics_path.parent.name + "_failure_*.json"
    for artifact in sorted(metrics_path.parent.glob(pattern)):
        try:
            payload = json.loads(artifact.read_text(encoding="utf-8"))
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            continue
        if not isinstance(payload, dict):
            continue
        failure_id = str(payload.get("failure_id", "")).strip()
        if failure_id:
            snapshots[failure_id] = payload

    path = metrics_path.parent / (metrics_path.parent.name + "_failure_evidence.log")
    if not path.is_file():
        return snapshots
    try:
        stream = path.open(encoding="utf-8", errors="replace")
    except OSError:
        return snapshots
    with stream:
        for line in stream:
            if " event=failure_snapshot " not in line or " data=" not in line:
                continue
            try:
                payload = json.loads(line.split(" data=", 1)[1])
            except (TypeError, ValueError, json.JSONDecodeError):
                continue
            failure_id = str(payload.get("failure_id", "")).strip()
            if failure_id:
                snapshots.setdefault(failure_id, payload)
    return snapshots


def load_trial_end_diagnostic(metrics_path: Path):
    """Load the runner-owned final-state artifact, when one exists.

    This is deliberately separate from ``load_failure_snapshots``: a trial
    deadline is an experiment boundary and must not be counted as a metrics
    failure episode.  Returning a compact view keeps the summary readable;
    ``artifact_path`` still points to the complete JSON object.
    """
    artifact = metrics_path.parent / (
        metrics_path.parent.name + "_trial_end_diagnostic.json"
    )
    try:
        payload = json.loads(artifact.read_text(encoding="utf-8"))
    except (OSError, TypeError, ValueError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict):
        return None
    diagnostic = payload.get("diagnostic")
    diagnostic = diagnostic if isinstance(diagnostic, dict) else {}
    return {
        "artifact_path": str(artifact),
        "artifact_kind": payload.get("artifact_kind"),
        "created_at": payload.get("created_at"),
        "outcome": payload.get("outcome"),
        "termination_id": diagnostic.get("termination_id")
        or payload.get("termination_id"),
        "classification": diagnostic.get("classification")
        or payload.get("classification"),
        "state": diagnostic.get("state"),
        "reason": diagnostic.get("reason"),
        "active_route_id": diagnostic.get("active_route_id"),
        "active_route_kind": diagnostic.get("active_route_kind"),
        "latest_sample": diagnostic.get("latest_sample"),
        "recent_samples": diagnostic.get("recent_samples", []),
        "progress": diagnostic.get("progress", {}),
        "evidence": diagnostic.get("evidence", {}),
        "recent_events": diagnostic.get("recent_events", []),
    }


def _compact_failure_sample(sample):
    """Keep investigator-facing fields without copying the full sample ring."""
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
            "distance_to_goal", "goal_source", "goal_transaction_id",
            "cmd_vel", "teb_cmd", "teb_planner_cmd", "teb_status",
            "teb_feedback_age_seconds", "mux_status_age_seconds",
            "move_base_status", "scan", "obstacle_clearance_threshold",
            "navfn_plan", "navfn_path_remaining_m", "navfn_path_endpoint",
            "teb_feedback", "global_costmap", "local_costmap",
            "global_costmap_window", "local_costmap_window",
            "move_base_feedback", "recovery", "route_identity", "controller", "bridge_active",
            "navigation_hold", "state", "frontier_route_unavailable_count",
            "frontier_route_unavailable_identity",
            "frontier_route_unavailable_latch",
            "frontier_route_unavailable_duration_seconds",
        )
        if key in sample
    }
    if route_context:
        result["route"] = route_context
    return result


def _compact_failure_evidence(snapshot):
    if not isinstance(snapshot, dict):
        return None
    trigger_context = snapshot.get("route_context_at_trigger")
    end_context = snapshot.get("route_context_at_end")
    if isinstance(trigger_context, dict):
        trigger_context = trigger_context.get("route", trigger_context)
    if isinstance(end_context, dict):
        end_context = end_context.get("route", end_context)
    diagnosis = snapshot.get("diagnosis") or snapshot.get("diagnosis_at_trigger")
    if isinstance(diagnosis, dict):
        diagnosis = dict(diagnosis)
        causal_route = snapshot.get("causal_route")
        if not isinstance(causal_route, dict):
            classification = snapshot.get("classification")
            classification = classification if isinstance(classification, dict) else {}
            evidence = classification.get("evidence")
            causal_route = evidence.get("route") if isinstance(evidence, dict) else {}
        if not isinstance(causal_route, dict):
            causal_route = {}
        try:
            route_missing = diagnosis.get("route_id") is None or float(
                diagnosis.get("route_id")
            ) <= 0.0
        except (TypeError, ValueError):
            route_missing = diagnosis.get("route_id") in (None, "")
        if route_missing:
            for key in ("route_id", "active_route_id", "released_route_id"):
                if causal_route.get(key) not in (None, "", 0, "0"):
                    diagnosis["route_id"] = causal_route[key]
                    break
        if not diagnosis.get("route_kind"):
            diagnosis["route_kind"] = (
                causal_route.get("route_kind")
                or causal_route.get("active_route_kind")
                or causal_route.get("released_route_kind")
                or (diagnosis.get("execution_phase") or {}).get(
                    "turn_supervisor_route_kind"
                )
            )
        if not diagnosis.get("graph_action"):
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
            diagnosis["graph_action"] = (
                graph_plan.get("action") or transaction_plan.get("action")
            )
    return {
        "artifact_path": snapshot.get("artifact_path"),
        "run_context": snapshot.get("run_context"),
        "resolution": snapshot.get("resolution"),
        "details": snapshot.get("details"),
        "causal_route": snapshot.get("causal_route"),
        "diagnosis": diagnosis,
        "diagnosis_at_trigger": snapshot.get("diagnosis_at_trigger"),
        "diagnosis_at_end": snapshot.get("diagnosis_at_end"),
        "route_context_at_trigger": trigger_context,
        "route_context_at_end": end_context,
        "trigger_sample": _compact_failure_sample(snapshot.get("trigger_sample")),
        "end_sample": _compact_failure_sample(snapshot.get("end_sample")),
        "classification_initial": snapshot.get("classification_initial"),
        "classification_at_end": snapshot.get("classification_at_end"),
        "sample_count": snapshot.get("sample_count", 0),
    }


def failure_episode_summary(events, snapshots=None):
    """Expose correlated failure IDs and their direct evidence."""
    snapshots = snapshots if isinstance(snapshots, dict) else {}
    starts = {}
    ends = {}
    related = {}
    for event, data in events:
        failure_id = str(data.get("failure_id", "")).strip()
        if not failure_id:
            continue
        if event == "failure_started":
            starts[failure_id] = data
        elif event == "failure_snapshot_ready":
            ends[failure_id] = data
        elif event == "failure_related_event":
            related[failure_id] = related.get(failure_id, 0) + 1
    episodes = []
    for failure_id, start in starts.items():
        end = ends.get(failure_id)
        initial = start.get("classification") or {}
        episode = {
            "failure_id": failure_id,
            "trigger": start.get("trigger"),
            "source": start.get("source"),
            "artifact_path": (
                (snapshots.get(failure_id) or {}).get("artifact_path")
                or (end or {}).get("artifact_path")
            ),
            "classification": (
                (end or {}).get("classification")
                or initial.get("label", "unknown")
            ),
            "confidence": (
                (end or {}).get("confidence")
                or initial.get("confidence", "low")
            ),
            "status": "closed" if end is not None else "open_at_log_end",
            "started_seconds": start.get("started_wall_elapsed_seconds"),
            "duration_seconds": None if end is None else end.get("duration_seconds"),
            "related_event_count": related.get(failure_id, 0),
        }
        evidence = _compact_failure_evidence(snapshots.get(failure_id))
        if evidence is not None:
            episode["evidence"] = evidence
            diagnosis = evidence.get("diagnosis") or evidence.get(
                "diagnosis_at_trigger"
            )
            if isinstance(diagnosis, dict):
                # Keep the causal reducer addressable beside the historical
                # class label. This matters when an artifact was generated by
                # an older classifier but contains richer trigger evidence.
                episode["technical_diagnosis"] = diagnosis
        episodes.append(episode)
    return {
        "episode_count": len(episodes),
        "closed_count": sum(item["status"] == "closed" for item in episodes),
        "open_count": sum(item["status"] == "open_at_log_end" for item in episodes),
        "episodes": episodes,
    }


def latest_log() -> Path:
    logs = []
    for root in LOG_ROOTS:
        logs.extend(root.glob("*/*_navigation_metrics.log"))
    if not logs:
        raise SystemExit(
            "no navigation metrics logs found under %s"
            % ", ".join(str(root) for root in LOG_ROOTS)
        )
    return max(logs, key=lambda path: path.stat().st_mtime)


def summarize(path: Path, manifest_path: Path = DEFAULT_MANIFEST) -> dict:
    events = Counter()
    samples = []
    structured_events = list(read_events(path))
    for event, data in structured_events:
        events[event] += 1
        if event == "sample":
            samples.append(data)
    if not samples:
        raise SystemExit("metrics log has no sample event: %s" % path)
    final = samples[-1]
    completion = next((sample for sample in samples if sample.get("task_done")), None)
    reference = completion or final
    task_done_events = [
        data for event, data in structured_events if event == "task_done"
    ]
    completion_event = task_done_events[-1] if task_done_events else None
    task_completed_events = [
        data for event, data in structured_events if event == "task_completed"
    ]
    task_completed_event = task_completed_events[-1] if task_completed_events else None
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
    scoped_manifest, benchmark_level = scope_manifest_for_run(
        manifest, structured_events
    )
    task_done = bool(completion or final.get("task_done"))
    auxiliary = motion_and_failure_summary(reference, task_done)
    failure_evidence = failure_episode_summary(
        structured_events, load_failure_snapshots(path)
    )
    trial_end_diagnostic = load_trial_end_diagnostic(path)
    failure_log_path = path.parent / (path.parent.name + "_failure_evidence.log")
    return {
        "log": str(path),
        "run_timestamp": path.parent.name,
        "benchmark_level": benchmark_level,
        "task_done": task_done,
        "task_done_seconds": (
            (completion_event or {}).get("ros_time")
            if completion
            else reference.get("task_done_seconds")
        ),
        "task_done_policy": (
            (task_completed_event or {}).get("task_done_policy")
            or reference.get("task_done_policy")
        ),
        "task_done_completion_evidence": (
            (task_completed_event or {}).get("completion_evidence")
            or (completion_event or {}).get("completion_evidence")
            or reference.get("task_done_completion_evidence")
        ),
        "ros_time": reference.get("ros_time"),
        "pose": reference.get("pose"),
        "goal": reference.get("goal"),
        "goal_source": reference.get("goal_source"),
        "path_length_m": reference.get("path_length"),
        "distance_to_goal_m": reference.get("distance_to_goal"),
        "map_coverage": reference.get("map_coverage"),
        "coverage": occupancy_coverage_summary(reference),
        "target_geometric_evaluation": (
            reference.get("target_geometric_evaluation") or {}
        ),
        "target_truth": auxiliary.get("target_truth"),
        "move_base_aborts": reference.get("move_base_aborts"),
        "move_base_preemptions": reference.get("move_base_preemptions"),
        "move_base_unexpected_preemptions": reference.get("move_base_unexpected_preemptions"),
        "turn_only_duration_seconds": reference.get("turn_only_duration_seconds"),
        "turn_only_events": reference.get("turn_only_events"),
        "stop_events": reference.get("stop_events"),
        "min_scan_clearance": reference.get("min_scan_clearance"),
        "goal_changes": reference.get("goal_changes"),
        "target_segments_committed": reference.get("target_segments_committed"),
        "target_route_rejections": reference.get("target_route_rejections"),
        "target_route_deferrals": reference.get("target_route_deferrals"),
        "completion_snapshot": bool(completion),
        "post_completion_samples": len(samples) - (samples.index(completion) + 1) if completion else 0,
        "target_event_counts": {
            name: count
            for name, count in sorted(events.items())
            if name.startswith("target_")
        },
        "event_counts": dict(sorted(events.items())),
        "room_coverage": room_visit_summary(samples, scoped_manifest),
        "work_item_execution": work_item_execution_summary(
            structured_events, reference.get("ros_time")
        ),
        "route_goal_lifecycle": route_goal_lifecycle_summary(structured_events),
        "transition_topology_cache": transition_topology_cache_summary(
            structured_events
        ),
        "frontier_region_lifecycle": frontier_region_summary(structured_events),
        "stall_diagnostics": stall_evidence_summary(structured_events),
        "termination_diagnosis": termination_diagnosis(
            structured_events, reference
        ),
        "failure_evidence": failure_evidence,
        "failure_evidence_log": (
            str(failure_log_path) if failure_log_path.is_file() else None
        ),
        "trial_end_diagnostic": trial_end_diagnostic,
        "trial_end_diagnostic_path": (
            None
            if trial_end_diagnostic is None
            else trial_end_diagnostic.get("artifact_path")
        ),
        "exploration_method": exploration_method_summary(structured_events),
        **auxiliary,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument(
        "--output",
        type=Path,
        help="also write the JSON summary to this path",
    )
    args = parser.parse_args()
    path = args.log or latest_log()
    if not path.is_absolute():
        path = ROOT / path
    if not path.is_file():
        raise SystemExit("metrics log not found: %s" % path)
    manifest = args.manifest if args.manifest.is_absolute() else ROOT / args.manifest
    if not manifest.is_file():
        raise SystemExit("manifest not found: %s" % manifest)
    rendered = json.dumps(
        summarize(path, manifest), ensure_ascii=False, indent=2, sort_keys=True
    )
    print(rendered)
    if args.output:
        output = args.output if args.output.is_absolute() else ROOT / args.output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
