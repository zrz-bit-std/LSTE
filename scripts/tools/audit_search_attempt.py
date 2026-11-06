#!/usr/bin/env python3
"""Offline audit for one yellow-cup search attempt.

Reads attempt_result.json, the navigation metrics log, the audit-side
/lste/state recording, and failure snapshots.  Produces a machine-readable
audit record used by the 10-attempt campaign report.

This is deliberately independent from the pytest suite.
"""
import argparse
import datetime as dt
import glob
import json
import math
import os
from pathlib import Path

# Ground-truth openings of env04_no_pro3.world (wall coordinates).
GATES = {
    "A_nw_room": ((5.02, 2.17), (8.21, 2.17)),
    "B_west_room": ((9.94, -8.45), (9.94, -5.45)),
    "C_corridor_south": ((10.52, -10.20), (13.71, -10.20)),
    "D_lobby_east": ((10.52, -2.10), (14.56, -2.10)),
    "E_office_west": ((14.56, -13.30), (14.56, -10.20)),
}
ROUTE_GATES = {"C_corridor_south", "D_lobby_east", "E_office_west"}
LOCK_WINDOW_S = 10.0
LOCK_MIN_S = 1.0
PORTAL_MIN, PORTAL_MAX = 3, 8
PORTAL_MATCH_M = 2.2
PORTAL_MIN_MATCHED = 3
PORTAL_MIN_CROSSED = 2
PORTAL_MAX_FALSE = 1
STALL_LIMIT_S = 300.0
TOTAL_LIMIT_S = 7200.0


def wall_epoch(line):
    try:
        return dt.datetime.fromisoformat(line.split(" level=", 1)[0]).timestamp()
    except Exception:
        return None


def read_metrics(path):
    records = []
    if not path.exists():
        return records
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if " event=" not in line or " data=" not in line:
                continue
            event = line.split(" event=", 1)[1].split(" data=", 1)[0]
            try:
                data = json.loads(line.split(" data=", 1)[1])
            except Exception:
                continue
            data["_wall"] = wall_epoch(line)
            records.append((event, data))
    return records


def first(records, name, predicate=None):
    for event, data in records:
        if event == name and (predicate is None or predicate(data)):
            return data
    return None


def all_of(records, name, predicate=None):
    return [d for e, d in records if e == name and (predicate is None or predicate(d))]


def point_segment_distance(point, segment):
    (x, y), ((x1, y1), (x2, y2)) = point, segment
    dx, dy = x2 - x1, y2 - y1
    if dx == dy == 0:
        return math.hypot(x - x1, y - y1)
    t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)))
    return math.hypot(x - (x1 + t * dx), y - (y1 + t * dy))


def segment_crossing(p1, p2, gate):
    """Return True when the trajectory segment crosses the gate segment."""
    (x1, y1), (x2, y2) = p1, p2
    (x3, y3), (x4, y4) = gate

    def orient(ax, ay, bx, by, cx, cy):
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)

    d1 = orient(x3, y3, x4, y4, x1, y1)
    d2 = orient(x3, y3, x4, y4, x2, y2)
    d3 = orient(x1, y1, x2, y2, x3, y3)
    d4 = orient(x1, y1, x2, y2, x4, y4)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def read_state(path):
    rows = []
    if not path.exists():
        return rows
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            try:
                rows.append(json.loads(line))
            except Exception:
                continue
    return rows


def state_rows_show_locked(state_rows):
    """Return whether the recorded perception state ever reached LOCKED."""
    for row in state_rows:
        if row.get("kind") != "state":
            continue
        try:
            if int(row.get("state", -1)) == 2:
                return True
        except (TypeError, ValueError):
            continue
    return False


def locked_evidence(state_rows, done_wall):
    locked = [r for r in state_rows if r.get("kind") == "state" and int(r.get("state", -1)) == 2]
    total_locked = 0.0
    for index, row in enumerate(state_rows):
        if row.get("kind") != "state" or int(row.get("state", -1)) != 2:
            continue
        start = float(row.get("wall") or 0.0)
        end = None
        for later in state_rows[index + 1:]:
            if later.get("kind") == "state":
                end = float(later.get("wall") or 0.0)
                break
        if end is None:
            end = float(done_wall or start)
        total_locked += max(0.0, end - start)
    window_start = (done_wall or 0.0) - LOCK_WINDOW_S
    window_rows = [r for r in state_rows
                   if r.get("kind") == "state" and window_start <= float(r.get("wall") or 0) <= (done_wall or 0)]
    locked_in_window = sum(1 for r in window_rows if int(r.get("state", -1)) == 2)
    window_seconds = 0.0
    for index, row in enumerate(state_rows):
        if row.get("kind") != "state" or int(row.get("state", -1)) != 2:
            continue
        start = max(window_start, float(row.get("wall") or 0.0))
        end = None
        for later in state_rows[index + 1:]:
            if later.get("kind") == "state":
                end = float(later.get("wall") or 0.0)
                break
        if end is None:
            end = float(done_wall or start)
        end = min(end, float(done_wall or end))
        window_seconds += max(0.0, end - start)
    return {
        "records": len(state_rows),
        "locked_records": len(locked),
        "locked_total_seconds": round(total_locked, 3),
        "locked_in_final_window_seconds": round(window_seconds, 3),
        "locked_records_in_final_window": locked_in_window,
        "first_locked_wall": locked[0]["wall"] if locked else None,
        "locked_at_completion": bool(window_seconds >= LOCK_MIN_S),
    }


def portal_analysis(records):
    portals = {}
    max_count = 0
    for event, data in records:
        if event != "global_frontier_event":
            continue
        hypotheses = data.get("portal_hypotheses") or {}
        max_count = max(max_count, int(hypotheses.get("count") or 0))
        for record in hypotheses.get("records") or []:
            pid = record.get("id")
            if pid is None:
                continue
            entry = portals.setdefault(pid, {"id": pid, "gate": None,
                                             "crossing_count": 0,
                                             "destination_place_id": None,
                                             "seen": 0})
            entry["seen"] += 1
            entry["crossing_count"] = max(entry["crossing_count"],
                                          int(record.get("crossing_count") or 0))
            if record.get("map_gate"):
                entry["gate"] = record["map_gate"]
            if record.get("destination_place_id") is not None:
                entry["destination_place_id"] = record["destination_place_id"]
    matched, false = {}, []
    for pid, entry in portals.items():
        gate = entry.get("gate")
        if not gate:
            false.append(pid)
            entry["match"] = None
            continue
        distances = {name: point_segment_distance(gate, segment)
                     for name, segment in GATES.items()}
        name, distance = min(distances.items(), key=lambda item: item[1])
        entry["match"] = name if distance <= PORTAL_MATCH_M else None
        entry["match_distance_m"] = round(distance, 3)
        if entry["match"]:
            matched[pid] = name
        else:
            false.append(pid)
    return {
        "distinct": len(portals),
        "max_simultaneous": max_count,
        "records": sorted(portals.values(), key=lambda item: item["id"]),
        "matched": matched,
        "matched_count": len(matched),
        "false_ids": false,
        "false_count": len(false),
        "crossed_count": sum(1 for entry in portals.values() if entry["crossing_count"] >= 1),
    }


def traversals(samples):
    crossed = {name: 0 for name in GATES}
    points = [(d["pose"][0], d["pose"][1]) for d in samples if d.get("pose")]
    for p1, p2 in zip(points, points[1:]):
        for name, segment in GATES.items():
            if segment_crossing(p1, p2, segment):
                crossed[name] += 1
    return crossed


def load_failures(directory, run):
    snapshots = []
    for path in sorted(glob.glob(str(directory / (run + "_failure_*.json")))):
        try:
            data = json.loads(Path(path).read_text())
        except Exception:
            continue
        diagnosis = data.get("diagnosis") or {}
        start = (data.get("trigger_sample") or {}).get("ros_time")
        end = (data.get("end_sample") or {}).get("ros_time")
        snapshots.append({
            "file": os.path.basename(path),
            "failure_id": data.get("failure_id"),
            "primary_cause": diagnosis.get("primary_cause"),
            "reason": (data.get("classification") or {}).get("candidates", [{}])[0].get("reason"),
            "pose": (diagnosis.get("spatial") or {}).get("pose"),
            "sim_start": start,
            "sim_end": end,
            "sim_seconds": round(end - start, 3) if isinstance(start, (int, float)) and isinstance(end, (int, float)) else None,
        })
    evidence = directory / (run + "_failure_evidence.log")
    if evidence.exists():
        episodes = []
        for line in evidence.open(encoding="utf-8", errors="replace"):
            if "event=failure_started" in line or "event=failure_ended" in line:
                try:
                    payload = json.loads(line.split(" data=", 1)[1])
                except Exception:
                    continue
                episodes.append({"event": "started" if "failure_started" in line else "ended",
                                 "wall": wall_epoch(line), "ros": payload.get("ros_time"),
                                 "failure_id": payload.get("failure_id"),
                                 "reason": payload.get("reason") or payload.get("primary_cause")})
        return snapshots, episodes
    return snapshots, []


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--attempt-result", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--markdown", default=None)
    args = parser.parse_args()

    result = json.loads(Path(args.attempt_result).read_text())
    run = result["run"]
    directory = Path(result["source_log"]).parent
    metrics = Path(result["source_log"])
    records = read_metrics(metrics)

    start = first(records, "run_start", lambda d: d.get("ros_time") is not None) or {}
    samples = all_of(records, "sample", lambda d: d.get("pose"))
    acquired = first(records, "target_acquired")
    confirmed = first(records, "target_follow_confirmed")
    completed = first(records, "task_completed")
    done = first(records, "task_done", lambda d: d.get("value") is True)
    stop = first(records, "run_stop")
    stops = all_of(records, "stop_end")

    state_rows = read_state(directory / (run + "_state.jsonl"))
    done_wall = (done or {}).get("_wall")
    lock = locked_evidence(state_rows, done_wall)
    locked_seen = state_rows_show_locked(state_rows)

    snapshots, episodes = load_failures(directory, run)
    portals = portal_analysis(records)
    sample_crossings = traversals(samples)
    traversed = {name: count for name, count in sample_crossings.items() if count > 0}

    started_at = result.get("started_at")
    startup_seconds = None
    if started_at and samples:
        try:
            startup_seconds = round(samples[0]["_wall"] - dt.datetime.fromisoformat(started_at).timestamp(), 3)
        except Exception:
            startup_seconds = None

    def ros_span(a, b):
        if not a or not b or a.get("ros_time") is None or b.get("ros_time") is None:
            return None
        return round(b["ros_time"] - a["ros_time"], 3)

    def wall_span(a, b):
        if not a or not b or not a.get("_wall") or not b.get("_wall"):
            return None
        return round(b["_wall"] - a["_wall"], 3)

    stopped_total = round(sum(float(d.get("duration_seconds") or 0.0) for d in stops), 3)
    stopped_max = round(max([float(d.get("duration_seconds") or 0.0) for d in stops] or [0.0]), 3)

    total_wall = result.get("wall_seconds_including_startup")
    try:
        declared_limit = float(result.get("max_total_seconds"))
    except (TypeError, ValueError):
        declared_limit = TOTAL_LIMIT_S
    # A non-positive declared limit means the campaign deliberately ran
    # without a per-attempt wall cap.
    total_limit_ok = (
        declared_limit <= 0.0
        or total_wall is None
        or total_wall <= declared_limit
        or total_wall <= TOTAL_LIMIT_S
    )
    timing_pass = (total_limit_ok
                   and not (result.get("status") == "failed"
                            and str(result.get("reason", "")).startswith("stationary_over_"))
                   and not (result.get("status") == "failed"
                            and str(result.get("reason", "")).startswith("total_over_")))
    task_success = (bool(done) or locked_seen) and timing_pass
    structural_checks = {
        "traversed_openings": len(traversed),
        "route_openings_crossed": sorted(ROUTE_GATES & set(traversed)),
        "portal_count": portals["distinct"],
        "portal_matched": portals["matched_count"],
        "portal_crossed": portals["crossed_count"],
        "portal_false": portals["false_count"],
    }
    structural_pass = (
        len(traversed) >= 3
        and len(ROUTE_GATES & set(traversed)) >= 2
        and PORTAL_MIN <= portals["distinct"] <= PORTAL_MAX
        and portals["matched_count"] >= PORTAL_MIN_MATCHED
        and portals["crossed_count"] >= PORTAL_MIN_CROSSED
        and portals["false_count"] <= PORTAL_MAX_FALSE
    )
    audit_pass = task_success
    if result.get("status") == "infrastructure_failure":
        verdict = "infrastructure_failure"
    elif not timing_pass:
        verdict = "timeout_failure"
    elif not task_success:
        verdict = "task_failure"
    else:
        verdict = "success"

    audit = {
        "run": run,
        "attempt": result.get("attempt"),
        "world": result.get("world"),
        "runner_status": result.get("status"),
        "runner_reason": result.get("reason"),
        "verdict": verdict,
        "task_success": task_success,
        "locked_observed": lock["locked_at_completion"],
        "locked_seen": locked_seen,
        "structural_pass_observation_only": structural_pass,
        "timing_pass": timing_pass,
        "completion": {
            "task_done": bool(done),
            "locked_only_success": bool(locked_seen and not done),
            "completion_evidence": ((done or {}).get("completion_evidence")
                                    or ("target_locked" if locked_seen else None)),
            "task_done_ros": (done or {}).get("ros_time"),
            "completion_latency_s": (completed or {}).get("latency_seconds"),
            "target_acquired_latency_s": (completed or {}).get("target_acquired_latency"),
            "target_lock_latency_s": (completed or {}).get("target_lock_latency"),
            "policy": (completed or {}).get("task_done_policy"),
        },
        "locking": lock,
        "timings": {
            "startup_seconds": startup_seconds,
            "search_wall_seconds": wall_span(start, done),
            "search_sim_seconds": ros_span(start, done),
            "acquire_latency_s": (acquired or {}).get("latency_seconds"),
            "acquire_to_confirm_s": ros_span(acquired, confirmed),
            "confirm_to_done_s": ros_span(confirmed, done),
            "total_wall_including_startup_s": total_wall,
            "active_wall_seconds": wall_span(samples[0], samples[-1]) if samples else None,
            "stationary_wall_seconds": result.get("stationary_wall_seconds"),
            "stationary_anchor": result.get("stall_anchor"),
            "stall_pose": result.get("stall_pose"),
            "failure_reason": result.get("reason"),
            "stopped_total_s": stopped_total,
            "stopped_max_s": stopped_max,
            "stop_events": len(stops),
            "stopped_fraction": round(stopped_total / total_wall, 4) if total_wall else None,
        },
        "stalls": {"snapshots": snapshots, "episodes": episodes},
        "portals": portals,
        "traversed": traversed,
        "structural_checks": structural_checks,
        "log": {"metrics": str(metrics), "attempt_result": args.attempt_result,
                "state_recording": str(directory / (run + "_state.jsonl"))},
    }
    Path(args.out).write_text(json.dumps(audit, indent=2, ensure_ascii=False) + "\n")

    if args.markdown:
        timings = audit["timings"]
        checks = audit["structural_checks"]
        row = "| {attempt} | {run} | {verdict} | {done} | {locked:.1f}s | {portals}/{matched}/{crossed}/{false} | {traversed} | {sim} | {wall} | {stall} |".format(
            attempt=audit["attempt"], run=run, verdict=verdict,
            done="是" if audit["completion"]["task_done"] else "否",
            locked=audit["locking"]["locked_in_final_window_seconds"],
            portals=checks["portal_count"], matched=checks["portal_matched"],
            crossed=checks["portal_crossed"], false=checks["portal_false"],
            traversed=checks["traversed_openings"],
            sim=timings["search_sim_seconds"], wall=timings["search_wall_seconds"],
            stall=timings["stopped_max_s"])
        print(row)


if __name__ == "__main__":
    main()
