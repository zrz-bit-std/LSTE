#!/usr/bin/env python3
"""Assert the topology-exploration behavior from one benchmark metrics log.

This verifier deliberately measures physical room visits, not internal
frontier-region IDs. One physical room can need several frontier endpoints;
that is healthy coverage. Re-entering a previously departed room or remaining
there without a bounded observation lifecycle is not.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from summarize_run import DEFAULT_MANIFEST, latest_log, read_events, summarize


ROOT = Path(__file__).resolve().parents[3]


def bridge_retry_count(events):
    """Count only the forbidden retry of an unchanged failed route."""
    return sum(
        1
        for event, data in events
        if event == "teb_bridge_event"
        and data.get("bridge_event") == "dispatch"
        and data.get("reason") == "retry_move_base_goal"
    )


def cross_place_endpoint_bypasses(events):
    """Return routes that bypass the explicit doorway action contract.

    Once a source place has a reached observation viewpoint, its graph edge
    must be executed as ``portal_transition``. A regular endpoint with one or
    more certified place hops hides the physical departure inside a remote
    route, which is exactly the architecture that produced excessive room
    dwell and ambiguous place identity.
    """
    bypasses = []
    for event, data in events:
        if (
            event != "global_frontier_event"
            or data.get("frontier_event") != "route_selected"
            or not data.get("source_place_observed")
        ):
            continue
        try:
            place_hops = int(data.get("place_graph_hops") or 0)
        except (TypeError, ValueError):
            continue
        if place_hops < 1 or data.get("route_kind") == "portal_transition":
            continue
        bypasses.append({
            "route_id": data.get("route_id"),
            "route_kind": data.get("route_kind"),
            "place_hops": place_hops,
            "goal": data.get("goal"),
            "ros_time": data.get("ros_time"),
        })
    return bypasses


def portal_self_loops(events):
    """Return durable Portal records that violate Place graph identity."""
    loops = []
    seen = set()
    for event, data in events:
        if event != "global_frontier_event" or not isinstance(data, dict):
            continue
        report = data.get("portal_hypotheses")
        records = report.get("records", ()) if isinstance(report, dict) else ()
        for record in records:
            if not isinstance(record, dict):
                continue
            source = record.get("source_place_id")
            destination = record.get("destination_place_id")
            try:
                source = int(source)
                destination = int(destination)
                portal_id = int(record.get("id"))
            except (TypeError, ValueError):
                continue
            if source <= 0 or destination <= 0 or source != destination:
                continue
            identity = (portal_id, source, destination)
            if identity in seen:
                continue
            seen.add(identity)
            loops.append({
                "portal_id": portal_id,
                "source_place_id": source,
                "destination_place_id": destination,
                "route_id": data.get("route_id"),
                "ros_time": data.get("ros_time"),
            })
    return loops


def topology_evidence(summary, events):
    """Extract stable acceptance facts from a metrics summary and its events."""
    coverage = summary.get("room_coverage") or {}
    dwell_by_room = coverage.get("max_dwell_seconds_by_region") or {}
    rooms = sorted(room for room in dwell_by_room if room != "main_corridor")
    # Corridor dwell is still a physical stall.  It is excluded from the
    # distinct-room denominator, but not from the longest-dwell diagnostic.
    max_dwell = max(
        (float(value) for value in dwell_by_room.values()),
        default=0.0,
    )
    lifecycle = summary.get("frontier_region_lifecycle") or {}
    work_execution = summary.get("work_item_execution") or {}
    failures = summary.get("failure_safety") or {}
    smoothness = summary.get("motion_smoothness") or {}
    stall = summary.get("stall_diagnostics") or {}
    repeated_work = lifecycle.get("repeated_work_item_dispatches") or {}
    collision_events = failures.get("collision_events")
    try:
        collision_events = None if collision_events is None else int(collision_events)
    except (TypeError, ValueError):
        collision_events = None
    return {
        "distinct_rooms": rooms,
        "distinct_room_count": len(rooms),
        "max_room_dwell_seconds": round(max_dwell, 3),
        "physical_room_reentries": int(coverage.get("total_room_reentries") or 0),
        "move_base_aborts": int(summary.get("move_base_aborts") or 0),
        "bridge_retry_move_base_goals": bridge_retry_count(events),
        "cross_place_endpoint_bypasses": cross_place_endpoint_bypasses(events),
        "portal_self_loops": portal_self_loops(events),
        "resolved_work_item_descendant_dispatches": lifecycle.get(
            "resolved_work_item_descendant_dispatches", []
        ),
        "repeated_work_item_dispatches": repeated_work,
        "repeated_work_item_dispatch_count": sum(
            int(value) for value in repeated_work.values()
        ),
        "duplicate_work_item_dispatches": int(
            work_execution.get("duplicate_dispatch_count") or 0
        ),
        "unmatched_work_item_settlements": int(
            work_execution.get("unmatched_settlement_count") or 0
        ),
        "collision_events": collision_events,
        "collision_truth_status": failures.get("collision_truth_status"),
        "route_stagnant_observed_count": int(
            stall.get("route_stagnant_observed_count") or 0
        ),
        "frontier_route_unavailable_event_count": int(
            stall.get("frontier_route_unavailable_event_count") or 0
        ),
        "max_frontier_route_unavailable_span_seconds": stall.get(
            "max_frontier_route_unavailable_span_seconds"
        ),
        "max_stop_duration_seconds": smoothness.get("max_stop_duration_seconds"),
        "unexplained_clear_path_brake_events": smoothness.get(
            "unexplained_clear_path_brake_events"
        ),
        "straight_path_steering_sign_flips": smoothness.get(
            "straight_path_steering_sign_flips"
        ),
    }


def verify(
    summary,
    events,
    min_distinct_rooms,
    max_room_dwell_seconds,
    max_room_reentries,
    max_move_base_aborts,
    max_bridge_retries,
    max_repeated_work_item_dispatches=0,
    max_collision_events=0,
    max_route_stagnant_events=0,
    max_route_unavailable_span_seconds=60.0,
    max_stop_duration_seconds=60.0,
    max_unexplained_clear_path_brakes=None,
    max_steering_sign_flips=None,
):
    """Return an inspectable pass/fail result without changing runtime state."""
    evidence = topology_evidence(summary, events)
    violations = []
    if evidence["distinct_room_count"] < min_distinct_rooms:
        violations.append(
            "visited %d distinct rooms, require at least %d"
            % (evidence["distinct_room_count"], min_distinct_rooms)
        )
    if evidence["physical_room_reentries"] > max_room_reentries:
        violations.append(
            "physical room re-entries=%d, limit=%d"
            % (evidence["physical_room_reentries"], max_room_reentries)
        )
    if evidence["max_room_dwell_seconds"] > max_room_dwell_seconds:
        violations.append(
            "max physical-room dwell=%.3fs, limit=%.3fs"
            % (evidence["max_room_dwell_seconds"], max_room_dwell_seconds)
        )
    if evidence["move_base_aborts"] > max_move_base_aborts:
        violations.append(
            "move_base aborts=%d, limit=%d"
            % (evidence["move_base_aborts"], max_move_base_aborts)
        )
    if evidence["bridge_retry_move_base_goals"] > max_bridge_retries:
        violations.append(
            "bridge same-route retries=%d, limit=%d"
            % (evidence["bridge_retry_move_base_goals"], max_bridge_retries)
        )
    if (
        evidence["repeated_work_item_dispatch_count"]
        > max_repeated_work_item_dispatches
    ):
        violations.append(
            "resolved ObservationWorkItem re-dispatches=%d, limit=%d"
            % (
                evidence["repeated_work_item_dispatch_count"],
                max_repeated_work_item_dispatches,
            )
        )
    if evidence["duplicate_work_item_dispatches"] > 0:
        violations.append(
            "duplicate active WorkItem dispatches=%d"
            % evidence["duplicate_work_item_dispatches"]
        )
    if (
        evidence["collision_events"] is not None
        and evidence["collision_events"] > max_collision_events
    ):
        violations.append(
            "collision events=%d, limit=%d"
            % (evidence["collision_events"], max_collision_events)
        )
    if evidence["route_stagnant_observed_count"] > max_route_stagnant_events:
        violations.append(
            "route stagnation observations=%d, limit=%d"
            % (
                evidence["route_stagnant_observed_count"],
                max_route_stagnant_events,
            )
        )
    unavailable_span = evidence["max_frontier_route_unavailable_span_seconds"]
    if (
        max_route_unavailable_span_seconds is not None
        and unavailable_span is not None
        and float(unavailable_span) > max_route_unavailable_span_seconds
    ):
        violations.append(
            "frontier route unavailable span=%.3fs, limit=%.3fs"
            % (float(unavailable_span), max_route_unavailable_span_seconds)
        )
    stop_duration = evidence["max_stop_duration_seconds"]
    if (
        max_stop_duration_seconds is not None
        and stop_duration is not None
        and float(stop_duration) > max_stop_duration_seconds
    ):
        violations.append(
            "max zero-velocity stop=%.3fs, limit=%.3fs"
            % (float(stop_duration), max_stop_duration_seconds)
        )
    clear_path_brakes = evidence["unexplained_clear_path_brake_events"]
    if (
        max_unexplained_clear_path_brakes is not None
        and clear_path_brakes is not None
        and float(clear_path_brakes) > max_unexplained_clear_path_brakes
    ):
        violations.append(
            "unexplained clear-path brakes=%d, limit=%d"
            % (clear_path_brakes, max_unexplained_clear_path_brakes)
        )
    steering_flips = evidence["straight_path_steering_sign_flips"]
    if (
        max_steering_sign_flips is not None
        and steering_flips is not None
        and float(steering_flips) > max_steering_sign_flips
    ):
        violations.append(
            "straight-path steering sign flips=%d, limit=%d"
            % (steering_flips, max_steering_sign_flips)
        )
    bypasses = evidence["cross_place_endpoint_bypasses"]
    if bypasses:
        violations.append(
            "observed-place cross-room endpoint bypasses=%d; explicit portal required"
            % len(bypasses)
        )
    self_loops = evidence["portal_self_loops"]
    if self_loops:
        violations.append(
            "Portal self-loops=%d; source and destination Place must differ"
            % len(self_loops)
        )
    descendants = evidence["resolved_work_item_descendant_dispatches"]
    if descendants:
        violations.append(
            "resolved ObservationWorkItem descendants dispatched=%d"
            % len(descendants)
        )
    return {
        "passed": not violations,
        "criteria": {
            "min_distinct_rooms": int(min_distinct_rooms),
            "max_room_dwell_seconds": float(max_room_dwell_seconds),
            "max_room_reentries": int(max_room_reentries),
            "max_move_base_aborts": int(max_move_base_aborts),
            "max_bridge_retries": int(max_bridge_retries),
            "max_repeated_work_item_dispatches": int(
                max_repeated_work_item_dispatches
            ),
            "max_collision_events": int(max_collision_events),
            "max_route_stagnant_events": int(max_route_stagnant_events),
            "max_route_unavailable_span_seconds": (
                None if max_route_unavailable_span_seconds is None
                else float(max_route_unavailable_span_seconds)
            ),
            "max_stop_duration_seconds": (
                None if max_stop_duration_seconds is None
                else float(max_stop_duration_seconds)
            ),
            "max_unexplained_clear_path_brakes": max_unexplained_clear_path_brakes,
            "max_steering_sign_flips": max_steering_sign_flips,
            "max_resolved_work_item_descendant_dispatches": 0,
        },
        "evidence": evidence,
        "violations": violations,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--min-distinct-rooms", type=int, default=3)
    parser.add_argument("--max-room-dwell-seconds", type=float, default=120.0)
    parser.add_argument("--max-room-reentries", type=int, default=0)
    parser.add_argument("--max-move-base-aborts", type=int, default=0)
    parser.add_argument("--max-bridge-retries", type=int, default=0)
    parser.add_argument("--max-repeated-work-item-dispatches", type=int, default=0)
    parser.add_argument("--max-collision-events", type=int, default=0)
    parser.add_argument("--max-route-stagnant-events", type=int, default=0)
    parser.add_argument(
        "--max-route-unavailable-span-seconds", type=float, default=60.0
    )
    parser.add_argument("--max-stop-duration-seconds", type=float, default=60.0)
    parser.add_argument("--max-unexplained-clear-path-brakes", type=int)
    parser.add_argument("--max-steering-sign-flips", type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    path = args.log or latest_log()
    if not path.is_absolute():
        path = ROOT / path
    manifest = args.manifest if args.manifest.is_absolute() else ROOT / args.manifest
    if not path.is_file():
        raise SystemExit("metrics log not found: %s" % path)
    if not manifest.is_file():
        raise SystemExit("manifest not found: %s" % manifest)

    events = list(read_events(path))
    result = verify(
        summarize(path, manifest),
        events,
        args.min_distinct_rooms,
        args.max_room_dwell_seconds,
        args.max_room_reentries,
        args.max_move_base_aborts,
        args.max_bridge_retries,
        args.max_repeated_work_item_dispatches,
        args.max_collision_events,
        args.max_route_stagnant_events,
        args.max_route_unavailable_span_seconds,
        args.max_stop_duration_seconds,
        args.max_unexplained_clear_path_brakes,
        args.max_steering_sign_flips,
    )
    result["log"] = str(path)
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        output = args.output if args.output.is_absolute() else ROOT / args.output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
