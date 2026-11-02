#!/usr/bin/env python3
"""Audit that each graph WI dispatch follows its same-Place TSP tour."""
import argparse
import json
from pathlib import Path


def events(path):
    with Path(path).open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            marker = "event=global_frontier_event data="
            if marker not in line:
                continue
            try:
                yield json.loads(line.split(" data=", 1)[1])
            except (ValueError, TypeError):
                continue

def audit(path):
    plans = []
    multi_item_tours = 0
    selected_ids_by_place = {}
    dispatched_ids_by_place = {}
    dispatches = []
    dispatch_matches = True
    dispatch_place_matches = True
    multi_item_dispatches = 0
    first_matches = True
    place_membership = True
    for event in events(path):
        plan = event.get("graph_route_plan")
        if not isinstance(plan, dict):
            continue
        if plan.get("action") not in {"bootstrap_observation", "observe_local_work", "retry_viewpoint"}:
            continue
        if plan.get("obligation_kind") != "work_item":
            continue
        tour = plan.get("work_item_tour") or []
        try:
            tour = [int(value) for value in tour]
        except (TypeError, ValueError):
            first_matches = False
            continue
        if not tour:
            # A hidden durable WorkItem can be retained as an obligation while
            # no current-map viewpoint is executable; it is not a TSP dispatch.
            continue
        try:
            item_id = int(plan.get("obligation_id"))
            place_id = int(plan.get("current_place_id"))
        except (TypeError, ValueError):
            first_matches = False
            continue
        first_matches = first_matches and tour[0] == item_id
        if len(tour) > 1:
            multi_item_tours += 1
        work_items = event.get("work_items") or []
        item_places = {}
        for item in work_items:
            try:
                item_places[int(item.get("id"))] = int(item.get("place_id"))
            except (AttributeError, TypeError, ValueError):
                continue
        place_membership = place_membership and all(
            item_places.get(candidate_id) == place_id for candidate_id in tour
        )
        if event.get("frontier_event") == "work_item_dispatched":
            context = event.get("goal_context") or {}
            try:
                dispatched_id = int(event.get("work_item_id"))
                context_id = int(context.get("work_item_id"))
                owner_id = int(context.get("owner_place_id"))
            except (TypeError, ValueError):
                dispatched_id, context_id, owner_id = None, None, None
            dispatch_matches = dispatch_matches and dispatched_id == context_id == tour[0]
            dispatch_place_matches = dispatch_place_matches and owner_id == place_id
            dispatched_ids_by_place.setdefault(str(place_id), []).append(dispatched_id)
            multi_item_dispatches += int(len(tour) > 1)
            dispatches.append({
                "route_id": event.get("route_id"),
                "place_id": place_id,
                "work_item_id": dispatched_id,
                "work_item_tour": tour,
                "reason": plan.get("reason"),
                "ros_time": event.get("ros_time"),
            })
        selected_ids_by_place.setdefault(str(place_id), []).append(item_id)
        plans.append({
            "place_id": place_id,
            "selected_work_item_id": item_id,
            "work_item_tour": tour,
            "reason": plan.get("reason"),
        })
    distinct_progress = any(
        len(set(item_ids) - {None}) >= 2 for item_ids in dispatched_ids_by_place.values()
    )
    checks = {
        "work_item_plans_recorded": bool(plans),
        "actual_work_item_dispatches_recorded": bool(dispatches),
        "dispatched_item_is_tour_head": dispatch_matches,
        "dispatch_owner_matches_planned_place": dispatch_place_matches,
        "multi_item_tour_actually_dispatched": multi_item_dispatches > 0,
        "selected_item_is_tour_head": first_matches,
        "tour_items_belong_to_planned_place": place_membership,
        "multi_item_tsp_used": multi_item_tours > 0,
        "multiple_items_dispatched_in_a_place": distinct_progress,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "plan_count": len(plans),
        "dispatch_count": len(dispatches),
        "multi_item_dispatch_count": multi_item_dispatches,
        "dispatches": dispatches,
        "dispatched_work_item_ids_by_place": dispatched_ids_by_place,
        "multi_item_tour_count": multi_item_tours,
        "selected_work_item_ids_by_place": selected_ids_by_place,
        "sample_plans": plans[:20],
        "metrics_log": str(path),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("metrics")
    parser.add_argument("--output")
    args = parser.parse_args()
    result = audit(args.metrics)
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
