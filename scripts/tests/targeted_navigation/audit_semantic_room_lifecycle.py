#!/usr/bin/env python3
"""Audit semantic Place closure, portal suppression, and parent-WI resumption."""
import argparse
import json
import math
from pathlib import Path


def records(path):
    previous={}
    with Path(path).open(encoding="utf-8", errors="strict") as stream:
        for line in stream:
            if "event=global_frontier_event data=" not in line:continue
            data=json.loads(line.split(" data=",1)[1])
            row={k:data[k] for k in ("frontier_event","ros_time","work_item_id") if k in data}
            tree=data.get("tree") or {}
            row["tree_recorded"]="tree" in data
            row["tree"]={"nodes":[{k:n[k] for k in ("id","junction_confirmed","physical_xy") if k in n} for n in tree.get("nodes",[])],
                         "branches":[{k:b[k] for k in ("id","node_id","heading","connection","canonical_portal_id","authentication_state","state",
                              "return_source_place_id","return_destination_place_id","return_portal_evidence") if k in b} for b in tree.get("branches",[])]}
            portal_snapshot=data.get("portal_hypotheses") or {}
            if "records" in portal_snapshot:
                row["portal_hypotheses"]={"records":[{k:p[k] for k in
                    ("id","state","source_place_id","destination_place_id","crossing_count",
                     "gp_authentication_state","physical_gate","physical_destination") if k in p}
                    for p in portal_snapshot.get("records",[])]}
            auth=data.get("portal_authentication") or {}
            row["portal_authentication"]={k:auth[k] for k in ("node_id","phase","all_finished") if k in auth}
            row["work_items"]=[{**{k:w[k] for k in ("id","place_id","state") if k in w},
                                "attempts":[{k:a[k] for k in ("id","state","reason") if k in a} for a in w.get("attempts",[])]}
                               for w in data.get("work_items",[])]
            for key in ("observation_tree","transition","active_semantic_room","active_semantic_rooms","goal_context"):
                if key in data:row[key]=data[key]
            row["suppression_evidence"]=[{k:item[k] for k in ("reentry_suppressed","portal_id","branch_id") if k in item}
                                         for item in walk(data) if item.get("reentry_suppressed") is True]
            # Reuse unchanged audit projections instead of keeping thousands
            # of duplicate histories, while retaining every chronological row.
            for key,value in list(row.items()):
                if isinstance(value,(dict,list)):
                    if key in previous and value==previous[key]:row[key]=previous[key]
                    else:previous[key]=value
            yield row

def walk(value):
    if isinstance(value, dict):
        yield value
        for child in value.values():
            yield from walk(child)
    elif isinstance(value, list):
        for child in value:
            yield from walk(child)


def unknown_semantics_certified(room, certified_ids):
    """An explicitly recorded unknown score ranks last, with physical proof."""
    return (room.get("branch_id") in certified_ids
            and room.get("semantic_evidence")=="unavailable"
            and room.get("state")=="observed"
            and room.get("invalidated") is False
            and not room.get("exploration_suppressed")
            and not room.get("labels")
            and int(room.get("stable_frame_count") or 0)==0
            and float(room.get("semantic_score",0))<0)


def verified_parent_directions(snapshots, junction_id, rooms):
    """A proven visited parent covers one arm, never an unexplored child room."""
    from check_gp_tree import verified_return_direction
    branches=[];portals={}
    for snapshot in snapshots:
        if snapshot.get('tree_recorded','tree' in snapshot):
            branches=(snapshot.get('tree') or {}).get('branches',[])
        records=(snapshot.get('portal_hypotheses') or {}).get('records')
        if records is not None:portals={p['id']:p for p in records}
    room_ids={room.get('branch_id') for room in rooms}
    counted_portals={b.get('canonical_portal_id') for b in branches if b.get('id') in room_ids}
    proven={}
    for branch in branches:
        portal_id=branch.get('canonical_portal_id')
        if (branch.get('node_id')!=junction_id or branch.get('id') in room_ids
                or portal_id in counted_portals or not verified_return_direction(branch,portals)):
            continue
        proven[portal_id]=dict(branch_id=branch['id'],portal_id=portal_id,
            parent_place_id=portals[portal_id]['source_place_id'],heading=branch['heading'],
            evidence=branch['return_portal_evidence'])
    return [proven[key] for key in sorted(proven)]


def audit(path, center, radius, expected_rooms):
    snapshots = list(records(path))
    latest = snapshots[-1] if snapshots else {}
    junction_id = None
    closest = float("inf")
    for snapshot in snapshots:
        tree = snapshot.get("tree") or {}
        for node in tree.get("nodes", []):
            if not node.get("junction_confirmed"):
                continue
            xy = node.get("physical_xy") or [float("inf"), float("inf")]
            gap = math.dist(xy, center)
            if gap <= radius and gap < closest:
                closest, junction_id = gap, node.get("id")

    tree_rooms = []
    if junction_id is not None:
        observation_tree = latest.get("observation_tree") or {}
        node_tree = observation_tree.get(str(junction_id), {})
        tree_rooms = list(node_tree.get("rooms", []))
    # The observation tree may be retired after returning to the parent. Keep
    # the last complete snapshot for this node in that case.
    if junction_id is not None and not tree_rooms:
        for snapshot in reversed(snapshots):
            node_tree = (snapshot.get("observation_tree") or {}).get(str(junction_id))
            if node_tree and node_tree.get("rooms"):
                tree_rooms = list(node_tree["rooms"])
                break

    closed = {}
    suppressed = {}
    for snapshot in snapshots:
        transition = snapshot.get("transition")
        if isinstance(transition, dict) and transition.get("event") == "semantic_room_closed":
            portal_id = transition.get("portal_id")
            if portal_id is not None:
                active = next((room for room in snapshot.get("active_semantic_rooms", [])
                               if room.get("portal_id") == portal_id), {})
                place_id = active.get("place_id")
                place_items = [item for item in snapshot.get("work_items", [])
                               if place_id is not None and item.get("place_id") == place_id]
                closed[int(portal_id)] = {
                    "portal_id": int(portal_id),
                    "branch_id": active.get("branch_id"),
                    "place_id": place_id,
                    "room_work_items": [
                        {"id": item.get("id"), "state": item.get("state")}
                        for item in place_items
                    ],
                    "room_items_settled": all(item.get("state") == "resolved"
                                              for item in place_items),
                }
        for item in walk(snapshot):
            if item.get("reentry_suppressed") is True and item.get("portal_id") is not None:
                suppressed[int(item["portal_id"])] = item.get("branch_id")

    parent_item = None
    if snapshots:
        for snapshot in snapshots:
            authentication = snapshot.get("portal_authentication") or {}
            try:
                snapshot_node_id = int(authentication.get("node_id"))
            except (TypeError, ValueError):
                snapshot_node_id = None
            # Only associate a suspended WI with the junction under audit.
            # Later junctions in the same world can suspend their own parent.
            if snapshot_node_id != int(junction_id or -1):
                continue
            if snapshot.get("frontier_event") == "work_item_resume_target_selected":
                try:
                    parent_item = int(snapshot.get("work_item_id"))
                except (TypeError, ValueError):
                    parent_item = None
            if parent_item is None and snapshot.get("frontier_event") == "work_item_suspended":
                # Bind the parent obligation from the actual suspension event
                # during this node's authentication. Suspension reasons can
                # describe a simultaneous scope revision, so do not require a
                # single reason string; require the event's WI identity and a
                # matching suspended attempt in that same snapshot.
                try:
                    event_item_id = int(snapshot.get("work_item_id"))
                except (TypeError, ValueError):
                    event_item_id = None
                if event_item_id is not None and any(
                        int(item.get("id", -1)) == event_item_id
                        and any(attempt.get("state") == "suspended"
                                for attempt in item.get("attempts", []))
                        for item in snapshot.get("work_items", [])):
                    parent_item = event_item_id
            if parent_item is not None:
                break
            for item in snapshot.get("work_items", []):
                if any(attempt.get("state") == "suspended"
                       and attempt.get("reason") == "portal_junction_observation_period"
                       for attempt in item.get("attempts", [])):
                    parent_item = int(item.get("id"))
                    break
            if parent_item is not None:
                break
    parent_latest = None
    if parent_item is not None:
        for snapshot in snapshots:
            for item in snapshot.get("work_items", []):
                try:
                    same_item = int(item.get("id", -1)) == parent_item
                except (TypeError, ValueError):
                    same_item = False
                if same_item:
                    parent_latest = item
    parent_resumed = bool(parent_latest and parent_latest.get("state") == "resolved"
                          and any(attempt.get("state") == "succeeded"
                                  for attempt in parent_latest.get("attempts", [])))

    watch_started = False
    observation_complete = False
    work_before_watch = False
    frozen_work_preserved = True
    places_deferred = True
    room_entries_after_watch = True
    room_local_work_only = True
    known_room_places = set()
    for snapshot in snapshots:
        event = snapshot.get("frontier_event")
        auth = snapshot.get("portal_authentication") or {}
        is_root_auth = auth.get("node_id") == junction_id
        observing = (is_root_auth and auth.get("phase") in ("watching", "authenticating")
                     and not auth.get("all_finished"))
        node_tree = (snapshot.get("observation_tree") or {}).get(str(junction_id), {})
        current_room_places = {room["place_id"] for room in node_tree.get("rooms", [])
                               if room.get("place_id") is not None}
        if observing:
            watch_started = True
            observation_complete = False
            # A later watch cycle may retain previously crossed child Places;
            # only newly created identities during the watch are forbidden.
            places_deferred = places_deferred and current_room_places <= known_room_places
            if event == "work_item_dispatched":
                frozen_work_preserved = False
        else:
            known_room_places.update(current_room_places)
        if is_root_auth and auth.get("phase") == "complete" and auth.get("all_finished"):
            observation_complete = True
        room = snapshot.get("active_semantic_room") or {}
        if event == "semantic_room_exploration_started" and room.get("node_id") == junction_id:
            room_entries_after_watch = room_entries_after_watch and observation_complete
        if event == "work_item_dispatched":
            if not watch_started:
                work_before_watch = True
            if room.get("node_id") == junction_id and room.get("place_id") is not None:
                context = snapshot.get("goal_context") or {}
                room_local_work_only = room_local_work_only and (
                    context.get("owner_place_id") == room.get("place_id"))

    active_rooms = [room for room in tree_rooms
                    if room.get("place_id") is not None
                    and not room.get("invalidated")
                    and not room.get("exploration_suppressed")]
    priorities = [room.get("priority") for room in active_rooms
                  if room.get("priority") is not None]
    ranked = sorted((room for room in active_rooms if room.get("priority") is not None),
                    key=lambda room: room["priority"])
    semantic_order = all(
        float(left.get("semantic_score") or 0.0) >= float(right.get("semantic_score") or 0.0)
        for left, right in zip(ranked, ranked[1:])
    )
    expected_order = [
        int(room["branch_id"]) for room in ranked
        if room.get("branch_id") is not None
        and not room.get("invalidated")
        and not room.get("exploration_suppressed")
    ]
    actual_order = []
    for snapshot in snapshots:
        room = snapshot.get("active_semantic_room") or {}
        if (snapshot.get("frontier_event") == "semantic_room_exploration_started"
                and room.get("node_id") == junction_id
                and room.get("branch_id") in expected_order):
            actual_order.append(int(room["branch_id"]))
    certified_ids={b.get("id") for snapshot in snapshots
                   for b in (snapshot.get("tree") or {}).get("branches",[])
                   if b.get("state")=="authenticated"
                   and b.get("authentication_state")=="completed"
                   and b.get("canonical_portal_id") is not None}
    stable_or_suppressed = all(
        int(room.get("stable_frame_count") or 0) > 0
        or (float(room.get("semantic_score") or -1.0) < 0
            and bool(room.get("exploration_suppressed")))
        or bool(room.get("invalidated"))
        or unknown_semantics_certified(room,certified_ids)
        for room in tree_rooms
    )
    target_branch_ids = {int(room["branch_id"]) for room in tree_rooms
                         if room.get("branch_id") is not None}
    target_portals = set()
    for snapshot in snapshots:
        for branch in (snapshot.get("tree") or {}).get("branches", []):
            try:
                if (int(branch.get("id")) in target_branch_ids
                        and branch.get("canonical_portal_id") is not None):
                    target_portals.add(int(branch["canonical_portal_id"]))
            except (TypeError, ValueError):
                continue
    cycle_portals = target_portals & set(closed)
    closed_branches = {int(item.get("branch_id")) for item in closed.values()
                       if item.get("branch_id") is not None}
    active_room_branches = {int(room["branch_id"]) for room in active_rooms
                            if room.get("branch_id") is not None}
    parent_directions=verified_parent_directions(snapshots,junction_id,tree_rooms)
    physical_direction_count=len(tree_rooms)+len(parent_directions)
    checks = {
        "junction_found": junction_id is not None,
        "normal_wi_used_before_junction": work_before_watch,
        "normal_wi_frozen_during_observation": watch_started and frozen_work_preserved,
        "place_creation_deferred_during_observation": watch_started and places_deferred,
        "room_exploration_after_observation_completed": observation_complete and room_entries_after_watch,
        "room_work_items_keep_current_place_owner": room_local_work_only,
        "expected_rooms_recorded": physical_direction_count >= expected_rooms,
        "semantic_scores_and_stable_frames_recorded": stable_or_suppressed,
        "priority_follows_semantic_score": semantic_order and len(priorities) >= len(active_rooms),
        "actual_room_exploration_follows_priority": bool(expected_order) and actual_order == expected_order,
        "each_target_portal_closed": bool(active_room_branches)
            and active_room_branches <= closed_branches,
        "closed_portals_suppress_reentry": cycle_portals <= set(suppressed),
        "room_work_items_settled_before_close": all(
            closed[portal].get("room_items_settled", False) for portal in cycle_portals
        ),
        "suspended_parent_work_item_resumed": parent_resumed,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "junction_id": junction_id,
        "junction_center_gap": closest if junction_id is not None else None,
        "expected_room_count": expected_rooms,
        "recorded_outgoing_room_count": len(tree_rooms),
        "recorded_physical_direction_count": physical_direction_count,
        "covered_parent_return_directions": parent_directions,
        "expected_exploration_order": expected_order,
        "actual_exploration_order": actual_order,
        "rooms": tree_rooms,
        "closed_rooms": [closed[key] for key in sorted(cycle_portals)],
        "reentry_suppressed_portals": sorted(suppressed),
        "parent_work_item_id": parent_item,
        "parent_work_item": parent_latest,
        "metrics_log": str(path),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("metrics")
    parser.add_argument("--center", nargs=2, type=float, required=True)
    parser.add_argument("--radius", type=float, default=2.0)
    parser.add_argument("--expected-rooms", type=int, required=True)
    parser.add_argument("--output")
    args = parser.parse_args()
    result = audit(args.metrics, args.center, args.radius, args.expected_rooms)
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
