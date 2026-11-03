#!/usr/bin/env python3
"""Audit one GP junction run's serial doorway-observation transactions."""
import argparse
import json
import math
from collections import Counter
from pathlib import Path

AUTH_PREFIX = "portal_authentication_"

def load_events(path):
    events = []
    tree_index = None
    dedup_max = {}
    candidate_history = {}
    damaged = 0
    with Path(path).open(encoding="utf-8", errors="replace") as stream:
        for lineno, line in enumerate(stream, 1):
            if not line.strip():
                continue
            if not line.endswith("\n"):
                # The metrics node is stopped with SIGINT; only the very last
                # line can be cut.  Anything else is real corruption.
                if stream.readline() == "":
                    break
                raise ValueError("incomplete metrics record at line %d" % lineno)
            if " data=" not in line:
                raise ValueError("incomplete metrics record at line %d" % lineno)
            # Every trajectory sample is evidence too: never let a corrupt
            # non-Portal record silently disappear behind a passing auth gate.
            try:
                data = json.loads(line.split(" data=", 1)[1])
            except ValueError:
                damaged += 1
                if damaged > 3:
                    raise
                continue
            if not isinstance(data, dict):
                raise ValueError("non-object metrics record at line %d" % lineno)
            if " event=global_frontier_event " not in line:
                continue
            name = str(data.get("frontier_event", ""))
            if name == "gp_branch_tree":
                for key, value in (data.get("dedup_audit") or {}).items():
                    if isinstance(value, (int, float)):
                        dedup_max[key] = max(dedup_max.get(key, 0), int(value))
                previous_tree = None if tree_index is None else events[tree_index][1].get("tree")
                snapshot_tree = data.get("tree") or previous_tree
                for branch_id, fact in candidate_branch_facts(snapshot_tree).items():
                    candidate_history.setdefault(branch_id, fact)
                record = (name, {"tree": snapshot_tree,
                                 "dedup_audit": dict(dedup_max),
                                 "candidate_history": dict(candidate_history)})
                if tree_index is None:
                    tree_index = len(events)
                    events.append(record)
                else:
                    events[tree_index] = record
            elif name.startswith(AUTH_PREFIX) or name == "portal_candidate_rejected":
                # Preserve every audit input, without retaining unrelated full
                # planner/region/WI snapshots for every event in a two-hour run.
                keys = ("node_id", "branch_id", "phase", "state", "failure_reason",
                        "snapshot", "left_frontier_cells", "right_frontier_cells",
                        "turn_side", "turn_direction", "target_yaw", "all_finished",
                        "reason", "spatial_evidence", "ros_time")
                events.append((name, {key: data[key] for key in keys if key in data}))
    return events

def candidate_branch_facts(tree):
    """Stable open doorways recorded on a confirmed junction in one snapshot."""
    facts = {}
    if not tree:
        return facts
    nodes = {int(n["id"]): n for n in tree.get("nodes", [])
             if n.get("id") is not None}
    for branch in tree.get("branches", []):
        try:
            branch_id = int(branch.get("id", -1))
            node = nodes.get(int(branch.get("node_id", -2)))
        except (TypeError, ValueError):
            continue
        if node is None or not node.get("junction_confirmed"):
            continue
        if not branch.get("stable") or branch.get("passability") != "open":
            continue
        if branch.get("physical_gate") is None:
            continue
        if (branch.get("connection") == "return"
                or branch.get("state") in ("return", "superseded")):
            continue
        facts[branch_id] = {
            "node_id": int(node["id"]),
            "node_xy": [float(v) for v in (node.get("physical_xy") or [])],
        }
    return facts


def observed_candidate_branch_ids(history, junction_center=None,
                                  junction_radius=2.0):
    """Branch ids ever observed as a candidate during the run."""
    observed = set()
    for branch_id, fact in (history or {}).items():
        if junction_center is not None:
            try:
                if math.dist([float(v) for v in fact.get("node_xy") or []],
                             [float(v) for v in junction_center]) > float(junction_radius):
                    continue
            except (TypeError, ValueError):
                continue
        try:
            observed.add(int(branch_id))
        except (TypeError, ValueError):
            continue
    return observed


def stable_unstarted_candidate(branch):
    """A candidate is only unfinished while its authentication is undecided.

    Run 20261103_141641 node 8 branches 26/27/29 were rejected analytically
    (authentication_state=failed, reason measured_section_not_passable after 28
    withdraw cycles) without a physical start.  They are decisions, not missing
    work.  Node 24 branches 75/76/77 were never touched at all; those stay
    candidates and still fail every_stable_candidate_started.
    """
    return bool(
        branch
        and branch.get("stable")
        and branch.get("passability") == "open"
        and branch.get("physical_gate") is not None
        and branch.get("connection") != "return"
        and branch.get("state") not in ("return", "superseded")
        and str(branch.get("authentication_state") or "") in ("", "pending")
    )


def ordered(names, required):
    at = -1
    for item in required:
        try:
            at = names.index(item, at + 1)
        except ValueError:
            return False
    return True

def bounded_stall_returned(snapshot):
    """Accept a timed-out candidate only with bounded stall and physical return."""
    reason = str(snapshot.get("failure_reason", ""))
    if not reason.startswith("authentication_physical_stall:"):
        return False
    events = snapshot.get("events", [])
    names = [event.get("event") for event in events]
    if not ordered(names, ["doorway_physical_progress_expired",
                           "doorway_failure_return_started",
                           "returned_to_junction_node_after_failure"]):
        return False
    phase = reason.rsplit(":", 1)[-1]
    expired = next((e for e in events if e.get("event") == "doorway_physical_progress_expired"
                    and e.get("phase") == phase), {})
    returned = next((e for e in reversed(events)
                     if e.get("event") == "returned_to_junction_node_after_failure"), {})
    try:
        budget, elapsed = float(expired["timeout"]), float(expired["elapsed"])
        source, pose = snapshot["source_xy"], returned["pose_xy"]
        if len(source) != 2 or len(pose) != 2:
            return False
        distance = math.dist(source, pose)
        radius = float(snapshot.get("return_radius", .75))
        return (all(math.isfinite(v) for v in (budget, elapsed, distance, radius))
                and 0 < budget <= 12 and budget <= elapsed <= budget + 2
                and 0 < radius <= .75 and distance <= radius
                and returned.get("reason") == reason
                and abs(distance - float(returned["distance"])) <= .02
                and snapshot.get("source_side_proven") is True)
    except (KeyError, TypeError, ValueError):
        return False


def bounded_stall_settled_at_source(snapshot):
    """No artificial return is needed when an entering probe never left source."""
    reason = snapshot.get("failure_reason")
    if reason != "authentication_physical_stall:entering":return False
    events = snapshot.get("events", [])
    names = [e.get("event") for e in events]
    if (snapshot.get("inside_xy") is not None or "vehicle_fully_inside" in names
            or not ordered(names,["doorway_physical_progress_expired",
                                  "doorway_failure_settled_at_source",
                                  "doorway_authentication_failed"])):return False
    expired=next((e for e in events if e.get("event")=="doorway_physical_progress_expired"
                  and e.get("phase")=="entering"),{})
    settled=next((e for e in events if e.get("event")=="doorway_failure_settled_at_source"),{})
    try:
        source,pose=snapshot["source_xy"],settled["pose_xy"]
        if len(source)!=2 or len(pose)!=2 or len(settled["source_xy"])!=2:return False
        budget,elapsed=float(expired["timeout"]),float(expired["elapsed"])
        depth=float(settled["signed_depth"]);distance=math.dist(source,pose)
        radius=float(snapshot.get("return_radius",.75))
        return (snapshot.get("source_side_proven") is True
                and settled.get("reason")==reason and settled.get("phase")=="entering"
                and all(math.isfinite(v) for v in (budget,elapsed,depth,distance,radius))
                and 0<budget<=12 and budget<=elapsed<=budget+2
                and depth<-.05 and 0<radius<=.75 and distance<=radius
                and math.dist(source,settled["source_xy"])<1e-6
                and math.dist(pose,expired["pose_xy"])<.02
                and abs(distance-float(settled["distance"]))<=.02)
    except (KeyError,TypeError,ValueError):return False


def settled_at_source_without_interior_goal(snapshot):
    """A probe whose planner found no interior goal never left the source side.

    Run 20261103_115704 branch 43: doorway_entry_started ->
    doorway_interior_plan_unavailable(feasible_interior_goal_count=0,
    map_stamp=1357.5) -> doorway_failure_settled_at_source(distance=0.44,
    signed_depth=-3.89) -> doorway_authentication_failed, with inside_xy null
    and no vehicle_fully_inside event.  There is nothing to return from, so the
    physical return that evidenced_rejection_returned() demands cannot exist;
    the same distance/source proof used for a bounded entering stall is enough.
    """
    reason = snapshot.get("failure_reason")
    if reason != "no_reachable_interior_entry_goal":
        return False
    events = snapshot.get("events", [])
    names = [e.get("event") for e in events]
    if (snapshot.get("inside_xy") is not None or "vehicle_fully_inside" in names
            or not ordered(names, ["doorway_interior_plan_unavailable",
                                   "doorway_failure_settled_at_source",
                                   "doorway_authentication_failed"])):
        return False
    proof = next((e for e in events
                  if e.get("event") == "doorway_interior_plan_unavailable"), {})
    settled = next((e for e in events
                    if e.get("event") == "doorway_failure_settled_at_source"), {})
    try:
        source, pose = snapshot["source_xy"], settled["pose_xy"]
        if len(source) != 2 or len(pose) != 2 or len(settled["source_xy"]) != 2:
            return False
        distance = math.dist(source, pose)
        radius = float(snapshot.get("return_radius", .75))
        depth = float(settled["signed_depth"])
        return (snapshot.get("source_side_proven") is True
                and proof.get("feasible_interior_goal_count") == 0
                and float(proof["map_stamp"]) > 0
                and settled.get("reason") == reason
                and settled.get("phase") == "entering"
                and all(math.isfinite(v) for v in (distance, radius, depth))
                and depth < -.05 and 0 < radius <= .75 and distance <= radius
                and math.dist(source, settled["source_xy"]) < 1e-6
                and abs(distance - float(settled["distance"])) <= .02)
    except (KeyError, TypeError, ValueError):
        return False


def same_place_rejection_evidence(reason, spatial, snapshot, success_sequence,
                                  target_locked, returned_failure,
                                  lineage_preserved):
    """A rejected candidate is resolved only with the full cycle and return.

    Two runtime reasons mean "this is not a new place": the destination merges
    with the source component (known_free_bypass_within_current_place) or lands
    on the entry side the robot came in through
    (visited_entry_side_of_current_place).  Both need the same proof: physical
    entry, interior observation, the room-facing turn, the return to the
    junction and an intact branch lineage.  Only the label geometry differs.
    """
    if not spatial or snapshot.get("source_side_proven") is not True:
        return False
    if snapshot.get("return_gate_cleared") is not True:
        return False
    if not (success_sequence and target_locked and returned_failure
            and lineage_preserved):
        return False
    try:
        source = int(spatial.get("source_label", 0))
        destination = int(spatial.get("destination_label", 0))
        entry = int(spatial.get("entry_label", 0))
    except (TypeError, ValueError):
        return False
    if reason == "known_free_bypass_within_current_place":
        return source > 0 and source == destination == entry
    if reason == "visited_entry_side_of_current_place":
        return destination > 0 and entry == destination and source != destination
    return False


def evidenced_rejection_returned(snapshot):
    """Planner rejection or phase deadline needs explicit evidence and real return."""
    events=snapshot.get("events", []);names=[e.get("event") for e in events]
    reason=str(snapshot.get("failure_reason", ""))
    proof_name=("doorway_interior_plan_unavailable" if reason == "no_reachable_interior_entry_goal"
                else "doorway_navigation_phase_budget_expired")
    if not ordered(names,[proof_name,"doorway_failure_return_started","returned_to_junction_node_after_failure"]):
        return False
    proof=next((e for e in events if e.get("event")==proof_name
                and (proof_name=="doorway_interior_plan_unavailable"
                     or reason=="navigation_route_failed:"+str(e.get("reason")))),None)
    if proof is None:return False
    returned=next(e for e in reversed(events) if e.get("event")=="returned_to_junction_node_after_failure")
    try:
        source,pose=snapshot["source_xy"],returned["pose_xy"]
        if len(source)!=2 or len(pose)!=2:return False
        distance=math.dist(source,pose);radius=float(snapshot.get("return_radius",.75))
        physical=(all(math.isfinite(v) for v in (distance,radius)) and 0<radius<=.75
                  and distance<=radius and abs(distance-float(returned["distance"]))<=.02
                  and returned.get("reason")==reason and snapshot.get("source_side_proven") is True)
        if not physical:return False
        if proof_name=="doorway_interior_plan_unavailable":
            clearance=float(proof["navigation_clearance"]);width=float(snapshot["opening_width"])
            bounds=snapshot["physical_bounds"]
            stamp,minimum,required=float(proof["map_stamp"]),float(proof["minimum_depth"]),float(snapshot["required_depth"])
            return (proof["feasible_interior_goal_count"]==0 and stamp>0
                    and all(math.isfinite(v) for v in (stamp,minimum,required)) and required>0
                    and int(proof["map_epoch"])>0 and 0<clearance and math.isfinite(clearance)
                    and math.isfinite(width) and width>=max(.65,2*clearance)
                    and len(bounds)==2 and all(len(p)==2 for p in bounds)
                    and math.dist(*bounds)>=2*clearance
                    and all(math.isfinite(float(v)) for p in bounds for v in p)
                    and float(proof["minimum_depth"])>=float(snapshot["required_depth"])+.1)
        timeout,elapsed=float(proof["timeout"]),float(proof["elapsed"])
        return (reason=="navigation_route_failed:"+str(proof["reason"])
                and proof.get("phase")=="turning" and int(proof["route_id"])>0
                and all(math.isfinite(v) for v in (timeout,elapsed))
                and 0<timeout<=12 and timeout<=elapsed<=timeout+2)
    except (KeyError,TypeError,ValueError,OverflowError,StopIteration):
        return False


def audit(path, junction_center=None, junction_radius=2.0, all_started_nodes=False):
    events = load_events(path)
    latest_tree = None
    for name, data in events:
        if name == "gp_branch_tree":
            latest_tree = data.get("tree") or latest_tree
    target_node_ids = set()
    if latest_tree:
        for node in latest_tree.get("nodes", []):
            if not node.get("junction_confirmed"):
                continue
            if junction_center is not None:
                try:
                    if math.dist(
                        [float(v) for v in node.get("physical_xy", [])],
                        [float(v) for v in junction_center],
                    ) > float(junction_radius):
                        continue
                except (TypeError, ValueError):
                    continue
            target_node_ids.add(int(node["id"]))
    named = [(name, data) for name, data in events
             if name.startswith(AUTH_PREFIX)
             and (all_started_nodes or not target_node_ids or int(data.get("node_id", -1)) in target_node_ids)]
    explicit_starts = [data for name, data in named
                       if name == "portal_authentication_branch_started"]
    explicit_ids = {int(x["branch_id"]) for x in explicit_starts
                    if x.get("branch_id") is not None}
    inferred_starts = [data for name, data in named
                       if name == "portal_authentication_route_selected"
                       and data.get("phase") == "entering"
                       and data.get("branch_id") is not None
                       and int(data["branch_id"]) not in explicit_ids]
    starts = explicit_starts + inferred_starts
    finished = [data for name, data in named if name == "portal_authentication_branch_finished"]
    started_ids = []
    for data in starts:
        branch_id = int(data["branch_id"])
        if branch_id not in started_ids:
            started_ids.append(branch_id)
    finish_by_id = {int(x["branch_id"]): x for x in finished if x.get("branch_id") is not None}
    evidence_by_id = {}
    inside_by_id = {}
    latest_auth_state = {}
    same_place_rejections = {}
    latest_tree = None
    dedup = Counter()
    for name, data in events:
        if name == "gp_branch_tree":
            latest_tree = data.get("tree") or latest_tree
            for k, v in (data.get("dedup_audit") or {}).items():
                if isinstance(v, (int, float)):
                    dedup[k] = max(dedup[k], int(v))
        if ((name == "portal_authentication_branch_started"
             or (name == "portal_authentication_route_selected"
                 and data.get("phase") == "entering"))
                and data.get("branch_id") is not None):
            latest_auth_state[int(data["branch_id"])] = data.get("state") or "entering"
        if name == "portal_authentication_inside_evidence" and data.get("branch_id") is not None:
            inside_by_id[int(data["branch_id"])] = data
            latest_auth_state[int(data["branch_id"])] = data.get("state")
        if name == "portal_authentication_terminal_evidence":
            snap = data.get("snapshot") or {}
            if data.get("branch_id") is not None:
                evidence_by_id[int(data["branch_id"])] = snap
                latest_auth_state[int(data["branch_id"])] = snap.get("state")
        if name == "portal_candidate_rejected" and data.get("reason") in (
                "known_free_bypass_within_current_place",
                "visited_entry_side_of_current_place"):
            spatial = data.get("spatial_evidence") or {}
            if data.get("branch_id") is not None and spatial:
                same_place_rejections[int(data["branch_id"])] = spatial
    tree_branches = {}
    if latest_tree:
        tree_branches = {int(b["id"]): b for b in latest_tree.get("branches", [])
                         if b.get("id") is not None}
        for branch in latest_tree.get("branches", []):
            snap = branch.get("authentication_evidence")
            if snap and branch.get("id") is not None:
                evidence_by_id[int(branch["id"])] = snap
    duplicate_starts = sorted(k for k, n in Counter(int(x["branch_id"]) for x in starts).items() if n > 1)
    branches = []
    for branch_id in started_ids:
        finish = finish_by_id.get(branch_id, {})
        snap = evidence_by_id.get(branch_id, {})
        names = [item.get("event") for item in snap.get("events", [])]
        state = finish.get("state") or snap.get("state") or latest_auth_state.get(branch_id)
        inside_live = inside_by_id.get(branch_id, {})
        inside_evidence = next((x for x in snap.get("events", [])
                                if x.get("event") == "inside_frontier_observed"), {})
        if not inside_evidence and inside_live.get("state") == "turning":
            inside_evidence = {
                "left": inside_live.get("left_frontier_cells", 0),
                "right": inside_live.get("right_frontier_cells", 0),
                "turn_side": inside_live.get("turn_side"),
                "turn_direction": inside_live.get("turn_direction"),
                "target_yaw": inside_live.get("target_yaw"),
            }
            names = ["doorway_entry_started", "vehicle_fully_inside",
                     "inside_frontier_observed"]
        success_sequence = ordered(names, [
            "doorway_entry_started", "vehicle_fully_inside",
            "inside_frontier_observed", "room_facing_turn_complete",
            "returned_to_junction_node",
        ])
        inside = inside_evidence
        routes = [d for n, d in named if n == "portal_authentication_route_selected"
                  and int(d.get("branch_id", -1)) == branch_id]
        turn_routes = [d for d in routes if d.get("phase") in ("turning", "returning")]
        target = inside.get("target_yaw")
        target_locked = bool(target is not None and turn_routes and all(
            d.get("target_yaw") is not None
            and abs(((float(d["target_yaw"]) - float(target) + 3.141592653589793)
                     % 6.283185307179586) - 3.141592653589793) < 0.03
            for d in turn_routes))
        returned_failure = (
            "returned_to_junction_node_after_failure" in names
            or ordered(names, ["returned_to_junction_node", "portal_candidate_rejected"])
        )
        tree_branch = tree_branches.get(branch_id, {})
        parent_node = next((n for n in (latest_tree or {}).get("nodes", [])
                            if int(n.get("id", -1)) == int(tree_branch.get("node_id", -2))), {})
        lineage_preserved = (
            branch_id in [int(x) for x in parent_node.get("branch_ids", [])]
            and tree_branch.get("state") == "authentication_failed"
            and tree_branch.get("authentication_state") == "failed"
        )
        failure_reason = finish.get("failure_reason") or snap.get("failure_reason")
        rejected_false_candidate = (
            state == "failed"
            and failure_reason in (
                "no_frontier_inside_doorway",
                "portal_source_side_geometry_unproven",
            )
            and returned_failure and lineage_preserved
        )
        rejected_same_place_candidate = (
            state == "failed"
            and same_place_rejection_evidence(
                failure_reason, same_place_rejections.get(branch_id), snap,
                success_sequence, target_locked, returned_failure,
                lineage_preserved)
        )
        bounded_navigation_failure = (
            state == "failed" and lineage_preserved
            and (bounded_stall_returned(snap) or bounded_stall_settled_at_source(snap))
        )
        evidenced_returned_failure = (state == "failed" and lineage_preserved
                                       and evidenced_rejection_returned(snap))
        interior_plan_settled_at_source = (
            state == "failed" and lineage_preserved
            and settled_at_source_without_interior_goal(snap)
        )
        passed_branch = (
            (state == "completed" and success_sequence and target_locked)
            or rejected_false_candidate or rejected_same_place_candidate or bounded_navigation_failure
            or evidenced_returned_failure or interior_plan_settled_at_source
        )
        branches.append({
            "branch_id": branch_id,
            "state": state,
            "failure_reason": failure_reason,
            "frontier_left": inside.get("left"),
            "frontier_right": inside.get("right"),
            "turn_side": inside.get("turn_side"),
            "turn_direction": inside.get("turn_direction"),
            "target_yaw": target,
            "event_sequence": names,
            "returned_after_failure": returned_failure,
            "checks": {
                "completed_full_cycle": bool(success_sequence),
                "locked_turn_target": target_locked,
                "rejected_false_candidate_and_returned": rejected_false_candidate,
                "rejected_same_place_candidate_and_returned": rejected_same_place_candidate,
                "tree_lineage_preserved": lineage_preserved,
                "bounded_stall_failed_and_returned": bounded_navigation_failure,
            },
            "passed": bool(passed_branch),
        })
    observation_updates = [data for name, data in named
                           if name == "portal_authentication_observation_period"]
    observation_complete = bool(observation_updates and
        observation_updates[-1].get("all_finished") is True and
        observation_updates[-1].get("phase") == "complete")
    candidate_ids = []
    if latest_tree:
        nodes = {int(x["id"]): x for x in latest_tree.get("nodes", [])}
        target_nodes = []
        for node in nodes.values():
            if not node.get("junction_confirmed"):
                continue
            if junction_center is not None:
                try:
                    if math.dist(
                        [float(v) for v in node.get("physical_xy", [])],
                        [float(v) for v in junction_center],
                    ) > float(junction_radius):
                        continue
                except (TypeError, ValueError):
                    continue
            target_nodes.append(node)
        for node in target_nodes:
            for branch_id in node.get("branch_ids", []):
                branch = next((x for x in latest_tree.get("branches", [])
                               if int(x.get("id", -1)) == int(branch_id)), None)
                if stable_unstarted_candidate(branch):
                    candidate_ids.append(int(branch_id))
    candidate_history = {}
    for name, data in events:
        if name == "gp_branch_tree":
            candidate_history = data.get("candidate_history") or candidate_history
    observed_candidate_ids = observed_candidate_branch_ids(
        candidate_history, junction_center, junction_radius)
    checks = {
        "gp_tree_recorded": latest_tree is not None,
        "confirmed_junction_candidates_observed": bool(observed_candidate_ids),
        "every_stable_candidate_started": set(candidate_ids).issubset(set(started_ids)),
        "no_branch_started_twice": not duplicate_starts,
        "all_started_branches_settled": bool(started_ids) and all(
            x in finish_by_id for x in started_ids),
        "all_doorway_candidates_resolved": bool(branches) and all(
            item["passed"] for item in branches),
        "observation_period_complete": observation_complete,
        "online_slam_dedup_audit_present": bool(dedup),
    }
    result = {
        "metrics_log": str(Path(path).resolve()),
        "passed": all(checks.values()),
        "checks": checks,
        "candidate_branch_ids": sorted(set(candidate_ids)),
        "observed_candidate_branch_ids": sorted(observed_candidate_ids),
        "all_started_nodes": bool(all_started_nodes),
        "started_branch_ids": started_ids,
        "duplicate_started_branch_ids": duplicate_starts,
        "branches": branches,
        "online_slam_dedup_audit": dict(dedup),
        "event_counts": dict(Counter(name for name, _ in named)),
    }
    return result

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("metrics_log", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--junction-center", nargs=2, type=float)
    parser.add_argument("--junction-radius", type=float, default=2.0)
    parser.add_argument("--all-started-nodes", action="store_true")
    args = parser.parse_args()
    result = audit(
        args.metrics_log, junction_center=args.junction_center,
        junction_radius=args.junction_radius, all_started_nodes=args.all_started_nodes,
    )
    rendered = json.dumps(result, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    raise SystemExit(0 if result["passed"] else 1)

if __name__ == "__main__":
    main()
