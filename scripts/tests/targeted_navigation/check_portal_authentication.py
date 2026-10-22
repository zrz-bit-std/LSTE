#!/usr/bin/env python3
"""Audit one GP junction run's serial doorway-observation transactions."""
import argparse
import json
from collections import Counter
from pathlib import Path

AUTH_PREFIX = "portal_authentication_"

def load_events(path):
    events = []
    with Path(path).open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if " event=global_frontier_event " not in line or " data=" not in line:
                continue
            try:
                data = json.loads(line.split(" data=", 1)[1])
            except (ValueError, TypeError):
                continue
            name = str(data.get("frontier_event", ""))
            events.append((name, data))
    return events

def ordered(names, required):
    at = -1
    for item in required:
        try:
            at = names.index(item, at + 1)
        except ValueError:
            return False
    return True

def audit(path):
    events = load_events(path)
    named = [(name, data) for name, data in events if name.startswith(AUTH_PREFIX)]
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
        turn_routes = [d for d in routes if d.get("phase") == "turning"]
        target = inside.get("target_yaw")
        target_locked = bool(target is not None and turn_routes and all(
            d.get("target_yaw") is not None
            and abs(((float(d["target_yaw"]) - float(target) + 3.141592653589793)
                     % 6.283185307179586) - 3.141592653589793) < 0.03
            for d in turn_routes))
        returned_failure = "returned_to_junction_node_after_failure" in names
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
            state == "failed" and failure_reason == "no_frontier_inside_doorway"
            and returned_failure and lineage_preserved
        )
        passed_branch = (
            (state == "completed" and success_sequence and target_locked)
            or rejected_false_candidate
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
                "tree_lineage_preserved": lineage_preserved,
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
        for node in nodes.values():
            if not node.get("junction_confirmed"):
                continue
            for branch_id in node.get("branch_ids", []):
                branch = next((x for x in latest_tree.get("branches", [])
                               if int(x.get("id", -1)) == int(branch_id)), None)
                if (branch and branch.get("stable")
                    and branch.get("passability") == "open"
                    and branch.get("physical_gate") is not None
                    and branch.get("connection") != "return"
                    and branch.get("state") != "return"
                    and branch.get("state") != "superseded"):
                    candidate_ids.append(int(branch_id))
    checks = {
        "gp_tree_recorded": latest_tree is not None,
        "confirmed_junction_candidates_observed": bool(candidate_ids),
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
    args = parser.parse_args()
    result = audit(args.metrics_log)
    rendered = json.dumps(result, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    raise SystemExit(0 if result["passed"] else 1)

if __name__ == "__main__":
    main()
