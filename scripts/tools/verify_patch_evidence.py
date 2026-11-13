#!/usr/bin/env python3
"""Per-run ledger of the evidence each frontier patch leaves in the logs.

The recent commits are deliberately small and mostly inert on a healthy run,
so "the run succeeded" is not enough to tell whether a patch was exercised.
This tool reads one run directory and prints the observable facts that map to
each patch:

  departure gate  88289d5 / dd8f3d9
  auth retry      43bfc28
  graph identity  7513b0b
  stale return    fe9eee4
  turn gate       2ea96cf / b856e8e
  room view       eebff9e
  DFS suspend     8af3bf3 / f9e56d8
  target bounds   34d1193

Usage:
    python3 scripts/tools/verify_patch_evidence.py [run_dir] [--json]

A run directory defaults to the newest one under runtime/navigation/logs.
"""
import glob
import json
import os
import sys
from collections import Counter, deque

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LOG_ROOT = os.path.join(ROOT, "runtime", "navigation", "logs")

# These are scanned on raw log lines, so a branch snapshot repeated in every
# status payload inflates them.  Treat them as "was this string ever present",
# not as an exact event count.
FREE_TEXT = {
    "leak": "navfn_place_scope_path_leak",
    "high_water": "graph_transaction_id_below_high_water",
    "negative_turn": "negative_room_turn",
    "turn_last_resort": "negative_room_turn_last_resort",
    "skipped_negative": "portal_entry_policy_skipped_negative_room",
    "handoff_stall": "authentication_handoff_stall",
}

EVENTS = {
    "departure_prepared": "frontier_place_departure_prepared",
    "departure_waiting": "frontier_place_departure_waiting",
    "gate_fallback": "frontier_place_departure_gate_fallback",
    "gate_downgraded": "frontier_place_departure_gate_downgraded",
    "boundary_crossed": "frontier_place_boundary_crossed",
    "place_closed": "frontier_place_closed",
    "place_transited": "frontier_place_transited",
    "place_entered": "place_dfs_entered",
    "portal_started": "portal_transaction_started",
    "portal_finished": "portal_transaction_finished",
    "portal_aborted": "portal_transaction_aborted",
    "auth_started": "portal_authentication_branch_started",
    "auth_finished": "portal_authentication_branch_finished",
    "auth_retry": "portal_authentication_branch_retry",
    "auth_preempt": "portal_authentication_preempted_graph_route",
    "stale_route_settle": "portal_authentication_stale_route_grace",
    "route_selected": "route_selected",
    "dfs_suspended": "place_dfs_suspended_incomplete",
    "dfs_return_rejected": "place_dfs_suspended_return_rejected",
    "claim_wall_timeout": "target_region_claim_wall_timeout",
    "starved": "frontier_successor_starvation",
    "work_released": "target_observation_work_released",
    "preempt_expired": "planning_preempt_expired",
    "task_done": "task_done",
    "target_acquired": "target_acquired",
}


def newest_run():
    runs = [p for p in glob.glob(os.path.join(LOG_ROOT, "*")) if os.path.isdir(p)]
    return max(runs, key=os.path.getmtime)


def main(argv):
    args = [a for a in argv[1:] if not a.startswith("--")]
    as_json = "--json" in argv
    run = os.path.abspath(args[0]) if args else newest_run()
    tag = os.path.basename(run)
    metrics = glob.glob(os.path.join(run, "*_navigation_metrics.log"))
    if not metrics:
        sys.exit("no metrics log in %s" % run)
    metrics = metrics[0]

    counts = Counter()
    free = Counter()
    prepared = []
    finished = []
    downgrade_reasons = Counter()
    boundary_evidence = Counter()
    transits = []
    last_payload = None
    samples = deque(maxlen=4000)
    last_sim = 0.0

    with open(metrics, errors="replace") as handle:
        for line in handle:
            for key, token in FREE_TEXT.items():
                if token in line:
                    free[key] += 1
            if "event=sample" in line:
                samples.append(line)
            if '"frontier_event"' not in line:
                continue
            try:
                payload = json.loads(line.split(" data=", 1)[1])
            except Exception:
                continue
            name = payload.get("frontier_event", "")
            counts[name] += 1
            sim = payload.get("ros_time")
            if isinstance(sim, (int, float)):
                last_sim = max(last_sim, sim)
            last_payload = payload
            if name == EVENTS["departure_prepared"]:
                prepared.append((sim or 0.0, payload.get("region_id")))
            elif name == EVENTS["portal_finished"]:
                finished.append(sim or 0.0)
            elif name == EVENTS["gate_downgraded"]:
                downgrade_reasons[str(payload.get("reason", ""))] += 1
            elif name == EVENTS["boundary_crossed"]:
                boundary_evidence[str(payload.get("evidence") or "footprint")] += 1
            elif name == EVENTS["place_transited"]:
                transition = payload.get("transition") or {}
                transits.append((
                    sim or 0.0,
                    transition.get("from_place_id"),
                    transition.get("portal_id"),
                    "transit",
                    None,
                ))
            elif name == EVENTS["place_entered"]:
                room = payload.get("active_semantic_room") or {}
                transits.append((
                    sim or 0.0,
                    room.get("source_place_id"),
                    payload.get("portal_id"),
                    "dfs_enter",
                    payload.get("place_id"),
                ))

    # --- DFS / current place -------------------------------------------------
    dfs = {}
    for payload in (last_payload or {}),:
        tree = (payload or {}).get("place_dfs") if payload else None
        if isinstance(tree, dict) and tree.get("nodes"):
            dfs = tree
    nodes = {n.get("place_id"): n for n in dfs.get("nodes", [])}
    stack = list(dfs.get("stack") or [])
    current = None
    for place_id in reversed(stack):
        if nodes.get(place_id, {}).get("state") == "exploring":
            current = place_id
            break
    if current is None:
        for place_id, node in nodes.items():
            if node.get("state") == "exploring":
                current = place_id

    # --- place sequence (destination of each certified transit) --------------
    sequence = [1]
    for _sim, source, portal, kind, place_id in transits:
        destination = place_id
        if destination is None:
            for node in nodes.values():
                if (node.get("portal_id") == portal
                        and node.get("parent_id") == source):
                    destination = node.get("place_id")
        if destination is None:
            continue
        if sequence and sequence[-1] == source:
            sequence.append(destination)
        elif destination not in sequence:
            sequence.append(destination)

    # --- gate waits: prepared -> next portal finish --------------------------
    waits = []
    for sim, _region in prepared:
        later = [t for t in finished if t >= sim]
        if later:
            waits.append(round(later[0] - sim, 1))
        else:
            waits.append(None)

    # --- movement ------------------------------------------------------------
    last_move = None
    previous = None
    for raw in samples:
        try:
            payload = json.loads(raw.split(" data=", 1)[1])
        except Exception:
            continue
        pose = payload.get("pose")
        sim = payload.get("ros_time") or 0.0
        if not isinstance(pose, (list, tuple)) or len(pose) < 2:
            continue
        if previous is not None:
            moved = abs(pose[0] - previous[0]) + abs(pose[1] - previous[1])
            if moved > 0.02:
                last_move = sim
        previous = pose
    stationary = None
    if last_move is not None:
        stationary = round(last_sim - last_move, 1)

    # --- unresolved target work ---------------------------------------------
    semantic = (last_payload or {}).get("semantic_place") or {}
    target_work = semantic.get("target_observation_work")
    target_state = ""
    if isinstance(target_work, dict):
        target_state = str(target_work.get("state") or "")
        if not target_state:
            target_state = "present"
    elif target_work is None:
        target_state = "none"

    pending_departure = bool(prepared) and not any(
        t >= prepared[-1][0] for t in finished)

    result = {
        "run": tag,
        "sim": round(last_sim, 1),
        "places": sequence,
        "current_place": current,
        "stack": stack,
        "task_done": counts.get(EVENTS["task_done"], 0),
        "target_acquired": counts.get(EVENTS["target_acquired"], 0),
        "departure": {
            "prepared": counts.get(EVENTS["departure_prepared"], 0),
            "portal_started": counts.get(EVENTS["portal_started"], 0),
            "portal_finished": counts.get(EVENTS["portal_finished"], 0),
            "portal_aborted": counts.get(EVENTS["portal_aborted"], 0),
            "waiting": counts.get(EVENTS["departure_waiting"], 0),
            "waits": waits,
            "downgraded": dict(downgrade_reasons),
            "gate_fallback": counts.get(EVENTS["gate_fallback"], 0),
            "boundary_crossed": dict(boundary_evidence),
            "place_closed": counts.get(EVENTS["place_closed"], 0),
            "pending_at_end": pending_departure,
            "leak_lines": free["leak"],
        },
        "auth": {
            "branch_started": counts.get(EVENTS["auth_started"], 0),
            "branch_finished": counts.get(EVENTS["auth_finished"], 0),
            "branch_retry": counts.get(EVENTS["auth_retry"], 0),
            "preempts": counts.get(EVENTS["auth_preempt"], 0),
            "stale_route_settle": counts.get(EVENTS["stale_route_settle"], 0),
            "handoff_stall": free["handoff_stall"],
            "high_water_rejections": free["high_water"],
        },
        "turn": {
            "negative_turn": free["negative_turn"],
            "last_resort": free["turn_last_resort"],
            "skipped_negative": free["skipped_negative"],
        },
        "dfs": {
            "suspended_incomplete": counts.get(EVENTS["dfs_suspended"], 0),
            "return_rejected": counts.get(EVENTS["dfs_return_rejected"], 0),
        },
        "target": {
            "observation_work": target_state,
            "claim_wall_timeout": counts.get(EVENTS["claim_wall_timeout"], 0),
            "successor_starvation": counts.get(EVENTS["starved"], 0),
            "released": counts.get(EVENTS["work_released"], 0),
            "preempt_expired": counts.get(EVENTS["preempt_expired"], 0),
        },
        "stationary_seconds": stationary,
    }

    dep = result["departure"]
    auth = result["auth"]
    turn = result["turn"]
    dfs = result["dfs"]
    target = result["target"]
    gate_exercised = bool(
        dep["downgraded"] or dep["gate_fallback"]
        or dep["boundary_crossed"].get("outside_place_scope_mask"))
    result["verdict"] = {
        # "exercised" means the recovery branch actually ran in this run.
        "departure_gate_exercised": gate_exercised,
        "departure_leak_class": dep["leak_lines"] > 0,
        "auth_retry_exercised": auth["branch_retry"] > 0,
        "stale_return_exercised": auth["stale_route_settle"] > 0,
        "graph_identity_defect": auth["high_water_rejections"] > 0,
        "turn_negative_mentions": turn["negative_turn"] > 0,
        "turn_last_resort_exercised": turn["last_resort"] > 0,
        "dfs_suspend_exercised": bool(
            dfs["suspended_incomplete"] or dfs["return_rejected"]),
        "target_bound_exercised": bool(
            target["released"] or target["preempt_expired"]
            or target["claim_wall_timeout"]),
        "target_unresolved_at_end": target["observation_work"]
        not in ("", "none", "completed"),
    }

    if as_json:
        print(json.dumps(result, sort_keys=True))
        return

    dep = result["departure"]
    auth = result["auth"]
    print("run=%s  sim=%.0f  places=%s  current=%s" % (
        tag, result["sim"],
        ">".join(str(p) for p in result["places"]), current))
    print("result: task_done=%d target_acquired=%d  stack=%s" % (
        result["task_done"], result["target_acquired"], stack))
    print("gate: prepared=%d started=%d finished=%d aborted=%d  waiting=%d  "
          "waits=%s" % (
              dep["prepared"], dep["portal_started"], dep["portal_finished"],
              dep["portal_aborted"], dep["waiting"], dep["waits"]))
    print("gate: downgraded=%s fallback=%d boundary=%s closed=%d "
          "pending_at_end=%s leak=%d" % (
              dep["downgraded"] or "{}", dep["gate_fallback"],
              dep["boundary_crossed"] or "{}", dep["place_closed"],
              dep["pending_at_end"], dep["leak_lines"]))
    print("auth: started=%d finished=%d retry=%d handoff_stall=%d "
          "preempts=%d stale_settle=%d high_water=%d" % (
              auth["branch_started"], auth["branch_finished"],
              auth["branch_retry"], auth["handoff_stall"], auth["preempts"],
              auth["stale_route_settle"], auth["high_water_rejections"]))
    print("turn: negative=%d last_resort=%d skipped_negative=%d" % (
        result["turn"]["negative_turn"], result["turn"]["last_resort"],
        result["turn"]["skipped_negative"]))
    print("dfs: suspended_incomplete=%d return_rejected=%d" % (
        result["dfs"]["suspended_incomplete"],
        result["dfs"]["return_rejected"]))
    print("target: observation_work=%s claim_timeout=%d starvation=%d" % (
        result["target"]["observation_work"],
        result["target"]["claim_wall_timeout"],
        result["target"]["successor_starvation"]))
    if stationary is not None:
        print("motion: last_move=%.1f  stationary=%.1fs" % (
            last_sim - stationary, stationary))
    verdict = result["verdict"]
    print("patches:")
    print("  88289d5/round-2 gate: exercised=%s   leak_class=%s" % (
        verdict["departure_gate_exercised"],
        verdict["departure_leak_class"]))
    print("  43bfc28 auth retry: exercised=%s" % (
        verdict["auth_retry_exercised"]))
    print("  7513b0b graph floor: defect_seen=%s" % (
        verdict["graph_identity_defect"]))
    print("  fe9eee4 stale return: exercised=%s" % (
        verdict["stale_return_exercised"]))
    print("  2ea96cf/b856e8e turn gate: negative_mentions=%s "
          "last_resort=%s" % (
              verdict["turn_negative_mentions"],
              verdict["turn_last_resort_exercised"]))
    print("  8af3bf3/f9e56d8 dfs suspend: exercised=%s" % (
        verdict["dfs_suspend_exercised"]))
    print("  34d1193 target bounds: exercised=%s unresolved_at_end=%s" % (
        verdict["target_bound_exercised"],
        verdict["target_unresolved_at_end"]))


if __name__ == "__main__":
    main(sys.argv)
