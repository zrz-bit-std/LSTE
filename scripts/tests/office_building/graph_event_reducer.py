"""Pure replay reducer for Place/Portal graph invariants.

The online ROS node owns mutable execution state.  This module deliberately
does not import ROS or inspect a map: it replays timestamped global-frontier
events and checks whether the persisted graph contract was respected.  A
development run can therefore be audited independently of the process that
produced it.
"""

from collections import Counter


_TRANSACTION_PHASE_ORDER = {
    "source_probe": 0,
    "throat": 1,
    "destination_standoff": 2,
    "crossing_verified": 3,
    "place_commit": 4,
}


def _positive_int(value):
    try:
        value = int(value)
    except (TypeError, ValueError):
        return None
    return value if value > 0 else None


def _nested_transaction(data):
    value = data.get("portal_transaction")
    return value if isinstance(value, dict) else {}


def _transaction_id(data):
    """Read a transaction id from either the event or its status snapshot."""
    value = _positive_int(data.get("transaction_id"))
    if value is not None:
        return value
    return _positive_int(_nested_transaction(data).get("transaction_id"))


def replay_graph_events(events):
    """Reduce structured global-frontier events into auditable invariants."""
    transactions = {}
    places = {}
    portal_destinations = {}
    action_counts = Counter()
    action_reasons = Counter()
    route_ids = set()
    illegal_actions = []
    transaction_violations = []
    phantom_place_commits = []
    stale_transaction_commits = []
    place_entries = []
    suspended_places = Counter()
    selected_portals = Counter()
    failed_portals = set()
    same_edge_reselections = []
    graph_plan_mismatches = []
    graph_plan_reconciliations = []
    route_plan_action_mismatches = []

    for outer_event, data in events or ():
        if outer_event != "global_frontier_event" or not isinstance(data, dict):
            continue
        event = str(data.get("frontier_event") or "")
        route_id = _positive_int(data.get("route_id"))
        if route_id is not None:
            route_ids.add(route_id)

        if event == "route_selected":
            action = str(data.get("graph_action") or "")
            if action:
                action_counts[action] += 1
            reason = str(data.get("graph_action_reason") or "")
            if reason:
                action_reasons[reason] += 1
            plan = data.get("graph_route_plan")
            if isinstance(plan, dict) and action:
                planned_action = str(plan.get("action") or "")
                if planned_action and planned_action != action:
                    route_plan_action_mismatches.append({
                        "route_id": route_id,
                        "planned_action": planned_action,
                        "selected_action": action,
                    })
            if action == "observe_local_work" and _positive_int(data.get("work_item_id")) is None:
                illegal_actions.append({
                    "route_id": route_id,
                    "reason": "local_action_without_work_item",
                })
            if action == "cross_portal":
                tx = _nested_transaction(data)
                tx_route = _positive_int(tx.get("route_id"))
                if tx.get("state") not in _TRANSACTION_PHASE_ORDER or tx_route != route_id:
                    illegal_actions.append({
                        "route_id": route_id,
                        "reason": "crossing_without_matching_transaction",
                    })
                portal_id = _positive_int(tx.get("portal_id"))
                if portal_id is not None:
                    selected_portals[portal_id] += 1

        elif event == "graph_route_plan_mismatch":
            graph_plan_mismatches.append({
                "route_id": route_id,
                "transaction_id": _transaction_id(data),
                "planned_action": data.get("planned_action"),
                "candidate_action": data.get("candidate_action"),
                "reason": data.get("reason"),
            })

        elif event == "graph_route_plan_reconciled":
            graph_plan_reconciliations.append({
                "route_id": route_id,
                "transaction_id": _transaction_id(data),
                "planned_action": data.get("planned_action"),
                "committed_action": data.get("committed_action"),
                "reason": data.get("reason"),
            })

        elif event == "portal_transaction_started":
            tx_id = _transaction_id(data)
            if tx_id is None:
                transaction_violations.append({"event": event, "reason": "missing_transaction_id"})
                continue
            current = transactions.get(tx_id)
            identity = (
                _positive_int(data.get("route_id")),
                _positive_int(data.get("portal_id")),
                _positive_int(data.get("source_place_id")),
            )
            if identity[1] in failed_portals:
                same_edge_reselections.append({
                    "transaction_id": tx_id,
                    "portal_id": identity[1],
                    "route_id": identity[0],
                    "reason": "failed_portal_reselected",
                })
            if current is not None and current.get("identity") != identity:
                transaction_violations.append({
                    "transaction_id": tx_id,
                    "reason": "transaction_identity_changed",
                })
            transactions[tx_id] = {
                "identity": identity,
                "route_id": identity[0],
                "state": str(data.get("state") or ""),
                "phase_history": [str(data.get("state") or "")],
                "finished": False,
            }

        elif event == "portal_transaction_phase":
            tx_id = _transaction_id(data)
            if tx_id is None or tx_id not in transactions:
                transaction_violations.append({
                    "transaction_id": tx_id,
                    "reason": "phase_without_transaction",
                })
                continue
            current = transactions[tx_id]
            phase = str(data.get("state") or "")
            previous = _TRANSACTION_PHASE_ORDER.get(current.get("state"), -1)
            next_phase = _TRANSACTION_PHASE_ORDER.get(phase, -1)
            if next_phase < previous:
                transaction_violations.append({
                    "transaction_id": tx_id,
                    "reason": "transaction_phase_regressed",
                    "from": current.get("state"),
                    "to": phase,
                })
            phase_route = _positive_int(data.get("route_id"))
            if phase_route is not None and phase_route != current.get("route_id"):
                transaction_violations.append({
                    "transaction_id": tx_id,
                    "reason": "transaction_route_changed",
                })
            current["state"] = phase
            current.setdefault("phase_history", []).append(phase)

        elif event == "portal_transaction_finished":
            tx_id = _transaction_id(data)
            if tx_id is None or tx_id not in transactions:
                transaction_violations.append({
                    "transaction_id": tx_id,
                    "reason": "finish_without_transaction",
                })
                continue
            current = transactions[tx_id]
            finish_route = _positive_int(data.get("route_id"))
            if (
                finish_route is not None
                and current.get("route_id") is not None
                and finish_route != current.get("route_id")
            ):
                transaction_violations.append({
                    "transaction_id": tx_id,
                    "reason": "transaction_finish_route_changed",
                })
            current["finished"] = True
            current["finish_route_id"] = finish_route
            current.setdefault("phase_history", []).append("finished")
            # The online node publishes an ``idle`` status after closing the
            # transaction. Keep the last meaningful phase separately so a
            # later arrival event can still be associated with its
            # place_commit proof.
            nested_state = str(_nested_transaction(data).get("state") or "")
            current["state"] = nested_state or "idle"

        elif event in ("portal_place_entered", "portal_place_covered_arrival"):
            place_id = _positive_int(data.get("region_id"))
            tx = _nested_transaction(data)
            tx_state = str(tx.get("state") or "")
            tx_route = _positive_int(tx.get("route_id"))
            tx_id = _transaction_id(data)
            transaction = transactions.get(tx_id) if tx_id is not None else None
            # ``portal_transaction_finished`` is emitted immediately after
            # the online place commit in some runs, while the arrival event
            # is emitted by the map callback just afterwards.  The nested
            # status then says ``idle`` and route_id=0 even though the same
            # transaction already passed through place_commit.  Validate the
            # causal history, rather than requiring the snapshot to still
            # expose the transient phase.
            history = () if transaction is None else tuple(
                transaction.get("phase_history", ())
            )
            committed_route = (
                None if transaction is None
                else _positive_int(transaction.get("route_id"))
            )
            direct_commit = tx_state == "place_commit" and tx_route == route_id
            historical_commit = (
                "place_commit" in history
                and committed_route is not None
                and route_id == committed_route
            )
            if not (direct_commit or historical_commit):
                phantom_place_commits.append({
                    "route_id": route_id,
                    "region_id": place_id,
                    "transaction_id": tx_id,
                    "reason": (
                        "place_arrival_without_place_commit"
                        if transaction is None or "place_commit" not in history
                        else "place_arrival_transaction_route_mismatch"
                    ),
                })
            if place_id is not None:
                places.setdefault(place_id, {"entries": 0})["entries"] += 1
                place_entries.append({
                    "route_id": route_id,
                    "region_id": place_id,
                    "event": event,
                })

        elif event == "portal_hypothesis_destination_bound":
            portal_id = _positive_int(data.get("portal_id"))
            destination = _positive_int(data.get("destination_place_id"))
            if portal_id is not None:
                portal_destinations[portal_id] = destination

        elif event == "portal_hypothesis_failed":
            portal_id = _positive_int(data.get("portal_id"))
            if portal_id is not None:
                failed_portals.add(portal_id)

        elif event == "portal_place_arrival_rejected":
            reason = str(data.get("reason") or "")
            if "stale" in reason or "mismatch" in reason:
                stale_transaction_commits.append({
                    "route_id": route_id,
                    "transaction_id": _positive_int(data.get("transaction_id")),
                    "reason": reason,
                })

        elif event == "frontier_place_suspended":
            place_id = _positive_int(data.get("region_id"))
            if place_id is not None:
                suspended_places[place_id] += 1

    crossing_count = int(action_counts.get("cross_portal", 0))
    place_entry_count = len(place_entries)
    return {
        "route_count": len(route_ids),
        "graph_action_counts": dict(sorted(action_counts.items())),
        "graph_action_reasons": dict(sorted(action_reasons.items())),
        "branch_first_crossings": int(
            action_reasons.get("certified_portal_branch_to_unobserved_place", 0)
        ),
        "illegal_graph_actions": illegal_actions,
        "illegal_graph_action_count": len(illegal_actions),
        "illegal_graph_action_rate": (
            None if not route_ids else round(len(illegal_actions) / float(len(route_ids)), 6)
        ),
        "transaction_violations": transaction_violations,
        "transaction_violation_count": len(transaction_violations),
        "phantom_place_commits": phantom_place_commits,
        "phantom_place_count": len(phantom_place_commits),
        "phantom_place_rate": (
            None if not place_entry_count else round(len(phantom_place_commits) / float(place_entry_count), 6)
        ),
        "stale_transaction_commits": stale_transaction_commits,
        "stale_transaction_commit_count": len(stale_transaction_commits),
        "place_entries": place_entries,
        "place_entry_count": place_entry_count,
        "suspended_place_counts": dict(sorted(suspended_places.items())),
        "selected_portal_counts": dict(sorted(selected_portals.items())),
        "same_edge_reselections": same_edge_reselections,
        "same_edge_reselection_count": len(same_edge_reselections),
        "graph_plan_mismatches": graph_plan_mismatches,
        "graph_plan_mismatch_count": len(graph_plan_mismatches),
        "graph_plan_reconciliations": graph_plan_reconciliations,
        "graph_plan_reconciliation_count": len(graph_plan_reconciliations),
        "route_plan_action_mismatches": route_plan_action_mismatches,
        "route_plan_action_mismatch_count": len(route_plan_action_mismatches),
        "portal_destinations": dict(sorted(portal_destinations.items())),
    }


__all__ = ["replay_graph_events"]
