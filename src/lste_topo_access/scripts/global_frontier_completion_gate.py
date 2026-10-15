"""Graph-level completion gate for unknown-environment exploration.

An empty candidate set is a local execution observation.  It is not proof
that the durable Place/Portal/WorkItem graph is complete.  This module keeps
that distinction explicit and replayable: an exploration run may emit a
terminal ``frontier_exhausted`` event only when every durable obligation is
settled (or the graph itself is not ready to make a completion claim).
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class CompletionGate:
    """Pure result of one durable graph-completion audit."""

    complete: bool
    reasons: tuple
    counts: dict


def _place_ids(region_memory):
    if region_memory is None:
        return ()
    regions = getattr(region_memory, "regions", ())
    result = []
    for region in regions or ():
        try:
            place_id = int(region.get("id"))
        except (AttributeError, TypeError, ValueError):
            continue
        if place_id > 0:
            result.append(place_id)
    return tuple(sorted(set(result)))


def _count_for_places(ledger, method_name, place_ids):
    method = getattr(ledger, method_name, None)
    if method is None:
        return 0
    total = 0
    for place_id in place_ids:
        try:
            total += max(0, int(method(place_id)))
        except (TypeError, ValueError):
            continue
    return total


def evaluate_completion(
    *,
    graph_ready,
    region_memory=None,
    work_item_ledger=None,
    portal_probe_ledger=None,
    portal_hypothesis_ledger=None,
    target_observation_work=None,
):
    """Return whether a run is allowed to publish exploration completion.

    The gate has no timer, distance, score or confidence parameter.  It only
    counts durable obligations owned by the current graph.  Route failure and
    a transient detector gap therefore leave the run non-terminal.
    """
    counts = {
        "unresolved_work_items": 0,
        "unresolved_portal_probes": 0,
        "unbound_portal_hypotheses": 0,
        "pending_target_observations": 0,
    }
    reasons = []
    if not graph_ready:
        return CompletionGate(False, ("graph_not_ready",), counts)

    place_ids = _place_ids(region_memory)
    counts["unresolved_work_items"] = _count_for_places(
        work_item_ledger, "unresolved_count", place_ids,
    )
    counts["unresolved_portal_probes"] = _count_for_places(
        portal_probe_ledger, "unresolved_count", place_ids,
    )

    unbound_query = getattr(
        portal_hypothesis_ledger, "unbound_for_source", None,
    )
    if unbound_query is not None:
        for place_id in place_ids:
            try:
                counts["unbound_portal_hypotheses"] += len(
                    tuple(unbound_query(place_id))
                )
            except (TypeError, ValueError):
                continue

    pending_query = getattr(
        target_observation_work, "pending_place_ids", None,
    )
    if pending_query is not None:
        try:
            counts["pending_target_observations"] = len(tuple(pending_query()))
        except (TypeError, ValueError):
            counts["pending_target_observations"] = 0

    reason_by_count = (
        ("unresolved_work_items", "unresolved_work_items"),
        ("unresolved_portal_probes", "unresolved_portal_probes"),
        ("unbound_portal_hypotheses", "unbound_portal_hypotheses"),
        ("pending_target_observations", "pending_target_observations"),
    )
    reasons.extend(
        reason for key, reason in reason_by_count if counts[key] > 0
    )
    return CompletionGate(not reasons, tuple(reasons), counts)


__all__ = ["CompletionGate", "evaluate_completion"]
