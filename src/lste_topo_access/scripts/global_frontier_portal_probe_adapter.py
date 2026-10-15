"""Adapter from frontier route tuples to the pure PortalProbeValue contract.

The planner still receives a legacy positional candidate tuple at its ROS
boundary.  This module keeps that compatibility detail in one place and
hands the value selector immutable, named facts.  It deliberately performs no
place admission, portal certification, or route mutation.
"""

import math

from global_frontier_action_policy import (
    ACTION_PROBE,
    ACTION_TARGET_DIRECTION,
    ACTION_VIEWPOINT_RETRY,
)
from global_frontier_portal_probe_value import (
    PortalProbeCandidate,
    PortalProbeConstraints,
    PortalProbeValue,
    select_portal_probe,
)


def _finite_scalar(value, default=0.0):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return float(default)
    return value if math.isfinite(value) else float(default)


def _physical_xy(request, x, y):
    projector = getattr(request, "map_to_physical_xy", None)
    if projector is None:
        return float(x), float(y)
    try:
        projected = projector(float(x), float(y))
        if projected is None:
            return None
        projected = float(projected[0]), float(projected[1])
    except (IndexError, TypeError, ValueError):
        return None
    return projected if all(math.isfinite(item) for item in projected) else None


def _probe_category(route, requested_category):
    """Keep retry/target ordering while restricting the action to probes."""
    if route[16]:
        return ACTION_VIEWPOINT_RETRY
    if requested_category == ACTION_TARGET_DIRECTION:
        return ACTION_TARGET_DIRECTION
    return ACTION_PROBE


def make_probe_candidate(
    route,
    pool_name,
    requested_category,
    request,
    context,
    ledger=None,
    route_rejected=False,
):
    """Translate one scored route tuple into an immutable value candidate."""
    probe = route[15] if len(route) > 15 else None
    if probe is None:
        return None
    probe_id = getattr(probe, "probe_id", None)
    try:
        probe_id = int(probe_id)
    except (TypeError, ValueError):
        probe_id = None
    viewpoint = _physical_xy(request, route[2], route[3])
    source_place_id = getattr(context, "source_place_id", None)
    route_reachable = not route_rejected and math.isfinite(
        _finite_scalar(route[4], default=float("nan"))
    )
    ledger_available = True
    viewpoint_available = viewpoint is not None
    if ledger is not None and probe_id is not None:
        available = getattr(ledger, "available", None)
        destination_available = getattr(ledger, "destination_available", None)
        if available is not None:
            ledger_available = bool(available(probe_id))
        if (
            not ledger_available
            and callable(destination_available)
        ):
            ledger_available = bool(
                destination_available(probe_id, viewpoint)
            )
        check_viewpoint = getattr(ledger, "viewpoint_available", None)
        if check_viewpoint is not None:
            viewpoint_available = viewpoint_available and bool(
                check_viewpoint(probe_id, viewpoint)
            )
    action_category = _probe_category(route, requested_category)
    candidate_id = (
        probe_id
        if probe_id is not None
        else (int(route[0]), int(route[1]), float(route[2]), float(route[3]))
    )
    tie_break = (
        candidate_id,
        _finite_scalar(route[2]),
        _finite_scalar(route[3]),
    )
    # ``information`` is the current unknown-side support.  Work-item support
    # is a structural novelty proxy; both are raw evidence counts, not a
    # calibrated reward.  Legal candidates have already passed costmap
    # validation, so clearance/risk remain explicit neutral facts here.
    probe_value = PortalProbeValue(
        information_gain=_finite_scalar(route[5]),
        task_relevance=(
            1.0 if requested_category == ACTION_TARGET_DIRECTION else 0.0
        ),
        novelty=_finite_scalar(route[14]),
        path_cost=_finite_scalar(route[4], default=float("nan")),
        risk=0.0,
        clearance=1.0,
    )
    constraints = PortalProbeConstraints(
        route_reachable=route_reachable,
        collision_free=True,
        source_place_current=source_place_id is not None,
        physical_identity_valid=probe_id is not None and probe_id > 0,
        probe_available=ledger_available,
        viewpoint_available=viewpoint_available,
    )
    return PortalProbeCandidate(
        candidate_id=candidate_id,
        action_category=action_category,
        value=probe_value,
        constraints=constraints,
        tie_break_key=tie_break,
    )


def select_probe_route(
    records,
    request,
    context,
    ledger=None,
    allowed_region_tiers=None,
    route_rejected=None,
):
    """Select a route and return ``(route, pure_selection_result)``.

    ``records`` contains ``(route_tuple, pool_name, action_category)`` entries
    captured before score-pool compression.  A callback may mark a viewpoint
    rejected by the current Navfn validation pass; it is a fact supplied by
    the caller, not a selector threshold.
    """
    candidates = []
    route_by_identity = {}
    for route, pool_name, requested_category in records or ():
        if (
            allowed_region_tiers is not None
            and pool_name not in allowed_region_tiers
        ):
            continue
        rejected = False if route_rejected is None else bool(
            route_rejected(float(route[2]), float(route[3]))
        )
        candidate = make_probe_candidate(
            route,
            pool_name,
            requested_category,
            request,
            context,
            ledger=ledger,
            route_rejected=rejected,
        )
        if candidate is None:
            continue
        candidates.append(candidate)
        route_by_identity[id(candidate)] = route
    result = select_portal_probe(candidates)
    selected_route = (
        None
        if result.selected is None
        else route_by_identity.get(id(result.selected))
    )
    return selected_route, result


def selection_report(result):
    """Return JSON-safe diagnostics for a planning status event."""
    selected = result.selected
    return {
        "action_category": result.action_category,
        "candidate_count": len(result.feasible) + len(result.rejected),
        "feasible_count": len(result.feasible),
        "pareto_front_count": len(result.pareto_front),
        "selected_candidate_id": (
            None if selected is None else selected.candidate_id
        ),
        "rejected": [
            {
                "candidate_id": rejection.candidate.candidate_id,
                "reasons": list(rejection.reasons),
            }
            for rejection in result.rejected
        ],
    }


__all__ = [
    "make_probe_candidate",
    "select_probe_route",
    "selection_report",
]
