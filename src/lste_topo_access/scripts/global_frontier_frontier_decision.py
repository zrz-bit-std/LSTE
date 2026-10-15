"""Evidence-gated selection for Place-owned frontier actions.

The original frontier selector compresses every local endpoint into one
weighted score.  That makes an implementation detail (for example a path
weight) decide whether a WorkItem is observed.  The graph method uses this
module instead: hard ownership facts are checked first, then a discrete
action category is chosen, and only incomparable evidence vectors survive
the Pareto reduction.

This module is deliberately ROS-free.  A candidate contains the original
route tuple for compatibility, while the decision contract exposes the
identities and evidence needed for replay and benchmark logs.  The legacy
scalar remains available to the baseline methods and as a diagnostic field;
it is never read by ``select_frontier_action``.
"""

from dataclasses import dataclass
import math


FRONTIER_ACTION_ORDER = (
    "viewpoint_retry",
    "target_direction",
    "adjacent",
    "local",
    "probe",
)


FRONTIER_OBJECTIVES = (
    ("unknown_support", "max"),
    ("work_item_novelty", "max"),
    ("target_relevance", "max"),
    ("clearance", "max"),
    ("path_cost", "min"),
    ("turn_cost", "min"),
    ("risk", "min"),
)


@dataclass(frozen=True)
class FrontierDecisionEvidence:
    """Measured, non-weighted evidence for one legal action."""

    unknown_support: object
    work_item_novelty: object
    target_relevance: object
    clearance: object
    path_cost: object
    turn_cost: object = 0.0
    risk: object = 0.0


@dataclass(frozen=True)
class FrontierActionCandidate:
    """One candidate after route construction and graph ownership checks."""

    candidate_id: object
    action_category: str
    route: object
    region_tier: str = "new"
    place_id: object = None
    work_item_id: object = None
    portal_id: object = None
    map_epoch: object = None
    evidence: FrontierDecisionEvidence = None
    route_valid: bool = True
    place_owned: bool = True
    work_item_unresolved: bool = True
    portal_certified: bool = True
    covered: bool = False


@dataclass(frozen=True)
class FrontierDecisionRejection:
    candidate: object
    reasons: tuple


@dataclass(frozen=True)
class FrontierActionDecision:
    selected: object
    category: object
    feasible: tuple
    pareto_front: tuple
    rejected: tuple
    decision_epoch: object = None


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _identity_present(value):
    if value is None:
        return False
    try:
        return int(value) > 0
    except (TypeError, ValueError):
        return bool(str(value).strip())


def _sortable(value):
    if isinstance(value, (tuple, list)):
        return ("sequence", tuple(_sortable(item) for item in value))
    if value is None:
        return ("none", "")
    if isinstance(value, bool):
        return ("bool", int(value))
    if isinstance(value, (int, float, str)):
        return (type(value).__name__, value)
    return (type(value).__name__, repr(value))


def _candidate_id(route, category):
    """Build an input-order-independent identity from durable route facts."""
    route = tuple(route or ())
    work_item = route[12] if len(route) > 12 else None
    portal_probe = route[15] if len(route) > 15 else None
    gate = route[11] if len(route) > 11 else None
    if work_item is not None:
        try:
            work_item = int(work_item)
        except (TypeError, ValueError):
            work_item = str(work_item)
        return (str(category), "work_item", work_item)
    probe_id = getattr(portal_probe, "probe_id", None)
    if probe_id is not None:
        try:
            probe_id = int(probe_id)
        except (TypeError, ValueError):
            probe_id = str(probe_id)
        return (str(category), "probe", probe_id)
    if gate is not None:
        try:
            return (
                str(category), "gate",
                round(float(gate[0]), 3), round(float(gate[1]), 3),
            )
        except (IndexError, TypeError, ValueError):
            pass
    return (
        str(category),
        "cell",
        route[0] if len(route) > 0 else None,
        route[1] if len(route) > 1 else None,
        round(float(route[2]), 3) if len(route) > 2 else None,
        round(float(route[3]), 3) if len(route) > 3 else None,
    )


def action_category(route, ranking_category=None):
    """Resolve the explicit category carried by a legacy route tuple."""
    if ranking_category in FRONTIER_ACTION_ORDER:
        return ranking_category
    route = tuple(route or ())
    if len(route) > 16 and bool(route[16]):
        return "viewpoint_retry"
    if len(route) > 15 and route[15] is not None:
        return "probe"
    if len(route) > 9:
        try:
            hops = int(route[9])
        except (TypeError, ValueError):
            hops = 0
        if hops >= 1:
            return "adjacent"
    return "local"


def candidate_from_route(
    route, region_tier="new", ranking_category=None, map_epoch=None,
    place_id=None,
):
    """Adapt the existing route tuple without changing its public layout."""
    route = tuple(route or ())
    category = action_category(route, ranking_category)
    information = route[5] if len(route) > 5 else 0.0
    structure = route[6] if len(route) > 6 else 0.0
    path_cost = route[4] if len(route) > 4 else float("inf")
    work_item_id = route[12] if len(route) > 12 else None
    portal_probe = route[15] if len(route) > 15 else None
    portal_id = getattr(portal_probe, "probe_id", None)
    try:
        hops = int(route[9]) if len(route) > 9 and route[9] is not None else 0
    except (TypeError, ValueError):
        hops = 0
    graph_action = route[17] if len(route) > 17 else None
    route_valid = graph_action not in (None, "hold")
    # A candidate has already passed Navfn/costmap admission at this boundary.
    # ``clearance`` is therefore a measured feasibility fact, not a tunable
    # reward. Structure remains separate and only supplies unknown-side
    # support when it is available in the legacy tuple.
    evidence = FrontierDecisionEvidence(
        unknown_support=information,
        work_item_novelty=(
            route[14] if len(route) > 14 and route[14] is not None else 0.0
        ),
        target_relevance=1.0 if category == "target_direction" else 0.0,
        clearance=1.0,
        path_cost=path_cost,
        turn_cost=0.0,
        risk=0.0,
    )
    return FrontierActionCandidate(
        candidate_id=_candidate_id(route, category),
        action_category=category,
        route=route,
        region_tier=str(region_tier or "new"),
        place_id=place_id,
        work_item_id=work_item_id,
        portal_id=portal_id,
        map_epoch=map_epoch,
        evidence=evidence,
        route_valid=route_valid,
        place_owned=(work_item_id is not None or hops >= 1 or category == "probe"),
        work_item_unresolved=(work_item_id is not None or category in ("adjacent", "probe")),
        portal_certified=(hops < 1 or len(route) > 11 and route[11] is not None),
        covered=False,
    )


def decision_violations(candidate):
    """Return hard-contract violations before any evidence comparison."""
    if not isinstance(candidate, FrontierActionCandidate):
        return ("candidate_type",)
    category = str(candidate.action_category or "").strip().lower()
    reasons = [] if category in FRONTIER_ACTION_ORDER else ["unknown_category"]
    if not candidate.route_valid:
        reasons.append("route_not_proven")
    if not candidate.place_owned:
        reasons.append("place_ownership_missing")
    if candidate.covered:
        reasons.append("covered_viewpoint")
    if category in ("local", "target_direction", "viewpoint_retry"):
        if not _identity_present(candidate.work_item_id):
            reasons.append("work_item_missing")
        if not candidate.work_item_unresolved:
            reasons.append("work_item_not_unresolved")
    if category == "adjacent" and not candidate.portal_certified:
        reasons.append("portal_not_certified")
    evidence = candidate.evidence
    if not isinstance(evidence, FrontierDecisionEvidence):
        reasons.append("evidence_type")
    else:
        reasons.extend(
            name for name, _direction in FRONTIER_OBJECTIVES
            if not _finite(getattr(evidence, name))
        )
    return tuple(dict.fromkeys(reasons))


def _not_worse(left, right, name, direction):
    left_value = float(getattr(left.evidence, name))
    right_value = float(getattr(right.evidence, name))
    return left_value >= right_value if direction == "max" else left_value <= right_value


def dominates(left, right):
    """Return whether ``left`` is no worse on every evidence dimension."""
    no_worse = all(
        _not_worse(left, right, name, direction)
        for name, direction in FRONTIER_OBJECTIVES
    )
    strict = any(
        (
            float(getattr(left.evidence, name)) > float(getattr(right.evidence, name))
            if direction == "max"
            else float(getattr(left.evidence, name)) < float(getattr(right.evidence, name))
        )
        for name, direction in FRONTIER_OBJECTIVES
    )
    return no_worse and strict


def pareto_front(candidates):
    candidates = tuple(candidates or ())
    return tuple(
        candidate
        for candidate in candidates
        if not any(
            dominates(other, candidate)
            for other in candidates
            if other is not candidate
        )
    )


def select_frontier_action(
    candidates, allowed_categories=None, allowed_region_tiers=None,
    decision_epoch=None,
):
    """Select one action using category order and Pareto evidence only.

    The stable candidate identity is the final tie-break.  No scalar score,
    coefficient, wall-clock age, or input-list order participates in this
    decision.
    """
    allowed_categories = tuple(
        category for category in (allowed_categories or FRONTIER_ACTION_ORDER)
        if category in FRONTIER_ACTION_ORDER
    )
    allowed_region_tiers = None if allowed_region_tiers is None else set(
        str(value) for value in allowed_region_tiers
    )
    feasible = []
    rejected = []
    for candidate in candidates or ():
        reasons = list(decision_violations(candidate))
        if not isinstance(candidate, FrontierActionCandidate):
            rejected.append(
                FrontierDecisionRejection(candidate, tuple(dict.fromkeys(reasons)))
            )
            continue
        if allowed_categories and candidate.action_category not in allowed_categories:
            reasons.append("action_category_not_allowed")
        if (
            allowed_region_tiers is not None
            and candidate.region_tier not in allowed_region_tiers
        ):
            reasons.append("region_tier_not_allowed")
        if reasons:
            rejected.append(FrontierDecisionRejection(candidate, tuple(dict.fromkeys(reasons))))
        else:
            feasible.append(candidate)
    if not feasible:
        return FrontierActionDecision(
            None, None, tuple(), tuple(), tuple(rejected), decision_epoch,
        )
    category = min(
        (candidate.action_category for candidate in feasible),
        key=FRONTIER_ACTION_ORDER.index,
    )
    category_candidates = tuple(
        candidate for candidate in feasible if candidate.action_category == category
    )
    front = pareto_front(category_candidates)
    selected = min(front, key=lambda candidate: _sortable(candidate.candidate_id))
    return FrontierActionDecision(
        selected, category, tuple(feasible), front, tuple(rejected), decision_epoch,
    )


__all__ = [
    "FRONTIER_ACTION_ORDER",
    "FRONTIER_OBJECTIVES",
    "FrontierActionCandidate",
    "FrontierActionDecision",
    "FrontierDecisionEvidence",
    "FrontierDecisionRejection",
    "action_category",
    "candidate_from_route",
    "decision_violations",
    "dominates",
    "pareto_front",
    "select_frontier_action",
]
