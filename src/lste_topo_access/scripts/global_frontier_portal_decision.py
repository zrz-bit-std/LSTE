"""Task-conditioned, evidence-gated selection of one Portal transition.

The frontier scorer is useful for choosing a viewpoint *inside* a Place, but
it should not decide which physical doorway the robot crosses by adding more
weights to the same scalar score.  This module treats a Portal transition as a
graph action with two layers:

1. hard legality is checked by the caller (route, crossing certificate and
   Place ownership); and
2. the remaining actions are ordered by an explicit information policy and
   Pareto-pruned on task relevance, graph novelty, expected unknown support,
   clearance, path cost and risk.

No ROS, map, timer, or controller state is imported here.  The result is
therefore replayable and can be compared with the historical score selector.
"""

from dataclasses import dataclass
import math


# A target-bearing doorway is the most direct information action.  A doorway
# to a never-observed Place is next; among equally novel actions, prefer the
# one exposing more unknown space.  Covered transit is retained only as a
# graph path operation and cannot outrank a new information opportunity.
PORTAL_CATEGORY_ORDER = (
    "target_relevant",
    "new_place",
    "information_gain",
    "covered_transit",
)

PORTAL_OBJECTIVES = (
    ("task_relevance", "max"),
    ("novelty", "max"),
    ("information_gain", "max"),
    ("clearance", "max"),
    ("path_cost", "min"),
    ("risk", "min"),
)


def portal_action_category(*, task_relevant, destination_unobserved, information_gain):
    """Map graph facts to a discrete information-action class.

    The distinction is deliberately categorical: a task-relevant Portal is a
    different action from merely finding a short path, and a covered transit
    edge is never promoted by a large geometric score.
    """
    if bool(task_relevant):
        return "target_relevant"
    if bool(destination_unobserved):
        return "new_place"
    try:
        has_unknown_support = float(information_gain) > 0.0
    except (TypeError, ValueError):
        has_unknown_support = False
    return "information_gain" if has_unknown_support else "covered_transit"


@dataclass(frozen=True)
class PortalDecisionEvidence:
    """Evidence vector for one already-certified Portal action."""

    task_relevance: object
    novelty: object
    information_gain: object
    clearance: object
    path_cost: object
    risk: object


@dataclass(frozen=True)
class PortalDecisionCandidate:
    """A legal graph action and its immutable evidence."""

    candidate_id: object
    category: str
    evidence: PortalDecisionEvidence
    tie_break_key: tuple = ()


@dataclass(frozen=True)
class PortalDecisionRejection:
    candidate: object
    reasons: tuple


@dataclass(frozen=True)
class PortalDecision:
    selected: object
    category: object
    feasible: tuple
    pareto_front: tuple
    rejected: tuple


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


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


def decision_violations(candidate):
    """Return structural/value violations without comparing candidates."""
    if not isinstance(candidate, PortalDecisionCandidate):
        return ("candidate_type",)
    category = str(candidate.category or "").strip().lower()
    reasons = [] if category in PORTAL_CATEGORY_ORDER else ["unknown_category"]
    evidence = candidate.evidence
    if not isinstance(evidence, PortalDecisionEvidence):
        return tuple(reasons) + ("evidence_type",)
    reasons.extend(
        name for name, _direction in PORTAL_OBJECTIVES
        if not _finite(getattr(evidence, name))
    )
    return tuple(reasons)


def _not_worse(left, right, name, direction):
    left_value = float(getattr(left.evidence, name))
    right_value = float(getattr(right.evidence, name))
    return left_value >= right_value if direction == "max" else left_value <= right_value


def dominates(left, right):
    """Return whether one candidate is no worse on every objective."""
    if not isinstance(left, PortalDecisionCandidate):
        raise TypeError("left must be a PortalDecisionCandidate")
    if not isinstance(right, PortalDecisionCandidate):
        raise TypeError("right must be a PortalDecisionCandidate")
    no_worse = all(
        _not_worse(left, right, name, direction)
        for name, direction in PORTAL_OBJECTIVES
    )
    strict = any(
        (
            float(getattr(left.evidence, name)) > float(getattr(right.evidence, name))
            if direction == "max"
            else float(getattr(left.evidence, name)) < float(getattr(right.evidence, name))
        )
        for name, direction in PORTAL_OBJECTIVES
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


def select_portal_transition(candidates):
    """Select one action without a weighted reward or tuning coefficient."""
    feasible = []
    rejected = []
    for candidate in candidates or ():
        reasons = decision_violations(candidate)
        if reasons:
            rejected.append(PortalDecisionRejection(candidate, reasons))
        else:
            feasible.append(candidate)
    if not feasible:
        return PortalDecision(None, None, (), (), tuple(rejected))

    category = min(
        (str(candidate.category).strip().lower() for candidate in feasible),
        key=PORTAL_CATEGORY_ORDER.index,
    )
    category_candidates = tuple(
        candidate
        for candidate in feasible
        if str(candidate.category).strip().lower() == category
    )
    front = pareto_front(category_candidates)
    selected = min(
        front,
        key=lambda candidate: (
            _sortable(candidate.tie_break_key),
            _sortable(candidate.candidate_id),
        ),
    )
    return PortalDecision(
        selected=selected,
        category=category,
        feasible=tuple(feasible),
        pareto_front=front,
        rejected=tuple(rejected),
    )


__all__ = [
    "PORTAL_CATEGORY_ORDER",
    "PORTAL_OBJECTIVES",
    "PortalDecision",
    "PortalDecisionCandidate",
    "PortalDecisionEvidence",
    "PortalDecisionRejection",
    "portal_action_category",
    "decision_violations",
    "dominates",
    "pareto_front",
    "select_portal_transition",
]
