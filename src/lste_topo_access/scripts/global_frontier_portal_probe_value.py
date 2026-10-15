"""Pure value and selection contract for source-side Portal probes.

Portal probes are information actions, not ordinary frontier goals.  Their
selection therefore has three deliberately separate stages:

1. hard physical constraints remove actions that cannot be executed safely;
2. the action category is chosen by an explicit lexicographic policy;
3. incomparable values remain on a Pareto front instead of being collapsed
   into a weighted scalar score.

This module contains no ROS imports, map access, timers, or mutable ledger
state.  A ROS adapter can construct :class:`PortalProbeCandidate` values from
the current snapshot and consume :func:`select_portal_probe` without changing
the selection semantics.
"""

from dataclasses import dataclass
import math

from global_frontier_action_policy import KNOWN_ACTION_TIERS


# Existing action-tier order is kept as the default contract.  Callers may
# supply a different explicit order for an experiment, but an implicit score
# must never decide between action classes.
DEFAULT_ACTION_CATEGORY_ORDER = tuple(KNOWN_ACTION_TIERS)

# A tuple, rather than a weighted expression, makes the objective contract
# inspectable and prevents accidental introduction of a hidden scalar reward.
OBJECTIVE_DIRECTIONS = (
    ("information_gain", "max"),
    ("task_relevance", "max"),
    ("novelty", "max"),
    ("clearance", "max"),
    ("path_cost", "min"),
    ("risk", "min"),
)


@dataclass(frozen=True)
class PortalProbeValue:
    """Multi-objective value vector for one legal probe candidate.

    The fields are physical or task evidence supplied by the caller.  They
    are not combined into a total score.  ``information_gain``,
    ``task_relevance``, ``novelty`` and ``clearance`` are maximized; ``path_cost``
    and ``risk`` are minimized.
    """

    information_gain: object
    task_relevance: object
    novelty: object
    path_cost: object
    risk: object
    clearance: object


@dataclass(frozen=True)
class PortalProbeConstraints:
    """Facts that must all hold before a probe may enter value selection.

    These are deliberately booleans instead of tunable margins.  The route
    validator, physical Place memory, and probe ledger each own the evidence
    behind one fact.
    """

    route_reachable: bool = False
    collision_free: bool = False
    source_place_current: bool = False
    physical_identity_valid: bool = False
    probe_available: bool = False
    viewpoint_available: bool = False


@dataclass(frozen=True)
class PortalProbeCandidate:
    """One probe action with immutable hard facts and value dimensions."""

    candidate_id: object
    action_category: str
    value: PortalProbeValue
    constraints: PortalProbeConstraints
    # This is only a final deterministic tie-break after category and Pareto
    # filtering.  It is not a value dimension and carries no preference
    # weight.  Typical values are ``(probe_id, viewpoint_x, viewpoint_y)``.
    tie_break_key: tuple = ()


@dataclass(frozen=True)
class CandidateRejection:
    """Audit record explaining why one candidate was not selectable."""

    candidate: object
    reasons: tuple


@dataclass(frozen=True)
class PortalProbeSelection:
    """Result of the pure selection pipeline.

    ``feasible`` contains hard-constraint-valid candidates with a known action
    category.  ``pareto_front`` is computed only within the selected category;
    lower-priority categories must never compete with a higher-priority one.
    """

    selected: object
    action_category: object
    feasible: tuple
    pareto_front: tuple
    rejected: tuple


def _finite(value):
    """Return whether a value is a finite real scalar."""
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _value_violations(value):
    if not isinstance(value, PortalProbeValue):
        return ("value_type",)
    return tuple(
        name
        for name, _direction in OBJECTIVE_DIRECTIONS
        if not _finite(getattr(value, name))
    )


def hard_constraint_violations(candidate):
    """Return named hard-constraint/value violations for one candidate.

    No objective is inspected for magnitude or compared with another
    candidate here.  Non-finite values are rejected because they cannot
    participate in a well-defined Pareto relation.
    """
    if not isinstance(candidate, PortalProbeCandidate):
        return ("candidate_type",)
    constraints = candidate.constraints
    if not isinstance(constraints, PortalProbeConstraints):
        return ("constraints_type",) + _value_violations(candidate.value)
    violations = tuple(
        name
        for name in (
            "route_reachable",
            "collision_free",
            "source_place_current",
            "physical_identity_valid",
            "probe_available",
            "viewpoint_available",
        )
        if not bool(getattr(constraints, name))
    )
    return violations + _value_violations(candidate.value)


def filter_hard_constraints(candidates):
    """Return only candidates that pass every hard constraint."""
    return tuple(
        candidate
        for candidate in candidates or ()
        if not hard_constraint_violations(candidate)
    )


def _objective_not_worse(left, right, name, direction):
    left_value = float(getattr(left.value, name))
    right_value = float(getattr(right.value, name))
    if direction == "max":
        return left_value >= right_value
    if direction == "min":
        return left_value <= right_value
    raise ValueError("unknown objective direction: %r" % direction)


def dominates(left, right):
    """Return whether ``left`` Pareto-dominates ``right``.

    Dominance means no worse on every declared objective and strictly better
    on at least one.  There is no normalization, coefficient, or weighted
    sum in this relation.
    """
    if not isinstance(left, PortalProbeCandidate):
        raise TypeError("left must be a PortalProbeCandidate")
    if not isinstance(right, PortalProbeCandidate):
        raise TypeError("right must be a PortalProbeCandidate")
    no_worse = all(
        _objective_not_worse(left, right, name, direction)
        for name, direction in OBJECTIVE_DIRECTIONS
    )
    strictly_better = any(
        (
            float(getattr(left.value, name)) > float(getattr(right.value, name))
            if direction == "max"
            else float(getattr(left.value, name)) < float(getattr(right.value, name))
        )
        for name, direction in OBJECTIVE_DIRECTIONS
    )
    return no_worse and strictly_better


def pareto_front(candidates):
    """Return the non-dominated candidates, preserving input order."""
    candidates = tuple(candidates or ())
    front = []
    for candidate in candidates:
        if any(dominates(other, candidate) for other in candidates if other is not candidate):
            continue
        front.append(candidate)
    return tuple(front)


def _canonical_category(value):
    return str(value or "").strip().lower()


def _category_order(value):
    """Normalize a sequence or mapping into ``category -> lexicographic rank``."""
    if value is None:
        value = DEFAULT_ACTION_CATEGORY_ORDER
    if hasattr(value, "items"):
        ordered = sorted(
            ((_canonical_category(key), rank) for key, rank in value.items()),
            key=lambda item: (int(item[1]), item[0]),
        )
        return {category: index for index, (category, _rank) in enumerate(ordered)}
    ranks = {}
    for category in value:
        category = _canonical_category(category)
        if category and category not in ranks:
            ranks[category] = len(ranks)
    return ranks


def _sortable(value):
    """Make explicit IDs/keys comparable without relying on mixed Python types."""
    if isinstance(value, (tuple, list)):
        return ("sequence", tuple(_sortable(item) for item in value))
    if value is None:
        return ("none", "")
    if isinstance(value, bool):
        return ("bool", int(value))
    if isinstance(value, (int, float, str)):
        return (type(value).__name__, value)
    return (type(value).__name__, repr(value))


def _candidate_sort_key(candidate):
    return (
        _sortable(candidate.tie_break_key),
        _sortable(candidate.candidate_id),
    )


def select_portal_probe(candidates, action_category_order=None):
    """Select one probe using constraints, category order, and Pareto value.

    The first category in ``action_category_order`` containing a feasible
    candidate wins.  Only candidates in that category are Pareto-compared.
    Among a Pareto front, a stable identity key selects one representative;
    this final operation is a deterministic tie-break, not a scalar utility.
    Unknown categories and hard-constraint failures are returned in
    ``PortalProbeSelection.rejected`` for logging and experiments.
    """
    candidates = tuple(candidates or ())
    ranks = _category_order(action_category_order)
    feasible = []
    rejected = []
    for candidate in candidates:
        reasons = list(hard_constraint_violations(candidate))
        category = (
            None
            if not isinstance(candidate, PortalProbeCandidate)
            else _canonical_category(candidate.action_category)
        )
        if category not in ranks:
            reasons.append("unknown_action_category")
        if reasons:
            rejected.append(CandidateRejection(candidate, tuple(reasons)))
            continue
        feasible.append(candidate)

    if not feasible:
        return PortalProbeSelection(
            selected=None,
            action_category=None,
            feasible=(),
            pareto_front=(),
            rejected=tuple(rejected),
        )

    selected_category = min(
        (_canonical_category(candidate.action_category) for candidate in feasible),
        key=lambda category: ranks[category],
    )
    category_candidates = tuple(
        candidate
        for candidate in feasible
        if _canonical_category(candidate.action_category) == selected_category
    )
    front = pareto_front(category_candidates)
    selected = min(front, key=_candidate_sort_key)
    return PortalProbeSelection(
        selected=selected,
        action_category=selected_category,
        feasible=tuple(feasible),
        pareto_front=front,
        rejected=tuple(rejected),
    )


def select_portal_probe_candidate(candidates, action_category_order=None):
    """Convenience facade returning only the selected candidate or ``None``."""
    return select_portal_probe(candidates, action_category_order).selected


__all__ = [
    "CandidateRejection",
    "DEFAULT_ACTION_CATEGORY_ORDER",
    "OBJECTIVE_DIRECTIONS",
    "PortalProbeCandidate",
    "PortalProbeConstraints",
    "PortalProbeSelection",
    "PortalProbeValue",
    "dominates",
    "filter_hard_constraints",
    "hard_constraint_violations",
    "pareto_front",
    "select_portal_probe",
    "select_portal_probe_candidate",
]
