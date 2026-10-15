"""Canonical action classes for the place/portal exploration policy.

The planner has two different kinds of state: a semantic action class (what
information operation is allowed next) and a route kind (how that operation
is executed).  Keeping action classes in one small contract prevents a
candidate from introducing a new string that the scorer or selector does not
know how to represent.
"""


ACTION_VIEWPOINT_RETRY = "viewpoint_retry"
ACTION_TARGET_DIRECTION = "target_direction"
ACTION_ADJACENT = "adjacent"
ACTION_PROBE = "probe"
ACTION_LOCAL = "local"
ACTION_UNCONSTRAINED = "unconstrained"

KNOWN_ACTION_TIERS = (
    ACTION_VIEWPOINT_RETRY,
    ACTION_TARGET_DIRECTION,
    ACTION_ADJACENT,
    ACTION_PROBE,
    ACTION_LOCAL,
    ACTION_UNCONSTRAINED,
)


def canonical_action_tier(value):
    """Return a stable action name, or ``None`` for malformed input."""
    value = str(value or "").strip().lower()
    return value if value in KNOWN_ACTION_TIERS else None


def unique_action_tiers(values, include_probe=False):
    """Preserve policy order while making the probe class representable.

    A source-side probe can be discovered while classifying a candidate, even
    when the current place had not yet produced a normal observation anchor.
    It therefore cannot be inferred solely from the context tuple.  Adding it
    to the contract here keeps that candidate auditable and prevents a hidden
    scoring-pool failure.
    """
    ordered = []
    for value in values or ():
        tier = canonical_action_tier(value)
        if tier is not None and tier not in ordered:
            ordered.append(tier)
    if include_probe and ACTION_PROBE not in ordered:
        ordered.append(ACTION_PROBE)
    return tuple(ordered)

