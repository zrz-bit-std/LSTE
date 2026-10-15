"""Pure ranking helpers for global-frontier candidate selection.

The ROS explorer owns map state and lifecycle counters.  This module only
contains the deterministic ranking data flow so the policy can be read and
tested without navigating the ROS callback code.
"""

import math

from global_frontier_action_policy import KNOWN_ACTION_TIERS, unique_action_tiers
from global_frontier_topology import frontier_score_bucket_prefixes


def make_score_pool():
    """Create one region/action-tier ranking bucket."""
    return {
        "pursuit_structured_best": None,
        "pursuit_structured_score": -float("inf"),
        "pursuit_fallback_best": None,
        "pursuit_fallback_score": -float("inf"),
        "structured_best": None,
        "structured_score": -float("inf"),
        "fallback_best": None,
        "fallback_score": -float("inf"),
    }


def initialise_score_pools(action_tiers):
    """Keep new/revisit decisions independent for every action tier.

    Candidate rehydration happens after the context is built, so a valid
    candidate can carry a canonical action class that was absent from the
    context's initial order.  Build a total pool map for the action contract;
    callers still decide priority by the explicit order they pass to the
    selector.
    """
    action_tiers = unique_action_tiers(
        tuple(action_tiers or ()) + tuple(KNOWN_ACTION_TIERS),
        include_probe=True,
    )
    return {
        action_tier: {
            "new": make_score_pool(),
            "revisit": make_score_pool(),
        }
        for action_tier in action_tiers
    }


def candidate_score(
    information,
    structure,
    score_path_distance,
    candidate_xy,
    heading_penalty,
    semantic_hint,
    structure_weight,
    semantic_hint_weight,
    semantic_hint_max_distance,
):
    """Return the common information/structure/path score for one action."""
    score = (
        information * 0.035
        + structure * structure_weight
        - score_path_distance * 0.12
        - heading_penalty
    )
    if semantic_hint is not None and semantic_hint_weight > 0.0:
        hint_distance = min(
            semantic_hint_max_distance,
            math.hypot(
                candidate_xy[0] - semantic_hint[0],
                candidate_xy[1] - semantic_hint[1],
            ),
        )
        score -= semantic_hint_weight * hint_distance
    return score


def record_scored_candidate(
    pools,
    action_tier,
    candidate,
    pool_name,
    pursuit_candidate,
    route_steps,
    minimum_steps,
    selection_tier,
    min_structure_cells,
):
    """Store a candidate in strict priority buckets without changing rank."""
    pool = pools[action_tier][pool_name]
    score = candidate[7]
    structure = candidate[6]
    if route_steps < minimum_steps:
        # A nearby observation point is a recovery-only last resort. It must
        # not make ordinary coverage look exhausted.
        if selection_tier == "navfn_observation_recovery":
            if score > pool["fallback_score"]:
                pool["fallback_score"] = score
                pool["fallback_best"] = candidate
            if pursuit_candidate and score > pool["pursuit_fallback_score"]:
                pool["pursuit_fallback_score"] = score
                pool["pursuit_fallback_best"] = candidate
        return
    if score > pool["fallback_score"]:
        pool["fallback_score"] = score
        pool["fallback_best"] = candidate
    if pursuit_candidate and score > pool["pursuit_fallback_score"]:
        pool["pursuit_fallback_score"] = score
        pool["pursuit_fallback_best"] = candidate
    if structure >= min_structure_cells and score > pool["structured_score"]:
        pool["structured_score"] = score
        pool["structured_best"] = candidate
    if (
        pursuit_candidate
        and structure >= min_structure_cells
        and score > pool["pursuit_structured_score"]
    ):
        pool["pursuit_structured_score"] = score
        pool["pursuit_structured_best"] = candidate


def select_score_pool_candidate(
    pools, action_tiers, allowed_region_tiers, semantic_pursuit
):
    """Return the highest-priority candidate and its region tier."""
    for action_tier in action_tiers:
        for pool_name in ("new", "revisit"):
            if (
                allowed_region_tiers is not None
                and pool_name not in allowed_region_tiers
            ):
                continue
            pool = pools[action_tier][pool_name]
            for prefix in frontier_score_bucket_prefixes(semantic_pursuit):
                structured = pool[prefix + "structured_best"]
                if structured is not None:
                    return structured, pool_name
                fallback = pool[prefix + "fallback_best"]
                if fallback is not None:
                    return fallback, pool_name
    return None, None
