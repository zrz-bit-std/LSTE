"""Small pure policy classifiers used by frontier selection."""

import math

def semantic_hint_is_forward(origin_xy, hint_xy, candidate_xy):
    """Return whether a candidate advances from the robot toward a hint.

    This is only a ranking predicate. The caller still requires its normal
    clearance, map-connectivity, and Navfn checks, and can fall back to an
    ordinary frontier when no forward candidate exists.
    """
    if origin_xy is None or hint_xy is None or candidate_xy is None:
        return False
    try:
        origin_x, origin_y = float(origin_xy[0]), float(origin_xy[1])
        hint_x, hint_y = float(hint_xy[0]), float(hint_xy[1])
        candidate_x, candidate_y = float(candidate_xy[0]), float(candidate_xy[1])
    except (TypeError, ValueError, IndexError):
        return False
    direction_x = hint_x - origin_x
    direction_y = hint_y - origin_y
    if math.hypot(direction_x, direction_y) < 1e-6:
        return False
    return (
        (candidate_x - origin_x) * direction_x
        + (candidate_y - origin_y) * direction_y
    ) > 0.0


def frontier_score_bucket_prefixes(semantic_pursuit):
    """Return score-pool prefixes in target-pursuit preference order.

    The pools use keys such as ``pursuit_structured_best``.  Keeping the
    separator in one helper prevents a semantic-recovery request from turning
    a missing key into an indefinitely stalled navigation session.
    """
    return ("pursuit_", "") if semantic_pursuit else ("",)


def place_action_tier(place_hops, place_graph_ready):
    """Classify a candidate by the place-level action it represents.

    The frontier score decides *which viewpoint* is best only after the
    topological state machine has decided *which place* may be explored next.
    A route that stays in the current structural component is local coverage;
    one crossing is an adjacent-place *candidate*. Once the source has an
    observation, runtime routing converts every cross-place candidate into an
    explicit portal transition before dispatch. Routes crossing more than one
    portal are likewise represented by that first portal action.
    """
    if not place_graph_ready:
        return "unconstrained"
    if place_hops == 0:
        return "local"
    if place_hops == 1:
        return "adjacent"
    return None
