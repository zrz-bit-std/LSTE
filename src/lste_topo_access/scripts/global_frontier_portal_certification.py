"""Compatibility facade for physical portal certification.

Structural labels alone may split one room around furniture. The implementation
therefore separates route evidence, local raw-free cut validation, and the
public transition queries. Keep this module as the stable import point for
callers outside the certification implementation.
"""

from global_frontier_portal_certification_models import (
    RoutePlaceTransition,
    portal_certification_window,
)
from global_frontier_portal_certification_routes import (
    portal_transition_exit_cell,
    portal_transition_verified_exit_cell,
    route_place_transition_candidates,
)
from global_frontier_portal_certification_transitions import (
    certified_adjacent_place_transition_goals,
    certified_adjacent_place_transitions,
    certified_route_place_hops,
    certified_route_place_transitions,
    first_certified_route_place_transition_goal,
)


__all__ = [
    "RoutePlaceTransition",
    "portal_certification_window",
    "route_place_transition_candidates",
    "portal_transition_exit_cell",
    "portal_transition_verified_exit_cell",
    "certified_route_place_transitions",
    "certified_route_place_hops",
    "first_certified_route_place_transition_goal",
    "certified_adjacent_place_transitions",
    "certified_adjacent_place_transition_goals",
]
