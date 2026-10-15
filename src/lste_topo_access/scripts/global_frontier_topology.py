#!/usr/bin/env python3
"""Compatibility facade for pure online-frontier topology helpers.

Implementation is separated by responsibility:
component labelling, structural place construction, evidence identity, grid
predicates, route transitions, and small selection-policy classifiers.  Keep
imports from this module stable for the frontier node and its tests.
"""

from global_frontier_topology_components import TopologicalFreeSpaceComponents
from global_frontier_topology_evidence import (
    copy_component_evidence,
    same_topology_component,
)
from global_frontier_topology_grid import (
    grid_frontier_observed_from_viewpoint,
    grid_line_is_known_free,
    grid_reachable_without_closed_places,
    grid_route_avoids_closed_places,
    grid_visible_free_footprint,
)
from global_frontier_topology_places import StructuralPlaceMap
from global_frontier_topology_policy import (
    frontier_score_bucket_prefixes,
    place_action_tier,
    semantic_hint_is_forward,
)
from global_frontier_portal_certification import (
    RoutePlaceTransition,
    certified_adjacent_place_transition_goals,
    certified_route_place_hops,
    certified_route_place_transitions,
    first_certified_route_place_transition_goal,
)
from global_frontier_topology_routes import (
    adjacent_place_transition_goals,
    first_route_place_portal,
    first_route_place_transition_goal,
    route_place_hops,
)

__all__ = [
    "TopologicalFreeSpaceComponents",
    "StructuralPlaceMap",
    "copy_component_evidence",
    "same_topology_component",
    "grid_frontier_observed_from_viewpoint",
    "grid_line_is_known_free",
    "grid_reachable_without_closed_places",
    "grid_route_avoids_closed_places",
    "grid_visible_free_footprint",
    "semantic_hint_is_forward",
    "frontier_score_bucket_prefixes",
    "place_action_tier",
    "adjacent_place_transition_goals",
    "first_route_place_portal",
    "first_route_place_transition_goal",
    "route_place_hops",
    "RoutePlaceTransition",
    "certified_adjacent_place_transition_goals",
    "certified_route_place_hops",
    "certified_route_place_transitions",
    "first_certified_route_place_transition_goal",
]
