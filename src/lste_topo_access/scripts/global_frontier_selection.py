#!/usr/bin/env python3
"""Compose the focused stages of global-frontier selection."""

from global_frontier_candidate_routing import GlobalFrontierCandidateRoutingMixin
from global_frontier_candidate_work_items import GlobalFrontierCandidateWorkItemMixin
from global_frontier_candidate_scoring import GlobalFrontierCandidateScoringMixin
from global_frontier_portal_selection import GlobalFrontierPortalSelectionMixin
from global_frontier_portal_egress import GlobalFrontierPortalEgressMixin
from global_frontier_selection_context import GlobalFrontierSelectionContextMixin
from global_frontier_selection_planner import GlobalFrontierSelectionPlannerMixin
from global_frontier_selection_validation import GlobalFrontierSelectionValidationMixin
from global_frontier_graph_route_adapter import GlobalFrontierGraphRouteAdapterMixin


class GlobalFrontierSelectionMixin(
    GlobalFrontierSelectionValidationMixin,
    GlobalFrontierGraphRouteAdapterMixin,
    GlobalFrontierSelectionPlannerMixin,
    GlobalFrontierSelectionContextMixin,
    GlobalFrontierCandidateScoringMixin,
    GlobalFrontierCandidateWorkItemMixin,
    GlobalFrontierCandidateRoutingMixin,
    GlobalFrontierPortalSelectionMixin,
    GlobalFrontierPortalEgressMixin,
):
    """Coordinate context, route construction, scoring, and validation."""
