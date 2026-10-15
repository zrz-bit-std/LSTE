#!/usr/bin/env python3
"""Composition point for online-frontier route planning.

The public planning mixin stays stable while implementation is separated into
costmap connectivity, Navfn validation, successor-cache lifecycle, and pure
route geometry.
"""

from global_frontier_planning_costmap import GlobalFrontierPlanningCostmapMixin
from global_frontier_planning_navfn import GlobalFrontierPlanningNavfnMixin
from global_frontier_planning_prefetch import GlobalFrontierPlanningPrefetchMixin
from global_frontier_planning_routes import GlobalFrontierPlanningRouteMixin


class GlobalFrontierPlanningMixin(
    GlobalFrontierPlanningCostmapMixin,
    GlobalFrontierPlanningNavfnMixin,
    GlobalFrontierPlanningPrefetchMixin,
    GlobalFrontierPlanningRouteMixin,
):
    """Compose the planning operations used by the frontier coordinator."""

