#!/usr/bin/env python3
"""Persistent map-derived place memory for online frontier exploration.

The public object owns the stored state.  Matching, queries, and lifecycle
transitions are separate mixins so a state-machine change is local and does
not obscure the topology-association rules that protect against room re-entry.
"""

import collections

from global_frontier_place_memory_association import FrontierRegionAssociationMixin
from global_frontier_place_memory_lifecycle import FrontierRegionLifecycleMixin
from global_frontier_place_memory_queries import FrontierRegionQueryMixin
from global_frontier_place_states import (
    PLACE_DORMANT,
    PLACE_OPEN,
    PLACE_READY_TO_EXIT,
    PLACE_SUSPENDED,
)


class FrontierRegionMemory(
    FrontierRegionAssociationMixin,
    FrontierRegionQueryMixin,
    FrontierRegionLifecycleMixin,
):
    """Store durable observation-place state for one exploration invocation."""

    def __init__(
        self,
        radius,
        information_delta,
        stagnation_timeout,
        failure_limit,
        limit,
    ):
        self.radius = max(0.2, float(radius))
        self.information_delta = max(1.0, float(information_delta))
        self.stagnation_timeout = max(1.0, float(stagnation_timeout))
        self.failure_limit = max(1, int(failure_limit))
        self.limit = max(8, int(limit))
        # Component labels are exact within one planning snapshot. This
        # conservative bridge only associates successive snapshots.
        self.component_match_distance = min(self.radius, 2.0)
        self.component_min_cells = 12
        # Wheel odometry is the physical identity layer.  It is intentionally
        # the same scale as place association, while current-map components
        # remain the wall-aware guard against merging adjacent rooms.
        self.physical_region_match_radius = self.radius
        # Entry gates are matched at the same physical scale as component
        # association. This is derived state, not a new navigation parameter.
        self.portal_entry_match_radius = max(
            0.30, min(self.radius, self.component_match_distance)
        )
        self.portal_entry_merge_radius = max(
            0.25, self.portal_entry_match_radius / 3.0
        )
        self._next_id = 1
        self.regions = collections.deque(maxlen=self.limit)


__all__ = [
    "FrontierRegionMemory",
    "PLACE_OPEN",
    "PLACE_READY_TO_EXIT",
    "PLACE_DORMANT",
    "PLACE_SUSPENDED",
]
