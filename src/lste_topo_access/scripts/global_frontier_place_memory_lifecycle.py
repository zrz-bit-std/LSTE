"""Compatibility facade for persistent frontier-place lifecycle behaviour."""

from global_frontier_place_memory_observation import FrontierRegionObservationMixin
from global_frontier_place_memory_portals import FrontierRegionPortalMixin
from global_frontier_place_memory_state import FrontierRegionStateMixin


class FrontierRegionLifecycleMixin(
    FrontierRegionPortalMixin,
    FrontierRegionObservationMixin,
    FrontierRegionStateMixin,
):
    """Compose portal facts, observation evidence, and state transitions."""

