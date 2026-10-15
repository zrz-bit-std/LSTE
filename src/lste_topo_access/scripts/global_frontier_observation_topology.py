"""Composition point for map-derived frontier observation state.

Keep this module deliberately small.  The three mixins below have separate
ownership: transient topology evidence, persistent place/target state, and
visibility certificates for completed viewpoints.
"""

from global_frontier_observation_components import (
    GlobalFrontierObservationComponentsMixin,
)
from global_frontier_observation_place_state import (
    GlobalFrontierObservationPlaceStateMixin,
)
from global_frontier_observation_viewpoints import (
    GlobalFrontierObservationViewpointMixin,
)


class GlobalFrontierObservationTopologyMixin(
    GlobalFrontierObservationComponentsMixin,
    GlobalFrontierObservationPlaceStateMixin,
    GlobalFrontierObservationViewpointMixin,
):
    """Compose all topology-derived observation behavior."""

