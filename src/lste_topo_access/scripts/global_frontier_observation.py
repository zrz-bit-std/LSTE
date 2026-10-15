"""Public composition point for frontier observation behavior.

The coordinator inherits this one mixin.  Individual responsibilities live in
small modules so topology association, coverage accounting, and the physical
departure transaction can evolve independently.
"""

from global_frontier_observation_coverage import (
    GlobalFrontierObservationCoverageMixin,
)
from global_frontier_observation_departure import (
    GlobalFrontierObservationDepartureMixin,
)
from global_frontier_observation_topology import (
    GlobalFrontierObservationTopologyMixin,
)


class GlobalFrontierObservationMixin(
    GlobalFrontierObservationTopologyMixin,
    GlobalFrontierObservationCoverageMixin,
    GlobalFrontierObservationDepartureMixin,
):
    """Compose map evidence, observation state, and departure behavior."""

