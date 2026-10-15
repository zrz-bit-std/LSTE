#!/usr/bin/env python3
"""Composition root for active-route execution.

Route execution has five independently editable concerns.  Keep their public
methods behind this stable mixin so the ROS node and its callers retain the
same API while each lifecycle stage remains small enough to inspect locally.
"""

from global_frontier_execution_active_route import (
    GlobalFrontierExecutionActiveRouteMixin,
)
from global_frontier_execution_egress import GlobalFrontierExecutionEgressMixin
from global_frontier_execution_observation import (
    GlobalFrontierExecutionObservationMixin,
)
from global_frontier_execution_prefetch import GlobalFrontierExecutionPrefetchMixin
from global_frontier_execution_resolution import (
    GlobalFrontierExecutionResolutionMixin,
)
from global_frontier_execution_routes import GlobalFrontierExecutionRouteMixin
from global_frontier_execution_watchdogs import GlobalFrontierExecutionWatchdogMixin


class GlobalFrontierExecutionMixin(
    GlobalFrontierExecutionActiveRouteMixin,
    GlobalFrontierExecutionEgressMixin,
    GlobalFrontierExecutionRouteMixin,
    GlobalFrontierExecutionResolutionMixin,
    GlobalFrontierExecutionPrefetchMixin,
    GlobalFrontierExecutionWatchdogMixin,
    GlobalFrontierExecutionObservationMixin,
):
    """Stable public execution API assembled from focused lifecycle stages."""

    pass
