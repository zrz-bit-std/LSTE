#!/usr/bin/env python3
"""Composition root for global-frontier route command handling.

Keeping this public module stable lets the ROS node retain one inheritance
point while command resolution and ROS publication remain independently
readable and editable.
"""

from global_frontier_route_command_publication import (
    GlobalFrontierRouteCommandPublicationMixin,
)
from global_frontier_route_command_resolution import (
    GlobalFrontierRouteCommandResolutionMixin,
)


class GlobalFrontierRouteCommandMixin(
    GlobalFrontierRouteCommandPublicationMixin,
    GlobalFrontierRouteCommandResolutionMixin,
):
    """Public route-command API composed from resolution and publication."""

    pass
