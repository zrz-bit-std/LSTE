"""Value objects and map-scale conventions for portal certification."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RoutePlaceTransition:
    """One candidate structural-place crossing on a BFS route.

    ``portal_cell`` identifies the physical doorway throat and
    ``destination_cell`` is the first labelled structural core in the next
    place. Runtime derives a short unlabelled-or-core exit point from their
    route; the core remains durable topology identity, while the opening is
    directional physical evidence.
    """

    source_label: int
    destination_label: int
    source_cell: object
    portal_cell: tuple
    destination_cell: tuple
    throat_cells: tuple


def portal_certification_window(
    resolution, structural_clearance_m, furniture_max_span_m,
):
    """Derive local cut geometry from existing structural-map semantics."""
    resolution = max(1e-6, float(resolution))
    radius = max(1, int(np.ceil(float(structural_clearance_m) / resolution)))
    margin = radius + max(
        radius * 2,
        int(np.ceil(float(furniture_max_span_m) / resolution)),
    )
    return radius, margin
