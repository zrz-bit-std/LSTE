"""Source-side doorway observation probes.

A probe is intentionally not a portal transition. It recognizes an unknown
opening bounded by long structural walls, moves only to the already-safe
frontier approach, and lets the next scan decide whether a certified directed
portal exists. This is the missing bridge between local room observation and
the stricter signed-depth crossing contract.
"""

from dataclasses import dataclass
import math

import numpy as np

from global_frontier_portal_certification_wall import (
    structural_wall_opening_for_normal,
)


@dataclass(frozen=True)
class PortalObservationProbe:
    """Evidence for a source-side observation action at one wall opening."""

    opening_cell: tuple
    normal: tuple
    # Durable physical identity assigned by PortalProbeLedger.  ``None`` is
    # retained for compatibility with geometry-only callers and unit fixtures.
    probe_id: object = None
    # The same physical probe has two information phases.  Keeping the phase
    # on the immutable candidate prevents a destination observation from being
    # downgraded to a generic frontier route while it crosses the ROS boundary.
    phase: str = "source"
    # ``frontier`` is the historical source.  ``structural_boundary`` means
    # the opening was compiled from the structural map even though its far
    # side is already known free and therefore no longer a frontier.
    observation_source: str = "frontier"


def portal_verification_viewpoints(gate_xy, normal_xy, depth):
    """Return a deterministic active-view ladder for one physical doorway.

    The first point is on the hypothesized destination side. If that side is
    still unknown to Navfn, the two following points remain on the source side
    but move tangentially around the same gate. They provide independent
    observations without entering unknown cells or creating a new doorway
    identity. ``depth`` comes from the existing crossing contract; this helper
    introduces no geometry knob of its own.
    """
    try:
        gate_x, gate_y = float(gate_xy[0]), float(gate_xy[1])
        normal_x, normal_y = float(normal_xy[0]), float(normal_xy[1])
        depth = float(depth)
    except (IndexError, TypeError, ValueError):
        return ()
    length = math.hypot(normal_x, normal_y)
    if length <= 1e-9 or not math.isfinite(depth) or depth <= 0.0:
        return ()
    normal_x, normal_y = normal_x / length, normal_y / length
    tangent_x, tangent_y = -normal_y, normal_x
    destination = (
        gate_x + depth * normal_x,
        gate_y + depth * normal_y,
    )
    source_lateral_left = (
        gate_x - depth * normal_x + depth * tangent_x,
        gate_y - depth * normal_y + depth * tangent_y,
    )
    source_lateral_right = (
        gate_x - depth * normal_x - depth * tangent_x,
        gate_y - depth * normal_y - depth * tangent_y,
    )
    return (
        ("destination_normal", destination),
        ("source_lateral_left", source_lateral_left),
        ("source_lateral_right", source_lateral_right),
    )


def source_verification_viewpoints(gate_xy, normal_xy, depth):
    """Return source-side viewpoints for a failed source probe.

    The first stance is the normal source-side observation. The two lateral
    stances reuse the same doorway geometry and provide independent views when
    a controller cannot reach the first stance; no new retry distance is added.
    """
    try:
        gate_x, gate_y = float(gate_xy[0]), float(gate_xy[1])
        normal_x, normal_y = float(normal_xy[0]), float(normal_xy[1])
        depth = float(depth)
    except (IndexError, TypeError, ValueError):
        return ()
    length = math.hypot(normal_x, normal_y)
    if length <= 1e-9 or not math.isfinite(depth) or depth <= 0.0:
        return ()
    normal_x, normal_y = normal_x / length, normal_y / length
    tangent_x, tangent_y = -normal_y, normal_x
    source = (
        gate_x - depth * normal_x,
        gate_y - depth * normal_y,
    )
    left = (
        source[0] + depth * tangent_x,
        source[1] + depth * tangent_y,
    )
    right = (
        source[0] - depth * tangent_x,
        source[1] - depth * tangent_y,
    )
    return (
        ("source_normal", source),
        ("source_lateral_left", left),
        ("source_lateral_right", right),
    )


def portal_observation_probe(
    unknown, structural_occupied, frontier_cell, *, support_radius,
    minimum_wall_span_cells, wall_search_radius=None,
):
    """Return a wall-bounded unknown opening suitable for an observation probe.

    The selected ``frontier_cell`` is known free and already route-validated
    by the caller. The immediate far-side cell must remain unknown. Known free
    space has already been observed and known occupied space is a wall, so
    neither can justify this information-gathering action.
    """
    if not isinstance(unknown, np.ndarray) or unknown.ndim != 2:
        return None
    row, col = int(frontier_cell[0]), int(frontier_cell[1])
    rows, cols = unknown.shape
    if not (0 <= row < rows and 0 <= col < cols):
        return None
    search_radius = max(
        int(support_radius),
        int(support_radius)
        if wall_search_radius is None else int(wall_search_radius),
    )
    for normal in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        next_row, next_col = row + normal[0], col + normal[1]
        if not (
            0 <= next_row < rows
            and 0 <= next_col < cols
            and unknown[next_row, next_col]
        ):
            continue
        if structural_wall_opening_for_normal(
            structural_occupied,
            (row, col),
            normal,
            search_radius,
            minimum_wall_span_cells,
        ):
            return PortalObservationProbe((row, col), normal)
    return None
