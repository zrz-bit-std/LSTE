"""Extract source-side architectural boundary evidence from one map snapshot.

Frontier cells are a projection of unknown space.  A doorway can stop being a
frontier as soon as a lidar ray observes the free space on its far side, even
though the robot has not yet inspected that doorway as a graph transition.
This module detects that missing intermediate fact from the independent
structural occupancy layer.

The result is deliberately weaker than a Portal certificate.  It is a
directional observation opportunity that must still be routed to a source-side
viewpoint and promoted by the existing PortalProbe lifecycle.  No ROS state,
score, timeout, or motion parameter is involved here.
"""

from dataclasses import dataclass

import numpy as np

from global_frontier_portal_certification_wall import (
    structural_wall_opening_for_normal,
)


_CARDINAL = ((1, 0), (-1, 0), (0, 1), (0, -1))


@dataclass(frozen=True)
class StructuralBoundary:
    """One directed, source-reachable opening hypothesis."""

    opening_cell: tuple
    normal: tuple
    source_cell: tuple
    destination_cell: tuple


def _in_bounds(shape, cell):
    return (
        0 <= int(cell[0]) < int(shape[0])
        and 0 <= int(cell[1]) < int(shape[1])
    )


def _shift(mask, delta_row, delta_col):
    """Sample ``mask`` at a positive grid offset with zero outside bounds."""
    result = np.zeros_like(mask, dtype=bool)
    rows, cols = mask.shape
    source_row_start = max(0, int(delta_row))
    source_row_stop = min(rows, rows + int(delta_row))
    source_col_start = max(0, int(delta_col))
    source_col_stop = min(cols, cols + int(delta_col))
    target_row_start = max(0, -int(delta_row))
    target_row_stop = min(rows, rows - int(delta_row))
    target_col_start = max(0, -int(delta_col))
    target_col_stop = min(cols, cols - int(delta_col))
    if source_row_start >= source_row_stop or source_col_start >= source_col_stop:
        return result
    result[
        target_row_start:target_row_stop,
        target_col_start:target_col_stop,
    ] = mask[
        source_row_start:source_row_stop,
        source_col_start:source_col_stop,
    ]
    return result


def _nearest_label(labels, cell, direction, maximum_distance):
    """Return the first structural label along one side of an opening."""
    if labels is None:
        return 0
    row, col = int(cell[0]), int(cell[1])
    delta_row, delta_col = int(direction[0]), int(direction[1])
    for distance in range(max(0, int(maximum_distance)) + 1):
        candidate = row + distance * delta_row, col + distance * delta_col
        if not _in_bounds(labels.shape, candidate):
            break
        label = int(labels[candidate])
        if label > 0:
            return label
    return 0


def _orientation_is_source_side(
    labels, steps, source_label, opening, normal, source, destination,
    support_radius,
):
    """Keep the direction that leaves the current Place through the opening.

    Structural labels are preferred when available.  During a partial SLAM
    update the throat or the far-side core may still be unlabelled, so the
    robot-rooted distance field is the fallback orientation witness.  This
    avoids generating the reverse direction as a second source obligation
    while never requiring a fabricated destination label.
    """
    try:
        source_label = int(source_label)
    except (TypeError, ValueError):
        source_label = 0
    tangent_limit = max(1, int(support_radius) * 2 + 1)
    negative = (-int(normal[0]), -int(normal[1]))
    positive = (int(normal[0]), int(normal[1]))
    source_side_label = _nearest_label(
        labels, source, negative, tangent_limit,
    )
    destination_side_label = _nearest_label(
        labels, destination, positive, tangent_limit,
    )
    if source_label > 0:
        if source_side_label == source_label and destination_side_label != source_label:
            return True
        if destination_side_label == source_label and source_side_label != source_label:
            return False

    if steps is None:
        return True
    if labels is not None and steps.shape != labels.shape:
        return True
    try:
        source_steps = int(steps[source])
        destination_steps = int(steps[destination])
    except (IndexError, TypeError, ValueError):
        return False
    if source_steps < 0:
        return False
    if destination_steps < 0:
        # A probe is source-side; an unobserved or low-clearance far side is
        # still valid evidence and does not need to be route-reachable yet.
        return True
    if source_steps != destination_steps:
        return source_steps < destination_steps
    # A symmetric distance tie contains no orientation information.  Use a
    # stable cardinal ordering; the physical ledger will merge repeated map
    # projections of the selected direction.
    return tuple(normal) in ((0, 1), (1, 0))


def _deduplicate(candidates, support_radius):
    """Keep one representative for each contiguous directed opening run."""
    selected = []
    separation = max(1, int(support_radius))
    for candidate in candidates:
        same_run = False
        for prior in selected:
            if prior.normal != candidate.normal:
                continue
            distance = (
                abs(int(prior.opening_cell[0]) - int(candidate.opening_cell[0]))
                + abs(int(prior.opening_cell[1]) - int(candidate.opening_cell[1]))
            )
            if distance <= separation:
                same_run = True
                break
        if not same_run:
            selected.append(candidate)
    return tuple(selected)


def structural_boundary_candidates(
    known_free,
    structural_occupied,
    steps,
    *,
    unknown=None,
    labels=None,
    source_label=None,
    support_radius,
    minimum_wall_span_cells,
    wall_search_radius=None,
):
    """Return directed wall openings reachable from the current Place.

    ``known_free`` is used instead of ``unknown`` on purpose.  The function
    handles the case where both sides of a doorway are already visible but no
    current frontier cell preserves the doorway lineage.  The source-side
    route remains the only executable part; the far side is evidence, not a
    crossing permission.
    """
    if not isinstance(known_free, np.ndarray) or known_free.ndim != 2:
        return ()
    if not isinstance(structural_occupied, np.ndarray):
        return ()
    if structural_occupied.shape != known_free.shape:
        return ()
    if steps is None or not isinstance(steps, np.ndarray):
        return ()
    if steps.shape != known_free.shape:
        return ()
    if unknown is not None and (
        not isinstance(unknown, np.ndarray) or unknown.shape != known_free.shape
    ):
        return ()
    if labels is not None and (
        not isinstance(labels, np.ndarray) or labels.shape != known_free.shape
    ):
        return ()

    radius = max(1, int(support_radius))
    # The throat radius controls candidate identity/deduplication. A wider
    # architectural opening may place its jambs farther from the center than
    # that radius, so allow the caller to use the existing structural window
    # only for wall evidence without merging nearby doors.
    wall_radius = max(
        radius,
        radius if wall_search_radius is None else int(wall_search_radius),
    )
    span = max(1, int(minimum_wall_span_cells))
    # The opening cell itself may be removed by the inflated navigation mask:
    # it is evidence at the wall plane, not an executable robot footprint.
    # Reachability is required on ``source`` below, where the actual probe
    # viewpoint is compiled.
    traversable = known_free & ~structural_occupied
    candidates = []
    for normal in _CARDINAL:
        tangent = (0, 1) if normal[0] else (1, 0)
        positive_support = np.zeros_like(traversable, dtype=bool)
        negative_support = np.zeros_like(traversable, dtype=bool)
        for distance in range(1, wall_radius + 1):
            positive_support |= _shift(
                structural_occupied,
                distance * tangent[0],
                distance * tangent[1],
            )
            negative_support |= _shift(
                structural_occupied,
                -distance * tangent[0],
                -distance * tangent[1],
            )
        possible_openings = traversable & positive_support & negative_support
        for row, col in np.argwhere(possible_openings):
            opening = int(row), int(col)
            source = opening[0] - normal[0], opening[1] - normal[1]
            destination = opening[0] + normal[0], opening[1] + normal[1]
            if not (_in_bounds(known_free.shape, source) and _in_bounds(
                known_free.shape, destination,
            )):
                continue
            if not known_free[source]:
                continue
            # A doorway whose far side is already mapped is a structural
            # boundary; a doorway whose far side is still unknown is the
            # source-side information action that should reveal it.  Both
            # cases remain probes until the Portal ledger certifies a directed
            # physical edge.
            destination_known = bool(known_free[destination])
            destination_unknown = (
                unknown is not None and bool(unknown[destination])
            )
            if not destination_known and not destination_unknown:
                continue
            if not structural_wall_opening_for_normal(
                structural_occupied,
                opening,
                normal,
                wall_radius,
                span,
            ):
                continue
            if not _orientation_is_source_side(
                labels,
                steps,
                source_label,
                opening,
                normal,
                source,
                destination,
                wall_radius,
            ):
                continue
            candidates.append(
                StructuralBoundary(
                    opening_cell=opening,
                    normal=tuple(normal),
                    source_cell=source,
                    destination_cell=destination,
                )
            )
    return _deduplicate(candidates, radius)


__all__ = ["StructuralBoundary", "structural_boundary_candidates"]
