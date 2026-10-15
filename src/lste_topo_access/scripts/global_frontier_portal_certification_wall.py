"""Wall-opening evidence for a candidate indoor portal.

An occupancy-grid cut alone is not enough to identify a doorway: the same
cut appears in an aisle between a desk and a wall.  This module validates the
missing architectural fact.  A portal has to be an opening between two long,
opposing structural wall runs aligned with the route crossing direction.
"""

import numpy as np


def _crossing_tangent(transition):
    """Return the cardinal wall direction perpendicular to one crossing."""
    source = transition.source_cell
    destination = transition.destination_cell
    if source is None or destination is None:
        return None
    row_delta = int(destination[0]) - int(source[0])
    col_delta = int(destination[1]) - int(source[1])
    if row_delta == 0 and col_delta == 0:
        return None
    # A route travelling mostly across rows crosses a horizontal wall, whose
    # supporting runs extend across columns; the converse is symmetric.
    return (0, 1) if abs(row_delta) >= abs(col_delta) else (1, 0)


def _first_wall_on_ray(mask, origin, direction, maximum_distance):
    """Return the first structural wall cell along one tangent ray."""
    rows, cols = mask.shape
    row, col = int(origin[0]), int(origin[1])
    delta_row, delta_col = direction
    for distance in range(1, int(maximum_distance) + 1):
        candidate_row = row + distance * delta_row
        candidate_col = col + distance * delta_col
        if not (0 <= candidate_row < rows and 0 <= candidate_col < cols):
            return None
        if mask[candidate_row, candidate_col]:
            return candidate_row, candidate_col
    return None


def _outward_wall_run(mask, start, direction, minimum_length):
    """Require a wall to continue away from the opening, not just touch it."""
    rows, cols = mask.shape
    row, col = int(start[0]), int(start[1])
    delta_row, delta_col = direction
    for offset in range(int(minimum_length)):
        candidate_row = row + offset * delta_row
        candidate_col = col + offset * delta_col
        if not (
            0 <= candidate_row < rows
            and 0 <= candidate_col < cols
            and mask[candidate_row, candidate_col]
        ):
            return False
    return True


def _opening_candidates(transition):
    """Yield each low-clearance route cell that could be the wall opening."""
    throat_cells = tuple(transition.throat_cells or ())
    if throat_cells:
        return throat_cells
    return (transition.portal_cell,)


def _has_opposing_wall_supports(
    structural_occupied, portal, tangent, support_radius, required_span,
):
    """Return whether ``portal`` is bounded by two persistent wall runs."""
    positive = _first_wall_on_ray(
        structural_occupied, portal, tangent, support_radius,
    )
    negative_tangent = (-tangent[0], -tangent[1])
    negative = _first_wall_on_ray(
        structural_occupied, portal, negative_tangent, support_radius,
    )
    if positive is None or negative is None:
        return False
    return _outward_wall_run(
        structural_occupied, positive, tangent, required_span,
    ) and _outward_wall_run(
        structural_occupied, negative, negative_tangent, required_span,
    )


def structural_wall_opening_for_normal(
    structural_occupied, opening_cell, normal, support_radius,
    minimum_wall_span_cells,
):
    """Validate a wall-bounded opening before its far side is mapped.

    Unlike :func:`structural_wall_opening_cell`, this does not require a
    labelled destination room or a graph cut. It is deliberately weaker
    evidence used only to schedule a source-side observation probe. It can
    never authorize a physical Place transition.
    """
    if not isinstance(structural_occupied, np.ndarray):
        return False
    if structural_occupied.ndim != 2 or not structural_occupied.any():
        return False
    try:
        normal_row, normal_col = int(normal[0]), int(normal[1])
    except (IndexError, TypeError, ValueError):
        return False
    if abs(normal_row) + abs(normal_col) != 1:
        return False
    tangent = (0, 1) if normal_row else (1, 0)
    return _has_opposing_wall_supports(
        structural_occupied,
        (int(opening_cell[0]), int(opening_cell[1])),
        tangent,
        max(1, int(support_radius)),
        max(1, int(minimum_wall_span_cells)),
    )


def structural_wall_opening_cell(
    structural_occupied, transition, throat_radius_cells,
    minimum_wall_span_cells,
):
    """Return the physical wall-opening cell for a transition, if present.

    ``structural_occupied`` is the occupancy layer with compact observed
    furniture removed.  The two nearest supports must both be close enough to
    bound a practical doorway and extend away from the gap beyond the maximum
    furnishing scale.  Therefore a narrow aisle around a table has no valid
    architectural portal even when a local graph-cut test says it disconnects
    high-clearance free space.
    """
    if not isinstance(structural_occupied, np.ndarray):
        return None
    if structural_occupied.ndim != 2 or not structural_occupied.any():
        return None
    tangent = _crossing_tangent(transition)
    if tangent is None:
        return None
    rows, cols = structural_occupied.shape
    support_radius = max(1, int(throat_radius_cells) + 1)
    required_span = max(2, int(minimum_wall_span_cells))
    for portal in _opening_candidates(transition):
        portal = int(portal[0]), int(portal[1])
        if not (0 <= portal[0] < rows and 0 <= portal[1] < cols):
            continue
        if _has_opposing_wall_supports(
            structural_occupied,
            portal,
            tangent,
            support_radius,
            required_span,
        ):
            return portal
    return None


def has_structural_wall_opening(
    structural_occupied, transition, throat_radius_cells,
    minimum_wall_span_cells,
):
    """Compatibility predicate for callers that only need a verdict."""
    return structural_wall_opening_cell(
        structural_occupied,
        transition,
        throat_radius_cells,
        minimum_wall_span_cells,
    ) is not None
