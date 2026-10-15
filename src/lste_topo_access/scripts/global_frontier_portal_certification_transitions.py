"""Certify route and adjacent-place portal transitions."""

from dataclasses import replace

import numpy as np

from global_frontier_portal_certification_cut import is_structural_cut
from global_frontier_portal_certification_routes import (
    route_place_transition_candidates,
)
from global_frontier_portal_certification_wall import (
    structural_wall_opening_cell,
)


def _certification_cache_key(
    transition, radius, margin, structural_occupied, minimum_wall_span_cells,
):
    """Describe evidence that is immutable during one selection pass."""
    return (
        transition.source_label,
        transition.destination_label,
        transition.source_cell,
        transition.destination_cell,
        transition.throat_cells,
        radius,
        margin,
        structural_occupied is not None,
        minimum_wall_span_cells,
    )


def _certify_transition(
    known_free, structural_occupied, transition, radius, margin,
    minimum_wall_span_cells,
):
    """Return ``(accepted, opening_cell)`` for one topology edge.

    The raw-free cut proves that the route crosses a narrow separator.  When
    structural occupancy is available, the wall scan identifies the exact
    throat cell that is a persistent architectural opening.
    """
    if not is_structural_cut(known_free, transition, radius, margin):
        return False, None
    if structural_occupied is None:
        return True, transition.portal_cell
    opening_cell = structural_wall_opening_cell(
        structural_occupied,
        transition,
        radius,
        minimum_wall_span_cells,
    )
    return opening_cell is not None, opening_cell


def _cached_certification(cache, key, certify):
    """Read or produce one immutable evidence result for this map snapshot."""
    result = None if cache is None else cache.get(key)
    if result is None:
        result = certify()
        if cache is not None:
            cache[key] = result
    # Older external callers may supply a cache built by the former boolean
    # contract.  Keep that cache usable for legacy grid-only certification.
    if isinstance(result, bool):
        return result, None
    return result


def certified_route_place_transitions(
    labels, steps, known_free, target_row, target_col, source_label,
    throat_radius_cells, window_margin_cells=None, cache=None,
    structural_occupied=None, minimum_wall_span_cells=None,
):
    """Return crossings with both local-cut and wall-opening evidence.

    The wall check is optional only for compatibility with pure legacy grid
    callers. Runtime selection always supplies ``structural_occupied`` from
    ``StructuralPlaceMap``, which removes compact furniture before deciding
    whether a cut can represent an architectural doorway.
    """
    if known_free is None or labels is None or known_free.shape != labels.shape:
        return []
    radius = max(1, int(throat_radius_cells))
    margin = (
        max(2 * radius + 1, 3)
        if window_margin_cells is None
        else max(radius + 1, int(window_margin_cells))
    )
    certified = []
    for transition in route_place_transition_candidates(
        labels, steps, target_row, target_col, source_label,
    ):
        key = _certification_cache_key(
            transition,
            radius,
            margin,
            structural_occupied,
            minimum_wall_span_cells,
        )
        accepted, opening_cell = _cached_certification(
            cache,
            key,
            lambda: _certify_transition(
                known_free,
                structural_occupied,
                transition,
                radius,
                margin,
                minimum_wall_span_cells,
            ),
        )
        if accepted:
            certified.append(replace(
                transition,
                portal_cell=(
                    transition.portal_cell if opening_cell is None else opening_cell
                ),
            ))
    return certified


def certified_route_place_hops(
    labels, steps, known_free, target_row, target_col, source_label,
    throat_radius_cells, window_margin_cells=None, cache=None,
    structural_occupied=None, minimum_wall_span_cells=None,
):
    """Count certified architectural transitions on one route."""
    return len(certified_route_place_transitions(
        labels,
        steps,
        known_free,
        target_row,
        target_col,
        source_label,
        throat_radius_cells,
        window_margin_cells=window_margin_cells,
        cache=cache,
        structural_occupied=structural_occupied,
        minimum_wall_span_cells=minimum_wall_span_cells,
    ))


def first_certified_route_place_transition_goal(
    labels, steps, known_free, target_row, target_col, source_label,
    throat_radius_cells, window_margin_cells=None, cache=None,
    structural_occupied=None, minimum_wall_span_cells=None,
):
    """Return the first destination core after a certified portal crossing."""
    transitions = certified_route_place_transitions(
        labels,
        steps,
        known_free,
        target_row,
        target_col,
        source_label,
        throat_radius_cells,
        window_margin_cells=window_margin_cells,
        cache=cache,
        structural_occupied=structural_occupied,
        minimum_wall_span_cells=minimum_wall_span_cells,
    )
    return None if not transitions else transitions[0].destination_cell


def certified_adjacent_place_transitions(
    labels, steps, known_free, source_label, throat_radius_cells,
    closed_labels=None, max_candidates=32, window_margin_cells=None,
    cache=None, structural_occupied=None, minimum_wall_span_cells=None,
):
    """Return first certified doorway edges without relying on frontiers."""
    if labels is None or steps is None or labels.shape != steps.shape:
        return []
    try:
        source_label = int(source_label)
    except (TypeError, ValueError):
        return []
    if source_label <= 0:
        return []
    closed = {int(label) for label in (closed_labels or ()) if int(label) > 0}
    reachable = (steps >= 0) & (labels > 0) & (labels != source_label)
    if closed:
        reachable &= ~np.isin(labels, list(closed))

    transitions = {}
    for destination in np.unique(labels[reachable]).tolist():
        destination = int(destination)
        cells = np.argwhere(reachable & (labels == destination))
        order = np.argsort(steps[cells[:, 0], cells[:, 1]], kind="stable")
        for index in order[:16].tolist():
            row, col = [int(value) for value in cells[index]]
            candidates = certified_route_place_transitions(
                labels,
                steps,
                known_free,
                row,
                col,
                source_label,
                throat_radius_cells,
                window_margin_cells=window_margin_cells,
                cache=cache,
                structural_occupied=structural_occupied,
                minimum_wall_span_cells=minimum_wall_span_cells,
            )
            if not candidates:
                continue
            transition = candidates[0]
            if transition.destination_label in closed:
                continue
            distance = int(steps[transition.destination_cell])
            previous = transitions.get(transition.destination_cell)
            if previous is None or distance < previous[0]:
                transitions[transition.destination_cell] = distance, transition
            break
    ordered = sorted(
        (distance, transition.destination_cell, transition)
        for _destination, (distance, transition) in transitions.items()
    )
    return [
        transition
        for _distance, _destination, transition in ordered[
            :max(0, int(max_candidates))
        ]
    ]


def certified_adjacent_place_transition_goals(
    labels, steps, known_free, source_label, throat_radius_cells,
    closed_labels=None, max_candidates=32, window_margin_cells=None,
    cache=None, structural_occupied=None, minimum_wall_span_cells=None,
):
    """Return legacy destination tuples while retaining full transition data."""
    return [
        (
            int(transition.destination_cell[0]),
            int(transition.destination_cell[1]),
            int(transition.destination_label),
        )
        for transition in certified_adjacent_place_transitions(
            labels,
            steps,
            known_free,
            source_label,
            throat_radius_cells,
            closed_labels=closed_labels,
            max_candidates=max_candidates,
            window_margin_cells=window_margin_cells,
            cache=cache,
            structural_occupied=structural_occupied,
            minimum_wall_span_cells=minimum_wall_span_cells,
        )
    ]
