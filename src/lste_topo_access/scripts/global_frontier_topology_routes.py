"""Route-derived structural place transitions.

This module intentionally preserves the raw label-change helpers for
diagnostics and compatibility.  New navigation decisions use
``global_frontier_portal_certification`` so furniture-induced label changes
cannot become lifecycle transitions.
"""

import numpy as np

from global_frontier_topology_paths import route_predecessor_path


def _validated_route(labels, steps, target_row, target_col, source_label):
    """Return a predecessor path only when its structural source is known."""
    if labels is None or steps is None or labels.shape != steps.shape:
        return None
    if source_label is None or int(source_label) <= 0:
        return None
    return route_predecessor_path(steps, target_row, target_col)


def route_place_hops(labels, steps, target_row, target_col, source_label=None):
    """Count raw structural-label transitions on one BFS predecessor path.

    Zero labels represent a low-clearance throat and are transparent.  This
    helper reports label changes only; callers that want to dispatch a
    cross-place action must use the certified companion API.
    """
    path = _validated_route(
        labels, steps, target_row, target_col, source_label,
    )
    if path is None:
        return None
    sequence = []
    for row, col in path:
        label = int(labels[row, col])
        if label > 0 and (not sequence or label != sequence[-1]):
            sequence.append(label)
    if not sequence or sequence[0] != int(source_label):
        sequence.insert(0, int(source_label))
    return max(0, len(sequence) - 1)


def first_route_place_portal(labels, steps, target_row, target_col, source_label):
    """Return the first raw throat/core cell beyond ``source_label``.

    Kept for compatibility with the temporary unlabelled-doorway recovery
    path.  It is intentionally not a certificate that this is a real door.
    """
    path = _validated_route(
        labels, steps, target_row, target_col, source_label,
    )
    if path is None:
        return None
    source_seen = False
    first_non_source_core = None
    for row, col in path:
        label = int(labels[row, col])
        if label > 0 and label != int(source_label) and first_non_source_core is None:
            first_non_source_core = int(row), int(col)
        if label == int(source_label):
            source_seen = True
            continue
        if source_seen:
            return int(row), int(col)
    return first_non_source_core


def first_route_place_transition_goal(
    labels, steps, target_row, target_col, source_label,
):
    """Return the first raw next-place core along a validated BFS route.

    A raw next-place core is useful for observing map topology.  It must pass
    portal certification before it can close a region or represent a doorway
    action in the navigation lifecycle.
    """
    path = _validated_route(
        labels, steps, target_row, target_col, source_label,
    )
    if path is None:
        return None
    source_seen = False
    left_source = False
    first_non_source_core = None
    for row, col in path:
        label = int(labels[row, col])
        if not source_seen:
            if label == int(source_label):
                source_seen = True
            continue
        if not left_source:
            if label != int(source_label):
                left_source = True
            else:
                continue
        if label > 0 and label != int(source_label):
            return int(row), int(col)
    # The base can be in an unlabelled door throat while a fresh map grows the
    # next room. Preserve the historic recovery behaviour for that condition.
    for row, col in path:
        label = int(labels[row, col])
        if label > 0 and label != int(source_label):
            first_non_source_core = int(row), int(col)
            break
    return first_non_source_core


def adjacent_place_transition_goals(
    labels, steps, source_label, closed_labels=None, max_candidates=32,
):
    """Return nearest raw next-place cores for topology diagnostics.

    This legacy helper deliberately remains label-only.  Runtime portal
    selection calls ``certified_adjacent_place_transition_goals`` instead.
    """
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
            goal = first_route_place_transition_goal(
                labels, steps, row, col, source_label,
            )
            if goal is None:
                continue
            goal_label = int(labels[goal])
            if goal_label <= 0 or goal_label in closed:
                continue
            distance = int(steps[goal])
            previous = transitions.get(goal)
            if previous is None or distance < previous[0]:
                transitions[goal] = distance, goal_label
            break
    ordered = sorted(
        (distance, row, col, label)
        for (row, col), (distance, label) in transitions.items()
    )
    return [
        (row, col, label)
        for _distance, row, col, label in ordered[:max(0, int(max_candidates))]
    ]
