"""Extract candidate place crossings from a robot-rooted grid route."""

from global_frontier_portal_certification_models import RoutePlaceTransition
from global_frontier_topology_paths import route_predecessor_path


def route_place_transition_candidates(
    labels, steps, target_row, target_col, source_label,
):
    """Return every structural label change on the route to ``target``.

    A route can start in an unlabelled doorway throat while SLAM is updating.
    The first labelled core is retained as incomplete evidence; cut
    certification deliberately withholds the portal until both sides are
    observable.
    """
    if labels is None or steps is None or labels.shape != steps.shape:
        return []
    try:
        source_label = int(source_label)
    except (TypeError, ValueError):
        return []
    if source_label <= 0:
        return []
    path = route_predecessor_path(steps, target_row, target_col)
    if not path:
        return []

    transitions = []
    active_label = None
    active_cell = None
    active_index = None
    for index, cell in enumerate(path):
        label = int(labels[cell])
        if label <= 0:
            continue
        if active_label is None:
            if label == source_label:
                active_label = label
                active_cell = cell
                active_index = index
                continue
            transitions.append(
                RoutePlaceTransition(
                    source_label=source_label,
                    destination_label=label,
                    source_cell=None,
                    portal_cell=cell,
                    destination_cell=cell,
                    throat_cells=tuple(path[:index]),
                )
            )
            active_label = label
            active_cell = cell
            active_index = index
            continue
        if label == active_label:
            active_cell = cell
            active_index = index
            continue

        throat_cells = tuple(path[active_index + 1:index])
        transitions.append(
            RoutePlaceTransition(
                source_label=int(active_label),
                destination_label=label,
                source_cell=active_cell,
                portal_cell=throat_cells[0] if throat_cells else cell,
                destination_cell=cell,
                throat_cells=throat_cells,
            )
        )
        active_label = label
        active_cell = cell
        active_index = index
    return transitions


def portal_transition_exit_cell(
    steps, transition, minimum_distance_cells=1, continuation_cell=None,
):
    """Return a known-free portal endpoint beyond the controller's goal ball.

    ``destination_cell`` is the first *structural core*, which can be several
    metres inside a newly mapped room because door throats and low-clearance
    approach cells are deliberately unlabelled. A portal action should only
    cross one graph edge, so execute it at a short point after the certified
    opening while retaining the core separately for durable place identity.

    When the certified route continues beyond the first destination core,
    ``continuation_cell`` lets the action use that same path to create enough
    physical depth beyond the doorway. This is necessary because TEB may
    report XY success before reaching a shallow endpoint. The route never
    crosses a second graph edge here: callers pass only the first transition.
    """
    if transition is None:
        return None
    destination = (
        int(transition.destination_cell[0]),
        int(transition.destination_cell[1]),
    )
    target = destination
    if continuation_cell is not None:
        candidate = (int(continuation_cell[0]), int(continuation_cell[1]))
        candidate_path = route_predecessor_path(steps, candidate[0], candidate[1])
        if candidate_path:
            target = candidate
    path = route_predecessor_path(steps, target[0], target[1])
    if not path:
        return destination
    try:
        portal_index = path.index(
            (int(transition.portal_cell[0]), int(transition.portal_cell[1]))
        )
    except ValueError:
        return destination
    offset = max(1, int(minimum_distance_cells))
    return path[min(len(path) - 1, portal_index + offset)]


def portal_transition_verified_exit_cell(
    steps, transition, minimum_signed_depth_cells, continuation_cell=None,
    next_transition=None,
):
    """Return a one-edge portal endpoint outside the controller goal ball.

    BFS distance is not crossing evidence: a route can turn after the gate,
    while physical portal validation projects odometry along the directed gate
    normal.  Select the first route cell that has the required directed depth.

    ``next_transition`` is the next *certified* portal on this same route.
    It is a hard upper bound: a remote frontier must never turn one portal
    command into two crossings. Raw structural labels are deliberately not a
    bound here because furniture and partial SLAM updates can split one room.
    ``None`` means the current map has no safe endpoint that can prove this
    one crossing.
    """
    if transition is None:
        return None
    destination = (
        int(transition.destination_cell[0]),
        int(transition.destination_cell[1]),
    )
    target = destination
    if continuation_cell is not None:
        candidate = (int(continuation_cell[0]), int(continuation_cell[1]))
        if route_predecessor_path(steps, *candidate):
            target = candidate
    path = route_predecessor_path(steps, *target)
    if not path:
        return None
    portal_cell = (int(transition.portal_cell[0]), int(transition.portal_cell[1]))
    try:
        portal_index = path.index(portal_cell)
    except ValueError:
        # The physical gate can be adjacent to the rasterised route when a
        # wall opening spans several cells. In that case the source core is
        # the last reliable route anchor before the crossing.
        source_cell = transition.source_cell
        if source_cell is None:
            return None
        try:
            portal_index = path.index((int(source_cell[0]), int(source_cell[1])))
        except ValueError:
            return None

    gate_row, gate_col = transition.portal_cell
    direction_row = int(transition.destination_cell[0]) - int(gate_row)
    direction_col = int(transition.destination_cell[1]) - int(gate_col)
    direction_length = float(
        direction_row * direction_row + direction_col * direction_col
    ) ** 0.5
    if direction_length <= 1e-6:
        return None
    required_depth = max(0.0, float(minimum_signed_depth_cells))
    stop_index = len(path)
    if next_transition is not None:
        next_cells = [next_transition.portal_cell, next_transition.source_cell]
        for next_cell in next_cells:
            if next_cell is None:
                continue
            try:
                candidate_index = path.index((int(next_cell[0]), int(next_cell[1])))
            except ValueError:
                continue
            if candidate_index > portal_index:
                stop_index = min(stop_index, candidate_index)
    for row, col in path[portal_index + 1:stop_index]:
        signed_depth = (
            (int(row) - int(gate_row)) * direction_row
            + (int(col) - int(gate_col)) * direction_col
        ) / direction_length
        if signed_depth >= required_depth:
            return int(row), int(col)
    return None
