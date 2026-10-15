"""Pure grid reachability and line-of-sight predicates for frontiers."""

import collections
import math

import numpy as np

def grid_reachable_without_closed_places(
    labels, steps, source_label, closed_labels, closed_footprint=None,
):
    """Return the route-reachable mask after removing closed place nodes.

    A completed place is a graph node, not a small negative term in a frontier
    score. Removing its current snapshot label from the reachable grid stops
    the planner from choosing a far frontier by driving back through a room it
    has already inspected. The source place stays available so the robot can
    leave the room in which it just completed an observation.

    ``steps`` already represents the actual route-clearance free space rooted
    at the route source. Re-running BFS over that finite mask avoids making a
    second, inconsistent clearance interpretation for this high-level check.
    """
    if labels is None or steps is None:
        return None
    if not closed_labels and closed_footprint is None:
        return steps >= 0
    blocked = np.isin(labels, list(closed_labels)) if closed_labels else np.zeros_like(
        labels, dtype=bool
    )
    if closed_footprint is not None:
        blocked |= closed_footprint
    if source_label is not None:
        blocked &= labels != int(source_label)
    reachable = (steps >= 0) & ~blocked
    seed_cells = np.argwhere(steps == 0)
    if seed_cells.size == 0:
        return np.zeros_like(steps, dtype=bool)
    seed = int(seed_cells[0][0]), int(seed_cells[0][1])
    if not reachable[seed]:
        # A source can sit in a doorway cell without a place label. It remains
        # a valid root of the metric route and must not create a false block.
        reachable[seed] = True
    bounded_steps = np.full(steps.shape, -1, dtype=np.int32)
    queue = collections.deque([seed])
    bounded_steps[seed] = 0
    rows, cols = steps.shape
    while queue:
        row, col = queue.popleft()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            next_row, next_col = row + dr, col + dc
            if (
                0 <= next_row < rows
                and 0 <= next_col < cols
                and reachable[next_row, next_col]
                and bounded_steps[next_row, next_col] < 0
            ):
                bounded_steps[next_row, next_col] = bounded_steps[row, col] + 1
                queue.append((next_row, next_col))
    return bounded_steps >= 0


def grid_route_avoids_closed_places(
    labels, steps, target_row, target_col, source_label, closed_labels,
):
    """Return whether a BFS route exists without transiting a closed place."""
    target_row, target_col = int(target_row), int(target_col)
    reachable = grid_reachable_without_closed_places(
        labels, steps, source_label, closed_labels,
    )
    return bool(
        reachable is not None
        and 0 <= target_row < reachable.shape[0]
        and 0 <= target_col < reachable.shape[1]
        and reachable[target_row, target_col]
    )


def grid_visible_free_footprint(known_free, anchor_cells, max_range_cells):
    """Reconstruct free cells directly visible from already reached anchors.

    This lightweight 2-D ray cast builds an online submap footprint.  It is
    intentionally derived from the current occupancy map rather than Gazebo
    model names or a hidden floor plan. Unknown and occupied cells terminate a
    ray, so visibility cannot leak through a wall or an unobserved doorway.
    """
    footprint = np.zeros_like(known_free, dtype=bool)
    if max_range_cells <= 0:
        return footprint
    rows, cols = known_free.shape
    max_range_cells = int(max_range_cells)
    # At the maximum range neighbouring rays are no more than one cell apart.
    ray_count = max(32, int(math.ceil(2.0 * math.pi * max_range_cells)))
    for anchor_row, anchor_col in anchor_cells:
        anchor_row, anchor_col = int(anchor_row), int(anchor_col)
        if not (0 <= anchor_row < rows and 0 <= anchor_col < cols):
            continue
        for ray_index in range(ray_count):
            angle = 2.0 * math.pi * float(ray_index) / float(ray_count)
            sin_angle = math.sin(angle)
            cos_angle = math.cos(angle)
            previous = None
            for distance in range(max_range_cells + 1):
                row = int(round(anchor_row + sin_angle * distance))
                col = int(round(anchor_col + cos_angle * distance))
                if not (0 <= row < rows and 0 <= col < cols):
                    break
                cell = row, col
                if cell == previous:
                    continue
                previous = cell
                if not known_free[row, col]:
                    break
                footprint[row, col] = True
    return footprint


def grid_line_is_known_free(known_free, start_row, start_col, end_row, end_col):
    """Return whether a discrete map ray crosses only known-free cells.

    A completed frontier is an observation viewpoint, not merely a point that
    happened to be reached.  This helper is the conservative visibility test
    used when deciding whether a later frontier has already been observed from
    that viewpoint.  Unknown cells deliberately block the ray: an incomplete
    map must never manufacture coverage behind an unobserved wall or desk.
    """
    rows, cols = known_free.shape
    start_row, start_col = int(start_row), int(start_col)
    end_row, end_col = int(end_row), int(end_col)
    if not (
        0 <= start_row < rows
        and 0 <= start_col < cols
        and 0 <= end_row < rows
        and 0 <= end_col < cols
    ):
        return False
    steps = max(abs(end_row - start_row), abs(end_col - start_col))
    if steps == 0:
        return bool(known_free[start_row, start_col])
    for index in range(steps + 1):
        fraction = float(index) / float(steps)
        row = int(round(start_row + (end_row - start_row) * fraction))
        col = int(round(start_col + (end_col - start_col) * fraction))
        if not (0 <= row < rows and 0 <= col < cols and known_free[row, col]):
            return False
    return True


def grid_frontier_observed_from_viewpoint(
    known_free,
    unknown,
    observer_row,
    observer_col,
    frontier_row,
    frontier_col,
    unknown_halo_cells,
):
    """Return whether one frontier endpoint is already observed from a pose.

    A frontier is an *observation task*, not a physical destination.  Online
    SLAM can resolve the endpoint while the base remains at a safe standoff.
    This certificate needs both a free line of sight and an unknown-free halo
    around the endpoint.  The ray prevents a wall-hidden boundary from being
    retired; the halo prevents a visible doorway from being mistaken for a
    resolved dead end.
    """
    if (
        known_free is None
        or unknown is None
        or known_free.shape != unknown.shape
    ):
        return False
    rows, cols = known_free.shape
    observer_row, observer_col = int(observer_row), int(observer_col)
    frontier_row, frontier_col = int(frontier_row), int(frontier_col)
    if not (
        0 <= observer_row < rows
        and 0 <= observer_col < cols
        and 0 <= frontier_row < rows
        and 0 <= frontier_col < cols
    ):
        return False
    if not grid_line_is_known_free(
        known_free,
        observer_row,
        observer_col,
        frontier_row,
        frontier_col,
    ):
        return False
    radius = max(0, int(unknown_halo_cells))
    r0, r1 = max(0, frontier_row - radius), min(rows, frontier_row + radius + 1)
    c0, c1 = max(0, frontier_col - radius), min(cols, frontier_col + radius + 1)
    return not bool(np.any(unknown[r0:r1, c0:c1]))



