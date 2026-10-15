"""Local raw-free-space cut test for candidate architectural portals."""

from collections import deque

import numpy as np


_NEIGHBOURS = ((1, 0), (-1, 0), (0, 1), (0, -1))


def _inflate_cells(shape, cells, radius_cells):
    """Return a circular mask around candidate throat cells."""
    result = np.zeros(shape, dtype=bool)
    rows, cols = shape
    for row, col in cells:
        row, col = int(row), int(col)
        for delta_row in range(-radius_cells, radius_cells + 1):
            for delta_col in range(-radius_cells, radius_cells + 1):
                if delta_row * delta_row + delta_col * delta_col > radius_cells ** 2:
                    continue
                next_row, next_col = row + delta_row, col + delta_col
                if 0 <= next_row < rows and 0 <= next_col < cols:
                    result[next_row, next_col] = True
    return result


def _throat_cut_probes(transition, maximum=32):
    """Return evenly distributed cut probes along one low-clearance run."""
    cells = transition.throat_cells
    if len(cells) <= maximum:
        return cells
    indices = np.linspace(0, len(cells) - 1, maximum, dtype=np.int32)
    return tuple(cells[int(index)] for index in np.unique(indices))


def _has_local_raw_free_bypass(
    known_free, transition, cut_cell, throat_radius_cells, window_margin_cells,
):
    """Return whether raw known-free space locally bypasses a candidate cut."""
    if transition.source_cell is None or not transition.throat_cells:
        return False
    source_row, source_col = transition.source_cell
    destination_row, destination_col = transition.destination_cell
    rows, cols = known_free.shape
    if not (
        0 <= source_row < rows
        and 0 <= source_col < cols
        and 0 <= destination_row < rows
        and 0 <= destination_col < cols
        and known_free[source_row, source_col]
        and known_free[destination_row, destination_col]
    ):
        return False

    support_cells = [cut_cell, transition.source_cell, transition.destination_cell]
    min_row = max(0, min(row for row, _col in support_cells) - window_margin_cells)
    max_row = min(
        rows, max(row for row, _col in support_cells) + window_margin_cells + 1
    )
    min_col = max(0, min(col for _row, col in support_cells) - window_margin_cells)
    max_col = min(
        cols, max(col for _row, col in support_cells) + window_margin_cells + 1
    )

    local_free = np.zeros_like(known_free, dtype=bool)
    local_free[min_row:max_row, min_col:max_col] = known_free[
        min_row:max_row, min_col:max_col
    ]
    local_free &= ~_inflate_cells(
        known_free.shape, [cut_cell], throat_radius_cells
    )
    # Keep the two sides usable even when the inflation disk touches a core.
    local_free[source_row, source_col] = True
    local_free[destination_row, destination_col] = True

    queue = deque([transition.source_cell])
    visited = np.zeros_like(known_free, dtype=bool)
    visited[transition.source_cell] = True
    while queue:
        row, col = queue.popleft()
        if (row, col) == transition.destination_cell:
            return True
        for delta_row, delta_col in _NEIGHBOURS:
            next_row, next_col = row + delta_row, col + delta_col
            if (
                0 <= next_row < rows
                and 0 <= next_col < cols
                and local_free[next_row, next_col]
                and not visited[next_row, next_col]
            ):
                visited[next_row, next_col] = True
                queue.append((next_row, next_col))
    return False


def is_structural_cut(
    known_free, transition, throat_radius_cells, window_margin_cells,
):
    """Return whether any throat probe separates the two place cores."""
    if transition.source_cell is None or not transition.throat_cells:
        return False
    probes = _throat_cut_probes(transition)
    # Prefer an interior probe. A disk centred against a core would isolate
    # it by construction, not because an architectural wall exists.
    interior_probes = tuple(
        cell for cell in probes
        if (
            (cell[0] - transition.source_cell[0]) ** 2
            + (cell[1] - transition.source_cell[1]) ** 2
            > throat_radius_cells ** 2
            and (cell[0] - transition.destination_cell[0]) ** 2
            + (cell[1] - transition.destination_cell[1]) ** 2
            > throat_radius_cells ** 2
        )
    )
    for cut_cell in interior_probes or probes:
        if not _has_local_raw_free_bypass(
            known_free,
            transition,
            cut_cell,
            throat_radius_cells,
            window_margin_cells,
        ):
            return True
    return False
