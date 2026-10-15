"""Deterministic path recovery from robot-rooted BFS fields.

The topology code reasons about the sequence of structural labels *along an
actual reachable route*.  Keeping the predecessor walk here avoids subtly
different tie handling in the portal, hop-count, and fallback selectors.
"""


_NEIGHBOURS = ((1, 0), (-1, 0), (0, 1), (0, -1))


def route_predecessor_path(steps, target_row, target_col):
    """Return the deterministic BFS path from its seed to ``target``.

    ``steps`` is the distance field produced by the online frontier BFS.  It
    has no explicit parent array, so recover a predecessor by selecting the
    first cardinal neighbour at ``step - 1``.  This preserves the historical
    route semantics while giving all topology helpers one shared contract.
    """
    if steps is None:
        return None
    row, col = int(target_row), int(target_col)
    rows, cols = steps.shape
    if not (
        0 <= row < rows
        and 0 <= col < cols
        and int(steps[row, col]) >= 0
    ):
        return None

    reverse_path = []
    while True:
        reverse_path.append((row, col))
        step = int(steps[row, col])
        if step <= 0:
            break
        predecessor = None
        for delta_row, delta_col in _NEIGHBOURS:
            next_row, next_col = row + delta_row, col + delta_col
            if (
                0 <= next_row < rows
                and 0 <= next_col < cols
                and int(steps[next_row, next_col]) == step - 1
            ):
                predecessor = next_row, next_col
                break
        if predecessor is None:
            return None
        row, col = predecessor
    reverse_path.reverse()
    return reverse_path
