"""Structural place-map construction independent of navigation safety."""

import collections
import math

import numpy as np

from global_frontier_topology_components import TopologicalFreeSpaceComponents


class StructuralPlaceMap:

    """Build a persistent *place* view without weakening navigation safety.

    An occupancy grid serves two distinct consumers in this system:

    * Navfn and TEB need every measured obstacle, including a desk or chair.
    * Exploration needs architectural boundaries.  A desk is inside a room and
      must not make that room look like two rooms.

    The previous implementation used one high-clearance free-space mask for
    both purposes.  It consequently cut a conference room into independent
    regions whenever furniture made both side passages too narrow for that
    mask.  This class deliberately creates a second, read-only structural
    interpretation of the same online map.  It removes only an isolated,
    fully-observed, compact obstacle from the *place* map; the raw occupancy
    grid is never changed and remains the sole navigation/collision source.

    Walls and partitions are retained.  Inflating those structural obstacles
    still removes a doorway throat, so the resulting components are places
    separated by architectural portals rather than by furniture.  No Gazebo
    room name, prebuilt floor plan, or object ground truth is used.
    """

    def __init__(
        self,
        known_free,
        occupied,
        resolution,
        place_clearance_m,
        furniture_max_span_m,
        epoch,
    ):
        self.resolution = max(1e-6, float(resolution))
        self.furniture_max_span_m = max(0.10, float(furniture_max_span_m))
        self.furniture_mask = self._furniture_mask(
            known_free, occupied, self.resolution, self.furniture_max_span_m
        )
        # ``semantic_free`` exists only to name a place.  It is intentionally
        # not exposed to the route planner or the local collision checker.
        self.structural_occupied = occupied & ~self.furniture_mask
        self.semantic_free = known_free | self.furniture_mask
        inflation_cells = max(
            1, int(math.ceil(float(place_clearance_m) / self.resolution)) - 1
        )
        self.core = self.semantic_free & ~self._inflate(
            self.structural_occupied, inflation_cells
        )
        self.index = TopologicalFreeSpaceComponents(self.core, epoch)
        # The existing frontier interfaces consume a snapshot-local label
        # index.  Delegating it keeps the metric/topological boundary narrow.
        self.labels = self.index.labels
        self.epoch = self.index.epoch
        self.components = self.index.components

    @staticmethod
    def _inflate(mask, cells):
        inflated = mask.copy()
        rows, cols = mask.shape
        for dr in range(-cells, cells + 1):
            for dc in range(-cells, cells + 1):
                if dr * dr + dc * dc > cells * cells:
                    continue
                source_r0, source_r1 = max(0, -dr), min(rows, rows - dr)
                source_c0, source_c1 = max(0, -dc), min(cols, cols - dc)
                target_r0, target_r1 = max(0, dr), min(rows, rows + dr)
                target_c0, target_c1 = max(0, dc), min(cols, cols + dc)
                inflated[target_r0:target_r1, target_c0:target_c1] |= mask[
                    source_r0:source_r1, source_c0:source_c1
                ]
        return inflated

    @staticmethod
    def _component_has_unknown_halo(component_cells, known_free, occupied):
        """Return whether an obstacle touches unobserved map space.

        Before a lidar has seen both sides of a wall segment, it can look like
        a compact isolated obstacle.  Such an obstacle must stay structural
        until the local map has enough evidence to classify it as furnishing.
        """
        rows, cols = known_free.shape
        for row, col in component_cells:
            for dr, dc in (
                (1, 0), (-1, 0), (0, 1), (0, -1),
                (1, 1), (1, -1), (-1, 1), (-1, -1),
            ):
                nr, nc = row + dr, col + dc
                if not (0 <= nr < rows and 0 <= nc < cols):
                    return True
                if not known_free[nr, nc] and not occupied[nr, nc]:
                    return True
        return False

    @classmethod
    def _furniture_mask(cls, known_free, occupied, resolution, max_span_m):
        """Return compact, observed obstacle components that are furnishings.

        This is intentionally conservative.  A component has to be entirely
        inside the observed map, surrounded by known cells, compact in both
        dimensions, and no wider than the physically meaningful furnishing
        span.  Long/thin walls and anything beside unknown space remain part
        of the building structure.
        """
        result = np.zeros_like(occupied, dtype=bool)
        visited = np.zeros_like(occupied, dtype=bool)
        rows, cols = occupied.shape
        max_span_cells = max(1, int(math.ceil(max_span_m / resolution)))
        for start_row, start_col in np.argwhere(occupied):
            start_row, start_col = int(start_row), int(start_col)
            if visited[start_row, start_col]:
                continue
            queue = collections.deque([(start_row, start_col)])
            visited[start_row, start_col] = True
            cells = []
            min_row = max_row = start_row
            min_col = max_col = start_col
            touches_edge = False
            while queue:
                row, col = queue.popleft()
                cells.append((row, col))
                min_row = min(min_row, row)
                max_row = max(max_row, row)
                min_col = min(min_col, col)
                max_col = max(max_col, col)
                if row in (0, rows - 1) or col in (0, cols - 1):
                    touches_edge = True
                for dr, dc in (
                    (1, 0), (-1, 0), (0, 1), (0, -1),
                    (1, 1), (1, -1), (-1, 1), (-1, -1),
                ):
                    nr, nc = row + dr, col + dc
                    if (
                        0 <= nr < rows
                        and 0 <= nc < cols
                        and occupied[nr, nc]
                        and not visited[nr, nc]
                    ):
                        visited[nr, nc] = True
                        queue.append((nr, nc))
            height = max_row - min_row + 1
            width = max_col - min_col + 1
            # A wall segment may be short while it is only partly scanned,
            # hence both compactness and a complete known-space halo matter.
            compact = max(height, width) <= max_span_cells
            if (
                compact
                and not touches_edge
                and not cls._component_has_unknown_halo(cells, known_free, occupied)
            ):
                for row, col in cells:
                    result[row, col] = True
        return result

    def _evidence_for_labels(self, labels):
        return self.index._evidence_for_labels(labels)

    def nearby_evidence(self, row, col, radius_cells):
        return self.index.nearby_evidence(row, col, radius_cells)

    def route_evidence(self, steps, row, col, max_steps):
        return self.index.route_evidence(steps, row, col, max_steps)


