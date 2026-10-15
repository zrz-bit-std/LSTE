"""Snapshot-local high-clearance component labelling for frontier topology."""

import collections

import numpy as np


class TopologicalFreeSpaceComponents:

    """Label doorway-separated known-free cores for one planning snapshot.

    A full unknown-space component is often the entire unobserved building or
    exterior, especially during SLAM startup.  It is therefore too coarse to
    represent a room.  Instead, this class labels the *high-clearance core* of
    known-free space after a modest additional obstacle inflation removes
    doorway throats.  The resulting components approximate room/corridor
    regions without using a prebuilt map or Gazebo room annotations.

    Four-connectivity is deliberate.  Diagonal free pixels can touch at a wall
    corner, but are not a traversable indoor passage.
    """

    def __init__(self, traversable_core, epoch):
        self.labels = np.zeros(traversable_core.shape, dtype=np.int32)
        self.epoch = int(epoch)
        self.components = {}
        rows, cols = traversable_core.shape
        label = 0
        for start_row, start_col in np.argwhere(traversable_core):
            start_row, start_col = int(start_row), int(start_col)
            if self.labels[start_row, start_col] != 0:
                continue
            label += 1
            queue = collections.deque([(start_row, start_col)])
            self.labels[start_row, start_col] = label
            count = 0
            row_sum = 0.0
            col_sum = 0.0
            min_row = max_row = start_row
            min_col = max_col = start_col
            while queue:
                row, col = queue.popleft()
                count += 1
                row_sum += row
                col_sum += col
                min_row = min(min_row, row)
                max_row = max(max_row, row)
                min_col = min(min_col, col)
                max_col = max(max_col, col)
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    next_row, next_col = row + dr, col + dc
                    if (
                        0 <= next_row < rows
                        and 0 <= next_col < cols
                        and traversable_core[next_row, next_col]
                        and self.labels[next_row, next_col] == 0
                    ):
                        self.labels[next_row, next_col] = label
                        queue.append((next_row, next_col))
            self.components[label] = {
                "label": label,
                "cells": count,
                "center_row": row_sum / count,
                "center_col": col_sum / count,
                "min_row": min_row,
                "max_row": max_row,
                "min_col": min_col,
                "max_col": max_col,
            }

    def _evidence_for_labels(self, labels):
        labels = [int(label) for label in labels if int(label) > 0]
        if not labels:
            return None
        # A boundary can touch more than one unknown island.  The largest one
        # is the interior that can reveal the most information and is stable
        # against one-cell scan noise at the frontier edge.
        label = max(set(labels), key=lambda value: self.components[value]["cells"])
        evidence = dict(self.components[label])
        evidence["epoch"] = self.epoch
        return evidence

    def nearby_evidence(self, row, col, radius_cells):
        """Find the nearest doorway-separated core around an endpoint."""
        radius_cells = max(0, int(radius_cells))
        rows, cols = self.labels.shape
        r0, r1 = max(0, int(row) - radius_cells), min(rows, int(row) + radius_cells + 1)
        c0, c1 = max(0, int(col) - radius_cells), min(cols, int(col) + radius_cells + 1)
        window = self.labels[r0:r1, c0:c1]
        candidates = np.argwhere(window > 0)
        if candidates.size == 0:
            return None
        candidates[:, 0] += r0
        candidates[:, 1] += c0
        distance = (candidates[:, 0] - int(row)) ** 2 + (candidates[:, 1] - int(col)) ** 2
        nearest = candidates[int(np.argmin(distance))]
        return self._evidence_for_labels([self.labels[int(nearest[0]), int(nearest[1])]])

    def route_evidence(self, steps, row, col, max_steps):
        """Return the first core met when walking a validated BFS route home.

        A frontier approach can sit in newly observed free space before that
        space has enough clearance to contain a core cell.  Walking its real
        BFS predecessor chain toward the robot lets it inherit the first
        established room/corridor core, without looking through walls or
        inventing a direct geometric connection.
        """
        if steps is None or not (0 <= int(row) < steps.shape[0] and 0 <= int(col) < steps.shape[1]):
            return None
        current = int(row), int(col)
        for _ in range(max(0, int(max_steps)) + 1):
            label = int(self.labels[current])
            if label > 0:
                return self._evidence_for_labels([label])
            current_step = int(steps[current])
            if current_step <= 0:
                return None
            predecessors = []
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                next_row, next_col = current[0] + dr, current[1] + dc
                if (
                    0 <= next_row < steps.shape[0]
                    and 0 <= next_col < steps.shape[1]
                    and 0 <= int(steps[next_row, next_col]) < current_step
                ):
                    predecessors.append((next_row, next_col))
            if not predecessors:
                return None
            current = min(
                predecessors,
                key=lambda cell: (int(steps[cell]), cell[0], cell[1]),
            )
        return None



