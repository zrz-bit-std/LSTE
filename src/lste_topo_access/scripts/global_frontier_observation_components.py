"""Conversion between a live occupancy grid and persistent place evidence."""

import math
import time

import numpy as np
import rospy

from global_frontier_topology import StructuralPlaceMap, grid_line_is_known_free


class GlobalFrontierObservationComponentsMixin:

    def build_topology_components(self, known_free, occupied, message):
        """Build doorway-separated structural places for frontier association.

        The navigation mask is deliberately not reused here. ``occupied``
        continues to be passed unchanged to costmap/Navfn/TEB, while the place
        map strips only compact, fully observed furnishings before separating
        the remaining architecture at doorway throats.
        """
        self.topology_component_epoch += 1
        self.completed_viewpoint_component_cache.clear()
        started = time.monotonic()
        components = StructuralPlaceMap(
            known_free,
            occupied,
            message.info.resolution,
            self.region_topology_clearance,
            self.place_furniture_max_span_m,
            self.topology_component_epoch,
        )
        build_seconds = time.monotonic() - started
        rospy.loginfo_throttle(
            10.0,
            "Global frontier structural places=%d core_cells=%d furniture_cells=%d "
            "clearance=%.2fm association_radius=%.2fm build=%.3fs",
            len(components.components),
            int(np.count_nonzero(components.core)),
            int(np.count_nonzero(components.furniture_mask)),
            self.region_topology_clearance,
            self.region_topology_association_radius,
            build_seconds,
        )
        return components

    def component_evidence(self, message, components, raw_evidence):
        """Convert a grid component descriptor into a map-frame identity."""
        if raw_evidence is None:
            return None
        resolution = float(message.info.resolution)
        origin_x = float(message.info.origin.position.x)
        origin_y = float(message.info.origin.position.y)
        return {
            "epoch": int(raw_evidence["epoch"]),
            "label": int(raw_evidence["label"]),
            "cells": int(raw_evidence["cells"]),
            "center_x": origin_x + (float(raw_evidence["center_col"]) + 0.5) * resolution,
            "center_y": origin_y + (float(raw_evidence["center_row"]) + 0.5) * resolution,
            # Bounds name the outer edges of unknown cells, not their centres,
            # so bbox overlap remains meaningful after a one-cell SLAM shift.
            "min_x": origin_x + float(raw_evidence["min_col"]) * resolution,
            "max_x": origin_x + (float(raw_evidence["max_col"]) + 1.0) * resolution,
            "min_y": origin_y + float(raw_evidence["min_row"]) * resolution,
            "max_y": origin_y + (float(raw_evidence["max_row"]) + 1.0) * resolution,
        }

    def topology_component_evidence(
        self, message, components, row, col, steps=None,
    ):
        route_search_cells = int(math.ceil(
            (self.region_memory_radius + self.frontier_approach_distance)
            / message.info.resolution
        ))
        raw_evidence = components.route_evidence(
            steps, row, col, route_search_cells,
        )
        if raw_evidence is None:
            raw_evidence = components.nearby_evidence(
                row,
                col,
                int(math.ceil(
                    self.region_topology_association_radius
                    / message.info.resolution
                )),
            )
        evidence = self.component_evidence(
            message,
            components,
            raw_evidence,
        )
        # A tiny high-clearance island is normally a raster artefact near a
        # chair or wall. Do not let it override point-level memory.
        if (
            evidence is not None
            and evidence["cells"] < self.region_memory.component_min_cells
        ):
            return None
        return evidence

    def component_at_map_position(self, message, components, known_free, x, y):
        """Resolve a stored map position to its current visible topology core.

        Region memory lives longer than an individual SLAM grid snapshot.  A
        position inside a current high-clearance core has an exact identity.
        A frontier approach may instead sit just outside that core, so search
        its small neighborhood, but only along known-free raster rays.  This
        makes the early/late-map association conservative at room walls and
        doorways.
        """
        cell = self.xy_to_grid_cell(message, x, y)
        if cell is None:
            return None
        row, col = cell
        if int(components.labels[row, col]) > 0:
            raw_evidence = components._evidence_for_labels(
                [components.labels[row, col]]
            )
            return self.component_evidence(message, components, raw_evidence)

        radius = max(
            1,
            int(math.ceil(
                self.region_topology_association_radius / message.info.resolution
            )),
        )
        row_start = max(0, row - radius)
        row_stop = min(components.labels.shape[0], row + radius + 1)
        col_start = max(0, col - radius)
        col_stop = min(components.labels.shape[1], col + radius + 1)
        nearby = np.argwhere(
            components.labels[row_start:row_stop, col_start:col_stop] > 0
        )
        if nearby.size == 0:
            return None
        nearby[:, 0] += row_start
        nearby[:, 1] += col_start
        distances = (nearby[:, 0] - row) ** 2 + (nearby[:, 1] - col) ** 2
        for index in np.argsort(distances).tolist():
            candidate_row, candidate_col = nearby[int(index)]
            if not grid_line_is_known_free(
                known_free, row, col, int(candidate_row), int(candidate_col)
            ):
                continue
            raw_evidence = components._evidence_for_labels(
                [components.labels[int(candidate_row), int(candidate_col)]]
            )
            return self.component_evidence(message, components, raw_evidence)
        return None


