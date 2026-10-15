"""Viewpoint coverage checks that suppress only proven duplicate observations."""

import math

from global_frontier_topology import grid_line_is_known_free


class GlobalFrontierObservationViewpointMixin:

    def completed_viewpoint_component(self, message, components, x, y):
        """Find one completed viewpoint's component in the current snapshot."""
        cell = self.xy_to_grid_cell(message, x, y)
        if cell is None:
            return None
        cached = self.completed_viewpoint_component_cache.get(cell, False)
        if cached is not False:
            return cached
        radius_cells = int(math.ceil(
            self.region_topology_association_radius / message.info.resolution
        ))
        raw = components.nearby_evidence(cell[0], cell[1], radius_cells)
        evidence = self.component_evidence(message, components, raw)
        # Cache ``None`` as well: a viewpoint outside a topology core should
        # not trigger repeated neighborhood searches for every candidate.
        self.completed_viewpoint_component_cache[cell] = evidence
        return evidence

    def candidate_covered_by_covered_place_viewpoint(
        self, message, known_free, row, col, x, y,
    ):
        """Return coverage evidence that already sees an endpoint approach.

        This is intentionally independent of a structural-component label.
        An office desk can divide one room into two high-clearance cores, but
        it cannot make a known-free lidar ray disappear.  The ray proof keeps
        visible duplicate viewpoints out of the selector while leaving an
        occluded branch eligible for a genuine additional observation.
        """
        if self.scan_range_max is None or self.scan_range_max <= 0.0:
            return None
        memory = getattr(self, "region_memory", None)
        if memory is None:
            return None
        covered_viewpoints = getattr(memory, "covered_viewpoints", None)
        if covered_viewpoints is None:
            return None
        candidate_cell = (int(row), int(col))
        for region_id, anchors in covered_viewpoints():
            for old_x, old_y in anchors:
                if math.hypot(float(x) - old_x, float(y) - old_y) > self.scan_range_max:
                    continue
                old_cell = self.xy_to_grid_cell(message, old_x, old_y)
                if old_cell is None:
                    continue
                if grid_line_is_known_free(
                    known_free,
                    old_cell[0],
                    old_cell[1],
                    candidate_cell[0],
                    candidate_cell[1],
                ):
                    return int(region_id), (float(old_x), float(old_y))
        return None

    def candidate_covered_by_ready_place_viewpoint(
        self, message, known_free, row, col, x, y,
    ):
        """Compatibility alias for callers using the former lifecycle name."""
        return self.candidate_covered_by_covered_place_viewpoint(
            message, known_free, row, col, x, y,
        )

    def candidate_covered_by_completed_viewpoint(
        self, message, known_free, components, row, col, x, y,
    ):
        """Return a completed viewpoint that already has a valid view of a candidate.

        This is a small, 2-D form of next-best-view coverage accounting.  It
        needs three independent proofs before suppressing a candidate:

        1. it is inside the current LiDAR range of a viewpoint actually reached;
        2. both points have the same current doorway-separated topology label;
        3. their grid ray is already known free.

        The combined test deliberately keeps a frontier behind furniture, a
        wall, or a doorway eligible.  It only removes an observation point that
        would re-enter an already visible part of the same room.
        """
        if (
            components is None
            or self.scan_range_max is None
            or self.scan_range_max <= 0.0
            or not self.completed_frontiers
        ):
            return None
        radius_cells = int(math.ceil(
            self.region_topology_association_radius / message.info.resolution
        ))
        candidate_raw = components.nearby_evidence(row, col, radius_cells)
        if candidate_raw is None:
            return None
        candidate_label = int(candidate_raw["label"])
        candidate_epoch = int(candidate_raw["epoch"])
        candidates = sorted(
            self.completed_frontiers,
            key=lambda point: math.hypot(float(x) - point[0], float(y) - point[1]),
        )
        candidate_cell = (int(row), int(col))
        for old_x, old_y in candidates:
            if math.hypot(float(x) - old_x, float(y) - old_y) > self.scan_range_max:
                # The list is distance-sorted, so all remaining viewpoints are
                # beyond this sensor's actual observation horizon.
                break
            old_component = self.completed_viewpoint_component(
                message, components, old_x, old_y,
            )
            if (
                old_component is None
                or int(old_component["epoch"]) != candidate_epoch
                or int(old_component["label"]) != candidate_label
            ):
                continue
            old_cell = self.xy_to_grid_cell(message, old_x, old_y)
            if old_cell is None:
                continue
            if grid_line_is_known_free(
                known_free,
                old_cell[0],
                old_cell[1],
                candidate_cell[0],
                candidate_cell[1],
            ):
                return float(old_x), float(old_y)
        return None



