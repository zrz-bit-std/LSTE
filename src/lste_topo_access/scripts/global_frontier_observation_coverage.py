"""Observation coverage and endpoint-state helpers for frontier exploration.

The methods here decide whether a viewpoint has already observed an area and
record a completed endpoint.  Physical departure is deliberately elsewhere.
"""

import math

import numpy as np
import rospy

from clock_provider import now_for
from global_frontier_models import ObservationDepartureSource
from global_frontier_topology import grid_visible_free_footprint


class GlobalFrontierObservationCoverageMixin:

    def _frontier_has_unknown(self, unknown, row, col):
        """Return whether unknown space remains within the dead-end margin.

        A genuine passage (doorway / room entrance) has unknown space extending
        beyond it, so the margin around the frontier cell still contains
        unknown.  A resolved dead-end wall has no unknown anywhere near the
        cell -- only scanned free space and the wall itself.
        """
        radius = self.dead_end_unknown_cells
        r0, r1 = max(0, row - radius), min(unknown.shape[0], row + radius + 1)
        c0, c1 = max(0, col - radius), min(unknown.shape[1], col + radius + 1)
        return bool(np.any(unknown[r0:r1, c0:c1]))

    def frontier_structure(self, occupied, row, col):
        radius = self.structure_radius
        r0, r1 = max(0, row - radius), min(occupied.shape[0], row + radius + 1)
        c0, c1 = max(0, col - radius), min(occupied.shape[1], col + radius + 1)
        return float(np.count_nonzero(occupied[r0:r1, c0:c1]))

    @staticmethod
    def nearest_safe_approach(
        steps, row, col, max_cells, preferred_steps=None, preferred_mask=None,
        selection_tier="strict_clearance", standoff_cells=0,
    ):
        """Find an approach point with an explicit endpoint/path contract.

        ``steps`` is the loose known-free observation topology. A normal
        frontier route must also be connected through ``preferred_steps``. A
        recovery route may cross a narrow map region only when its *endpoint*
        remains inside ``preferred_mask``; Navfn then validates that exact
        route. It must never publish a loose, wall-adjacent endpoint merely
        because Navfn found a path close to it.

        When ``standoff_cells`` is set, walk the certified BFS predecessor
        chain back from the boundary before considering the endpoint. A
        frontier is an information boundary, not a docking marker: the
        observation action should stop at the existing sensor-scale approach
        distance. This keeps a newly observed wall from becoming a goal that
        TEB can validate only with a near-zero command.
        """
        rows, cols = steps.shape

        try:
            standoff_cells = max(0, int(standoff_cells))
        except (TypeError, ValueError):
            standoff_cells = 0
        if standoff_cells > 0 and selection_tier == "strict_clearance":
            frontier = (int(row), int(col))
            if (
                0 <= frontier[0] < rows
                and 0 <= frontier[1] < cols
                and int(steps[frontier]) >= standoff_cells
            ):
                current = frontier
                target_step = int(steps[frontier]) - standoff_cells
                while int(steps[current]) > target_step:
                    current_step = int(steps[current])
                    predecessors = []
                    for delta_row, delta_col in (
                        (1, 0), (-1, 0), (0, 1), (0, -1),
                    ):
                        predecessor = (
                            current[0] + delta_row,
                            current[1] + delta_col,
                        )
                        if (
                            0 <= predecessor[0] < rows
                            and 0 <= predecessor[1] < cols
                            and 0 <= int(steps[predecessor]) < current_step
                        ):
                            predecessors.append(predecessor)
                    if not predecessors:
                        break
                    current = min(
                        predecessors,
                        key=lambda cell: (int(steps[cell]), cell[0], cell[1]),
                    )
                # Door throats can be absent from the strict mask. Continue
                # toward the robot until the first strict route cell appears;
                # this is a topology fact, not another distance threshold.
                while True:
                    if (
                        int(steps[current]) >= 0
                        and (
                            preferred_steps is None
                            or int(preferred_steps[current]) >= 0
                        )
                    ):
                        return int(current[0]), int(current[1])
                    current_step = int(steps[current])
                    if current_step <= 0:
                        break
                    predecessors = []
                    for delta_row, delta_col in (
                        (1, 0), (-1, 0), (0, 1), (0, -1),
                    ):
                        predecessor = (
                            current[0] + delta_row,
                            current[1] + delta_col,
                        )
                        if (
                            0 <= predecessor[0] < rows
                            and 0 <= predecessor[1] < cols
                            and 0 <= int(steps[predecessor]) < current_step
                        ):
                            predecessors.append(predecessor)
                    if not predecessors:
                        break
                    current = min(
                        predecessors,
                        key=lambda cell: (int(steps[cell]), cell[0], cell[1]),
                    )

        r0 = max(0, row - max_cells)
        r1 = min(rows, row + max_cells + 1)
        c0 = max(0, col - max_cells)
        c1 = min(cols, col + max_cells + 1)
        candidates = np.argwhere(steps[r0:r1, c0:c1] >= 0)
        if candidates.size == 0:
            return None
        candidates[:, 0] += r0
        candidates[:, 1] += c0
        if selection_tier == "strict_clearance":
            if preferred_steps is None:
                return None
            strict = preferred_steps[candidates[:, 0], candidates[:, 1]] >= 0
            candidates = candidates[strict]
        elif selection_tier == "navfn_observation_recovery":
            if preferred_mask is None:
                return None
            recovery = preferred_mask[candidates[:, 0], candidates[:, 1]]
            # A recovery is meaningful only when the endpoint cannot already
            # be reached through the strict topology. Otherwise the strict
            # first pass owns this candidate and its audit identity.
            if preferred_steps is not None:
                recovery &= preferred_steps[candidates[:, 0], candidates[:, 1]] < 0
            candidates = candidates[recovery]
        else:
            return None
        if candidates.size == 0:
            return None
        distance = (candidates[:, 0] - row) ** 2 + (candidates[:, 1] - col) ** 2
        # Prefer the closest safe cell, then the shortest route from the
        # robot.  The latter keeps ties from selecting a remote branch.
        order = np.lexsort((steps[candidates[:, 0], candidates[:, 1]], distance))
        selected = candidates[int(order[0])]
        return int(selected[0]), int(selected[1])

    def frontier_is_rejected(self, x, y, now):
        while (
            self.rejected_frontiers
            and now - self.rejected_frontiers[0][0] > self.rejected_timeout
        ):
            self.rejected_frontiers.popleft()
        return any(math.hypot(x - old_x, y - old_y) < 1.5 for _, old_x, old_y in self.rejected_frontiers)

    def frontier_is_completed(self, x, y):
        return any(
            math.hypot(x - old_x, y - old_y) < self.completed_radius
            for old_x, old_y in self.completed_frontiers
        )

    def activate_frontier_region(
        self, x, y, information, now, transition, component=None,
        physical_place_id=None,
    ):
        """Commit the region only after Navfn accepted this frontier route."""
        region = self.region_memory.by_id(physical_place_id)
        if region is not None:
            if region.get("state") not in ("open", "ready_to_exit", "suspended"):
                region, tier = None, region.get("state", "unavailable")
            else:
                # The route is certified local to this physical Place. Do not
                # replace its identity with a component split by furniture.
                if region.get("state") == "suspended":
                    # Selecting a durable local WorkItem is the only action
                    # that resumes a branch-first Place. Transit never calls
                    # this path, so a covered room cannot reopen accidentally.
                    resume = getattr(self.region_memory, "resume", None)
                    if callable(resume):
                        resume(region, now)
                    else:
                        # Compatibility for narrow injected memory fixtures.
                        region["state"] = "open"
                        region["last_reason"] = "suspended_place_reopened"
                region["route_dispatches"] = int(
                    region.get("route_dispatches", 0)
                ) + 1
                region["last_selected"] = float(now)
                region["last_association"] = "current_physical_place_owner"
                tier = "revisit"
        else:
            region, tier = self.region_memory.activate(
                x, y, information, now, component=component,
            )
        if region is None:
            rospy.logwarn(
                "Global frontier refused dormant observation region at map=(%.2f,%.2f) "
                "transition=%s",
                x,
                y,
                transition,
            )
            return None
        self.last_frontier_region_tier = tier
        rospy.loginfo(
            "Global frontier region id=%d tier=%s state=%s visits=%d "
            "information=%.0f component_cells=%s association=%s transition=%s",
            region["id"],
            tier,
            region["state"],
            region["visits"],
            float(information),
            None if component is None else int(component["cells"]),
            region.get("last_association", "unknown"),
            transition,
        )
        return region

    def mark_frontier_observed(
        self, x, y, component=None, region_id=None, now=None,
        physical_xy=None,
    ):
        """Record a reached viewpoint and freeze ordinary place coverage."""
        if now is None:
            now = now_for(self)
        if physical_xy is None and getattr(self, "pose_odom", None) is not None:
            physical_xy = (
                float(self.pose_odom.x),
                float(self.pose_odom.y),
            )
        region = self.region_memory.endpoint_observed(
            x,
            y,
            now,
            component=component,
            region_id=region_id,
            retain_for_target=bool(self.target_region_claim_active),
            physical_xy=physical_xy,
        )
        if region is not None:
            settle_probe = getattr(self, "settle_active_portal_probe", None)
            source_probe_active = (
                getattr(self, "active_portal_probe_id", None) is not None
            )
            if settle_probe is not None:
                probe_ledger = getattr(self, "portal_probe_ledger", None)
                active_probe = (
                    None
                    if probe_ledger is None or not source_probe_active
                    else probe_ledger.get(self.active_portal_probe_id)
                )
                # The probe ledger is a typed transaction. A source-side
                # viewpoint only proves that the opening was reached; a
                # destination-side viewpoint is the distinct observation
                # terminal that settles the doorway WorkItem. The previous
                # default of ``source_arrived`` for every active probe made
                # destination attempts loop forever.
                if not source_probe_active:
                    probe_result = "observed"
                elif (
                    active_probe is not None
                    and active_probe.get("active_phase") == "destination"
                ):
                    probe_result = "observed"
                else:
                    # Missing phase metadata is handled conservatively as a
                    # source arrival; it must never silently certify an
                    # unobserved destination.
                    probe_result = "source_arrived"
                settle_probe(
                    probe_result,
                    now,
                    (
                        "source_viewpoint_arrived"
                        if probe_result == "source_arrived"
                        else "destination_viewpoint_observed"
                    ),
                )
            # A source-side probe is an Attempt against a Portal hypothesis.
            # Reaching its viewpoint does not observe the unknown side and
            # must not resolve the associated ObservationWorkItem.  The probe
            # ledger closes that item only after directed Portal evidence is
            # certified.
            settle = getattr(self, "settle_active_work_item", None)
            if settle is not None and not source_probe_active:
                settle(now, "resolved", "endpoint_coverage_recorded")
        if not self.frontier_is_completed(x, y):
            self.completed_frontiers.append((x, y))
            rospy.loginfo(
                "Global frontier endpoint observed at map=(%.2f,%.2f); "
                "remembered=%d region_id=%s active_region_id=%s "
                "endpoint_observations=%s state=%s reason=%s",
                x,
                y,
                len(self.completed_frontiers),
                None if region is None else region["id"],
                region_id,
                None if region is None else region["endpoint_observations"],
                None if region is None else region["state"],
                None if region is None else region["last_reason"],
            )
        if region is not None:
            self.publish_status(
                "frontier_endpoint_observed",
                reason=region["last_reason"],
                region_id=int(region["id"]),
                region_visits=int(region["visits"]),
                endpoint_observations=int(region["endpoint_observations"]),
                active_region_id=region_id,
                goal=[round(float(x), 3), round(float(y), 3)],
            )
        return region

    def remember_completed_observation_source(self, region):
        """Retain a reached place for the next outward route decision."""
        if (
            region is None
            or region.get("state") not in ("open", "ready_to_exit", "suspended")
            or region.get("entered_at") is None
            or int(region.get("endpoint_observations", 0)) < 1
        ):
            return
        self.observation_departure_source = ObservationDepartureSource(
            region_id=int(region["id"]),
            anchor_map=(float(region["x"]), float(region["y"])),
            route_id=int(self.active_route_id),
        )

    def observation_anchor_cells(self, message, known_free, anchors):
        """Map durable viewpoint coordinates onto current known-free cells."""
        cells = []
        for x, y in anchors:
            cell = self.xy_to_grid_cell(message, x, y)
            if cell is None:
                continue
            if not known_free[cell]:
                cell = self.nearest_seed(
                    known_free,
                    cell[0],
                    cell[1],
                    max(1, int(math.ceil(0.5 / message.info.resolution))),
                )
            if cell is not None:
                cells.append(cell)
        return cells

    def observation_footprint_from_anchors(
        self, message, known_free, anchor_cells, components=None,
    ):
        """Rebuild a wall-bounded submap from reached observation anchors."""
        footprint = np.zeros_like(known_free, dtype=bool)
        if (
            self.scan_range_max is None
            or self.scan_range_max <= 0.0
            or not anchor_cells
        ):
            return footprint
        max_range_cells = max(
            1, int(math.ceil(float(self.scan_range_max) / message.info.resolution)),
        )
        for cell in anchor_cells:
            local_footprint = grid_visible_free_footprint(
                known_free, [cell], max_range_cells,
            )
            if components is not None:
                # For an already closed structural place, retain the old
                # doorway cut so a ray through an open door cannot block the
                # adjacent corridor. The departure fallback intentionally
                # passes ``None`` here because it is used precisely when that
                # structural cut is not trustworthy yet.
                raw = components.nearby_evidence(
                    cell[0],
                    cell[1],
                    int(math.ceil(
                        self.region_topology_association_radius
                        / message.info.resolution
                    )),
                )
                if raw is not None:
                    local_footprint &= components.labels == int(raw["label"])
            footprint |= local_footprint
        return footprint

    @staticmethod
    def route_crosses_place_boundary(place_hops):
        """Return whether a selected action leaves the current place graph node."""
        try:
            return place_hops is not None and int(place_hops) >= 1
        except (TypeError, ValueError):
            return False
