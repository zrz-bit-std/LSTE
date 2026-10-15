"""Topology and geometry association for persistent frontier places."""

import math

from global_frontier_place_memory_physical import as_xy

from global_frontier_place_states import (
    PLACE_DORMANT,
    PLACE_OPEN,
    PLACE_READY_TO_EXIT,
    PLACE_SUSPENDED,
)
from global_frontier_topology import same_topology_component


class FrontierRegionAssociationMixin:

    @staticmethod
    def _component_bbox_iou(first, second):
        left = max(float(first["min_x"]), float(second["min_x"]))
        right = min(float(first["max_x"]), float(second["max_x"]))
        bottom = max(float(first["min_y"]), float(second["min_y"]))
        top = min(float(first["max_y"]), float(second["max_y"]))
        intersection = max(0.0, right - left) * max(0.0, top - bottom)
        first_area = max(0.0, float(first["max_x"]) - float(first["min_x"])) * max(
            0.0, float(first["max_y"]) - float(first["min_y"])
        )
        second_area = max(0.0, float(second["max_x"]) - float(second["min_x"])) * max(
            0.0, float(second["max_y"]) - float(second["min_y"])
        )
        union = first_area + second_area - intersection
        return 0.0 if union <= 1e-9 else intersection / union

    def _component_match(self, region, component):
        previous = region.get("component")
        if previous is None or component is None:
            return None
        same_epoch = int(previous.get("epoch", -1)) == int(component.get("epoch", -2))
        if same_epoch:
            if int(previous.get("label", -1)) == int(component.get("label", -2)):
                return "component_exact"
            # Labels have one authoritative meaning inside a snapshot. Do not
            # let a similar-looking centroid merge two components separated by
            # a known wall or doorway in that same map.
            return None
        if (
            int(previous.get("cells", 0)) < self.component_min_cells
            or int(component.get("cells", 0)) < self.component_min_cells
        ):
            return None
        center_distance = math.hypot(
            float(previous["center_x"]) - float(component["center_x"]),
            float(previous["center_y"]) - float(component["center_y"]),
        )
        if center_distance > self.component_match_distance:
            return None
        area_ratio = min(float(previous["cells"]), float(component["cells"])) / max(
            float(previous["cells"]), float(component["cells"])
        )
        # Stable component centres with broadly comparable unknown extent are
        # enough to bridge a normal SLAM update.  Overlap gives a second path
        # for a partially scanned room whose centre shifts toward one wall.
        if area_ratio >= 0.20 or self._component_bbox_iou(previous, component) >= 0.12:
            return "component_signature"
        return None

    def _physical_nearest(self, physical_xy, component=None):
        """Match reached physical anchors across SLAM map-coordinate changes.

        Current-snapshot component labels are authoritative when available:
        nearby odom points must never merge two labels separated by a wall in
        that same snapshot.  Across snapshots, the anchor is deliberately
        stronger than stale map coordinates.
        """
        physical_xy = as_xy(physical_xy)
        if physical_xy is None:
            return None
        nearest = None
        nearest_distance = float("inf")
        for region in self.regions:
            previous = region.get("component")
            if component is not None and previous is not None:
                try:
                    same_epoch = int(previous["epoch"]) == int(component["epoch"])
                    same_label = int(previous["label"]) == int(component["label"])
                except (KeyError, TypeError, ValueError):
                    same_epoch = False
                    same_label = False
                if same_epoch and not same_label:
                    # Two labels in one snapshot name two wall-bounded cores.
                    # Do not let geometry bridge the known separator.
                    continue
            anchors = list(region.get("physical_viewpoints", []))
            if not anchors:
                anchors = list(region.get("physical_arrival_points", []))
            for anchor in anchors:
                anchor = as_xy(anchor)
                if anchor is None:
                    continue
                distance = math.hypot(
                    physical_xy[0] - anchor[0], physical_xy[1] - anchor[1]
                )
                if (
                    distance <= self.physical_region_match_radius
                    and distance < nearest_distance
                ):
                    nearest = region
                    nearest_distance = distance
        if nearest is None:
            return None
        return nearest, "physical_odom_anchor"

    @staticmethod
    def _portal_direction_compatible(
        recorded_inside, proposed_inside, gate,
        recorded_normal=None, proposed_normal=None,
    ):
        """Match noisy doorway entries by their inward half-plane.

        Map-to-odom correction can move the arrival standoff laterally while
        preserving the side of the doorway on which the robot entered.  The
        dominant cardinal direction is therefore a better identity cue than a
        raw dot product between two noisy standoff vectors.  The reverse side
        of the same gate still has the opposite dominant direction and cannot
        match.
        """
        if recorded_normal is not None and proposed_normal is not None:
            normal_dot = (
                float(recorded_normal[0]) * float(proposed_normal[0])
                + float(recorded_normal[1]) * float(proposed_normal[1])
            )
            return normal_dot > 0.0

        recorded = (
            float(recorded_inside[0]) - float(gate[0]),
            float(recorded_inside[1]) - float(gate[1]),
        )
        proposed = (
            float(proposed_inside[0]) - float(gate[0]),
            float(proposed_inside[1]) - float(gate[1]),
        )
        recorded_axis = 0 if abs(recorded[0]) >= abs(recorded[1]) else 1
        proposed_axis = 0 if abs(proposed[0]) >= abs(proposed[1]) else 1
        return (
            recorded_axis == proposed_axis
            and recorded[recorded_axis] * proposed[proposed_axis] > 0.0
        )

    def _portal_entry_nearest(
        self, gate_xy, inside_xy, physical_gate_xy=None,
        physical_inside_xy=None, normal_xy=None,
    ):
        """Resolve an arrival through a known physical doorway first.

        A Portal is a durable graph edge.  Reaching the same destination side
        of that edge must reuse the existing Place even when the transient
        structural label or the exact standoff point changed.  This lookup is
        intentionally performed before component and viewpoint association.
        """
        frames = []
        physical_gate = as_xy(physical_gate_xy)
        physical_inside = as_xy(physical_inside_xy)
        if physical_gate is not None and physical_inside is not None:
            frames.append((physical_gate, physical_inside, "physical_portal_entry"))
        map_gate = as_xy(gate_xy)
        map_inside = as_xy(inside_xy)
        if map_gate is not None and map_inside is not None:
            frames.append((map_gate, map_inside, "map_portal_entry"))
        if not frames:
            return None

        best = None
        best_distance = float("inf")
        for gate, inside, basis in frames:
            for region in self.regions:
                for entry in region.get("entry_portals", ()):
                    if not isinstance(entry, dict):
                        continue
                    recorded_gate = as_xy(
                        entry.get("physical_gate")
                        if basis == "physical_portal_entry"
                        else entry.get("gate")
                    )
                    recorded_inside = as_xy(
                        entry.get("physical_inside")
                        if basis == "physical_portal_entry"
                        else entry.get("inside")
                    )
                    if recorded_gate is None or recorded_inside is None:
                        continue
                    distance = math.hypot(
                        gate[0] - recorded_gate[0],
                        gate[1] - recorded_gate[1],
                    )
                    if distance > self.portal_entry_match_radius:
                        continue
                    if not self._portal_direction_compatible(
                        recorded_inside,
                        inside,
                        gate,
                        recorded_normal=entry.get("normal"),
                        proposed_normal=normal_xy,
                    ):
                        continue
                    if distance < best_distance:
                        best = region, basis
                        best_distance = distance
        return best

    def _nearest(self, x, y, component=None, physical_xy=None):
        if component is not None:
            component_matches = []
            for region in self.regions:
                basis = self._component_match(region, component)
                if basis is not None:
                    component_matches.append((region, basis))
            if component_matches:
                # An exact current-map identity is stronger than an older
                # signature.  Within one type, preserve the most recently
                # selected region to keep route lifecycle association stable.
                component_matches.sort(
                    key=lambda item: (
                        item[1] != "component_exact",
                        -float(item[0]["last_selected"]),
                    )
                )
                return component_matches[0]
        physical_match = self._physical_nearest(physical_xy, component)
        if physical_match is not None:
            return physical_match
        if component is not None:
            # When both candidate and remembered regions carry topology
            # evidence, an *open* region must not silently override that
            # disagreement with Euclidean proximity. That would merge a room
            # and its adjacent corridor before either has been observed.
            legacy_regions = [
                region for region in self.regions
                if region.get("component") is None
            ]
            # Completion has a different semantic meaning from an open
            # selection. A desk or conference table can split one physical
            # room into multiple high-clearance cores, so a later frontier
            # within the already observed local envelope is not a new room.
            # Restrict this fallback to *dormant* regions: nearby open
            # components still remain independent candidates until one has
            # actually been inspected.
            completed_nearest = None
            completed_distance = float("inf")
            for region in self.regions:
                if region.get("state") != PLACE_DORMANT:
                    continue
                distance = math.hypot(float(x) - region["x"], float(y) - region["y"])
                if distance <= self.radius and distance < completed_distance:
                    completed_nearest = region
                    completed_distance = distance
            if completed_nearest is not None:
                return completed_nearest, "completed_observation_envelope"
        else:
            legacy_regions = list(self.regions)
        nearest = None
        nearest_distance = float("inf")
        for region in legacy_regions:
            distance = math.hypot(float(x) - region["x"], float(y) - region["y"])
            if distance <= self.radius and distance < nearest_distance:
                nearest = region
                nearest_distance = distance
        return (nearest, "geometry") if nearest is not None else (None, "new")

    def _resolve(self, region_id, x, y, component=None, physical_xy=None):
        """Prefer a route-bound identity; coordinates are only a fallback."""
        region = self.by_id(region_id)
        if region is not None:
            return region, "route_id"
        return self._nearest(x, y, component, physical_xy=physical_xy)

    def candidate_tier(self, x, y, information, component=None, physical_xy=None):
        """Classify a candidate without mutating memory during score scans."""
        region, _basis = self._nearest(
            x, y, component, physical_xy=physical_xy,
        )
        if region is None:
            return "new", None
        if region["state"] == PLACE_DORMANT:
            return "dormant", region
        if region["state"] == PLACE_READY_TO_EXIT:
            return "ready_to_exit", region
        return "revisit", region

    @staticmethod
    def is_observed_region_reentry(region, source_component, candidate_component):
        """Return whether an ordinary revisit would physically re-enter a place.

        An ``open`` region can have two very different meanings.  Before a
        real endpoint observation it is merely a reserved route destination
        and may be selected again after a map refresh.  After an observation,
        however, selecting the same region from another doorway-separated
        structural component is a physical return to a place the robot has
        already inspected.  That is not ordinary local coverage and must not
        compete in the score pool with genuinely unexplored places.

        Both component descriptors must come from the *current* map epoch.
        Without that proof the method fails open: SLAM may temporarily lack a
        high-clearance core near a frontier, and guessing across a wall would
        be worse than retaining one conservative candidate.
        """
        if (
            region is None
            or region.get("state") not in (
                PLACE_OPEN,
                PLACE_READY_TO_EXIT,
                PLACE_SUSPENDED,
            )
            or int(region.get("endpoint_observations", 0)) < 1
            or source_component is None
            or candidate_component is None
        ):
            return False
        try:
            if int(source_component["epoch"]) != int(candidate_component["epoch"]):
                return False
        except (KeyError, TypeError, ValueError):
            return False
        return not same_topology_component(source_component, candidate_component)
