"""Read/query operations over persistent frontier-place memory."""

from global_frontier_place_memory_physical import as_xy, project_points

from global_frontier_place_states import (
    PLACE_DORMANT,
    PLACE_OPEN,
    PLACE_READY_TO_EXIT,
    PLACE_SUSPENDED,
)
from global_frontier_topology import copy_component_evidence


class FrontierRegionQueryMixin:

    def by_id(self, region_id):
        """Return an exact region identity, or ``None`` for legacy callers."""
        if region_id is None:
            return None
        try:
            expected = int(region_id)
        except (TypeError, ValueError):
            return None
        for region in self.regions:
            if int(region.get("id", -1)) == expected:
                return region
        return None

    def dormant_component_labels(self, epoch):
        """Return closed structural-place labels for one map snapshot."""
        labels = set()
        for region in self.regions:
            component = region.get("component")
            if (
                region.get("state") == PLACE_DORMANT
                and component is not None
                and int(component.get("epoch", -1)) == int(epoch)
            ):
                labels.add(int(component["label"]))
        return labels

    def observed_open_component_labels(self, epoch):
        """Return current labels for places observed but not yet closed.

        A successful endpoint observation is durable evidence.  ``open``
        regions with an observation and ``ready_to_exit`` regions both remain
        graph barriers until physical departure closes them; otherwise a later
        planning cycle could drive back through the same room.
        """
        labels = set()
        for region in self.regions:
            component = region.get("component")
            if (
                region.get("state") in (
                    PLACE_OPEN,
                    PLACE_READY_TO_EXIT,
                    PLACE_SUSPENDED,
                )
                and int(region.get("endpoint_observations", 0)) > 0
                and component is not None
                and int(component.get("epoch", -1)) == int(epoch)
            ):
                labels.add(int(component["label"]))
        return labels

    def dormant_viewpoints(self):
        """Return per-place anchors for rebuilding closed observation submaps."""
        places = []
        for region in self.regions:
            if region.get("state") != PLACE_DORMANT:
                continue
            anchors = self.viewpoints_for(region["id"])
            if anchors:
                places.append((int(region["id"]), anchors))
        return places

    def covered_viewpoints(self):
        """Return observation anchors independently of a place's topology state.

        Furniture may split a high-clearance topology component even though a
        reached lidar viewpoint already saw the candidate through known free
        space.  This coverage proof belongs to every observed place, whether
        it is still locally open or has since been physically departed.
        """
        places = []
        for region in self.regions:
            if int(region.get("endpoint_observations", 0)) < 1:
                continue
            anchors = self.viewpoints_for(region["id"])
            if anchors:
                places.append((int(region["id"]), anchors))
        return places

    def physical_coverage_region(self, physical_xy, radius):
        """Return observed-place coverage near an odom point without merging IDs.

        This query is intentionally narrower than normal physical place
        association. It is used only to refuse a proposed portal endpoint
        already covered by a real robot viewpoint, so a same-snapshot label
        split around furniture cannot create a duplicate doorway action.
        """
        point = as_xy(physical_xy)
        if point is None:
            return None
        maximum = max(0.05, float(radius))
        nearest = None
        nearest_distance = float("inf")
        for region in self.regions:
            if int(region.get("endpoint_observations", 0)) < 1:
                continue
            for anchor in region.get("physical_viewpoints", []):
                anchor = as_xy(anchor)
                if anchor is None:
                    continue
                distance = ((point[0] - anchor[0]) ** 2 + (point[1] - anchor[1]) ** 2) ** 0.5
                if distance <= maximum and distance < nearest_distance:
                    nearest = region
                    nearest_distance = distance
        return nearest

    def ready_to_exit_viewpoints(self):
        """Compatibility alias for the former endpoint-completion API."""
        return self.covered_viewpoints()

    def viewpoints_for(self, region_id):
        """Return the reached map-frame viewpoints for one exact region.

        Arrival and selected-goal coordinates are deliberately excluded.  A
        point becomes coverage evidence only after the controller reaches the
        observation viewpoint and ``observe`` records it.
        """
        region = self.by_id(region_id)
        if region is None:
            return []
        return [(float(x), float(y)) for x, y in region.get("viewpoints", [])]

    @staticmethod
    def _portal_coordinates(entry, physical=False):
        """Return one portal fact in its requested coordinate frame."""
        if not isinstance(entry, dict):
            return None, None
        prefix = "physical_" if physical else ""
        return as_xy(entry.get(prefix + "gate")), as_xy(entry.get(prefix + "inside"))

    def sealed_portal_entry(
        self, gate_xy, destination_xy, physical_gate_xy=None,
        physical_destination_xy=None,
    ):
        """Return a dormant place when a route re-enters through its known gate.

        A room can have several entrances and its current structural component
        can change as SLAM fills in furniture. The gate is therefore the
        durable topological identity. The dot-product check distinguishes
        re-entry from a legal route that leaves the same room through that
        doorway in the reverse direction.
        """
        gate = as_xy(gate_xy)
        destination = as_xy(destination_xy)
        physical_gate = as_xy(physical_gate_xy)
        physical_destination = as_xy(physical_destination_xy)
        if gate is None or destination is None:
            return None
        use_physical = physical_gate is not None and physical_destination is not None
        candidate_gate = physical_gate if use_physical else gate
        candidate_destination = physical_destination if use_physical else destination
        gate_x, gate_y = candidate_gate
        destination_x, destination_y = candidate_destination
        best = None
        best_distance = float("inf")
        for region in self.regions:
            if region.get("state") != PLACE_DORMANT:
                continue
            for entry in region.get("entry_portals", []):
                entry_gate, inside = self._portal_coordinates(
                    entry, physical=use_physical,
                )
                if entry_gate is None or inside is None:
                    # Legacy facts remain map-only until the next verified
                    # crossing stores physical evidence.
                    entry_gate, inside = self._portal_coordinates(entry)
                if entry_gate is None or inside is None:
                    continue
                entry_gate_x, entry_gate_y = entry_gate
                inside_x, inside_y = inside
                distance = ((gate_x - entry_gate_x) ** 2 + (gate_y - entry_gate_y) ** 2) ** 0.5
                if distance > self.portal_entry_match_radius:
                    continue
                entered_x, entered_y = inside_x - entry_gate_x, inside_y - entry_gate_y
                proposed_x, proposed_y = destination_x - gate_x, destination_y - gate_y
                if (
                    entered_x * entered_x + entered_y * entered_y < 1e-6
                    or proposed_x * proposed_x + proposed_y * proposed_y < 1e-6
                    or entered_x * proposed_x + entered_y * proposed_y <= 0.0
                ):
                    continue
                if distance < best_distance:
                    best = region, entry
                    best_distance = distance
        return best

    def portal_exit_from_entry(
        self, gate_xy, source_xy, destination_xy, physical_gate_xy=None,
        physical_source_xy=None, physical_destination_xy=None,
    ):
        """Return the remembered source place for a reverse doorway crossing.

        A dormant destination normally rejects a portal action. The one safe
        exception is an outward crossing through the current place's recorded
        entrance: the robot is on the ``inside`` side of that gate and the
        proposed endpoint lies on its opposite side. This is graph traversal,
        not permission to revisit a covered room.
        """
        gate = as_xy(gate_xy)
        source = as_xy(source_xy)
        destination = as_xy(destination_xy)
        physical_gate = as_xy(physical_gate_xy)
        physical_source = as_xy(physical_source_xy)
        physical_destination = as_xy(physical_destination_xy)
        if gate is None or source is None or destination is None:
            return None
        use_physical = (
            physical_gate is not None
            and physical_source is not None
            and physical_destination is not None
        )
        gate_x, gate_y = physical_gate if use_physical else gate
        source_x, source_y = physical_source if use_physical else source
        destination_x, destination_y = (
            physical_destination if use_physical else destination
        )
        best = None
        best_distance = float("inf")
        for region in self.regions:
            for entry in region.get("entry_portals", []):
                entry_gate, inside = self._portal_coordinates(
                    entry, physical=use_physical,
                )
                if entry_gate is None or inside is None:
                    entry_gate, inside = self._portal_coordinates(entry)
                if entry_gate is None or inside is None:
                    continue
                entry_gate_x, entry_gate_y = entry_gate
                inside_x, inside_y = inside
                distance = (
                    (gate_x - entry_gate_x) ** 2
                    + (gate_y - entry_gate_y) ** 2
                ) ** 0.5
                if distance > self.portal_entry_match_radius:
                    continue
                inward_x, inward_y = inside_x - entry_gate_x, inside_y - entry_gate_y
                source_vector_x, source_vector_y = source_x - gate_x, source_y - gate_y
                destination_vector_x = destination_x - gate_x
                destination_vector_y = destination_y - gate_y
                if (
                    inward_x * inward_x + inward_y * inward_y < 1e-6
                    or inward_x * source_vector_x + inward_y * source_vector_y <= 0.0
                    or inward_x * destination_vector_x
                    + inward_y * destination_vector_y >= 0.0
                ):
                    continue
                if distance < best_distance:
                    # New portal entries carry the directed graph source. The
                    # entry record lives on the destination Place, so returning
                    # ``region`` here would turn a reverse edge into
                    # Place->same-Place and make valid egress look like a
                    # self-loop. Keep the legacy fallback for older in-memory
                    # records that predate source identity.
                    source_place_id = entry.get("source_place_id")
                    source_region = None
                    if source_place_id is not None:
                        try:
                            source_region = self.by_id(int(source_place_id))
                        except (TypeError, ValueError):
                            source_region = None
                    best = (
                        region if source_region is None else source_region,
                        entry,
                    )
                    best_distance = distance
        return best

    def refresh_components(self, component_at_xy, map_from_physical_xy=None):
        """Rebind stored regions to the current SLAM topology snapshot.

        A selected frontier endpoint often sits beside an unknown boundary or
        in a doorway throat, neither of which is guaranteed to have a
        high-clearance topology label in the next SLAM snapshot.  A viewpoint
        reached by the robot is stronger evidence: it is both a physical fact
        and normally inside the room core.  Rebind from those observation
        anchors first, then fall back to portal arrivals and the old selected
        endpoint.  This keeps a covered room identifiable when the transient
        frontier cell loses its label.

        ``component_at_xy`` must return either a current-snapshot component
        descriptor or ``None``.  A missing answer deliberately preserves the
        previous identity: an unknown cell must never be associated through a
        wall just to make region memory more aggressive.
        """
        refreshed = []
        for region in self.regions:
            # Projection does not alter physical history. It refreshes the
            # map-frame evidence consumed by Navfn-facing coverage and the
            # structural component lookup below.
            projected_viewpoints = project_points(
                region.get("physical_viewpoints", []), map_from_physical_xy,
            )
            if projected_viewpoints:
                region["viewpoints"] = projected_viewpoints
            projected_arrivals = project_points(
                region.get("physical_arrival_points", []), map_from_physical_xy,
            )
            if projected_arrivals:
                region["arrival_points"] = projected_arrivals
            if map_from_physical_xy is not None:
                for entry in region.get("entry_portals", []):
                    if not isinstance(entry, dict):
                        continue
                    physical_gate = as_xy(entry.get("physical_gate"))
                    physical_inside = as_xy(entry.get("physical_inside"))
                    if physical_gate is None or physical_inside is None:
                        continue
                    projected_gate = as_xy(
                        map_from_physical_xy(physical_gate[0], physical_gate[1])
                    )
                    projected_inside = as_xy(
                        map_from_physical_xy(physical_inside[0], physical_inside[1])
                    )
                    if projected_gate is not None and projected_inside is not None:
                        entry["gate"] = list(projected_gate)
                        entry["inside"] = list(projected_inside)
            # Coverage viewpoints prove where the base actually stood. Newer
            # anchors are usually the best representation of the current
            # local room core, while portal/endpoint coordinates are retained
            # for a place that has not yet received an observation route.
            anchors = list(reversed(region.get("viewpoints", [])))
            anchors.extend(reversed(region.get("arrival_points", [])))
            anchors.append((region["x"], region["y"]))
            component = None
            for anchor_x, anchor_y in anchors:
                component = component_at_xy(anchor_x, anchor_y)
                if component is not None:
                    break
            if component is None:
                continue
            previous = region.get("component")
            copied = copy_component_evidence(component)
            if (
                previous is None
                or int(previous.get("epoch", -1)) != int(copied["epoch"])
                or int(previous.get("label", -1)) != int(copied["label"])
            ):
                refreshed.append(int(region["id"]))
            region["component"] = copied
            region["last_association"] = "current_map_rehydrated"
        return refreshed
