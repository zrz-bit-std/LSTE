"""Directional doorway facts for persistent frontier-place memory."""

import math

from global_frontier_place_memory_physical import as_xy, append_spaced_anchor
from global_frontier_place_states import PLACE_OPEN


class FrontierRegionPortalMixin:
    """Record physical arrivals and entry gates without changing coverage."""

    @staticmethod
    def _portal_normal(gate_xy, component, inside_xy):
        """Infer a doorway's inward normal from structural bounds.

        Arrival standoffs are allowed to be lateral to a doorway.  A component
        bounding box gives a stable side-of-room normal, so two noisy entries
        through the same gate do not need to agree on the exact standoff ray.
        """
        try:
            gate_x, gate_y = float(gate_xy[0]), float(gate_xy[1])
        except (IndexError, TypeError, ValueError):
            return None
        if isinstance(component, dict):
            try:
                distances = (
                    (abs(gate_x - float(component["min_x"])), (1.0, 0.0)),
                    (abs(gate_x - float(component["max_x"])), (-1.0, 0.0)),
                    (abs(gate_y - float(component["min_y"])), (0.0, 1.0)),
                    (abs(gate_y - float(component["max_y"])), (0.0, -1.0)),
                )
                _distance, normal = min(distances, key=lambda item: item[0])
                return normal
            except (KeyError, TypeError, ValueError):
                pass
        inside = as_xy(inside_xy)
        if inside is None:
            return None
        dx, dy = inside[0] - gate_x, inside[1] - gate_y
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            return None
        return dx / length, dy / length

    def _append_entry_portal(
        self, region, gate_xy, inside_xy, physical_gate_xy=None,
        physical_inside_xy=None, normal_xy=None, source_place_id=None,
    ):
        """Store one directional entry edge, merging repeated door crossings."""
        if gate_xy is None or inside_xy is None:
            return None
        gate = as_xy(gate_xy)
        inside = as_xy(inside_xy)
        if gate is None or inside is None:
            return None
        gate_x, gate_y = gate
        inside_x, inside_y = inside
        if math.hypot(inside_x - gate_x, inside_y - gate_y) < 1e-3:
            return None
        portals = region.setdefault("entry_portals", [])
        for existing in portals:
            existing_gate = existing.get("gate") if isinstance(existing, dict) else None
            if existing_gate is None:
                continue
            if math.hypot(
                gate_x - float(existing_gate[0]), gate_y - float(existing_gate[1])
            ) < self.portal_entry_merge_radius:
                existing["inside"] = [inside_x, inside_y]
                physical_gate = as_xy(physical_gate_xy)
                physical_inside = as_xy(physical_inside_xy)
                if physical_gate is not None and physical_inside is not None:
                    existing["physical_gate"] = list(physical_gate)
                    existing["physical_inside"] = list(physical_inside)
                if normal_xy is not None:
                    existing["normal"] = list(normal_xy)
                if source_place_id is not None:
                    try:
                        existing["source_place_id"] = int(source_place_id)
                    except (TypeError, ValueError):
                        pass
                return existing
        entry = {"gate": [gate_x, gate_y], "inside": [inside_x, inside_y]}
        if source_place_id is not None:
            try:
                entry["source_place_id"] = int(source_place_id)
            except (TypeError, ValueError):
                pass
        physical_gate = as_xy(physical_gate_xy)
        physical_inside = as_xy(physical_inside_xy)
        if physical_gate is not None and physical_inside is not None:
            entry["physical_gate"] = list(physical_gate)
            entry["physical_inside"] = list(physical_inside)
        if normal_xy is not None:
            entry["normal"] = list(normal_xy)
        portals.append(entry)
        return entry

    def record_entry_portal(
        self, region_id, gate_xy, inside_xy, physical_gate_xy=None,
        physical_inside_xy=None, normal_xy=None, source_place_id=None,
    ):
        """Attach a verified entry edge to an already selected destination."""
        region = self.by_id(region_id)
        if region is None:
            return None
        return self._append_entry_portal(
            region,
            gate_xy,
            inside_xy,
            physical_gate_xy=physical_gate_xy,
            physical_inside_xy=physical_inside_xy,
            normal_xy=normal_xy,
            source_place_id=source_place_id,
        )

    def portal_entry_region(
        self, gate_xy, destination_xy, component=None,
        physical_gate_xy=None, physical_destination_xy=None,
    ):
        """Return the durable Place on a known doorway's destination side.

        This query is used before a new Portal action is dispatched.  Arrival
        association alone is too late: the selector must be able to reject a
        route that would re-enter an already represented destination Place.
        """
        normal_xy = self._portal_normal(gate_xy, component, destination_xy)
        return self._portal_entry_nearest(
            gate_xy,
            destination_xy,
            physical_gate_xy=physical_gate_xy,
            physical_inside_xy=physical_destination_xy,
            normal_xy=normal_xy,
        )

    def _record_portal_arrival(
        self, region, x, y, now, association, covered, entry_portal=None,
        physical_xy=None, physical_entry_portal=None, normal_xy=None,
        source_place_id=None,
    ):
        """Persist a physical doorway arrival without changing coverage state."""
        region["portal_arrivals"] = int(region.get("portal_arrivals", 0)) + 1
        region["physical_entry_count"] = int(
            region.get("physical_entry_count", 0)
        ) + 1
        region["last_arrived_at"] = float(now)
        if covered:
            region["covered_arrivals"] = int(region.get("covered_arrivals", 0)) + 1
            region["last_reason"] = "portal_arrival_already_covered"
        else:
            region["last_reason"] = "portal_arrival"
        region["last_association"] = association
        arrival = float(x), float(y)
        minimum_anchor_spacing = max(0.25, self.radius / 4.0)
        append_spaced_anchor(
            region.setdefault("arrival_points", []), arrival, minimum_anchor_spacing,
        )
        append_spaced_anchor(
            region.setdefault("physical_arrival_points", []),
            physical_xy,
            minimum_anchor_spacing,
        )
        self._append_entry_portal(
            region,
            entry_portal,
            arrival,
            physical_gate_xy=physical_entry_portal,
            physical_inside_xy=physical_xy,
            normal_xy=normal_xy,
            source_place_id=source_place_id,
        )

    def enter(
        self, x, y, now, component=None, entry_portal=None,
        physical_xy=None, physical_entry_portal=None, source_place_id=None,
        force_new=False,
    ):
        """Record a physical arrival in a place without claiming coverage.

        A portal transition terminates at the first safe core of the adjacent
        structural place. It proves the base crossed the doorway, but it does
        not prove that a frontier has been observed there.
        """
        normal_xy = self._portal_normal(entry_portal, component, (x, y))
        # A verified crossing with no bound destination is explicitly asking
        # for a new Place identity.  Reusing an entry-portal match here can
        # resolve the destination back to the source Place, which later turns
        # a physical crossing into a rejected self-loop.  Bound reverse
        # egresses use enter_bound_portal_destination and retain the normal
        # portal matcher, so this bypass is limited to force_new arrivals.
        portal_match = None
        if not force_new:
            portal_match = self._portal_entry_nearest(
                entry_portal,
                (x, y),
                physical_gate_xy=physical_entry_portal,
                physical_inside_xy=physical_xy,
                normal_xy=normal_xy,
            )
        if portal_match is not None:
            matched, association = portal_match
            covered = matched.get("state") != PLACE_OPEN
            self._record_portal_arrival(
                matched,
                x,
                y,
                now,
                association,
                covered=covered,
                entry_portal=entry_portal,
                physical_xy=physical_xy,
                physical_entry_portal=physical_entry_portal,
                normal_xy=normal_xy,
                source_place_id=source_place_id,
            )
            if not covered and matched.get("entered_at") is None:
                matched["entered_at"] = float(now)
            return matched, "covered_arrival" if covered else "portal_revisit"

        matched, association = (
            (None, "portal_ledger_identity")
            if force_new
            else self._nearest(x, y, component, physical_xy=physical_xy)
        )
        if matched is not None and matched.get("state") != PLACE_OPEN:
            # A route may legitimately traverse a covered room when the
            # online map changed after selection. Keep this fact for topology
            # auditing; selection, not arrival commit, owns prevention policy.
            self._record_portal_arrival(
                matched,
                x,
                y,
                now,
                association,
                covered=True,
                entry_portal=entry_portal,
                physical_xy=physical_xy,
                physical_entry_portal=physical_entry_portal,
                normal_xy=normal_xy,
                source_place_id=source_place_id,
            )
            return matched, "covered_arrival"

        region, tier = self.activate(
            x,
            y,
            0.0,
            now,
            component=component,
            physical_xy=physical_xy,
            force_new=force_new,
        )
        if region is None:
            return None, tier
        if region.get("entered_at") is None:
            region["entered_at"] = float(now)
        self._record_portal_arrival(
            region,
            x,
            y,
            now,
            region.get("last_association", tier),
            covered=False,
            entry_portal=entry_portal,
            physical_xy=physical_xy,
            physical_entry_portal=physical_entry_portal,
            normal_xy=normal_xy,
            source_place_id=source_place_id,
        )
        return region, tier

    def enter_new_portal_destination(
        self, x, y, now, component=None, entry_portal=None,
        physical_xy=None, physical_entry_portal=None, source_place_id=None,
    ):
        """Create a destination Place from a verified Portal identity.

        This path is used only when the graph has proved a physical crossing
        but the current structural map has not separated the two connected
        free-space cores.  Existing entry-portal matching still runs first,
        so a known destination is reused rather than duplicated.
        """
        return self.enter(
            x,
            y,
            now,
            component=component,
            entry_portal=entry_portal,
            physical_xy=physical_xy,
            physical_entry_portal=physical_entry_portal,
            source_place_id=source_place_id,
            force_new=True,
        )

    def enter_bound_portal_destination(
        self, destination_place_id, x, y, now, component=None,
        entry_portal=None, physical_xy=None, physical_entry_portal=None,
        source_place_id=None,
    ):
        """Record arrival for a destination named by the durable Portal edge.

        Reverse traversal presents the opposite half-plane from the original
        entry.  Geometric entry matching must reject that opposite direction,
        but the directed Portal identity already proves which Place is being
        reached.  This method records covered transit without allowing the
        current SLAM component to mint a duplicate Place.
        """
        try:
            destination_place_id = int(destination_place_id)
        except (TypeError, ValueError):
            return None, "invalid_destination_place"
        region = self.by_id(destination_place_id)
        if region is None:
            return None, "destination_place_missing"
        covered = region.get("state") != PLACE_OPEN
        self._record_portal_arrival(
            region,
            x,
            y,
            now,
            "portal_ledger_identity",
            covered=covered,
            entry_portal=entry_portal,
            physical_xy=physical_xy,
            physical_entry_portal=physical_entry_portal,
            source_place_id=source_place_id,
        )
        if not covered and region.get("entered_at") is None:
            region["entered_at"] = float(now)
        return region, "covered_arrival" if covered else "portal_revisit"
