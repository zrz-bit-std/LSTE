"""State transitions for persistent frontier-place memory."""

from global_frontier_place_states import (
    PLACE_DORMANT,
    PLACE_OPEN,
    PLACE_READY_TO_EXIT,
    PLACE_SUSPENDED,
)
from global_frontier_topology import copy_component_evidence


class FrontierRegionStateMixin:
    """Create, close, fail, and rebind place records without observation logic."""

    def activate(
        self, x, y, information, now, component=None, physical_xy=None,
        force_new=False,
    ):
        """Commit an already selected endpoint as an active observation region."""
        if force_new:
            # A certified physical Portal can create a destination Place even
            # when the transient structural map still reports one connected
            # core for both sides of the doorway.  The Portal identity is the
            # durable separator in that case; ordinary frontier activation
            # keeps the conservative nearest-region association.
            region, association = None, "portal_ledger_identity"
        else:
            region, association = self._nearest(
                x, y, component, physical_xy=physical_xy,
            )
        information = float(information)
        if region is None:
            region = {
                "id": self._next_id,
                "x": float(x),
                "y": float(y),
                "state": PLACE_OPEN,
                # ``visits`` is retained as a compatibility field for older
                # summaries.  It is a Place-registration count, not a route
                # dispatch counter; the latter is stored separately below.
                "visits": 1,
                # Kept for compatibility; separate counters below distinguish
                # route dispatches from physical entry and observation.
                "route_dispatches": 0,
                "physical_entry_count": 0,
                "observation_sessions": 0,
                "failures": 0,
                "completions": 0,
                "endpoint_observations": 0,
                "information": information,
                "max_information": information,
                "dormant_information": information,
                "last_gain": float(now),
                "entered_at": None,
                # A doorway crossing proves physical presence but it does not
                # start a frontier-observation dwell. Keep those facts
                # separate so transit cannot consume an observation budget.
                "observation_started_at": None,
                "portal_arrivals": 0,
                "covered_arrivals": 0,
                "last_arrived_at": None,
                # Arrival and coverage are distinct evidence. A doorway
                # terminal proves that the base crossed a place boundary, but
                # says nothing about what its lidar observed there.
                "arrival_points": [],
                # The stable identity of a place is recorded only when the
                # base physically arrives or observes. Selected map goals are
                # intentionally not stored here.
                "physical_arrival_points": [],
                # Each verified doorway crossing is a directional edge in the
                # online place graph. Closed places cannot be normally
                # re-entered through a recorded entrance.
                "entry_portals": [],
                # Map-frame anchors reached by the base reconstruct a local
                # footprint after later SLAM snapshots relabel components.
                "viewpoints": [],
                "physical_viewpoints": [],
                "last_selected": float(now),
                "last_reason": "selected",
                "component": copy_component_evidence(component),
                "last_association": "new",
            }
            self._next_id += 1
            self.regions.append(region)
            tier = "new"
        else:
            tier = "revisit"
            if region["state"] == PLACE_DORMANT:
                return None, "dormant"
            if region["state"] == PLACE_READY_TO_EXIT:
                # A reached endpoint is a state transition, not a weaker
                # score. Ordinary endpoint selection cannot reopen it.
                return None, "ready_to_exit"
            if region["state"] == PLACE_SUSPENDED:
                # Branch-first exploration pauses, rather than discards, the
                # source Place's local work. Selecting one of its WorkItems is
                # the explicit graph decision that resumes the place.
                self.resume(region, now)
            if information >= region["max_information"] + self.information_delta:
                region["max_information"] = information
                region["last_gain"] = float(now)
                region["last_reason"] = "new_information_boundary"
            region["information"] = max(region["information"], information)
            region["x"] = float(x)
            region["y"] = float(y)
            if component is not None:
                region["component"] = copy_component_evidence(component)
            region["last_association"] = association
        region["route_dispatches"] = int(
            region.get("route_dispatches", 0)
        ) + 1
        region["last_selected"] = float(now)
        return region, tier

    def close(
        self, x, y, now, reason, component=None, region_id=None,
        retain_for_target=False,
    ):
        """Close a place only at an explicit place-level lifecycle edge."""
        region, _association = self._resolve(region_id, x, y, component)
        if region is not None:
            region["completions"] += 1
            region["failures"] = 0
            region["last_selected"] = float(now)
            if region["state"] == PLACE_DORMANT:
                return region
            if region["entered_at"] is None:
                # The exact route-bound region may have been reassociated by
                # SLAM, but the base never reached it. Treat this as an
                # unentered terminal rather than blacklisting a room.
                region["last_reason"] = "route_terminal_without_region_entry"
                return region
            if retain_for_target:
                region["last_reason"] = "target_claim_requires_more_observation"
                return region
            self.dormant(region, now, reason)
        return region

    def complete(
        self, x, y, now, component=None, region_id=None,
        retain_for_target=False,
    ):
        """Compatibility wrapper for callers that intentionally close a place."""
        return self.close(
            x,
            y,
            now,
            "place_observation_complete",
            component=component,
            region_id=region_id,
            retain_for_target=retain_for_target,
        )

    def fail(self, x, y, now, reason, component=None, region_id=None):
        """Record route failure without changing the physical Place state.

        A route failure is evidence about one ViewpointAttempt, not evidence
        that a physical Place has been observed.  Retiring a Place after an
        arbitrary failure count erases its unresolved WorkItems and turns an
        execution problem into false ``frontier_exhausted``.  Place closure is
        exclusively owned by explicit observation/coverage transitions.
        """
        region, _association = self._resolve(region_id, x, y, component)
        if region is None:
            return None
        region["failures"] += 1
        region["last_selected"] = float(now)
        region["last_reason"] = "viewpoint_attempt_failed:%s" % str(reason)
        return region

    def retain_for_local_egress(self, region_id, now, reason):
        """Keep an open place alive while its route retreats to known space."""
        region = self.by_id(region_id)
        if region is None or region.get("state") != PLACE_OPEN:
            return None
        region["last_selected"] = float(now)
        region["last_reason"] = "local_egress_pending:%s" % str(reason)
        return region

    def rebind_after_local_egress(self, region_id, x, y, now, component):
        """Attach a recovered map component to its pre-egress place node."""
        region = self.by_id(region_id)
        if region is None or region.get("state") != PLACE_OPEN or component is None:
            return None
        region["x"] = float(x)
        region["y"] = float(y)
        region["component"] = copy_component_evidence(component)
        region["last_association"] = "local_egress_rebind"
        region["last_selected"] = float(now)
        region["last_reason"] = "local_egress_rebound"
        return region

    def dormant(self, region, now, reason):
        """Transition an open region into immutable completed-place memory."""
        if region is None or region["state"] == PLACE_DORMANT:
            return False
        region["state"] = PLACE_DORMANT
        region["dormant_information"] = max(
            float(region["information"]), float(region["max_information"])
        )
        region["entered_at"] = None
        region["observation_started_at"] = None
        region["last_selected"] = float(now)
        region["last_reason"] = str(reason)
        return True

    def suspend(self, region, now, reason="novel_portal_branch_selected"):
        """Pause a covered Place while retaining unresolved local WorkItems."""
        if region is None or region.get("state") != PLACE_OPEN:
            return False
        region["state"] = PLACE_SUSPENDED
        region["last_selected"] = float(now)
        region["last_reason"] = str(reason)
        return True

    def resume(self, region, now, reason="suspended_place_reopened"):
        """Resume one suspended Place after an explicit local-work action."""
        if region is None or region.get("state") != PLACE_SUSPENDED:
            return False
        region["state"] = PLACE_OPEN
        region["last_selected"] = float(now)
        region["last_reason"] = str(reason)
        return True
