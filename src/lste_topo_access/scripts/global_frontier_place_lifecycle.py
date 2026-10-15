#!/usr/bin/env python3
"""Two-phase lifecycle state for place-to-place frontier transitions.

Choosing a route through a doorway proves that the current place has no local
frontier, but it does not prove that the robot crossed the doorway.  This
small transaction records that intent at selection time and closes the source
place only after the matching route reports success.
"""

from global_frontier_topology import copy_component_evidence
from global_frontier_place_memory import (
    PLACE_DORMANT,
    PLACE_OPEN,
    PLACE_READY_TO_EXIT,
    PLACE_SUSPENDED,
)


class PlaceDepartureTransaction:
    """Keep pending departure state out of the ROS orchestration node."""

    def __init__(self):
        self.last_committed_basis = None
        self.clear()

    @property
    def active(self):
        return self.region_id is not None

    def clear(self):
        """Discard an uncommitted transition without changing place memory."""
        self.region_id = None
        self.component = None
        self.anchor_map = None
        self.place_hops = None
        self.basis = None
        # A route can be selected while the robot is still standing at the
        # source endpoint.  The explorer must observe real translation before
        # treating a changing SLAM footprint as a room departure.
        self.minimum_travel_distance = 0.0
        self.exit_wait_reported = False
        # A portal uses a directional odom proof in addition to the ordinary
        # map-footprint departure check. This prevents a TEB goal-circle
        # success at the source side of a doorway from closing the room.
        self.requires_physical_gate_crossing = False
        self.physical_gate_crossing_confirmed = False
        self.gate_crossing_wait_reported = False
        # A completed place may be crossed as a graph corridor on the way to
        # an unexplored branch.  That is transit, not a second observation
        # lifecycle, so its commit must never call ``memory.close`` again.
        self.transit_only = False
        # Branch-first execution keeps unresolved local work resumable after
        # the robot follows a novel Portal into another Place.
        self.suspend_local_work = False

    def require_physical_gate_crossing(self):
        """Bind this departure to a portal's physical gate-crossing proof."""
        if not self.active:
            return False
        self.requires_physical_gate_crossing = True
        return True

    def suspend_local_work_on_commit(self):
        """Retain unresolved WorkItems when this departure pauses a Place."""
        if not self.active or self.transit_only:
            return False
        self.suspend_local_work = True
        return True

    def suspend_if_local_work_pending(
        self, work_item_ledger, portal_probe_ledger=None,
    ):
        """Protect unresolved local WorkItems on every non-transit departure.

        A route may leave an open Place through a durable graph edge even when
        the selector reached that edge through recovery or covered transit.
        Closing such a Place as ``dormant`` would erase its unfinished local
        branch. The ledger owns the fact; this transaction only records the
        resulting lifecycle state.
        """
        if not self.active or self.transit_only or self.suspend_local_work:
            return False
        if work_item_ledger is None:
            return False
        count = getattr(work_item_ledger, "unresolved_observation_count", None)
        if callable(count):
            try:
                pending = count(
                    self.region_id,
                    portal_probe_ledger=portal_probe_ledger,
                )
            except TypeError:
                pending = count(self.region_id)
        else:
            count = getattr(work_item_ledger, "unresolved_count", None)
            pending = 0 if not callable(count) else count(self.region_id)
        if int(pending or 0) <= 0:
            return False
        self.suspend_local_work = True
        return True

    def confirm_physical_gate_crossing(self):
        """Allow source closure after the active portal crossed its gate."""
        if not self.active or not self.requires_physical_gate_crossing:
            return False
        self.physical_gate_crossing_confirmed = True
        return True

    def _prepare_region(
        self,
        region,
        anchor_map,
        place_hops,
        target_region_claim_active,
        component,
        basis,
        minimum_travel_distance,
        allow_dormant=False,
    ):
        """Install a departure only when its exact source remains enterable."""
        if (
            self.active
            or
            region is None
            or region.get("state") not in (
                (PLACE_OPEN, PLACE_READY_TO_EXIT, PLACE_DORMANT, PLACE_SUSPENDED)
                if allow_dormant
                else (PLACE_OPEN, PLACE_READY_TO_EXIT, PLACE_SUSPENDED)
            )
            or target_region_claim_active
        ):
            return None
        self.region_id = int(region["id"])
        self.component = copy_component_evidence(component)
        self.anchor_map = (float(anchor_map[0]), float(anchor_map[1]))
        self.place_hops = int(place_hops)
        self.basis = str(basis)
        self.minimum_travel_distance = max(0.0, float(minimum_travel_distance))
        self.exit_wait_reported = False
        self.transit_only = bool(allow_dormant)
        return region

    def prepare(
        self,
        memory,
        component,
        robot_map,
        place_hops,
        target_region_claim_active,
        minimum_travel_distance=0.0,
    ):
        """Record a pending departure and return its open source region.

        ``None`` means no state was changed.  The target-place claim takes
        precedence because visual target confirmation can require continued
        local observation even after ordinary frontier coverage is exhausted.
        """
        if place_hops is None or int(place_hops) < 1 or component is None:
            return None
        region, _association = memory._nearest(
            float(robot_map[0]), float(robot_map[1]), component
        )
        return self._prepare_region(
            region,
            robot_map,
            place_hops,
            target_region_claim_active,
            component,
            "structural_transition",
            minimum_travel_distance,
        )

    def prepare_region(
        self,
        memory,
        region_id,
        anchor_map,
        place_hops,
        target_region_claim_active,
        component=None,
        basis="observation_boundary",
        minimum_travel_distance=0.0,
        allow_dormant=False,
    ):
        """Prepare a departure from an exact, previously observed region.

        Structural place labels can be temporarily unavailable or can merge
        through an incompletely scanned doorway.  A reached viewpoint is still
        an authoritative observation fact, so the selector may bind a future
        route to that exact region without guessing from the current label.
        """
        if place_hops is None or int(place_hops) < 1:
            return None
        return self._prepare_region(
            memory.by_id(region_id),
            anchor_map,
            place_hops,
            target_region_claim_active,
            component,
            basis,
            minimum_travel_distance,
            allow_dormant=allow_dormant,
        )

    def commit(self, memory, now):
        """Close the prepared source place and return its execution context.

        State is consumed before calling ``memory.close``.  A duplicate route
        terminal or an exception in an outer ROS callback therefore cannot
        close the same place twice.
        """
        region_id = self.region_id
        component = self.component
        anchor = self.anchor_map
        place_hops = self.place_hops
        basis = self.basis
        transit_only = self.transit_only
        suspend_local_work = self.suspend_local_work
        self.clear()
        self.last_committed_basis = basis
        if region_id is None or anchor is None:
            return None, None, None
        if transit_only:
            # The room was already closed before this action.  Retain the
            # crossing as route provenance, but do not mutate its observation
            # state or completion counters simply because it was traversed.
            return memory.by_id(region_id), anchor, place_hops
        if suspend_local_work:
            region = memory.by_id(region_id)
            if region is None:
                return None, anchor, place_hops
            memory.suspend(region, now, "novel_portal_branch_committed")
            return region, anchor, place_hops
        closed = memory.close(
            anchor[0],
            anchor[1],
            now,
            "no_local_executable_frontier",
            component=component,
            region_id=region_id,
        )
        return closed, anchor, place_hops


class LocalEgressPlaceLease:
    """Retain one place identity while a route backs out of a local trap.

    A local egress follows a previously traversed path; it is not a doorway
    crossing and therefore cannot create a new place.  The lease has four
    explicit states: ``idle -> queued -> active -> rebind_pending -> idle``.
    """

    def __init__(self):
        self.clear()

    @property
    def rebind_pending(self):
        return self.phase == "rebind_pending" and self.source_region_id is not None

    def clear(self):
        self.source_region_id = None
        self.phase = "idle"

    def queue(self, region_id):
        self.clear()
        if region_id is None:
            return False
        self.source_region_id = int(region_id)
        self.phase = "queued"
        return True

    def activate(self):
        if self.phase != "queued":
            return False
        self.phase = "active"
        return True

    def complete(self):
        if self.phase != "active":
            return False
        self.phase = "rebind_pending"
        return True

    def fail(self):
        region_id = self.source_region_id
        self.clear()
        return region_id
