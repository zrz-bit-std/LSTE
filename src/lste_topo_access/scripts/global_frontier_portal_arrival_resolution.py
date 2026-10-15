#!/usr/bin/env python3
"""Structural-evidence lookup and pending state for portal arrivals."""

from global_frontier_models import PendingPortalArrival


class GlobalFrontierPortalArrivalResolutionMixin:
    """Resolve portal destinations only from fresh, wall-bounded map evidence."""

    def portal_arrival_can_commit_without_component(self):
        """Allow durable Portal identity to bridge a missing SLAM label."""
        transaction = getattr(self, "portal_transaction", None)
        commit_authorized = getattr(transaction, "commit_authorized", None)
        return bool(
            transaction is not None
            and str(getattr(transaction, "state", "")) == "crossing_verified"
            and callable(commit_authorized)
            and commit_authorized(
                route_id=int(self.active_route_id),
                portal_id=getattr(self, "last_portal_hypothesis_id", None),
                source_place_id=getattr(
                    getattr(self, "place_departure", None), "region_id", None
                ),
            )
        )

    def retry_pending_portal_arrivals(
        self, message, components, known_free, now,
    ):
        """Resolve delayed arrivals from one fresh structural snapshot."""
        entered, remaining = [], []
        for arrival in self.pending_portal_arrivals:
            component = self.portal_arrival_component_from_snapshot(
                message, components, known_free, arrival,
            )
            if component is None or self.portal_arrival_still_in_source_place(
                arrival, component
            ):
                remaining.append(arrival)
                continue
            region = self.commit_portal_arrival(
                arrival, component, now, evidence_source="fresh_map_snapshot"
            )
            if region is not None:
                entered.append(region)
        self.pending_portal_arrivals = remaining
        return entered

    def queue_pending_portal_arrival(self, completed, now):
        """Retain one verified terminal until a future map can identify its room."""
        arrival = PendingPortalArrival(
            route_id=int(self.active_route_id),
            goal_xy=(float(completed[2]), float(completed[3])),
            terminal_time=float(now),
            source_region_id=self.place_departure.region_id,
            portal_gate_xy=getattr(self, "active_portal_gate_xy", None),
            physical_goal_xy=(
                None
                if getattr(self, "pose_odom", None) is None
                else (float(self.pose_odom.x), float(self.pose_odom.y))
            ),
            physical_portal_gate_xy=getattr(
                self, "active_portal_gate_odom_xy", None,
            ),
        )
        if any(
            int(existing.route_id) == arrival.route_id
            for existing in self.pending_portal_arrivals
        ):
            return arrival
        self.pending_portal_arrivals.append(arrival)
        self.publish_status(
            "portal_place_arrival_pending",
            route_id=arrival.route_id,
            goal=[round(arrival.goal_xy[0], 3), round(arrival.goal_xy[1], 3)],
            reason="waiting_for_fresh_destination_structural_component",
        )
        return arrival

    def portal_arrival_component_from_snapshot(
        self, message, components, known_free, arrival,
    ):
        """Look up a pending terminal in exactly the supplied map snapshot."""
        return self.component_at_map_position(
            message,
            components,
            known_free,
            arrival.goal_xy[0],
            arrival.goal_xy[1],
        )

    def portal_arrival_still_in_source_place(self, arrival, component):
        """Return whether fresh evidence still resolves to the departure place.

        The terminal proves physical crossing, but SLAM can need one more scan
        before the destination core separates from the source room.  In that
        interval a source match is explicitly inconclusive, never evidence
        that the base entered an already closed source place again.
        """
        source_region_id = arrival.source_region_id
        if source_region_id is None or component is None:
            return False
        # A directed Portal transaction is stronger than a snapshot-local
        # structural label.  Some valid environments (including a corridor
        # branch with no room wall) remain one connected high-clearance core
        # forever; waiting for a different label would strand a physically
        # verified arrival indefinitely.  The commit path can safely create a
        # new Place from the Portal identity in that case.
        transaction = getattr(self, "portal_transaction", None)
        if transaction is not None:
            try:
                if (
                    str(transaction.state) == "crossing_verified"
                    and transaction.commit_authorized(
                        route_id=arrival.route_id,
                        portal_id=getattr(
                            self, "last_portal_hypothesis_id", None,
                        ),
                        source_place_id=source_region_id,
                    )
                ):
                    return False
            except (AttributeError, TypeError, ValueError):
                pass
        _tier, matched = self.region_memory.candidate_tier(
            arrival.goal_xy[0],
            arrival.goal_xy[1],
            0.0,
            component=component,
            physical_xy=arrival.physical_goal_xy,
        )
        return matched is not None and int(matched["id"]) == int(source_region_id)
