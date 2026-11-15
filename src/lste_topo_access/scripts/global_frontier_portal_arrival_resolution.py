#!/usr/bin/env python3
"""Structural-evidence lookup and pending state for portal arrivals."""

import math
import time

from clock_provider import now_for
from global_frontier_models import PendingPortalArrival


class GlobalFrontierPortalArrivalResolutionMixin:
    """Resolve portal destinations only from fresh, wall-bounded map evidence."""

    def portal_arrival_can_commit_identity(self, arrival):
        """Return whether a crossing-verified transaction owns this arrival.

        A rerouted or late terminal can outlive the mutable
        ``last_portal_hypothesis_id`` and the active route kind.  The durable
        transaction route/portal/source identity is the authoritative owner.
        """
        transaction = getattr(self, "portal_transaction", None)
        if transaction is None or not transaction.active:
            return False
        if str(transaction.state) != "crossing_verified":
            return False
        snapshot = transaction.snapshot()
        try:
            return bool(
                int(snapshot.route_id or 0) == int(arrival.route_id)
                and int(snapshot.portal_id or 0) > 0
                and int(snapshot.source_place_id or 0)
                == int(arrival.source_region_id)
            )
        except (AttributeError, TypeError, ValueError):
            return False

    def crossing_transaction_portal_id(self, arrival):
        """Return the transaction Portal id for a matching verified arrival."""
        if not self.portal_arrival_can_commit_identity(arrival):
            return 0
        try:
            return int(self.portal_transaction.snapshot().portal_id or 0)
        except (AttributeError, TypeError, ValueError):
            return 0

    def resolve_verified_portal_crossing(self, now=None):
        """Commit or bound a crossing-verified lease that lost its trigger.

        Live crossing and terminal callbacks can race successor selection:
        the edge is physically verified, but neither callback records the
        destination arrival.  Rebuild the arrival from the transaction itself
        and commit by identity.  If that is impossible, queue one pending
        arrival and close the lease after a bounded no-route window so the
        selector cannot be blocked forever.
        """
        transaction = getattr(self, "portal_transaction", None)
        if (transaction is None
                or not transaction.active
                or str(transaction.state) != "crossing_verified"):
            self._crossing_verified_uncommitted_since = 0.0
            self._crossing_verified_uncommitted_pose = None
            return False
        snapshot = transaction.snapshot()
        try:
            route_id = int(snapshot.route_id or 0)
            source_place_id = int(snapshot.source_place_id or 0)
            goal_xy = (
                None if snapshot.destination_xy is None
                else (
                    float(snapshot.destination_xy[0]),
                    float(snapshot.destination_xy[1]),
                )
            )
        except (AttributeError, TypeError, ValueError):
            return False
        if route_id <= 0 or goal_xy is None:
            return False
        now = now_for(self) if now is None else float(now)
        pose = getattr(self, "pose_odom", None)
        arrival = PendingPortalArrival(
            route_id=route_id,
            goal_xy=goal_xy,
            terminal_time=now,
            source_region_id=source_place_id,
            portal_gate_xy=snapshot.gate_xy,
            physical_goal_xy=(
                None if pose is None
                else (float(pose.x), float(pose.y))
            ),
            physical_portal_gate_xy=snapshot.gate_xy,
        )
        if self.portal_arrival_can_commit_identity(arrival):
            region = self.commit_portal_arrival(
                arrival, None, now,
                evidence_source="crossing_transaction_recovery",
            )
            if region is not None:
                self._crossing_verified_uncommitted_since = 0.0
                self._crossing_verified_uncommitted_pose = None
                return True
        component = getattr(self, "active_frontier_component", None)
        if (component is not None
                and not self.portal_arrival_still_in_source_place(
                    arrival, component)):
            region = self.commit_portal_arrival(
                arrival, component, now,
                evidence_source="crossing_transaction_component_recovery",
            )
            if region is not None:
                self._crossing_verified_uncommitted_since = 0.0
                self._crossing_verified_uncommitted_pose = None
                return True
        self.queue_portal_arrival(arrival)
        pose_xy = (
            None if pose is None else (float(pose.x), float(pose.y))
        )
        wall_now = time.monotonic()
        anchor = getattr(self, "_crossing_verified_uncommitted_pose", None)
        since = float(
            getattr(self, "_crossing_verified_uncommitted_since", 0.0) or 0.0
        )
        if since <= 0.0 or anchor is None:
            self._crossing_verified_uncommitted_since = wall_now
            self._crossing_verified_uncommitted_pose = pose_xy
            return False
        if (
            pose_xy is not None
            and math.hypot(
                pose_xy[0] - anchor[0], pose_xy[1] - anchor[1],
            ) >= 0.35
        ):
            self._crossing_verified_uncommitted_since = wall_now
            self._crossing_verified_uncommitted_pose = pose_xy
            return False
        timeout = float(
            getattr(self, "crossing_uncommitted_timeout", 30.0) or 0.0
        )
        if timeout > 0.0 and wall_now - since < timeout:
            return False
        transition = transaction.abort("crossing_uncommitted")
        if transition is None:
            return False
        self.publish_status(
            "portal_transaction_aborted",
            transaction_id=int(transition.transaction_id),
            route_id=int(transition.route_id),
            portal_id=transition.portal_id,
            state=transition.state,
            reason=transition.last_reason,
            preserved_portal_obligation=True,
            crossing_uncommitted=True,
        )
        self.discard_pending_portal_arrival(route_id)
        finished = transaction.finish("crossing_uncommitted")
        if finished is not None:
            self.publish_status(
                "portal_transaction_finished",
                transaction_id=int(finished.transaction_id),
                portal_id=finished.portal_id,
                source_place_id=finished.source_place_id,
                state=finished.state,
                reason=finished.last_reason,
            )
        self._crossing_verified_uncommitted_since = 0.0
        self._crossing_verified_uncommitted_pose = None
        return True

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

    def discard_pending_portal_arrival(self, route_id):
        """Drop one queued arrival after commit or explicit lease closure."""
        try:
            route_id = int(route_id)
        except (TypeError, ValueError):
            return
        self.pending_portal_arrivals = [
            arrival for arrival in self.pending_portal_arrivals
            if int(arrival.route_id) != route_id
        ]

    def queue_portal_arrival(self, arrival):
        """Retain one verified arrival until a map or identity commit lands."""
        if any(
            int(existing.route_id) == int(arrival.route_id)
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
        return self.queue_portal_arrival(arrival)

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
        if self.portal_arrival_can_commit_identity(arrival):
            return False
        _tier, matched = self.region_memory.candidate_tier(
            arrival.goal_xy[0],
            arrival.goal_xy[1],
            0.0,
            component=component,
            physical_xy=arrival.physical_goal_xy,
        )
        return matched is not None and int(matched["id"]) == int(source_region_id)
