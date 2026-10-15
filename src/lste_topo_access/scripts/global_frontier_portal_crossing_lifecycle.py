"""Bind portal-terminal outcomes to physical gate-crossing evidence."""

from global_frontier_portal_crossing import (
    portal_crossing_depth,
    portal_crossing_evidence,
    portal_signed_distance,
)


class GlobalFrontierPortalCrossingLifecycleMixin:
    """Confirm a place-graph edge in the stable odom frame."""

    def active_portal_crossing_evidence(self):
        """Evaluate the active portal using frozen odom geometry only."""
        terminal_xy = (
            None
            if getattr(self, "pose_odom", None) is None
            else (float(self.pose_odom.x), float(self.pose_odom.y))
        )
        # Room-observation and controller endpoint radii intentionally do not
        # enter this test.  They are much larger than a door and made a robot
        # that had visibly passed through an opening wait indefinitely for an
        # unrelated room-scale depth.  This is a pure doorway geometry fact.
        minimum_depth = portal_crossing_depth(
            getattr(self, "clearance", 0.0)
        )
        transaction = getattr(self, "portal_transaction", None)
        transaction_snapshot = (
            None if transaction is None else transaction.snapshot()
        )
        # The route-local start can be at the doorway after a recovery retry.
        # Preserve the transaction-level source proof and feed the latest
        # signed distance into it as additional physical evidence.
        current_signed_distance = portal_signed_distance(
            terminal_xy,
            getattr(self, "active_portal_gate_odom_xy", None),
            getattr(self, "active_portal_destination_odom_xy", None),
        )
        if transaction is not None and current_signed_distance is not None:
            transaction.observe_source_side(
                current_signed_distance,
                route_id=getattr(self, "active_route_id", None),
                reason="source_side_observed_during_route",
            )
            transaction_snapshot = transaction.snapshot()
        return portal_crossing_evidence(
            getattr(self, "active_start_odom_xy", None),
            getattr(self, "active_portal_gate_odom_xy", None),
            getattr(self, "active_portal_destination_odom_xy", None),
            terminal_xy,
            getattr(self, "active_portal_gate_approached_at", None) is not None,
            minimum_depth,
            getattr(self, "active_portal_crossing_preobserved", False),
            source_side_proven=(
                False
                if transaction_snapshot is None
                else bool(transaction_snapshot.source_side_proven)
            ),
        )

    def confirm_active_portal_crossing(self):
        """Persist a verified crossing on the pending source-place transaction."""
        evidence = self.active_portal_crossing_evidence()
        if evidence.verified:
            transaction = getattr(self, "portal_transaction", None)
            if transaction is not None:
                standoff = transaction.destination_standoff(
                    self.active_route_id,
                    reason="physical_destination_standoff_reached",
                )
                if standoff is not None:
                    self.publish_status(
                        "portal_transaction_phase",
                        transaction_id=int(standoff.transaction_id),
                        route_id=int(standoff.route_id),
                        state=standoff.state,
                        transition=standoff.last_transition,
                        reason=standoff.last_reason,
                    )
                transition = transaction.crossing_verified(
                    self.active_route_id,
                    reason=evidence.reason,
                )
                if transition is not None:
                    self.publish_status(
                        "portal_transaction_phase",
                        transaction_id=int(transition.transaction_id),
                        route_id=int(transition.route_id),
                        state=transition.state,
                        transition=transition.last_transition,
                        reason=transition.last_reason,
                    )
            confirm = getattr(
                self.place_departure, "confirm_physical_gate_crossing", None,
            )
            if confirm is not None:
                confirm()
        return evidence
