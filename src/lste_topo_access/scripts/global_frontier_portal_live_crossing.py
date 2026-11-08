"""Continuous physical evidence for a directed portal transaction."""


class GlobalFrontierPortalLiveCrossingMixin:
    """Commit a crossed graph edge before the local action terminal arrives."""

    def observe_active_portal_crossing(self, now=None):
        """Commit a physical portal crossing without waiting for TEB success.

        A portal is a graph-edge transaction, while a TEB terminal only says
        that its local action has stopped. Once odometry proves a crossing,
        retaining the old source place until a final few centimetres of TEB
        motion creates the repeated-room failure this graph prevents.
        """
        if (
            self.active_route_kind != "portal_transition"
            or self.active_frontier is None
            or getattr(self, "active_portal_crossing_observed", False)
            or getattr(self, "active_portal_crossing_rejected", False)
        ):
            return None
        crossing = self.active_portal_crossing_evidence()
        if not crossing.verified:
            signature = (
                int(getattr(self, "active_route_id", 0) or 0),
                str(crossing.reason),
            )
            if signature != getattr(
                self, "_portal_crossing_unverified_signature", None
            ):
                self._portal_crossing_unverified_signature = signature
                pose = getattr(self, "pose_odom", None)
                self.publish_status(
                    "portal_crossing_unverified",
                    route_id=int(getattr(self, "active_route_id", 0) or 0),
                    reason=str(crossing.reason),
                    source_signed_distance=crossing.source_signed_distance,
                    terminal_signed_distance=crossing.terminal_signed_distance,
                    required_destination_depth=(
                        crossing.required_destination_depth
                    ),
                    gate=getattr(self, "active_portal_gate_odom_xy", None),
                    destination=getattr(
                        self, "active_portal_destination_odom_xy", None
                    ),
                    start=getattr(self, "active_start_odom_xy", None),
                    pose=(
                        None if pose is None
                        else (round(float(pose.x), 3), round(float(pose.y), 3))
                    ),
                )
            return None
        destination = self.record_portal_place_arrival(
            self.active_frontier,
            now=now,
            evidence_source="continuous_odom",
            retry_on_unconfirmed=False,
        )
        if (
            destination is not None
            and bool(getattr(self, "persistent_execution", False))
            and getattr(self, "active_frontier", None) is not None
        ):
            # The physical edge is complete as soon as the destination Place
            # is committed.  Keeping the old point lease alive until its last
            # few centimetres are reached can feed a reverse-only TEB command
            # into the forward-only actuator and waste the route watchdog.
            # Persistent execution may release the semantic route here; the
            # long-lived MoveBase action remains the controller lease and can
            # adopt the next graph route without a stop/restart cycle.
            released_route_id = int(getattr(self, "active_route_id", 0) or 0)
            release = getattr(self, "release_active_frontier", None)
            if callable(release):
                release(
                    discard_prefetch=True,
                    preserve_place_departure=False,
                    queue_successor=True,
                )
                publish = getattr(self, "publish_status", None)
                if callable(publish):
                    publish(
                        "portal_crossing_route_released",
                        route_id=released_route_id,
                        destination_place_id=int(destination.get("id", 0) or 0),
                        reason="physical_portal_crossing_committed",
                        controller_lease="persistent_stream",
                    )
            return destination
        return destination
