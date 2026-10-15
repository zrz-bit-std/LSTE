"""Place-memory facts committed after a verified route terminal."""


class GlobalFrontierTerminalRecordingMixin:
    """Record the terminal outcome without choosing its successor route."""

    def record_execution_terminal(self, completed):
        """Record observation and place-crossing facts for a verified terminal."""
        self.active_terminal_received = True
        portal_gate_xy = getattr(self, "active_portal_gate_xy", None)
        place_hops = getattr(self, "active_place_hops", None)
        crosses_place_boundary = False
        boundary_check = getattr(self, "route_crosses_place_boundary", None)
        if boundary_check is not None:
            crosses_place_boundary = bool(boundary_check(place_hops))
        if completed is not None and self.active_route_kind == "portal_transition":
            self.record_portal_place_arrival(completed)
        elif (
            completed is not None
            and crosses_place_boundary
            and portal_gate_xy is not None
            and self.active_route_kind != "local_egress"
            and self.active_frontier_region_id is not None
        ):
            entry = self.region_memory.record_entry_portal(
                self.active_frontier_region_id,
                portal_gate_xy,
                (float(completed[2]), float(completed[3])),
                physical_gate_xy=getattr(
                    self, "active_portal_gate_odom_xy", None,
                ),
                physical_inside_xy=(
                    None
                    if getattr(self, "pose_odom", None) is None
                    else (
                        float(self.pose_odom.x),
                        float(self.pose_odom.y),
                    )
                ),
                source_place_id=getattr(
                    getattr(self, "place_departure", None),
                    "region_id",
                    None,
                ),
            )
            if entry is not None:
                self.publish_status(
                    "frontier_region_portal_entry_recorded",
                    route_id=int(self.active_route_id),
                    region_id=int(self.active_frontier_region_id),
                    route_kind=str(self.active_route_kind),
                    place_graph_hops=int(place_hops),
                    gate=[
                        round(float(portal_gate_xy[0]), 3),
                        round(float(portal_gate_xy[1]), 3),
                    ],
                    inside=[
                        round(float(completed[2]), 3),
                        round(float(completed[3]), 3),
                    ],
                )
        if completed is not None and self.active_route_kind == "local_egress":
            if self.active_local_egress_resumes_portal:
                retry = self.pending_portal_retry
                self.publish_status(
                    "portal_transition_egress_completed",
                    route_id=int(self.active_route_id),
                    recovery_goal=[
                        round(float(completed[2]), 3),
                        round(float(completed[3]), 3),
                    ],
                    retry_goal=(
                        None if retry is None else [
                            round(float(retry.goal_xy[0]), 3),
                            round(float(retry.goal_xy[1]), 3),
                        ]
                    ),
                )
            else:
                self.local_egress_place_lease.complete()
                self.publish_status(
                    "local_egress_completed",
                    route_id=int(self.active_route_id),
                    recovery_goal=[
                        round(float(completed[2]), 3),
                        round(float(completed[3]), 3),
                    ],
                    source_region_id=self.local_egress_place_lease.source_region_id,
                )
        if completed is not None and self.active_route_kind not in (
            "portal_transition", "local_egress",
        ):
            self.close_stagnant_place_after_terminal(completed)
            physical_xy = (
                None
                if getattr(self, "pose_odom", None) is None
                else (float(self.pose_odom.x), float(self.pose_odom.y))
            )
            observed_region = self.mark_frontier_observed(
                completed[2],
                completed[3],
                component=self.active_frontier_component,
                region_id=self.active_frontier_region_id,
                physical_xy=physical_xy,
            )
            self.remember_completed_observation_source(observed_region)
        if self.place_departure.active:
            self.publish_status(
                "frontier_place_departure_terminal_pending",
                region_id=int(self.place_departure.region_id),
                route_id=int(self.active_route_id),
                departure_basis=self.place_departure.basis,
                source=[
                    round(float(self.place_departure.anchor_map[0]), 3),
                    round(float(self.place_departure.anchor_map[1]), 3),
                ],
            )
