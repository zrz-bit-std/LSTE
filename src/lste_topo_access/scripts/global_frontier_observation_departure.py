"""Two-phase physical place-departure helpers for frontier exploration.

Preparing a departure and committing it after the robot crossed the boundary
are kept together so the state transition is auditable and easy to change.
"""

import math

import numpy as np

from clock_provider import now_for


class GlobalFrontierObservationDepartureMixin:

    def prepare_place_departure(
        self, message, components, known_free, robot_map, place_hops,
    ):
        """Attach map evidence and publish a prepared departure transaction."""
        component = None
        if components is not None:
            component = self.component_at_map_position(
                message,
                components,
                known_free,
                robot_map[0],
                robot_map[1],
            )
        current_place_id = getattr(self, "current_physical_place_id", None)
        if current_place_id is not None:
            # The durable physical Place owns every outgoing edge.  A
            # component label is only snapshot-local geometry and may have
            # merged with an older room after a SLAM correction; using it to
            # infer the source can bind a new Portal transaction to stale
            # ownership.  Keep the component for evidence, but prepare the
            # departure against the exact current Place identity.
            region = self.place_departure.prepare_region(
                self.region_memory,
                current_place_id,
                robot_map,
                place_hops,
                self.target_region_claim_active,
                component=component,
                basis="structural_transition",
                minimum_travel_distance=getattr(
                    self, "place_departure_min_travel_distance", 0.0
                ),
            )
        else:
            # Compatibility path for the process bootstrap and narrow legacy
            # fixtures that do not yet expose a durable Place owner.
            region = self.place_departure.prepare(
                self.region_memory,
                component,
                robot_map,
                place_hops,
                self.target_region_claim_active,
                minimum_travel_distance=getattr(
                    self, "place_departure_min_travel_distance", 0.0
                ),
            )
        if region is None:
            return None
        self.publish_prepared_place_departure(
            region,
            place_hops,
            robot_map,
            basis=self.place_departure.basis,
        )
        return region

    def publish_prepared_place_departure(self, region, place_hops, source, basis):
        """Publish the shared audit event for either departure proof."""
        self.publish_status(
            "frontier_place_departure_prepared",
            region_id=int(region["id"]),
            region_visits=int(region["visits"]),
            endpoint_observations=int(region["endpoint_observations"]),
            selected_place_hops=int(place_hops),
            departure_basis=str(basis),
            source=[round(float(source[0]), 3), round(float(source[1]), 3)],
        )

    def prepare_observation_place_departure(self, message, known_free, selection):
        """Prepare an outward transition when a reached submap no longer contains it.

        This is the topology-independent fallback.  It is intentionally
        evaluated only after one endpoint completed: a candidate outside a
        view from an unvisited route is not proof that the robot left a place.
        """
        source = self.observation_departure_source
        self.last_observation_departure_footprint_cells = 0
        self.last_observation_departure_anchor_count = 0
        if source is None or self.target_region_claim_active:
            return None
        region = self.region_memory.by_id(source.region_id)
        if region is None or region.get("state") not in (
            "open", "ready_to_exit", "suspended",
        ):
            self.observation_departure_source = None
            return None
        anchor_cells = self.observation_anchor_cells(
            message,
            known_free,
            self.region_memory.viewpoints_for(source.region_id),
        )
        footprint = self.observation_footprint_from_anchors(
            message, known_free, anchor_cells,
        )
        self.last_observation_departure_footprint_cells = int(
            np.count_nonzero(footprint)
        )
        self.last_observation_departure_anchor_count = len(anchor_cells)
        target_cell = self.xy_to_grid_cell(message, selection.x, selection.y)
        if (
            target_cell is None
            or not anchor_cells
            or footprint[target_cell]
        ):
            return None
        departed = self.place_departure.prepare_region(
            self.region_memory,
            source.region_id,
            source.anchor_map,
            place_hops=1,
            target_region_claim_active=self.target_region_claim_active,
            basis="observation_boundary",
            minimum_travel_distance=getattr(
                self, "place_departure_min_travel_distance", 0.0
            ),
        )
        if departed is None:
            return None
        self.publish_prepared_place_departure(
            departed,
            place_hops=1,
            source=source.anchor_map,
            basis=self.place_departure.basis,
        )
        return departed

    def prepare_covered_place_transit(self, robot_map, place_hops):
        """Bind a portal route to a completed room without reopening it.

        A building graph is not a tree.  Once a side office has been explored,
        the robot can still need to cross it or its entry corridor to reach an
        unvisited branch.  This records that physical source for the portal
        contract while preserving the room as closed to local work.
        """
        current_id = getattr(self, "current_physical_place_id", None)
        if current_id is None or self.target_region_claim_active:
            return None
        region = self.region_memory.by_id(current_id)
        if region is None or region.get("state") != "dormant":
            return None
        departed = self.place_departure.prepare_region(
            self.region_memory,
            current_id,
            robot_map,
            place_hops=place_hops,
            target_region_claim_active=False,
            basis="covered_transit",
            minimum_travel_distance=0.0,
            allow_dormant=True,
        )
        if departed is not None:
            self.publish_prepared_place_departure(
                departed,
                place_hops=place_hops,
                source=robot_map,
                basis=self.place_departure.basis,
            )
        return departed

    def commit_active_place_departure(self):
        """Commit a prepared source-place closure after a successful crossing."""
        source_region = self.region_memory.by_id(self.place_departure.region_id)
        protect = getattr(
            self.place_departure, "suspend_if_local_work_pending", None,
        )
        if source_region is not None and protect is not None:
            protect(
                getattr(self, "place_work_items", None),
                portal_probe_ledger=getattr(self, "portal_probe_ledger", None),
            )
        closed, anchor, place_hops = self.place_departure.commit(
            self.region_memory, now_for(self)
        )
        if closed is not None and closed.get("state") == "dormant":
            if self.place_departure.last_committed_basis == "covered_transit":
                self.publish_status(
                    "frontier_place_transited",
                    region_id=int(closed["id"]),
                    selected_place_hops=place_hops,
                    departure_basis="covered_transit",
                    source=[round(float(anchor[0]), 3), round(float(anchor[1]), 3)],
                )
                return closed
            if (
                self.observation_departure_source is not None
                and self.observation_departure_source.region_id == int(closed["id"])
            ):
                self.observation_departure_source = None
            self.publish_status(
                "frontier_place_closed",
                reason=closed["last_reason"],
                region_id=int(closed["id"]),
                region_visits=int(closed["visits"]),
                region_completions=int(closed["completions"]),
                endpoint_observations=int(closed["endpoint_observations"]),
                selected_place_hops=place_hops,
                departure_basis=self.place_departure.last_committed_basis,
                source=[round(float(anchor[0]), 3), round(float(anchor[1]), 3)],
            )
        elif closed is not None and closed.get("state") == "suspended":
            work_items = getattr(self, "place_work_items", None)
            self.publish_status(
                "frontier_place_suspended",
                reason=closed.get("last_reason", "novel_portal_branch_committed"),
                region_id=int(closed["id"]),
                region_visits=int(closed["visits"]),
                endpoint_observations=int(closed["endpoint_observations"]),
                unresolved_work_items=(
                    None
                    if work_items is None
                    else int(work_items.unresolved_count(closed["id"]))
                ),
                selected_place_hops=place_hops,
                departure_basis=self.place_departure.last_committed_basis,
                source=[round(float(anchor[0]), 3), round(float(anchor[1]), 3)],
            )
        return closed

    def commit_departed_place_after_boundary_crossing(self, snapshot):
        """Close a prepared source place once the robot has physically left it.

        A target-replan can preempt the move_base action after the robot has
        crossed a door but before that action reports its terminal state.  The
        controller terminal is then unavailable, but the map still supplies a
        stronger fact than a timer: the robot is outside the source place's
        wall-bounded observation footprint.  Preserve this transaction until
        that fact is available, otherwise the source place remains ``open``
        and can be selected again later.
        """
        transaction = self.place_departure
        if not transaction.active:
            return None
        source_region = self.region_memory.by_id(transaction.region_id)
        transit_only = transaction.basis == "covered_transit"
        if source_region is None or (
            source_region.get("state") not in (
                "open", "ready_to_exit", "suspended",
            )
            and not (transit_only and source_region.get("state") == "dormant")
        ):
            transaction.clear()
            return None
        if (
            getattr(transaction, "requires_physical_gate_crossing", False)
            and not getattr(transaction, "physical_gate_crossing_confirmed", False)
        ):
            if not getattr(transaction, "gate_crossing_wait_reported", False):
                transaction.gate_crossing_wait_reported = True
                self.publish_status(
                    "frontier_place_departure_waiting",
                    region_id=int(source_region["id"]),
                    route_id=int(self.active_route_id),
                    departure_basis=transaction.basis,
                    reason="waiting_for_physical_portal_crossing",
                    source=[
                        round(float(transaction.anchor_map[0]), 3),
                        round(float(transaction.anchor_map[1]), 3),
                    ],
                )
            return None
        if transit_only:
            # The physical gate proof is sufficient for a closed room.  Its
            # observation footprint is intentionally irrelevant: transit is
            # allowed to pass through known floor space but cannot select work
            # there.
            return self.commit_active_place_departure()
        anchor_cells = self.observation_anchor_cells(
            snapshot.message,
            snapshot.map_context.known_free,
            self.region_memory.viewpoints_for(transaction.region_id),
        )
        travel_distance = math.hypot(
            float(snapshot.robot_map[0]) - float(transaction.anchor_map[0]),
            float(snapshot.robot_map[1]) - float(transaction.anchor_map[1]),
        )
        required_travel = max(0.0, float(
            getattr(transaction, "minimum_travel_distance", 0.0)
        ))
        if travel_distance < required_travel:
            if not getattr(transaction, "exit_wait_reported", False):
                transaction.exit_wait_reported = True
                self.publish_status(
                    "frontier_place_departure_waiting",
                    region_id=int(source_region["id"]),
                    route_id=int(self.active_route_id),
                    departure_basis=transaction.basis,
                    travel_distance=round(travel_distance, 3),
                    required_travel_distance=round(required_travel, 3),
                    source=[
                        round(float(transaction.anchor_map[0]), 3),
                        round(float(transaction.anchor_map[1]), 3),
                    ],
                )
            return None
        robot_cell = self.xy_to_grid_cell(
            snapshot.message,
            snapshot.robot_map[0],
            snapshot.robot_map[1],
        )
        if not anchor_cells or robot_cell is None:
            return None
        footprint = self.observation_footprint_from_anchors(
            snapshot.message,
            snapshot.map_context.known_free,
            anchor_cells,
            components=snapshot.map_context.components,
        )
        if footprint[robot_cell]:
            return None
        self.publish_status(
            "frontier_place_boundary_crossed",
            region_id=int(source_region["id"]),
            route_id=int(self.active_route_id),
            departure_basis=transaction.basis,
            source=[
                round(float(transaction.anchor_map[0]), 3),
                round(float(transaction.anchor_map[1]), 3),
            ],
            travel_distance=round(travel_distance, 3),
            required_travel_distance=round(required_travel, 3),
            robot=[
                round(float(snapshot.robot_map[0]), 3),
                round(float(snapshot.robot_map[1]), 3),
            ],
        )
        return self.commit_active_place_departure()
