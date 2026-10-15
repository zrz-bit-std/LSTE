#!/usr/bin/env python3
"""Durable place-memory commit for structurally verified portal arrivals."""

import rospy


class GlobalFrontierPortalArrivalCommitMixin:
    """Write one verified portal arrival to place memory and report the result."""

    def _reject_portal_self_loop(self, arrival, portal_id, now, evidence_source):
        """Close an invalid crossing that resolved to its source Place.

        A self-loop is not an arrival that can wait for another SLAM snapshot:
        the physical destination evidence already says the selected doorway did
        not lead to a different Place.  Leaving the transaction in
        ``crossing_verified`` would permanently block successor planning after
        the controller terminal, so the edge is recorded as durable negative
        evidence and its source departure is released atomically.
        """
        ledger = getattr(self, "portal_hypothesis_ledger", None)
        failed = None
        if ledger is not None and portal_id > 0:
            fail = getattr(ledger, "failed", None)
            if callable(fail):
                failed = fail(portal_id, now=now)
        if failed is not None:
            self.publish_status(
                "portal_hypothesis_failed",
                portal_id=int(failed["id"]),
                source_place_id=int(failed["source_place_id"]),
                route_id=int(arrival.route_id),
                reason="portal_self_loop_rejected",
                failure_count=int(failed.get("failure_count", 0)),
                state=str(failed.get("state", "failed")),
                evidence_source=str(evidence_source),
            )

        transaction = getattr(self, "portal_transaction", None)
        if transaction is not None and transaction.active:
            transition = transaction.abort("portal_self_loop_rejected")
            if transition is not None:
                self.publish_status(
                    "portal_transaction_aborted",
                    transaction_id=int(transition.transaction_id),
                    route_id=int(transition.route_id),
                    portal_id=transition.portal_id,
                    source_place_id=transition.source_place_id,
                    state=transition.state,
                    reason=transition.last_reason,
                )
                finished = transaction.finish("portal_self_loop_rejected")
                if finished is not None:
                    self.publish_status(
                        "portal_transaction_finished",
                        transaction_id=int(finished.transaction_id),
                        portal_id=finished.portal_id,
                        source_place_id=finished.source_place_id,
                        state=finished.state,
                        reason=finished.last_reason,
                    )

        # The route crossed no new physical boundary.  Do not preserve the
        # source departure lease when terminal handling releases the route.
        self.active_portal_crossing_observed = False
        self.active_portal_crossing_rejected = True
        clear_departure = getattr(self, "_clear_active_place_departure", None)
        if callable(clear_departure):
            clear_departure()
        else:
            departure = getattr(self, "place_departure", None)
            clear = getattr(departure, "clear", None)
            if callable(clear):
                clear()

    def commit_portal_arrival(self, arrival, component, now, evidence_source):
        """Enter the destination place after a structural identity is available."""
        transaction = getattr(self, "portal_transaction", None)
        portal_id = int(getattr(self, "last_portal_hypothesis_id", 0) or 0)
        if transaction is not None and not transaction.commit_authorized(
            route_id=getattr(arrival, "route_id", None),
            portal_id=(portal_id if portal_id > 0 else None),
            source_place_id=getattr(arrival, "source_region_id", None),
        ):
            self.publish_status(
                "portal_place_arrival_rejected",
                route_id=int(getattr(arrival, "route_id", 0)),
                goal=[
                    round(float(arrival.goal_xy[0]), 3),
                    round(float(arrival.goal_xy[1]), 3),
                ],
                reason="stale_or_mismatched_portal_transaction",
                evidence_source=str(evidence_source),
                transaction_id=int(transaction.snapshot().transaction_id),
                transaction_route_id=int(transaction.snapshot().route_id),
                transaction_portal_id=transaction.snapshot().portal_id,
                transaction_source_place_id=transaction.snapshot().source_place_id,
            )
            rospy.logwarn(
                "Global frontier rejected stale portal arrival route_id=%d "
                "transaction_route_id=%d",
                int(getattr(arrival, "route_id", 0)),
                int(transaction.snapshot().route_id),
            )
            return None
        # Once a Portal has a destination Place, its directed identity is
        # stronger than the current SLAM component.  In particular, reverse
        # egress approaches the same gate from the opposite half-plane, so
        # ordinary entry geometry intentionally cannot match it.  Bind that
        # reverse arrival directly to the edge's source Place instead of
        # creating a duplicate region from a transient component.
        bound_destination = None
        hypothesis_ledger = getattr(self, "portal_hypothesis_ledger", None)
        hypothesis = (
            None
            if hypothesis_ledger is None or portal_id <= 0
            else hypothesis_ledger.get(portal_id)
        )
        try:
            if (
                isinstance(hypothesis, dict)
                and int(hypothesis.get("destination_place_id"))
                == int(arrival.source_region_id)
                and int(hypothesis.get("source_place_id"))
                != int(arrival.source_region_id)
            ):
                bound_destination = int(hypothesis["source_place_id"])
        except (TypeError, ValueError):
            bound_destination = None
        enter_bound = getattr(
            self.region_memory, "enter_bound_portal_destination", None,
        )
        if bound_destination is not None and callable(enter_bound):
            region, tier = enter_bound(
                bound_destination,
                arrival.goal_xy[0],
                arrival.goal_xy[1],
                now,
                component=component,
                entry_portal=arrival.portal_gate_xy,
                physical_xy=arrival.physical_goal_xy,
                physical_entry_portal=arrival.physical_portal_gate_xy,
                source_place_id=arrival.source_region_id,
            )
        else:
            # A connected structural core is not proof that both sides of a
            # physical doorway are the same Place.  If this Portal transaction
            # is already crossing-verified and the current component resolves
            # to its source, create a new destination Place from the durable
            # edge identity.  Ordinary arrivals retain the conservative
            # nearest-component association.
            destination_enter = getattr(
                self.region_memory, "enter_new_portal_destination", None,
            )
            source_component_match = False
            if component is not None:
                try:
                    _candidate_tier, candidate_region = (
                        self.region_memory.candidate_tier(
                            arrival.goal_xy[0],
                            arrival.goal_xy[1],
                            0.0,
                            component=component,
                            physical_xy=arrival.physical_goal_xy,
                        )
                    )
                    source_component_match = (
                        candidate_region is not None
                        and int(candidate_region["id"])
                        == int(arrival.source_region_id)
                    )
                except (AttributeError, KeyError, TypeError, ValueError):
                    source_component_match = False
            force_new_destination = bool(
                destination_enter is not None
                and transaction is not None
                and str(transaction.state) == "crossing_verified"
                and transaction.commit_authorized(
                    route_id=arrival.route_id,
                    portal_id=(portal_id if portal_id > 0 else None),
                    source_place_id=arrival.source_region_id,
                )
                and (component is None or source_component_match)
            )
            enter = (
                destination_enter
                if force_new_destination
                else self.region_memory.enter
            )
            region, tier = enter(
                arrival.goal_xy[0],
                arrival.goal_xy[1],
                now,
                component=component,
                entry_portal=arrival.portal_gate_xy,
                physical_xy=arrival.physical_goal_xy,
                physical_entry_portal=arrival.physical_portal_gate_xy,
                source_place_id=arrival.source_region_id,
            )
            if force_new_destination:
                self.publish_status(
                    "portal_place_component_fallback",
                    route_id=int(arrival.route_id),
                    source_place_id=int(arrival.source_region_id),
                    reason=(
                        "physical_portal_crossing_without_component"
                        if component is None
                        else "physical_portal_crossing_overrode_connected_component"
                    ),
                    component=(
                        None
                        if component is None
                        else {
                            "epoch": int(component["epoch"]),
                            "label": int(component["label"]),
                        }
                    ),
                )
        if region is None:
            self.publish_status(
                "portal_place_arrival_rejected",
                route_id=int(arrival.route_id),
                goal=[
                    round(float(arrival.goal_xy[0]), 3),
                    round(float(arrival.goal_xy[1]), 3),
                ],
                reason="destination_region_%s" % str(tier),
                evidence_source=str(evidence_source),
            )
            rospy.logwarn(
                "Global frontier reached portal destination with unavailable "
                "place state=%s route_id=%d",
                tier,
                int(arrival.route_id),
            )
            return None
        # A transient structural component can still associate the crossing
        # endpoint with the source Place after the route has physically passed
        # the doorway.  Re-materialize a destination Place from the durable
        # crossing identity before any arrival/observation evidence is
        # committed; otherwise the graph would record a Place->same-Place
        # self-loop.
        if (
            portal_id > 0
            and getattr(arrival, "source_region_id", None) is not None
        ):
            try:
                source_place_id = int(arrival.source_region_id)
                same_source_region = int(region["id"]) == source_place_id
            except (KeyError, TypeError, ValueError):
                same_source_region = False
                source_place_id = None
            if same_source_region:
                destination_enter = getattr(
                    self.region_memory, "enter_new_portal_destination", None,
                )
                can_rematerialize = bool(
                    callable(destination_enter)
                    and transaction is not None
                    and transaction.commit_authorized(
                        route_id=arrival.route_id,
                        portal_id=portal_id,
                        source_place_id=source_place_id,
                    )
                )
                if can_rematerialize:
                    rematerialized, rematerialized_tier = destination_enter(
                        arrival.goal_xy[0],
                        arrival.goal_xy[1],
                        now,
                        component=component,
                        entry_portal=arrival.portal_gate_xy,
                        physical_xy=arrival.physical_goal_xy,
                        physical_entry_portal=arrival.physical_portal_gate_xy,
                        source_place_id=arrival.source_region_id,
                    )
                    if (
                        rematerialized is not None
                        and int(rematerialized.get("id", -1)) != source_place_id
                    ):
                        region, tier = rematerialized, rematerialized_tier
                        self.publish_status(
                            "portal_place_component_fallback",
                            route_id=int(arrival.route_id),
                            source_place_id=source_place_id,
                            reason=(
                                "self_loop_prevented_by_portal_destination_materialization"
                            ),
                        )
        try:
            self_loop = (
                portal_id > 0
                and int(region.get("id"))
                == int(getattr(arrival, "source_region_id", -1))
            )
        except (AttributeError, TypeError, ValueError):
            self_loop = False
        if self_loop:
            self._reject_portal_self_loop(
                arrival,
                portal_id,
                now,
                evidence_source,
            )
            self.publish_status(
                "portal_place_arrival_rejected",
                route_id=int(arrival.route_id),
                portal_id=portal_id,
                source_place_id=int(arrival.source_region_id),
                destination_place_id=int(region.get("id")),
                reason="portal_self_loop_rejected",
                evidence_source=str(evidence_source),
            )
            return None
        # If an unbound branch lands in an already-known Place, the source
        # Place may still own suspended local obligations. Evaluate this only
        # after rematerialization and self-loop rejection so a discarded or
        # repaired destination cannot leave a stale resume marker behind.
        source_resume_id = None
        source_resume_work = 0
        source_resume_portals = 0
        try:
            hypothesis_source_id = int(
                hypothesis.get("source_place_id")
            ) if isinstance(hypothesis, dict) else None
            destination_id = int(region.get("id"))
        except (TypeError, ValueError, AttributeError):
            hypothesis_source_id = None
            destination_id = None
        if (
            isinstance(hypothesis, dict)
            and hypothesis.get("destination_place_id") is None
            and hypothesis_source_id is not None
            and destination_id is not None
            and hypothesis_source_id != destination_id
            and str(tier) == "covered_arrival"
        ):
            work_items = getattr(self, "place_work_items", None)
            if work_items is not None:
                count = getattr(work_items, "unresolved_observation_count", None)
                if callable(count):
                    try:
                        source_resume_work = int(
                            count(
                                hypothesis_source_id,
                                portal_probe_ledger=getattr(
                                    self, "portal_probe_ledger", None
                                ),
                            )
                        )
                    except TypeError:
                        source_resume_work = int(count(hypothesis_source_id))
                else:
                    count = getattr(work_items, "unresolved_count", None)
                    if callable(count):
                        source_resume_work = int(count(hypothesis_source_id))
            probe_ledger = getattr(self, "portal_probe_ledger", None)
            count = (
                None if probe_ledger is None
                else getattr(probe_ledger, "unresolved_count", None)
            )
            if callable(count):
                source_resume_portals = int(count(hypothesis_source_id))
            if source_resume_work > 0 or source_resume_portals > 0:
                source_resume_id = hypothesis_source_id
                self.pending_graph_resume_place_id = source_resume_id
                self.pending_graph_resume_portal_id = portal_id
                self.pending_graph_resume_destination_place_id = destination_id
                self.publish_status(
                    "graph_place_resume_pending",
                    source_place_id=source_resume_id,
                    destination_place_id=destination_id,
                    portal_id=portal_id,
                    unresolved_work_items=source_resume_work,
                    unresolved_portals=source_resume_portals,
                    reason="covered_arrival_from_unbound_branch",
                )
        # The crossing route terminates inside the destination component and
        # its lidar has already produced a destination-side scan.  Record that
        # first local view separately from the Portal crossing itself.  Any
        # unknown boundary exposed by later snapshots is still reconciled into
        # destination-owned WorkItems; this fact only prevents an empty room
        # from remaining permanently in the bootstrap phase.
        entry_observe = getattr(self.region_memory, "endpoint_observed", None)
        if (
            callable(entry_observe)
            and region.get("state") == "open"
            and int(region.get("endpoint_observations", 0)) == 0
        ):
            observed_region = entry_observe(
                arrival.goal_xy[0],
                arrival.goal_xy[1],
                now,
                component=component,
                region_id=region["id"],
                physical_xy=arrival.physical_goal_xy,
            )
            if observed_region is not None:
                self.publish_status(
                    "portal_place_entry_observed",
                    place_id=int(observed_region["id"]),
                    route_id=int(arrival.route_id),
                    viewpoint=[
                        round(float(arrival.goal_xy[0]), 3),
                        round(float(arrival.goal_xy[1]), 3),
                    ],
                    physical_viewpoint=(
                        None
                        if arrival.physical_goal_xy is None
                        else [
                            round(float(arrival.physical_goal_xy[0]), 3),
                            round(float(arrival.physical_goal_xy[1]), 3),
                        ]
                    ),
                    evidence_source="portal_destination_arrival",
                )
        if transaction is not None:
            transition = transaction.place_commit(
                reason="destination_place_committed",
                route_id=int(arrival.route_id),
                portal_id=(portal_id if portal_id > 0 else None),
                source_place_id=arrival.source_region_id,
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
                if getattr(self, "active_frontier", None) is None:
                    finished = transaction.finish("portal_transaction_finished")
                    if finished is not None:
                        self.publish_status(
                            "portal_transaction_finished",
                            transaction_id=int(finished.transaction_id),
                            portal_id=finished.portal_id,
                            source_place_id=finished.source_place_id,
                            state=finished.state,
                            reason=finished.last_reason,
                        )
        event = (
            "portal_place_covered_arrival"
            if tier == "covered_arrival"
            else "portal_place_entered"
        )
        # The destination is now the only durable owner for local frontier
        # work.  Current SLAM components may split it around furniture, but
        # those splits cannot mint another physical Place.
        portal_id = int(getattr(self, "last_portal_hypothesis_id", 0) or 0)
        hypothesis_ledger = getattr(self, "portal_hypothesis_ledger", None)
        crossed = (
            None
            if hypothesis_ledger is None or portal_id <= 0
            else hypothesis_ledger.crossed(portal_id, now=now)
        )
        if crossed is not None:
            prior_destination = crossed.get("destination_place_id")
            self.publish_status(
                "portal_hypothesis_crossed",
                portal_id=int(crossed["id"]),
                source_place_id=int(crossed["source_place_id"]),
                route_id=int(arrival.route_id),
                traversal_source_place_id=getattr(
                    arrival, "source_region_id", None,
                ),
                traversal_destination_place_id=int(region["id"]),
                crossing_count=int(crossed.get("crossing_count", 0)),
            )
            # Destination identity is established exactly once, on the first
            # forward crossing. A later reverse traversal reuses the same
            # physical edge and must never overwrite its original endpoint
            # with the Place it is returning to.
            if prior_destination is None:
                bound = hypothesis_ledger.bind_destination(
                    crossed["id"],
                    region["id"],
                    now=now,
                )
                if bound is None:
                    self.publish_status(
                        "portal_place_arrival_rejected",
                        route_id=int(arrival.route_id),
                        portal_id=int(crossed["id"]),
                        source_place_id=int(crossed["source_place_id"]),
                        destination_place_id=int(region["id"]),
                        reason="portal_destination_binding_rejected",
                        evidence_source=str(evidence_source),
                    )
                    return None
                self.publish_status(
                    "portal_hypothesis_destination_bound",
                    portal_id=int(crossed["id"]),
                    source_place_id=int(crossed["source_place_id"]),
                    destination_place_id=int(bound["destination_place_id"]),
                    route_id=int(arrival.route_id),
                )
                mark_branch_transit = getattr(
                    self, "mark_directional_branch_transit", None
                )
                if callable(mark_branch_transit):
                    mark_branch_transit(
                        crossed["source_place_id"],
                        crossed["id"],
                        bound["destination_place_id"],
                        now,
                        event_id="portal:%d:route:%d"
                        % (int(crossed["id"]), int(arrival.route_id)),
                    )
        # Destination-place commit is the atomic boundary of a Portal
        # transaction.  Do not leave the source departure lease pending until
        # the next timer tick: an immediate successor selection could then
        # reuse the old source identity and materialize a duplicate Place on
        # reverse transit.  The crossing proof has already been verified by
        # ``record_portal_place_arrival`` before this method is reached.
        departure = getattr(self, "place_departure", None)
        commit_departure = getattr(self, "commit_active_place_departure", None)
        if (
            callable(commit_departure)
            and bool(getattr(departure, "active", False))
        ):
            try:
                departure_source = int(getattr(departure, "region_id"))
                arrival_source = int(getattr(arrival, "source_region_id"))
            except (TypeError, ValueError):
                departure_source = arrival_source = None
            if departure_source == arrival_source:
                commit_departure()
            else:
                self.publish_status(
                    "portal_departure_identity_mismatch",
                    route_id=int(arrival.route_id),
                    departure_source_place_id=departure_source,
                    arrival_source_place_id=arrival_source,
                    destination_place_id=int(region["id"]),
                    reason="stale_departure_lease_not_committed",
                )
        self.current_physical_place_id = int(region["id"])
        # The Place identity is now authoritative, but its local observation
        # ledger is still snapshot-local.  The next selection pass must first
        # reconcile one map projection for this Place before graph transit can
        # be selected.  This avoids a false empty-room decision at the exact
        # boundary where a Portal terminal and a new SLAM snapshot race.
        self.place_entry_rehydration_pending = int(region["id"])
        clear_lease = getattr(self, "_clear_graph_route_plan_lease", None)
        if clear_lease is not None:
            clear_lease("portal_place_entered")
        self.publish_status(
            event,
            route_id=int(arrival.route_id),
            portal_id=(None if portal_id <= 0 else int(portal_id)),
            region_id=int(region["id"]),
            region_tier=str(tier),
            region_visits=int(region["visits"]),
            portal_arrivals=int(region.get("portal_arrivals", 0)),
            covered_arrivals=int(region.get("covered_arrivals", 0)),
            goal=[
                round(float(arrival.goal_xy[0]), 3),
                round(float(arrival.goal_xy[1]), 3),
            ],
            portal_gate=(
                None
                if arrival.portal_gate_xy is None
                else [
                    round(float(arrival.portal_gate_xy[0]), 3),
                    round(float(arrival.portal_gate_xy[1]), 3),
                ]
            ),
            physical_goal=(
                None
                if arrival.physical_goal_xy is None
                else [
                    round(float(arrival.physical_goal_xy[0]), 3),
                    round(float(arrival.physical_goal_xy[1]), 3),
                ]
            ),
            component={
                "epoch": int(component["epoch"]),
                "label": int(component["label"]),
            } if component is not None else None,
            evidence_source=str(evidence_source),
        )
        rospy.loginfo(
            "Global frontier recorded portal destination region=%d tier=%s "
            "route_id=%d evidence=%s",
            int(region["id"]),
            tier,
            int(arrival.route_id),
            evidence_source,
        )
        return region
