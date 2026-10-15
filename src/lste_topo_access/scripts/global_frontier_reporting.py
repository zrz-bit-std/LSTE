#!/usr/bin/env python3
"""ROS status and route-transaction reporting for global-frontier exploration.

The exploration modules update in-memory route state.  This mixin is the only
place that converts that state into the JSON messages consumed by the goal
manager and the benchmark tools.  Keeping this boundary separate prevents
transport formatting from leaking into planning and execution decisions.
"""

import json

import rospy
from std_msgs.msg import String

from global_frontier_place_progress import derive_place_progress
from global_frontier_transition import build_transition_envelope

from goal_context import default_goal_context


class GlobalFrontierReportingMixin:
    def active_region_report(self):
        """Return stable place-lifecycle context for one status message.

        A route id alone cannot show whether the selector revisited an
        observed place.  Include the durable region-memory id and state in
        every event so the benchmark can verify that route transitions respect
        the online place graph without depending on temporary grid labels.
        """
        region_id = getattr(self, "active_frontier_region_id", None)
        if region_id is None:
            return None, None
        region = self.region_memory.by_id(region_id)
        return int(region_id), None if region is None else region.get("state")

    @staticmethod
    def _optional_place_id(value):
        """Return a JSON-safe positive Place/WorkItem identity or ``None``."""
        try:
            value = int(value)
        except (TypeError, ValueError):
            return None
        return value if value > 0 else None

    def active_goal_context(self, route_kind=None, graph_transaction_id=None):
        """Describe the durable owner of the active geometric command.

        ``/map`` may move a command point while a route is executing.  This
        context is deliberately independent of that coordinate: it says
        whether the command is ordinary geometry-only exploration, work owned
        by one physical Place, a certified doorway crossing, or a retreat
        through known space.  It is telemetry and an execution contract, not
        a second controller policy.
        """
        execution_kind = str(
            route_kind or getattr(self, "active_route_kind", "frontier_endpoint")
        ).strip().lower() or "frontier_endpoint"
        method = str(
            getattr(self, "exploration_method", "place_portal_workitem")
        ).strip().lower() or "place_portal_workitem"
        context = default_goal_context(
            method,
            task_id=str(getattr(self, "current_task_id", "") or ""),
            mission_id=str(getattr(self, "current_mission_id", "") or ""),
            task_version=str(getattr(self, "current_task_version", "") or ""),
        )
        graph_transaction = getattr(
            self, "graph_route_action_transaction", None
        )
        if graph_transaction_id is None:
            graph_transaction_id = getattr(
                graph_transaction, "graph_transaction_id", 0
            )
        context["graph_transaction_id"] = int(
            graph_transaction_id or 0
        )
        if not getattr(self, "place_memory_enabled", True):
            return context

        active_place_id = self._optional_place_id(
            getattr(self, "active_frontier_region_id", None)
        )
        current_place_id = self._optional_place_id(
            getattr(self, "current_physical_place_id", None)
        )
        departure = getattr(self, "place_departure", None)
        departure_active = bool(getattr(departure, "active", False))
        departure_place_id = self._optional_place_id(
            getattr(departure, "region_id", None)
        )
        source_place_id = departure_place_id or current_place_id or active_place_id
        work_item_id = self._optional_place_id(
            getattr(self, "active_work_item_id", None)
        )
        portal_probe_id = self._optional_place_id(
            getattr(self, "active_portal_probe_id", None)
        )

        if portal_probe_id is not None:
            probe_phase = str(
                getattr(self, "active_portal_probe_phase", "") or ""
            ).strip().lower()
            if not probe_phase:
                probe_ledger = getattr(self, "portal_probe_ledger", None)
                probe_record = (
                    None
                    if probe_ledger is None
                    else probe_ledger.get(portal_probe_id)
                )
                probe_phase = str(
                    (probe_record or {}).get("active_phase") or "source"
                ).strip().lower()
            context.update({
                "goal_role": "portal_probe",
                "owner_place_id": active_place_id or current_place_id,
                "source_place_id": source_place_id,
                "portal_probe_id": portal_probe_id,
                "portal_probe_phase": probe_phase,
                "work_item_id": work_item_id,
            })
            return context

        if work_item_id is not None:
            context.update({
                "goal_role": "place_observation_work",
                "owner_place_id": active_place_id or current_place_id,
                "source_place_id": source_place_id,
                "work_item_id": work_item_id,
            })
            return context

        if execution_kind == "portal_transition":
            gate_odom = getattr(self, "active_portal_gate_odom_xy", None)
            if gate_odom is not None:
                try:
                    gate_odom = [round(float(gate_odom[0]), 4), round(float(gate_odom[1]), 4)]
                except (IndexError, TypeError, ValueError):
                    gate_odom = None
            context.update({
                "goal_role": "certified_portal_crossing",
                "source_place_id": source_place_id,
                "portal_crossing_certified": gate_odom is not None,
                "portal_gate_odom": gate_odom,
            })
            return context

        if execution_kind == "local_egress":
            lease = getattr(self, "local_egress_place_lease", None)
            lease_place_id = self._optional_place_id(
                getattr(lease, "source_region_id", None)
            )
            context.update({
                "goal_role": "place_egress",
                "source_place_id": lease_place_id or source_place_id,
            })
            return context

        if departure_active and bool(getattr(departure, "transit_only", False)):
            context.update({
                "goal_role": "covered_place_transit",
                "source_place_id": source_place_id,
            })
            return context

        if active_place_id is not None:
            context.update({
                "goal_role": "place_observation",
                "owner_place_id": active_place_id,
                "source_place_id": source_place_id,
            })
        return context

    def semantic_place_report(self):
        """Return the task-conditioned belief used by the selector."""
        belief = getattr(self, "semantic_place_belief", None)
        if belief is None:
            return None
        place_id = getattr(self, "current_physical_place_id", None)
        evidence = belief.evidence(place_id) if place_id is not None else None
        region = (
            None
            if place_id is None
            else self.region_memory.by_id(place_id)
        )
        place_observed = bool(
            region is not None
            and int(region.get("endpoint_observations", 0)) > 0
        )
        ledger = getattr(self, "place_work_items", None)
        local_work_pending = (
            True
            if ledger is None or place_id is None
            else bool(ledger.unresolved_count(place_id) > 0)
        )
        selection_context = getattr(self, "last_selection_context", None)
        context_pending = getattr(self, "context_has_pending_local_work", None)
        if (
            selection_context is not None
            and getattr(selection_context, "source_place_id", None) == place_id
            and context_pending is not None
        ):
            local_work_pending = bool(context_pending(selection_context))
            local_work_pending = local_work_pending or bool(
                getattr(self, "last_portal_probe_candidates", 0) > 0
            )
        probe_ledger = getattr(self, "portal_probe_ledger", None)
        if (
            selection_context is None
            and probe_ledger is not None
            and place_id is not None
        ):
            local_work_pending = local_work_pending or bool(
                probe_ledger.unresolved_count(place_id) > 0
            )
        action = getattr(self, "semantic_place_action", None)
        policy = (
            None
            if action is None
            else action(place_id, place_observed, local_work_pending)
        )
        return {
            "task_id": str(getattr(self, "current_task_id", "") or "") or None,
            "task_version": str(getattr(self, "current_task_version", "") or "") or None,
            "frontier_action_policy": str(
                getattr(self, "frontier_action_policy", "legacy_scalar")
            ),
            "place_id": None if place_id is None else int(place_id),
            "policy": policy,
            "local_work_pending": local_work_pending,
            "target_reinspection_pending": bool(
                getattr(self, "target_reinspection_pending", False)
            ),
            "evidence": evidence,
            "target_belief": (
                None
                if getattr(self, "target_belief", None) is None
                else self.target_belief.evidence(place_id)
            ),
            "target_observation_work": (
                None
                if getattr(self, "target_observation_work", None) is None
                else self.target_observation_work.evidence(place_id)
            ),
        }

    def portal_hypothesis_report(self):
        """Return compact durable portal state for run-time diagnostics."""
        ledger = getattr(self, "portal_hypothesis_ledger", None)
        if ledger is None:
            return None
        counts = {}
        records = []
        for record in ledger.snapshot():
            state = str(record.get("state", "unknown"))
            counts[state] = counts.get(state, 0) + 1
            # Keep the physical witness visible in the run log.  It is the
            # identity layer that must remain stable while map projections
            # change, and it is small enough to include in every status
            # snapshot without dumping the entire ledger.
            records.append({
                "id": int(record.get("id", 0)),
                "source_place_id": record.get("source_place_id"),
                "destination_place_id": record.get("destination_place_id"),
                "state": state,
                "physical_gate": record.get("physical_gate"),
                "physical_destination": record.get("physical_destination"),
                "map_gate": record.get("map_gate"),
                "map_destination": record.get("map_destination"),
                "selection_count": int(record.get("selection_count", 0)),
                "crossing_count": int(record.get("crossing_count", 0)),
                "failure_count": int(record.get("failure_count", 0)),
                "execution_failure_count": int(
                    record.get("execution_failure_count", 0)
                ),
                "rejected_map_epoch": record.get("rejected_map_epoch"),
                "last_rejection_reason": record.get("last_rejection_reason", ""),
            })
        return {
            "active_id": int(getattr(self, "last_portal_hypothesis_id", 0) or 0),
            "count": int(sum(counts.values())),
            "states": counts,
            "records": records,
        }

    def portal_probe_report(self):
        """Return compact source-side probe obligations for every status."""
        ledger = getattr(self, "portal_probe_ledger", None)
        if ledger is None:
            return None
        counts = {}
        for record in ledger.snapshot():
            state = str(record.get("state", "unknown"))
            counts[state] = counts.get(state, 0) + 1
        return {
            "active_id": int(getattr(self, "active_portal_probe_id", 0) or 0),
            "count": int(sum(counts.values())),
            "states": counts,
        }

    def work_item_report(self):
        """Expose durable WorkItem geometry and Attempt lineage in status."""
        ledger = getattr(self, "place_work_items", None)
        if ledger is None:
            return None
        snapshot = getattr(ledger, "snapshot", None)
        if not callable(snapshot):
            return None
        return snapshot()

    def portal_transaction_report(self):
        """Return the active physical-edge lease for every run-time event."""
        transaction = getattr(self, "portal_transaction", None)
        if transaction is None:
            return None
        snapshot = transaction.snapshot()
        return {
            "state": str(snapshot.state),
            "transaction_id": int(snapshot.transaction_id),
            "route_id": int(snapshot.route_id),
            "portal_id": snapshot.portal_id,
            "source_place_id": snapshot.source_place_id,
            "gate": snapshot.gate_xy,
            "destination": snapshot.destination_xy,
            "retry_count": int(snapshot.retry_count),
            "source_side_proven": bool(snapshot.source_side_proven),
            "source_signed_distance": snapshot.source_signed_distance,
            "active_crossing_observed": bool(
                getattr(self, "active_portal_crossing_observed", False)
            ),
            "active_crossing_rejected": bool(
                getattr(self, "active_portal_crossing_rejected", False)
            ),
            "last_reason": str(snapshot.last_reason),
            "last_transition": str(snapshot.last_transition),
        }

    def portal_selection_audit_report(self):
        """Expose candidate-vs-executable counts for graph recovery.

        A structural Portal source is only a hypothesis. Keeping these counts
        beside every status event prevents recovery and benchmark analysis from
        treating a rejected candidate as an executable transit action.
        """
        return {
            "candidates_seen": int(
                getattr(self, "last_portal_candidates_seen", 0)
            ),
            "hard_rejected": int(
                getattr(self, "last_portal_hard_rejected", 0)
            ),
            "executable_candidates": int(
                getattr(self, "last_portal_executable_candidates", 0)
            ),
            "structural_boundary_candidates": int(
                getattr(self, "last_structural_boundary_candidates", 0)
            ),
            "structural_boundary_rejections": int(
                getattr(self, "last_structural_boundary_rejections", 0)
            ),
            "structural_boundary_source_place_rejections": int(
                getattr(
                    self,
                    "last_structural_boundary_source_place_rejections",
                    0,
                )
            ),
        }

    def directional_branch_report(self):
        """Expose persistent branch coverage without leaking map geometry."""
        ledger = getattr(self, "directional_branch_coverage", None)
        if ledger is None:
            return None
        snapshots = tuple(ledger.snapshots())
        states = {}
        for branch in snapshots:
            state = str(branch.state or "unknown")
            states[state] = states.get(state, 0) + 1
        return {
            "count": len(snapshots),
            "states": states,
            "pending_work_items": len(ledger.pending_work_items()),
            "transit_edges": sum(1 for branch in snapshots if branch.transit),
            "reopens": sum(int(branch.reopen_count) for branch in snapshots),
        }

    def place_progress_report(self):
        """Expose the durable Place phase used by graph action admission."""
        place_id = getattr(self, "current_physical_place_id", None)
        region_memory = getattr(self, "region_memory", None)
        region = (
            None
            if region_memory is None or place_id is None
            else region_memory.by_id(place_id)
        )
        unresolved_work = 0
        work_items = getattr(self, "place_work_items", None)
        if work_items is not None and place_id is not None:
            count = getattr(work_items, "unresolved_observation_count", None)
            if callable(count):
                unresolved_work = count(
                    place_id,
                    portal_probe_ledger=getattr(
                        self, "portal_probe_ledger", None,
                    ),
                )
            else:
                count = getattr(work_items, "unresolved_count", None)
                if callable(count):
                    unresolved_work = count(place_id)
        unresolved_portals = 0
        probes = getattr(self, "portal_probe_ledger", None)
        if probes is not None and place_id is not None:
            count = getattr(probes, "unresolved_count", None)
            if callable(count):
                unresolved_portals = count(place_id)
        progress = derive_place_progress(
            place_id=place_id,
            state=None if region is None else region.get("state"),
            observed=(
                False
                if region is None
                else int(region.get("endpoint_observations", 0)) > 0
            ),
            unresolved_work_items=unresolved_work,
            unresolved_portals=unresolved_portals,
        )
        return {
            "place_id": progress.place_id,
            "phase": progress.phase,
            "observed": bool(progress.observed),
            "covered": bool(progress.covered),
            "unresolved_work_items": int(progress.unresolved_work_items),
            "unresolved_portals": int(progress.unresolved_portals),
            "local_observation_complete": bool(
                progress.local_observation_complete
            ),
            "has_durable_progress": bool(progress.has_durable_progress),
            "reason": progress.reason,
        }

    def publish_status(self, event, **fields):
        """Publish one lifecycle event with the current route context."""
        active_region_id, active_region_state = self.active_region_report()
        pending_goal = (
            None
            if self.prefetched_frontier is None
            else [
                round(float(self.prefetched_frontier[2]), 3),
                round(float(self.prefetched_frontier[3]), 3),
            ]
        )
        graph_plan = getattr(self, "last_graph_route_plan", None)
        graph_plan_payload = None
        if graph_plan is not None:
            as_dict = getattr(graph_plan, "as_dict", None)
            graph_plan_payload = (
                as_dict() if callable(as_dict) else graph_plan
            )
        graph_transaction = getattr(
            self, "graph_route_action_transaction", None
        )
        graph_transaction_payload = (
            None
            if graph_transaction is None
            else graph_transaction.as_dict()
        )
        graph_transaction_id = (
            0
            if graph_transaction is None
            else int(getattr(graph_transaction, "graph_transaction_id", 0) or 0)
        )
        map_epoch = (
            getattr(graph_transaction, "map_epoch", None)
            if graph_transaction is not None
            else getattr(self, "last_map_epoch", None)
        )

        def identity_integer(value, default=0):
            try:
                return int(value or default)
            except (TypeError, ValueError):
                return int(default)

        released_route_id = identity_integer(
            getattr(self, "last_released_route_id", 0)
        )
        active_route_id = identity_integer(
            getattr(self, "active_route_id", 0)
        )
        action_generation = fields.get("action_generation")
        if action_generation is None:
            action_generation = identity_integer(
                getattr(self, "active_action_generation", 0)
            )
        if (
            identity_integer(action_generation) <= 0
            and released_route_id > 0
            and released_route_id == max(active_route_id, released_route_id)
        ):
            action_generation = identity_integer(
                getattr(self, "last_released_route_action_generation", 0)
            )
        transaction_id = fields.get("transaction_id")
        if transaction_id is None:
            transaction_id = graph_transaction_id
        if identity_integer(transaction_id) <= 0:
            transaction_id = identity_integer(
                getattr(self, "last_released_route_transaction_id", 0)
            )
        payload = {
            "event": str(event),
            "active": self.active_frontier is not None,
            # Keep the experiment contract at the top level of every status
            # event.  The same method identity also appears in goal_context,
            # but benchmark reducers should not need to infer a run contract
            # from a route-specific nested object.
            "exploration_method": str(
                getattr(self, "exploration_method", "place_portal_workitem")
            ).strip().lower() or "place_portal_workitem",
            "frontier_action_policy": str(
                getattr(self, "frontier_action_policy", "legacy_scalar")
            ).strip().lower() or "legacy_scalar",
            "target_done_policy": str(
                getattr(self, "target_done_policy", "visual_target")
            ).strip().lower() or "visual_target",
            "pending": self.prefetched_frontier is not None,
            # A prefetched point is lifecycle evidence, never an executable
            # goal.  It becomes a command only after route promotion.
            "pending_goal": pending_goal,
            "pending_route_id": (
                int(self.active_route_id + 1)
                if pending_goal is not None and self.active_route_id > 0
                else 0
            ),
            "turn_supervisor_state": self.turn_supervisor_state,
            "turn_supervisor_route_kind": getattr(
                self, "turn_supervisor_route_kind", ""
            ),
            "turn_supervisor_turn_phase": getattr(
                self, "turn_supervisor_turn_phase", ""
            ),
            "turn_supervisor_yaw_error": getattr(
                self, "turn_supervisor_yaw_error", None
            ),
            "turn_supervisor_target_yaw": getattr(
                self, "turn_supervisor_target_yaw", None
            ),
            "route_id": int(self.active_route_id),
            # Keep all graph and controller handoff events independently
            # traceable, even when a caller has no extra event-specific fields.
            "transaction_id": identity_integer(transaction_id),
            "action_generation": identity_integer(action_generation),
            "lifecycle_transaction_id": int(
                getattr(self, "route_lifecycle_transaction_id", self.lifecycle_transaction_id)()
                or 0
            ),
            "active_region_id": active_region_id,
            "active_region_state": active_region_state,
            "released_controller_route": {
                "route_id": int(
                    getattr(self, "last_released_route_id", 0) or 0
                ),
                "route_kind": str(
                    getattr(self, "last_released_route_kind", "") or ""
                ),
                "transaction_id": int(
                    getattr(self, "last_released_route_transaction_id", 0)
                    or 0
                ),
                "terminal_received": bool(
                    getattr(
                        self, "last_released_route_terminal_received", False
                    )
                ),
                "controller_pending": bool(
                    getattr(
                        self, "last_released_route_controller_pending", False
                    )
                ),
                "lifecycle_transaction_id": int(
                    getattr(
                        self,
                        "last_released_route_lifecycle_transaction_id",
                        0,
                    )
                    or 0
                ),
                "action_generation": int(
                    getattr(self, "last_released_route_action_generation", 0)
                    or 0
                ),
                "graph_transaction_id": int(
                    getattr(
                        self, "last_released_route_graph_transaction_id", 0
                    )
                    or 0
                ),
                "map_epoch": getattr(
                    self, "last_released_route_map_epoch", None
                ),
                "origin_epoch": getattr(
                    self, "last_released_route_origin_epoch", None
                ),
                "projection_epoch": getattr(
                    self, "last_released_route_projection_epoch", None
                ),
            },
            "awaiting_controller_lease_release_ack": bool(
                getattr(self, "awaiting_controller_lease_release_ack", False)
            ),
            "pending_controller_lease_release": getattr(
                self, "pending_controller_lease_release", None
            ),
            "pending_controller_lease_releases": getattr(
                self, "pending_controller_lease_releases", None
            ),
            "pending_controller_lease_release_count": len(
                getattr(self, "pending_controller_lease_releases", {}) or {}
            ),
            "last_controller_lease_release_ack": getattr(
                self, "last_controller_lease_release_ack", None
            ),
            "goal_context": self.active_goal_context(),
            "semantic_place": self.semantic_place_report(),
            "place_progress": self.place_progress_report(),
            "place_entry_rehydration_pending": getattr(
                self, "place_entry_rehydration_pending", None
            ),
            "pending_graph_resume": {
                "place_id": getattr(self, "pending_graph_resume_place_id", None),
                "portal_id": getattr(self, "pending_graph_resume_portal_id", None),
                "destination_place_id": getattr(
                    self, "pending_graph_resume_destination_place_id", None
                ),
            },
            "portal_hypotheses": self.portal_hypothesis_report(),
            "portal_probes": self.portal_probe_report(),
            "work_items": self.work_item_report(),
            "portal_selection_audit": self.portal_selection_audit_report(),
            "directional_branch_coverage": self.directional_branch_report(),
            "portal_transaction": self.portal_transaction_report(),
            "graph_route_plan": graph_plan_payload,
            "graph_route_action_transaction": graph_transaction_payload,
            "graph_transaction_id": graph_transaction_id,
            "map_epoch": map_epoch,
            "origin_epoch": getattr(
                graph_transaction,
                "origin_epoch",
                getattr(
                    getattr(self, "last_graph_route_plan", None),
                    "map_epoch",
                    map_epoch,
                ),
            ) if graph_transaction is not None else map_epoch,
            "projection_epoch": getattr(
                self, "last_graph_snapshot_epoch", None
            ) or map_epoch,
            "graph_route_fsm_state": str(
                getattr(self, "graph_route_fsm_state", "SEARCHING")
            ),
            "graph_route_materialization": {
                "miss_count": int(
                    getattr(self, "graph_route_materialization_miss_count", 0)
                ),
                "obligation_key": (
                    list(getattr(
                        self,
                        "graph_route_materialization_obligation_key",
                        (),
                    ))
                    if getattr(
                        self,
                        "graph_route_materialization_obligation_key",
                        None,
                    ) is not None
                    else None
                ),
                "observed_epochs": list(
                    getattr(self, "graph_route_materialization_epochs", [])
                ),
                "first_epoch": getattr(
                    self, "graph_route_materialization_first_epoch", None
                ),
                "last_epoch": getattr(
                    self, "graph_route_materialization_last_epoch", None
                ),
                "last_observation": (
                    list(getattr(
                        self,
                        "graph_route_materialization_last_observation",
                        (),
                    ))
                    if getattr(
                        self,
                        "graph_route_materialization_last_observation",
                        None,
                    ) is not None
                    else None
                ),
                "invalidated": bool(
                    getattr(self, "graph_route_materialization_invalidated", False)
                ),
                "last_invalidation": getattr(
                    self,
                    "last_graph_route_materialization_invalidation",
                    None,
                ),
                "negative_evidence": getattr(
                    self,
                    "last_graph_route_materialization_negative_evidence",
                    None,
                ),
            },
            "graph_route_rehydration": {
                "projection_complete": bool(
                    getattr(
                        self,
                        "last_graph_snapshot_projection_complete",
                        False,
                    )
                ),
                "projection_epoch": getattr(
                    self, "last_graph_snapshot_epoch", None
                ),
                "candidate_count": int(
                    getattr(self, "last_graph_snapshot_candidate_count", 0)
                ),
                "visible_work_item_ids": list(
                    getattr(
                        self,
                        "last_graph_snapshot_visible_work_item_ids",
                        ()
                    )
                    or ()
                ),
                "visible_probe_ids": list(
                    getattr(
                        self,
                        "last_graph_snapshot_visible_probe_ids",
                        ()
                    )
                    or ()
                ),
                "visible_portal_ids": list(
                    getattr(
                        self,
                        "last_graph_snapshot_visible_portal_ids",
                        ()
                    )
                    or ()
                ),
            },
            "durable_portal_crossing_unavailable": getattr(
                self, "last_durable_portal_crossing_unavailable", None
            ),
            "durable_portal_egress_unavailable": getattr(
                self, "last_durable_portal_egress_unavailable", None
            ),
            # Planning is a slow evidence consumer.  Keep its ownership
            # boundary in the same replayable status stream as route events so
            # a stale proposal can be diagnosed without scraping ROS timing.
            "planning_contract": {
                "cycle_active": bool(
                    getattr(self, "planning_cycle_active", False)
                ),
                "cycle_skipped": int(
                    getattr(self, "planning_cycle_skipped", 0)
                ),
                "proposal_generation": int(
                    getattr(
                        getattr(self, "planning_proposal_gate", None),
                        "generation",
                        0,
                    )
                ),
                "preempt_requested": bool(
                    getattr(self, "planning_preempt_requested", False)
                ),
                "preempt_reason": str(
                    getattr(self, "planning_preempt_reason", "") or ""
                ),
            },
        }
        payload.update(fields)
        transition = build_transition_envelope(payload["event"], payload)
        if transition is not None:
            # Keep causal ownership next to the mutable status snapshot.  A
            # replay consumer can now distinguish a source-place plan from a
            # destination-place commit without guessing from event order.
            payload["transition"] = transition
        # The event graph is a slow-layer projection of this status stream.
        # Keep it in the same message so a run can be replayed from logs
        # without scraping ROS internals. Missing in narrow test fixtures is
        # intentionally tolerated.
        event_graph = getattr(self, "event_graph", None)
        if event_graph is not None:
            graph_event = event_graph.ingest(payload["event"], payload)
            payload["graph_event"] = {
                "sequence": int(graph_event.sequence),
                "structural": bool(graph_event.structural),
                "wake": bool(graph_event.wake),
                "reason": str(graph_event.reason),
            }
            payload["event_graph"] = event_graph.report()
        decision_scheduler = getattr(self, "decision_wake_scheduler", None)
        if decision_scheduler is not None:
            decision_scheduler.observe_status(payload["event"], payload)
            payload["decision_wake"] = decision_scheduler.report()
        try:
            self.status_publisher.publish(
                String(data=json.dumps(payload, sort_keys=True))
            )
        except (TypeError, ValueError):
            rospy.logwarn_throttle(
                5.0, "Global frontier status serialization failed"
            )

    def publish_route_command(
        self, frame_id, x, y, yaw, route_kind, mission_route_kind=None,
    ):
        """Publish one self-contained route command transaction."""
        graph_transaction = getattr(
            self, "graph_route_action_transaction", None
        )
        route_map_epoch = (
            getattr(graph_transaction, "map_epoch", None)
            if graph_transaction is not None
            else getattr(self, "last_map_epoch", None)
        )
        if route_map_epoch is None:
            # A route command is only published after snapshot construction.
            # Keep the wire contract non-empty even for compatibility fixtures
            # that omit the structural epoch object.
            route_map_epoch = max(
                1, int(getattr(self, "topology_component_epoch", 1) or 1)
            )
        graph_transaction_id = int(
            getattr(graph_transaction, "graph_transaction_id", 0) or 0
        )
        lifecycle_getter = getattr(self, "route_lifecycle_transaction_id", None)
        lifecycle_transaction_id = int(
            lifecycle_getter()
            if callable(lifecycle_getter)
            else getattr(self, "lifecycle_transaction_id", lambda: 0)()
        )
        route_transaction_id = int(
            getattr(graph_transaction, "transaction_id", 0) or 0
        )
        if route_transaction_id <= 0:
            # GoalManager allocates its mission transaction after this command
            # crosses the ROS boundary.  Until then, the graph transaction is
            # the only valid transaction identity for this handoff.
            route_transaction_id = graph_transaction_id
        action_generation = int(
            getattr(self, "active_action_generation", 0) or 0
        )
        origin_epoch = (
            getattr(graph_transaction, "origin_epoch", None)
            if graph_transaction is not None
            else route_map_epoch
        )
        if origin_epoch is None:
            origin_epoch = route_map_epoch
        projection_epoch = (
            getattr(graph_transaction, "projection_epoch", None)
            if graph_transaction is not None
            else getattr(self, "last_graph_snapshot_epoch", None) or route_map_epoch
        )
        if projection_epoch is None:
            projection_epoch = route_map_epoch
        goal_context = self.active_goal_context(
            route_kind, graph_transaction_id=graph_transaction_id
        )
        payload = {
            "event": "route_command",
            "frame_id": str(frame_id or "map").strip().lstrip("/") or "map",
            "goal": [round(float(x), 4), round(float(y), 4)],
            "yaw": None if yaw is None else round(float(yaw), 4),
            "route_id": int(self.active_route_id),
            # The controller action generation is allocated downstream.  Keep
            # the field explicit at zero here rather than borrowing the
            # predecessor route's generation and creating a false identity.
            "action_generation": action_generation,
            "transaction_id": route_transaction_id,
            "lifecycle_transaction_id": lifecycle_transaction_id,
            "route_kind": str(route_kind or "frontier_endpoint"),
            "mission_route_kind": str(
                mission_route_kind
                or getattr(self, "active_mission_route_kind", None)
                or route_kind
                or "frontier_endpoint"
            ),
            "graph_action": str(
                getattr(
                    getattr(self, "last_graph_route_plan", None),
                    "action",
                    "",
                ) or ""
            ),
            "graph_obligation_kind": str(
                getattr(
                    getattr(self, "last_graph_route_plan", None),
                    "obligation_kind",
                    "",
                ) or ""
            ),
            # This is the graph planner transaction, intentionally separate
            # from GoalManager's mission transaction_id allocated later.
            "graph_transaction_id": graph_transaction_id,
            "origin_epoch": origin_epoch,
            "projection_epoch": projection_epoch,
            "portal_probe_phase": str(
                getattr(self, "active_portal_probe_phase", "") or ""
            ),
            "transition_kind": str(self.active_transition_kind),
            "predecessor_route_id": int(self.active_predecessor_route_id),
            "predecessor_contract": (
                None
                if not isinstance(
                    getattr(self, "active_route_dispatch_contract", None), dict
                )
                else getattr(self, "active_route_dispatch_contract", {}).get(
                    "predecessor_contract"
                )
            ),
            "transition_distance_to_previous_endpoint": (
                None
                if self.active_transition_distance is None
                else round(float(self.active_transition_distance), 4)
            ),
            # Use the identity snapshot captured above. Reading the mutable
            # graph transaction again here can pair transaction N with the
            # nested context from transaction N-1 during a replan callback.
            "goal_context": goal_context,
            "portal_transaction": self.portal_transaction_report(),
            "map_epoch": int(route_map_epoch),
            "origin_epoch": origin_epoch,
            "projection_epoch": projection_epoch,
            "stamp": rospy.Time.now().to_sec(),
        }
        decision_scheduler = getattr(self, "decision_wake_scheduler", None)
        if decision_scheduler is not None:
            # ``route_command`` is the handoff from the slow graph to the
            # fast Navfn/TEB executor. Clear stale map wakes before recording
            # the command, so the command event and its scheduler state form
            # one atomic handoff in the lifecycle log.
            decision_scheduler.observe_status("route_command", payload)
            payload["decision_wake"] = decision_scheduler.report()
        self.command_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))
