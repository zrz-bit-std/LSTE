"""Frontier lifecycle observations for the TEB action bridge."""

import json
import math

import rospy


class TebGoalBridgeFrontierStatusMixin:
    def _clear_frontier_intent_after_release_locked(self):
        """Prevent the bridge timer from replaying a released frontier goal."""
        self.persistent_frontier_endpoint_pending = None
        self.latest_goal = None
        self.last_dispatched_goal = None
        self.last_dispatch_identity = None
        self.last_terminal_goal = None
        self.latest_intent_source = "waiting_global_slam_frontier"
        self.latest_intent_priority = 0
        self.latest_route_kind = ""
        self.latest_mission_route_kind = ""
        self.latest_graph_transaction_id = 0
        self.latest_origin_epoch = None
        self.latest_projection_epoch = None
        self.latest_route_id = 0
        self.latest_intent_goal = None

    def _persistent_frontier_endpoint_release_pending_locked(
        self, payload, route_id, action_generation
    ):
        """Return the semantic release record for a held stream endpoint."""
        pending = getattr(
            self, "persistent_frontier_endpoint_pending", None
        )
        if (
            not self.persistent_execution
            or not self.action_active
            or not isinstance(pending, dict)
        ):
            return None
        try:
            pending_route_id = int(pending.get("route_id", 0) or 0)
            pending_generation = int(
                pending.get("action_generation", 0) or 0
            )
        except (TypeError, ValueError):
            return None
        if pending_route_id != int(route_id) or pending_generation != int(
            action_generation
        ):
            return None
        reason = str(payload.get("release_reason", "") or "").strip()
        if not reason.startswith(("terminal_boundary", "endpoint_terminal")):
            return None
        return pending

    def _apply_controller_lease_release_locked(self, payload):
        """Apply an identity-checked terminal release from GlobalFrontier."""
        released = payload.get("released_controller_route")
        released = released if isinstance(released, dict) else {}
        terminal_received = bool(
            payload.get("terminal_received") or released.get("terminal_received")
        )
        controller_pending = bool(
            payload.get("controller_pending")
            or released.get("controller_pending")
        )
        # A lifecycle timeout or graph materialization invalidation can prove
        # that the semantic route must be abandoned before MoveBase emits a
        # terminal. Such a request is still safe only when it carries the
        # exact dispatch contract checked below.
        forced_controller_release = (
            str(payload.get("release_phase", "") or "").strip().lower()
            == "awaiting_transport_ack"
            or bool(payload.get("force_controller_lease_release", False))
        )
        if (not terminal_received and not forced_controller_release) or not controller_pending:
            return False
        try:
            route_id = max(
                0,
                int(
                    released.get(
                        "route_id",
                        payload.get(
                            "released_route_id", payload.get("route_id", 0)
                        ),
                    )
                    or 0
                ),
            )
        except (TypeError, ValueError):
            route_id = 0
        if route_id <= 0:
            return False

        def contract_int(*values):
            for value in values:
                try:
                    value = int(value or 0)
                except (TypeError, ValueError):
                    continue
                # A nested compatibility envelope may carry a zero sentinel
                # while the canonical scalar is present at the outer level.
                # Treat zero as missing so a valid fallback is not masked.
                if value > 0:
                    return value
            return 0

        # A delayed release tombstone can outlive the action that created it.
        # Select a contract by the route being released before validating any
        # other field; otherwise a predecessor's transaction can be compared
        # with a successor's route and produce a false identity rejection.
        def mapping(value):
            return value if hasattr(value, "get") else None

        contract_candidates = []
        pending_prepare = getattr(self, "pending_terminal_prepare", None)
        if isinstance(pending_prepare, dict):
            contract_candidates.append(
                (
                    "pending_terminal_prepare",
                    mapping(pending_prepare.get("old_contract")),
                )
            )
        pending_release = getattr(
            self, "pending_lease_release_contract", None
        )
        if isinstance(pending_release, dict):
            contract_candidates.append(
                (
                    "pending_lease_release_contract",
                    mapping(
                        pending_release.get("old_contract") or pending_release
                    ),
                )
            )
        watchdog = getattr(self, "route_lease_watchdog", None)
        if isinstance(watchdog, dict):
            # A watchdog can outlive the action-scoped contract after terminal
            # cleanup. It is the immutable predecessor contract and must stay
            # available for a delayed, identity-checked release request.
            contract_candidates.append(
                ("route_lease_watchdog", mapping(watchdog))
            )
        contract_candidates.append(
            ("active_action_contract", mapping(
                getattr(self, "active_action_contract", None)
            ))
        )
        contract_candidates.append(
            ("last_dispatch_identity", mapping(
                getattr(self, "last_dispatch_identity", None)
            ))
        )
        latest_frontier_contract = getattr(
            self, "_latest_frontier_release_contract_locked", None
        )
        if callable(latest_frontier_contract):
            contract_candidates.append(
                (
                    "latest_frontier_mission_contract",
                    mapping(latest_frontier_contract(route_id)),
                )
            )

        expected_contract = {}
        expected_contract_source = ""
        candidate_route_ids = []
        for source, candidate in contract_candidates:
            if candidate is None:
                continue
            candidate_route_id = contract_int(candidate.get("route_id"))
            if candidate_route_id > 0:
                candidate_route_ids.append(
                    {"source": source, "route_id": candidate_route_id}
                )
            if candidate_route_id == route_id:
                expected_contract = dict(candidate)
                expected_contract_source = source
                break
        active_route_id = int(getattr(self, "active_route_id", 0) or 0)
        latest_route_id = int(getattr(self, "latest_route_id", 0) or 0)
        expected_route_id = contract_int(
            expected_contract.get("route_id")
        ) if expected_contract else 0
        pending_release_contract = getattr(
            self, "pending_lease_release_contract", None
        )
        pending_release = (
            pending_release_contract.get("old_contract")
            if isinstance(pending_release_contract, dict)
            else None
        )
        pending_release_route_id = contract_int(
            None if not isinstance(pending_release, dict)
            else pending_release.get("route_id")
        )
        pending_release_matches = bool(
            pending_release_route_id == route_id
            and expected_contract_source == "pending_lease_release_contract"
        )

        # A Portal crossing can publish the semantic release for its next
        # route before that route's mission contract reaches this subscriber.
        # Handle this ordering case before the active/latest route guards:
        # those guards intentionally reject an unknown route, but this one
        # future route is precisely the bounded contract we must retain and
        # replay after adoption.
        if not expected_contract:
            release_phase = str(
                payload.get("release_phase", "") or ""
            ).strip().lower()
            frontier_source = (
                str(
                    getattr(self, "active_intent_source", "") or ""
                ).strip().lower()
                == "global_slam_frontier"
                or str(
                    getattr(self, "latest_intent_source", "") or ""
                ).strip().lower()
                == "global_slam_frontier"
            )
            route_floor = max(active_route_id, latest_route_id)
            if (
                self.persistent_execution
                and release_phase == "awaiting_transport_ack"
                and frontier_source
                and route_id > route_floor
            ):
                deferred = getattr(
                    self, "deferred_frontier_lease_release", None
                )
                deferred_route_id = 0
                if isinstance(deferred, dict):
                    try:
                        deferred_route_id = int(
                            deferred.get("route_id", 0) or 0
                        )
                    except (TypeError, ValueError):
                        deferred_route_id = 0
                if deferred_route_id and route_id <= deferred_route_id:
                    self.publish_bridge_status(
                        "controller_lease_release_duplicate_deferred",
                        route_id=route_id,
                        deferred_route_id=deferred_route_id,
                        reason="newer_contract_not_yet_adopted",
                    )
                    return False
                self.deferred_frontier_lease_release = dict(payload)
                self.publish_bridge_status(
                    "controller_lease_release_deferred",
                    route_id=route_id,
                    reason="mission_contract_not_yet_adopted",
                    release_phase=release_phase,
                    active_route_id=active_route_id,
                    latest_route_id=latest_route_id,
                    graph_transaction_id=contract_int(
                        payload.get("graph_transaction_id")
                    ),
                    lifecycle_transaction_id=contract_int(
                        payload.get("lifecycle_transaction_id")
                    ),
                )
                return False
        if active_route_id > route_id or (
            latest_route_id > route_id and not pending_release_matches
        ):
            return False
        if active_route_id != route_id and expected_route_id != route_id:
            # The action-health clear path intentionally removes active route
            # state after publishing a terminal. The pending tombstone is the
            # only valid identity in that short release window; a different
            # active/latest route must never be cancelled by this ACK.
            return False
        if not expected_contract:
            self.publish_bridge_status(
                "controller_lease_release_rejected",
                reason="release_contract_not_found_for_route",
                route_id=route_id,
                active_route_id=active_route_id,
                latest_route_id=latest_route_id,
                candidate_route_ids=candidate_route_ids,
            )
            return False
        expected_source = str(
            expected_contract.get(
                "source", getattr(self, "active_intent_source", "unknown")
            )
            or "unknown"
        ).strip().lower()
        active_source = str(
            getattr(self, "active_intent_source", "unknown") or "unknown"
        ).strip().lower()
        if (
            expected_source != "global_slam_frontier"
            and active_source != "global_slam_frontier"
        ):
            return False
        release = getattr(
            self, "_release_frontier_controller_lease_locked", None
        )
        if not callable(release):
            return False
        incoming_lifecycle = contract_int(
            released.get("lifecycle_transaction_id"),
            payload.get("lifecycle_transaction_id"),
        )
        incoming_transaction = contract_int(
            released.get("transaction_id"), payload.get("transaction_id")
        )
        incoming_generation = contract_int(
            released.get("action_generation"),
            payload.get("action_generation"),
        )
        incoming_graph = contract_int(
            released.get("graph_transaction_id"),
            payload.get("graph_transaction_id"),
        )
        incoming_origin = contract_int(
            released.get("origin_epoch"), payload.get("origin_epoch")
        )
        incoming_projection = contract_int(
            released.get("projection_epoch"), payload.get("projection_epoch")
        )
        incoming_map = released.get("map_epoch")
        if incoming_map is None:
            incoming_map = payload.get("map_epoch")
        expected_generation = contract_int(
            expected_contract.get("generation"),
            expected_contract.get("action_generation"),
        )
        expected_transaction = contract_int(
            expected_contract.get("transaction_id")
        )
        expected_lifecycle = contract_int(
            expected_contract.get("lifecycle_transaction_id")
        )
        expected_graph = contract_int(expected_contract.get("graph_transaction_id"))
        expected_origin = contract_int(expected_contract.get("origin_epoch"))
        expected_projection = contract_int(
            expected_contract.get("projection_epoch")
        )
        expected_map = expected_contract.get("map_epoch")
        hard_identity_mismatch = any(
            expected_value > 0
            and incoming_value > 0
            and incoming_value != expected_value
            for expected_value, incoming_value in (
                (expected_generation, incoming_generation),
                (expected_lifecycle, incoming_lifecycle),
                (expected_graph, incoming_graph),
            )
        )
        rebound_fields = []
        if not hard_identity_mismatch:
            if incoming_transaction <= 0 and expected_transaction > 0:
                incoming_transaction = expected_transaction
                rebound_fields.append("transaction_id")
            if incoming_generation <= 0 and expected_generation > 0:
                incoming_generation = expected_generation
                rebound_fields.append("action_generation")
            if incoming_lifecycle <= 0 and expected_lifecycle > 0:
                incoming_lifecycle = expected_lifecycle
                rebound_fields.append("lifecycle_transaction_id")
            if incoming_graph <= 0 and expected_graph > 0:
                incoming_graph = expected_graph
                rebound_fields.append("graph_transaction_id")
            if incoming_map in (None, 0) and expected_map is not None:
                incoming_map = expected_map
                rebound_fields.append("map_epoch")
            if incoming_origin <= 0 and expected_origin > 0:
                incoming_origin = expected_origin
                rebound_fields.append("origin_epoch")
            if incoming_projection <= 0 and expected_projection > 0:
                incoming_projection = expected_projection
                rebound_fields.append("projection_epoch")
        if rebound_fields:
            self.publish_bridge_status(
                "controller_lease_release_identity_rebound",
                route_id=route_id,
                rebound_fields=rebound_fields,
                expected_contract_source=expected_contract_source,
                expected_transaction_id=expected_transaction,
                action_generation=incoming_generation,
                lifecycle_transaction_id=incoming_lifecycle,
                graph_transaction_id=incoming_graph,
                map_epoch=incoming_map,
                origin_epoch=incoming_origin,
                projection_epoch=incoming_projection,
                reason="missing_predecessor_identity_fields",
            )
        if expected_transaction > 0 and incoming_transaction != expected_transaction:
            self.publish_bridge_status(
                "controller_lease_release_rejected",
                reason="transaction_id_mismatch",
                route_id=route_id,
                expected_contract_source=expected_contract_source,
                expected_contract_route_id=expected_route_id,
                expected_transaction_id=expected_transaction,
                received_transaction_id=incoming_transaction,
            )
            return False
        if expected_generation > 0 and incoming_generation != expected_generation:
            self.publish_bridge_status(
                "controller_lease_release_rejected",
                reason="action_generation_mismatch",
                route_id=route_id,
                expected_action_generation=expected_generation,
                received_action_generation=incoming_generation,
            )
            return False
        if expected_lifecycle > 0 and incoming_lifecycle != expected_lifecycle:
            self.publish_bridge_status(
                "controller_lease_release_rejected",
                reason="lifecycle_transaction_id_mismatch",
                route_id=route_id,
                expected_lifecycle_transaction_id=expected_lifecycle,
                received_lifecycle_transaction_id=incoming_lifecycle,
            )
            return False
        if expected_graph > 0 and incoming_graph != expected_graph:
            self.publish_bridge_status(
                "controller_lease_release_rejected",
                reason="graph_transaction_id_mismatch",
                route_id=route_id,
                expected_graph_transaction_id=expected_graph,
                received_graph_transaction_id=incoming_graph,
            )
            return False
        if expected_origin > 0 and incoming_origin != expected_origin:
            self.publish_bridge_status(
                "controller_lease_release_rejected",
                reason="origin_epoch_mismatch",
                route_id=route_id,
                expected_origin_epoch=expected_origin,
                received_origin_epoch=incoming_origin,
            )
            return False
        if expected_projection > 0 and incoming_projection != expected_projection:
            self.publish_bridge_status(
                "controller_lease_release_rejected",
                reason="projection_epoch_mismatch",
                route_id=route_id,
                expected_projection_epoch=expected_projection,
                received_projection_epoch=incoming_projection,
            )
            return False
        if expected_map is not None:
            try:
                if int(incoming_map) != int(expected_map):
                    raise ValueError
            except (TypeError, ValueError):
                self.publish_bridge_status(
                    "controller_lease_release_rejected",
                    reason="map_epoch_mismatch",
                    route_id=route_id,
                    expected_map_epoch=expected_map,
                    received_map_epoch=incoming_map,
                )
                return False
        semantic_pending = (
            self._persistent_frontier_endpoint_release_pending_locked(
                payload,
                route_id,
                incoming_generation or expected_generation,
            )
        )
        if semantic_pending is not None:
            if bool(semantic_pending.get("ack_sent")):
                self.publish_bridge_status(
                    "controller_lease_release_duplicate_ignored",
                    route_id=route_id,
                    action_generation=incoming_generation,
                    reason="persistent_stream_semantic_ack_already_sent",
                    semantic_only=True,
                )
                return False
            semantic_pending["ack_sent"] = True
            released_controller_route = {
                "route_id": route_id,
                "route_kind": str(
                    released.get("route_kind")
                    or payload.get("released_route_kind")
                    or "frontier_endpoint"
                ),
                "transaction_id": incoming_transaction,
                "lifecycle_transaction_id": incoming_lifecycle,
                "action_generation": incoming_generation,
                "graph_transaction_id": incoming_graph,
                "map_epoch": incoming_map,
                "origin_epoch": incoming_origin,
                "projection_epoch": incoming_projection,
                "terminal_received": True,
                "controller_pending": True,
            }
            semantic_fields = {
                "released_route_id": route_id,
                "released_route_kind": released_controller_route[
                    "route_kind"
                ],
                "route_id": route_id,
                "transaction_id": incoming_transaction,
                "lifecycle_transaction_id": incoming_lifecycle,
                "action_generation": incoming_generation,
                "new_action_generation": int(
                    getattr(self, "action_generation", 0) or 0
                ),
                "graph_transaction_id": incoming_graph,
                "map_epoch": incoming_map,
                "origin_epoch": incoming_origin,
                "projection_epoch": incoming_projection,
                "controller_lease": "released",
                "transport_lease": "held",
                "semantic_only": True,
                "release_phase": "persistent_stream_semantic_boundary",
                "released_controller_route": released_controller_route,
            }
            self.publish_bridge_status(
                "controller_lease_release_applied", **semantic_fields
            )
            # This ACK releases GlobalFrontier's semantic lease. It must not
            # call _release_frontier_controller_lease_locked: the transport
            # action is intentionally still alive and will accept the next
            # mission command through StreamingNavfn.
            self.publish_bridge_status("lease_release_ack", **semantic_fields)
            return True
        release_key = self._action_identity_key(
            route_id, incoming_generation or expected_generation
        )
        if self._released_action_key_seen_locked(release_key):
            self.publish_bridge_status(
                "controller_lease_release_duplicate_ignored",
                route_id=route_id,
                transaction_id=incoming_transaction,
                lifecycle_transaction_id=incoming_lifecycle,
                action_generation=incoming_generation,
                graph_transaction_id=incoming_graph,
                map_epoch=incoming_map,
                origin_epoch=incoming_origin,
                projection_epoch=incoming_projection,
                reason="release_key_already_applied",
                dedupe_key=[int(release_key[0]), int(release_key[1])]
                if release_key is not None else None,
            )
            return False
        released_route_kind = str(
            released.get("route_kind")
            or payload.get("released_route_kind")
            or getattr(self, "active_route_kind", "")
            or ""
        )
        release_reason = str(
            payload.get("release_reason", "controller_lease_released")
            or "controller_lease_released"
        )
        if (
            release_reason != "terminal_boundary"
            and not release_reason.startswith(
                ("terminal_boundary_", "endpoint_terminal_")
            )
        ):
            release_reason = "terminal_boundary_%s" % release_reason
        if not release(
            route_id,
            release_reason,
            release_contract=expected_contract,
        ):
            return False
        # The exact upstream release has now been applied. Keep released
        # action tombstones for duplicate rejection, but no longer use this
        # predecessor as the expected contract for a later unrelated message.
        self.pending_lease_release_contract = None
        self._clear_frontier_intent_after_release_locked()
        self.publish_bridge_status(
            "controller_lease_release_applied",
            released_route_id=route_id,
            released_route_kind=released_route_kind,
            terminal_received=True,
            controller_lease="released",
            next_owner="global_slam_frontier",
            route_id=route_id,
            transaction_id=incoming_transaction,
            lifecycle_transaction_id=incoming_lifecycle,
            action_generation=incoming_generation,
            new_action_generation=int(
                getattr(self, "action_generation", 0) or 0
            ),
            graph_transaction_id=incoming_graph,
            map_epoch=incoming_map,
            origin_epoch=incoming_origin,
            projection_epoch=incoming_projection,
            release_reason=release_reason,
        )
        self.publish_bridge_status(
            "lease_release_ack",
            released_route_id=route_id,
            released_route_kind=released_route_kind,
            route_id=route_id,
            transaction_id=incoming_transaction,
            lifecycle_transaction_id=incoming_lifecycle,
            action_generation=incoming_generation,
            new_action_generation=int(
                getattr(self, "action_generation", 0) or 0
            ),
            graph_transaction_id=incoming_graph,
            map_epoch=incoming_map,
            origin_epoch=incoming_origin,
            projection_epoch=incoming_projection,
            controller_lease="released",
            released_controller_route={
                "route_id": route_id,
                "route_kind": released_route_kind,
                "transaction_id": incoming_transaction,
                "lifecycle_transaction_id": incoming_lifecycle,
                "action_generation": incoming_generation,
                "graph_transaction_id": incoming_graph,
                "map_epoch": incoming_map,
                "origin_epoch": incoming_origin,
                "projection_epoch": incoming_projection,
            },
        )
        # Keep the exact predecessor ACK local to the bridge.  The next
        # dispatch may consume it only for a newer frontier route; a target or
        # same-route replay must never be labeled as a successor.
        self.pending_successor_release_ack = {
            "route_id": route_id,
            "route_kind": released_route_kind,
            "transaction_id": incoming_transaction,
            "lifecycle_transaction_id": incoming_lifecycle,
            "action_generation": incoming_generation,
            "new_action_generation": int(
                getattr(self, "action_generation", 0) or 0
            ),
            "graph_transaction_id": incoming_graph,
            "map_epoch": incoming_map,
            "origin_epoch": incoming_origin,
            "projection_epoch": incoming_projection,
        }
        return True

    def _retry_deferred_frontier_lease_release_locked(self):
        """Replay a release that arrived before its mission contract."""
        payload = getattr(self, "deferred_frontier_lease_release", None)
        if not isinstance(payload, dict):
            return False
        try:
            route_id = int(payload.get("route_id", 0) or 0)
        except (TypeError, ValueError):
            route_id = 0
        contract = getattr(self, "active_action_contract", None)
        try:
            contract_route_id = int((contract or {}).get("route_id", 0) or 0)
        except (AttributeError, TypeError, ValueError):
            contract_route_id = 0
        if contract_route_id != route_id:
            contract = self._latest_frontier_release_contract_locked(route_id)
            contract_route_id = int((contract or {}).get("route_id", 0) or 0)
        if route_id <= 0 or contract_route_id != route_id:
            return False
        self.deferred_frontier_lease_release = None
        if contract is not getattr(self, "active_action_contract", None):
            # The persistent stream keeps one action handle while the semantic
            # route changes. Let release validation use the newly adopted
            # mission contract rather than the predecessor action contract.
            payload = dict(payload)
            for key, value in contract.items():
                if (
                    value not in (None, "", 0)
                    and payload.get(key) in (None, "", 0)
                ):
                    payload[key] = value
        self.publish_bridge_status(
            "controller_lease_release_replayed",
            route_id=route_id,
            reason="mission_contract_adopted",
        )
        return bool(self._apply_controller_lease_release_locked(payload))

    def _latest_frontier_release_contract_locked(self, route_id):
        """Build the latest semantic route contract for deferred release."""
        try:
            route_id = int(route_id or 0)
        except (TypeError, ValueError):
            return None
        if route_id <= 0 or int(getattr(self, "latest_route_id", 0) or 0) != route_id:
            return None
        source = str(
            getattr(self, "latest_intent_source", "") or ""
        ).strip().lower()
        if source != "global_slam_frontier":
            return None
        action_contract = getattr(self, "active_action_contract", None)
        action_generation = int(getattr(self, "action_generation", 0) or 0)
        if isinstance(action_contract, dict):
            action_generation = int(
                action_contract.get(
                    "generation", action_contract.get("action_generation", 0)
                )
                or action_generation
            )
        return {
            "route_id": route_id,
            "transaction_id": int(
                getattr(self, "latest_goal_transaction_id", 0) or 0
            ),
            "lifecycle_transaction_id": int(
                getattr(self, "latest_lifecycle_transaction_id", 0) or 0
            ),
            "action_generation": action_generation,
            "generation": action_generation,
            "graph_transaction_id": int(
                getattr(self, "latest_graph_transaction_id", 0) or 0
            ),
            "map_epoch": getattr(self, "latest_route_map_epoch", None),
            "origin_epoch": getattr(self, "latest_origin_epoch", None),
            "projection_epoch": getattr(self, "latest_projection_epoch", None),
            "route_kind": str(getattr(self, "latest_route_kind", "") or ""),
            "mission_route_kind": str(
                getattr(self, "latest_mission_route_kind", "") or ""
            ),
            "source": source,
            "priority": int(getattr(self, "latest_intent_priority", 0) or 0),
        }

    def _remember_frontier_map_epoch_locked(self, payload):
        """Bind the newest frontier route to its graph snapshot epoch."""
        raw_route_id = payload.get("route_id")
        try:
            route_id = max(0, int(raw_route_id or 0))
        except (TypeError, ValueError):
            route_id = 0
        if route_id <= 0:
            return
        raw_epoch = payload.get("map_epoch")
        try:
            epoch = None if raw_epoch is None else int(raw_epoch)
        except (TypeError, ValueError):
            epoch = None
        previous_route_id = int(
            getattr(self, "latest_frontier_map_route_id", 0) or 0
        )
        try:
            graph_transaction_id = max(
                0, int(payload.get("graph_transaction_id", 0) or 0)
            )
        except (TypeError, ValueError):
            graph_transaction_id = 0
        if route_id > previous_route_id:
            self.latest_frontier_map_route_id = route_id
            self.latest_frontier_map_epoch = epoch
            self.latest_frontier_graph_transaction_id = graph_transaction_id
        elif route_id == previous_route_id and epoch is not None:
            self.latest_frontier_map_epoch = epoch
            if graph_transaction_id > 0:
                self.latest_frontier_graph_transaction_id = graph_transaction_id

        # In atomic mission-command mode, status is an observation stream, not
        # an ownership boundary.  Its projection can advance while a target
        # or frontier route is still executing.  Promoting that newer snapshot
        # into ``latest_*`` relabels the live route without changing the
        # planner command already installed for it, so Supervisor correctly
        # rejects every subsequent command as an identity mismatch.  The
        # route-command callback is the sole writer of the canonical execution
        # epochs; keep the status-only map observation above for diagnostics and
        # legacy fallback logic.
        if getattr(self, "use_goal_command", True):
            return

        if epoch is not None and (
            not bool(getattr(self, "intent_seen", False))
            or str(getattr(self, "latest_intent_source", "") or "").strip().lower()
            == "global_slam_frontier"
            or route_id >= int(getattr(self, "latest_route_id", 0) or 0)
        ):
            self.latest_route_map_epoch = epoch
            if graph_transaction_id > 0:
                self.latest_graph_transaction_id = graph_transaction_id
        for attribute, key in (
            ("latest_origin_epoch", "origin_epoch"),
            ("latest_projection_epoch", "projection_epoch"),
        ):
            raw_value = payload.get(key)
            try:
                value = None if raw_value is None else int(raw_value)
            except (TypeError, ValueError):
                value = None
            if value is not None and value >= 0:
                setattr(self, attribute, value)

    def _handle_frontier_route_unavailable_locked(self, payload):
        """Release an idle route, or defer while its controller still runs.

        ``frontier_route_unavailable`` describes a planning snapshot, not a
        physical failure of the currently executing route.  A durable graph
        obligation may be temporarily absent from that snapshot while the
        current endpoint or Portal transition is still making progress.  The
        controller lease therefore stays with that route until its terminal
        callback.  An explicit graph materialization invalidation is different:
        it is the cancellation boundary because the graph has exhausted the
        durable obligation and must hand control to a new route owner.
        """
        try:
            successor_route_id = max(
                0, int(payload.get("successor_route_id", 0) or 0)
            )
        except (TypeError, ValueError):
            successor_route_id = 0
        if successor_route_id > 0:
            return
        reason = str(payload.get("reason", "graph_route_unavailable"))
        force_release = bool(
            payload.get("force_controller_lease_release", False)
        )
        try:
            route_id = max(0, int(payload.get("route_id", 0) or 0))
        except (TypeError, ValueError):
            route_id = 0
        if route_id <= 0:
            route_id = max(
                int(getattr(self, "active_route_id", 0) or 0),
                int(getattr(self, "latest_route_id", 0) or 0),
            )
        if payload.get("controller_release_allowed") is False:
            # This event belongs to a successor graph obligation.  Its
            # materialization failure is not permission to cancel the active
            # persistent action, even if the companion status arrives after a
            # transport callback has already changed the bridge's mutable
            # route fields.
            self.publish_bridge_status(
                "frontier_route_unavailable_deferred",
                route_id=route_id,
                active_route_id=int(getattr(self, "active_route_id", 0) or 0),
                action_active=bool(getattr(self, "action_active", False)),
                controller_lease="held_until_terminal",
                active_route_retained=True,
                reason=reason,
                source_status=payload,
            )
            return
        if (
            self.latest_intent_priority >= 2
            or self.active_intent_priority >= 2
            or (
                self.persistent_execution
                and self.persistent_target_pending_transaction > 0
            )
        ):
            self.publish_bridge_status(
                "frontier_route_unavailable_ignored",
                reason="higher_priority_target_owns_controller",
                route_id=route_id,
                source_status=payload,
            )
            return
        frontier_owned = (
            self.active_intent_source == "global_slam_frontier"
            or self.latest_intent_source == "global_slam_frontier"
        )
        if not frontier_owned or not self._is_active_mode() or route_id <= 0:
            return
        active_route_id = int(getattr(self, "active_route_id", 0) or 0)
        released_route = payload.get("released_controller_route")
        if not isinstance(released_route, dict):
            released_route = {}
        # Graph materialization invalidation carries the immutable controller
        # predecessor that caused the graph failure.  Do not reconstruct that
        # contract from the bridge's mutable active state: a persistent stream
        # can already be observing a newer semantic transaction when the
        # invalidation arrives.  Passing the explicit contract also makes a
        # duplicate forced release emit an ACK for the same predecessor key.
        predecessor_contract = payload.get("predecessor_contract")
        if not isinstance(predecessor_contract, dict):
            predecessor_contract = released_route
        else:
            predecessor_contract = dict(predecessor_contract)
        explicit_release_contract = None
        try:
            explicit_release_route_id = int(
                predecessor_contract.get("route_id", 0) or 0
            )
        except (TypeError, ValueError):
            explicit_release_route_id = 0
        if explicit_release_route_id > 0:
            explicit_release_contract = predecessor_contract
        release_route_id = (
            explicit_release_route_id if explicit_release_route_id > 0 else route_id
        )
        try:
            released_route_id = int(released_route.get("route_id", 0) or 0)
        except (TypeError, ValueError):
            released_route_id = 0
        # GlobalFrontier publishes the endpoint terminal and then starts a
        # new planning transaction.  During that boundary the persistent
        # MoveBase action is intentionally still alive, but it no longer owns
        # the old endpoint.  Treating this as an ordinary active route keeps
        # ``terminal_hold_active`` set forever and prevents the next route from
        # being admitted.  The explicit route identity and terminal bit are
        # the only evidence that authorizes this release; a plain unavailable
        # event while a route is moving must remain deferred.
        terminal_boundary = bool(
            released_route.get("terminal_received")
            and released_route.get("controller_pending")
            and released_route_id == route_id
        )
        if (
            bool(getattr(self, "action_active", False))
            and active_route_id == route_id
            and str(
                getattr(self, "active_intent_source", "unknown") or "unknown"
            ).strip().lower() == "global_slam_frontier"
            and int(getattr(self, "active_intent_priority", 0) or 0) == 0
            and not terminal_boundary
            and not force_release
        ):
            # Planning and control have separate leases. Keep the physical
            # route alive and let its terminal event trigger the next graph
            # decision; cancelling here would turn ordinary map churn into a
            # PREEMPTED action and an avoidable stop/restart cycle.
            self.publish_bridge_status(
                "frontier_route_unavailable_deferred",
                route_id=route_id,
                active_route_id=active_route_id,
                active_route_kind=str(
                    getattr(self, "active_route_kind", "") or ""
                ),
                action_active=True,
                controller_lease="held_until_terminal",
                reason=reason,
                source_status=payload,
            )
            return
        stale_goal = self.last_dispatched_goal
        release = getattr(
            self, "_release_frontier_controller_lease_locked", None
        )
        release_reason = reason
        if terminal_boundary and release_reason == "terminal_boundary":
            release_reason = "terminal_boundary"
        elif terminal_boundary and not release_reason.startswith(
            ("endpoint_terminal_", "terminal_boundary_")
        ):
            release_reason = "endpoint_terminal_%s" % release_reason
        if release is None or not release(
            release_route_id,
            release_reason,
            publish_release_ack=force_release,
            release_contract=explicit_release_contract,
        ):
            return
        self.prefetched_frontier_goal = None
        self.prefetched_frontier_route_id = 0
        self.prefetched_frontier_entry_yaw = None
        self.prefetched_frontier_entry_yaw_basis = ""
        self.latest_goal = None
        self.last_dispatched_goal = None
        self.last_dispatch_identity = None
        self.last_terminal_goal = None
        self.latest_intent_source = "waiting_global_slam_frontier"
        self.latest_intent_priority = 0
        self.latest_route_kind = ""
        self.latest_mission_route_kind = ""
        self.latest_graph_transaction_id = 0
        self.latest_route_id = 0
        self.latest_intent_goal = None
        self.publish_bridge_status(
            "frontier_route_unavailable",
            route_id=release_route_id,
            unavailable_route_id=route_id,
            released_route_id=release_route_id,
            successor_route_id=0,
            controller_lease="released",
            reason=release_reason,
            forced_materialization_release=force_release,
            predecessor_contract=(
                None
                if explicit_release_contract is None
                else dict(explicit_release_contract)
            ),
            stale_goal=(
                None
                if stale_goal is None
                else [
                    round(float(stale_goal.pose.position.x), 3),
                    round(float(stale_goal.pose.position.y), 3),
                ]
            ),
            source_status=payload,
        )

    def on_frontier_status(self, message):
        """Consume frontier lifecycle state without treating it as a goal."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        event = str(payload.get("event", "")).strip()
        with self.lock:
            self._remember_frontier_map_epoch_locked(payload)
            observe_watchdog = getattr(
                self, "_observe_route_lease_watchdog_status_locked", None
            )
            if callable(observe_watchdog):
                observe_watchdog(payload)
            if event == "controller_lease_released":
                self._apply_controller_lease_release_locked(payload)
                return
            if event == "frontier_route_unavailable":
                self._handle_frontier_route_unavailable_locked(payload)
                return
            if event == "graph_route_materialization_invalidated":
                # This is an explicit graph ownership boundary, unlike a
                # transient projection miss. Route it through the same
                # identity-checked release path even if the companion
                # frontier_route_unavailable event is delayed or dropped.
                invalidation = dict(payload)
                invalidation["force_controller_lease_release"] = bool(
                    payload.get("controller_release_allowed", True)
                )
                invalidation.setdefault(
                    "reason", "graph_route_materialization_expired"
                )
                self._handle_frontier_route_unavailable_locked(invalidation)
                return
            if event == "place_graph_waiting_for_portal_frontier":
                self.frontier_portal_wait = True
                self.frontier_portal_wait_route_id = max(
                    self.frontier_portal_wait_route_id,
                    int(payload.get("route_id", 0) or 0),
                )
                self.prefetched_frontier_goal = None
                self.prefetched_frontier_route_id = 0
                self.prefetched_frontier_entry_yaw = None
                self.prefetched_frontier_entry_yaw_basis = ""
                self.publish_bridge_status(
                    "frontier_portal_wait",
                    waiting_route_id=int(self.frontier_portal_wait_route_id),
                    hop_limit=payload.get("hop_limit"),
                    skipped_candidates=payload.get("skipped_candidates"),
                )
                return
            if event == "frontier_prefetched":
                raw_goal = payload.get("pending_goal")
                try:
                    pending_route_id = max(
                        0, int(payload.get("pending_route_id", 0) or 0)
                    )
                    active_route_id = max(
                        0, int(payload.get("route_id", 0) or 0)
                    )
                    x, y = float(raw_goal[0]), float(raw_goal[1])
                except (TypeError, ValueError, IndexError):
                    return
                if (
                    pending_route_id == active_route_id + 1
                    and math.isfinite(x)
                    and math.isfinite(y)
                ):
                    self.prefetched_frontier_goal = (x, y)
                    self.prefetched_frontier_route_id = pending_route_id
                    raw_entry_yaw = payload.get("pending_entry_yaw")
                    try:
                        entry_yaw = float(raw_entry_yaw)
                        self.prefetched_frontier_entry_yaw = (
                            entry_yaw if math.isfinite(entry_yaw) else None
                        )
                    except (TypeError, ValueError):
                        self.prefetched_frontier_entry_yaw = None
                    self.prefetched_frontier_entry_yaw_basis = str(
                        payload.get("pending_entry_yaw_basis", "")
                    ).strip()
                    self.publish_bridge_status(
                        "frontier_prefetch_available",
                        active_route_id=active_route_id,
                        pending_route_id=pending_route_id,
                        pending_goal=[round(x, 3), round(y, 3)],
                        pending_entry_yaw=(
                            None
                            if self.prefetched_frontier_entry_yaw is None
                            else round(self.prefetched_frontier_entry_yaw, 4)
                        ),
                        pending_entry_yaw_basis=(
                            self.prefetched_frontier_entry_yaw_basis or None
                        ),
                    )
                return
            # A promotion, invalidation, or fresh non-pending route makes the
            # old endpoint's cached successor ineligible.
            if event in (
                "terminal_prefetch_promoted",
                "route_invalidated",
                "frontier_exhausted",
                "terminal_replan_requested",
                "replan_acknowledged",
            ):
                self.prefetched_frontier_goal = None
                self.prefetched_frontier_route_id = 0
                self.prefetched_frontier_entry_yaw = None
                self.prefetched_frontier_entry_yaw_basis = ""
            if event not in ("route_invalidated", "frontier_exhausted"):
                return
            reason = str(payload.get("reason", "unknown"))
            # Do not cancel the persistent action lease while a visual target
            # waits for validation or already owns the mission.
            if self.persistent_execution and (
                self.latest_intent_priority >= 2
                or self.active_intent_priority >= 2
                or self.persistent_target_pending_transaction > 0
            ):
                self.publish_bridge_status(
                    "frontier_invalidation_ignored",
                    reason=reason,
                    latest_transaction_id=int(self.latest_goal_transaction_id),
                    latest_priority=int(self.latest_intent_priority),
                    active_priority=int(self.active_intent_priority),
                    pending_target_transaction=int(
                        self.persistent_target_pending_transaction
                    ),
                )
                return
            if (
                event == "route_invalidated"
                and self._defer_persistent_frontier_invalidation_locked(payload)
            ):
                return
            frontier_owned = (
                self.active_intent_source == "global_slam_frontier"
                or self.latest_intent_source == "global_slam_frontier"
            )
            if not frontier_owned or not self._is_active_mode():
                return
            stale_goal = self.last_dispatched_goal
            self.cancel_locked("frontier_%s_%s" % (event, reason))
            # Wait for a validated replacement route; never redispatch this
            # stale source pose from the bridge timer.
            self.latest_goal = None
            self.last_dispatched_goal = None
            self.last_dispatch_identity = None
            self.last_terminal_goal = None
            self.latest_intent_source = "waiting_global_slam_frontier"
            self.latest_intent_priority = 0
            self.latest_route_kind = ""
            self.latest_graph_transaction_id = 0
            self.latest_route_id = 0
            self.latest_intent_goal = None
            self.publish_bridge_status(
                "frontier_%s" % event,
                reason=reason,
                stale_goal=(
                    None
                    if stale_goal is None
                    else [
                        round(float(stale_goal.pose.position.x), 3),
                        round(float(stale_goal.pose.position.y), 3),
                    ]
                ),
                source_status=payload,
            )
            rospy.logwarn(
                "TEB goal bridge released stale frontier action: reason=%s",
                reason,
            )

    def _defer_persistent_frontier_invalidation_locked(self, payload):
        """Keep the transport lease across a normal semantic endpoint release."""
        pending = getattr(
            self, "persistent_frontier_endpoint_pending", None
        )
        if (
            not self.persistent_execution
            or not self.action_active
            or not isinstance(pending, dict)
            or str(payload.get("reason", "") or "").strip()
            != "frontier_observed_at_standoff"
        ):
            return False
        try:
            route_id = int(payload.get("route_id", 0) or 0)
            pending_route_id = int(pending.get("route_id", 0) or 0)
        except (TypeError, ValueError):
            return False
        if route_id <= 0 or route_id != pending_route_id:
            return False
        self.publish_bridge_status(
            "frontier_invalidation_deferred",
            route_id=route_id,
            reason="semantic_endpoint_terminal_transport_lease_held",
            controller_lease="held_until_successor",
            semantic_only=True,
            source_status=payload,
        )
        return True
