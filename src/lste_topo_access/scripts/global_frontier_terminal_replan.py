"""Successor scheduling after a verified route terminal.

The live node gathers the terminal into ``LifecycleManager`` and consumes it
from the fixed tick.  The lock/timer path remains only for older compositions
and focused compatibility fixtures.
"""

import rospy

from lifecycle_manager import EventType


class GlobalFrontierTerminalReplanMixin:
    """Release completed work and request one fresh planning snapshot."""

    _FAILURE_STATUS_CODES = frozenset((4, 5, 8, 9))
    _MAX_PENDING_CONTROLLER_RELEASES = 64

    def _pending_controller_release_table(self):
        """Return the route-keyed predecessor release contracts.

        ``pending_controller_lease_release`` is retained as a compatibility
        alias for older reports and compositions, but it cannot represent two
        asynchronous predecessors.  The table is the authoritative state;
        each entry remains immutable until the matching ACK removes it.
        """
        table = getattr(self, "pending_controller_lease_releases", None)
        if not isinstance(table, dict):
            table = {}
            legacy = getattr(self, "pending_controller_lease_release", None)
            if isinstance(legacy, dict):
                try:
                    route_id = int(legacy.get("route_id", 0) or 0)
                except (TypeError, ValueError):
                    route_id = 0
                if route_id > 0:
                    table[route_id] = dict(legacy)
            self.pending_controller_lease_releases = table
        return table

    def _sync_controller_release_compatibility(self):
        """Update legacy scalar diagnostics from the route-keyed table."""
        table = self._pending_controller_release_table()
        for raw_route_id, contract in list(table.items()):
            try:
                route_id = int(raw_route_id or 0)
            except (TypeError, ValueError):
                route_id = 0
            if route_id <= 0 or not isinstance(contract, dict):
                table.pop(raw_route_id, None)
                continue
            if route_id != raw_route_id:
                table.pop(raw_route_id, None)
                table[route_id] = dict(contract)
        # A route chain should close promptly. Keep a bounded diagnostic set
        # if a broken peer sends an unbounded stream of release requests.
        if len(table) > self._MAX_PENDING_CONTROLLER_RELEASES:
            for route_id in sorted(table)[:-self._MAX_PENDING_CONTROLLER_RELEASES]:
                table.pop(route_id, None)
        latest_route_id = max(table) if table else 0
        self.pending_controller_lease_release = (
            dict(table[latest_route_id]) if latest_route_id > 0 else None
        )
        self.awaiting_controller_lease_release_ack = bool(table)
        return table

    def _remember_controller_release_contract(self, contract):
        """Store one predecessor contract without overwriting another route."""
        if not isinstance(contract, dict):
            return None
        try:
            route_id = int(contract.get("route_id", 0) or 0)
        except (TypeError, ValueError):
            route_id = 0
        if route_id <= 0:
            return None
        table = self._pending_controller_release_table()
        table[route_id] = dict(contract)
        self._sync_controller_release_compatibility()
        return table[route_id]

    def _pending_controller_release_for_route(self, route_id):
        """Return only the pending predecessor contract for ``route_id``."""
        try:
            route_id = int(route_id or 0)
        except (TypeError, ValueError):
            route_id = 0
        if route_id <= 0:
            return None
        table = self._pending_controller_release_table()
        pending = table.get(route_id)
        if isinstance(pending, dict):
            return pending
        legacy = getattr(self, "pending_controller_lease_release", None)
        if isinstance(legacy, dict):
            try:
                legacy_route_id = int(legacy.get("route_id", 0) or 0)
            except (TypeError, ValueError):
                legacy_route_id = 0
            if legacy_route_id == route_id:
                table[route_id] = dict(legacy)
                self._sync_controller_release_compatibility()
                return table[route_id]
        return None

    @classmethod
    def _is_failure_terminal(cls, message):
        """Identify a typed controller failure without inspecting geometry."""
        try:
            status = int(getattr(message, "status", 3) or 3)
        except (TypeError, ValueError):
            status = 3
        return bool(
            getattr(message, "failure", False)
            or status in cls._FAILURE_STATUS_CODES
        )

    @staticmethod
    def _failure_status_name(status):
        return {
            4: "aborted",
            5: "rejected",
            8: "recalled",
            9: "lost",
        }.get(int(status), "failed")

    @staticmethod
    def _lease_release_ack_identity(payload):
        """Decode the exact predecessor identity returned by the bridge."""
        released = payload.get("released_controller_route")
        released = released if isinstance(released, dict) else {}

        def integer(*values):
            for value in values:
                try:
                    value = int(value or 0)
                except (TypeError, ValueError):
                    continue
                # Nested release records may use zero as an unset sentinel;
                # allow the canonical outer field to supply the identity.
                if value > 0:
                    return value
            return 0

        return {
            "route_id": integer(
                released.get("route_id"),
                payload.get("released_route_id"),
                payload.get("route_id"),
            ),
            "transaction_id": integer(
                released.get("transaction_id"),
                payload.get("transaction_id"),
            ),
            "route_kind": str(
                released.get("route_kind")
                or payload.get("released_route_kind")
                or payload.get("route_kind")
                or ""
            ).strip().lower(),
            "lifecycle_transaction_id": integer(
                released.get("lifecycle_transaction_id"),
                payload.get("lifecycle_transaction_id"),
            ),
            "action_generation": integer(
                released.get("action_generation"),
                payload.get("action_generation"),
            ),
            "graph_transaction_id": integer(
                released.get("graph_transaction_id"),
                payload.get("graph_transaction_id"),
            ),
            "map_epoch": (
                released.get("map_epoch")
                if released.get("map_epoch") is not None
                else payload.get("map_epoch")
            ),
            "origin_epoch": (
                released.get("origin_epoch")
                if released.get("origin_epoch") is not None
                else payload.get("origin_epoch")
            ),
            "projection_epoch": (
                released.get("projection_epoch")
                if released.get("projection_epoch") is not None
                else payload.get("projection_epoch")
            ),
        }

    def _accept_controller_lease_release_ack(self, payload):
        """Open successor planning only after an exact transport ACK."""
        identity = self._lease_release_ack_identity(payload)
        pending = self._pending_controller_release_for_route(identity["route_id"])
        if not isinstance(pending, dict):
            self.publish_status(
                "stale_lease_release_ack",
                reason="no_pending_controller_release",
                received_identity=identity,
            )
            return False
        # A semantic endpoint release can be published before the Bridge's
        # asynchronous dispatch status has reached this node.  In that
        # narrow window the route-side contract deliberately carries zero
        # sentinels for fields assigned by the controller lease (currently
        # transaction and action generation).  The ACK is the authoritative
        # predecessor contract for those unset fields.  Rebind only missing
        # values; any two non-zero values must still match exactly.
        rebound_fields = []
        for field in (
            "transaction_id",
            "action_generation",
            "lifecycle_transaction_id",
            "graph_transaction_id",
        ):
            expected = pending.get(field, 0)
            received = identity.get(field, 0)
            try:
                expected_value = int(expected or 0)
                received_value = int(received or 0)
            except (TypeError, ValueError):
                continue
            if expected_value <= 0 and received_value > 0:
                pending[field] = received_value
                rebound_fields.append(field)
        for field in ("map_epoch", "origin_epoch", "projection_epoch"):
            if pending.get(field) is None and identity.get(field) is not None:
                pending[field] = identity[field]
                rebound_fields.append(field)
        if pending.get("route_kind", "") in (None, "") and identity.get(
            "route_kind", ""
        ):
            pending["route_kind"] = identity["route_kind"]
            rebound_fields.append("route_kind")
        if rebound_fields:
            self.publish_status(
                "controller_lease_release_ack_identity_rebound",
                route_id=identity.get("route_id", 0),
                rebound_fields=rebound_fields,
                expected_identity=dict(pending),
                received_identity=dict(identity),
                reason="missing_predecessor_identity_fields",
            )

        mismatch = None
        for field in (
            "route_id",
            "route_kind",
            "transaction_id",
            "lifecycle_transaction_id",
            "action_generation",
            "graph_transaction_id",
        ):
            expected = pending.get(field, 0 if field != "route_kind" else "")
            if identity[field] != expected:
                mismatch = "lease_release_ack_%s_mismatch" % field
                break
        if mismatch is None and pending.get("map_epoch") is not None:
            try:
                received_epoch = int(identity.get("map_epoch"))
                expected_epoch = int(pending["map_epoch"])
            except (TypeError, ValueError):
                mismatch = "lease_release_ack_map_epoch_invalid"
            else:
                if received_epoch != expected_epoch:
                    mismatch = "lease_release_ack_map_epoch_mismatch"
        for field in ("origin_epoch", "projection_epoch"):
            if mismatch is not None or pending.get(field) is None:
                continue
            try:
                if int(identity.get(field)) != int(pending[field]):
                    mismatch = "lease_release_ack_%s_mismatch" % field
            except (TypeError, ValueError):
                mismatch = "lease_release_ack_%s_invalid" % field
        if mismatch is not None:
            self.publish_status(
                "stale_lease_release_ack",
                reason=mismatch,
                expected_identity=pending,
                received_identity=identity,
            )
            return False
        if str(payload.get("controller_lease", "")).strip().lower() not in {
            "released",
            "transport_released",
        }:
            self.publish_status(
                "stale_lease_release_ack",
                reason="lease_release_ack_not_released",
                expected_identity=pending,
                received_identity=identity,
            )
            return False

        pending_table = self._pending_controller_release_table()
        pending_table.pop(identity["route_id"], None)
        self._sync_controller_release_compatibility()
        self.last_controller_lease_release_ack = dict(identity)
        last_released_route_id = int(
            getattr(self, "last_released_route_id", 0) or 0
        )
        self.last_released_route_controller_pending = bool(
            last_released_route_id in pending_table
        )
        self.last_planning_wall = 0.0
        self.publish_status(
            "controller_lease_release_acknowledged",
            **identity,
            new_action_generation=int(
                payload.get("new_action_generation", 0) or 0
            ),
            successor_planning="released",
            pending_controller_lease_release_count=len(pending_table),
        )
        active_route_id = int(getattr(self, "active_route_id", 0) or 0)
        successor_active = bool(
            getattr(self, "active_frontier", None) is not None
            and active_route_id > identity["route_id"]
        )
        if successor_active:
            # A delayed predecessor ACK closes only that predecessor. The
            # successor already owns the planner, so waking a second replan
            # here would create a duplicate route decision and can overwrite
            # its immutable contract.
            self.publish_status(
                "controller_lease_release_ack_late",
                route_id=identity["route_id"],
                successor_route_id=active_route_id,
                pending_controller_lease_release_count=len(pending_table),
                successor_planning="already_active",
            )
            return False
        lifecycle = getattr(self, "lifecycle_manager", None)
        current_lifecycle_id = int(
            getattr(lifecycle, "current_transaction_id", 0) or 0
        ) if lifecycle is not None else 0
        wake_payload = {
            "event": "terminal_replan_after_lease_ack",
            "route_id": identity["route_id"],
            # The ACK identifies the predecessor, but the wake belongs to the
            # lifecycle transaction currently owning the planner after that
            # predecessor has been released. Reusing the predecessor ID here
            # would enqueue a stale REPLAN_REQUESTED event after a timeout.
            "lifecycle_transaction_id": current_lifecycle_id,
        }
        if lifecycle is not None:
            lifecycle.enqueue_type(
                EventType.REPLAN_REQUESTED,
                wake_payload,
                transaction_id=current_lifecycle_id,
            )
        elif self.immediate_plan_timer is None and not rospy.is_shutdown():
            self.immediate_plan_timer = rospy.Timer(
                rospy.Duration(0.01), self.on_immediate_plan, oneshot=True
            )
        return False

    def schedule_terminal_replan(self, completed, terminal_delta):
        """Release a finished lease and schedule normal successor selection."""
        terminal_contract = getattr(self, "active_terminal_contract", None)
        terminal_contract = (
            dict(terminal_contract)
            if isinstance(terminal_contract, dict)
            else None
        )
        # Do not let the next SLAM timer re-project a completed mission endpoint
        # onto a nearby grid cell and publish a synthetic follow-up action.
        awaiting_ack = self.release_active_frontier(
            preserve_place_departure=bool(self.place_departure.active),
            queue_successor=False,
            terminal_contract=terminal_contract,
        )
        self.active_progress_time = 0.0
        self.active_last_progress_signal = "terminal_replan"
        self.publish_status(
            "terminal_replan_requested",
            completed_goal=(
                None if completed is None else [
                    round(float(completed[2]), 3),
                    round(float(completed[3]), 3),
                ]
            ),
            awaiting_controller_lease_release_ack=bool(awaiting_ack),
        )
        # Bypass only the compute-throttle, never route validation. The next
        # fixed lifecycle tick consumes this fact; no callback-owned timer is
        # needed for the successor boundary.
        self.last_planning_wall = 0.0
        if awaiting_ack:
            rospy.loginfo(
                "Global frontier holds successor planning until controller "
                "lease ACK route_id=%d",
                int(terminal_contract.get("route_id", 0) or 0)
                if terminal_contract is not None
                else 0,
            )
            return
        lifecycle = getattr(self, "lifecycle_manager", None)
        if lifecycle is not None:
            lifecycle.enqueue_type(
                EventType.REPLAN_REQUESTED,
                {"event": "terminal_replan"},
            )
            return
        if self.immediate_plan_timer is None and not rospy.is_shutdown():
            self.immediate_plan_timer = rospy.Timer(
                rospy.Duration(0.01), self.on_immediate_plan, oneshot=True
            )
        rospy.loginfo(
            "Global frontier schedules immediate successor selection "
            "after terminal delta=%.3fm route_id=%d pending=%s",
            terminal_delta,
            self.active_route_id,
            self.prefetched_frontier is not None,
        )

    def _arm_terminal_drain_timer(self):
        """Schedule a fast, non-blocking retry for queued terminal facts."""
        if getattr(self, "lifecycle_manager", None) is not None:
            return
        with self.terminal_ingress_lock:
            if (
                not self.pending_execution_terminals
                or self.terminal_drain_timer is not None
                or getattr(rospy, "is_shutdown", lambda: False)()
            ):
                return
            self.terminal_drain_timer = rospy.Timer(
                rospy.Duration(0.01),
                self.on_terminal_drain_timer,
                oneshot=True,
            )

    def on_execution_terminal(self, message):
        """Queue a terminal immediately; never wait for the planning lock."""
        # Keep this method self-contained for the narrow AST policy fixtures
        # that load the callback without executing module-level imports.
        from copy import deepcopy

        lifecycle = getattr(self, "lifecycle_manager", None)
        if lifecycle is not None:
            return lifecycle.enqueue_type(
                EventType.EXECUTION_TERMINAL, deepcopy(message)
            )
        # Compatibility doubles and old external compositions may not have
        # the runtime ingress state yet. Preserve their synchronous contract;
        # the production node initializes the queue before subscriptions.
        if not hasattr(self, "terminal_ingress_lock"):
            self._process_execution_terminal_locked(message)
            return
        terminal = deepcopy(message)
        # Do this before scheduling the drain timer. If the planning thread is
        # already inside a slow candidate pass, it can cooperatively abandon
        # that obsolete snapshot even though the callback never takes the
        # planning lock.
        with self.terminal_ingress_lock:
            proposal_gate = getattr(self, "planning_proposal_gate", None)
            if proposal_gate is not None:
                proposal_gate.invalidate("execution_terminal")
            self.planning_preempt_requested = True
            self.planning_preempt_reason = "execution_terminal"
            if len(self.pending_execution_terminals) >= (
                self.pending_execution_terminals.maxlen
            ):
                self.pending_execution_terminals.popleft()
                self.terminal_ingress_dropped += 1
            self.pending_execution_terminals.append(terminal)
        self._arm_terminal_drain_timer()

    def _process_execution_terminal_locked(self, message):
        """Apply one queued terminal at the normal planning boundary."""
        consume_released = getattr(
            self, "consume_late_released_terminal", None
        )
        if callable(consume_released) and consume_released(message):
            return
        terminal = self.matching_execution_terminal(message)
        if terminal is None:
            return
        completed, terminal_delta = terminal
        if self._is_failure_terminal(message):
            identity = self._terminal_identity(message)
            try:
                status = int(getattr(message, "status", 4) or 4)
            except (TypeError, ValueError):
                status = 4
            reason = str(
                getattr(message, "reason", "")
                or "controller_%s" % self._failure_status_name(status)
            )
            # The typed terminal is the controller's authoritative failure
            # boundary. Mark it before the next planning tick so the existing
            # route-resolution path performs semantic invalidation exactly once.
            self.active_terminal_received = True
            self.recovery_pending_route_id = int(identity["route_id"])
            self.recovery_pending_behavior = "controller_stall"
            self.recovery_pending_reason = reason
            self.recovery_pending_wall = rospy.Time.now().to_sec()
            self.last_planning_wall = 0.0
            self.publish_status(
                "execution_terminal_failure",
                route_id=int(identity["route_id"]),
                route_kind=str(identity["route_kind"] or ""),
                mission_route_kind=str(identity["mission_route_kind"] or ""),
                status=status,
                status_text=self._failure_status_name(status).upper(),
                reason=reason,
                terminal_kind="typed_failure",
                controller_stall=bool(
                    str(reason).startswith("controller_stall")
                    or str(getattr(message, "reason", "") or "").startswith(
                        "controller_stall"
                    )
                ),
                action_generation=int(identity["action_generation"]),
                transaction_id=int(identity.get("transaction_id", 0) or 0),
                lifecycle_transaction_id=int(
                    identity["lifecycle_transaction_id"]
                ),
                graph_transaction_id=int(identity["graph_transaction_id"]),
                map_epoch=int(identity["map_epoch"]),
                origin_epoch=int(identity.get("origin_epoch", 0) or 0),
                projection_epoch=int(
                    identity.get("projection_epoch", 0) or 0
                ),
            )
            rospy.logwarn(
                "Global frontier accepted typed controller failure: "
                "route_id=%d generation=%d reason=%s",
                int(identity["route_id"]),
                int(identity["action_generation"]),
                reason,
            )
            return
        consume_connector = getattr(
            self, "consume_connector_terminal", lambda: False,
        )
        if consume_connector():
            return
        self.record_execution_terminal(completed)
        if self.promote_prefetched_terminal(message):
            return
        self.schedule_terminal_replan(completed, terminal_delta)

    def _drain_execution_terminals_locked(self):
        """Drain a stable batch while the planning lock is already held."""
        if getattr(self, "lifecycle_manager", None) is not None:
            return
        with self.terminal_ingress_lock:
            pending = list(self.pending_execution_terminals)
            self.pending_execution_terminals.clear()
            self.terminal_drain_timer = None
        for message in pending:
            self._process_execution_terminal_locked(message)
        self._arm_terminal_drain_timer()

    def on_terminal_drain_timer(self, _event):
        """Try the planning lock without turning terminal ingress into a wait."""
        if getattr(self, "lifecycle_manager", None) is not None:
            return
        # A one-shot timer is no longer a pending wake once its callback starts.
        # Clear the handle before trying the planning lock; otherwise a busy
        # planning cycle makes ``_arm_terminal_drain_timer`` see the expired
        # handle and leaves queued terminal facts stranded indefinitely.
        with self.terminal_ingress_lock:
            self.terminal_drain_timer = None
        # The slow planner owns the state transition until its proposal has
        # either committed or been discarded. The terminal is already queued
        # and will be drained by the cycle's short cleanup boundary; running
        # it concurrently would mutate the same route lease mid-selection.
        if getattr(self, "planning_cycle_active", False):
            return
        acquired = self.planning_lock.acquire(False)
        if not acquired:
            self._arm_terminal_drain_timer()
            return
        try:
            self._drain_execution_terminals_locked()
        finally:
            self.planning_lock.release()
