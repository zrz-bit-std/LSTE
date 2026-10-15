"""External execution-event telemetry for the navigation observer.

This module translates read-only scan, controller, TEB bridge, and frontier
status topics into structured evidence. It has no publisher and never changes
controller state.
"""

import json
import math
import re
import time


def route_recovery_preemption_reason(event, payload, status_code):
    """Return the explicit reason for an intentional frontier cancellation."""
    if int(status_code) != 2 or not isinstance(payload, dict):
        return None
    event = str(event or "").strip()
    if event == "controller_lease_released":
        return str(
            payload.get("reason")
            or payload.get("frontier_lease_released_reason")
            or "controller_lease_released"
        )
    # The bridge reports the cancellation first and the frontier node reports
    # the matching invalidation immediately afterwards. Count only the
    # cancellation to avoid double-accounting one action boundary.
    if event == "cancel":
        reason = str(payload.get("reason", ""))
        if "frontier_route_invalidated" in reason:
            return reason
    return None


_EXPECTED_ROUTE_RECOVERY_WINDOW_SECONDS = 10.0
_EXPECTED_HARD_RESET_PREEMPTION_WINDOW_SECONDS = 10.0


def route_recovery_route_id(payload):
    """Return the route identity carried by a bridge/frontier event."""
    if not isinstance(payload, dict):
        return None
    released = payload.get("released_controller_route")
    released = released if isinstance(released, dict) else {}
    for value in (
        payload.get("route_id"),
        payload.get("released_route_id"),
        payload.get("active_route_id"),
        released.get("route_id"),
    ):
        try:
            value = int(value or 0)
        except (TypeError, ValueError):
            continue
        if value > 0:
            return value
    return None


def route_recovery_preemption_from_frontier_event(event, payload):
    """Detect a frontier release that will cause an intentional PREEMPTED.

    The frontier status and the bridge status use separate ROS topics. The
    frontier release can therefore arrive before the bridge publishes its
    ``controller_lease_released`` event, while move_base may publish PREEMPTED
    in between. Only the explicit terminal boundary is strong enough to arm
    this correlation; a deferred unavailable snapshot is not.
    """
    if str(event or "").strip() != "frontier_route_unavailable":
        return None
    payload = payload if isinstance(payload, dict) else {}
    released = payload.get("released_controller_route")
    released = released if isinstance(released, dict) else {}
    if not (
        bool(released.get("controller_pending"))
        and bool(released.get("terminal_received"))
    ):
        return None
    route_id = route_recovery_route_id(payload)
    if route_id is None:
        return None
    reason = str(payload.get("reason", "frontier_route_unavailable")).strip()
    return {
        "route_id": route_id,
        "reason": "endpoint_terminal_%s" % (
            reason or "frontier_route_unavailable"
        ),
        "source_event": str(event),
    }


def route_invalidation_failure_trigger(event, payload):
    """Map an explicit frontier route invalidation to a failure trigger.

    ``frontier_observed_at_standoff`` is a normal observation boundary: the
    robot has learned enough at the endpoint and the old route is intentionally
    released. Other invalidations are terminal evidence from the exploration
    executive and must remain visible even when the bridge subsequently emits
    an intentional ``PREEMPTED`` during recovery.
    """
    if str(event or "").strip() != "route_invalidated":
        return None
    if not isinstance(payload, dict):
        return None
    reason = str(payload.get("reason", "")).strip().lower()
    if not reason or reason == "frontier_observed_at_standoff":
        return None
    if (
        reason in ("stall", "post_turn_no_progress")
        or "stall" in reason
        or "no_progress" in reason
    ):
        return "route_invalidated_stall"
    if reason in ("active_timeout", "portal_edge_deadline") or "timeout" in reason:
        return "route_invalidated_timeout"
    if reason == "disconnected" or any(
        token in reason for token in ("no_path", "unreachable", "disconnected")
    ):
        return "route_invalidated_disconnected"
    if any(token in reason for token in ("recovery", "abort", "controller")):
        return "route_invalidated_controller_failure"
    return "route_invalidated_failure"


def graph_materialization_failure_trigger(event, payload):
    """Return the terminal failure trigger for a dropped graph obligation.

    A materialization miss is normal planning context.  The graph adapter's
    explicit invalidation, or its corresponding expired-unavailable event, is
    the terminal boundary that must enter the metrics failure pipeline.
    """
    event = str(event or "").strip().lower()
    payload = payload if isinstance(payload, dict) else {}
    reason = str(payload.get("reason", "") or "").strip().lower()
    if event in {
        "graph_route_materialization_invalidated",
        "graph_route_materialization_expired",
    }:
        return "graph_route_materialization_expired"
    if event in {
        "frontier_route_unavailable",
        "graph_route_candidate_unavailable",
    } and reason == "graph_route_materialization_expired":
        return "graph_route_materialization_expired"
    return None


def recoverable_portal_route_failure(event, payload):
    """Return whether one Portal route failure should await successor replanning.

    A self-loop or rejected Portal destination is a terminal fact for the
    current route, but the exploration executive can legitimately select a
    different probe/work item immediately afterwards.  Treat that narrow
    boundary as context first; the sustained unavailable-route watchdog still
    promotes it to a formal episode when no successor is materialized.
    """
    if str(event or "").strip() not in {
        "portal_hypothesis_failed",
        "portal_execution_failure_observed",
        "frontier_route_failed",
    } or not isinstance(payload, dict):
        return None
    reason = str(payload.get("reason", "") or "").strip().lower()
    route_kind = str(
        payload.get("route_kind")
        or payload.get("active_route_kind")
        or payload.get("mission_route_kind")
        or ""
    ).strip().lower()
    if route_kind not in {"portal_transition", "portal_probe"}:
        return None
    if reason not in {
        "portal_self_loop_rejected",
        "portal_destination_binding_rejected",
        "portal_destination_component_rejected",
    }:
        return None
    route_id = route_recovery_route_id(payload)
    if route_id is None:
        return None
    return {
        "route_id": route_id,
        "route_kind": route_kind,
        "reason": reason,
        "successor_replan_expected": True,
    }


class NavigationMetricsExecutionEventsMixin:
    @staticmethod
    def _metrics_int(value, default=0):
        raw = default if value is None or value == "" else value
        try:
            return int(raw or 0)
        except (TypeError, ValueError):
            try:
                return int(default or 0)
            except (TypeError, ValueError):
                return 0

    @classmethod
    def _bridge_dispatch_contract_from_payload(cls, payload):
        """Extract one bridge dispatch identity from its JSON status."""
        route_kind = payload.get("route_kind")
        if route_kind is None:
            route_kind = payload.get("active_route_kind", "")
        return {
            "transaction_id": cls._metrics_int(
                payload.get("transaction_id"),
                payload.get("active_goal_transaction_id", 0),
            ),
            "route_id": cls._metrics_int(
                payload.get("route_id"), payload.get("active_route_id", 0)
            ),
            "graph_transaction_id": cls._metrics_int(
                payload.get("graph_transaction_id"),
                payload.get("active_graph_transaction_id", 0),
            ),
            "map_epoch": (
                payload.get("map_epoch")
                if payload.get("map_epoch") is not None
                else payload.get("active_route_map_epoch")
            ),
            "origin_epoch": (
                payload.get("origin_epoch")
                if payload.get("origin_epoch") is not None
                else payload.get("active_origin_epoch")
            ),
            "projection_epoch": (
                payload.get("projection_epoch")
                if payload.get("projection_epoch") is not None
                else payload.get("active_projection_epoch")
            ),
            "lifecycle_transaction_id": cls._metrics_int(
                payload.get("lifecycle_transaction_id")
            ),
            "action_generation": cls._metrics_int(
                payload.get("action_generation"),
                payload.get("active_action_generation", 0),
            ),
            "route_kind": str(route_kind or "").strip().lower(),
            "mission_route_kind": str(
                payload.get("mission_route_kind")
                or payload.get("active_mission_route_kind", "")
                or ""
            ).strip().lower(),
        }

    def _remember_metrics_bridge_dispatch_locked(self, payload):
        event = str(payload.get("event", "") or "").strip()
        if event not in {
            "dispatch",
            "persistent_mission_goal_adopted",
            # Persistent execution keeps one actionlib goal alive while the
            # semantic route changes.  In that mode path adoption, rather
            # than a new transport dispatch, is the authoritative contract
            # boundary for the route that owns the next controller event.
            "persistent_mission_path_adopted",
            "persistent_frontier_successor_adopted",
            "dispatch_contract_reaffirmed",
            "successor_dispatched",
        }:
            return None
        contract = self._bridge_dispatch_contract_from_payload(payload)
        if contract["action_generation"] <= 0:
            return None
        self.bridge_latest_dispatch_contract = dict(contract)
        route_id = contract["route_id"]
        if route_id > 0:
            previous = self.bridge_dispatch_contracts.get(route_id)
            if (
                previous is None
                or contract["action_generation"] >= previous["action_generation"]
            ):
                self.bridge_dispatch_contracts[route_id] = dict(contract)
            for old_route_id in sorted(self.bridge_dispatch_contracts)[:-64]:
                self.bridge_dispatch_contracts.pop(old_route_id, None)
        return contract

    def _terminal_contract_reference_locked(self, identity):
        if identity["route_id"] > 0:
            return self.bridge_dispatch_contracts.get(identity["route_id"])
        return self.bridge_latest_dispatch_contract

    def _validate_terminal_contract_locked(self, identity):
        """Validate the typed terminal against the bridge dispatch identity."""
        if identity["lifecycle_transaction_id"] <= 0:
            return "missing_lifecycle_transaction_id"
        if identity["action_generation"] <= 0:
            return "missing_action_generation"
        if identity["route_id"] > 0 and identity["transaction_id"] <= 0:
            return "missing_transaction_id"
        reference = self._terminal_contract_reference_locked(identity)
        if reference is None:
            return "pending_bridge_dispatch"
        for field in (
            "route_id",
            "transaction_id",
            "lifecycle_transaction_id",
            "action_generation",
            "graph_transaction_id",
        ):
            expected = self._metrics_int(reference.get(field))
            if expected > 0 and identity[field] != expected:
                return "terminal_%s_mismatch" % field
        for field in ("origin_epoch", "projection_epoch"):
            expected_epoch = reference.get(field)
            if expected_epoch is None:
                continue
            try:
                if identity[field] != int(expected_epoch):
                    return "terminal_%s_mismatch" % field
            except (TypeError, ValueError):
                return "terminal_%s_invalid" % field
        expected_map_epoch = reference.get("map_epoch")
        if expected_map_epoch is not None:
            try:
                if identity["map_epoch"] != int(expected_map_epoch):
                    return "terminal_map_epoch_mismatch"
            except (TypeError, ValueError):
                return "terminal_map_epoch_invalid"
        for field in ("route_kind", "mission_route_kind"):
            expected = str(reference.get(field, "") or "").strip().lower()
            received = str(identity.get(field, "") or "").strip().lower()
            if expected and received and expected != received:
                return "terminal_%s_mismatch" % field
        return ""

    def _validate_controller_stall_identity_locked(self, payload):
        """Compare a controller-stall report with its dispatch contract."""
        payload = payload if isinstance(payload, dict) else {}
        route_id = self._metrics_int(payload.get("route_id"))
        reference = self.bridge_dispatch_contracts.get(route_id)
        if reference is None:
            reference = self.bridge_latest_dispatch_contract
        if reference is None:
            return "dispatch_contract_missing"
        mismatches = []
        for field in (
            "route_id",
            "transaction_id",
            "lifecycle_transaction_id",
            "action_generation",
            "graph_transaction_id",
        ):
            expected = self._metrics_int(reference.get(field))
            actual = self._metrics_int(payload.get(field))
            if expected > 0 and actual != expected:
                mismatches.append(field)
        for field in ("origin_epoch", "projection_epoch"):
            expected_epoch = reference.get(field)
            if expected_epoch is None:
                continue
            try:
                if int(payload.get(field)) != int(expected_epoch):
                    mismatches.append(field)
            except (TypeError, ValueError):
                mismatches.append(field)
        expected_map_epoch = reference.get("map_epoch")
        if expected_map_epoch is not None:
            try:
                if int(payload.get("map_epoch")) != int(expected_map_epoch):
                    mismatches.append("map_epoch")
            except (TypeError, ValueError):
                mismatches.append("map_epoch")
        return (
            "mismatch:%s" % ",".join(mismatches)
            if mismatches else "accepted"
        )

    @staticmethod
    def _terminal_contract_identity(message):
        """Extract typed terminal identity without using endpoint geometry."""
        def integer(value, default=0):
            try:
                return int(value or default)
            except (TypeError, ValueError):
                return int(default)

        return {
            "route_id": integer(getattr(message, "route_id", 0)),
            "transaction_id": integer(
                getattr(message, "transaction_id", 0)
            ),
            "map_epoch": integer(getattr(message, "map_epoch", 0)),
            "origin_epoch": integer(
                getattr(message, "origin_epoch", 0)
            ),
            "projection_epoch": integer(
                getattr(message, "projection_epoch", 0)
            ),
            "graph_transaction_id": integer(
                getattr(message, "graph_transaction_id", 0)
            ),
            "lifecycle_transaction_id": integer(
                getattr(message, "lifecycle_transaction_id", "")
            ),
            "action_generation": integer(
                getattr(message, "action_generation", 0)
            ),
            "route_kind": str(getattr(message, "route_kind", "") or ""),
            "mission_route_kind": str(
                getattr(message, "mission_route_kind", "") or ""
            ),
            "status": integer(getattr(message, "status", 3), default=3),
            "failure": bool(getattr(message, "failure", False)),
            "reason": str(getattr(message, "reason", "") or ""),
        }

    def _record_terminal_contract_locked(self, identity, reconciled=False):
        validation = self._validate_terminal_contract_locked(identity)
        if validation == "pending_bridge_dispatch":
            self.pending_terminal_contracts.append(dict(identity))
        elif validation:
            self.terminal_contract_rejections += 1
        else:
            self.terminal_contract_identity = dict(identity)
            self.terminal_contract_validation = "accepted"
        if validation:
            self.terminal_contract_validation = validation
        self._write(
            "WARN" if validation and validation != "pending_bridge_dispatch" else "INFO",
            "frontier_execution_terminal_contract",
            validation=validation or "accepted",
            reconciled=bool(reconciled),
            contract_identity=identity,
        )

    def _reconcile_pending_terminal_contracts_locked(self):
        pending = getattr(self, "pending_terminal_contracts", None)
        if not pending:
            return
        unresolved = []
        for identity in list(pending):
            validation = self._validate_terminal_contract_locked(identity)
            if validation == "pending_bridge_dispatch":
                unresolved.append(identity)
            else:
                self._record_terminal_contract_locked(identity, reconciled=True)
        pending.clear()
        pending.extend(unresolved)

    def on_frontier_execution_terminal(self, message):
        """Observe the identity-bearing terminal before geometry is analyzed."""
        with self.lock:
            self.terminal_contract_count += 1
            self._record_terminal_contract_locked(
                self._terminal_contract_identity(message)
            )

    def _arm_hard_reset_preemption_locked(self, had_active_action=None):
        """Correlate the transport PREEMPTED caused by a warm hard reset."""
        if had_active_action is None:
            had_active_action = bool(
                getattr(self, "bridge_active", False)
                or str(getattr(self, "last_move_base_status", "")).upper()
                in {"ACTIVE", "PENDING", "PREEMPTING"}
            )
        if not had_active_action:
            return False
        now = time.monotonic()
        if (
            int(getattr(self, "pending_hard_reset_preemptions", 0) or 0) > 0
            and float(
                getattr(self, "hard_reset_preemption_deadline_wall", 0.0)
                or 0.0
            ) > now
        ):
            return True
        self.pending_hard_reset_preemptions = 1
        self.hard_reset_preemption_deadline_wall = (
            now + _EXPECTED_HARD_RESET_PREEMPTION_WINDOW_SECONDS
        )
        self._write(
            "INFO",
            "expected_hard_reset_preemption",
            correlation_window_seconds=(
                _EXPECTED_HARD_RESET_PREEMPTION_WINDOW_SECONDS
            ),
        )
        return True

    def _consume_hard_reset_preemption_locked(self):
        """Consume one reset cancellation, rejecting stale expectations."""
        pending = int(getattr(self, "pending_hard_reset_preemptions", 0) or 0)
        if pending <= 0:
            return False
        deadline = float(
            getattr(self, "hard_reset_preemption_deadline_wall", 0.0) or 0.0
        )
        now = time.monotonic()
        if deadline > 0.0 and now > deadline:
            self.pending_hard_reset_preemptions = 0
            self.hard_reset_preemption_deadline_wall = 0.0
            self._write(
                "WARN",
                "expected_hard_reset_preemption_expired",
            )
            return False
        self.pending_hard_reset_preemptions = pending - 1
        self.hard_reset_preemption_deadline_wall = 0.0
        return True

    def _purge_route_recovery_expectations_locked(self, now=None):
        """Drop preemption expectations that outlived their route boundary."""
        now = time.monotonic() if now is None else float(now)
        pending = getattr(self, "expected_route_recovery_preemptions", None)
        if pending is None:
            return
        while pending and (
            now - float(pending[0].get("armed_wall", now))
            > _EXPECTED_ROUTE_RECOVERY_WINDOW_SECONDS
        ):
            pending.popleft()

    def _arm_expected_route_recovery_preemption_locked(self, expectation):
        """Remember a route-specific cancellation before transport reports it."""
        if not isinstance(expectation, dict):
            return False
        route_id = route_recovery_route_id(expectation)
        if route_id is None:
            return False
        now = time.monotonic()
        self._purge_route_recovery_expectations_locked(now)
        pending = self.expected_route_recovery_preemptions
        consumed = self.consumed_route_recovery_route_ids
        if route_id in consumed or any(
            int(item.get("route_id", 0) or 0) == route_id for item in pending
        ):
            return False
        # A bridge release may win the topic race. In that case the existing
        # legacy counter already represents this exact pending PREEMPTED.
        if int(getattr(self, "pending_route_recovery_preemptions", 0) or 0) > 0:
            return False
        item = {
            "route_id": route_id,
            "reason": str(expectation.get("reason", "route_released")),
            "source_event": str(expectation.get("source_event", "frontier")),
            "armed_wall": now,
        }
        pending.append(item)
        self._write(
            "INFO",
            "expected_route_recovery_preemption",
            route_id=route_id,
            reason=item["reason"],
            source_event=item["source_event"],
            correlation_window_seconds=_EXPECTED_ROUTE_RECOVERY_WINDOW_SECONDS,
        )
        return True

    def _consume_expected_route_recovery_preemption_locked(self):
        """Consume the oldest still-valid route-specific PREEMPTED expectation."""
        now = time.monotonic()
        self._purge_route_recovery_expectations_locked(now)
        pending = self.expected_route_recovery_preemptions
        if not pending:
            return None
        item = pending.popleft()
        route_id = int(item.get("route_id", 0) or 0)
        if route_id > 0:
            self.consumed_route_recovery_route_ids.append(route_id)
        return item

    def _consume_recent_frontier_release_preemption_locked(self):
        """Correlate a release that raced the bridge/action status topics.

        Frontier, bridge, and move_base publish on separate ROS topics. In a
        busy callback cycle the explicit expectation can be observed after
        the bridge release but before actionlib reports PREEMPTED, or the
        legacy counter can be consumed by that same ordering. The bounded
        context history still contains the causal release and is sufficient
        to distinguish this handoff from an unexplained preemption.
        """
        history = getattr(self, "failure_event_history", ())
        if not history:
            return None
        now_elapsed = max(
            0.0,
            time.monotonic() - float(getattr(self, "start_wall", time.monotonic())),
        )
        for context in reversed(history):
            if not isinstance(context, dict):
                continue
            try:
                age = now_elapsed - float(context.get("wall_elapsed_seconds", 0.0))
            except (TypeError, ValueError):
                age = float("inf")
            if age > _EXPECTED_ROUTE_RECOVERY_WINDOW_SECONDS:
                break
            event = str(context.get("event", "") or "").strip().lower()
            source = str(context.get("source", "") or "").strip().lower()
            if event not in {
                "frontier_route_unavailable",
                "controller_lease_released",
            }:
                continue
            payload = context.get("payload")
            payload = payload if isinstance(payload, dict) else {}
            released = payload.get("released_controller_route")
            released = released if isinstance(released, dict) else {}
            try:
                status = int(
                    payload.get("status", payload.get("result_status", 2))
                    or 2
                )
            except (TypeError, ValueError):
                status = 2
            if event == "controller_lease_released" and status != 2:
                continue
            if event == "frontier_route_unavailable" and not (
                bool(released.get("controller_pending"))
                and bool(released.get("terminal_received"))
            ):
                continue
            route_id = route_recovery_route_id(payload)
            if route_id is None:
                route_id = route_recovery_route_id(released)
            if route_id is None:
                continue
            self.consumed_route_recovery_route_ids.append(route_id)
            return {
                "route_id": route_id,
                "reason": str(
                    payload.get("reason")
                    or released.get("reason")
                    or "frontier_release_context"
                ),
                "source_event": event,
                "source": source,
            }
        return None

    def _route_recovery_preemption_already_accounted_locked(self, payload):
        """Avoid counting the bridge release after frontier-side correlation."""
        route_id = route_recovery_route_id(payload)
        if route_id is None:
            return False
        now = time.monotonic()
        self._purge_route_recovery_expectations_locked(now)
        if route_id in self.consumed_route_recovery_route_ids:
            return True
        return any(
            int(item.get("route_id", 0) or 0) == route_id
            for item in self.expected_route_recovery_preemptions
        )

    def on_scan(self, message):
        all_ranges = list(message.ranges)
        forward, left, right = [], [], []
        for index, value in enumerate(all_ranges):
            angle = message.angle_min + index * message.angle_increment
            if not math.isfinite(value) or value <= 0.01:
                continue
            if abs(angle) <= math.radians(20.0):
                forward.append(value)
            elif 0.0 < angle <= math.radians(90.0):
                left.append(value)
            elif -math.radians(90.0) <= angle < 0.0:
                right.append(value)
        with self.lock:
            self.last_scan_wall = time.monotonic()
            self.scan_minimum = self._finite_min(all_ranges)
            self.scan_forward_minimum = self._finite_min(forward)
            self.scan_left_minimum = self._finite_min(left)
            self.scan_right_minimum = self._finite_min(right)
            if math.isfinite(self.scan_minimum):
                self.min_clearance = min(self.min_clearance, self.scan_minimum)

    def on_controller_mode(self, message):
        with self.lock:
            value = message.data.strip().lower()
            if value and value != self.controller_mode:
                self.controller_mode = value
                if value != "sappo":
                    # The SA-PPO status topic remains alive while TEB is
                    # selected. Never expose that stale policy decision as a
                    # TEB diagnosis in the run summary.
                    self.controller_status = "not_applicable"
                self._write("INFO", "controller_mode", mode=value)

    def on_bridge_status(self, message):
        """Record action-level goal queueing separately from move_base status.

        A queued update is expected during a healthy TEB action; a move_base
        PREEMPTED status is not. Keeping both counters makes that distinction
        explicit in the run summary.
        """
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = {"event": "invalid", "raw": message.data}
        if not isinstance(payload, dict):
            payload = {"event": "invalid", "raw": message.data}
        with self.lock:
            event = str(payload.get("event", "unknown"))
            dispatch_contract = self._remember_metrics_bridge_dispatch_locked(
                payload
            )
            if dispatch_contract is not None:
                self._reconcile_pending_terminal_contracts_locked()
                reconcile_planner = getattr(
                    self, "_reconcile_pending_planner_contract_locked", None
                )
                if callable(reconcile_planner):
                    reconcile_planner()
            context_payload = dict(payload)
            self._failure_record_context_locked("bridge", event, context_payload)
            # Bridge status calls this field ``result_status`` while
            # actionlib uses ``status``. Accept both so an intentional cancel
            # is correlated before /move_base/status arrives.
            status_code = int(
                payload.get("status", payload.get("result_status", -1)) or -1
            )
            route_recovery_reason = route_recovery_preemption_reason(
                event, payload, status_code
            )
            if (
                route_recovery_reason is not None
                and not self._route_recovery_preemption_already_accounted_locked(
                    payload
                )
            ):
                self.pending_route_recovery_preemptions += 1
                self._increment_reason(
                    self.route_recovery_preemption_reasons,
                    route_recovery_reason,
                )
            if (
                event == "cancel"
                and str(payload.get("reason", "") or "").strip()
                == "experiment_hard_reset"
            ):
                # The hard-reset topic is delivered independently to each
                # node. The bridge's explicit cancel event is the earliest
                # cross-node evidence that the following PREEMPTED belongs to
                # the warm-slice reset, so arm correlation here as well as in
                # the metrics reset callback.
                self._arm_hard_reset_preemption_locked(True)
            if event == "route_lease_watchdog_expired" and not self.task_done:
                # The bridge has already enforced the ten-second transport
                # boundary and revoked the old owner. Preserve that explicit
                # handoff failure instead of letting the subsequent idle
                # sample degrade into a generic no-route timeout.
                self._begin_failure_episode_locked(
                    "handoff_timeout",
                    "teb_goal_bridge",
                    context_payload,
                )
            elif event == "controller_stall" and not self.task_done:
                # The bridge has already made the identity-checked terminal
                # transition. Enter the same metrics-owned episode path as
                # other explicit failures so the bounded ring and artifact
                # capture the state immediately before the lease is revoked.
                failure_details = dict(context_payload)
                failure_details.setdefault(
                    "failure_reason",
                    failure_details.get("reason", "controller_stall"),
                )
                failure_details["identity_validation"] = (
                    self._validate_controller_stall_identity_locked(
                        context_payload
                    )
                )
                self._begin_failure_episode_locked(
                    "controller_stall",
                    "teb_goal_bridge",
                    failure_details,
                )
            elif event in (
                "target_route_failed",
                "persistent_target_plan_failed",
                "target_plan_failed",
            ) or (event == "terminal" and status_code in (4, 5, 8, 9)):
                self._begin_failure_episode_locked(
                    "target_route_failed" if "target" in event else "bridge_terminal",
                    "teb_goal_bridge",
                    context_payload,
                )
            self.lifecycle_event_wall[event] = time.monotonic()
            self.bridge_events += 1
            self.bridge_last_event = event
            self.bridge_active = bool(payload.get("active", False))
            if "persistent_execution" in payload:
                self.persistent_execution = self._as_bool(
                    payload.get("persistent_execution")
                )
                self.execution_architecture = (
                    "persistent_stream" if self.persistent_execution
                    else "endpoint_action"
                )
            self.bridge_active_intent_source = str(
                payload.get("active_intent_source", self.bridge_active_intent_source)
            )
            self.bridge_latest_intent_source = str(
                payload.get("latest_intent_source", self.bridge_latest_intent_source)
            )
            active_route_id = int(payload.get("active_route_id", 0) or 0)
            active_goal_transaction_id = int(
                payload.get("active_goal_transaction_id", 0) or 0
            )
            active_generation = int(
                payload.get("active_action_generation", 0) or 0
            )
            active_graph_transaction_id = int(
                payload.get("active_graph_transaction_id", 0) or 0
            )
            active_map_epoch = payload.get("active_route_map_epoch")
            owner_identity = (
                active_generation,
                active_goal_transaction_id,
                active_route_id,
                active_graph_transaction_id,
                active_map_epoch,
            ) if self.bridge_active else None
            previous_owner_identity = getattr(
                self, "teb_feedback_owner_identity", None
            )
            self.teb_feedback_owner_identity = owner_identity
            invalidate_feedback = getattr(
                self, "_invalidate_teb_feedback_locked", None
            )
            if callable(invalidate_feedback):
                if (
                    previous_owner_identity is not None
                    and owner_identity != previous_owner_identity
                ):
                    invalidate_feedback(
                        "route_owner_changed", clear_planner=True
                    )
                elif not self.bridge_active:
                    invalidate_feedback("bridge_inactive", clear_planner=True)
                elif (
                    event == "teb_feedback_invalidated"
                    or payload.get("teb_feedback_valid") is False
                ):
                    invalid_reason = str(
                        payload.get(
                            "teb_feedback_invalid_reason",
                            payload.get("reason", "bridge_feedback_invalidated"),
                        )
                        or "bridge_feedback_invalidated"
                    )
                    invalidate_feedback(
                        invalid_reason,
                        clear_planner=bool(payload.get("clear_planner", False)),
                    )
            if event == "goal_deferred":
                self.bridge_deferred_goal_updates += 1
            elif event == "dispatch":
                self.bridge_dispatches += 1
            elif event == "endpoint_detected":
                self.bridge_endpoint_detections += 1
            elif event == "successor_dispatched":
                self.bridge_successor_dispatches += 1
            elif event == "terminal":
                self.bridge_terminal_events += 1
                self._mark_action_terminal_locked(time.monotonic(), "bridge_terminal")
                self._reclassify_recent_discontinuities_locked(
                    time.monotonic(), "bridge_terminal"
                )
            elif event == "frontier_observation_completion_requested":
                self.bridge_frontier_observation_completions += 1
                self.pending_frontier_observation_preemptions += 1
            elif event == "frontier_terminal_settle_completion_requested":
                self.bridge_frontier_terminal_settle_completions += 1
                self.pending_frontier_terminal_settle_preemptions += 1
            elif event == "frontier_continuous_prefetch_handoff_requested":
                # Native `send_goal` replacement still appears as PREEMPTED
                # in /move_base/status. It is an intentional continuous route
                # transition, not an action failure or goal churn defect.
                self.pending_frontier_continuous_prefetch_preemptions += 1
            elif event == "frontier_continuous_prefetch_handoff_completed":
                self.bridge_frontier_continuous_prefetch_handoffs += 1
            elif event == "frontier_continuous_prefetch_handoff_fallback":
                self.bridge_frontier_continuous_prefetch_fallbacks += 1
            elif event in (
                "persistent_frontier_lookahead_handoff_promoted",
                "persistent_frontier_curve_handoff_promoted",
            ):
                self.bridge_persistent_lookahead_handoffs += 1
                if event == "persistent_frontier_curve_handoff_promoted":
                    self.bridge_persistent_curve_handoffs += 1
            elif event == "persistent_frontier_prefetch_admission_deferred":
                self.bridge_persistent_lookahead_admission_deferred += 1
                deferred_reason = str(payload.get("reason", "unknown"))
                self._increment_reason(
                    self.bridge_persistent_lookahead_admission_reasons,
                    deferred_reason,
                )
                if deferred_reason in (
                    "entry_tangent_too_sharp",
                    "navfn_entry_tangent_too_sharp",
                ):
                    active_goal = payload.get("active_goal")
                    if isinstance(active_goal, (list, tuple)) and len(active_goal) >= 2:
                        try:
                            self.pending_terminal_native_reorientation = {
                                "armed_wall": time.monotonic(),
                                "goal": (
                                    float(active_goal[0]),
                                    float(active_goal[1]),
                                ),
                                "route_id": int(payload.get("route_id", 0) or 0),
                                "entry_heading_delta_deg": float(
                                    payload.get("entry_heading_delta_deg", 0.0)
                                ),
                            }
                        except (TypeError, ValueError):
                            self.pending_terminal_native_reorientation = None
            elif event == "frontier_prefetch_requires_turn":
                self.bridge_frontier_prefetch_requires_turn += 1
            elif event == "priority_handoff_requested":
                self.bridge_priority_handoffs += 1
                self.pending_priority_preemptions += 1
            elif (
                event == "handoff_requested"
                and str(payload.get("reason", "")) == "higher_priority_intent"
            ):
                # With in-place replacement disabled, the bridge uses the
                # explicit cancel -> terminal callback -> dispatch lifecycle.
                # It has the same mission semantics as a priority-intent
                # replacement, but carries ``handoff_requested`` rather than
                # ``replacement_kind=priority_intent`` on the wire.
                self.bridge_priority_handoffs += 1
                self.pending_priority_preemptions += 1
            elif event == "target_retry_requested":
                self.bridge_target_retries += 1
            elif event == "target_route_failed":
                self.target_route_failures += 1
            elif (
                event == "cancel"
                and str(payload.get("reason", ""))
                == "target_terminal_observation"
                and bool(payload.get("was_active", True))
            ):
                # The terminal-observation intent deliberately cancels the
                # current target action before the fresh observation phase.
                # actionlib reports that cancellation as PREEMPTED; correlate
                # it explicitly so the observer does not open a failure
                # episode for a normal ownership boundary.
                if self.pending_target_terminal_observation_preemptions <= 0:
                    self.pending_target_terminal_observation_preemptions = 1
            elif event == "target_segment_handoff_requested":
                self.bridge_target_segment_handoffs += 1
            elif event == "cancel" and str(payload.get("reason", "")) == "task_done":
                # Only task completion is a deliberate cancel. Other bridge
                # cancel reasons remain observable as unexpected unless their
                # own explicit lifecycle classification handles them.
                self.pending_task_done_preemptions += 1
            replacement = bool(payload.get("replacement", False))
            replacement_kind = str(payload.get("replacement_kind", "none"))
            if replacement:
                self.bridge_goal_replacements += 1
                if replacement_kind == "priority_intent":
                    self.bridge_priority_goal_replacements += 1
                    # New bridge versions report the handoff on the dispatch
                    # itself; keep the legacy counter meaningful as well.
                    if event == "dispatch":
                        self.bridge_priority_handoffs += 1
                        # Native actionlib replacement reports the superseded
                        # move_base goal as PREEMPTED. It is an intentional
                        # mission-priority transition, not goal churn.
                        self.pending_priority_preemptions += 1
                elif replacement_kind == "target_segment":
                    self.bridge_target_goal_replacements += 1
                    if event == "dispatch":
                        self.bridge_target_segment_handoffs += 1
                        # Native actionlib replacement appears as PREEMPTED on
                        # /move_base/status. This is an expected same-track
                        # continuation, not unexpected goal churn.
                        self.pending_target_segment_preemptions += 1
                elif replacement_kind in (
                    "frontier_segment",
                    "frontier_route_connector",
                    "frontier_route_endpoint",
                    "frontier_sharp_branch",
                ):
                    self.bridge_frontier_segment_handoffs += 1
                    if replacement_kind == "frontier_sharp_branch":
                        self.bridge_frontier_sharp_replacements += 1
                    if event == "dispatch":
                        # Legacy rolling connector experiments still use a
                        # native action replacement. It is intentionally
                        # observable in the smoothness metrics, but must not
                        # be reported as an unexplained move_base failure.
                        self.pending_frontier_segment_preemptions += 1
            # ``_write`` already has an ``event`` positional argument; keep
            # the bridge's event name as data instead of passing it twice.
            bridge_event = payload.pop("event", event)
            self._write(
                "INFO",
                "teb_bridge_event",
                bridge_event=bridge_event,
                **payload,
            )

    def on_persistent_execution_terminal(self, message):
        """Record a bridge endpoint-release signal with its real architecture."""
        with self.lock:
            endpoint = (
                round(float(message.pose.position.x), 3),
                round(float(message.pose.position.y), 3),
            )
            now = time.monotonic()
            lifecycle_event = (
                "persistent_execution_terminal"
                if self.persistent_execution else "endpoint_action_terminal"
            )
            replacement_reason = (
                "persistent_endpoint_terminal"
                if self.persistent_execution else "endpoint_action_terminal"
            )
            self.lifecycle_event_wall[lifecycle_event] = now
            self._reclassify_recent_discontinuities_locked(
                now,
                lifecycle_event,
                replacement_reason=replacement_reason,
                window=1.0,
            )
            self._write(
                "INFO",
                lifecycle_event,
                execution_architecture=self.execution_architecture,
                endpoint=endpoint,
                frame=(message.header.frame_id or "").strip().lstrip("/"),
                current_goal=(
                    None
                    if self.goal is None
                    else [round(float(self.goal[0]), 3), round(float(self.goal[1]), 3)]
                ),
                teb_command=[
                    round(float(self.teb_command.linear.x), 4),
                    round(float(self.teb_command.angular.z), 4),
                ],
            )

    def on_global_frontier_status(self, message):
        """Preserve frontier route ownership changes beside controller events."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        with self.lock:
            event = str(payload.pop("event", "unknown")).strip() or "unknown"
            now = time.monotonic()
            expected_preemption = route_recovery_preemption_from_frontier_event(
                event, payload
            )
            if expected_preemption is not None:
                self._arm_expected_route_recovery_preemption_locked(
                    expected_preemption
                )
            self._failure_record_context_locked("frontier", event, payload)
            materialization_trigger = graph_materialization_failure_trigger(
                event, payload
            )
            deferred_portal_failure = recoverable_portal_route_failure(
                event, payload
            )
            route_failure_trigger = route_invalidation_failure_trigger(
                event, payload
            )
            if not self.task_done and materialization_trigger is not None:
                # ``graph_route_materialization_invalidated`` is the graph
                # layer's explicit obligation-drop boundary. The companion
                # unavailable event is accepted as a fallback when topic
                # delivery races the invalidation status.
                failure_details = dict(payload)
                failure_details["materialization_event"] = event
                failure_details.setdefault(
                    "route_invalidation_reason",
                    payload.get("reason", materialization_trigger),
                )
                self._begin_failure_episode_locked(
                    materialization_trigger,
                    "global_frontier",
                    failure_details,
                )
            elif not self.task_done and event in (
                "execution_terminal_failure",
                "frontier_route_failed",
                "portal_hypothesis_failed",
                "portal_execution_failure_observed",
            ):
                if deferred_portal_failure is not None:
                    self._write(
                        "INFO",
                        "frontier_route_failure_deferred",
                        **deferred_portal_failure,
                    )
                else:
                    self._begin_failure_episode_locked(
                        "frontier_route_failure", "global_frontier", payload
                    )
            elif route_failure_trigger is not None and not self.task_done:
                # The following bridge PREEMPTED is an intentional recovery
                # action, but the invalidation itself is already a failure
                # boundary. Keep both facts with one failure ID.
                failure_details = dict(payload)
                failure_details["route_invalidation_reason"] = str(
                    payload.get("reason", "unknown")
                )
                self._begin_failure_episode_locked(
                    route_failure_trigger,
                    "global_frontier",
                    failure_details,
                )
            self.lifecycle_event_wall["global_frontier_" + event] = now
            # The terminal pose topic is intentionally non-latched. A metrics
            # node can attach after a rapid terminal callback, so the frontier
            # node's direct successor-promotion record is the durable evidence
            # needed to classify the preceding zero command correctly.
            if event == "terminal_prefetch_promoted":
                lifecycle_event = (
                    "persistent_execution_terminal"
                    if self.persistent_execution else "endpoint_action_terminal"
                )
                replacement_reason = (
                    "persistent_endpoint_terminal"
                    if self.persistent_execution else "endpoint_action_terminal"
                )
                self.lifecycle_event_wall[lifecycle_event] = now
                self._mark_action_terminal_locked(
                    now, "global_frontier_terminal_prefetch_promoted"
                )
                self._reclassify_recent_discontinuities_locked(
                    now,
                    "global_frontier_terminal_prefetch_promoted",
                    replacement_reason=replacement_reason,
                    window=1.5,
                )
            elif event in {
                "persistent_frontier_endpoint_ignored",
                "persistent_frontier_goal_replaced",
            }:
                # A persistent bridge can reject a stale endpoint while the
                # graph layer installs the next route identity. The resulting
                # zero command is a goal handoff, not a clear-path safety
                # brake; preserve the raw bridge event and repair telemetry
                # causality for the preceding command sample.
                self._reclassify_recent_discontinuities_locked(
                    now,
                    "global_frontier_%s" % event,
                    replacement_reason="goal_transition",
                    window=1.5,
                )
            self._write(
                "INFO",
                "global_frontier_event",
                frontier_event=event,
                execution_architecture=self.execution_architecture,
                **payload,
            )

    def on_controller_status(self, message):
        with self.lock:
            if self.controller_mode == "sappo":
                text = message.data.strip()
                self.controller_status = text
                self.last_status_text = text
                match = re.search(
                    r"source=(\S+)\s+reason=(.*?)\s+requested=\(([-+0-9.eE]+),([-+0-9.eE]+)\)\s+"
                    r"action=\(([-+0-9.eE]+),([-+0-9.eE]+)\)\s+clearance=([-+0-9.eE]+|nan|inf)",
                    text,
                )
                if match:
                    self.controller_source = match.group(1)
                    self.controller_reason = match.group(2).strip()
                    self.controller_requested = (float(match.group(3)), float(match.group(4)))
                    self.controller_action = (float(match.group(5)), float(match.group(6)))
                    self.controller_action_delta = math.hypot(
                        self.controller_requested[0] - self.controller_action[0],
                        self.controller_requested[1] - self.controller_action[1],
                    )
                    safety_source = self.controller_source in (
                        "grid_guard", "dwa_guard", "mppi_guard", "turn_recovery"
                    )
                    safety_reason = any(
                        token in self.controller_reason
                        for token in ("collision", "emergency_stop", "no_safe", "blocked")
                    )
                    if safety_source or safety_reason:
                        self.safety_intervention_samples += 1
                    if any(
                        token in self.controller_reason
                        for token in ("emergency_stop", "no_safe")
                    ):
                        self.hard_stop_events += 1
                    try:
                        self.controller_predicted_clearance = float(match.group(7))
                    except ValueError:
                        self.controller_predicted_clearance = float("nan")
                else:
                    self.controller_source = "unknown"
                    self.controller_reason = text
                # The status contains rolling diagnostics (point counts,
                # waypoint coordinates, lock timers) that change every control
                # cycle. Treat only source plus the stable reason prefix as a
                # controller-state transition; the latest numeric fields are
                # still retained in every periodic sample.
                reason_signature = self.controller_reason.split(" grid_points=", 1)[0]
                reason_signature = re.sub(
                    r"grid_waypoint=\([^)]*\)", "grid_waypoint", reason_signature
                )
                signature = (self.controller_source, reason_signature)
                if signature == self.last_status_signature:
                    return
                self.last_status_signature = signature
                self.controller_status_changes += 1
                if self.controller_source not in ("policy", "unknown", "stop"):
                    self.safety_override_events += 1
                self._write(
                    "INFO" if self.controller_source in ("policy", "unknown") else "WARN",
                    "controller_status_change",
                    count=self.controller_status_changes,
                    source=self.controller_source,
                    reason=reason_signature,
                    requested=list(self.controller_requested),
                    action=list(self.controller_action),
                    action_delta=round(self.controller_action_delta, 4),
                    predicted_clearance=None if not math.isfinite(self.controller_predicted_clearance) else round(self.controller_predicted_clearance, 4),
                    safety_override_events=self.safety_override_events,
                    safety_intervention_samples=self.safety_intervention_samples,
                    hard_stop_events=self.hard_stop_events,
                    pose=None if self.pose is None else [round(value, 3) for value in self.pose],
                    goal=None if self.goal is None else [round(value, 3) for value in self.goal],
                    scan_forward_min=None if not math.isfinite(self.scan_forward_minimum) else round(self.scan_forward_minimum, 4),
                    scan_min=None if not math.isfinite(self.scan_minimum) else round(self.scan_minimum, 4),
                )
