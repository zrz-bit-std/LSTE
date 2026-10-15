"""ROS input adapters for :mod:`lste_teb_turn_supervisor`.

This mixin owns message decoding and cached runtime observations.  It never
decides a velocity itself; route and execution modules own those decisions.
"""

import copy
import json
import math

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from lste_topo_access.msg import PlannerCommandContract

from clock_provider import now_for

from teb_turn_supervisor_contract import (
    FRONTIER_ENDPOINT_KIND,
    FRONTIER_SOURCE,
    LOCAL_EGRESS_KIND,
    PORTAL_PROBE_KIND,
    STATE_TURNING,
    TURN_ROUTE_KIND,
    PORTAL_TRANSITION_KIND,
)


class TebTurnSupervisorCallbacksMixin:
    """Decode ROS callbacks into the supervisor's synchronized state."""

    _BRIDGE_FEEDBACK_BOUNDARY_REASONS = frozenset(
        {
            "new_action_dispatch",
            "route_owner_changed",
            "planner_contract_endpoint_reached",
            "planner_contract_terminal_hold",
            "terminal_prepare",
            "controller_stall",
            "bridge_inactive",
            "action_lease_cleared",
            "controller_lease_released",
            "persistent_endpoint_terminal",
            "task_done",
            "navigation_hold",
            "controller_mode_changed",
            "planner_zero_command",
        }
    )

    def _clear_recovery_execution_locked(self, reason):
        """Drop the recovery-command bridge after a route/contract boundary."""
        if not getattr(self, "recovery_active", False):
            return
        self.recovery_active = False
        self.recovery_behavior = ""
        self.recovery_identity = None
        self.latest_recovery_command = Twist()
        self.latest_recovery_command_wall = 0.0
        self.publish_status_locked(
            "recovery_command_bridge_cleared",
            reason=str(reason),
        )

    def on_recovery_status(self, message):
        """Bind move_base recovery output to the current action identity."""
        try:
            current = int(message.current_recovery_number)
            total = int(message.total_number_of_recoveries)
        except (AttributeError, TypeError, ValueError):
            return
        behavior = str(
            getattr(message, "recovery_behavior_name", "") or ""
        ).strip()
        with self.lock:
            active = bool(
                behavior
                and self.active_action
                and current >= 0
                and (total <= 0 or current < total)
            )
            if not active:
                self._clear_recovery_execution_locked("recovery_status_inactive")
                return
            self.recovery_active = True
            self.recovery_behavior = behavior
            self.recovery_identity = self.active_action_identity
            self.latest_recovery_command = Twist()
            self.latest_recovery_command_wall = 0.0
            self.publish_status_locked(
                "recovery_command_bridge_armed",
                behavior=behavior,
                recovery_number=current,
                recovery_total=total,
                action_identity=self.active_action_identity,
            )

    @staticmethod
    def _planner_contract_identity(message):
        """Decode a typed planner frame into scalar identity fields."""
        def integer(value, default=0):
            try:
                return int(value or default)
            except (TypeError, ValueError):
                return int(default)

        return {
            "transaction_id": integer(getattr(message, "transaction_id", 0)),
            "route_id": integer(getattr(message, "route_id", 0)),
            "graph_transaction_id": integer(
                getattr(message, "graph_transaction_id", 0)
            ),
            "map_epoch": integer(getattr(message, "map_epoch", 0)),
            "origin_epoch": integer(
                getattr(message, "origin_epoch", 0)
            ),
            "projection_epoch": integer(
                getattr(message, "projection_epoch", 0)
            ),
            "lifecycle_transaction_id": integer(
                getattr(message, "lifecycle_transaction_id", "")
            ),
            "action_generation": integer(
                getattr(message, "action_generation", 0)
            ),
            "planner_sequence": integer(
                getattr(message, "planner_sequence", 0)
            ),
            "state": integer(getattr(message, "state", 0)),
            "producer": str(getattr(message, "producer", "") or ""),
        }

    @staticmethod
    def _planner_contract_command(message):
        command = Twist()
        source = getattr(message, "command", None)
        if source is not None:
            command.linear.x = float(source.linear.x)
            command.angular.z = float(source.angular.z)
        return command

    def _planner_contract_alignment_pending_locked(self, identity):
        """Return whether a newer frame is waiting for Bridge status."""
        if not self.active_action:
            return True
        if self._lifecycle_transaction_id_locked() <= 0:
            return True
        has_future_field = False
        for field, active_field in (
            ("transaction_id", "active_action_goal_transaction_id"),
            ("route_id", "active_action_route_id"),
            ("graph_transaction_id", "active_action_graph_transaction_id"),
            ("action_generation", "active_action_generation"),
            ("origin_epoch", "active_action_origin_epoch"),
            ("projection_epoch", "active_action_projection_epoch"),
        ):
            active_value = int(getattr(self, active_field, 0) or 0)
            received_value = int(identity[field] or 0)
            # Direct target routes are valid without a Global Frontier graph
            # route.  When both sides carry graph=0 there is no graph
            # identity to align; non-zero graph identities remain strict.
            if field == "graph_transaction_id" and active_value <= 0 and received_value <= 0:
                continue
            if active_value <= 0:
                return True
            if received_value > active_value:
                return True
        active_epoch = getattr(self, "active_action_map_epoch", None)
        received_epoch = int(identity.get("map_epoch", 0) or 0)
        if active_epoch is None:
            return received_epoch > 0
        try:
            if received_epoch > int(active_epoch):
                return True
        except (TypeError, ValueError):
            return True
        return False

    def _mux_status_identity_matches_action_locked(self, payload):
        """Check that an actuator result belongs to the current route owner."""
        if not isinstance(payload, dict) or not bool(payload.get("active")):
            return False
        identity = payload.get("command_identity")
        if not isinstance(identity, dict):
            return False

        def integer(value):
            try:
                return int(value or 0)
            except (TypeError, ValueError):
                return 0

        expected_route = integer(getattr(self, "active_action_route_id", 0))
        expected_generation = integer(
            getattr(self, "active_action_generation", 0)
        )
        expected_transaction = integer(
            getattr(self, "active_action_transaction_id", 0)
        )
        expected_graph = integer(
            getattr(self, "active_action_graph_transaction_id", 0)
        )
        expected_map = getattr(self, "active_action_map_epoch", None)
        if expected_route <= 0 or expected_generation <= 0:
            return False

        incoming_route = integer(
            identity.get("active_action_route_id", identity.get("route_id"))
        )
        incoming_generation = integer(
            identity.get(
                "active_action_generation", identity.get("planner_action_generation")
            )
        )
        incoming_transaction = integer(
            identity.get(
                "active_action_transaction_id", identity.get("transaction_id")
            )
        )
        incoming_graph = integer(
            identity.get(
                "active_action_graph_transaction_id",
                identity.get("graph_transaction_id"),
            )
        )
        incoming_map = identity.get(
            "active_action_map_epoch", identity.get("map_epoch")
        )
        if incoming_route != expected_route or incoming_generation != expected_generation:
            return False
        if expected_transaction > 0 and incoming_transaction != expected_transaction:
            return False
        if expected_graph > 0 and incoming_graph > 0 and incoming_graph != expected_graph:
            return False
        if expected_map is not None and incoming_map is not None:
            try:
                if int(incoming_map) != int(expected_map):
                    return False
            except (TypeError, ValueError):
                return False
        return True

    def _current_forward_only_clamp_locked(self, action_identity=None):
        """Return a current mux clamp that makes this route non-executable."""
        status = getattr(self, "latest_mux_status", None)
        if not isinstance(status, dict):
            return None
        now = now_for(self)
        status_wall = float(getattr(self, "latest_mux_status_wall", 0.0) or 0.0)
        if status_wall <= 0.0 or now - status_wall > 0.50:
            return None
        if not self._mux_status_identity_matches_action_locked(status):
            return None
        if str(status.get("filter_reason", "") or "").strip().lower() != (
            "forward_only_reverse_clamp"
        ):
            return None
        input_command = status.get("input")
        output_command = status.get("output")
        if not isinstance(input_command, dict) or not isinstance(output_command, dict):
            return None
        try:
            input_linear = float(input_command.get("linear_x", 0.0) or 0.0)
            output_linear = float(output_command.get("linear_x", 0.0) or 0.0)
            output_angular = float(output_command.get("angular_z", 0.0) or 0.0)
        except (TypeError, ValueError):
            return None
        if input_linear >= -0.002 or abs(output_linear) > 0.001:
            return None
        if action_identity is not None:
            expected_identity = getattr(self, "active_action_identity", None)
            if expected_identity is not None and action_identity != expected_identity:
                return None
        return {
            "reason": "forward_only_reverse_clamp",
            "input_linear": round(input_linear, 4),
            "output_linear": round(output_linear, 4),
            "output_angular": round(output_angular, 4),
            "status_sequence": int(status.get("sequence", 0) or 0),
        }

    def on_cmd_vel_mux_status(self, message):
        """Record the final actuator result for the timer-owned turn FSM."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        with self.lock:
            self.latest_mux_status = payload
            self.latest_mux_status_wall = now_for(self)
            clamp = self._current_forward_only_clamp_locked(
                getattr(self, "active_action_identity", None)
            )
            if clamp is None:
                return
            identity = payload.get("command_identity") or {}
            boundary_key = (
                int(identity.get("active_action_route_id", 0) or 0),
                int(identity.get("active_action_generation", 0) or 0),
            )
            if boundary_key == getattr(self, "latest_mux_boundary_key", None):
                return
            self.latest_mux_boundary_key = boundary_key
            self.publish_status_locked(
                "actuator_command_boundary",
                reason=clamp["reason"],
                input_linear=clamp["input_linear"],
                output_linear=clamp["output_linear"],
                output_angular=clamp["output_angular"],
                mux_status_sequence=clamp["status_sequence"],
                command_identity=dict(identity),
            )

    @staticmethod
    def _planner_contract_order_key(identity):
        """Return a monotonic key for coalescing pre-adoption contracts."""
        return tuple(
            int(identity.get(field, 0) or 0)
            for field in (
                "lifecycle_transaction_id",
                "route_id",
                "graph_transaction_id",
                "action_generation",
                "transaction_id",
                "planner_sequence",
            )
        )

    def _queue_planner_contract_pending_locked(
        self, identity, command, state, reason, producer
    ):
        """Retain a future contract until Bridge publishes action adoption."""
        previous = getattr(self, "planner_contract_pending", None)
        incoming_key = self._planner_contract_order_key(identity)
        if isinstance(previous, dict):
            previous_identity = previous.get("identity", {})
            previous_key = self._planner_contract_order_key(previous_identity)
            if incoming_key < previous_key:
                self.publish_status_locked(
                    "planner_contract_pending_ignored",
                    reason="older_pending_contract",
                    planner_contract_identity=identity,
                )
                return False
            if (
                incoming_key == previous_key
                and int(identity.get("planner_sequence", 0) or 0)
                <= int(previous_identity.get("planner_sequence", 0) or 0)
            ):
                self.publish_status_locked(
                    "planner_contract_pending_ignored",
                    reason="duplicate_pending_contract",
                    planner_contract_identity=identity,
                )
                return False
        self.planner_contract_pending = {
            "identity": dict(identity),
            "command": copy.deepcopy(command),
            "state": int(state),
            "reason": str(reason or ""),
            "producer": str(producer or ""),
            "received_wall": now_for(self),
        }
        self.publish_status_locked(
            "planner_contract_pending",
            reason="bridge_action_alignment_pending",
            planner_contract_identity=dict(identity),
            pending_age_seconds=0.0,
        )
        return True

    def _hold_for_pending_planner_contract_locked(self):
        """Stop forwarding the predecessor while a future owner aligns."""
        self.planner_contract_valid = False
        self.planner_contract_state = PlannerCommandContract.STATE_INVALIDATED
        self.planner_contract_reason = "bridge_action_alignment_pending"
        self._invalidate_trajectory_feedback_locked(
            "bridge_action_alignment_pending", clear_planner=True
        )

    def _planner_contract_mismatch_locked(self, identity):
        current_lifecycle = self._lifecycle_transaction_id_locked()
        if identity["lifecycle_transaction_id"] <= 0:
            return "lifecycle_transaction_id_missing"
        if (
            current_lifecycle <= 0
            or identity["lifecycle_transaction_id"] != current_lifecycle
        ):
            return "lifecycle_transaction_id_mismatch"
        if identity["action_generation"] <= 0:
            return "action_generation_missing"
        if not self.active_action:
            return "bridge_action_inactive"
        for field, active_field in (
            ("transaction_id", "active_action_goal_transaction_id"),
            ("route_id", "active_action_route_id"),
            ("graph_transaction_id", "active_action_graph_transaction_id"),
            ("action_generation", "active_action_generation"),
            ("origin_epoch", "active_action_origin_epoch"),
            ("projection_epoch", "active_action_projection_epoch"),
        ):
            active_value = int(getattr(self, active_field, 0) or 0)
            if active_value > 0 and identity[field] != active_value:
                return "planner_%s_mismatch" % field
        active_epoch = getattr(self, "active_action_map_epoch", None)
        if active_epoch is not None and identity["map_epoch"] != int(active_epoch):
            return "planner_map_epoch_mismatch"
        if not identity["producer"]:
            return "planner_producer_missing"
        return ""

    def _store_planner_contract_locked(
        self, identity, command, state, reason, producer
    ):
        previous_identity = getattr(self, "planner_contract_identity", None)
        previous_valid = bool(
            getattr(self, "planner_contract_valid", False)
        )
        previous_state = getattr(self, "planner_contract_state", None)
        identity_fields = (
            "transaction_id",
            "route_id",
            "graph_transaction_id",
            "map_epoch",
            "origin_epoch",
            "projection_epoch",
            "lifecycle_transaction_id",
            "action_generation",
        )
        identity_changed = not isinstance(previous_identity, dict) or any(
            int(identity.get(field, 0) or 0)
            != int(previous_identity.get(field, 0) or 0)
            for field in identity_fields
        )
        contract_boundary = bool(
            identity_changed
            or not previous_valid
            or previous_state != PlannerCommandContract.STATE_ACTIVE
        )
        self.planner_contract_valid = state == PlannerCommandContract.STATE_ACTIVE
        self.planner_contract_state = int(state)
        self.planner_contract_reason = str(reason or "")
        self.planner_contract_identity = dict(identity)
        self.planner_contract_sequence = int(identity["planner_sequence"])
        self.planner_contract_wall = now_for(self)
        self.planner_contract_command = copy.deepcopy(command)
        if contract_boundary:
            # FeedbackMsg has no wire identity and is much slower than the
            # command-contract stream.  A new route/state must re-arm it, but
            # a command sample within the same authenticated route must not
            # invalidate the latest usable trajectory on every callback.
            self.planner_contract_awaiting_feedback = bool(
                self.planner_contract_valid
            )
        self.latest_planner_command = copy.deepcopy(command)
        self.latest_planner_command_wall = self.planner_contract_wall
        self.planner_command_transaction_id = int(
            identity["lifecycle_transaction_id"]
        )
        self.planner_command_route_id = int(identity["route_id"])
        self.planner_command_map_epoch = int(identity["map_epoch"])
        self.planner_command_origin_epoch = int(identity["origin_epoch"])
        self.planner_command_projection_epoch = int(
            identity["projection_epoch"]
        )
        self.planner_command_graph_transaction_id = int(
            identity["graph_transaction_id"]
        )
        self.planner_command_action_generation = int(
            identity["action_generation"]
        )
        self.planner_command_action_identity = self.active_action_identity
        if self.planner_contract_valid and contract_boundary:
            self._invalidate_trajectory_feedback_locked(
                "planner_contract_identity_changed"
                if identity_changed
                else "planner_contract_boundary",
                clear_planner=False,
            )
        else:
            if not self.planner_contract_valid:
                self._invalidate_trajectory_feedback_locked(
                    "planner_contract_%s" % (reason or "boundary"),
                    clear_planner=True,
                )

    def _apply_planner_contract_locked(
        self, identity, command, state, reason, producer
    ):
        """Validate and install one planner contract after bridge alignment."""
        if state in (
            PlannerCommandContract.STATE_ZERO,
            PlannerCommandContract.STATE_INVALIDATED,
        ):
            current_lifecycle = self._lifecycle_transaction_id_locked()
            if (
                identity["lifecycle_transaction_id"] <= 0
                or (
                    current_lifecycle <= 0
                    or (
                        current_lifecycle > 0
                    and identity["lifecycle_transaction_id"] != current_lifecycle
                    )
                )
            ):
                self.publish_status_locked(
                    "planner_contract_rejected",
                    reason="stale_boundary_lifecycle",
                    planner_contract_identity=identity,
                )
                return False
            self.planner_contract_pending = None
            self._store_planner_contract_locked(
                identity, command, state, reason, producer
            )
            self.publish_status_locked(
                "planner_contract_boundary",
                planner_contract_identity=identity,
                reason=str(reason or "planner_boundary"),
            )
            return True

        missing_identity = [
            field
            for field in (
                "transaction_id",
                "route_id",
                "lifecycle_transaction_id",
                "action_generation",
                "map_epoch",
                "origin_epoch",
                "projection_epoch",
            )
            if int(identity.get(field, 0) or 0) <= 0
        ]
        if missing_identity:
            self._reject_planner_contract_locked(
                "missing_identity_%s" % missing_identity[0],
                identity,
            )
            return False
        mismatch = self._planner_contract_mismatch_locked(identity)
        if mismatch:
            self._reject_planner_contract_locked(mismatch, identity)
            return False
        previous = getattr(self, "planner_contract_identity", None)
        if (
            isinstance(previous, dict)
            and identity["planner_sequence"] <= int(
                previous.get("planner_sequence", 0) or 0
            )
        ):
            self._reject_planner_contract_locked(
                "stale_planner_sequence", identity
            )
            return False
        self.planner_contract_pending = None
        self._store_planner_contract_locked(
            identity, command, state, reason, producer
        )
        self.publish_status_locked(
            "planner_command_authorized",
            planner_contract_identity=identity,
            linear_x=round(float(command.linear.x), 6),
            angular_z=round(float(command.angular.z), 6),
        )
        return True

    def _reject_planner_contract_locked(self, reason, identity):
        """Invalidate the supervisor output until a fresh identity arrives."""
        self.planner_contract_pending = None
        self.planner_contract_valid = False
        self.planner_contract_state = PlannerCommandContract.STATE_INVALIDATED
        self.planner_contract_reason = str(reason or "identity_mismatch")
        self.planner_contract_identity = dict(identity or {})
        self.planner_contract_sequence = int(
            identity.get("planner_sequence", 0) or 0
        )
        self._invalidate_trajectory_feedback_locked(
            "identity_mismatch", clear_planner=True
        )
        self.publish_status_locked(
            "planner_contract_rejected",
            reason=str(reason or "identity_mismatch"),
            identity_mismatch=True,
            planner_contract_identity=dict(identity or {}),
        )

    def on_planner_command_contract(self, message):
        """Consume the identity-bearing planner stream in persistent mode."""
        if not getattr(self, "require_planner_command_contract", False):
            return
        identity = self._planner_contract_identity(message)
        command = self._planner_contract_command(message)
        reason = str(getattr(message, "reason", "") or "")
        with self.lock:
            # The persistent planner publishes a ZERO/INVALIDATED boundary as
            # soon as it receives a newer mission command.  That wire frame is
            # a future route contract, not a stale command, while the Bridge
            # still reports the predecessor action as active.  Queue all
            # future states until the matching Bridge action status arrives;
            # otherwise a valid successor boundary is rejected by comparing
            # its lifecycle with the predecessor's lifecycle.
            if self._planner_contract_alignment_pending_locked(identity):
                self._queue_planner_contract_pending_locked(
                    identity,
                    command,
                    identity["state"],
                    reason,
                    identity["producer"],
                )
                self._hold_for_pending_planner_contract_locked()
                return
            accepted = self._apply_planner_contract_locked(
                identity,
                command,
                identity["state"],
                reason,
                identity["producer"],
            )
            if accepted and identity["state"] == PlannerCommandContract.STATE_ACTIVE:
                self._clear_recovery_execution_locked("active_planner_contract")

    def _promote_pending_planner_contract_locked(self):
        pending = getattr(self, "planner_contract_pending", None)
        if not isinstance(pending, dict):
            return False
        pending_identity = pending.get("identity")
        if self._pending_planner_contract_is_stale_locked(pending_identity):
            self.planner_contract_pending = None
            self.publish_status_locked(
                "planner_contract_pending_ignored",
                reason="pending_contract_older_than_active_action",
                planner_contract_identity=dict(pending_identity or {}),
            )
            return False
        # Do not turn an in-flight Bridge status update into a hard rejection.
        # The pending frame is promoted on the next matching active status.
        if self._planner_contract_alignment_pending_locked(
            pending_identity
        ):
            return False
        accepted = self._apply_planner_contract_locked(
            pending_identity,
            pending["command"],
            pending["state"],
            pending["reason"],
            pending["producer"],
        )
        return bool(accepted)

    def _pending_planner_contract_is_stale_locked(self, identity):
        """Reject an old boundary that arrived after successor adoption.

        Planner contracts and Bridge status are separate ROS streams.  A
        predecessor ZERO/INVALIDATED frame can therefore be queued while the
        Bridge is inactive and only be promoted after a newer persistent
        route is already active.  It must not be allowed to repeatedly run
        the lifecycle-mismatch path or alter the successor's output state.
        Future contracts remain pending until their matching Bridge status.
        """
        if not self.active_action or not isinstance(identity, dict):
            return False
        try:
            incoming_lifecycle = int(
                identity.get("lifecycle_transaction_id", 0) or 0
            )
            active_lifecycle = int(
                getattr(self, "active_action_lifecycle_transaction_id", 0)
                or getattr(self, "active_action_transaction_id", 0)
                or 0
            )
        except (TypeError, ValueError):
            return False
        if (
            incoming_lifecycle <= 0
            or active_lifecycle <= 0
            or incoming_lifecycle == active_lifecycle
        ):
            return False

        # Lifecycle IDs are opaque UUID-like values, so classify ordering by
        # the monotonic execution identity instead of comparing their integer
        # values.  A future route must stay queued; an older or otherwise
        # equal-shaped contract is stale once this action is active.
        has_future_field = False
        for field, active_field in (
            ("route_id", "active_action_route_id"),
            ("transaction_id", "active_action_goal_transaction_id"),
            ("graph_transaction_id", "active_action_graph_transaction_id"),
            ("action_generation", "active_action_generation"),
        ):
            try:
                incoming = int(identity.get(field, 0) or 0)
                active = int(getattr(self, active_field, 0) or 0)
            except (TypeError, ValueError):
                continue
            if active <= 0 or incoming <= 0:
                continue
            if incoming < active:
                return True
            if incoming > active:
                has_future_field = True
        return not has_future_field

    def _invalidate_trajectory_feedback_locked(
        self, reason, clear_planner=False
    ):
        """Drop identity-free TEB feedback at a control ownership boundary."""
        reason = str(reason or "control_pipeline_boundary")
        previous_command = self.latest_trajectory_command
        previous_selected = [
            round(float(previous_command.linear.x), 4),
            round(float(previous_command.angular.z), 4),
        ]
        previous_valid = bool(getattr(self, "trajectory_feedback_valid", False))
        previous_reason = str(
            getattr(self, "trajectory_feedback_invalid_reason", "") or ""
        )
        previous_required = bool(
            getattr(
                self,
                "trajectory_feedback_requires_fresh_planner_command",
                False,
            )
        )
        changed = bool(
            previous_valid
            or abs(previous_command.linear.x) > 0.001
            or abs(previous_command.angular.z) > 0.01
            or previous_reason != reason
            or not previous_required
            or clear_planner
        )
        self.latest_trajectory_command = Twist()
        self.latest_trajectory_command_wall = 0.0
        self.trajectory_feedback_valid = False
        self.trajectory_feedback_invalid_reason = reason
        self.trajectory_feedback_requires_fresh_planner_command = True
        self.trajectory_feedback_identity = None
        self.trajectory_zero_started_wall = 0.0
        self.trajectory_continuity_goal_distance = None
        self.trajectory_feedback_invalidation_count = int(
            getattr(self, "trajectory_feedback_invalidation_count", 0) or 0
        ) + 1
        if clear_planner:
            self.latest_planner_command = Twist()
            self.latest_planner_command_wall = 0.0
            self.planner_command_transaction_id = 0
            self.planner_command_route_id = 0
            self.planner_command_map_epoch = None
            self.planner_command_origin_epoch = None
            self.planner_command_projection_epoch = None
            self.planner_command_graph_transaction_id = 0
            self.planner_command_action_identity = None
            # A bridge status can race the typed planner stream. If the
            # current contract was already admitted, keep the next feedback
            # eligible until the next frame arrives instead of waiting for an
            # identity change that may already have been processed.
            if bool(getattr(self, "planner_contract_valid", False)):
                self.planner_contract_awaiting_feedback = True
        if changed:
            self.publish_status_locked(
                "trajectory_feedback_invalidated",
                reason=reason,
                clear_planner=bool(clear_planner),
                previous_selected_command=previous_selected,
                previous_valid=previous_valid,
                invalidation_count=int(
                    self.trajectory_feedback_invalidation_count
                ),
            )

    def on_intent(self, message):
        source = "unknown"
        priority = 0
        route_kind = ""
        intent_goal = None
        target_track_id = ""
        route_id = 0
        map_epoch = None
        origin_epoch = None
        projection_epoch = None
        graph_transaction_id = 0
        try:
            payload = json.loads(message.data)
            if isinstance(payload, dict):
                source = str(payload.get("source", source)).strip().lower() or source
                priority = int(payload.get("priority", priority))
                route_kind = str(payload.get("route_kind", "")).strip().lower()
                route_id = max(0, int(payload.get("route_id", 0) or 0))
                raw_map_epoch = payload.get("map_epoch")
                map_epoch = (
                    None
                    if raw_map_epoch is None
                    else max(0, int(raw_map_epoch or 0))
                )
                raw_origin_epoch = payload.get("origin_epoch", map_epoch)
                raw_projection_epoch = payload.get(
                    "projection_epoch", map_epoch
                )
                origin_epoch = (
                    None
                    if raw_origin_epoch is None
                    else max(0, int(raw_origin_epoch or 0))
                )
                projection_epoch = (
                    None
                    if raw_projection_epoch is None
                    else max(0, int(raw_projection_epoch or 0))
                )
                graph_transaction_id = max(
                    0, int(payload.get("graph_transaction_id", 0) or 0)
                )
                target_track_id = str(
                    payload.get("target_track_id", "")
                ).strip()
                raw_goal = payload.get("goal")
                if isinstance(raw_goal, (list, tuple)) and len(raw_goal) >= 2:
                    intent_goal = (float(raw_goal[0]), float(raw_goal[1]))
            else:
                source = str(message.data).strip().lower() or source
        except (TypeError, ValueError, json.JSONDecodeError):
            source = str(message.data).strip().lower() or source
        with self.lock:
            self.latest_intent_source = source
            self.latest_intent_priority = max(0, min(3, priority))
            self.latest_route_kind = route_kind
            self.latest_route_id = route_id
            self.latest_map_epoch = map_epoch
            self.latest_origin_epoch = origin_epoch
            self.latest_projection_epoch = projection_epoch
            self.latest_graph_transaction_id = graph_transaction_id
            self.latest_intent_goal = intent_goal
            self.latest_target_track_id = target_track_id
            endpoint_intent = (
                route_kind == FRONTIER_ENDPOINT_KIND
                and source == FRONTIER_SOURCE
            )
            portal_intent = (
                route_kind == PORTAL_TRANSITION_KIND
                and source == FRONTIER_SOURCE
            )
            local_egress_intent = (
                route_kind == LOCAL_EGRESS_KIND
                and source == FRONTIER_SOURCE
            )
            portal_probe_intent = (
                route_kind == PORTAL_PROBE_KIND
                and source == FRONTIER_SOURCE
            )
            if (
                route_kind != TURN_ROUTE_KIND
                and not endpoint_intent
                and not portal_intent
                and not local_egress_intent
                and not portal_probe_intent
            ):
                self._release_turn_locked("route_contract_released", completed=False)
            else:
                # Intent describes a pending mission decision. Bridge status
                # remains the authority that makes an action executable.
                self._activate_turn_locked()

    def on_bridge_status(self, message):
        """Synchronize execution state with the bridge's active action."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        with self.lock:
            previous_identity = self.active_action_identity
            self.active_action = bool(payload.get("active", False))
            self.active_action_route_kind = str(
                payload.get("active_route_kind", "")
            ).strip().lower()
            self.active_action_source = str(
                payload.get("active_intent_source", "unknown")
            ).strip().lower() or "unknown"
            self.active_action_priority = max(
                0, min(3, int(payload.get("active_intent_priority", 0) or 0))
            )
            try:
                self.active_action_goal_transaction_id = int(
                    payload.get("active_goal_transaction_id", 0) or 0
                )
            except (TypeError, ValueError):
                self.active_action_goal_transaction_id = 0
            try:
                self.active_action_route_id = max(
                    0,
                    int(
                        payload.get(
                            "active_route_id", payload.get("route_id", 0)
                        )
                        or 0
                    ),
                )
            except (TypeError, ValueError):
                self.active_action_route_id = 0
            raw_active_epoch = payload.get(
                "active_route_map_epoch",
                payload.get("map_epoch", payload.get("latest_route_map_epoch")),
            )
            try:
                self.active_action_map_epoch = (
                    None
                    if raw_active_epoch is None
                    else max(0, int(raw_active_epoch or 0))
                )
            except (TypeError, ValueError):
                self.active_action_map_epoch = None
            raw_active_origin_epoch = payload.get(
                "active_origin_epoch",
                payload.get("origin_epoch", payload.get("latest_origin_epoch")),
            )
            raw_active_projection_epoch = payload.get(
                "active_projection_epoch",
                payload.get(
                    "projection_epoch", payload.get("latest_projection_epoch")
                ),
            )
            try:
                self.active_action_origin_epoch = (
                    None
                    if raw_active_origin_epoch is None
                    else max(0, int(raw_active_origin_epoch or 0))
                )
            except (TypeError, ValueError):
                self.active_action_origin_epoch = None
            try:
                self.active_action_projection_epoch = (
                    None
                    if raw_active_projection_epoch is None
                    else max(0, int(raw_active_projection_epoch or 0))
                )
            except (TypeError, ValueError):
                self.active_action_projection_epoch = None
            try:
                self.active_action_graph_transaction_id = max(
                    0,
                    int(
                        payload.get(
                            "active_graph_transaction_id",
                            payload.get("graph_transaction_id", 0),
                        )
                        or 0
                    ),
                )
            except (TypeError, ValueError):
                self.active_action_graph_transaction_id = 0
            self.active_action_target_track_id = str(
                payload.get("active_target_track_id", "")
            ).strip()
            try:
                self.active_action_generation = int(
                    payload.get("active_action_generation", 0) or 0
                )
            except (TypeError, ValueError):
                self.active_action_generation = 0
            try:
                self.active_action_lifecycle_transaction_id = int(
                    payload.get(
                        "active_action_lifecycle_transaction_id",
                        payload.get("lifecycle_transaction_id", 0),
                    )
                    or 0
                )
            except (TypeError, ValueError):
                self.active_action_lifecycle_transaction_id = 0
            # Keep the historical field name for compatibility with the
            # command/reporting surface.  Its value is the active action's
            # lifecycle identity, never the newer status-event identity.
            self.active_action_transaction_id = int(
                getattr(self, "active_action_lifecycle_transaction_id", 0)
                or 0
            )
            self.latest_target_track_id = str(
                payload.get("latest_target_track_id", self.latest_target_track_id)
            ).strip()
            raw_goal = payload.get("active_goal")
            frame = str(payload.get("active_goal_frame", "") or "map").strip()
            if (
                isinstance(raw_goal, (list, tuple))
                and len(raw_goal) >= 2
                and self.active_action
            ):
                active_goal = PoseStamped()
                active_goal.header.stamp = rospy.Time.now()
                active_goal.header.frame_id = frame.lstrip("/") or "map"
                active_goal.pose.position.x = float(raw_goal[0])
                active_goal.pose.position.y = float(raw_goal[1])
                yaw = float(raw_goal[2]) if len(raw_goal) >= 3 else 0.0
                active_goal.pose.orientation.z = math.sin(0.5 * yaw)
                active_goal.pose.orientation.w = math.cos(0.5 * yaw)
                self.active_action_goal = active_goal
                raw_source_goal = payload.get("active_source_goal")
                source_frame = str(
                    payload.get("active_source_goal_frame", frame) or frame
                ).strip().lstrip("/") or "map"
                if (
                    isinstance(raw_source_goal, (list, tuple))
                    and len(raw_source_goal) >= 2
                ):
                    source_goal = PoseStamped()
                    source_goal.header.stamp = rospy.Time.now()
                    source_goal.header.frame_id = source_frame
                    source_goal.pose.position.x = float(raw_source_goal[0])
                    source_goal.pose.position.y = float(raw_source_goal[1])
                    source_yaw = (
                        float(raw_source_goal[2])
                        if len(raw_source_goal) >= 3 else 0.0
                    )
                    source_goal.pose.orientation.z = math.sin(0.5 * source_yaw)
                    source_goal.pose.orientation.w = math.cos(0.5 * source_yaw)
                    self.active_action_source_goal = source_goal
                else:
                    self.active_action_source_goal = None
            else:
                self.active_action_goal = None
                self.active_action_source_goal = None
            self.active_action_identity = self._active_action_key_locked()
            if self.active_action_identity != previous_identity:
                if (
                    previous_identity is not None
                    and self.state == STATE_TURNING
                    and self.turn_action_identity != self.active_action_identity
                ):
                    self._release_turn_locked(
                        "bridge_action_identity_changed",
                        completed=False,
                    )
                self.pre_turn_checked_identity = None
                self.completed_turn_key = None
                self.latest_navfn_plan = None
                self.trajectory_continuity_sharp_entry_identity = None
                self.stalled_route_candidate_identity = None
                self.stalled_route_candidate_since_wall = 0.0
                self.stalled_route_ready_identity = None
                self.stalled_route_completed_identity = None
                self.stalled_route_reorientation_attempted_identity = None
                self.forward_only_trajectory_identity = None
                self.latest_mux_boundary_key = None
                self._invalidate_trajectory_feedback_locked(
                    "route_owner_changed", clear_planner=True
                )
            elif not self.active_action:
                self._invalidate_trajectory_feedback_locked(
                    "bridge_inactive", clear_planner=True
                )
            elif payload.get("teb_feedback_valid") is False:
                invalid_reason = str(
                    payload.get(
                        "teb_feedback_invalid_reason",
                        "bridge_feedback_invalidated",
                    )
                    or "bridge_feedback_invalidated"
                ).strip().lower()
                # ``awaiting_teb_feedback`` and ``no_feedback`` are normal
                # post-contract observations. They must not repeatedly erase
                # the supervisor's usable trajectory while the bridge waits
                # for the first sample of the current contract.
                boundary = bool(payload.get("clear_planner", False)) or (
                    invalid_reason in self._BRIDGE_FEEDBACK_BOUNDARY_REASONS
                )
                if boundary:
                    self._invalidate_trajectory_feedback_locked(
                        invalid_reason,
                        clear_planner=bool(payload.get("clear_planner", False)),
                    )
                    if bool(getattr(self, "planner_contract_valid", False)):
                        # The bridge may report a semantic route boundary
                        # after the matching planner contract was already
                        # consumed. Re-arm the first post-boundary feedback
                        # explicitly so callback ordering cannot strand the
                        # supervisor in the old feedback epoch.
                        self.planner_contract_awaiting_feedback = True
            self._promote_pending_planner_contract_locked()
            if not self._is_managed_action_locked():
                if self.state == STATE_TURNING:
                    self._release_turn_locked("bridge_action_changed", completed=False)
            else:
                self._activate_turn_locked()

    def on_goal(self, message):
        goal = copy.deepcopy(message)
        if not goal.header.frame_id:
            goal.header.frame_id = "odom"
        with self.lock:
            self.latest_goal = goal
            if self._is_managed_action_locked():
                activated = self._activate_turn_locked()
                if (
                    not activated
                    and self.state == STATE_TURNING
                    and not self._intent_matches_goal_locked(goal)
                ):
                    # A different frontier pose is a queued next segment, not
                    # permission to interrupt the current turn.
                    pass
            elif self.state == STATE_TURNING:
                self._release_turn_locked("goal_route_changed", completed=False)

    def on_navfn_plan(self, message):
        """Cache the active global path for one-time endpoint alignment."""
        with self.lock:
            self.latest_navfn_plan = copy.deepcopy(message)
            if (
                self._is_frontier_endpoint_action_locked()
                or self._is_portal_transition_action_locked()
            ):
                self._activate_turn_locked()

    def on_pose(self, message):
        now = now_for(self)
        with self.lock:
            self.pose = copy.deepcopy(message)
            observe_turn = getattr(
                self, "_observe_turn_odom_pose_locked", None
            )
            if callable(observe_turn):
                observe_turn(self.pose, now)

    def on_scan(self, message):
        minimum = float("inf")
        for value in message.ranges:
            if math.isfinite(value) and value > 0.01:
                minimum = min(minimum, float(value))
        with self.lock:
            self.scan_minimum = minimum
            self.scan_monotonic = now_for(self)

    def on_planner_command(self, message):
        if getattr(self, "require_planner_command_contract", False):
            with self.lock:
                if (
                    getattr(self, "recovery_active", False)
                    and getattr(self, "recovery_identity", None)
                    == getattr(self, "active_action_identity", None)
                ):
                    self.latest_recovery_command = copy.deepcopy(message)
                    self.latest_recovery_command_wall = now_for(self)
                    return
                self.publish_status_locked(
                    "planner_raw_command_ignored",
                    reason="typed_planner_contract_required",
                    raw_command=[
                        round(float(message.linear.x), 4),
                        round(float(message.angular.z), 4),
                    ],
                )
            return
        with self.lock:
            self.latest_planner_command = copy.deepcopy(message)
            self.latest_planner_command_wall = now_for(self)
            self.planner_command_sequence += 1
            self.planner_command_transaction_id = int(
                getattr(
                    getattr(self, "lifecycle_manager", None),
                    "current_transaction_id",
                    0,
                )
                or 0
            )
            self.planner_command_route_id = int(
                getattr(self, "active_action_route_id", 0) or 0
            )
            self.planner_command_map_epoch = getattr(
                self, "active_action_map_epoch", None
            )
            self.planner_command_origin_epoch = getattr(
                self, "active_action_origin_epoch", None
            )
            self.planner_command_projection_epoch = getattr(
                self, "active_action_projection_epoch", None
            )
            self.planner_command_graph_transaction_id = int(
                getattr(self, "active_action_graph_transaction_id", 0) or 0
            )
            self.planner_command_action_generation = int(
                getattr(self, "active_action_generation", 0) or 0
            )
            self.planner_contract_sequence = int(
                getattr(self, "planner_command_sequence", 0) or 0
            )
            self.planner_command_action_identity = self.active_action_identity
            if (
                abs(float(message.linear.x)) <= 0.001
                and abs(float(message.angular.z)) <= 0.01
            ):
                self._invalidate_trajectory_feedback_locked(
                    "planner_zero_command"
                )

    def on_teb_feedback(self, message):
        """Cache the selected TEB velocity for a bounded raw-command gap."""
        selected_index = int(message.selected_trajectory_idx)
        trajectories = list(message.trajectories)
        if selected_index < 0 or selected_index >= len(trajectories):
            return
        points = list(trajectories[selected_index].trajectory)
        if not points:
            return
        velocity = points[0].velocity
        command = Twist()
        command.linear.x = float(velocity.linear.x)
        command.angular.z = float(velocity.angular.z)
        with self.lock:
            now = now_for(self)
            if not self.active_action:
                self._invalidate_trajectory_feedback_locked("no_active_action")
                return
            if getattr(self, "require_planner_command_contract", False):
                if not bool(getattr(self, "planner_contract_valid", False)):
                    self.publish_status_locked(
                        "trajectory_feedback_rejected",
                        reason="planner_contract_missing",
                    )
                    return
                if self.planner_contract_awaiting_feedback:
                    # FeedbackMsg exposes TEB's selected trajectory point,
                    # not the post-processed command in the planner contract.
                    # Treating their velocities as an equality check rejects
                    # valid feedback during normal angular interpolation. The
                    # identity-bearing contract is the boundary; once it is
                    # accepted, this first feedback frame may re-arm the
                    # telemetry stream.
                    self.planner_contract_awaiting_feedback = False
                    self.trajectory_feedback_requires_fresh_planner_command = False
            if self.trajectory_feedback_requires_fresh_planner_command:
                # This may be a delayed sample from the previous route. It has
                # no wire-level identity and cannot re-arm the control clock.
                return
            if self.latest_trajectory_command_wall > 0.0:
                period = now - self.latest_trajectory_command_wall
                if 0.005 <= period <= self.trajectory_feedback_timeout_cap:
                    if self.trajectory_feedback_period_ema is None:
                        self.trajectory_feedback_period_ema = period
                    else:
                        self.trajectory_feedback_period_ema = (
                            0.75 * self.trajectory_feedback_period_ema
                            + 0.25 * period
                        )
            self.latest_trajectory_command = command
            self.latest_trajectory_command_wall = now
            self.trajectory_feedback_valid = True
            self.trajectory_feedback_invalid_reason = ""
            self.trajectory_feedback_requires_fresh_planner_command = False
            self.trajectory_feedback_identity = self.active_action_identity
