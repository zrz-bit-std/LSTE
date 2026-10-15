"""Live TEB execution observations used by the action-health policy."""

import copy
import json
import math

import rospy
from geometry_msgs.msg import Twist
from lste_topo_access.msg import PlannerCommandContract

from clock_provider import now_for


class TebGoalBridgeTebRuntimeMixin:
    _TEB_ZERO_LINEAR_EPSILON = 0.001
    _TEB_ZERO_ANGULAR_EPSILON = 0.01
    _TEB_TERMINAL_BOUNDARY_REASONS = frozenset(
        {
            "persistent_endpoint_terminal",
            "execution_terminal",
            "action_lease_cleared",
            "controller_lease_released",
            "hard_reset",
            "new_action_dispatch",
            "route_owner_changed",
            "bridge_inactive",
            "terminal_prepare",
        }
    )

    @classmethod
    def _teb_planner_command_is_zero(cls, linear, angular):
        """Return whether the planner emitted a true stop command."""
        return (
            abs(float(linear)) <= cls._TEB_ZERO_LINEAR_EPSILON
            and abs(float(angular)) <= cls._TEB_ZERO_ANGULAR_EPSILON
        )

    @staticmethod
    def _contract_int(value, default=0):
        try:
            return int(value or 0)
        except (TypeError, ValueError):
            return int(default)

    def _planner_contract_identity(self, message):
        """Decode the identity carried by one persistent planner frame."""
        return {
            "transaction_id": self._contract_int(
                getattr(message, "transaction_id", 0)
            ),
            "route_id": self._contract_int(getattr(message, "route_id", 0)),
            "graph_transaction_id": self._contract_int(
                getattr(message, "graph_transaction_id", 0)
            ),
            "map_epoch": self._contract_int(getattr(message, "map_epoch", 0)),
            "origin_epoch": self._contract_int(
                getattr(message, "origin_epoch", 0)
            ),
            "projection_epoch": self._contract_int(
                getattr(message, "projection_epoch", 0)
            ),
            "lifecycle_transaction_id": self._contract_int(
                getattr(message, "lifecycle_transaction_id", "")
            ),
            "action_generation": self._contract_int(
                getattr(message, "action_generation", 0)
            ),
            "planner_sequence": self._contract_int(
                getattr(message, "planner_sequence", 0)
            ),
            "state": self._contract_int(getattr(message, "state", 0)),
            "producer": str(getattr(message, "producer", "") or ""),
        }

    def _active_planner_contract_identity_locked(self):
        """Return the identity currently authorized by the action bridge."""
        contract = getattr(self, "active_action_contract", None)
        # A persistent successor publishes its mission boundary before the
        # controller lease is active.  The old action has already cleared the
        # ``active_*`` fields at that point, so using them here turns a valid
        # successor boundary into ``stale_boundary_transaction_id``.  Until
        # dispatch installs a new immutable action contract, the latest
        # accepted mission route is the only canonical boundary identity.
        if contract is None:
            transaction_id = getattr(
                self,
                "latest_goal_transaction_id",
                getattr(self, "active_goal_transaction_id", 0),
            )
            route_id = getattr(
                self, "latest_route_id", getattr(self, "active_route_id", 0)
            )
            graph_transaction_id = getattr(
                self,
                "latest_graph_transaction_id",
                getattr(self, "active_graph_transaction_id", 0),
            )
            map_epoch = getattr(
                self,
                "latest_route_map_epoch",
                getattr(self, "active_route_map_epoch", 0),
            )
            origin_epoch = getattr(
                self,
                "latest_origin_epoch",
                getattr(self, "active_origin_epoch", 0),
            )
            projection_epoch = getattr(
                self,
                "latest_projection_epoch",
                getattr(self, "active_projection_epoch", 0),
            )
        else:
            transaction_id = contract.get("transaction_id", 0)
            route_id = contract.get("route_id", 0)
            graph_transaction_id = contract.get("graph_transaction_id", 0)
            map_epoch = contract.get("map_epoch", 0)
            origin_epoch = contract.get("origin_epoch", 0)
            projection_epoch = contract.get("projection_epoch", 0)
        return {
            "transaction_id": self._contract_int(transaction_id),
            "route_id": self._contract_int(route_id),
            "graph_transaction_id": self._contract_int(graph_transaction_id),
            "map_epoch": self._contract_int(map_epoch),
            "lifecycle_transaction_id": self._contract_int(
                self._canonical_lifecycle_transaction_id()
                if contract is None
                else contract.get("lifecycle_transaction_id", 0)
            ),
            "origin_epoch": self._contract_int(origin_epoch),
            "projection_epoch": self._contract_int(projection_epoch),
            "action_generation": self._contract_int(
                getattr(self, "action_generation", 0)
                if contract is None
                else contract.get("generation", 0)
            ),
        }

    def _planner_boundary_expected_identity_locked(self):
        """Use the predecessor tombstone while its invalidation is in flight.

        Termination clears the live route before the planner's invalidation
        frame is delivered.  During that short window the current route fields
        intentionally contain zeroes, while ``pending_terminal_prepare`` still
        owns the exact predecessor identity.  Boundary validation must use
        that immutable contract; it must not weaken any identity comparison.
        """
        expected = self._active_planner_contract_identity_locked()
        try:
            expected_route = int(expected.get("route_id", 0) or 0)
        except (TypeError, ValueError):
            expected_route = 0
        if expected_route > 0:
            return expected
        pending = getattr(self, "pending_terminal_prepare", None)
        if not isinstance(pending, dict):
            return expected
        predecessor = pending.get("old_contract")
        if not isinstance(predecessor, dict):
            return expected
        try:
            predecessor_route = int(predecessor.get("route_id", 0) or 0)
        except (TypeError, ValueError):
            predecessor_route = 0
        if predecessor_route <= 0:
            return expected
        return {
            "transaction_id": self._contract_int(
                predecessor.get("transaction_id")
            ),
            "route_id": predecessor_route,
            "graph_transaction_id": self._contract_int(
                predecessor.get("graph_transaction_id")
            ),
            "map_epoch": self._contract_int(predecessor.get("map_epoch")),
            "lifecycle_transaction_id": self._contract_int(
                predecessor.get("lifecycle_transaction_id")
            ),
            "origin_epoch": self._contract_int(
                predecessor.get("origin_epoch")
            ),
            "projection_epoch": self._contract_int(
                predecessor.get("projection_epoch")
            ),
            "action_generation": self._contract_int(
                predecessor.get(
                    "generation", predecessor.get("action_generation")
                )
            ),
        }

    def _planner_contract_mismatch_locked(self, identity):
        """Reject a nonzero planner frame outside the live route contract."""
        expected = self._active_planner_contract_identity_locked()
        current_lifecycle = self._contract_int(
            self._canonical_lifecycle_transaction_id()
        )
        if identity["lifecycle_transaction_id"] <= 0:
            return "missing_lifecycle_transaction_id"
        if (
            current_lifecycle <= 0
            or identity["lifecycle_transaction_id"] != current_lifecycle
        ):
            return "planner_lifecycle_transaction_mismatch"
        for field in (
            "transaction_id",
            "route_id",
            "graph_transaction_id",
            "map_epoch",
            "origin_epoch",
            "projection_epoch",
            "action_generation",
        ):
            expected_value = expected[field]
            if field == "graph_transaction_id" and expected_value <= 0:
                continue
            if field == "map_epoch" and expected_value <= 0:
                continue
            if identity[field] != expected_value:
                return "planner_%s_mismatch" % field
        if not self.action_active:
            return "planner_contract_without_active_action"
        return ""

    def _publish_planner_invalidation_contract_locked(
        self, old_contract, new_generation, reason
    ):
        """Publish the Prepare stop frame with the predecessor identity."""
        publisher = getattr(self, "planner_command_invalidation_pub", None)
        if publisher is None:
            return
        contract = old_contract if isinstance(old_contract, dict) else {}
        # This topic is also observed by the bridge itself.  There is no
        # predecessor contract to invalidate when lifecycle cleanup is already
        # idle; publishing a zero-identity frame would be re-admitted as a
        # planner boundary and falsely report stale lifecycle ownership.
        if not contract or self._action_identity_key_locked(contract) is None:
            return
        message = PlannerCommandContract()
        message.transaction_id = self._contract_int(contract.get("transaction_id"))
        message.route_id = self._contract_int(contract.get("route_id"))
        message.graph_transaction_id = self._contract_int(
            contract.get("graph_transaction_id")
        )
        message.map_epoch = self._contract_int(contract.get("map_epoch"))
        message.origin_epoch = self._contract_int(contract.get("origin_epoch"))
        message.projection_epoch = self._contract_int(
            contract.get("projection_epoch")
        )
        message.lifecycle_transaction_id = str(
            self._contract_int(contract.get("lifecycle_transaction_id"))
        )
        message.action_generation = self._contract_int(new_generation)
        self.planner_contract_sequence = int(
            getattr(self, "planner_contract_sequence", 0) or 0
        ) + 1
        message.planner_sequence = self.planner_contract_sequence
        message.command = Twist()
        message.state = PlannerCommandContract.STATE_INVALIDATED
        message.reason = str(reason or "control_pipeline_prepare")
        message.producer = "teb_goal_bridge"
        publisher.publish(message)

    def _prepare_control_pipeline_terminal_locked(
        self, reason, action_contract=None
    ):
        """Phase 1: revoke ownership before touching actionlib transport."""
        existing = getattr(self, "pending_terminal_prepare", None)
        contract = (
            action_contract
            if action_contract is not None
            else getattr(self, "active_action_contract", None)
        )
        if contract is None:
            contract = getattr(self, "last_dispatch_identity", None)
        old_contract = {} if contract is None else dict(contract)
        identity_key = self._action_identity_key_locked(old_contract)
        old_generation = self._contract_int(
            old_contract.get(
                "generation",
                old_contract.get(
                    "action_generation", getattr(self, "action_generation", 0)
                ),
            )
        )
        if isinstance(existing, dict):
            existing_key = existing.get("identity_key")
            if existing_key is None:
                existing_key = self._action_identity_key_locked(
                    existing.get("old_contract")
                )
            if (
                existing_key == identity_key
                or (
                    identity_key is None
                    and self._contract_int(existing.get("old_generation"))
                    == old_generation
                )
            ):
                return existing
        tombstone = self._terminal_tombstone_for_key_locked(identity_key)
        if tombstone is not None:
            tombstone["duplicate_attempts"] = self._contract_int(
                tombstone.get("duplicate_attempts")
            ) + 1
            self.publish_bridge_status(
                "termination_duplicate_ignored",
                route_id=self._contract_int(identity_key[0]),
                action_generation=self._contract_int(identity_key[1]),
                transaction_id=self._contract_int(
                    old_contract.get("transaction_id")
                ),
                lifecycle_transaction_id=self._contract_int(
                    old_contract.get("lifecycle_transaction_id")
                ),
                graph_transaction_id=self._contract_int(
                    old_contract.get("graph_transaction_id")
                ),
                map_epoch=old_contract.get("map_epoch"),
                origin_epoch=old_contract.get("origin_epoch"),
                projection_epoch=old_contract.get("projection_epoch"),
                reason=str(reason or "control_pipeline_prepare"),
                dedupe_key=[int(identity_key[0]), int(identity_key[1])],
                duplicate_attempts=int(tombstone["duplicate_attempts"]),
            )
            return tombstone
        previous_active = bool(getattr(self, "action_active", False))
        new_generation = max(
            self._contract_int(getattr(self, "action_generation", 0)),
            old_generation,
        ) + 1
        prepared = {
            "old_contract": old_contract,
            "old_generation": old_generation,
            "new_generation": new_generation,
            "reason": str(reason or "control_pipeline_prepare"),
            "previous_active": previous_active,
            "cancel_requested": False,
            "terminal_published": False,
            "lease_release_requested": False,
            "identity_key": identity_key,
        }
        self.pending_terminal_prepare = prepared
        self.pending_lease_release_contract = prepared
        self._remember_terminal_tombstone_locked(identity_key, prepared)
        self._begin_transport_handoff_locked(
            old_contract,
            old_generation,
            new_generation,
            prepared["reason"],
            previous_active,
        )
        # This assignment is the first termination side effect. Every callback
        # which entered actionlib with the predecessor generation is stale
        # immediately, even before the zero boundary is published.
        self.action_generation = new_generation
        self.action_active = False
        self.handoff_requested = False
        invalidate = getattr(self, "_invalidate_teb_feedback_locked", None)
        if callable(invalidate):
            invalidate("terminal_prepare", clear_planner=True)
        self._publish_planner_invalidation_contract_locked(
            old_contract, new_generation, prepared["reason"]
        )
        self.publish_bridge_status(
            "action_generation_advanced",
            old_action_generation=old_generation,
            action_generation=new_generation,
            route_id=self._contract_int(old_contract.get("route_id")),
            transaction_id=self._contract_int(old_contract.get("transaction_id")),
            lifecycle_transaction_id=self._contract_int(
                old_contract.get("lifecycle_transaction_id")
            ),
            phase="prepare",
            reason=prepared["reason"],
        )
        self.publish_bridge_status(
            "terminal_prepare",
            route_id=self._contract_int(old_contract.get("route_id")),
            action_generation=new_generation,
            predecessor_action_generation=old_generation,
            controller_lease="revoked",
            planner_output="forced_zero",
            reason=prepared["reason"],
        )
        return prepared

    def _commit_termination(
        self,
        reason,
        source_goal=None,
        action_contract=None,
        *,
        terminal_status=None,
        watchdog_reason=None,
        terminal_reason=None,
        publish_terminal=True,
        arm_watchdog=True,
    ):
        """Run every Bridge termination through one prepare/commit boundary.

        Callers execute from the lifecycle tick or another already serialized
        Bridge critical section. The returned record is retained as a
        tombstone until the upstream owner acknowledges release.
        """
        prepared = self._prepare_control_pipeline_terminal_locked(
            reason,
            action_contract=action_contract,
        )
        if source_goal is None:
            contract = prepared.get("old_contract") or {}
            source_goal = copy.deepcopy(
                contract.get("source_goal")
                or getattr(self, "last_dispatched_goal", None)
            )
        committed = self._commit_control_pipeline_terminal_locked(
            prepared,
            source_goal,
            terminal_status=terminal_status,
            watchdog_reason=watchdog_reason,
            terminal_reason=terminal_reason,
            publish_terminal=publish_terminal,
            arm_watchdog=arm_watchdog,
        )
        prepared["committed"] = bool(committed or not publish_terminal)
        return prepared

    def _commit_control_pipeline_terminal_locked(
        self,
        prepared,
        source_goal,
        *,
        terminal_status=None,
        watchdog_reason=None,
        terminal_reason=None,
        publish_terminal=True,
        arm_watchdog=True,
    ):
        """Phase 2: cancel, publish terminal, then request exact release ACK."""
        if not isinstance(prepared, dict):
            return False
        old_contract = dict(prepared.get("old_contract") or {})
        if not prepared.get("cancel_requested"):
            cancel_goal = getattr(
                getattr(self, "action_client", None), "cancel_goal", None
            )
            transport_cancelled = False
            transport_error = None
            if callable(cancel_goal):
                try:
                    cancel_goal()
                    transport_cancelled = True
                except Exception as exc:  # pragma: no cover - ROS edge
                    transport_error = str(exc)
            else:
                transport_error = "cancel_goal_unavailable"
            prepared["cancel_requested"] = True
            prepared["transport_cancelled"] = transport_cancelled
            prepared["transport_error"] = transport_error
            self.publish_bridge_status(
                "move_base_cancel_or_terminal",
                route_id=self._contract_int(old_contract.get("route_id")),
                lifecycle_transaction_id=self._contract_int(
                    old_contract.get("lifecycle_transaction_id")
                ),
                action_generation=self._contract_int(
                    old_contract.get("generation")
                ),
                new_action_generation=self._contract_int(
                    prepared.get("new_generation")
                ),
                transport_cancelled=transport_cancelled,
                transport_error=transport_error,
            )
        self._poll_transport_handoff_locked()
        if getattr(self, "transport_handoff_pending", None) is None:
            # If actionlib was already terminal (for example, the prepare was
            # entered from done_cb), there is no asynchronous transport event
            # left to wait for. Clear the prepare immediately; otherwise the
            # successor remains blocked by a phantom terminal barrier.
            self._clear_terminal_prepare_if_matches_locked(
                prepared,
                self._transport_action_state_locked(
                    prepared.get("client")
                    if isinstance(prepared, dict)
                    else None
                ),
            )
        if (
            publish_terminal
            and not prepared.get("terminal_published")
            and source_goal is not None
        ):
            self._publish_execution_terminal_locked(
                source_goal,
                action_contract=old_contract,
                invalidate_feedback=False,
                status=terminal_status,
                reason=str(
                    terminal_reason
                    or watchdog_reason
                    or prepared.get("reason")
                    or "execution_terminal"
                ),
            )
            prepared["terminal_published"] = True
        if (
            publish_terminal
            and
            not prepared.get("lease_release_requested")
            and source_goal is not None
        ):
            self.publish_bridge_status(
                "lease_release_requested",
                route_id=self._contract_int(old_contract.get("route_id")),
                transaction_id=self._contract_int(
                    old_contract.get("transaction_id")
                ),
                lifecycle_transaction_id=self._contract_int(
                    old_contract.get("lifecycle_transaction_id")
                ),
                graph_transaction_id=self._contract_int(
                    old_contract.get("graph_transaction_id")
                ),
                map_epoch=self._contract_int(old_contract.get("map_epoch")),
                origin_epoch=self._contract_int(old_contract.get("origin_epoch")),
                projection_epoch=self._contract_int(
                    old_contract.get("projection_epoch")
                ),
                action_generation=self._contract_int(
                    old_contract.get("generation")
                ),
                new_action_generation=self._contract_int(
                    prepared.get("new_generation")
                ),
                reason=str(watchdog_reason or prepared.get("reason")),
            )
            prepared["lease_release_requested"] = True
        if arm_watchdog:
            arm_watchdog_fn = getattr(
                self, "_arm_route_lease_watchdog_locked", None
            )
            if callable(arm_watchdog_fn):
                arm_watchdog_fn(
                    status=terminal_status,
                    reason=str(watchdog_reason or prepared.get("reason")),
                    action_contract=old_contract,
                )
        return bool(prepared.get("terminal_published"))

    def _teb_feedback_identity_locked(self):
        """Return the route identity attached to the locally accepted feedback."""
        return {
            "generation": int(getattr(self, "action_generation", 0) or 0),
            "action_generation": int(
                getattr(self, "action_generation", 0) or 0
            ),
            "route_id": int(getattr(self, "active_route_id", 0) or 0),
            "transaction_id": int(
                getattr(self, "active_goal_transaction_id", 0) or 0
            ),
            "lifecycle_transaction_id": int(
                self._canonical_lifecycle_transaction_id()
            ),
            "map_epoch": getattr(self, "active_route_map_epoch", None),
            "origin_epoch": getattr(self, "active_origin_epoch", None),
            "projection_epoch": getattr(self, "active_projection_epoch", None),
            "graph_transaction_id": int(
                getattr(self, "active_graph_transaction_id", 0) or 0
            ),
        }

    def _invalidate_teb_feedback_locked(self, reason, clear_planner=False):
        """Invalidate feedback at every planner/route ownership boundary.

        ``FeedbackMsg`` does not carry the route transaction that produced it.
        The only safe contract is therefore to discard the selected trajectory
        whenever ownership changes or the planner emits a true stop, and to
        accept feedback again only after a fresh non-zero planner command.
        """
        reason = str(reason or "control_pipeline_boundary")
        previous_selected = (
            None
            if getattr(self, "latest_teb_selected_linear", None) is None
            else [
                round(float(self.latest_teb_selected_linear), 4),
                round(float(self.latest_teb_selected_angular), 4),
            ]
        )
        previous_valid = bool(getattr(self, "teb_feedback_valid", False))
        previous_reason = str(
            getattr(self, "teb_feedback_invalid_reason", "") or ""
        )
        previous_required = bool(
            getattr(self, "teb_feedback_requires_fresh_planner_command", False)
        )
        if (
            reason == "planner_zero_command"
            and previous_required
            and previous_reason in self._TEB_TERMINAL_BOUNDARY_REASONS
        ):
            # The planner's zero is the transport consequence of a terminal
            # already recorded above. Keep the stronger boundary reason so a
            # watchdog cannot reopen a failure window for the old route.
            return
        changed = bool(
            previous_valid
            or previous_selected is not None
            or previous_reason != reason
            or not previous_required
        )
        self.latest_teb_selected_linear = None
        self.latest_teb_selected_angular = None
        self.latest_teb_feedback_monotonic = 0.0
        self.teb_feedback_valid = False
        self.teb_feedback_invalid_reason = reason
        self.teb_feedback_requires_fresh_planner_command = True
        self.teb_feedback_generation = 0
        self.teb_feedback_route_id = 0
        self.teb_feedback_transaction_id = 0
        self.teb_feedback_map_epoch = None
        self.teb_feedback_graph_transaction_id = 0
        self._reset_teb_reorientation_locked()
        self.teb_feedback_invalidation_count = int(
            getattr(self, "teb_feedback_invalidation_count", 0) or 0
        ) + 1
        if clear_planner:
            self.latest_teb_planner_linear = None
            self.latest_teb_planner_angular = None
            self.latest_teb_planner_command_monotonic = 0.0
            self.teb_planner_stationary_since = 0.0
        if changed:
            identity = self._teb_feedback_identity_locked()
            self.publish_bridge_status(
                "teb_feedback_invalidated",
                reason=reason,
                clear_planner=bool(clear_planner),
                previous_selected_velocity=previous_selected,
                previous_valid=previous_valid,
                invalidation_count=int(self.teb_feedback_invalidation_count),
                **identity,
            )

    def on_turn_supervisor_status(self, message):
        """Hold same-priority frontier replacements during an atomic turn."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        state = str(payload.get("state", "UNKNOWN")).strip().upper() or "UNKNOWN"
        event = str(payload.get("event", "unknown"))
        with self.lock:
            self.turn_supervisor_state = state
            self.turn_supervisor_last_event = event
            if (
                event in ("turn_completed", "turn_released")
                and self.action_active
                and self.active_route_kind == "frontier_turn_connector"
            ):
                # XY feedback is stationary during a mandated yaw connector.
                # Completing it is still action-health progress.
                self.active_progress_monotonic = now_for(self)
                if self.active_feedback_distance is not None:
                    self.active_best_distance = self.active_feedback_distance
            if (
                event == "turn_completed"
                and self.action_active
                and self.active_route_kind == "frontier_turn_connector"
            ):
                self.turn_transition_ready = True
                self.dispatch_locked(
                    force=False, reason="turn_completed_route_release"
                )
        if state == "TURNING":
            rospy.loginfo_throttle(
                3.0,
                "TEB goal bridge holds frontier action while turn supervisor is TURNING",
            )

    def _reset_teb_reorientation_locked(self):
        """Forget a native TEB turn when its command/action boundary changes."""
        self.teb_reorientation_started_monotonic = 0.0
        self.teb_reorientation_reference_yaw = None
        self.teb_reorientation_last_yaw_progress_monotonic = 0.0
        self.teb_reorientation_total_yaw = 0.0

    def _teb_reorientation_command_active_locked(self, now):
        """Return true only for a fresh selected TEB in-place command."""
        return bool(
            self.teb_reorientation_enabled
            and self.teb_feedback_valid
            and int(getattr(self, "teb_feedback_generation", 0) or 0)
            == int(getattr(self, "action_generation", 0) or 0)
            and self.latest_teb_selected_linear is not None
            and self.latest_teb_selected_angular is not None
            and now - self.latest_teb_feedback_monotonic
            <= self.teb_reorientation_feedback_timeout
            and abs(float(self.latest_teb_selected_linear))
            <= self.teb_reorientation_linear_threshold
            and abs(float(self.latest_teb_selected_angular))
            >= self.teb_reorientation_angular_threshold
        )

    def on_teb_feedback(self, message):
        """Track TEB's selected command without becoming a local controller."""
        trajectories = list(message.trajectories)
        selected_index = int(message.selected_trajectory_idx)
        selected = (
            trajectories[selected_index]
            if 0 <= selected_index < len(trajectories)
            else None
        )
        first = (
            selected.trajectory[0]
            if selected is not None and selected.trajectory
            else None
        )
        command = Twist()
        if first is not None:
            command.linear.x = float(first.velocity.linear.x)
            command.angular.z = float(first.velocity.angular.z)
        now = now_for(self)
        with self.lock:
            if not self.action_active:
                self._invalidate_teb_feedback_locked("no_active_route")
                return
            if getattr(self, "require_planner_command_contract", False):
                if not bool(getattr(self, "planner_contract_valid", False)):
                    self.publish_bridge_status(
                        "teb_feedback_rejected",
                        reason="planner_contract_missing",
                        route_id=int(getattr(self, "active_route_id", 0) or 0),
                    )
                    return
                if getattr(self, "planner_contract_awaiting_feedback", False):
                    # FeedbackMsg reports the first point of TEB's selected
                    # trajectory, while PlannerCommandContract carries the
                    # post-processed command returned to move_base. They are
                    # related samples, not an equality contract; angular
                    # interpolation and command limiting can make their
                    # values differ substantially. The identity-bearing
                    # planner contract already fenced off pre-boundary
                    # feedback, so do not reject current feedback by speed
                    # deltas.
                    self.planner_contract_awaiting_feedback = False
            if self.teb_feedback_requires_fresh_planner_command:
                # A feedback message can be delivered after the old route was
                # invalidated but before the planner's new command. It has no
                # identity on the wire, so it cannot re-arm the health clock.
                return
            if first is None:
                self._invalidate_teb_feedback_locked("empty_teb_feedback")
                return
            self.latest_teb_selected_linear = float(first.velocity.linear.x)
            self.latest_teb_selected_angular = float(first.velocity.angular.z)
            self.latest_teb_feedback_monotonic = now
            identity = self._teb_feedback_identity_locked()
            self.teb_feedback_valid = True
            self.teb_feedback_invalid_reason = ""
            self.teb_feedback_requires_fresh_planner_command = False
            self.teb_feedback_generation = identity["generation"]
            self.teb_feedback_route_id = identity["route_id"]
            self.teb_feedback_transaction_id = identity["transaction_id"]
            self.teb_feedback_map_epoch = identity["map_epoch"]
            self.teb_feedback_graph_transaction_id = identity[
                "graph_transaction_id"
            ]
            if not self._teb_reorientation_command_active_locked(now):
                self._reset_teb_reorientation_locked()
            elif (
                self.action_active
                and self.odom_yaw is not None
                and self.teb_reorientation_started_monotonic <= 0.0
            ):
                self.teb_reorientation_started_monotonic = now
                self.teb_reorientation_reference_yaw = float(self.odom_yaw)
                self.teb_reorientation_last_yaw_progress_monotonic = now
                self.teb_reorientation_total_yaw = 0.0

    def on_teb_planner_command(self, message):
        """Track raw TEB output used for endpoint lifecycle release."""
        now = now_for(self)
        linear = float(message.linear.x)
        angular = float(message.angular.z)
        with self.lock:
            self.latest_teb_planner_linear = linear
            self.latest_teb_planner_angular = angular
            self.latest_teb_planner_command_monotonic = now
            if abs(linear) <= self.frontier_observation_completion_max_linear_speed:
                if self.teb_planner_stationary_since <= 0.0:
                    self.teb_planner_stationary_since = now
            else:
                self.teb_planner_stationary_since = 0.0
            if self._teb_planner_command_is_zero(linear, angular):
                # TEB can publish a zero post-processed command for one cycle
                # while its selected trajectory is still performing a bounded
                # in-place reorientation.  That is not a route boundary: the
                # selected feedback is the only available evidence that the
                # action is intentionally turning. Preserve it and let the
                # finite reorientation watchdog decide whether yaw progresses.
                preserving_reorientation = bool(
                    self.action_active
                    and self._teb_reorientation_command_active_locked(now)
                )
                if not preserving_reorientation:
                    self._invalidate_teb_feedback_locked("planner_zero_command")

    def on_planner_command_contract(self, message):
        """Accept only an identity-matched persistent planner command."""
        identity = self._planner_contract_identity(message)
        command = getattr(message, "command", Twist())
        state = identity["state"]
        # The bridge publishes prepare/invalidation telemetry on the same ROS
        # topic as the C++ planner contract. It has already applied this
        # boundary locally, so feeding the self-published frame back through
        # external planner admission would manufacture a stale-boundary
        # rejection during lifecycle cleanup.
        if identity["producer"] == "teb_goal_bridge":
            return
        with self.lock:
            is_boundary = state in (
                PlannerCommandContract.STATE_ZERO,
                PlannerCommandContract.STATE_INVALIDATED,
            )
            if is_boundary:
                expected = self._planner_boundary_expected_identity_locked()
                identity_fields = (
                    "transaction_id",
                    "route_id",
                    "graph_transaction_id",
                    "map_epoch",
                    "lifecycle_transaction_id",
                )
                identity_mismatch = None
                for field in identity_fields:
                    expected_value = expected[field]
                    if field in ("graph_transaction_id", "map_epoch") and expected_value <= 0:
                        continue
                    if identity[field] != expected_value:
                        identity_mismatch = "stale_boundary_%s" % field
                        break
                generation_matches = identity["action_generation"] in (
                    0,
                    expected["action_generation"],
                    int(getattr(self, "action_generation", 0) or 0),
                )
                if identity_mismatch or not generation_matches:
                    self.publish_bridge_status(
                        "planner_contract_rejected",
                        reason=identity_mismatch or "stale_boundary_contract",
                        **identity,
                    )
                    return
                self.planner_contract_valid = False
                self.planner_contract_state = state
                self.planner_contract_reason = str(
                    getattr(message, "reason", "planner_boundary")
                    or "planner_boundary"
                )
                self.planner_contract_identity = identity
                self.planner_contract_command = Twist()
                self.planner_contract_awaiting_feedback = True
                self._invalidate_teb_feedback_locked(
                    "planner_contract_%s" % self.planner_contract_reason,
                    clear_planner=True,
                )
                return
            mismatch = self._planner_contract_mismatch_locked(identity)
            if mismatch:
                self.publish_bridge_status(
                    "planner_contract_rejected", reason=mismatch, **identity
                )
                return
            sequence = identity["planner_sequence"]
            previous = getattr(self, "planner_contract_identity", None)
            if (
                isinstance(previous, dict)
                and previous.get("producer") == identity["producer"]
                and sequence <= self._contract_int(previous.get("planner_sequence"))
            ):
                self.publish_bridge_status(
                    "planner_contract_rejected",
                    reason="stale_planner_sequence",
                    **identity,
                )
                return
            previous_state = getattr(self, "planner_contract_state", None)
            previous_feedback_valid = bool(
                getattr(self, "teb_feedback_valid", False)
            )
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
            identity_changed = not isinstance(previous, dict) or any(
                int(identity.get(field, 0) or 0)
                != int(previous.get(field, 0) or 0)
                for field in identity_fields
            )
            contract_boundary = bool(
                identity_changed
                or not previous_feedback_valid
                or previous_state != PlannerCommandContract.STATE_ACTIVE
            )
            self.planner_contract_valid = True
            self.planner_contract_state = state
            self.planner_contract_reason = str(getattr(message, "reason", ""))
            self.planner_contract_identity = identity
            self.planner_contract_sequence = sequence
            self.planner_contract_producer = identity["producer"]
            self.planner_contract_wall = now_for(self)
            self.planner_contract_command = Twist()
            self.planner_contract_command.linear.x = float(command.linear.x)
            self.planner_contract_command.angular.z = float(command.angular.z)
            if contract_boundary:
                self.planner_contract_awaiting_feedback = True
                # FeedbackMsg has no identity and is slower than this stream.
                # Clear it when the route/state changes, but do not make every
                # same-route command sample invalidate the latest trajectory.
                self.teb_feedback_valid = False
                self.teb_feedback_invalid_reason = "awaiting_teb_feedback"
                self.latest_teb_selected_linear = None
                self.latest_teb_selected_angular = None
                self.latest_teb_feedback_monotonic = 0.0
                self.teb_feedback_generation = 0
                self.teb_feedback_route_id = 0
                self.teb_feedback_transaction_id = 0
                self.teb_feedback_map_epoch = None
                self.teb_feedback_graph_transaction_id = 0
                self._reset_teb_reorientation_locked()
            self.teb_feedback_requires_fresh_planner_command = False
            self.latest_teb_planner_linear = float(command.linear.x)
            self.latest_teb_planner_angular = float(command.angular.z)
            self.latest_teb_planner_command_monotonic = self.planner_contract_wall
            if self._teb_planner_command_is_zero(
                command.linear.x, command.angular.z
            ):
                self._invalidate_teb_feedback_locked("planner_zero_command")
                return
            self.publish_bridge_status(
                "planner_command_authorized",
                **identity,
                linear_x=round(float(command.linear.x), 6),
                angular_z=round(float(command.angular.z), 6),
            )

    def on_pose(self, message):
        """Measure real turn progress in odometry, independent of SLAM drift."""
        now = now_for(self)
        x = float(message.x)
        y = float(message.y)
        yaw = float(message.theta)
        with self.lock:
            self.odom_yaw = yaw
            self.odom_pose_monotonic = now
            if self.action_active:
                # Physical motion health must come from odometry, not from
                # MoveBase feedback.  Feedback may report a moving base pose
                # while the robot is actually oscillating in place.
                if self.active_motion_reference is None:
                    self.active_motion_reference = (x, y)
                    self.active_motion_progress_monotonic = now
                elif math.hypot(
                    x - self.active_motion_reference[0],
                    y - self.active_motion_reference[1],
                ) >= self.progress_epsilon:
                    self.active_motion_reference = (x, y)
                    self.active_motion_progress_monotonic = now
            if (
                self.teb_reorientation_started_monotonic <= 0.0
                or self.teb_reorientation_reference_yaw is None
            ):
                return
            yaw_delta = abs(
                self._angle_delta(yaw, self.teb_reorientation_reference_yaw)
            )
            if yaw_delta >= self.teb_reorientation_yaw_progress:
                self.teb_reorientation_total_yaw += yaw_delta
                self.teb_reorientation_reference_yaw = yaw
                self.teb_reorientation_last_yaw_progress_monotonic = now

    def _teb_reorientation_progressing_locked(self, now):
        """Check the finite action-health exemption for a native TEB turn."""
        if (
            not self.action_active
            or self.turn_supervisor_state == "TURNING"
            or not self._teb_reorientation_command_active_locked(now)
            or self.odom_yaw is None
            or now - self.odom_pose_monotonic
            > self.teb_reorientation_feedback_timeout
        ):
            self._reset_teb_reorientation_locked()
            return False
        if self.teb_reorientation_started_monotonic <= 0.0:
            self.teb_reorientation_started_monotonic = now
            self.teb_reorientation_reference_yaw = float(self.odom_yaw)
            self.teb_reorientation_last_yaw_progress_monotonic = now
            self.teb_reorientation_total_yaw = 0.0
            return False
        duration = now - self.teb_reorientation_started_monotonic
        yaw_progress_age = now - self.teb_reorientation_last_yaw_progress_monotonic
        if (
            duration > self.teb_reorientation_max_extension
            or yaw_progress_age > self.teb_reorientation_stagnation_timeout
            or self.teb_reorientation_total_yaw < self.teb_reorientation_yaw_progress
        ):
            return False
        return True
