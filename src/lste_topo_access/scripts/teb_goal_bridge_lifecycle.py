#!/usr/bin/env python3
"""Single-threaded ingress boundary for the TEB action bridge."""

from copy import deepcopy
import json
import math

import rospy

from experiment_reset_contract import decode_reset_request, publish_reset_ack
from lifecycle_manager import EventType, LifecycleManager, State
from teb_goal_bridge_action_client import TebGoalBridgeActionClientMixin
from teb_goal_bridge_action_feedback import TebGoalBridgeActionFeedbackMixin
from teb_goal_bridge_action_terminal import TebGoalBridgeActionTerminalMixin
from teb_goal_bridge_frontier_status import TebGoalBridgeFrontierStatusMixin
from teb_goal_bridge_intent import TebGoalBridgeIntentMixin
from teb_goal_bridge_mission_control import TebGoalBridgeMissionControlMixin
from teb_goal_bridge_mission_input import TebGoalBridgeMissionInputMixin
from teb_goal_bridge_mission_runtime import TebGoalBridgeMissionRuntimeMixin
from teb_goal_bridge_persistent_frontier_endpoint import (
    TebGoalBridgePersistentFrontierEndpointMixin,
)
from teb_goal_bridge_persistent_target_result import (
    TebGoalBridgePersistentTargetResultMixin,
)
from teb_goal_bridge_route_monitoring import TebGoalBridgeRouteMonitoringMixin
from teb_goal_bridge_teb_runtime import TebGoalBridgeTebRuntimeMixin
from teb_goal_bridge_state import initialize_bridge_state


class TebGoalBridgeLifecycleMixin:
    """Gather ROS/actionlib facts and compute bridge state from one timer."""

    def _initialize_lifecycle_manager(self):
        gp = rospy.get_param
        self.lifecycle_tick_period = max(
            0.05, float(gp("~lifecycle_tick_period", 0.2))
        )
        self.lifecycle_manager = LifecycleManager(
            event_handler=self._handle_lifecycle_event,
            transition_handler=self._on_lifecycle_transition,
            timeout_handler=self._on_lifecycle_timeout,
            allow_local_transactions=False,
            timeouts={
                State.DISPATCHED: max(
                    1.0, float(gp("~lifecycle_dispatch_timeout", 120.0))
                ),
            },
        )
        self._last_hard_reset_id = None
        self._last_hard_reset_transaction_id = 0
        self._hard_reset_count = 0

    def _transaction_id_from_message(self, message):
        try:
            payload = json.loads(message.data)
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError):
            return None
        if not isinstance(payload, dict):
            return None
        # ``transaction_id`` is a semantic GoalManager sequence.  It is not a
        # lifecycle identity and must never be used to advance this FSM.
        raw = payload.get("lifecycle_transaction_id")
        try:
            value = int(raw or 0)
        except (TypeError, ValueError):
            return None
        return value if value > 0 else None

    def _local_lifecycle_transaction_id(self):
        return self._canonical_lifecycle_transaction_id()

    def _canonical_lifecycle_transaction_id(self):
        """Return the exact lifecycle identity of the latest mission route."""
        inherited = int(
            getattr(self, "latest_lifecycle_transaction_id", 0) or 0
        )
        if inherited > 0:
            # The mission command is the canonical owner.  A bridge-local
            # lifecycle high-water may advance while this route is draining;
            # it must not rewrite the action's identity.
            return inherited
        manager = getattr(self, "lifecycle_manager", None)
        latest = getattr(manager, "latest_transaction_id", None)
        if callable(latest):
            current = int(latest() or 0)
        else:
            current = int(getattr(manager, "current_transaction_id", 0) or 0)
        return current

    @staticmethod
    def _canonical_instruction_identity(payload):
        """Decode the lifecycle/route identity carried by a mission command."""
        try:
            value = json.loads(payload.data)
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError):
            return None, "malformed_payload"
        if not isinstance(value, dict):
            return None, "malformed_payload"
        try:
            lifecycle_id = int(value.get("lifecycle_transaction_id", 0) or 0)
            route_id = int(value.get("route_id", 0) or 0)
            graph_id = int(value.get("graph_transaction_id", 0) or 0)
            map_epoch = int(value.get("map_epoch", 0) or 0)
            origin_epoch = int(
                value.get("origin_epoch", map_epoch) or 0
            )
            projection_epoch = int(
                value.get("projection_epoch", map_epoch) or 0
            )
            priority = int(value.get("priority", 0) or 0)
        except (TypeError, ValueError):
            return None, "invalid_canonical_identity"
        try:
            semantic_transaction_id = int(
                value.get("transaction_id", 0) or 0
            )
        except (TypeError, ValueError):
            return None, "invalid_canonical_identity"
        identity = {
            "lifecycle_transaction_id": lifecycle_id,
            "route_id": route_id,
            "graph_transaction_id": graph_id,
            "map_epoch": map_epoch,
            "origin_epoch": origin_epoch,
            "projection_epoch": projection_epoch,
            "transaction_id": semantic_transaction_id,
            "priority": priority,
            "source": str(value.get("source", "") or "").strip().lower(),
            "event": str(value.get("event", "") or "").strip().lower(),
        }
        if lifecycle_id <= 0:
            return identity, "missing_lifecycle_transaction_id"
        if map_epoch <= 0:
            return identity, "missing_map_epoch"
        if origin_epoch <= 0:
            return identity, "missing_origin_epoch"
        if projection_epoch <= 0:
            return identity, "missing_projection_epoch"
        if identity["event"] in ("mission_goal", "target_terminal_observation") and (
            identity["transaction_id"] <= 0
        ):
            return identity, "missing_transaction_id"
        if identity["event"] == "mission_goal":
            raw_goal = value.get("goal")
            if not isinstance(raw_goal, (list, tuple)) or len(raw_goal) < 2:
                return identity, "missing_goal"
            try:
                if not all(math.isfinite(float(item)) for item in raw_goal[:2]):
                    return identity, "invalid_goal"
            except (TypeError, ValueError):
                return identity, "invalid_goal"
        direct_target_mission = (
            identity["event"] == "mission_goal"
            and identity["source"].startswith("target_")
        )
        # ``target_terminal_observation`` is an ownership-boundary command,
        # not an executable route.  It intentionally has route=0/graph=0
        # while it asks the bridge to release the predecessor target lease.
        # Keep the lifecycle/map identity checks above strict, but do not
        # confuse this boundary with a malformed mission route.
        target_terminal_boundary = (
            identity["event"] == "target_terminal_observation"
        )
        if route_id <= 0 and not target_terminal_boundary:
            return identity, "missing_route_identity"
        if (
            graph_id <= 0
            and not direct_target_mission
            and not target_terminal_boundary
        ):
            return identity, "missing_route_identity"
        return identity, ""

    def _canonical_instruction_stale_reason(self, identity):
        """Reject an instruction below any local canonical high-water mark."""
        if not isinstance(identity, dict):
            return "missing_canonical_identity"
        if identity["lifecycle_transaction_id"] < self.lifecycle_manager.transaction_high_water_mark():
            return "lifecycle_transaction_id_below_high_water"
        for field, attribute in (
            ("route_id", "canonical_route_high_water"),
            ("graph_transaction_id", "canonical_graph_high_water"),
            ("map_epoch", "canonical_map_epoch_high_water"),
            ("origin_epoch", "canonical_origin_epoch_high_water"),
            ("projection_epoch", "canonical_projection_epoch_high_water"),
        ):
            incoming = int(identity.get(field, 0) or 0)
            high_water = int(getattr(self, attribute, 0) or 0)
            if incoming > 0 and incoming < high_water:
                return "%s_below_high_water" % field
        return ""

    def _remember_canonical_instruction(self, identity):
        """Advance downstream high-water marks only after command admission."""
        if not isinstance(identity, dict):
            return
        self.canonical_lifecycle_high_water = max(
            int(getattr(self, "canonical_lifecycle_high_water", 0) or 0),
            int(identity.get("lifecycle_transaction_id", 0) or 0),
        )
        self.canonical_route_high_water = max(
            int(getattr(self, "canonical_route_high_water", 0) or 0),
            int(identity.get("route_id", 0) or 0),
        )
        self.canonical_graph_high_water = max(
            int(getattr(self, "canonical_graph_high_water", 0) or 0),
            int(identity.get("graph_transaction_id", 0) or 0),
        )
        self.canonical_map_epoch_high_water = max(
            int(getattr(self, "canonical_map_epoch_high_water", 0) or 0),
            int(identity.get("map_epoch", 0) or 0),
        )
        self.canonical_origin_epoch_high_water = max(
            int(getattr(self, "canonical_origin_epoch_high_water", 0) or 0),
            int(identity.get("origin_epoch", 0) or 0),
        )
        self.canonical_projection_epoch_high_water = max(
            int(
                getattr(self, "canonical_projection_epoch_high_water", 0)
                or 0
            ),
            int(identity.get("projection_epoch", 0) or 0),
        )

    def _record_canonical_rejection(self, reason, identity=None):
        """Log a dropped downstream instruction without mutating route state."""
        identity = identity if isinstance(identity, dict) else {}
        event = (
            "stale_rejected"
            if "below_high_water" in str(reason)
            else "canonical_identity_rejected"
        )
        self.publish_bridge_status(
            event,
            reason=str(reason),
            received_lifecycle_transaction_id=int(
                identity.get("lifecycle_transaction_id", 0) or 0
            ),
            received_transaction_id=int(identity.get("transaction_id", 0) or 0),
            received_route_id=int(identity.get("route_id", 0) or 0),
            received_graph_transaction_id=int(
                identity.get("graph_transaction_id", 0) or 0
            ),
            received_map_epoch=int(identity.get("map_epoch", 0) or 0),
            received_origin_epoch=int(
                identity.get("origin_epoch", 0) or 0
            ),
            received_projection_epoch=int(
                identity.get("projection_epoch", 0) or 0
            ),
        )
        rospy.logwarn(
            "TEB goal bridge discarded instruction: event=%s reason=%s "
            "lifecycle=%s route=%s graph=%s map=%s",
            event,
            reason,
            identity.get("lifecycle_transaction_id", 0),
            identity.get("route_id", 0),
            identity.get("graph_transaction_id", 0),
            identity.get("map_epoch", 0),
        )

    @staticmethod
    def _external_transaction_admission(payload, event_type):
        """Validate wire fields before importing a canonical lifecycle ID."""
        if event_type not in (EventType.BRIDGE_GOAL_COMMAND, EventType.BRIDGE_INTENT):
            return False, "event_is_not_a_mission_owner"
        _, reason = TebGoalBridgeLifecycleMixin._canonical_instruction_identity(
            payload
        )
        return (False, reason) if reason else (True, "accepted")

    def _enqueue_bridge_event(self, event_type, payload=None, transaction_id=None):
        ownership_event = event_type in (
            EventType.BRIDGE_GOAL,
            EventType.BRIDGE_GOAL_COMMAND,
            EventType.BRIDGE_INTENT,
        )
        if ownership_event and getattr(self, "require_canonical_identity", False):
            identity, reason = self._canonical_instruction_identity(payload)
            if reason:
                self._record_canonical_rejection(reason, identity)
                return False
            stale_reason = self._canonical_instruction_stale_reason(identity)
            if stale_reason:
                self._record_canonical_rejection(stale_reason, identity)
                return False
            self._remember_canonical_instruction(identity)
            transaction_id = identity["lifecycle_transaction_id"]
        elif transaction_id is None:
            transaction_id = self._transaction_id_from_message(payload)
            if (
                transaction_id is not None
                and self.lifecycle_manager.is_stale_transaction(transaction_id)
            ):
                self._record_canonical_rejection(
                    "lifecycle_transaction_id_below_high_water",
                    {"lifecycle_transaction_id": transaction_id},
                )
                return False
        return self.lifecycle_manager.enqueue_type(
            event_type,
            deepcopy(payload),
            transaction_id=transaction_id,
        )

    # Gather-only topic callbacks.
    def on_goal(self, message):
        if self.use_goal_command:
            return False
        return self._enqueue_bridge_event(EventType.BRIDGE_GOAL, message)

    def on_goal_command(self, message):
        # A mission command is an ownership event.  Let the ingress helper
        # copy its wire lifecycle identity into the queued event so the tick
        # can adopt it before the handler runs.  Tagging it with the local ID
        # first allows the handler to create a newer local transaction and
        # then incorrectly reject the valid upstream command as stale.
        return self._enqueue_bridge_event(EventType.BRIDGE_GOAL_COMMAND, message)

    def on_intent(self, message):
        if self.use_goal_command:
            return False
        # Legacy intent is also a mission-owner event and must carry the wire
        # lifecycle identity for the same reason as on_goal_command().
        return self._enqueue_bridge_event(EventType.BRIDGE_INTENT, message)

    def on_frontier_status(self, message):
        # Frontier status is an observation stream.  Its owner's lifecycle ID
        # must not advance the bridge before an accepted mission command does.
        return self._enqueue_bridge_event(
            EventType.BRIDGE_FRONTIER_STATUS,
            message,
            transaction_id=self._local_lifecycle_transaction_id(),
        )

    def on_turn_supervisor_status(self, message):
        # Turn-supervisor status is telemetry from a downstream adapter, not a
        # new mission owner. Do not let its independently generated lifecycle
        # UUID advance this bridge past a GoalManager transaction that is still
        # waiting at the warm-slice boundary.
        current_transaction_id = int(
            self._canonical_lifecycle_transaction_id()
        )
        return self._enqueue_bridge_event(
            EventType.BRIDGE_TURN_STATUS,
            message,
            transaction_id=current_transaction_id,
        )

    def on_teb_feedback(self, message):
        return self._enqueue_bridge_event(EventType.BRIDGE_TEB_FEEDBACK, message)

    def on_teb_planner_command(self, message):
        return self._enqueue_bridge_event(
            EventType.BRIDGE_PLANNER_COMMAND, message
        )

    def on_planner_command_contract(self, message):
        # Planner contracts are observations from the current lifecycle, not
        # new mission ownership. Validate their wire identity in the handler
        # without letting a delayed planner sample advance this lifecycle.
        return self._enqueue_bridge_event(
            EventType.BRIDGE_PLANNER_COMMAND_CONTRACT,
            message,
            transaction_id=self._local_lifecycle_transaction_id(),
        )

    def on_navfn_plan(self, message):
        return self._enqueue_bridge_event(EventType.BRIDGE_NAVFN_PLAN, message)

    def on_local_costmap(self, message):
        return self._enqueue_bridge_event(EventType.BRIDGE_COSTMAP, message)

    def on_pose(self, message):
        return self._enqueue_bridge_event(EventType.BRIDGE_POSE, message)

    def on_mode(self, message):
        return self._enqueue_bridge_event(EventType.BRIDGE_MODE, message)

    def on_task_done(self, message):
        return self._enqueue_bridge_event(EventType.BRIDGE_TASK_DONE, message)

    def on_navigation_hold(self, message):
        """Queue the mission-level intentional zero-velocity gate."""
        return self._enqueue_bridge_event(
            EventType.BRIDGE_NAVIGATION_HOLD,
            message,
            transaction_id=0,
        )

    def on_persistent_target_plan_result(self, message):
        # ``transaction_id`` in the C++ result is the semantic target command
        # identity, not this bridge's distributed lifecycle transaction. Keep
        # the local lifecycle identity on the ingress event so a valid target
        # installation cannot be discarded as stale after a newer mission
        # command adopted a UUID-based lifecycle transaction.
        return self._enqueue_bridge_event(
            EventType.BRIDGE_TARGET_RESULT,
            message,
            transaction_id=int(
                self._canonical_lifecycle_transaction_id()
            ),
        )

    def on_persistent_frontier_endpoint_reached(self, message):
        return self._enqueue_bridge_event(
            EventType.BRIDGE_FRONTIER_ENDPOINT, message
        )

    def on_hard_reset(self, message):
        """Queue a reset so cancellation and state clearing share one tick."""
        return self._enqueue_bridge_event(EventType.RESET, deepcopy(message))

    # Actionlib callbacks are also gather-only.
    def _admit_action_callback_generation(self, callback_name, generation):
        """Reject late actionlib callbacks before they enter the event queue."""
        try:
            callback_generation = int(generation)
        except (TypeError, ValueError):
            callback_generation = -1
        current_generation = int(
            getattr(self, "action_generation", 0) or 0
        )
        if callback_generation == current_generation:
            return True
        self.publish_bridge_status(
            "stale_action_callback_ignored",
            callback=str(callback_name),
            callback_action_generation=callback_generation,
            current_action_generation=current_generation,
            reason="generation_mismatch",
        )
        rospy.logwarn(
            "TEB goal bridge dropped late action callback: callback=%s "
            "callback_generation=%s current_generation=%s reason=generation_mismatch",
            callback_name,
            callback_generation,
            current_generation,
        )
        return False

    def on_active(self, generation):
        if not self._admit_action_callback_generation("active", generation):
            return False
        return self._enqueue_bridge_event(
            EventType.ACTION_FEEDBACK, ("active", generation)
        )

    def on_feedback(self, generation, feedback):
        if not self._admit_action_callback_generation("feedback", generation):
            return False
        return self._enqueue_bridge_event(
            EventType.ACTION_FEEDBACK, ("feedback", generation, deepcopy(feedback))
        )

    def on_done(self, generation, status, result):
        if not self._admit_action_callback_generation("done", generation):
            return False
        return self._enqueue_bridge_event(
            EventType.ACTION_DONE,
            (generation, status, deepcopy(result)),
        )

    def on_terminal_timer(self, event):
        del event
        return self._enqueue_bridge_event(EventType.BRIDGE_WAKE, None)

    def on_timer(self, event):
        return self.lifecycle_manager.tick(
            compute_handler=lambda _now: TebGoalBridgeMissionRuntimeMixin.on_timer(
                self, event
            )
        )

    def _handle_lifecycle_event(self, event):
        if event.type == EventType.RESET:
            return self._apply_hard_reset(event)
        if (
            getattr(self, "require_canonical_identity", False)
            and event.type
            in (
                EventType.BRIDGE_GOAL,
                EventType.BRIDGE_GOAL_COMMAND,
                EventType.BRIDGE_INTENT,
            )
        ):
            identity, reason = self._canonical_instruction_identity(event.payload)
            if reason:
                self._record_canonical_rejection(reason, identity)
                return None
            if int(event.transaction_id or 0) != int(
                identity["lifecycle_transaction_id"]
            ):
                self._record_canonical_rejection(
                    "event_lifecycle_transaction_mismatch", identity
                )
                return None
        handlers = {
            EventType.BRIDGE_GOAL: TebGoalBridgeMissionInputMixin.on_goal,
            EventType.BRIDGE_GOAL_COMMAND: TebGoalBridgeMissionInputMixin.on_goal_command,
            EventType.BRIDGE_INTENT: TebGoalBridgeIntentMixin.on_intent,
            EventType.BRIDGE_FRONTIER_STATUS: TebGoalBridgeFrontierStatusMixin.on_frontier_status,
            EventType.BRIDGE_TURN_STATUS: TebGoalBridgeTebRuntimeMixin.on_turn_supervisor_status,
            EventType.BRIDGE_TEB_FEEDBACK: TebGoalBridgeTebRuntimeMixin.on_teb_feedback,
            EventType.BRIDGE_PLANNER_COMMAND: TebGoalBridgeTebRuntimeMixin.on_teb_planner_command,
            EventType.BRIDGE_PLANNER_COMMAND_CONTRACT: TebGoalBridgeTebRuntimeMixin.on_planner_command_contract,
            EventType.BRIDGE_NAVFN_PLAN: TebGoalBridgeRouteMonitoringMixin.on_navfn_plan,
            EventType.BRIDGE_COSTMAP: TebGoalBridgeRouteMonitoringMixin.on_local_costmap,
            EventType.BRIDGE_POSE: TebGoalBridgeTebRuntimeMixin.on_pose,
            EventType.BRIDGE_MODE: TebGoalBridgeMissionControlMixin.on_mode,
            EventType.BRIDGE_TASK_DONE: TebGoalBridgeMissionControlMixin.on_task_done,
            EventType.BRIDGE_NAVIGATION_HOLD: TebGoalBridgeMissionControlMixin.on_navigation_hold,
            EventType.BRIDGE_TARGET_RESULT: TebGoalBridgePersistentTargetResultMixin.on_persistent_target_plan_result,
            EventType.BRIDGE_FRONTIER_ENDPOINT: TebGoalBridgePersistentFrontierEndpointMixin.on_persistent_frontier_endpoint_reached,
        }
        if event.type == EventType.ACTION_FEEDBACK:
            payload = event.payload
            if payload[0] == "active":
                return TebGoalBridgeActionClientMixin.on_active(
                    self, payload[1]
                )
            return TebGoalBridgeActionFeedbackMixin.on_feedback(
                self, payload[1], payload[2]
            )
        if event.type == EventType.ACTION_DONE:
            generation, status, result = event.payload
            return TebGoalBridgeActionTerminalMixin.on_done(
                self, generation, status, result
            )
        handler = handlers.get(event.type)
        if handler is None:
            return None
        if self.use_goal_command and event.type in (
            EventType.BRIDGE_GOAL,
            EventType.BRIDGE_INTENT,
        ):
            return None
        previous_goal_transaction = int(
            getattr(self, "latest_goal_transaction_id", 0) or 0
        )
        handler(self, event.payload)
        if event.type == EventType.BRIDGE_GOAL_COMMAND:
            # An ignored/stale command must not manufacture a lifecycle state
            # that claims an executable route exists. Accepted mission and
            # target-terminal commands both advance this semantic transaction.
            if int(getattr(self, "latest_goal_transaction_id", 0) or 0) <= previous_goal_transaction:
                return None
            return State.DISPATCHED
        if event.type in (EventType.BRIDGE_GOAL, EventType.BRIDGE_INTENT):
            return State.DISPATCHED
        if event.type == EventType.BRIDGE_TASK_DONE and bool(
            getattr(event.payload, "data", False)
        ):
            return State.COMPLETED
        return None

    def _apply_hard_reset(self, event):
        """Cancel MoveBase and clear every bridge-side route lease."""
        request = decode_reset_request(event.payload)
        if request is None:
            return None
        reset_id = request["reset_id"]
        if reset_id == self._last_hard_reset_id:
            publish_reset_ack(
                self.hard_reset_ack_pub,
                "lste_teb_goal_bridge",
                request,
                state=State.IDLE.value,
                transaction_id=int(
                    getattr(self, "_last_hard_reset_transaction_id", 0) or 0
                ),
                duplicate=True,
                action_active=False,
                route_owner=False,
                active_route_id=0,
                latest_route_id=0,
                intent_seen=False,
                route_lease_watchdog_active=False,
                persistent_commands_cleared=True,
            )
            return None

        previous_generation = int(getattr(self, "action_generation", 0) or 0)
        previous_route_id = int(getattr(self, "active_route_id", 0) or 0)
        previous_intent = bool(getattr(self, "intent_seen", False))
        previous_action_active = bool(getattr(self, "action_active", False))
        # A hard reset may be the boundary that issued cancel_goal. Preserve
        # the old client's transport gate across state reinitialization; a new
        # SimpleActionClient cannot observe the goal that is still settling.
        transport_handoff_pending = getattr(
            self, "transport_handoff_pending", None
        )
        if callable(getattr(self, "cancel_locked", None)):
            self.cancel_locked("experiment_hard_reset")
        clear_persistent = getattr(
            self, "_publish_persistent_hard_reset_locked", None
        )
        persistent_commands_cleared = True
        if callable(clear_persistent):
            persistent_commands_cleared = bool(clear_persistent(request["reason"]))
        new_transaction_id = self.lifecycle_manager.hard_reset(
            event=event,
            now=rospy.Time.now().to_sec(),
            transaction_id=request.get("transaction_id"),
        )
        # Reuse the bridge's complete initialization list so newly added route
        # fields cannot survive a slice boundary by accident. Preserve a
        # monotonic action generation to reject late callbacks from the old
        # MoveBase goal after the reset.
        initialize_bridge_state(self)
        self.action_generation = max(1, previous_generation + 1)
        if isinstance(transport_handoff_pending, dict):
            self.transport_handoff_pending = transport_handoff_pending
            self.transport_handoff_last_report_monotonic = 0.0
        self._last_hard_reset_id = reset_id
        self._last_hard_reset_transaction_id = int(new_transaction_id)
        self._hard_reset_count += 1
        self.publish_bridge_status(
            "hard_reset_complete",
            reset_id=reset_id,
            reason=request["reason"],
            state=State.IDLE.value,
            previous_route_id=previous_route_id,
            previous_intent_seen=previous_intent,
            previous_action_active=previous_action_active,
            hard_reset_count=int(self._hard_reset_count),
            persistent_commands_cleared=persistent_commands_cleared,
        )
        publish_reset_ack(
            self.hard_reset_ack_pub,
            "lste_teb_goal_bridge",
            request,
            state=State.IDLE.value,
            transaction_id=new_transaction_id,
            action_active=bool(getattr(self, "action_active", False)),
            route_owner=bool(getattr(self, "active_route_id", 0)),
            active_route_id=int(getattr(self, "active_route_id", 0) or 0),
            latest_route_id=int(getattr(self, "latest_route_id", 0) or 0),
            intent_seen=bool(getattr(self, "intent_seen", False)),
            route_lease_watchdog_active=bool(
                getattr(self, "route_lease_watchdog", None)
            ),
            persistent_commands_cleared=persistent_commands_cleared,
        )
        return None

    def _on_lifecycle_transition(self, previous, current, event):
        publish = getattr(self, "publish_bridge_status", None)
        if callable(publish):
            publish(
                "lifecycle_transition",
                previous_state=previous.value,
                current_state=current.value,
                lifecycle_transaction_id=int(
                    self.lifecycle_manager.current_transaction_id
                ),
                transition_event=(
                    None if event is None else event.type.value
                ),
            )
        if current == State.FAILED:
            cancel = getattr(self, "cancel_locked", None)
            if callable(cancel):
                cancel("lifecycle_failed")
            if callable(publish):
                publish(
                    "lifecycle_failure_cleanup",
                    lifecycle_transaction_id=int(
                        self._canonical_lifecycle_transaction_id()
                    ),
                )

    def _on_lifecycle_timeout(self, event):
        publish = getattr(self, "publish_bridge_status", None)
        if callable(publish):
            publish(
                "lifecycle_timeout",
                state=event.payload.get("state"),
                timeout=event.payload.get("timeout"),
                lifecycle_transaction_id=int(event.transaction_id),
            )
