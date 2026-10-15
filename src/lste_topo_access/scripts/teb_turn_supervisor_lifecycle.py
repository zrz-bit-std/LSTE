#!/usr/bin/env python3
"""Single-threaded ingress boundary for the TEB turn supervisor."""

from copy import deepcopy
import json

import rospy
from geometry_msgs.msg import Twist

from experiment_reset_contract import decode_reset_request, publish_reset_ack
from lifecycle_manager import EventType, LifecycleManager, State
from teb_turn_supervisor_callbacks import TebTurnSupervisorCallbacksMixin
from teb_turn_supervisor_control import TebTurnSupervisorControlMixin
from teb_turn_supervisor_contract import STATE_PASS_THROUGH
from teb_turn_supervisor_state import initialize_turn_supervisor_state


class TebTurnSupervisorLifecycleMixin:
    """Gather turn inputs and run the phase adapter from one timer tick."""

    def _initialize_lifecycle_manager(self):
        gp = rospy.get_param
        self.lifecycle_manager = LifecycleManager(
            event_handler=self._handle_lifecycle_event,
            transition_handler=self._on_lifecycle_transition,
            timeout_handler=self._on_lifecycle_timeout,
            allow_local_transactions=not bool(
                getattr(self, "require_planner_command_contract", False)
            ),
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
        try:
            value = int(payload.get("lifecycle_transaction_id", 0) or 0)
        except (TypeError, ValueError):
            return None
        return value if value > 0 else None

    def _enqueue_turn_event(
        self, event_type, payload=None, transaction_id=None,
        transaction_independent=False,
    ):
        if transaction_independent:
            # Telemetry may describe a newer bridge route before the bridge's
            # adoption status reaches this node.  Keep its wire identity for
            # handler-side validation, but bind queue ordering to the local
            # lifecycle at tick time rather than advancing high-water here.
            transaction_id = 0
        elif transaction_id is None:
            transaction_id = self._transaction_id_from_message(payload)
        if (
            not transaction_independent
            and
            transaction_id is not None
            and self.lifecycle_manager.is_stale_transaction(transaction_id)
        ):
            self.publish_status_locked(
                "stale_rejected",
                reason="lifecycle_transaction_id_below_high_water",
                received_lifecycle_transaction_id=int(transaction_id),
            )
            return False
        return self.lifecycle_manager.enqueue_type(
            event_type,
            deepcopy(payload),
            transaction_id=transaction_id,
        )

    def on_intent(self, message):
        return self._enqueue_turn_event(
            EventType.BRIDGE_INTENT,
            message,
            self._transaction_id_from_message(message),
        )

    def on_bridge_status(self, message):
        return self._enqueue_turn_event(
            EventType.BRIDGE_FRONTIER_STATUS,
            message,
            transaction_independent=True,
        )

    def on_goal(self, message):
        return self._enqueue_turn_event(EventType.BRIDGE_GOAL, message)

    def on_navfn_plan(self, message):
        return self._enqueue_turn_event(EventType.BRIDGE_NAVFN_PLAN, message)

    def on_pose(self, message):
        return self._enqueue_turn_event(EventType.BRIDGE_POSE, message)

    def on_scan(self, message):
        return self._enqueue_turn_event(EventType.SCAN_UPDATED, message)

    def on_planner_command(self, message):
        return self._enqueue_turn_event(EventType.BRIDGE_PLANNER_COMMAND, message)

    def on_planner_command_contract(self, message):
        return self._enqueue_turn_event(
            EventType.BRIDGE_PLANNER_COMMAND_CONTRACT,
            message,
            transaction_independent=True,
        )

    def on_teb_feedback(self, message):
        return self._enqueue_turn_event(EventType.BRIDGE_TEB_FEEDBACK, message)

    def on_mode(self, message):
        return self._enqueue_turn_event(EventType.BRIDGE_MODE, message)

    def on_task_done(self, message):
        return self._enqueue_turn_event(EventType.BRIDGE_TASK_DONE, message)

    def on_navigation_hold(self, message):
        # Keep the latest actuator gate outside mission transaction ordering.
        # The queued event remains useful for serialized lifecycle replay, but
        # the timer applies this latest sample last so a stale queued value
        # cannot re-arm a hold after a release.
        self.latest_navigation_hold_sample = bool(message.data)
        return self._enqueue_turn_event(
            EventType.BRIDGE_NAVIGATION_HOLD, message
        )

    def on_cmd_vel_mux_status(self, message):
        """Queue the actuator-boundary result for the serialized controller FSM."""
        return self._enqueue_turn_event(
            EventType.BRIDGE_MUX_STATUS,
            message,
            transaction_independent=True,
        )

    def on_hard_reset(self, message):
        """Queue reset with the same timer-owned boundary as other inputs."""
        return self._enqueue_turn_event(EventType.RESET, deepcopy(message))

    def on_timer(self, event):
        def compute(now):
            pending_hold = self.latest_navigation_hold_sample
            self.latest_navigation_hold_sample = None
            if pending_hold is not None:
                TebTurnSupervisorControlMixin.apply_navigation_hold_sample(
                    self, pending_hold
                )
            return TebTurnSupervisorControlMixin.on_timer(self, event)

        return self.lifecycle_manager.tick(
            compute_handler=compute
        )

    def _handle_lifecycle_event(self, event):
        if event.type == EventType.RESET:
            return self._apply_hard_reset(event)
        if (
            not getattr(self, "require_planner_command_contract", False)
            and
            self.lifecycle_manager.current_transaction_id == 0
            and event.type in (EventType.BRIDGE_INTENT, EventType.BRIDGE_GOAL)
        ):
            self.lifecycle_manager.begin_transaction(State.DISPATCHED)
        handlers = {
            EventType.BRIDGE_INTENT: TebTurnSupervisorCallbacksMixin.on_intent,
            EventType.BRIDGE_FRONTIER_STATUS: TebTurnSupervisorCallbacksMixin.on_bridge_status,
            EventType.BRIDGE_GOAL: TebTurnSupervisorCallbacksMixin.on_goal,
            EventType.BRIDGE_NAVFN_PLAN: TebTurnSupervisorCallbacksMixin.on_navfn_plan,
            EventType.BRIDGE_POSE: TebTurnSupervisorCallbacksMixin.on_pose,
            EventType.SCAN_UPDATED: TebTurnSupervisorCallbacksMixin.on_scan,
            EventType.BRIDGE_PLANNER_COMMAND: TebTurnSupervisorCallbacksMixin.on_planner_command,
            EventType.BRIDGE_PLANNER_COMMAND_CONTRACT: TebTurnSupervisorCallbacksMixin.on_planner_command_contract,
            EventType.RECOVERY_OBSERVED: TebTurnSupervisorCallbacksMixin.on_recovery_status,
            EventType.BRIDGE_TEB_FEEDBACK: TebTurnSupervisorCallbacksMixin.on_teb_feedback,
            EventType.BRIDGE_MODE: TebTurnSupervisorControlMixin.on_mode,
            EventType.BRIDGE_TASK_DONE: TebTurnSupervisorControlMixin.on_task_done,
            EventType.BRIDGE_NAVIGATION_HOLD: TebTurnSupervisorControlMixin.on_navigation_hold,
            EventType.BRIDGE_MUX_STATUS: TebTurnSupervisorCallbacksMixin.on_cmd_vel_mux_status,
        }
        handler = handlers.get(event.type)
        if handler is None:
            return None
        handler(self, event.payload)
        if event.type in (
            EventType.BRIDGE_INTENT,
            EventType.BRIDGE_GOAL,
        ):
            return State.DISPATCHED
        if event.type == EventType.BRIDGE_TASK_DONE and bool(
            getattr(event.payload, "data", False)
        ):
            return State.COMPLETED
        return None

    def _apply_hard_reset(self, event):
        """Clear every cached action/command before a warm slice resumes."""
        request = decode_reset_request(event.payload)
        if request is None:
            return None
        reset_id = request["reset_id"]
        if reset_id == self._last_hard_reset_id:
            publish_reset_ack(
                self.hard_reset_ack_pub,
                "lste_teb_turn_supervisor",
                request,
                # A duplicate request is an idempotent acknowledgement of the
                # original cleared snapshot.  Do not expose live control-tick
                # counters or a later mission transaction here.
                state=State.IDLE.value,
                transaction_id=int(
                    getattr(self, "_last_hard_reset_transaction_id", 0) or 0
                ),
                duplicate=True,
                active_action=False,
                planner_command_sequence=0,
                planner_transaction_id=0,
                output_sequence=0,
                command=[0.0, 0.0],
                planner_command_cleared=True,
            )
            return None

        previous_state = str(getattr(self, "state", STATE_PASS_THROUGH))
        previous_action = bool(getattr(self, "active_action", False))
        previous_task_done = bool(getattr(self, "task_done", False))
        new_transaction_id = self.lifecycle_manager.hard_reset(
            event=event,
            now=rospy.Time.now().to_sec(),
            transaction_id=request.get("transaction_id"),
        )
        # Keep publishers, parameters, and the timer intact; only reset the
        # supervisor's mutable execution state.  Late callbacks already queued
        # before this call are discarded by LifecycleManager.hard_reset().
        initialize_turn_supervisor_state(self)
        self._last_hard_reset_id = reset_id
        self._last_hard_reset_transaction_id = int(new_transaction_id)
        self._hard_reset_count += 1
        self.output_pub.publish(Twist())
        self.publish_status_locked(
            "hard_reset_complete",
            reset_id=reset_id,
            reason=request["reason"],
            state=State.IDLE.value,
            previous_state=previous_state,
            previous_active_action=previous_action,
            previous_task_done=previous_task_done,
            hard_reset_count=int(self._hard_reset_count),
            planner_command_cleared=True,
            output_command=[0.0, 0.0],
        )
        publish_reset_ack(
            self.hard_reset_ack_pub,
            "lste_teb_turn_supervisor",
            request,
            state=State.IDLE.value,
            transaction_id=new_transaction_id,
            active_action=False,
            planner_command_sequence=0,
            planner_transaction_id=0,
            output_sequence=0,
            command=[0.0, 0.0],
            planner_command_cleared=True,
        )
        rospy.loginfo(
            "TEB turn supervisor hard reset complete: reset_id=%s "
            "state=IDLE planner_command_cleared=true",
            reset_id,
        )
        return None

    def _on_lifecycle_transition(self, previous, current, event):
        publish = getattr(self, "publish_status_locked", None)
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
            release = getattr(self, "_release_turn_locked", None)
            if callable(release):
                release("lifecycle_failed", completed=False)
            self.state = STATE_PASS_THROUGH
            self.active_action = False
            self.active_action_goal = None
            self.active_action_source_goal = None
            self.active_action_identity = None
            self.turn_action_identity = None
            if callable(publish):
                publish(
                    "lifecycle_failure_cleanup",
                    lifecycle_transaction_id=int(
                        self.lifecycle_manager.current_transaction_id
                    ),
                )

    def _on_lifecycle_timeout(self, event):
        publish = getattr(self, "publish_status_locked", None)
        if callable(publish):
            publish(
                "lifecycle_timeout",
                state=event.payload.get("state"),
                timeout=event.payload.get("timeout"),
                lifecycle_transaction_id=int(event.transaction_id),
            )
