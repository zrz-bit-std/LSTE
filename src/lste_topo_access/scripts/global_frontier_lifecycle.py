#!/usr/bin/env python3
"""Lifecycle ingress and tick ownership for the online frontier node."""

from copy import deepcopy
import json

import rospy

from clock_provider import ClockProvider, now_for
from experiment_reset_contract import decode_reset_request, publish_reset_ack
from lifecycle_manager import EventType, LifecycleManager, State
from global_frontier_event_callbacks import GlobalFrontierEventCallbacksMixin
from global_frontier_planning_runtime import GlobalFrontierPlanningRuntimeMixin
from global_frontier_planning_navfn import GlobalFrontierPlanningNavfnMixin
from global_frontier_route_state import GlobalFrontierRouteStateMixin
from global_frontier_terminal_replan import GlobalFrontierTerminalReplanMixin


class GlobalFrontierLifecycleMixin:
    """Keep ROS ingress passive and route lifecycle state single-threaded."""

    def _initialize_lifecycle_manager(self):
        gp = rospy.get_param
        self.lifecycle_tick_period = max(
            0.05, float(gp("~lifecycle_tick_period", 0.2))
        )
        self.time_provider = getattr(self, "time_provider", None) or ClockProvider()
        self.lifecycle_manager = LifecycleManager(
            event_handler=self._handle_lifecycle_event,
            transition_handler=self._on_lifecycle_transition,
            timeout_handler=self._on_lifecycle_timeout,
            timeouts={
                State.DISPATCHED: max(
                    1.0, float(gp("~lifecycle_dispatch_timeout", 120.0))
                ),
                State.SYNCING: max(
                    0.1, float(gp("~lifecycle_sync_timeout", 10.0))
                ),
            },
            time_provider=self.time_provider,
        )
        self._lifecycle_sync_seen = set()
        self._lifecycle_sync_started_transaction = 0
        self._last_hard_reset_id = None
        self._last_hard_reset_transaction_id = 0
        self._hard_reset_count = 0
        self._rehydration_transaction_id = 0
        self._rehydration_pending_streams = set()
        self._rehydration_applied_streams = set()
        self._rehydration_ready_for_planning = True
        self._rehydration_wait_signature = None

    def lifecycle_state(self):
        return self.lifecycle_manager.current_state

    def lifecycle_transaction_id(self):
        manager = getattr(self, "lifecycle_manager", None)
        if manager is None:
            return 0
        latest = getattr(manager, "latest_transaction_id", None)
        if callable(latest):
            return int(latest() or 0)
        return int(getattr(manager, "current_transaction_id", 0) or 0)

    def route_lifecycle_transaction_id(self):
        """Return the identity belonging to the current route lease.

        The lifecycle manager can advance while a released controller route is
        still draining.  Keep that predecessor's wire identity stable, while
        allowing a newly activated route to capture the newest identity.
        """
        active_route_id = int(getattr(self, "active_route_id", 0) or 0)
        active_identity = int(
            getattr(self, "active_route_lifecycle_transaction_id", 0) or 0
        )
        if active_route_id > 0 and active_identity > 0:
            return active_identity
        return self.lifecycle_transaction_id()

    def lifecycle_active(self):
        return self.lifecycle_manager.is_in(
            State.DISPATCHED,
            State.EXECUTION_DONE,
            State.SYNCING,
        )

    def _enqueue_lifecycle_event(self, event_type, payload=None, transaction_id=None):
        """The only operation exposed to asynchronous ROS ingress."""
        if transaction_id is None:
            transaction_id = self._transaction_id_from_message(payload, event_type)
        return self.lifecycle_manager.enqueue_type(
            event_type,
            payload,
            transaction_id=transaction_id,
        )

    @staticmethod
    def _transaction_id_from_message(message, event_type=None):
        """Read a distributed lifecycle identity without applying the event."""
        # Bridge status is an observation stream, not an owner of this
        # lifecycle. Importing the bridge's local transaction here can adopt a
        # newer ID while a frontier route is being selected and lets a stale
        # status event cross the route boundary.
        if event_type == EventType.NAV_FAILED:
            return None
        try:
            value = int(getattr(message, "lifecycle_transaction_id", 0) or 0)
        except (TypeError, ValueError):
            value = 0
        if value > 0:
            if (
                event_type == EventType.EXECUTION_TERMINAL
                and int(getattr(message, "route_id", 0) or 0) <= 0
            ):
                return None
            return value
        try:
            payload = json.loads(message.data)
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError):
            return None
        if not isinstance(payload, dict):
            return None
        if event_type == EventType.NAV_FAILED:
            source = str(
                payload.get(
                    "active_intent_source",
                    payload.get("intent_source", ""),
                )
                or ""
            ).strip().lower()
            if source and source != "global_slam_frontier":
                return None
        if event_type == EventType.TURN_STATUS:
            source = str(
                payload.get(
                    "intent_source",
                    payload.get("active_action_source", ""),
                )
                or ""
            ).strip().lower()
            route_kind = str(
                payload.get(
                    "route_kind",
                    payload.get("active_action_route_kind", ""),
                )
                or ""
            ).strip().lower()
            if source and source != "global_slam_frontier" and route_kind not in {
                "frontier_endpoint",
                "portal_transition",
                "portal_probe",
                "local_egress",
            }:
                return None
        try:
            value = int(payload.get("lifecycle_transaction_id", 0) or 0)
        except (TypeError, ValueError):
            return None
        return value if value > 0 else None

    # ROS callbacks: gather an immutable message reference and return.
    def on_map(self, message):
        return self._enqueue_lifecycle_event(
            EventType.MAP_UPDATED, deepcopy(message)
        )

    def on_costmap(self, message):
        return self._enqueue_lifecycle_event(
            EventType.COSTMAP_UPDATED, ("full", deepcopy(message))
        )

    def on_costmap_update(self, message):
        return self._enqueue_lifecycle_event(
            EventType.COSTMAP_UPDATED, ("delta", deepcopy(message))
        )

    def on_pose(self, message):
        return self._enqueue_lifecycle_event(
            EventType.POSE_UPDATED, deepcopy(message)
        )

    def on_scan(self, message):
        return self._enqueue_lifecycle_event(
            EventType.SCAN_UPDATED, deepcopy(message)
        )

    def on_task(self, message):
        return self._enqueue_lifecycle_event(
            EventType.TASK_UPDATED, deepcopy(message)
        )

    def on_detections(self, message):
        return self._enqueue_lifecycle_event(
            EventType.DETECTIONS_UPDATED, deepcopy(message)
        )

    def on_goal_arbitration(self, message):
        return self._enqueue_lifecycle_event(
            EventType.GOAL_UPDATED, deepcopy(message)
        )

    def on_task_done(self, message):
        return self._enqueue_lifecycle_event(
            EventType.TASK_COMPLETED, deepcopy(message)
        )

    def on_move_base_recovery(self, message):
        return self._enqueue_lifecycle_event(
            EventType.RECOVERY_OBSERVED, deepcopy(message)
        )

    def on_bridge_status(self, message):
        # Lease ACKs are causal evidence for a predecessor route. They may be
        # delivered after this owner has already adopted a newer route
        # transaction, so keep them FIFO but rebind them to the transaction
        # current when the lifecycle tick consumes them. Other bridge status
        # events retain the normal stale-transaction filtering.
        event_type = EventType.NAV_FAILED
        try:
            payload = json.loads(message.data)
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError):
            payload = None
        if (
            isinstance(payload, dict)
            and str(payload.get("event", "") or "").strip()
            == "lease_release_ack"
        ):
            event_type = EventType.CONTROLLER_LEASE_RELEASE_ACK
        return self._enqueue_lifecycle_event(event_type, deepcopy(message))

    def on_replan_request(self, message):
        return self._enqueue_lifecycle_event(
            EventType.REPLAN_REQUESTED, deepcopy(message)
        )

    def on_turn_status(self, message):
        return self._enqueue_lifecycle_event(
            EventType.TURN_STATUS, deepcopy(message)
        )

    def on_execution_terminal(self, message):
        # The terminal is a causal ownership boundary.  Mark the current
        # planning snapshot obsolete before enqueueing so a slow candidate
        # pass yields before the bridge's handoff watchdog can expire.  Route
        # state remains exclusively owned by LifecycleManager.tick().
        request_preempt = getattr(self, "request_planning_preempt", None)
        if callable(request_preempt):
            request_preempt("execution_terminal")
        else:
            self.planning_preempt_requested = True
            self.planning_preempt_reason = "execution_terminal"
        accepted = self._enqueue_lifecycle_event(
            EventType.EXECUTION_TERMINAL, deepcopy(message)
        )
        if not accepted:
            clear_preempt = getattr(self, "clear_planning_preempt", None)
            if callable(clear_preempt):
                clear_preempt()
            else:
                self.planning_preempt_requested = False
                self.planning_preempt_reason = ""
        return accepted

    def on_hard_reset(self, message):
        """Queue a slice reset; domain state changes stay inside ``tick``."""
        return self._enqueue_lifecycle_event(
            EventType.RESET, deepcopy(message)
        )

    def on_immediate_plan(self, event):
        del event
        return self._enqueue_lifecycle_event(
            EventType.REPLAN_REQUESTED, {"event": "immediate_plan"}
        )

    def on_terminal_drain_timer(self, event):
        del event
        return self._enqueue_lifecycle_event(
            EventType.EXECUTION_TERMINAL, {"event": "terminal_drain"}
        )

    def on_timer(self, event):
        """Run gather, compute, and scatter from one fixed-rate timer."""
        return self.lifecycle_manager.tick(
            compute_handler=lambda _now: GlobalFrontierPlanningRuntimeMixin.on_timer(
                self, event
            )
        )

    def _handle_lifecycle_event(self, event):
        """Apply one gathered fact from inside ``LifecycleManager.tick``."""
        expire_pending = getattr(
            self, "_expire_terminals_waiting_for_dispatch", None
        )
        if callable(expire_pending):
            expire_pending()
        if event.type == EventType.RESET:
            return self._apply_hard_reset(event)
        handlers = {
            EventType.MAP_UPDATED: self._apply_map,
            EventType.POSE_UPDATED: self._apply_pose,
            EventType.SCAN_UPDATED: self._apply_scan,
            EventType.TASK_UPDATED: self._apply_task,
            EventType.DETECTIONS_UPDATED: self._apply_detections,
            EventType.GOAL_UPDATED: self._apply_goal_arbitration,
            EventType.TASK_COMPLETED: self._apply_task_done,
            EventType.RECOVERY_OBSERVED: self._apply_move_base_recovery,
            EventType.EXECUTION_TERMINAL: self._apply_execution_terminal_event,
            EventType.NAV_FAILED: self._apply_bridge_status,
            EventType.CONTROLLER_LEASE_RELEASE_ACK: self._apply_bridge_status,
            EventType.REPLAN_REQUESTED: self._apply_replan_request_event,
            EventType.TURN_STATUS: self._apply_turn_status,
            EventType.NAVFN_RESULT: self._apply_navfn_result,
        }
        if event.type == EventType.COSTMAP_UPDATED:
            kind, payload = event.payload
            rehydration_costmap = bool(
                kind == "full"
                and int(getattr(event, "transaction_id", 0) or 0)
                == int(getattr(self, "_rehydration_transaction_id", 0) or 0)
                and "costmap" in getattr(
                    self, "_rehydration_pending_streams", set()
                )
            )
            self._applying_rehydration_costmap = rehydration_costmap
            try:
                result = (
                    self._apply_costmap(payload)
                    if kind == "full"
                    else (
                        self._apply_costmap_update(payload)
                        if kind == "delta"
                        else GlobalFrontierRouteStateMixin.invalidate_costmap(self)
                    )
                )
            finally:
                self._applying_rehydration_costmap = False
            if kind == "full" and not rehydration_costmap:
                self._rehydration_costmap_fallback = None
                self._rehydration_costmap_fallback_active = False
            # Only a complete full grid satisfies the reset planning input
            # barrier. A delta/resync is an update to an already accepted
            # projection and must not make a partially rebuilt snapshot look
            # ready for graph materialization.
            if kind == "full":
                self._record_rehydration_event_applied(event, "costmap")
            if (
                kind != "resync"
                and
                self.lifecycle_state() == State.SYNCING
                and event.created_at >= self.lifecycle_manager.state_entry_time
            ):
                self._lifecycle_sync_seen.add("costmap")
                if self._lifecycle_sync_seen == {"map", "costmap"}:
                    self._lifecycle_sync_seen.clear()
                    return State.IDLE
            return result
        handler = handlers.get(event.type)
        if handler is None:
            return None
        if event.type == EventType.MAP_UPDATED:
            if (
                self.lifecycle_state() == State.SYNCING
                and event.created_at >= self.lifecycle_manager.state_entry_time
            ):
                self._lifecycle_sync_seen.add("map")
        result = handler(event.payload)
        if event.type == EventType.MAP_UPDATED:
            self._record_rehydration_event_applied(event, "map")
        elif event.type == EventType.POSE_UPDATED:
            self._record_rehydration_event_applied(event, "pose")
        if result is not None:
            return result
        if (
            self.lifecycle_state() == State.SYNCING
            and self._lifecycle_sync_seen == {"map", "costmap"}
        ):
            self._lifecycle_sync_seen.clear()
            return State.IDLE
        if event.type == EventType.TASK_COMPLETED:
            return State.COMPLETED if bool(getattr(event.payload, "data", False)) else State.IDLE
        return None

    def _record_rehydration_event_applied(self, event, stream):
        """Publish an auditable marker when a reset snapshot is consumed."""
        transaction_id = int(getattr(event, "transaction_id", 0) or 0)
        if transaction_id != int(getattr(self, "_rehydration_transaction_id", 0) or 0):
            return
        pending = getattr(self, "_rehydration_pending_streams", set())
        if stream not in pending:
            return
        pending.discard(stream)
        applied = getattr(self, "_rehydration_applied_streams", set())
        applied.add(stream)
        self.publish_status(
            "hard_reset_rehydration_applied",
            rehydrated=True,
            rehydrated_stream=str(stream),
            rehydration_transaction_id=transaction_id,
            rehydration_applied_streams=sorted(applied),
        )
        if pending:
            return
        self._rehydration_ready_for_planning = bool(
            all(name in applied for name in ("map", "costmap", "pose"))
        )
        self._rehydration_wait_signature = None
        self.publish_status(
            "hard_reset_rehydration_complete",
            rehydrated=True,
            rehydration_transaction_id=transaction_id,
            rehydration_applied_streams=sorted(applied),
            planner_inputs_ready=all(
                name in applied for name in ("map", "costmap", "pose")
            ),
        )

    def _rehydration_inputs_ready_for_planning(self):
        """Prevent graph planning until the current reset inputs are applied."""
        transaction_id = int(
            getattr(self, "_rehydration_transaction_id", 0) or 0
        )
        if transaction_id <= 0:
            return True
        pending = tuple(
            sorted(getattr(self, "_rehydration_pending_streams", set()))
        )
        ready = bool(
            getattr(self, "_rehydration_ready_for_planning", False)
            and not pending
        )
        if ready:
            return True
        signature = (transaction_id, pending)
        if signature != getattr(self, "_rehydration_wait_signature", None):
            self._rehydration_wait_signature = signature
            publish = getattr(self, "publish_status", None)
            if callable(publish):
                publish(
                    "hard_reset_rehydration_wait",
                    rehydrated=False,
                    rehydration_transaction_id=transaction_id,
                    pending_streams=list(pending),
                    reason="planner_inputs_not_ready",
                )
        return False

    def _capture_hard_reset_snapshots(self):
        """Capture the latest valid sensor projection before clearing state."""
        map_message = getattr(self, "map_msg", None)
        costmap_message = getattr(self, "costmap_msg", None)
        costmap_snapshot_stale = False
        if costmap_message is None:
            # The delta stream may have invalidated the working projection
            # after the last full grid. An explicit hard reset is a stopped
            # actuator boundary, so retain that full grid as the next planner
            # bootstrap instead of forcing the next slice to wait forever.
            costmap_message = getattr(self, "_last_full_costmap", None)
            costmap_snapshot_stale = costmap_message is not None
        fresh_costmap = getattr(self, "fresh_costmap", None)
        if costmap_message is not None and callable(fresh_costmap):
            # Keep the same freshness contract used by planning while a route
            # owns the robot. A stopped slice boundary may use the structural
            # fallback below when the latched full-grid stream is quiet.
            fresh = fresh_costmap()
            if fresh is None:
                # A slice boundary is also a controller stop boundary. When
                # the route has already been released, retain a structurally
                # valid full grid as a short-lived bootstrap baseline instead
                # of making the next slice wait forever for a latched topic.
                # Never do this while an active route still owns the robot.
                active_route = int(getattr(self, "active_route_id", 0) or 0)
                stationary = bool(getattr(self, "costmap_stationary", False))
                valid_shape = False
                try:
                    expected = int(costmap_message.info.width) * int(
                        costmap_message.info.height
                    )
                    valid_shape = bool(
                        expected > 0
                        and len(costmap_message.data) == expected
                        and float(costmap_message.info.resolution) > 0.0
                    )
                except (AttributeError, TypeError, ValueError):
                    valid_shape = False
                if valid_shape:
                    costmap_snapshot_stale = True
                else:
                    costmap_message = None
            else:
                costmap_message = fresh
        self._last_hard_reset_costmap_snapshot_stale = bool(
            costmap_snapshot_stale
        )
        pose_message = getattr(self, "pose_odom", None)
        return {
            "map": None if map_message is None else deepcopy(map_message),
            "costmap": (
                None
                if costmap_message is None
                else deepcopy(costmap_message)
            ),
            "pose": None if pose_message is None else deepcopy(pose_message),
        }

    def _enqueue_hard_reset_rehydration(self, transaction_id, snapshots):
        """Re-enter cached inputs through the normal lifecycle event queue."""
        # Keep all required streams pending, including a stream for which the
        # reset had no cached sample. A live callback in the new transaction
        # may satisfy that stream; planning remains closed until all three
        # inputs have crossed this boundary.
        self._rehydration_transaction_id = int(transaction_id)
        self._rehydration_pending_streams = {"map", "costmap", "pose"}
        self._rehydration_applied_streams = set()
        self._rehydration_ready_for_planning = False
        self._rehydration_wait_signature = None
        pending = set()
        queued = {}
        for stream, event_type, payload in (
            ("map", EventType.MAP_UPDATED, snapshots.get("map")),
            (
                "costmap",
                EventType.COSTMAP_UPDATED,
                (
                    "full",
                    snapshots.get("costmap"),
                ),
            ),
            ("pose", EventType.POSE_UPDATED, snapshots.get("pose")),
        ):
            if payload is None or (
                stream == "costmap" and payload[1] is None
            ):
                queued[stream] = False
                continue
            accepted = self._enqueue_lifecycle_event(
                event_type,
                payload,
                transaction_id=int(transaction_id),
            )
            queued[stream] = bool(accepted)
            if accepted:
                pending.add(stream)
        fallback = snapshots.get("costmap")
        self._rehydration_costmap_fallback = (
            None if fallback is None else deepcopy(fallback)
        )
        self._last_full_costmap = (
            None if fallback is None else deepcopy(fallback)
        )
        self._rehydration_costmap_fallback_active = bool(
            queued.get("costmap") and self._rehydration_costmap_fallback is not None
        )
        self._costmap_rehydrated_pose_grace = bool(queued.get("costmap"))
        return queued

    def _apply_hard_reset(self, event):
        """Clear the complete explorer projection for one warm slice."""
        request = decode_reset_request(event.payload)
        if request is None:
            return None
        reset_id = request["reset_id"]
        if reset_id == self._last_hard_reset_id:
            publish_reset_ack(
                self.hard_reset_ack_publisher,
                "lste_global_frontier",
                request,
                state=State.IDLE.value,
                transaction_id=int(
                    getattr(self, "_last_hard_reset_transaction_id", 0) or 0
                ),
                duplicate=True,
                active_route_id=0,
                route_owner=False,
                graph_transaction_active=False,
                graph_route_lease_active=False,
            )
            return None

        task = deepcopy(getattr(self, "latest_task", None))
        snapshots = self._capture_hard_reset_snapshots()
        previous_route_id = int(getattr(self, "active_route_id", 0) or 0)
        previous_graph_transaction = bool(
            getattr(self, "graph_route_action_transaction", None)
        )
        previous_owner = bool(getattr(self, "active_frontier", None))
        new_transaction_id = self.lifecycle_manager.hard_reset(
            event=event,
            now=now_for(self),
            transaction_id=request.get("transaction_id"),
        )
        # Re-running the existing initializer is the authoritative clear list
        # for every graph ledger, route history, lease, and map projection.
        self._initialize_runtime_state()
        if task is not None:
            GlobalFrontierEventCallbacksMixin.on_task(self, task)
        rehydration = self._enqueue_hard_reset_rehydration(
            new_transaction_id,
            snapshots,
        )
        self._last_hard_reset_id = reset_id
        self._last_hard_reset_transaction_id = int(new_transaction_id)
        self._hard_reset_count += 1
        self.publish_status(
            "hard_reset_complete",
            reset_id=reset_id,
            reason=request["reason"],
            previous_active_route_id=previous_route_id,
            previous_route_owner=previous_owner,
            previous_graph_transaction_active=previous_graph_transaction,
            state=State.IDLE.value,
            hard_reset_count=int(self._hard_reset_count),
            rehydration_transaction_id=int(new_transaction_id),
            rehydration_queued=rehydration,
            rehydration_missing_streams=sorted(
                stream for stream, accepted in rehydration.items() if not accepted
            ),
            costmap_snapshot_stale=bool(
                getattr(self, "_last_hard_reset_costmap_snapshot_stale", False)
            ),
        )
        publish_reset_ack(
            self.hard_reset_ack_publisher,
            "lste_global_frontier",
            request,
            state=State.IDLE.value,
            transaction_id=new_transaction_id,
            active_route_id=int(getattr(self, "active_route_id", 0) or 0),
            route_owner=bool(getattr(self, "active_frontier", None)),
            graph_transaction_active=bool(
                getattr(self, "graph_route_action_transaction", None)
            ),
            graph_route_lease_active=bool(
                getattr(self, "graph_route_plan_lease_active", False)
            ),
            map_epoch=getattr(self, "last_map_epoch", None),
            task_preserved=task is not None,
        )
        return None

    def _on_lifecycle_transition(self, previous, current, event):
        publish = getattr(self, "publish_status", None)
        if callable(publish):
            publish(
                "lifecycle_transition",
                previous_state=previous.value,
                current_state=current.value,
                transaction_id=int(self.lifecycle_transaction_id()),
                transition_event=(
                    None if event is None else event.type.value
                ),
            )
        if current == State.FAILED:
            self._settle_failed_route_obligations(
                now_for(self), reason="lifecycle_failed"
            )
            active_frontier = getattr(self, "active_frontier", None)
            release = getattr(self, "release_active_frontier", None)
            active_route_id = int(getattr(self, "active_route_id", 0) or 0)
            dispatch_contract = getattr(self, "_bridge_dispatch_contracts", {}).get(
                active_route_id
            )
            terminal_contract = getattr(self, "active_terminal_contract", None)
            controller_release_pending = bool(
                active_frontier is not None
                and getattr(self, "controller_owned_route_failure", False)
                and active_route_id > 0
                and (
                    isinstance(dispatch_contract, dict)
                    or isinstance(terminal_contract, dict)
                )
            )
            if active_frontier is not None and callable(release):
                release(
                    discard_prefetch=True,
                    queue_successor=not controller_release_pending,
                    await_controller_release_ack=controller_release_pending,
                    terminal_contract=(
                        dict(terminal_contract)
                        if isinstance(terminal_contract, dict)
                        else None
                    ),
                )
            else:
                clear_prefetch = getattr(self, "clear_prefetched_frontier", None)
                if callable(clear_prefetch):
                    clear_prefetch()
            self.last_planning_wall = 0.0
            if callable(publish):
                publish(
                    "lifecycle_failure_cleanup",
                    transaction_id=int(self.lifecycle_transaction_id()),
                    released_route=active_frontier is not None,
                    controller_release_pending=controller_release_pending,
                )
        elif current == State.EXECUTION_DONE:
            self._lifecycle_sync_seen.clear()
            self._lifecycle_sync_started_transaction = (
                self.lifecycle_transaction_id()
            )
            self.lifecycle_manager.enqueue_type(EventType.SYNC_REQUESTED)
        elif current == State.SYNCING:
            self._enqueue_sync_snapshot()
            if callable(publish):
                publish(
                    "lifecycle_sync_requested",
                    transaction_id=int(self.lifecycle_transaction_id()),
                    timeout=float(
                        self.lifecycle_manager._timeouts.get(State.SYNCING, 0.0)
                    ),
                )

    def _settle_failed_route_obligations(self, now, reason):
        """Close semantic attempts before releasing their physical route.

        Lifecycle failure cleanup is the common boundary for controller
        failures. Clearing the active route first used to orphan a durable
        PortalProbe/WorkItem attempt, so every later planning pass saw an
        active owner and returned ``hold`` forever.
        """
        settle_probe = getattr(self, "settle_active_portal_probe", None)
        if (
            callable(settle_probe)
            and getattr(self, "active_portal_probe_id", None) is not None
        ):
            settle_probe("failed", now, reason)

        settle_work_item = getattr(self, "settle_active_work_item", None)
        if (
            callable(settle_work_item)
            and getattr(self, "active_work_item_id", None) is not None
        ):
            settle_work_item(now, "deferred", reason)

    def _enqueue_sync_snapshot(self):
        """Feed valid cached map state back through the SYNCING FSM.

        Map and full costmap topics may be quiet while the robot is stopped.
        Reusing a still-valid cache preserves the event contract without
        requiring a new geometric publication merely to prove synchronization.
        """
        publish = getattr(self, "publish_status", None)
        transaction_id = int(self.lifecycle_transaction_id())
        state_entry_time = self.lifecycle_manager.state_entry_time
        map_message = getattr(self, "map_msg", None)
        costmap_message = getattr(self, "costmap_msg", None)
        max_age = max(0.0, float(getattr(self, "costmap_max_age", 0.0)))
        last_receive = float(getattr(self, "costmap_last_receive_wall", 0.0))
        costmap_age = (
            float("inf")
            if last_receive <= 0.0
            else now_for(self) - last_receive
        )
        fresh_costmap = getattr(self, "fresh_costmap", None)
        if callable(fresh_costmap):
            costmap_message = fresh_costmap()
            costmap_valid = costmap_message is not None
        else:
            costmap_valid = (
                costmap_message is not None
                and max_age > 0.0
                and costmap_age <= max_age
            )
        if callable(publish):
            publish(
                "lifecycle_sync_snapshot_requested",
                transaction_id=transaction_id,
                map_cached=map_message is not None,
                costmap_cached=costmap_message is not None,
                costmap_age_seconds=(
                    None if costmap_age == float("inf") else round(costmap_age, 3)
                ),
            )
        if map_message is None or not costmap_valid:
            if callable(publish):
                publish(
                    "lifecycle_sync_waiting_for_publish",
                    transaction_id=transaction_id,
                    missing=(
                        ["map"] if map_message is None else []
                    ) + (["costmap"] if not costmap_valid else []),
                )
            return False
        map_event = self.lifecycle_manager.make_event(
            EventType.MAP_UPDATED,
            deepcopy(map_message),
            transaction_id=transaction_id,
            created_at=state_entry_time,
        )
        costmap_event = self.lifecycle_manager.make_event(
            EventType.COSTMAP_UPDATED,
            ("full", deepcopy(costmap_message)),
            transaction_id=transaction_id,
            created_at=state_entry_time,
        )
        map_enqueued = self.lifecycle_manager.enqueue(map_event)
        costmap_enqueued = self.lifecycle_manager.enqueue(costmap_event)
        if callable(publish):
            publish(
                "lifecycle_sync_snapshot_reused",
                transaction_id=transaction_id,
                map_enqueued=bool(map_enqueued),
                costmap_enqueued=bool(costmap_enqueued),
            )
        return bool(map_enqueued and costmap_enqueued)

    def _on_lifecycle_timeout(self, event):
        if hasattr(self, "publish_status"):
            self.publish_status(
                "lifecycle_timeout",
                state=event.payload.get("state"),
                timeout=event.payload.get("timeout"),
                transaction_id=int(event.transaction_id),
            )

    def _apply_map(self, message):
        return GlobalFrontierRouteStateMixin.apply_map(self, message)

    def _apply_costmap(self, message):
        return GlobalFrontierRouteStateMixin.apply_costmap(self, message)

    def _apply_costmap_update(self, message):
        return GlobalFrontierRouteStateMixin.apply_costmap_update(self, message)

    def _apply_pose(self, message):
        return GlobalFrontierRouteStateMixin.apply_pose(self, message)

    def _apply_scan(self, message):
        return GlobalFrontierEventCallbacksMixin.apply_scan(self, message)

    def _apply_task(self, message):
        previous = (
            getattr(self, "current_task_id", ""),
            getattr(self, "current_task_version", ""),
        )
        result = GlobalFrontierEventCallbacksMixin.apply_task(self, message)
        current = (
            getattr(self, "current_task_id", ""),
            getattr(self, "current_task_version", ""),
        )
        if current != previous:
            self.lifecycle_manager.begin_transaction(State.IDLE)
        return result

    def _apply_detections(self, message):
        return GlobalFrontierEventCallbacksMixin.apply_detections(self, message)

    def _apply_goal_arbitration(self, message):
        return GlobalFrontierEventCallbacksMixin.apply_goal_arbitration(
            self, message
        )

    def _apply_task_done(self, message):
        return GlobalFrontierEventCallbacksMixin.apply_task_done(self, message)

    def _apply_move_base_recovery(self, message):
        return GlobalFrontierEventCallbacksMixin.apply_move_base_recovery(
            self, message
        )

    def _apply_bridge_status(self, message):
        accepted = GlobalFrontierEventCallbacksMixin.apply_bridge_status(
            self, message
        )
        # NAV_FAILED is also the ingress type for non-terminal bridge status
        # events. Returning the current state suppresses LifecycleManager's
        # default NAV_FAILED -> FAILED transition for those observations.
        return State.FAILED if accepted else self.lifecycle_state()

    def _apply_turn_status(self, message):
        return GlobalFrontierEventCallbacksMixin.apply_turn_status(self, message)

    def _apply_navfn_result(self, message):
        if not isinstance(message, dict):
            return None
        GlobalFrontierPlanningNavfnMixin._store_navfn_validation_result(
            self,
            message.get("key"),
            message.get("payload", {}),
            message.get("result", {}),
            message.get("completed_wall"),
        )
        return None

    def _apply_execution_terminal_event(self, message):
        try:
            if isinstance(message, dict):
                return None
            before_route = getattr(self, "active_route_id", 0)
            GlobalFrontierTerminalReplanMixin._process_execution_terminal_locked(
                self, message
            )
            if (
                before_route > 0
                and getattr(self, "active_terminal_received", False)
            ):
                return State.EXECUTION_DONE
            return None
        finally:
            # The queued terminal, including a stale/mismatched one, has now
            # crossed the lifecycle boundary.  A later snapshot may compute
            # again, but it must not inherit this event's invalidation.
            clear_preempt = getattr(self, "clear_planning_preempt", None)
            if callable(clear_preempt):
                clear_preempt()
            else:
                self.planning_preempt_requested = False
                self.planning_preempt_reason = ""

    def _apply_replan_request_event(self, message):
        if isinstance(message, dict):
            if message.get("event") in ("immediate_plan", "terminal_replan"):
                self.last_planning_wall = 0.0
            return None
        self.lifecycle_manager.begin_transaction(State.IDLE)
        return GlobalFrontierEventCallbacksMixin.apply_replan_request(
            self, message
        )
