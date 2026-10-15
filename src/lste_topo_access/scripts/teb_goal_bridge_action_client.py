"""Action-client operations for the TEB goal bridge.

This module is deliberately policy-free.  It owns the mechanics of waiting
for ``move_base``, creating an action goal, resetting action-scoped state, and
sending or cancelling an action.  Dispatch policy lives in sibling modules.
"""

import copy
import subprocess
import time
from types import MappingProxyType

import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseGoal

from clock_provider import now_for


class TebGoalBridgeActionClientMixin:
    # A cancelled move_base goal normally reaches PREEMPTED quickly.  Recovery
    # behaviors can nevertheless leave the action server in PREEMPTING forever;
    # keep that transport fault bounded without allowing a successor to run in
    # parallel with the predecessor.
    _TRANSPORT_HANDOFF_TIMEOUT_SECONDS = 15.0
    _TRANSPORT_HANDOFF_CONTROLLER_RESTART_TIMEOUT_SECONDS = 15.0
    _TRANSPORT_HANDOFF_CONTROLLER_RESTART_MAX_ATTEMPTS = 2

    def _effective_action_epoch_locked(self):
        """Use the map snapshot epoch for every executable route kind."""
        route_map_epoch = getattr(self, "latest_route_map_epoch", None)
        if route_map_epoch is not None:
            try:
                return int(route_map_epoch)
            except (TypeError, ValueError):
                pass
        source = str(
            getattr(self, "latest_intent_source", "unknown") or "unknown"
        ).strip().lower()
        if source == "global_slam_frontier":
            value = getattr(self, "latest_frontier_map_epoch", None)
            if value is not None:
                try:
                    return int(value)
                except (TypeError, ValueError):
                    pass
        return int(getattr(self, "latest_target_epoch", 0) or 0)

    def _remember_last_dispatch_identity_locked(self):
        """Retain the semantic identity paired with ``last_dispatched_goal``."""
        map_epoch = getattr(self, "latest_route_map_epoch", None)
        if map_epoch is None:
            map_epoch = getattr(self, "latest_frontier_map_epoch", None)
        self.last_dispatch_identity = {
            "action_generation": int(getattr(self, "action_generation", 0) or 0),
            "transaction_id": int(
                getattr(self, "latest_goal_transaction_id", 0) or 0
            ),
            "lifecycle_transaction_id": int(
                self._canonical_lifecycle_transaction_id()
            ),
            "route_id": int(getattr(self, "latest_route_id", 0) or 0),
            "graph_transaction_id": int(
                getattr(self, "latest_graph_transaction_id", 0) or 0
            ),
            "epoch": self._effective_action_epoch_locked(),
            "map_epoch": map_epoch,
            "origin_epoch": getattr(self, "latest_origin_epoch", None),
            "projection_epoch": getattr(self, "latest_projection_epoch", None),
            "route_kind": str(
                getattr(self, "latest_route_kind", "") or ""
            ).strip().lower(),
            "mission_route_kind": str(
                getattr(self, "latest_mission_route_kind", "") or ""
            ).strip().lower(),
            "source": str(
                getattr(self, "latest_intent_source", "unknown") or "unknown"
            ).strip().lower(),
            "priority": int(getattr(self, "latest_intent_priority", 0) or 0),
        }

    def _action_server_ready(self):
        if self.action_server_seen:
            return True
        if not self.action_client.wait_for_server(rospy.Duration(0.0)):
            rospy.loginfo_throttle(3.0, "TEB goal bridge waiting for move_base action server")
            return False
        self.action_server_seen = True
        rospy.loginfo("TEB goal bridge connected to move_base action server")
        return True

    @staticmethod
    def _transport_state_is_terminal(status):
        """Return whether actionlib has finished the tracked MoveBase goal."""
        try:
            status = int(status)
        except (TypeError, ValueError):
            return False
        return status in (
            GoalStatus.PREEMPTED,
            GoalStatus.SUCCEEDED,
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        )

    @staticmethod
    def _transport_action_server_available_locked(client):
        """Probe one action client without waiting inside the lifecycle tick."""
        wait_for_server = getattr(client, "wait_for_server", None)
        if not callable(wait_for_server):
            return False
        try:
            return bool(wait_for_server(rospy.Duration(0.0)))
        except (TypeError, RuntimeError, rospy.ROSException):
            return False

    def _request_transport_controller_restart_locked(self, record, reason):
        """Restart only move_base when a cancelled action never becomes terminal."""
        if not isinstance(record, dict):
            return False
        attempts = int(record.get("controller_restart_attempts", 0) or 0)
        if attempts >= self._TRANSPORT_HANDOFF_CONTROLLER_RESTART_MAX_ATTEMPTS:
            return False
        attempts += 1
        record["controller_restart_attempts"] = attempts
        record["controller_restart_requested"] = True
        record["controller_restart_started_wall_monotonic"] = time.monotonic()
        record["controller_restart_server_disconnect_observed"] = False
        record["controller_restart_reason"] = str(reason or "transport_handoff_timeout")
        record["controller_restart_error"] = None
        # Force the action server to drop the recovery state.  The launch file
        # owns respawn, so this does not restart Gazebo, SLAM, or the map.
        client = record.get("client")
        cancel_all_goals = getattr(client, "cancel_all_goals", None)
        if callable(cancel_all_goals):
            try:
                cancel_all_goals()
            except Exception as exc:  # pragma: no cover - ROS shutdown edge
                record["controller_restart_error"] = "cancel_all_goals:%s" % exc
        self.action_server_seen = False
        try:
            process = subprocess.Popen(
                ["rosnode", "kill", "/move_base"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                close_fds=True,
            )
            record["controller_restart_pid"] = int(process.pid)
        except (OSError, ValueError) as exc:
            record["controller_restart_error"] = str(exc)
            record["controller_restart_pid"] = None
        self.publish_bridge_status(
            "transport_handoff_controller_restart_requested",
            route_id=int((record.get("old_contract") or {}).get("route_id", 0) or 0),
            action_generation=int(record.get("old_generation", 0) or 0),
            new_action_generation=int(record.get("new_generation", 0) or 0),
            transport_handoff="pending",
            controller_restart_attempts=attempts,
            controller_restart_timeout_seconds=(
                self._TRANSPORT_HANDOFF_CONTROLLER_RESTART_TIMEOUT_SECONDS
            ),
            reason=str(reason or "transport_handoff_timeout"),
            controller_restart_error=record.get("controller_restart_error"),
        )
        return True

    def _transport_action_state_locked(self, client=None):
        """Read the tracked action state without blocking the lifecycle tick."""
        if client is None:
            client = getattr(self, "action_client", None)
        getter = getattr(client, "get_state", None)
        if not callable(getter):
            return None
        try:
            return int(getter())
        except (TypeError, ValueError, RuntimeError):
            return None

    def _clear_terminal_prepare_if_matches_locked(self, record, status=None):
        """Drop the semantic prepare only after its transport has settled."""
        prepare = getattr(self, "pending_terminal_prepare", None)
        if not isinstance(prepare, dict) or not isinstance(record, dict):
            return False
        prepare_key = prepare.get("identity_key")
        record_key = record.get("identity_key")
        same_identity = bool(
            prepare_key is not None
            and record_key is not None
            and tuple(prepare_key) == tuple(record_key)
        )
        same_generation = int(
            prepare.get("old_generation", 0) or 0
        ) == int(record.get("old_generation", 0) or 0)
        if not same_identity and not same_generation:
            return False
        prepare["transport_terminal_observed"] = True
        if status is not None:
            try:
                prepare["transport_terminal_status"] = int(status)
            except (TypeError, ValueError):
                pass
        self.pending_terminal_prepare = None
        # ``pending_lease_release_contract`` is a separate semantic tombstone.
        # Global Frontier may publish its controller-lease release after the
        # actionlib callback, so transport completion must not erase the
        # contract needed to validate that later ACK.
        return True

    def _finish_transport_handoff_locked(self, status, source):
        """Release the successor gate after actionlib reports a terminal."""
        record = getattr(self, "transport_handoff_pending", None)
        if not isinstance(record, dict):
            return False
        try:
            status = int(status)
        except (TypeError, ValueError):
            return False
        if not self._transport_state_is_terminal(status):
            return False
        self.transport_handoff_pending = None
        self._clear_terminal_prepare_if_matches_locked(record, status)
        old_contract = dict(record.get("old_contract") or {})
        self.publish_bridge_status(
            "transport_terminal",
            route_id=int(old_contract.get("route_id", 0) or 0),
            transaction_id=int(old_contract.get("transaction_id", 0) or 0),
            lifecycle_transaction_id=int(
                old_contract.get("lifecycle_transaction_id", 0) or 0
            ),
            action_generation=int(record.get("old_generation", 0) or 0),
            new_action_generation=int(
                record.get("new_generation", 0) or 0
            ),
            transport_status=status,
            transport_status_text=GoalStatus.to_string(status),
            transport_handoff="complete",
            source=str(source or "unknown"),
            controller_lease="released",
        )
        lifecycle = getattr(self, "lifecycle_manager", None)
        enqueue = getattr(lifecycle, "enqueue_type", None)
        if callable(enqueue):
            from lifecycle_manager import EventType

            enqueue(EventType.BRIDGE_WAKE, None)
        else:
            schedule = getattr(self, "schedule_terminal_dispatch_locked", None)
            if callable(schedule):
                schedule()
        return True

    def _observe_transport_terminal_locked(self, generation, status, source):
        """Match a late predecessor callback to the transport gate."""
        record = getattr(self, "transport_handoff_pending", None)
        if not isinstance(record, dict):
            return False
        try:
            generation = int(generation)
            expected = int(record.get("old_generation", 0) or 0)
        except (TypeError, ValueError):
            return False
        if generation != expected:
            return False
        return self._finish_transport_handoff_locked(status, source)

    def _poll_transport_handoff_locked(self):
        """Poll actionlib once; never wait inside the bridge lifecycle tick."""
        record = getattr(self, "transport_handoff_pending", None)
        if not isinstance(record, dict):
            return False
        status = self._transport_action_state_locked(
            record.get("client") if isinstance(record, dict) else None
        )
        if status is not None and self._transport_state_is_terminal(status):
            return self._finish_transport_handoff_locked(status, "state_poll")
        now = now_for(self)
        wall_now = time.monotonic()
        started_wall = float(
            record.get("started_wall_monotonic", wall_now) or wall_now
        )
        restart_requested = bool(record.get("controller_restart_requested"))
        if restart_requested:
            client = record.get("client")
            server_available = self._transport_action_server_available_locked(
                client
            )
            if not server_available:
                record["controller_restart_server_disconnect_observed"] = True
            restart_started = float(
                record.get("controller_restart_started_wall_monotonic", wall_now)
                or wall_now
            )
            restart_age = max(0.0, wall_now - restart_started)
            if (
                server_available
                and record.get("controller_restart_server_disconnect_observed")
            ):
                # A respawned action server cannot retain the predecessor goal.
                # Treat an absent/nonterminal client state as LOST, then wake the
                # normal dispatch path only after this transport boundary closes.
                restart_status = self._transport_action_state_locked(client)
                if not self._transport_state_is_terminal(restart_status):
                    restart_status = GoalStatus.LOST
                return self._finish_transport_handoff_locked(
                    restart_status, "controller_restart"
                )
            if restart_age >= self._TRANSPORT_HANDOFF_CONTROLLER_RESTART_TIMEOUT_SECONDS:
                if self._request_transport_controller_restart_locked(
                    record, "controller_restart_timeout"
                ):
                    return True
                # Two restart requests have been issued.  Close the old lease as
                # LOST so the successor is no longer trapped behind an endless
                # PREEMPTING gate; dispatch itself still waits for a live server.
                self.publish_bridge_status(
                    "transport_handoff_failed",
                    route_id=int(
                        (record.get("old_contract") or {}).get("route_id", 0)
                        or 0
                    ),
                    action_generation=int(record.get("old_generation", 0) or 0),
                    new_action_generation=int(
                        record.get("new_generation", 0) or 0
                    ),
                    transport_handoff="forced_terminal",
                    transport_status=GoalStatus.LOST,
                    transport_status_text=GoalStatus.to_string(GoalStatus.LOST),
                    controller_restart_attempts=int(
                        record.get("controller_restart_attempts", 0) or 0
                    ),
                    reason="controller_restart_exhausted",
                )
                return self._finish_transport_handoff_locked(
                    GoalStatus.LOST, "controller_restart_exhausted"
                )
        elif wall_now - started_wall >= self._TRANSPORT_HANDOFF_TIMEOUT_SECONDS:
            if self._request_transport_controller_restart_locked(
                record, "transport_handoff_timeout"
            ):
                return True
        last_report = float(
            getattr(self, "transport_handoff_last_report_monotonic", 0.0) or 0.0
        )
        if now - last_report >= 2.0:
            self.transport_handoff_last_report_monotonic = now
            self.publish_bridge_status(
                "transport_handoff_waiting",
                route_id=int(
                    (record.get("old_contract") or {}).get("route_id", 0) or 0
                ),
                action_generation=int(record.get("old_generation", 0) or 0),
                new_action_generation=int(
                    record.get("new_generation", 0) or 0
                ),
                transport_status=status,
                transport_status_text=(
                    GoalStatus.to_string(status)
                    if status is not None
                    else "UNKNOWN"
                ),
                transport_handoff="pending",
                wait_seconds=round(
                    max(0.0, now - float(record.get("started_monotonic", now))),
                    3,
                ),
            )
        return True

    def _transport_handoff_blocks_dispatch_locked(self):
        """Return true while the predecessor action still owns the transport."""
        self._poll_transport_handoff_locked()
        return isinstance(
            getattr(self, "transport_handoff_pending", None), dict
        )

    def _begin_transport_handoff_locked(
        self, old_contract, old_generation, new_generation, reason, previous_active
    ):
        """Arm a non-blocking gate for an action being cancelled."""
        existing = getattr(self, "transport_handoff_pending", None)
        if isinstance(existing, dict):
            return existing
        status = self._transport_action_state_locked()
        needs_gate = bool(
            (previous_active and status is None)
            or (status is not None and not self._transport_state_is_terminal(status))
        )
        if not needs_gate:
            return None
        now = now_for(self)
        record = {
            "old_contract": dict(old_contract or {}),
            "old_generation": int(old_generation or 0),
            "new_generation": int(new_generation or 0),
            "reason": str(reason or "transport_handoff"),
            "previous_active": bool(previous_active),
            "status_before_cancel": status,
            "started_monotonic": now,
            "started_wall_monotonic": time.monotonic(),
            "handoff_timeout_seconds": self._TRANSPORT_HANDOFF_TIMEOUT_SECONDS,
            "controller_restart_requested": False,
            "controller_restart_attempts": 0,
            "controller_restart_server_disconnect_observed": False,
            # Keep polling the same SimpleActionClient across a warm-reset
            # state reinitialization; a fresh client has no knowledge of the
            # goal that is still being cancelled by the old client.
            "client": getattr(self, "action_client", None),
            "identity_key": self._action_identity_key_locked(old_contract),
        }
        self.transport_handoff_pending = record
        self.transport_handoff_last_report_monotonic = now
        self.publish_bridge_status(
            "transport_handoff_started",
            route_id=int((old_contract or {}).get("route_id", 0) or 0),
            action_generation=int(old_generation or 0),
            new_action_generation=int(new_generation or 0),
            transport_status=status,
            transport_status_text=(
                GoalStatus.to_string(status) if status is not None else "UNKNOWN"
            ),
            transport_handoff="pending",
            reason=str(reason or "transport_handoff"),
        )
        return record

    def _request_cancel_for_pending_goal_locked(self, reason, pending_delta):
        """Cancel once and let the terminal callback authorize the handoff."""
        if not self.action_active or self.handoff_requested:
            return
        self._commit_termination(
            "pending_goal_handoff",
            source_goal=getattr(self, "last_dispatched_goal", None),
            action_contract=getattr(self, "active_action_contract", None),
            watchdog_reason="pending_goal_handoff",
            publish_terminal=False,
            arm_watchdog=False,
        )
        self.handoff_requested = True
        self.schedule_terminal_dispatch_locked()
        self.publish_bridge_status(
            "handoff_requested",
            reason=reason,
            pending_delta=round(float(pending_delta), 3),
            from_source=self.active_intent_source,
            to_source=self.latest_intent_source,
            lifecycle="explicit_cancel_then_terminal_dispatch",
        )
        rospy.loginfo(
            "TEB goal bridge queued higher-priority goal behind action terminal: "
            "reason=%s pending_delta=%.2fm",
            reason,
            pending_delta,
        )

    def _execution_goal_locked(self, goal):
        """Return the frame-correct pose that the local planner must execute."""
        execution_goal = copy.deepcopy(goal)
        if (
            self.latest_route_kind == "frontier_turn_connector"
            and self.last_feedback_pose_global is not None
        ):
            execution_goal.pose.position.x = self.last_feedback_pose_global.pose.position.x
            execution_goal.pose.position.y = self.last_feedback_pose_global.pose.position.y
            execution_goal.header.stamp = rospy.Time.now()
        return execution_goal

    def _begin_action_lifecycle_locked(self, source_goal, execution_goal):
        """Reset state which is meaningful only for one MoveBase action."""
        cancel_watchdog = getattr(
            self, "_cancel_route_lease_watchdog_locked", None
        )
        if callable(cancel_watchdog):
            cancel_watchdog("new_dispatch")
        self._clear_failed_route_lease_locked()
        pending_generation = int(
            getattr(self, "pending_dispatch_generation", 0) or 0
        )
        if pending_generation > 0:
            generation = max(
                pending_generation,
                int(getattr(self, "action_generation", 0) or 0),
            )
            self.action_generation = generation
            self.pending_dispatch_generation = 0
        else:
            self.action_generation += 1
            generation = self.action_generation
        self.last_dispatched_goal = copy.deepcopy(source_goal)
        self._remember_last_dispatch_identity_locked()
        self.active_portal_source_goal = (
            copy.deepcopy(source_goal)
            if self.active_route_kind == "portal_transition"
            else None
        )
        self.last_terminal_goal = None
        self.last_dispatch_monotonic = now_for(self)
        self.last_result_status = None
        self.action_active = True
        self.active_intent_source = self.latest_intent_source
        self.active_intent_priority = self.latest_intent_priority
        self.active_goal_transaction_id = int(self.latest_goal_transaction_id)
        self.active_route_kind = self.latest_route_kind
        self.active_mission_route_kind = self.latest_mission_route_kind
        self.active_route_id = int(self.latest_route_id)
        self.active_target_epoch = int(self.latest_target_epoch)
        self.active_route_map_epoch = getattr(
            self, "latest_route_map_epoch", None
        )
        self.active_origin_epoch = getattr(self, "latest_origin_epoch", None)
        self.active_projection_epoch = getattr(
            self, "latest_projection_epoch", None
        )
        self.active_graph_transaction_id = int(
            getattr(self, "latest_graph_transaction_id", 0) or 0
        )
        self.active_graph_action = str(
            getattr(self, "latest_graph_action", "") or ""
        )
        self.active_graph_obligation_kind = str(
            getattr(self, "latest_graph_obligation_kind", "") or ""
        )
        self.active_frontier_map_epoch = (
            getattr(self, "latest_frontier_map_epoch", None)
            if str(
                getattr(self, "latest_intent_source", "unknown") or "unknown"
            ).strip().lower() == "global_slam_frontier"
            else None
        )
        self.active_target_track_id = self.latest_target_track_id
        self.active_target_viewpoint_candidate_id = str(
            getattr(self, "latest_target_viewpoint_candidate_id", "") or ""
        )
        self.active_target_viewpoint_attempt_id = str(
            getattr(self, "latest_target_viewpoint_attempt_id", "") or ""
        )
        self.active_goal_context = dict(self.latest_goal_context)
        self.handoff_requested = False
        self.frontier_observation_completion_pending = None
        self.active_goal_global = copy.deepcopy(execution_goal)
        self.active_feedback_distance = None
        # Feedback callbacks are action-scoped.  In-place replacement does
        # not pass through ``_clear_action_health_locked``, so explicitly
        # discard the previous action's pose before a new Navfn plan can
        # arrive.  Otherwise ``on_navfn_plan`` may project stale feedback
        # onto the new route and start its health clock with false progress.
        self.active_feedback_pose = None
        self.active_feedback_frame = ""
        self.last_feedback_pose_global = None
        self.feedback_transform_failures = 0
        self.active_best_distance = None
        self.active_progress_monotonic = now_for(self)
        self.active_motion_reference = None
        self.active_motion_progress_monotonic = now_for(self)
        self.active_navfn_plan_points = []
        self.active_navfn_plan_endpoint = None
        self.active_navfn_remaining = None
        self.active_navfn_best_remaining = None
        self.active_navfn_progress_monotonic = 0.0
        self.last_feedback_monotonic = 0.0
        self._reset_teb_reorientation_locked()
        self.move_base_terminal_pending = False
        # A later semantic update at these coordinates remains observable, but
        # repeated timer evaluations must not be treated as new work.
        self.deferred_signature = None
        # Persistent execution may adopt a newer semantic route while this
        # MoveBase lease is still active. Keep the transport callback bound to
        # the route and mission metadata that were dispatched for this
        # generation; the mutable active_* fields describe the latest stream
        # state and are not a terminal identity.
        self.active_action_contract = MappingProxyType({
            "generation": int(generation),
            "transaction_id": int(self.latest_goal_transaction_id),
            "lifecycle_transaction_id": int(
                self._canonical_lifecycle_transaction_id()
            ),
            "route_id": int(self.latest_route_id),
            "graph_transaction_id": int(
                getattr(self, "latest_graph_transaction_id", 0) or 0
            ),
            "epoch": self._effective_action_epoch_locked(),
            "map_epoch": self.active_route_map_epoch,
            "origin_epoch": self.active_origin_epoch,
            "projection_epoch": self.active_projection_epoch,
            "route_kind": str(self.latest_route_kind or ""),
            "mission_route_kind": str(self.latest_mission_route_kind or ""),
            "graph_action": str(
                getattr(self, "latest_graph_action", "") or ""
            ),
            "graph_obligation_kind": str(
                getattr(self, "latest_graph_obligation_kind", "") or ""
            ),
            "source": str(self.latest_intent_source or "unknown"),
            "priority": int(self.latest_intent_priority),
            "source_goal": copy.deepcopy(source_goal),
            "execution_goal": copy.deepcopy(execution_goal),
            "target_epoch": int(self.latest_target_epoch),
            "target_track_id": str(self.latest_target_track_id or ""),
            "target_viewpoint_candidate_id": str(
                getattr(self, "latest_target_viewpoint_candidate_id", "") or ""
            ),
            "target_viewpoint_attempt_id": str(
                getattr(self, "latest_target_viewpoint_attempt_id", "") or ""
            ),
            "goal_context": copy.deepcopy(dict(self.latest_goal_context)),
        })
        invalidate_feedback = getattr(
            self, "_invalidate_teb_feedback_locked", None
        )
        if callable(invalidate_feedback):
            invalidate_feedback("new_action_dispatch", clear_planner=True)
        self.dispatch_count += 1
        return generation

    def _reserve_persistent_dispatch_generation_locked(self):
        """Reserve the generation before the planner receives a mission command."""
        if self.action_active:
            return int(self.action_generation)
        pending = int(getattr(self, "pending_dispatch_generation", 0) or 0)
        if pending > int(getattr(self, "action_generation", 0) or 0):
            return pending
        self.action_generation = int(getattr(self, "action_generation", 0) or 0) + 1
        self.pending_dispatch_generation = int(self.action_generation)
        return int(self.action_generation)

    def _action_contract_for_generation_locked(self, generation):
        """Return the immutable dispatch contract for one action callback."""
        contract = getattr(self, "active_action_contract", None)
        if contract is None:
            return None
        try:
            contract_generation = int(contract["generation"])
        except (KeyError, TypeError, ValueError):
            return None
        if contract_generation != int(generation):
            return None
        return contract

    def _send_goal_locked(self, source_message, reason, replacement=False,
                          replacement_kind=None, **event_fields):
        """Send one goal, optionally replacing an action without a stop phase.

        ``SimpleActionClient.send_goal`` drops the old client-side handle but
        does not cancel the server's execute loop.  That is required for smooth
        visual-servo handoffs; explicit cancellation is used only by the
        dispatch policy when a terminal handoff is required.
        """
        if source_message is None or not self._is_active_mode():
            return False
        if self._transport_handoff_blocks_dispatch_locked():
            self.publish_bridge_status(
                "dispatch_deferred_transport_handoff",
                reason=str(reason or "dispatch"),
                transport_handoff="pending",
            )
            return False
        source_goal = self._normalize_goal(source_message)
        goal = self._goal_in_global_frame(source_goal)
        if goal is None:
            return False

        execution_goal = self._execution_goal_locked(goal)
        action_goal = MoveBaseGoal()
        action_goal.target_pose = execution_goal
        generation = self._begin_action_lifecycle_locked(
            source_goal, execution_goal
        )
        if bool(getattr(self, "persistent_execution", False)):
            # A persistent MoveBase restart advances action_generation while
            # keeping the semantic mission transaction. Re-announce the
            # complete route contract before actionlib can ask Navfn/TEB for a
            # command; otherwise the planner keeps publishing the predecessor
            # generation and the strict downstream identity gates correctly
            # reject every command from the new action.
            republish_mission = getattr(
                self, "_publish_persistent_mission_goal_locked", None
            )
            # A new target request is a two-phase transaction.  The action goal
            # gives StreamingNavfnPlanner a planning cycle in which to validate
            # the target; publishing the approved mission here would make
            # ``onMissionCommand`` clear that still-provisional request before
            # ``makePlan`` can emit ``target_plan_installed``.  The install
            # acknowledgement publishes the approved mission itself.  An
            # already-installed target, or an ordinary frontier route, still
            # needs the generation rebind immediately.
            target_plan_pending = bool(
                int(
                    getattr(
                        self, "persistent_target_pending_transaction", 0
                    )
                    or 0
                )
                > 0
                and int(getattr(self, "latest_intent_priority", 0) or 0) >= 2
            )
            if callable(republish_mission) and not target_plan_pending:
                republish_mission("action_generation_rebind")
            elif target_plan_pending:
                self.publish_bridge_status(
                    "persistent_target_mission_approval_deferred",
                    reason="await_target_plan_installation",
                    transaction_id=int(
                        getattr(self, "persistent_target_pending_transaction", 0)
                        or 0
                    ),
                    action_generation=int(self.action_generation),
                )
        dispatch_contract = self.active_action_contract or {}
        self.action_client.send_goal(
            action_goal,
            done_cb=lambda status, result: self.on_done(generation, status, result),
            active_cb=lambda: self.on_active(generation),
            feedback_cb=lambda feedback: self.on_feedback(generation, feedback),
        )
        dispatch_fields = {
            "reason": reason,
            "action_generation": int(generation),
            "transaction_id": int(self.active_goal_transaction_id),
            "epoch": int(
                dispatch_contract.get("epoch", self.active_target_epoch) or 0
            ),
            "map_epoch": dispatch_contract.get("map_epoch"),
            "origin_epoch": dispatch_contract.get("origin_epoch"),
            "projection_epoch": dispatch_contract.get("projection_epoch"),
            "lifecycle_transaction_id": int(
                dispatch_contract.get("lifecycle_transaction_id", 0)
                or 0
            ),
            "route_id": int(self.active_route_id),
            "graph_transaction_id": int(
                getattr(self, "active_graph_transaction_id", 0) or 0
            ),
            "mission_route_kind": self.active_mission_route_kind,
            "source_goal": [
                round(source_goal.pose.position.x, 3),
                round(source_goal.pose.position.y, 3),
            ],
            "dispatched_goal": [
                round(execution_goal.pose.position.x, 3),
                round(execution_goal.pose.position.y, 3),
            ],
            "handoffs": int(self.handoff_count),
            "intent_source": self.active_intent_source,
            "intent_priority": int(self.active_intent_priority),
            "route_kind": self.active_route_kind,
            "goal_context": self.active_goal_context,
            "execution_goal": [
                round(execution_goal.pose.position.x, 3),
                round(execution_goal.pose.position.y, 3),
                round(self._yaw(execution_goal), 4),
            ],
            "replacement": bool(replacement),
            "replacement_kind": (replacement_kind or "none"),
        }
        dispatch_fields.update(event_fields)
        if (
            bool(getattr(self, "persistent_execution", False))
            and str(
                getattr(self, "active_intent_source", "unknown") or "unknown"
            ).strip().lower() == "global_slam_frontier"
            and int(getattr(self, "active_intent_priority", 0) or 0) == 0
        ):
            # This is the transport adoption boundary: the semantic mission
            # has been accepted into the persistent MoveBase action. Keep it
            # distinct from later path rollovers so the short experiment gate
            # can require both adoption and dispatch for one identity.
            adoption_fields = dict(dispatch_fields)
            adoption_fields["adoption_kind"] = "move_base_action"
            self.publish_bridge_status(
                "persistent_mission_goal_adopted",
                **adoption_fields,
            )
        self.publish_bridge_status("dispatch", **dispatch_fields)
        predecessor_ack = getattr(self, "pending_successor_release_ack", None)
        if (
            isinstance(predecessor_ack, dict)
            and self.active_intent_source == "global_slam_frontier"
            and int(self.active_intent_priority) == 0
            and int(self.active_route_id) > int(predecessor_ack.get("route_id", 0) or 0)
        ):
            self.publish_bridge_status(
                "successor_dispatched",
                predecessor_route_id=int(predecessor_ack["route_id"]),
                predecessor_route_kind=str(
                    predecessor_ack.get("route_kind", "") or ""
                ),
                predecessor_lifecycle_transaction_id=int(
                    predecessor_ack.get("lifecycle_transaction_id", 0) or 0
                ),
                predecessor_action_generation=int(
                    predecessor_ack.get("action_generation", 0) or 0
                ),
                predecessor_new_action_generation=int(
                    predecessor_ack.get("new_action_generation", 0) or 0
                ),
                predecessor_graph_transaction_id=int(
                    predecessor_ack.get("graph_transaction_id", 0) or 0
                ),
                predecessor_map_epoch=predecessor_ack.get("map_epoch"),
                predecessor_origin_epoch=predecessor_ack.get("origin_epoch"),
                predecessor_projection_epoch=predecessor_ack.get(
                    "projection_epoch"
                ),
                release_ack="exact",
                **dispatch_fields
            )
            self.pending_successor_release_ack = None
        rospy.loginfo(
            "TEB goal bridge %s move_base action: reason=%s frame=%s "
            "target=(%.2f,%.2f) source_frame=%s source=(%.2f,%.2f)",
            "replaced" if replacement else "dispatched",
            reason,
            goal.header.frame_id,
            execution_goal.pose.position.x,
            execution_goal.pose.position.y,
            source_goal.header.frame_id,
            source_goal.pose.position.x,
            source_goal.pose.position.y,
        )
        return True

    def on_active(self, generation):
        with self.lock:
            if generation != self.action_generation:
                self.publish_bridge_status(
                    "stale_action_callback_ignored",
                    callback="active",
                    callback_action_generation=int(generation),
                    current_action_generation=int(self.action_generation),
                    reason="generation_mismatch",
                )
                return
            self.action_active = True
            self.publish_bridge_status("active")
