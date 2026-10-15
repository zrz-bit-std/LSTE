"""Timer-driven persistent-mission progression for the TEB bridge."""

import copy
from types import MappingProxyType

import rospy

from clock_provider import now_for
from lifecycle_manager import EventType


class TebGoalBridgeMissionRuntimeMixin:
    def on_timer(self, _event):
        with self.lock:
            poll_transport = getattr(
                self, "_poll_transport_handoff_locked", None
            )
            if callable(poll_transport):
                poll_transport()
            tick_watchdog = getattr(
                self, "_tick_route_lease_watchdog_locked", None
            )
            if callable(tick_watchdog):
                tick_watchdog()
            if self._is_active_mode():
                if self.persistent_execution:
                    # Persistent execution has one long-lived action client,
                    # so it must still run the same health FSM as endpoint
                    # execution. Otherwise an invalidated TEB feedback stream
                    # can leave an active route emitting zero forever.
                    self.maybe_handoff_locked()
                    if self.action_active:
                        self._promote_persistent_frontier_prefetch_locked()
                else:
                    self.maybe_handoff_locked()
                self.dispatch_locked(force=False, reason="coalesced_global_goal")

    def _refresh_persistent_action_contract_locked(self, semantic_goal):
        """Bind transport terminal identity to the current stream route.

        Persistent execution keeps one MoveBase action alive while the
        semantic route changes underneath it. The callback generation stays
        the same, but its route contract must follow the semantic lease that
        is actually running; otherwise a later transport failure is reported
        against the predecessor route and can replay the successor.
        """
        contract = getattr(self, "active_action_contract", None)
        if contract is None:
            return
        updated = dict(contract)
        updated.update(
            {
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
                "mission_route_kind": str(
                    self.latest_mission_route_kind or ""
                ),
                "graph_action": str(
                    getattr(self, "latest_graph_action", "") or ""
                ),
                "graph_obligation_kind": str(
                    getattr(self, "latest_graph_obligation_kind", "") or ""
                ),
                "source": str(self.latest_intent_source or "unknown"),
                "priority": int(self.latest_intent_priority),
                "source_goal": copy.deepcopy(self.latest_goal),
                "execution_goal": copy.deepcopy(semantic_goal),
                "target_epoch": int(self.latest_target_epoch),
                "target_track_id": str(self.latest_target_track_id or ""),
                "target_viewpoint_candidate_id": str(
                    getattr(
                        self, "latest_target_viewpoint_candidate_id", ""
                    )
                    or ""
                ),
                "target_viewpoint_attempt_id": str(
                    getattr(self, "latest_target_viewpoint_attempt_id", "")
                    or ""
                ),
                "goal_context": copy.deepcopy(
                    dict(self.latest_goal_context)
                ),
            }
        )
        self.active_action_contract = MappingProxyType(updated)

    def _adopt_persistent_mission_goal_locked(self):
        """Advance the semantic route while retaining the actionlib lease."""
        if self.latest_goal is None or not self.action_active:
            return
        semantic_goal = self._goal_in_global_frame(self.latest_goal)
        if semantic_goal is None:
            return
        previous_route_kind = self.active_route_kind
        previous_route_id = int(self.active_route_id)
        previous_transaction_id = int(self.active_goal_transaction_id)
        previous_epoch = int(self.active_target_epoch)
        previous_graph_transaction_id = int(
            getattr(self, "active_graph_transaction_id", 0) or 0
        )
        previous_source = str(
            getattr(self, "active_intent_source", "unknown") or "unknown"
        ).strip().lower()
        previous_priority = int(getattr(self, "active_intent_priority", 0) or 0)
        previous_mission_route_kind = str(
            getattr(self, "active_mission_route_kind", "") or ""
        ).strip().lower()
        previous_map_epoch = getattr(self, "active_route_map_epoch", None)
        previous_origin_epoch = getattr(self, "active_origin_epoch", None)
        previous_projection_epoch = getattr(
            self, "active_projection_epoch", None
        )
        previous_action_contract = getattr(
            self, "active_action_contract", None
        )
        try:
            previous_lifecycle_transaction_id = int(
                (previous_action_contract or {}).get(
                    "lifecycle_transaction_id", 0
                )
                or 0
            )
        except (AttributeError, TypeError, ValueError):
            previous_lifecycle_transaction_id = 0
        current_lifecycle_transaction_id = int(
            self._canonical_lifecycle_transaction_id()
        )
        previous_target_track_id = str(
            getattr(self, "active_target_track_id", "") or ""
        )
        route_changed = bool(
            previous_route_id != int(self.latest_route_id)
            or previous_transaction_id != int(self.latest_goal_transaction_id)
            or previous_epoch != int(self.latest_target_epoch)
            or previous_graph_transaction_id
            != int(getattr(self, "latest_graph_transaction_id", 0) or 0)
            or previous_route_kind != self.latest_route_kind
            or previous_source
            != str(self.latest_intent_source or "unknown").strip().lower()
            or previous_priority != int(self.latest_intent_priority)
            or previous_mission_route_kind
            != str(self.latest_mission_route_kind or "").strip().lower()
            or previous_map_epoch != getattr(self, "latest_route_map_epoch", None)
            or previous_origin_epoch
            != getattr(self, "latest_origin_epoch", None)
            or previous_projection_epoch
            != getattr(self, "latest_projection_epoch", None)
            or previous_lifecycle_transaction_id
            != current_lifecycle_transaction_id
            or previous_target_track_id
            != str(self.latest_target_track_id or "")
        )
        predecessor_endpoint_pending = getattr(
            self, "persistent_frontier_endpoint_pending", None
        )
        if not (
            isinstance(predecessor_endpoint_pending, dict)
            and int(predecessor_endpoint_pending.get("route_id", 0) or 0)
            == previous_route_id
            and int(self.latest_route_id) > previous_route_id
        ):
            predecessor_endpoint_pending = None
        if route_changed:
            if predecessor_endpoint_pending is not None:
                self.persistent_frontier_endpoint_pending = None
            invalidate_feedback = getattr(
                self, "_invalidate_teb_feedback_locked", None
            )
            if callable(invalidate_feedback):
                invalidate_feedback("route_owner_changed", clear_planner=True)
            cancel_watchdog = getattr(
                self, "_cancel_route_lease_watchdog_locked", None
            )
            if callable(cancel_watchdog):
                cancel_watchdog("new_route_adopted")
        self.active_goal_global = copy.deepcopy(semantic_goal)
        self.last_dispatched_goal = copy.deepcopy(semantic_goal)
        # Persistent execution changes the semantic route without creating a
        # new actionlib goal. Keep idle retry diagnostics aligned with the
        # latest route represented by ``last_dispatched_goal``.
        self._remember_last_dispatch_identity_locked()
        self.active_intent_source = self.latest_intent_source
        self.active_intent_priority = int(self.latest_intent_priority)
        self.active_goal_transaction_id = int(self.latest_goal_transaction_id)
        self.active_route_kind = self.latest_route_kind
        self.active_mission_route_kind = self.latest_mission_route_kind
        self.active_graph_action = str(
            getattr(self, "latest_graph_action", "") or ""
        )
        self.active_graph_obligation_kind = str(
            getattr(self, "latest_graph_obligation_kind", "") or ""
        )
        self.active_route_id = int(self.latest_route_id)
        self.active_graph_transaction_id = int(
            getattr(self, "latest_graph_transaction_id", 0) or 0
        )
        if self.active_route_kind == "portal_transition":
            if (
                previous_route_kind != "portal_transition"
                or previous_route_id != self.active_route_id
                or getattr(self, "active_portal_source_goal", None) is None
            ):
                self.active_portal_source_goal = copy.deepcopy(self.latest_goal)
        else:
            self.active_portal_source_goal = None
        self.active_target_epoch = int(self.latest_target_epoch)
        self.active_route_map_epoch = getattr(
            self, "latest_route_map_epoch", None
        )
        # A persistent action keeps its transport generation, but the planner
        # contract still belongs to the newly adopted map snapshot. Refresh
        # these epochs together with the semantic route before the next
        # identity-bearing planner frame is admitted.
        self.active_origin_epoch = getattr(
            self, "latest_origin_epoch", None
        )
        self.active_projection_epoch = getattr(
            self, "latest_projection_epoch", None
        )
        self.active_frontier_map_epoch = getattr(
            self, "latest_frontier_map_epoch", None
        ) if self.active_intent_source == "global_slam_frontier" else None
        self.active_target_track_id = self.latest_target_track_id
        self.active_target_viewpoint_candidate_id = str(
            getattr(self, "latest_target_viewpoint_candidate_id", "") or ""
        )
        self.active_target_viewpoint_attempt_id = str(
            getattr(self, "latest_target_viewpoint_attempt_id", "") or ""
        )
        self.active_goal_context = dict(self.latest_goal_context)
        self.active_predecessor_contract = (
            None
            if getattr(self, "latest_predecessor_contract", None) is None
            else dict(self.latest_predecessor_contract)
        )
        self._refresh_persistent_action_contract_locked(semantic_goal)
        self.active_best_distance = None
        self.active_progress_monotonic = now_for(self)
        self.active_motion_reference = None
        self.active_motion_progress_monotonic = now_for(self)
        self.active_navfn_plan_points = []
        self.active_navfn_plan_endpoint = None
        self.active_navfn_remaining = None
        self.active_navfn_best_remaining = None
        self.active_navfn_progress_monotonic = 0.0
        self.publish_bridge_status(
            "persistent_mission_path_adopted",
            action_generation=int(self.action_generation),
            route_id=int(self.active_route_id),
            priority=int(self.active_intent_priority),
            source=self.active_intent_source,
            transaction_id=int(self.latest_goal_transaction_id),
            lifecycle_transaction_id=int(
                self._canonical_lifecycle_transaction_id()
            ),
            graph_transaction_id=int(self.active_graph_transaction_id),
            epoch=int(self.latest_target_epoch),
            map_epoch=self.active_route_map_epoch,
            origin_epoch=self.active_origin_epoch,
            projection_epoch=self.active_projection_epoch,
            route_kind=str(self.active_route_kind or ""),
            mission_route_kind=str(self.active_mission_route_kind or ""),
            graph_action=str(getattr(self, "active_graph_action", "") or ""),
            graph_obligation_kind=str(
                getattr(self, "active_graph_obligation_kind", "") or ""
            ),
            predecessor_contract=(
                None
                if self.active_predecessor_contract is None
                else dict(self.active_predecessor_contract)
            ),
            target_track_id=self.active_target_track_id,
            goal_context=self.active_goal_context,
            goal=[
                round(float(semantic_goal.pose.position.x), 3),
                round(float(semantic_goal.pose.position.y), 3),
            ],
        )
        if predecessor_endpoint_pending is not None:
            self.publish_bridge_status(
                "persistent_frontier_successor_adopted",
                predecessor_route_id=previous_route_id,
                predecessor_action_generation=int(
                    predecessor_endpoint_pending.get("action_generation", 0)
                    or 0
                ),
                predecessor_transaction_id=int(
                    predecessor_endpoint_pending.get("transaction_id", 0)
                    or 0
                ),
                predecessor_lifecycle_transaction_id=int(
                    predecessor_endpoint_pending.get(
                        "lifecycle_transaction_id", 0
                    )
                    or 0
                ),
                route_id=int(self.active_route_id),
                action_generation=int(self.action_generation),
                transaction_id=int(self.latest_goal_transaction_id),
                lifecycle_transaction_id=int(
                    self._canonical_lifecycle_transaction_id()
                ),
                graph_transaction_id=int(self.active_graph_transaction_id),
                map_epoch=self.active_route_map_epoch,
                origin_epoch=self.active_origin_epoch,
                projection_epoch=self.active_projection_epoch,
                controller_lease="held",
                semantic_only=True,
            )

    def _promote_persistent_frontier_prefetch_locked(self):
        """Release one pre-admitted successor before a route-end stop."""
        if (
            not self.persistent_execution
            or not self.action_active
            or self.task_done
            or self.active_intent_priority != 0
            or self.latest_intent_priority != 0
            or self.active_intent_source != "global_slam_frontier"
            or self.latest_intent_source != "global_slam_frontier"
            or self.active_route_kind != "frontier_endpoint"
            or self.active_route_id <= 0
            or self.active_feedback_distance is None
            or self.active_feedback_distance
            > (
                self.persistent_frontier_lookahead_trigger_distance
                if self.persistent_frontier_lookahead_handoff_enabled
                else self.frontier_observation_completion_radius
            )
            or self.last_dispatched_goal is None
            or self.prefetched_frontier_goal is None
            or self.prefetched_frontier_route_id != self.active_route_id + 1
        ):
            return False
        route_pair = (int(self.active_route_id), int(self.prefetched_frontier_route_id))
        if route_pair in self.persistent_frontier_prefetch_promoted_pairs:
            return False
        admission = None
        if self.persistent_frontier_lookahead_handoff_enabled:
            admitted, admission = self._admit_persistent_frontier_prefetch_locked()
            if not admitted:
                now = now_for(self)
                if now - self.persistent_prefetch_admission_last_report_monotonic >= 1.0:
                    self.persistent_prefetch_admission_last_report_monotonic = now
                    self.publish_bridge_status(
                        "persistent_frontier_prefetch_admission_deferred",
                        **admission,
                    )
                return False
        elif not self._frontier_observation_speed_ready_locked():
            # Legacy persistent behavior waits for TEB's terminal speed envelope.
            return False
        source_goal = copy.deepcopy(self.last_dispatched_goal)
        feedback_distance = float(self.active_feedback_distance)
        selected_linear_speed = getattr(self, "latest_teb_selected_linear", None)
        self.persistent_frontier_prefetch_promoted_pairs.add(route_pair)
        termination = self._commit_termination(
            "persistent_frontier_prefetch_progress",
            source_goal=source_goal,
            action_contract=getattr(self, "active_action_contract", None),
            watchdog_reason="persistent_frontier_prefetch_progress",
        )
        if not termination.get("committed", False):
            self.persistent_frontier_prefetch_promoted_pairs.discard(route_pair)
            return False
        self._clear_target_failure_locked("persistent_frontier_prefetch_progress")
        # The terminal event releases the successor. The successor is admitted
        # only after Global Frontier acknowledges the predecessor release.
        self.terminal_count += 1
        transition_kind = (
            str(admission.get("transition_kind", "smooth_handoff"))
            if isinstance(admission, dict)
            else "smooth_handoff"
        )
        self.publish_bridge_status(
            (
                "persistent_frontier_curve_handoff_promoted"
                if transition_kind == "curve_handoff"
                else (
                    "persistent_frontier_lookahead_handoff_promoted"
                    if self.persistent_frontier_lookahead_handoff_enabled
                    else "persistent_frontier_prefetch_promoted"
                )
            ),
            route_id=route_pair[0],
            successor_route_id=route_pair[1],
            feedback_distance=round(feedback_distance, 3),
            completion_radius=round(self.frontier_observation_completion_radius, 3),
            lookahead_trigger_distance=round(
                self.persistent_frontier_lookahead_trigger_distance, 3
            ),
            selected_linear_speed=(
                None
                if selected_linear_speed is None
                else round(float(selected_linear_speed), 3)
            ),
            completion_max_linear_speed=round(
                float(self.frontier_observation_completion_max_linear_speed), 3
            ),
            lifecycle=(
                "pre_admitted_%s_then_streaming_plan_update" % transition_kind
                if self.persistent_frontier_lookahead_handoff_enabled
                else "logical_terminal_then_streaming_plan_update"
            ),
            transition_kind=transition_kind,
            admission=admission,
        )
        rospy.loginfo(
            "TEB goal bridge promoted persistent frontier successor: "
            "route_id=%d successor_route_id=%d distance=%.2fm lookahead=%s",
            route_pair[0],
            route_pair[1],
            feedback_distance,
            self.persistent_frontier_lookahead_handoff_enabled,
        )
        return True

    def on_terminal_timer(self, _event):
        with self.lock:
            self.terminal_dispatch_timer = None
            if self._is_active_mode():
                self.dispatch_locked(force=False, reason="terminal_followup")

    def schedule_terminal_dispatch_locked(self):
        lifecycle = getattr(self, "lifecycle_manager", None)
        if lifecycle is not None:
            self.terminal_dispatch_timer = None
            lifecycle.enqueue_type(EventType.BRIDGE_WAKE, None)
            return
        if self.terminal_dispatch_timer is None and not rospy.is_shutdown():
            self.terminal_dispatch_timer = rospy.Timer(
                # This is a lifecycle synchronization barrier, not a planning
                # delay. It gives actionlib one callback turn to finish DONE.
                rospy.Duration(0.10), self.on_terminal_timer, oneshot=True
            )
