"""Inbound mission-goal messages for the TEB action bridge.

This module owns conversion and admission of GoalManager's atomic
``mission_goal`` transactions.  It deliberately does not own action dispatch
or timer lifecycle; those concerns live in sibling modules.
"""

import json
import math

import rospy
from geometry_msgs.msg import PoseStamped

from goal_context import normalize_goal_context


class TebGoalBridgeMissionInputMixin:
    def on_goal(self, message):
        """Receive the legacy pose-only topic when atomic commands are disabled."""
        if self.use_goal_command:
            return
        with self.lock:
            self.latest_goal = self._normalize_goal(message)
            if self._is_active_mode():
                if not self._intent_matches_goal_locked(self.latest_goal):
                    intent_x = (
                        float(self.latest_intent_goal[0])
                        if self.latest_intent_goal is not None else float("nan")
                    )
                    intent_y = (
                        float(self.latest_intent_goal[1])
                        if self.latest_intent_goal is not None else float("nan")
                    )
                    rospy.loginfo_throttle(
                        2.0,
                        "TEB goal bridge waiting for matching goal intent before "
                        "dispatch: pose=(%.2f,%.2f) intent=(%.2f,%.2f)",
                        self.latest_goal.pose.position.x,
                        self.latest_goal.pose.position.y,
                        intent_x,
                        intent_y,
                    )
                    return
                self.dispatch_locked(force=False, reason="global_goal_changed")

    def on_goal_command(self, message):
        """Accept one atomic GoalManager pose-plus-intent transaction."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            rospy.logwarn_throttle(
                3.0, "TEB goal bridge ignored malformed mission_goal transaction"
            )
            return
        if not isinstance(payload, dict) or payload.get("event") != "mission_goal":
            if (
                isinstance(payload, dict)
                and payload.get("event") == "target_terminal_observation"
            ):
                try:
                    transaction_id = max(
                        0, int(payload.get("transaction_id", 0) or 0)
                    )
                    target_epoch = max(
                        0, int(payload.get("target_epoch", 0) or 0)
                    )
                    map_epoch = max(
                        0, int(payload.get("map_epoch", 0) or 0)
                    )
                except (TypeError, ValueError):
                    return
                with self.lock:
                    try:
                        lifecycle_transaction_id = max(
                            0,
                            int(
                                payload.get(
                                    "lifecycle_transaction_id", 0
                                )
                                or 0
                            ),
                        )
                    except (TypeError, ValueError):
                        lifecycle_transaction_id = 0
                    tombstone_transaction = int(
                        getattr(
                            self, "target_lease_tombstone_transaction_id", 0
                        )
                        or 0
                    )
                    tombstone_epoch = int(
                        getattr(self, "target_lease_tombstone_epoch", 0) or 0
                    )
                    tombstone_track = str(
                        getattr(self, "target_lease_tombstone_track_id", "")
                        or ""
                    ).strip()
                    incoming_track = str(
                        payload.get("target_track_id", "") or ""
                    ).strip()
                    if (
                        transaction_id > 0
                        and tombstone_transaction > 0
                        and transaction_id <= tombstone_transaction
                    ) or (
                        tombstone_epoch > 0
                        and target_epoch > 0
                        and target_epoch <= tombstone_epoch
                        and tombstone_track
                        and incoming_track == tombstone_track
                    ):
                        self.publish_bridge_status(
                            "target_terminal_observation_ignored",
                            reason="released_target_transaction",
                            received_transaction_id=transaction_id,
                            tombstone_transaction_id=tombstone_transaction,
                            target_epoch=target_epoch,
                            tombstone_epoch=tombstone_epoch,
                            target_track_id=incoming_track,
                        )
                        return
                    target_failure_guard = getattr(
                        self, "_target_failure_blocks_identity_locked", None
                    )
                    if callable(target_failure_guard) and target_failure_guard(
                        2,
                        incoming_track,
                        target_epoch,
                        "target_terminal_observation",
                    ):
                        return
                    if (
                        transaction_id > 0
                        and self.latest_goal_transaction_id > 0
                        and transaction_id < self.latest_goal_transaction_id
                    ):
                        self.publish_bridge_status(
                            "target_terminal_observation_ignored",
                            reason="stale_transaction",
                            received_transaction_id=transaction_id,
                            latest_transaction_id=int(
                                self.latest_goal_transaction_id
                            ),
                        )
                        return
                    if self.action_active:
                        self.cancel_locked("target_terminal_observation")
                    self._clear_persistent_target_request_locked(
                        "target_terminal_observation", force=True
                    )
                    # The terminal-observation intent is an ownership
                    # boundary, not a goal-less retry of the previous target.
                    # Clear the latched pose before the timer can coalesce and
                    # redispatch the action that was just cancelled.
                    self.latest_goal = None
                    self.intent_seen = True
                    self.latest_intent_source = "target_terminal_observation"
                    self.latest_intent_priority = 2
                    self.latest_route_kind = ""
                    self.latest_mission_route_kind = ""
                    self.latest_route_map_epoch = map_epoch or None
                    self.latest_graph_transaction_id = 0
                    self.latest_graph_action = ""
                    self.latest_graph_obligation_kind = ""
                    self.latest_route_id = 0
                    self.latest_intent_goal = None
                    self.latest_target_epoch = target_epoch
                    self.latest_target_track_id = str(
                        payload.get("target_track_id", "")
                    ).strip()
                    self.latest_target_viewpoint_candidate_id = str(
                        payload.get("target_viewpoint_candidate_id", "")
                    ).strip()
                    self.latest_target_viewpoint_attempt_id = str(
                        payload.get("target_viewpoint_attempt_id", "")
                    ).strip()
                    self.latest_goal_context = normalize_goal_context(
                        payload.get("goal_context")
                    )
                    self.latest_goal_transaction_id = transaction_id
                    self.latest_lifecycle_transaction_id = (
                        lifecycle_transaction_id
                    )
                    self.publish_bridge_status(
                        "target_terminal_observation_started",
                        transaction_id=transaction_id,
                        target_epoch=target_epoch,
                        target_track_id=self.latest_target_track_id,
                        reason=str(
                            payload.get(
                                "reason", "target_terminal_observation"
                            )
                        ),
                    )
                    # This is the ownership acknowledgement.  The bridge has
                    # already invalidated the old action generation and
                    # cleared its persistent target request above, so the
                    # mission layer may now publish a successor transaction.
                    self.publish_bridge_status(
                        "target_terminal_observation_released",
                        transaction_id=transaction_id,
                        target_epoch=target_epoch,
                        target_track_id=self.latest_target_track_id,
                        controller_lease="released",
                        reason=str(
                            payload.get(
                                "reason", "target_terminal_observation"
                            )
                        ),
                    )
                return
            return
        raw_goal = payload.get("goal")
        if not isinstance(raw_goal, (list, tuple)) or len(raw_goal) < 2:
            return
        try:
            x, y = float(raw_goal[0]), float(raw_goal[1])
            yaw = float(payload.get("yaw", 0.0))
            priority = max(0, min(3, int(payload.get("priority", 0))))
            route_id = max(0, int(payload.get("route_id", 0) or 0))
            target_epoch = max(0, int(payload.get("target_epoch", 0) or 0))
            transaction_id = max(0, int(payload.get("transaction_id", 0) or 0))
            map_epoch = max(0, int(payload.get("map_epoch", 0) or 0))
            origin_epoch = max(
                0, int(payload.get("origin_epoch", map_epoch) or 0)
            )
            projection_epoch = max(
                0, int(payload.get("projection_epoch", map_epoch) or 0)
            )
            graph_transaction_id = max(
                0, int(payload.get("graph_transaction_id", 0) or 0)
            )
        except (TypeError, ValueError):
            return
        frame = str(payload.get("frame_id", self.global_frame)).strip().lstrip("/")
        if not frame:
            frame = self.global_frame
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = frame
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(0.5 * yaw)
        goal.pose.orientation.w = math.cos(0.5 * yaw)
        with self.lock:
            if map_epoch <= 0:
                self.publish_bridge_status(
                    "mission_goal_ignored",
                    reason="missing_map_epoch",
                    source=str(payload.get("source", "unknown") or "unknown"),
                    route_kind=str(payload.get("route_kind", "") or ""),
                    transaction_id=transaction_id,
                )
                return
            command_source = payload.get("source", "unknown")
            normalized_source = (
                str(command_source or "unknown").strip().lower() or "unknown"
            )
            graph_route = (
                normalized_source == "global_slam_frontier"
                and route_id > 0
            )
            if graph_route and graph_transaction_id <= 0:
                self.publish_bridge_status(
                    "mission_goal_ignored",
                    reason="missing_graph_transaction_id",
                    route_id=route_id,
                    map_epoch=map_epoch,
                    transaction_id=transaction_id,
                )
                return
            raw_goal_context = payload.get("goal_context")
            normalized_goal_context = normalize_goal_context(raw_goal_context)
            context_graph_transaction_id = int(
                normalized_goal_context.get("graph_transaction_id", 0) or 0
            )
            if (
                graph_transaction_id > 0
                and context_graph_transaction_id > 0
                and context_graph_transaction_id != graph_transaction_id
            ):
                # A positive nested mismatch means two layers could adopt
                # different graph owners. Keep the canonical route strict and
                # reject the malformed mission before it reaches actionlib.
                self.publish_bridge_status(
                    "mission_goal_ignored",
                    reason="goal_context_graph_transaction_mismatch",
                    route_id=route_id,
                    transaction_id=transaction_id,
                    graph_transaction_id=graph_transaction_id,
                    goal_context_graph_transaction_id=(
                        context_graph_transaction_id
                    ),
                )
                return
            if graph_transaction_id > 0:
                normalized_goal_context["graph_transaction_id"] = (
                    graph_transaction_id
                )
            # Target and frontier transactions share the wire field for
            # backwards compatibility, but they are different ownership
            # domains. A failed target may deliberately use a newer semantic
            # transaction than the next frontier route. Permit only a route
            # that is demonstrably newer than the released/active frontier
            # route and only while the matching target-failure boundary is
            # latched; an arbitrary lower transaction must remain stale.
            target_failure_frontier_release = False
            if (
                priority < 2
                and normalized_source == "global_slam_frontier"
                and bool(getattr(self, "target_failure_latched", False))
                and int(
                    getattr(
                        self,
                        "persistent_target_terminal_boundary_transaction",
                        0,
                    )
                    or 0
                )
                == int(getattr(self, "latest_goal_transaction_id", 0) or 0)
            ):
                route_floor = max(
                    int(getattr(self, "active_route_id", 0) or 0),
                    int(getattr(self, "latest_route_id", 0) or 0),
                    int(
                        getattr(self, "frontier_lease_released_route_id", 0)
                        or 0
                    ),
                )
                target_failure_frontier_release = route_id > route_floor
            # Topic queues and latched reconnects can deliver an older command
            # after its successor. The transaction number is the ownership
            # boundary, so an older command must never reissue an obsolete goal.
            if (
                transaction_id > 0
                and self.latest_goal_transaction_id > 0
                and transaction_id < self.latest_goal_transaction_id
                and not target_failure_frontier_release
            ):
                self.publish_bridge_status(
                    "mission_goal_ignored",
                    reason="stale_transaction",
                    received_transaction_id=transaction_id,
                    latest_transaction_id=int(self.latest_goal_transaction_id),
                )
                return
            if target_failure_frontier_release:
                self.publish_bridge_status(
                    "mission_goal_stale_target_boundary_overridden",
                    reason="frontier_after_target_failure",
                    received_transaction_id=transaction_id,
                    latest_target_transaction_id=int(
                        self.latest_goal_transaction_id
                    ),
                    route_id=route_id,
                    route_floor=max(
                        int(getattr(self, "active_route_id", 0) or 0),
                        int(getattr(self, "latest_route_id", 0) or 0),
                        int(
                            getattr(
                                self, "frontier_lease_released_route_id", 0
                            )
                            or 0
                        ),
                    ),
                )
            tombstone_transaction = int(
                getattr(self, "target_lease_tombstone_transaction_id", 0) or 0
            )
            tombstone_epoch = int(
                getattr(self, "target_lease_tombstone_epoch", 0) or 0
            )
            tombstone_track = str(
                getattr(self, "target_lease_tombstone_track_id", "") or ""
            ).strip()
            incoming_track = str(payload.get("target_track_id", "") or "").strip()
            if priority >= 2 and (
                (
                    transaction_id > 0
                    and tombstone_transaction > 0
                    and transaction_id <= tombstone_transaction
                )
                or (
                    tombstone_epoch > 0
                    and target_epoch > 0
                    and target_epoch <= tombstone_epoch
                    and tombstone_track
                    and incoming_track == tombstone_track
                )
            ):
                self.publish_bridge_status(
                    "mission_goal_ignored",
                    reason="released_target_transaction",
                    received_transaction_id=transaction_id,
                    tombstone_transaction_id=tombstone_transaction,
                    target_epoch=target_epoch,
                    tombstone_epoch=tombstone_epoch,
                    target_track_id=incoming_track,
                )
                return
            if self._frontier_route_is_stale_locked(
                command_source, priority, route_id
            ):
                self.publish_bridge_status(
                    "mission_goal_ignored",
                    reason="stale_frontier_route",
                    route_id=route_id,
                    current_route_id=max(
                        int(getattr(self, "active_route_id", 0) or 0),
                        int(getattr(self, "latest_route_id", 0) or 0),
                    ),
                )
                return
            if self._frontier_route_released_locked(
                command_source, priority, route_id
            ):
                self.publish_bridge_status(
                    "mission_goal_ignored",
                    reason="released_frontier_route",
                    route_id=route_id,
                    released_route_id=int(
                        getattr(self, "frontier_lease_released_route_id", 0)
                        or 0
                    ),
                )
                return
            incoming_target_track_id = str(
                payload.get("target_track_id", "") or ""
            ).strip()
            target_failure_guard = getattr(
                self, "_target_failure_blocks_identity_locked", None
            )
            if callable(target_failure_guard) and target_failure_guard(
                priority,
                incoming_target_track_id,
                target_epoch,
                command_source,
            ):
                return
            # The persistent planner has one streaming route lease.  A target
            # or active parallax transaction owns that lease until its
            # terminal boundary; a frontier update received in the meantime
            # is stale with respect to the controller owner even when its
            # transaction number is newer.  Do not mutate ``latest_*`` here:
            # keeping the target metadata intact is what lets the terminal
            # callback close the same semantic action.
            target_owner_active = bool(
                self.persistent_execution
                and self.action_active
                and (
                    int(getattr(self, "active_intent_priority", 0) or 0) >= 2
                    or int(getattr(self, "latest_intent_priority", 0) or 0) >= 2
                    or int(
                        getattr(
                            self, "persistent_target_pending_transaction", 0
                        )
                        or 0
                    )
                    > 0
                )
            )
            target_terminal_boundary = bool(
                int(
                    getattr(
                        self,
                        "persistent_target_terminal_boundary_transaction",
                        0,
                    )
                    or 0
                )
                > 0
                and int(
                    getattr(
                        self,
                        "persistent_target_terminal_boundary_transaction",
                        0,
                    )
                    or 0
                )
                == int(getattr(self, "latest_goal_transaction_id", 0) or 0)
            )
            if (
                priority < 2
                and normalized_source == "global_slam_frontier"
                and target_owner_active
                and not target_terminal_boundary
            ):
                self.publish_bridge_status(
                    "mission_goal_ignored",
                    reason="higher_priority_target_active",
                    received_transaction_id=transaction_id,
                    active_transaction_id=int(
                        getattr(self, "active_goal_transaction_id", 0) or 0
                    ),
                    active_priority=int(
                        getattr(self, "active_intent_priority", 0) or 0
                    ),
                    active_source=str(
                        getattr(self, "active_intent_source", "unknown")
                        or "unknown"
                    ),
                    latest_priority=int(
                        getattr(self, "latest_intent_priority", 0) or 0
                    ),
                    latest_source=str(
                        getattr(self, "latest_intent_source", "unknown")
                        or "unknown"
                    ),
                    route_id=route_id,
                    deferred_until="target_terminal",
                )
                rospy.loginfo_throttle(
                    2.0,
                    "TEB goal bridge ignored frontier route=%d while target "
                    "transaction owns the persistent controller",
                    route_id,
                )
                return
            if priority < 2 and normalized_source == "global_slam_frontier":
                # Consume the one logical target terminal boundary. The
                # transport action may remain alive, but this frontier command
                # now owns the persistent route.
                self.persistent_target_terminal_boundary_transaction = 0
            self._accept_newer_frontier_route_locked(
                command_source, priority, route_id
            )
            self.intent_seen = True
            self.latest_intent_source = (
                str(payload.get("source", "unknown")).strip().lower() or "unknown"
            )
            try:
                self.latest_lifecycle_transaction_id = max(
                    0,
                    int(
                        payload.get("lifecycle_transaction_id", 0) or 0
                    ),
                )
            except (TypeError, ValueError):
                self.latest_lifecycle_transaction_id = 0
            self.latest_intent_priority = priority
            self.latest_route_kind = str(
                payload.get("route_kind", "")
            ).strip().lower() or (
                "direct_goal" if priority >= 2 else "frontier_endpoint"
            )
            self.latest_mission_route_kind = str(
                payload.get("mission_route_kind", self.latest_route_kind)
            ).strip().lower() or self.latest_route_kind
            self.latest_route_map_epoch = map_epoch or None
            self.latest_origin_epoch = origin_epoch or map_epoch or None
            self.latest_projection_epoch = (
                projection_epoch or map_epoch or None
            )
            self.latest_graph_transaction_id = graph_transaction_id
            self.latest_graph_action = str(
                payload.get("graph_action", "") or ""
            ).strip().lower()
            self.latest_graph_obligation_kind = str(
                payload.get("graph_obligation_kind", "") or ""
            ).strip().lower()
            self.latest_route_id = route_id
            self.latest_target_epoch = target_epoch
            if (
                self.frontier_portal_wait
                and self.latest_intent_source == "global_slam_frontier"
                and route_id > self.frontier_portal_wait_route_id
            ):
                self.frontier_portal_wait = False
                self.publish_bridge_status(
                    "frontier_portal_wait_released", route_id=int(route_id)
                )
            self.latest_target_track_id = str(
                payload.get("target_track_id", "")
            ).strip()
            self.latest_target_viewpoint_candidate_id = str(
                payload.get("target_viewpoint_candidate_id", "")
            ).strip()
            self.latest_target_viewpoint_attempt_id = str(
                payload.get("target_viewpoint_attempt_id", "")
            ).strip()
            self.latest_goal_context = normalized_goal_context
            predecessor_contract = payload.get("predecessor_contract")
            self.latest_predecessor_contract = (
                dict(predecessor_contract)
                if isinstance(predecessor_contract, dict)
                else None
            )
            self.latest_intent_goal = (x, y)
            self.latest_goal_transaction_id = transaction_id
            self.latest_goal = self._normalize_goal(goal)
            self.publish_bridge_status(
                "mission_goal_received",
                transaction_id=transaction_id,
                lifecycle_transaction_id=int(
                    self.latest_lifecycle_transaction_id
                ),
                priority=priority,
                source=self.latest_intent_source,
                goal=[round(x, 3), round(y, 3)],
                goal_context=self.latest_goal_context,
                route_id=int(self.latest_route_id),
                map_epoch=self.latest_route_map_epoch,
                origin_epoch=self.latest_origin_epoch,
                projection_epoch=self.latest_projection_epoch,
                graph_transaction_id=int(self.latest_graph_transaction_id),
                predecessor_route_id=int(
                    payload.get("predecessor_route_id", 0) or 0
                ),
                predecessor_contract=(
                    None
                    if self.latest_predecessor_contract is None
                    else dict(self.latest_predecessor_contract)
                ),
            )
            if priority >= 2:
                # A visual ray remains a request until Navfn installs the route
                # for exactly this transaction.
                self.persistent_target_terminal_boundary_transaction = 0
                self.persistent_installed_target_goal = None
                self.persistent_installed_target_transaction = 0
                self.persistent_target_approach_reported_transaction = 0
                self._request_persistent_target_locked("mission_goal_transaction")
            elif self.persistent_execution:
                # GlobalFrontier has already map-validated the frontier route.
                # It supersedes any older speculative target request.
                self.persistent_target_pending_transaction = 0
                self.persistent_target_pending_goal = None
                self._clear_persistent_target_request_locked(
                    "superseded_by_non_target_mission"
                )
                self._publish_persistent_mission_goal_locked(
                    "non_target_mission_goal"
                )
                if self.action_active:
                    self._adopt_persistent_mission_goal_locked()
            if self._is_active_mode():
                self.dispatch_locked(force=False, reason="mission_goal_transaction")
                retry_release = getattr(
                    self, "_retry_deferred_frontier_lease_release_locked", None
                )
                if callable(retry_release):
                    retry_release()
