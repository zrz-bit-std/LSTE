"""Validate persistent-Teb frontier endpoint reports.

The local planner can report that it reached a frontier endpoint while its
single MoveBase action remains alive.  This mixin verifies the report against
the bridge-owned route transaction before it becomes a mission terminal.
"""

import math

import rospy
from actionlib_msgs.msg import GoalStatus


class TebGoalBridgePersistentFrontierEndpointMixin:
    def on_persistent_frontier_endpoint_reached(self, message):
        """Commit one identity-checked persistent frontier/portal endpoint.

        ``PersistentTebLocalPlanner`` reports every reached non-target endpoint
        through the same topic. A certified portal deliberately uses a frozen
        odom goal, and a local egress uses the same persistent action lease, so
        both route kinds must reach this identity-checked terminal boundary.
        The global-frontier terminal validator still rechecks the directional
        gate-crossing evidence before committing Place state.
        """
        with self.lock:
            active_route_kind = str(self.active_route_kind or "").strip()
            if (
                not self.persistent_execution
                or self.task_done
                or not self.action_active
                or self.active_intent_source != "global_slam_frontier"
                or self.active_intent_priority != 0
                or active_route_kind not in (
                    "frontier_endpoint",
                    "portal_transition",
                    "local_egress",
                )
                or self.active_route_id <= 0
            ):
                return
            # A target request can arrive after the active frontier has
            # reached its local endpoint but before its target plan installs.
            # Never use that stale frontier report to overwrite the
            # higher-priority ownership decision.
            if (
                self.latest_intent_source != "global_slam_frontier"
                or self.latest_intent_priority != 0
                or str(self.latest_route_kind or "").strip()
                not in (
                    "frontier_endpoint",
                    "portal_transition",
                    "local_egress",
                )
                or self.latest_route_id != self.active_route_id
            ):
                self.publish_bridge_status(
                    "persistent_frontier_endpoint_ignored",
                    reason="newer_non_frontier_or_route_intent",
                    route_id=int(self.active_route_id),
                    latest_route_id=int(self.latest_route_id),
                    latest_source=self.latest_intent_source,
                    latest_priority=int(self.latest_intent_priority),
                )
                return
            route_id = int(self.active_route_id)
            action_contract = getattr(self, "active_action_contract", None)
            action_key = self._action_identity_key_locked(action_contract)
            if action_key is None:
                # Persistent endpoint reports must be tied to one transport
                # action. A route-only fallback could suppress a legitimate
                # later action generation that reused the semantic route id.
                self.publish_bridge_status(
                    "persistent_frontier_endpoint_ignored",
                    reason="missing_action_identity",
                    route_id=route_id,
                )
                return
            terminal_keys = getattr(
                self, "persistent_frontier_endpoint_terminal_keys", set()
            )
            if action_key in terminal_keys:
                self.publish_bridge_status(
                    "persistent_frontier_endpoint_duplicate_ignored",
                    route_id=route_id,
                    action_generation=int(action_key[1]),
                    reason="terminal_key_already_applied",
                    dedupe_key=[int(action_key[0]), int(action_key[1])],
                )
                return
            if self.last_dispatched_goal is None:
                self.publish_bridge_status(
                    "persistent_frontier_endpoint_ignored",
                    reason="missing_active_endpoint",
                    route_id=route_id,
                )
                return
            reported_goal = self._goal_in_global_frame(message)
            canonical_goal = self._goal_in_global_frame(self.last_dispatched_goal)
            active_goal = self._goal_in_global_frame(self.active_goal_global)
            if (
                reported_goal is None
                or canonical_goal is None
                or active_goal is None
            ):
                self.publish_bridge_status(
                    "persistent_frontier_endpoint_ignored",
                    reason="endpoint_transform_failed",
                    route_id=route_id,
                )
                return
            canonical_delta = math.hypot(
                float(reported_goal.pose.position.x)
                - float(canonical_goal.pose.position.x),
                float(reported_goal.pose.position.y)
                - float(canonical_goal.pose.position.y),
            )
            action_delta = math.hypot(
                float(reported_goal.pose.position.x)
                - float(active_goal.pose.position.x),
                float(reported_goal.pose.position.y)
                - float(active_goal.pose.position.y),
            )
            endpoint_epsilon = max(
                self.position_epsilon,
                self.persistent_frontier_admission_endpoint_epsilon,
            )
            # A portal goal is frozen in odom, while the planner report is the
            # current map-frame endpoint after SLAM has updated. Its old
            # canonical map coordinate may therefore drift even when the
            # active route is unchanged. The Place layer rechecks the frozen
            # gate crossing after this terminal; only the current action goal
            # must match here. Ordinary frontier endpoints retain both guards.
            # Navfn may snap a frontier endpoint to a different free cell
            # after an online-SLAM/costmap update.  The bridge already records
            # the latest plan for this exact active action; matching the
            # reported terminal to that plan is stronger route identity than
            # comparing it with the stale command coordinates.  This keeps
            # target routes strict while allowing map-derived frontier hints
            # to follow their validated plan endpoint.
            active_navfn_endpoint = getattr(
                self, "active_navfn_plan_endpoint", None
            )
            navfn_endpoint_delta = None
            navfn_endpoint_match = False
            if (
                isinstance(active_navfn_endpoint, (list, tuple))
                and len(active_navfn_endpoint) >= 2
            ):
                try:
                    navfn_endpoint_delta = math.hypot(
                        float(reported_goal.pose.position.x)
                        - float(active_navfn_endpoint[0]),
                        float(reported_goal.pose.position.y)
                        - float(active_navfn_endpoint[1]),
                    )
                    navfn_endpoint_match = (
                        navfn_endpoint_delta <= max(self.position_epsilon, 0.05)
                    )
                except (TypeError, ValueError):
                    navfn_endpoint_delta = None
            endpoint_match_basis = "action_goal"
            endpoint_mismatch = action_delta > endpoint_epsilon
            if (
                active_route_kind != "portal_transition"
                and endpoint_mismatch
                and navfn_endpoint_match
            ):
                endpoint_mismatch = False
                endpoint_match_basis = "active_navfn_plan_endpoint"
            if active_route_kind != "portal_transition":
                canonical_mismatch = canonical_delta > endpoint_epsilon
                if canonical_mismatch and navfn_endpoint_match:
                    canonical_mismatch = False
                    endpoint_match_basis = "active_navfn_plan_endpoint"
                endpoint_mismatch = endpoint_mismatch or canonical_mismatch
            if endpoint_mismatch:
                self.publish_bridge_status(
                    "persistent_frontier_endpoint_ignored",
                    reason="endpoint_mismatch",
                    route_id=route_id,
                    canonical_delta=round(canonical_delta, 4),
                    action_delta=round(action_delta, 4),
                    navfn_endpoint_delta=(
                        None
                        if navfn_endpoint_delta is None
                        else round(navfn_endpoint_delta, 4)
                    ),
                    endpoint_epsilon=round(endpoint_epsilon, 4),
                    reported_goal=[
                        round(float(reported_goal.pose.position.x), 3),
                        round(float(reported_goal.pose.position.y), 3),
                    ],
                    canonical_goal=[
                        round(float(canonical_goal.pose.position.x), 3),
                        round(float(canonical_goal.pose.position.y), 3),
                    ],
                    action_goal=[
                        round(float(active_goal.pose.position.x), 3),
                        round(float(active_goal.pose.position.y), 3),
                    ],
                )
                return
            terminal_source_goal = (
                getattr(self, "active_portal_source_goal", None)
                if active_route_kind == "portal_transition"
                and getattr(self, "active_portal_source_goal", None) is not None
                else self.last_dispatched_goal
            )
            try:
                contract = dict(action_contract or {})
            except (TypeError, ValueError):
                contract = {}
            self.publish_bridge_status(
                "endpoint_detected",
                route_id=route_id,
                transaction_id=int(contract.get("transaction_id", 0) or 0),
                lifecycle_transaction_id=int(
                    contract.get("lifecycle_transaction_id", 0) or 0
                ),
                action_generation=int(
                    contract.get(
                        "generation", contract.get("action_generation", 0)
                    )
                    or 0
                ),
                graph_transaction_id=int(
                    contract.get("graph_transaction_id", 0) or 0
                ),
                map_epoch=contract.get("map_epoch"),
                origin_epoch=contract.get("origin_epoch"),
                projection_epoch=contract.get("projection_epoch"),
                route_kind=active_route_kind,
                mission_route_kind=str(
                    contract.get("mission_route_kind", "") or ""
                ),
                endpoint=[
                    round(float(reported_goal.pose.position.x), 3),
                    round(float(reported_goal.pose.position.y), 3),
                ],
                endpoint_match_basis=endpoint_match_basis,
                canonical_delta=round(canonical_delta, 4),
                action_delta=round(action_delta, 4),
                navfn_endpoint_delta=(
                    None
                    if navfn_endpoint_delta is None
                    else round(navfn_endpoint_delta, 4)
                ),
            )
            if active_route_kind == "frontier_endpoint":
                # In persistent-stream mode the C++ planner has already put
                # its current path into terminal hold.  Close the semantic
                # endpoint transaction, but keep the one MoveBase action
                # alive so the next mission command can replace the stream
                # without an actionlib cancel/re-dispatch pulse.  Portal and
                # egress routes remain on the explicit transport-terminal
                # path because their physical ownership must be rebound.
                try:
                    contract = dict(action_contract or {})
                except (TypeError, ValueError):
                    contract = {}
                pending = {
                    "route_id": route_id,
                    "action_generation": int(
                        contract.get("generation", 0) or 0
                    ),
                    "transaction_id": int(
                        contract.get("transaction_id", 0) or 0
                    ),
                    "lifecycle_transaction_id": int(
                        contract.get("lifecycle_transaction_id", 0) or 0
                    ),
                    "graph_transaction_id": int(
                        contract.get("graph_transaction_id", 0) or 0
                    ),
                    "map_epoch": contract.get("map_epoch"),
                    "origin_epoch": contract.get("origin_epoch"),
                    "projection_epoch": contract.get("projection_epoch"),
                    "ack_sent": False,
                }
                self.persistent_frontier_endpoint_pending = pending
                terminal_goal = self._publish_execution_terminal_locked(
                    terminal_source_goal,
                    action_contract=action_contract,
                    invalidate_feedback=False,
                    status=GoalStatus.SUCCEEDED,
                    reason="persistent_frontier_endpoint_terminal",
                )
                if terminal_goal is None:
                    self.persistent_frontier_endpoint_pending = None
                    return
                self.persistent_frontier_endpoint_terminal_keys.add(action_key)
                self._clear_target_failure_locked("persistent_frontier_endpoint")
                self.terminal_count += 1
                endpoint_fields = self._action_contract_status_fields(
                    action_contract
                )
                endpoint_fields.update(
                    {
                        "successor_route_id": int(
                            self.prefetched_frontier_route_id
                        ),
                        "canonical_delta": round(canonical_delta, 4),
                        "action_delta": round(action_delta, 4),
                        "navfn_endpoint_delta": (
                            None
                            if navfn_endpoint_delta is None
                            else round(navfn_endpoint_delta, 4)
                        ),
                        "endpoint_match_basis": endpoint_match_basis,
                        "endpoint_epsilon": round(endpoint_epsilon, 4),
                        "lifecycle": (
                            "semantic_terminal_transport_lease_held"
                        ),
                        "controller_lease": "held",
                        "semantic_terminal": True,
                        "transport_action_active": True,
                    }
                )
                self.publish_bridge_status(
                    "persistent_frontier_endpoint_terminal",
                    **endpoint_fields,
                )
                rospy.loginfo(
                    "TEB goal bridge accepted persistent frontier endpoint "
                    "without transport cancellation: route_id=%d",
                    route_id,
                )
                return
            termination = self._commit_termination(
                "persistent_frontier_endpoint_terminal",
                source_goal=terminal_source_goal,
                action_contract=action_contract,
                watchdog_reason="persistent_frontier_endpoint_terminal",
            )
            if not termination.get("committed", False):
                return
            self.persistent_frontier_endpoint_terminal_keys.add(action_key)
            self._clear_target_failure_locked("persistent_frontier_endpoint")
            self.terminal_count += 1
            terminal_event = (
                "persistent_portal_transition_terminal"
                if active_route_kind == "portal_transition"
                else "persistent_local_egress_terminal"
                if active_route_kind == "local_egress"
                else "persistent_frontier_endpoint_terminal"
            )
            self.publish_bridge_status(
                terminal_event,
                route_id=route_id,
                successor_route_id=int(self.prefetched_frontier_route_id),
                canonical_delta=round(canonical_delta, 4),
                action_delta=round(action_delta, 4),
                navfn_endpoint_delta=(
                    None
                    if navfn_endpoint_delta is None
                    else round(navfn_endpoint_delta, 4)
                ),
                endpoint_match_basis=endpoint_match_basis,
                endpoint_epsilon=round(endpoint_epsilon, 4),
                lifecycle=(
                    "portal_edge_reached_then_place_graph_terminal"
                    if active_route_kind == "portal_transition"
                    else "local_egress_anchor_reached_then_place_rebind"
                    if active_route_kind == "local_egress"
                    else "local_teb_endpoint_reached_then_prefetched_successor"
                    if self.prefetched_frontier_goal is not None
                    else "local_teb_endpoint_reached_then_fresh_frontier"
                ),
            )
            rospy.loginfo(
                "TEB goal bridge accepted persistent frontier endpoint: "
                "route_id=%d successor_route_id=%d canonical_delta=%.3fm "
                "action_delta=%.3fm",
                route_id,
                int(self.prefetched_frontier_route_id),
                canonical_delta,
                action_delta,
            )
