"""Creation of internally consistent map, route, and pose planning snapshots."""

import math

import numpy as np
import rospy

from clock_provider import now_for
from global_frontier_models import (
    FrontierMapContext,
    FrontierPlanningSnapshot,
    FrontierRouteGraph,
)
from global_frontier_completion_gate import evaluate_completion
from global_frontier_grid import conservative_clearance_cells


class GlobalFrontierPlanningSnapshotMixin:

    def _build_frontier_map_context(self, message):
        """Interpret the latest occupancy grid once for a planning cycle."""
        data = np.asarray(message.data, dtype=np.int8).reshape(
            message.info.height, message.info.width,
        )
        unknown = data == -1
        known_free = data == 0
        occupied = data >= 50
        components = self.build_topology_components(
            known_free, occupied, message,
        )
        refreshed_region_ids = []
        if getattr(self, "place_memory_enabled", True):
            refreshed_region_ids = self.refresh_region_components(
                message, components, known_free,
            )
        if refreshed_region_ids:
            rospy.loginfo_throttle(
                5.0,
                "Global frontier rehydrated region topology ids=%s epoch=%d",
                refreshed_region_ids,
                components.epoch,
            )
        return FrontierMapContext(
            unknown=unknown,
            known_free=known_free,
            occupied=occupied,
            components=components,
        )

    def _build_frontier_route_graph(self, message, map_context, robot_map, now):
        """Build the planning masks and robot-rooted BFS fields for this map."""
        # The endpoint is a cell centre, but TEB's clearance is measured from
        # the robot footprint to the physical obstacle.  Do not subtract a
        # cell after rounding: that admits a frontier one cell inside the
        # controller's feasible set (the T-junction east-wall failure).
        clearance_cells = conservative_clearance_cells(
            self.clearance, message.info.resolution,
        )
        strict_free = map_context.known_free & ~self.inflate(
            map_context.occupied, clearance_cells,
        )
        frontier_cells = conservative_clearance_cells(
            self.frontier_clearance, message.info.resolution,
        )
        frontier_free = map_context.known_free & ~self.inflate(
            map_context.occupied, frontier_cells,
        )
        robot_col = int(
            (robot_map[0] - message.info.origin.position.x) / message.info.resolution
        )
        robot_row = int(
            (robot_map[1] - message.info.origin.position.y) / message.info.resolution
        )
        strict_seed = self.nearest_seed(
            strict_free,
            robot_row,
            robot_col,
            max(1, int(0.8 / message.info.resolution)),
        )
        strict_steps = (
            None
            if strict_seed is None
            else self.bfs(
                strict_free,
                strict_seed,
                getattr(self, "planning_should_preempt", None),
            )
        )
        seed = self.nearest_seed(
            frontier_free,
            robot_row,
            robot_col,
            max(1, int(0.8 / message.info.resolution)),
        )
        if seed is None:
            rospy.logwarn_throttle(
                3.0,
                "Global frontier has no known-free observation seed at odom=(%.2f,%.2f)",
                self.pose_odom.x,
                self.pose_odom.y,
            )
            return None
        route_steps = self.bfs(
            frontier_free,
            seed,
            getattr(self, "planning_should_preempt", None),
        )
        if route_steps is None:
            return None
        return FrontierRouteGraph(
            strict_free=strict_free,
            strict_steps=strict_steps,
            frontier_free=frontier_free,
            frontier=self.is_frontier(frontier_free, map_context.unknown),
            validation=self.cached_costmap_steps(robot_map, now),
            route_steps=route_steps,
            seed=seed,
        )

    def _choose_frontier_for_new_action(
        self, message, route_graph, map_context, now, robot_map, robot_yaw_map,
        allowed_region_tiers=None, include_semantic=True,
    ):
        """Select one Navfn-valid action, optionally limited to a region tier."""
        return self.choose_valid_frontier(
            message,
            route_graph.route_steps,
            route_graph.frontier,
            map_context.unknown,
            map_context.occupied,
            now,
            robot_map,
            validation=route_graph.validation,
            heading_reference=robot_yaw_map,
            max_heading_delta=(
                self.heading_hard_limit
                if getattr(self, "heading_policy_enabled", True)
                else None
            ),
            route_seed=route_graph.seed,
            preferred_steps=route_graph.strict_steps,
            preferred_mask=route_graph.strict_free,
            allow_observation_recovery=self.navfn_observation_recovery_enabled,
            semantic_hint=(
                self.pending_semantic_hint_map
                if (
                    include_semantic
                    and getattr(self, "semantic_hint_enabled", True)
                    and self.pending_replan_request_id > 0
                ) else None
            ),
            semantic_pursuit=(
                self.pending_semantic_pursuit
                if (
                    include_semantic
                    and getattr(self, "semantic_hint_enabled", True)
                    and self.pending_replan_request_id > 0
                ) else False
            ),
            components=map_context.components,
            allowed_region_tiers=allowed_region_tiers,
            allow_portal_transitions=allowed_region_tiers is None,
        )

    def _release_exhausted_target_region_claim(self):
        """Record the only normal transition out of a target-room claim."""
        target_work = getattr(self, "target_observation_work", None)
        current_place_id = getattr(self, "current_physical_place_id", None)
        if (
            target_work is not None
            and current_place_id is not None
            and target_work.has_pending(current_place_id)
        ):
            # A missing local endpoint is not negative target evidence. Keep
            # the room lease and let the completion gate request another
            # evidence/recovery cycle instead of opening an unrelated Portal.
            self.publish_status(
                "target_region_claim_waiting_for_obligation",
                place_id=int(current_place_id),
                pending_target_observation=True,
            )
            return
        claimed_component = self.target_region_claim_component
        self.publish_status(
            "target_region_claim_exhausted",
            request_id=int(self.target_region_claim_request_id),
            anchor=(
                None if self.target_region_claim_anchor_map is None
                else [
                    round(self.target_region_claim_anchor_map[0], 3),
                    round(self.target_region_claim_anchor_map[1], 3),
                ]
            ),
            component=(
                None if claimed_component is None else {
                    "epoch": int(claimed_component["epoch"]),
                    "label": int(claimed_component["label"]),
                }
            ),
            cross_region_candidates=int(self.target_region_claim_cross_region_skips),
        )
        self.clear_target_region_claim("no_executable_frontier_in_claimed_room")

    def _report_no_frontier_selection(self):
        """Record why this cycle cannot create a new execution action."""
        completion_gate = evaluate_completion(
            graph_ready=(
                not getattr(self, "place_memory_enabled", True)
                or getattr(self, "current_physical_place_id", None) is not None
            ),
            region_memory=getattr(self, "region_memory", None),
            work_item_ledger=getattr(self, "place_work_items", None),
            portal_probe_ledger=getattr(self, "portal_probe_ledger", None),
            portal_hypothesis_ledger=getattr(
                self, "portal_hypothesis_ledger", None,
            ),
            target_observation_work=getattr(
                self, "target_observation_work", None,
            ),
        )
        validation_pending = bool(
            self.frontier_validation_budget_exhausted
            or self.frontier_validation_pending
        )
        transaction = getattr(self, "portal_transaction", None)
        active_transaction = bool(
            transaction is not None and transaction.active
        )
        # A remote frontier or a durable unresolved ledger entry is a graph
        # obligation even when the ordinary candidate scorer returned no
        # endpoint. Keep these facts outside CompletionGate: the gate counts
        # durable obligations, while this flag represents a live snapshot
        # boundary that still needs a legal Portal conversion.
        structural_candidate_pending = bool(
            getattr(self, "last_place_graph_hop_skips", 0) > 0
        )
        local_reobserve_available = bool(
            getattr(self, "last_work_item_failed_viewpoint_skips", 0) > 0
            or getattr(self, "last_portal_probe_viewpoint_rejections", 0) > 0
            or getattr(self, "last_graph_policy_rejections", 0) > 0
        )
        graph_transit_available = bool(
            # ``last_portal_sources`` is only a structural candidate count.
            # Recovery may request transit only after portal selection has
            # proved at least one route executable through all admissions.
            getattr(self, "last_portal_executable_candidates", 0) > 0
            or getattr(self, "pending_local_egress", None) is not None
            or getattr(self, "pending_portal_retry", None) is not None
        )
        graph_plan = getattr(self, "last_graph_route_plan", None)
        if graph_plan is not None and getattr(graph_plan, "status", None) == "ready":
            graph_transit_available = graph_transit_available or getattr(
                graph_plan, "action", None
            ) in ("cross_portal", "probe_portal")
        evidence_signature = (
            getattr(self, "current_physical_place_id", None),
            getattr(self, "last_place_graph_hop_skips", 0),
            getattr(self, "last_portal_sources", 0),
            getattr(self, "last_portal_candidates_seen", 0),
            getattr(self, "last_portal_hard_rejected", 0),
            getattr(self, "last_portal_executable_candidates", 0),
            getattr(self, "last_portal_missing_transition_goals", 0),
            getattr(self, "last_portal_unbound_reprojection_skips", 0),
            getattr(self, "last_work_item_failed_viewpoint_skips", 0),
            getattr(self, "last_portal_probe_viewpoint_rejections", 0),
            getattr(self, "last_graph_policy_rejections", 0),
            (
                None
                if graph_plan is None
                else graph_plan.signature()
            ),
        )
        completion_state = getattr(self, "graph_completion_state", None)
        if completion_state is None:
            # Narrow injected fixtures may not construct runtime state. The
            # pure gate behavior remains available for those callers.
            completion_decision = None
        else:
            completion_decision = completion_state.decide(
                completion_gate,
                graph_ready=(
                    not getattr(self, "place_memory_enabled", True)
                    or getattr(self, "current_physical_place_id", None)
                    is not None
                ),
                active_transaction=active_transaction,
                validation_pending=validation_pending,
                has_executable_viewpoint=False,
                local_reobserve_available=local_reobserve_available,
                graph_transit_available=graph_transit_available,
                structural_candidate_pending=structural_candidate_pending,
                evidence_signature=evidence_signature,
            )

        if validation_pending:
            self.publish_status(
                "frontier_validation_pending",
                reason=(
                    "navfn_validation_budget_exhausted"
                    if self.frontier_validation_budget_exhausted
                    else "navfn_validation_unavailable"
                ),
                validation_state=self.navfn_last_validation_state,
            )
            rospy.logwarn_throttle(
                3.0,
                "Global frontier deferred exhaustion: Navfn validation "
                "is pending state=%s budget_exhausted=%s",
                self.navfn_last_validation_state,
                self.frontier_validation_budget_exhausted,
            )
            return
        if active_transaction:
            self.publish_status(
                "portal_transaction_selection_blocked",
                transaction_id=int(transaction.snapshot().transaction_id),
                route_id=int(transaction.snapshot().route_id),
                state=transaction.state,
                reason="portal_transaction_owned",
            )
            return

        if completion_decision is not None:
            if completion_decision.action == "complete_exploration":
                self.last_completion_gate_signature = None
                if not self.frontier_exhausted:
                    self.frontier_exhausted = True
                    self.publish_status(
                        "frontier_exhausted",
                        reason="no_safe_reachable_frontier",
                        route_clearance=round(float(self.clearance), 3),
                        frontier_clearance=round(float(self.frontier_clearance), 3),
                        observed_place_reentry_skips=int(
                            self.last_observed_place_reentry_skips
                        ),
                    )
                    rospy.logwarn(
                        "Global frontier exhausted: no safe reachable boundary; "
                        "releasing the current execution lease"
                    )
                return

            if completion_decision.action == "blocked_unresolved_obligations":
                # This is an explicit graph failure, not exploration success.
                # Do not set ``frontier_exhausted``: a future map/task event can
                # change the evidence signature and reopen recovery.
                if completion_decision.transition:
                    self.publish_status(
                        "graph_exploration_blocked",
                        reason=completion_decision.reason,
                        recovery_state=completion_decision.state,
                        obligation_counts=completion_decision.obligation_counts,
                        structural_candidate_pending=(
                            structural_candidate_pending
                        ),
                        local_reobserve_available=local_reobserve_available,
                        graph_transit_available=graph_transit_available,
                        graph_route_plan=(
                            None
                            if graph_plan is None
                            else (
                                graph_plan.as_dict()
                                if callable(getattr(graph_plan, "as_dict", None))
                                else graph_plan
                            )
                        ),
                        portal_candidates_seen=int(
                            getattr(self, "last_portal_candidates_seen", 0)
                        ),
                        portal_hard_rejected=int(
                            getattr(self, "last_portal_hard_rejected", 0)
                        ),
                        portal_executable_candidates=int(
                            getattr(self, "last_portal_executable_candidates", 0)
                        ),
                    )
                    rospy.logwarn(
                        "Global frontier graph blocked: unresolved obligations "
                        "have no executable viewpoint counts=%s",
                        completion_decision.obligation_counts,
                    )
                return

            if completion_decision.action in (
                "reobserve_current_place",
                "transit_to_pending_place",
            ):
                # Force the next planning cycle to rebuild the graph snapshot.
                # This is a logical recovery request; it never bypasses Navfn,
                # Portal certification, or the current collision checks.
                self.last_planning_wall = 0.0
                self.frontier_exhausted = False
                self.place_graph_waiting_for_portal = False
                if completion_decision.transition:
                    self.publish_status(
                        "graph_recovery_requested",
                        action=completion_decision.action,
                        reason=completion_decision.reason,
                        recovery_state=completion_decision.state,
                        obligation_counts=completion_decision.obligation_counts,
                        structural_candidate_pending=(
                            structural_candidate_pending
                        ),
                        local_reobserve_available=local_reobserve_available,
                        graph_transit_available=graph_transit_available,
                        graph_route_plan=(
                            None
                            if graph_plan is None
                            else (
                                graph_plan.as_dict()
                                if callable(getattr(graph_plan, "as_dict", None))
                                else graph_plan
                            )
                        ),
                        portal_candidates_seen=int(
                            getattr(self, "last_portal_candidates_seen", 0)
                        ),
                        portal_hard_rejected=int(
                            getattr(self, "last_portal_hard_rejected", 0)
                        ),
                        portal_executable_candidates=int(
                            getattr(self, "last_portal_executable_candidates", 0)
                        ),
                    )
                    rospy.logwarn(
                        "Global frontier requested graph recovery action=%s "
                        "reason=%s counts=%s",
                        completion_decision.action,
                        completion_decision.reason,
                        completion_decision.obligation_counts,
                    )
                return

            if completion_decision.action == "wait_for_evidence":
                if completion_decision.reason == "graph_not_ready":
                    return

        if self.last_place_graph_hop_skips > 0:
            # A remote frontier must wait for a future adjacent-place action;
            # bypassing that transition would reintroduce room reentry.
            if not self.place_graph_waiting_for_portal:
                self.place_graph_waiting_for_portal = True
                self.publish_status(
                    "place_graph_waiting_for_portal_frontier",
                    hop_limit=int(self.place_graph_hop_limit),
                    skipped_candidates=int(self.last_place_graph_hop_skips),
                    observed_place_reentry_skips=int(
                        self.last_observed_place_reentry_skips
                    ),
                    unresolved_candidates=int(
                        self.last_place_graph_unresolved_candidates
                    ),
                    uncertified_place_transition_skips=int(
                        self.last_uncertified_place_transition_skips
                    ),
                    portal_sources=int(self.last_portal_sources),
                    portal_candidates_seen=int(
                        getattr(self, "last_portal_candidates_seen", 0)
                    ),
                    portal_hard_rejected=int(
                        getattr(self, "last_portal_hard_rejected", 0)
                    ),
                    portal_executable_candidates=int(
                        getattr(self, "last_portal_executable_candidates", 0)
                    ),
                    portal_missing_transition_goals=int(
                        self.last_portal_missing_transition_goals
                    ),
                    portal_excluded=int(self.last_portal_excluded),
                    portal_rejected=int(self.last_portal_rejected),
                    portal_costmap_rejected=int(
                        self.last_portal_costmap_rejected
                    ),
                    portal_heading_rejected=int(
                        self.last_portal_heading_rejected
                    ),
                    portal_costmap_fallbacks=int(
                        self.last_portal_costmap_fallbacks
                    ),
                    portal_endpoint=self.last_portal_endpoint,
                )
            rospy.logwarn_throttle(
                3.0,
                "Global frontier holds exploration: all remaining "
                "candidates cross more than %d structural portal(s) "
                "(skipped=%d)",
                self.place_graph_hop_limit,
                self.last_place_graph_hop_skips,
            )
            return
        # Fixtures from before the recovery protocol may not expose its state
        # machine. Preserve the original strict gate semantics for them.
        if completion_decision is not None:
            return
        if not completion_gate.complete:
            signature = (
                completion_gate.reasons,
                tuple(sorted(completion_gate.counts.items())),
            )
            if signature != getattr(self, "last_completion_gate_signature", None):
                self.publish_status(
                    "frontier_waiting_for_durable_obligations",
                    reasons=list(completion_gate.reasons),
                    obligation_counts=completion_gate.counts,
                )
                self.last_completion_gate_signature = signature
            rospy.logwarn_throttle(
                3.0,
                "Global frontier waiting for durable obligations reasons=%s counts=%s",
                completion_gate.reasons,
                completion_gate.counts,
            )
            return
        self.last_completion_gate_signature = None
        if not self.frontier_exhausted:
            self.frontier_exhausted = True
            self.publish_status(
                "frontier_exhausted",
                reason="no_safe_reachable_frontier",
                route_clearance=round(float(self.clearance), 3),
                frontier_clearance=round(float(self.frontier_clearance), 3),
                observed_place_reentry_skips=int(
                    self.last_observed_place_reentry_skips
                ),
            )
            rospy.logwarn(
                "Global frontier exhausted: no safe reachable boundary; "
                "releasing the current execution lease"
            )
        rospy.logwarn_throttle(
            3.0,
            "Global frontier found no unknown boundary with a safe "
            "approach (route_clearance=%.2fm frontier_clearance=%.2fm)",
            self.clearance,
            self.frontier_clearance,
        )

    def planning_pose_snapshot(self):
        """Resolve the current map-frame robot pose for one planning cycle."""
        if not self._rehydration_inputs_ready_for_planning():
            return None
        if self.task_done or self.map_msg is None or self.pose_odom is None:
            return None
        message = self.map_msg
        map_frame = message.header.frame_id or "map"
        robot_map = self.transform_xy(
            map_frame,
            "odom",
            self.pose_odom.x,
            self.pose_odom.y,
        )
        if robot_map is None or message.info.resolution <= 0.0:
            return None
        now = now_for(self)
        if not self.navigation_stack_is_ready(robot_map, map_frame, now):
            return None
        robot_yaw_map = self.transform_yaw(
            map_frame,
            "odom",
            self.pose_odom.theta,
        )
        return message, robot_map, robot_yaw_map, now

    def active_route_planning_is_deferred(self, robot_map, now):
        """Avoid rebuilding global grids while a stable route is progressing."""
        if self.active_frontier is None:
            return False
        active_distance = math.hypot(
            self.active_frontier[2] - robot_map[0],
            self.active_frontier[3] - robot_map[1],
        )
        command_distance = (
            float("inf")
            if self.active_last_waypoint_map is None
            else math.hypot(
                self.active_last_waypoint_map[0] - robot_map[0],
                self.active_last_waypoint_map[1] - robot_map[1],
            )
        )
        needs_planning = (
            self.recovery_pending_route_id == self.active_route_id
            or self.prefetched_frontier is not None
            or active_distance <= self.prefetch_distance
            or command_distance <= self.waypoint_release_radius
            or (
                not getattr(self, "controller_owned_route_failure", False)
                and
                self.active_progress_time > 0.0
                and now - self.active_progress_time >= self.stall_timeout
            )
        )
        return (
            not needs_planning
            and now - self.last_planning_wall < self.planning_period
        )

    def build_frontier_planning_snapshot(
        self, message, robot_map, robot_yaw_map, now,
    ):
        """Build one internally consistent map, route graph, and pose snapshot."""
        map_context = self._build_frontier_map_context(message)
        # The terminal callback never invents a room identity when the final
        # scan has not yet produced a structural core. Resolve queued arrivals
        # from this complete map before selection consults place memory.
        if getattr(self, "place_memory_enabled", True):
            self.retry_pending_portal_arrivals(
                message,
                map_context.components,
                map_context.known_free,
                now,
            )
            bootstrap = getattr(self, "ensure_current_physical_place", None)
            if bootstrap is not None:
                bootstrap(
                    message,
                    map_context.components,
                    map_context.known_free,
                    robot_map,
                    now,
                )
        # A recovery action is explicitly inside its source place. Wait for a
        # structural component at the recovered pose before any selector pass;
        # otherwise a temporary SLAM split can mint a duplicate room node.
        if getattr(self, "place_memory_enabled", True) and not self.rebind_completed_local_egress_place(
            message,
            map_context.components,
            map_context.known_free,
            robot_map,
            now,
        ):
            return None
        # A visual target is a higher-level mission fact than an individual
        # frontier route. Do not select unrestricted exploration until its
        # current map-derived room identity is available.
        if getattr(self, "place_memory_enabled", True) and not self.refresh_target_region_claim(
            message,
            map_context.components,
            map_context.known_free,
            robot_map,
        ):
            return None
        route_graph = self._build_frontier_route_graph(
            message, map_context, robot_map, now,
        )
        if route_graph is None:
            return None
        return FrontierPlanningSnapshot(
            message=message,
            map_context=map_context,
            route_graph=route_graph,
            robot_map=robot_map,
            robot_yaw_map=robot_yaw_map,
            now=now,
        )
