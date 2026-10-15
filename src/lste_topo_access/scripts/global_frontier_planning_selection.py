"""Terminal successor selection and no-frontier reporting policy."""

import rospy

from global_frontier_graph_route_planner import (
    PLAN_READY,
)
from global_frontier_graph_route_gate import (
    candidate_refines_graph_plan,
    graph_plan_is_exclusive,
)


class GlobalFrontierPlanningSelectionMixin:

    def _current_graph_snapshot_prepare_kwargs(self, snapshot_epoch):
        """Return the completed current-snapshot identity projection."""
        ready = getattr(
            self, "_graph_snapshot_projection_is_current", None
        )
        if not callable(ready) or not ready(snapshot_epoch):
            return None
        get_ids = getattr(self, "_graph_snapshot_projection_ids", None)
        if not callable(get_ids):
            return None
        visible_work_item_ids, visible_probe_ids, visible_portal_ids = get_ids()
        return {
            "visible_work_item_ids": visible_work_item_ids,
            "visible_probe_ids": visible_probe_ids,
            "visible_portal_ids": visible_portal_ids,
            "map_epoch": snapshot_epoch,
        }

    def _materialize_graph_successor_first(self, snapshot, snapshot_epoch):
        """Materialize a leased Portal action before scanning raw frontiers."""
        if not getattr(self, "graph_route_planner_enabled", False):
            return None
        if not getattr(self, "graph_route_plan_lease_active", False):
            return None
        plan = getattr(self, "last_graph_route_plan", None)
        if plan is None or getattr(plan, "status", None) != PLAN_READY:
            return None
        if getattr(plan, "action", None) not in (
            "cross_portal", "probe_portal",
        ):
            return None
        prepare = getattr(self, "_prepare_graph_route_action", None)
        if prepare is None:
            return None
        projection_kwargs = self._current_graph_snapshot_prepare_kwargs(
            snapshot_epoch
        )
        if projection_kwargs is None:
            return None
        _plan, candidate = prepare(
            snapshot,
            reuse_existing_plan=True,
            **projection_kwargs,
        )
        if candidate is None:
            return None
        return candidate, "graph_route_edge", False

    def terminal_prefetch_novelty_decision(self, snapshot):
        """Prefer a newly exposed place over a cached revisit at a terminal.

        The return tuple is ``(candidate, mode, wait_for_validation)``.  A
        validation wait is distinct from no candidate: it must preserve the
        current decision rather than reporting exploration as exhausted.
        """
        if self.prefetched_frontier is None:
            return None, None, False
        pending_x, pending_y = self.prefetched_goal_map
        pending_information = (
            0.0
            if self.prefetched_frontier_information is None
            else self.prefetched_frontier_information
        )
        pending_tier, pending_region = self.region_memory.candidate_tier(
            pending_x,
            pending_y,
            pending_information,
            component=self.prefetched_frontier_component,
        )
        if pending_tier == "dormant":
            rospy.loginfo(
                "Global frontier discarded dormant prefetched region "
                "id=%s map=(%.2f,%.2f)",
                None if pending_region is None else pending_region["id"],
                pending_x,
                pending_y,
            )
            self.publish_status(
                "frontier_prefetch_discarded",
                reason="prefetched_region_dormant",
                pending_goal=[round(float(pending_x), 3), round(float(pending_y), 3)],
                pending_region_id=(
                    None if pending_region is None else int(pending_region["id"])
                ),
            )
            self.clear_prefetched_frontier()
            return None, None, False
        if pending_tier != "revisit":
            return None, None, False
        novel = self._choose_frontier_for_new_action(
            snapshot.message,
            snapshot.route_graph,
            snapshot.map_context,
            snapshot.now,
            snapshot.robot_map,
            snapshot.robot_yaw_map,
            allowed_region_tiers=("new",),
            include_semantic=False,
        )
        if novel is None:
            return None, None, self.frontier_validation_pending
        novel_component = novel[8] if len(novel) > 8 else None
        rospy.loginfo(
            "Global frontier replaced revisited prefetched branch "
            "map=(%.2f,%.2f) region=%s with newly exposed region "
            "map=(%.2f,%.2f)",
            pending_x,
            pending_y,
            None if pending_region is None else pending_region["id"],
            novel[2],
            novel[3],
        )
        self.publish_status(
            "frontier_prefetch_superseded",
            reason="new_region_available_at_terminal",
            prefetched_goal=[round(float(pending_x), 3), round(float(pending_y), 3)],
            prefetched_region_id=(
                None if pending_region is None else int(pending_region["id"])
            ),
            selected_goal=[round(float(novel[2]), 3), round(float(novel[3]), 3)],
            selected_component_cells=(
                None if novel_component is None else int(novel_component["cells"])
            ),
        )
        self.clear_prefetched_frontier()
        return novel, "terminal_new_region", False

    def select_pending_local_egress(self, snapshot):
        """Convert one queued safe-history anchor into a normal route command."""
        pending = self.pending_local_egress
        if pending is None:
            return None, None, False
        x, y = pending["anchor_map"]
        route_graph = snapshot.route_graph
        reassociated = self.nearest_reachable_cell(
            snapshot.message,
            route_graph.route_steps,
            x,
            y,
        )
        costmap_distance = self.candidate_costmap_distance(
            route_graph.validation, x, y,
        )
        reachable = self.navfn_goal_reachable(
            snapshot.robot_map,
            (x, y),
            snapshot.message.header.frame_id or "map",
        )
        if reachable is None:
            return None, None, True
        if reassociated is None or reachable is False or (
            route_graph.validation is not None and costmap_distance is None
        ):
            self.publish_status(
                "local_egress_discarded",
                source_route_id=int(pending["source_route_id"]),
                recovery_goal=[round(float(x), 3), round(float(y), 3)],
                reason="egress_anchor_not_reachable",
            )
            if pending.get("resume_portal_transition", False):
                self.discard_pending_portal_retry("egress_anchor_not_reachable")
            self.pending_local_egress = None
            return None, None, False
        row, col = reassociated
        path_distance = (
            float(costmap_distance)
            if costmap_distance is not None
            else float(route_graph.route_steps[row, col])
            * float(snapshot.message.info.resolution)
        )
        self.active_local_egress_resumes_portal = bool(
            pending.get("resume_portal_transition", False)
        )
        self.pending_local_egress = None
        self.publish_status(
            "local_egress_selected",
            source_route_id=int(pending["source_route_id"]),
            recovery_goal=[round(float(x), 3), round(float(y), 3)],
            reason=pending["reason"],
            path_distance=round(path_distance, 3),
            resume_portal_transition=self.active_local_egress_resumes_portal,
        )
        return (
            row,
            col,
            float(x),
            float(y),
            path_distance,
            0.0,
            0.0,
            0.0,
            None,
            0,
            "local_egress",
        ), "local_egress_recovery", False

    def _select_durable_portal_probe_fallback(self, snapshot):
        """Materialize a current-place probe without repeating full selection.

        A blocked graph plan can mean that its preferred crossing has no
        endpoint in this map projection.  A pending source/destination probe
        is the same durable evidence chain and is therefore a valid immediate
        successor.  The old path reached this code only after rerunning the
        full frontier chooser, which left a multi-second no-goal gap.
        """
        durable_probe = getattr(self, "select_durable_portal_probe", None)
        if not callable(durable_probe):
            return None, None, False
        candidate = durable_probe(snapshot)
        if candidate is None:
            return None, None, False
        probe = candidate[15] if len(candidate) > 15 else None
        try:
            probe_id = int(getattr(probe, "probe_id", 0) or 0)
        except (TypeError, ValueError):
            probe_id = 0
        graph_prepare = getattr(self, "_set_graph_route_preference", None)
        commit = getattr(self, "_commit_graph_route_candidate", None)
        if probe_id > 0 and callable(graph_prepare):
            snapshot_epoch = getattr(
                getattr(getattr(snapshot, "map_context", None), "components", None),
                "epoch",
                None,
            )
            graph_plan = graph_prepare(
                map_epoch=snapshot_epoch,
                visible_work_item_ids=(),
                visible_probe_ids=(probe_id,),
                visible_portal_ids=(),
            )
            if (
                graph_plan is None
                or getattr(graph_plan, "status", None) != PLAN_READY
                or not callable(commit)
                or not commit(
                    graph_plan,
                    candidate,
                    source="durable_portal_probe_fallback",
                )
            ):
                self.publish_status(
                    "graph_route_materialization_wait",
                    action=(
                        None if graph_plan is None else graph_plan.action
                    ),
                    obligation_kind=(
                        None
                        if graph_plan is None else graph_plan.obligation_kind
                    ),
                    obligation_id=(
                        None if graph_plan is None else graph_plan.obligation_id
                    ),
                    reason="durable_probe_fallback_plan_not_committed",
                )
                return None, "graph_route_materialization_wait", True
        self.graph_route_probe_id = None
        return candidate, "durable_portal_probe", False

    def select_next_active_frontier(self, snapshot):
        """Choose a terminal successor after resolving cache and claim policy."""
        snapshot_epoch = getattr(
            getattr(getattr(snapshot, "map_context", None), "components", None),
            "epoch",
            None,
        )
        transaction = getattr(self, "portal_transaction", None)
        if (
            transaction is not None
            and transaction.active
            and (
                (
                    getattr(self, "pending_local_egress", None) is None
                    and getattr(self, "pending_portal_retry", None) is None
                )
                or transaction.state not in ("source_probe", "throat")
            )
        ):
            # A transaction outside its source-side retry phases owns the
            # route until its arrival is committed or explicitly closed. A
            # normal frontier or a premature retry is not a substitute for it.
            self.publish_status(
                "portal_transaction_selection_blocked",
                transaction_id=int(transaction.snapshot().transaction_id),
                route_id=int(transaction.snapshot().route_id),
                state=transaction.state,
                reason="portal_transaction_owned",
            )
            return None, "portal_transaction_wait", True
        durable_probe_checked = False
        candidate, selection_mode, wait_for_validation = (
            self.select_pending_local_egress(snapshot)
        )
        if candidate is not None or wait_for_validation:
            return candidate, selection_mode, wait_for_validation
        candidate, selection_mode, wait_for_validation = (
            self.select_pending_portal_retry(snapshot)
        )
        if candidate is not None or wait_for_validation:
            return candidate, selection_mode, wait_for_validation

        # The full graph method is a two-stage planner: the current map first
        # rehydrates WorkItems/Probes, then the durable graph owns the next
        # action.  The call below is deliberately the first graph entry point
        # for this snapshot; no prior successor may bypass projection.
        if getattr(self, "graph_route_planner_enabled", False):
            candidate = self._choose_frontier_for_new_action(
                snapshot.message,
                snapshot.route_graph,
                snapshot.map_context,
                snapshot.now,
                snapshot.robot_map,
                snapshot.robot_yaw_map,
            )
            if candidate is not None:
                graph_candidate = getattr(
                    self, "last_graph_selected_candidate", None
                ) or candidate
                graph_plan = getattr(self, "last_graph_route_plan", None)
                if graph_plan is None:
                    graph_prepare = getattr(
                        self, "_prepare_graph_route_action", None
                    )
                    if graph_prepare is not None:
                        projection_kwargs = (
                            self._current_graph_snapshot_prepare_kwargs(
                                snapshot_epoch
                            )
                        )
                        if projection_kwargs is None:
                            return (
                                None,
                                "graph_route_rehydration_wait",
                                True,
                            )
                        graph_plan, _unused_graph_candidate = graph_prepare(
                            None, **projection_kwargs
                        )
                commit = getattr(self, "_commit_graph_route_candidate", None)
                if graph_plan is None or commit is None:
                    return candidate, self.last_frontier_selection_mode, False
                if getattr(graph_plan, "status", None) != PLAN_READY:
                    # A blocked/complete graph decision is a hard barrier. Do
                    # not feed its candidate into the transaction layer: that
                    # only creates an event storm with no possible commit.
                    self.publish_status(
                        "graph_route_materialization_wait",
                        status=getattr(graph_plan, "status", None),
                        action=getattr(graph_plan, "action", None),
                        obligation_kind=getattr(graph_plan, "obligation_kind", ""),
                        obligation_id=getattr(graph_plan, "obligation_id", None),
                        reason="graph_plan_not_ready_before_candidate_commit",
                    )
                    return None, "graph_route_materialization_wait", True
                if graph_plan_is_exclusive(graph_plan) and not candidate_refines_graph_plan(
                    graph_plan, graph_candidate
                ):
                    signature = (
                        graph_plan.signature(),
                        getattr(graph_candidate, "graph_action", None),
                        getattr(graph_candidate, "work_item_id", None),
                        getattr(graph_candidate, "portal_id", None),
                    )
                    if signature != getattr(
                        self, "last_graph_candidate_unavailable_signature", None
                    ):
                        self.last_graph_candidate_unavailable_signature = signature
                        self.publish_status(
                            "graph_route_candidate_unavailable",
                            action=str(graph_plan.action),
                            obligation_id=graph_plan.obligation_id,
                            first_portal_id=graph_plan.first_portal_id,
                            reason="candidate_does_not_refine_graph_plan",
                        )
                    # This is a snapshot-local candidate mismatch, not a
                    # controller failure.  Keep the durable plan leased and
                    # let a later projection or the bounded miss tracker
                    # decide whether the route is actually unavailable.
                    return None, "graph_route_materialization_wait", True
                if commit(graph_plan, graph_candidate, source="frontier_selector"):
                    return candidate, self.last_frontier_selection_mode, False
                # The transaction validator has already emitted the exact
                # mismatch reason. Keep the durable intent pending instead of
                # letting a second selector branch replace it.
                return None, "graph_route_materialization_wait", True

            # A leased Portal action must get one real current-snapshot
            # materialization attempt before the post-arrival rehydration gate
            # can turn an absent frontier projection into a generic wait. The
            # helper preserves the graph identity and routes misses through
            # the common negative-evidence/invalidation tracker.
            graph_successor = self._materialize_graph_successor_first(
                snapshot, snapshot_epoch
            )
            if graph_successor is not None:
                return graph_successor

            # If the context pass could not run (for example a transient
            # missing frontier projection), keep the entry barrier intact.
            if getattr(self, "place_entry_rehydration_pending", None) is not None:
                self.publish_status(
                    "place_entry_rehydration_wait",
                    place_id=getattr(self, "current_physical_place_id", None),
                    reason="successor_requires_first_post_arrival_snapshot",
                )
                return None, "place_entry_rehydration_wait", True
            # A terminal can leave a durable graph probe ready while an
            # unrelated snapshot candidate is still waiting for Navfn. Give
            # the graph adapter first chance to materialize its successor;
            # otherwise the ordinary validation wait can strand the released
            # route until the materialization watchdog fires.
            projection_kwargs = self._current_graph_snapshot_prepare_kwargs(
                snapshot_epoch
            )
            if projection_kwargs is None:
                self.publish_status(
                    "graph_route_rehydration_wait",
                    map_epoch=snapshot_epoch,
                    reason="snapshot_projection_pending",
                )
                return None, "graph_route_rehydration_wait", True
            graph_prepare = getattr(self, "_prepare_graph_route_action", None)
            if graph_prepare is None:
                graph_plan, graph_candidate = None, None
            else:
                graph_plan, graph_candidate = graph_prepare(
                    snapshot,
                    reuse_existing_plan=True,
                    **projection_kwargs,
                )
            if graph_candidate is not None:
                return graph_candidate, "graph_route_edge", False
            # ``choose_valid_frontier`` may have selected the exact durable
            # WorkItem/Portal candidate while its asynchronous Navfn check is
            # still pending.  That is a validation wait, not evidence that
            # graph materialization failed.  Do not release the controller
            # lease here: the Navfn worker schedules a planning wake when the
            # result arrives, and the same graph intent must be retried then.
            if getattr(self, "graph_route_materialization_pending", False):
                retain_lease = getattr(
                    self, "_retain_graph_route_plan_lease", None
                )
                if retain_lease is not None and graph_plan is not None:
                    retain_lease(
                        graph_plan, "same_snapshot_materialization_miss"
                    )
                publish = getattr(self, "publish_status", None)
                if publish is not None and graph_plan is not None:
                    publish(
                        "graph_route_materialization_wait",
                        status=graph_plan.status,
                        action=graph_plan.action,
                        obligation_kind=graph_plan.obligation_kind,
                        obligation_id=graph_plan.obligation_id,
                        portal_path=list(graph_plan.portal_path),
                        first_portal_id=graph_plan.first_portal_id,
                        reason="same_snapshot_materialization_miss",
                    )
                return None, "graph_route_materialization_wait", True

            graph_plan = getattr(self, "last_graph_route_plan", None)
            if (
                getattr(self, "frontier_validation_pending", False)
                and graph_plan is not None
                and getattr(graph_plan, "status", None) == PLAN_READY
            ):
                retain_lease = getattr(
                    self, "_retain_graph_route_plan_lease", None
                )
                if retain_lease is not None:
                    retain_lease(graph_plan, "navfn_validation_pending")
                publish = getattr(self, "publish_status", None)
                if publish is not None:
                    signature = (
                        graph_plan.signature(),
                        "navfn_validation_pending",
                    )
                    if signature != getattr(
                        self, "last_graph_validation_wait_signature", None
                    ):
                        self.last_graph_validation_wait_signature = signature
                        publish(
                            "graph_route_materialization_wait",
                            status=graph_plan.status,
                            action=graph_plan.action,
                            obligation_kind=graph_plan.obligation_kind,
                            obligation_id=graph_plan.obligation_id,
                            portal_path=list(graph_plan.portal_path),
                            first_portal_id=graph_plan.first_portal_id,
                            reason="navfn_validation_pending",
                        )
                return None, "graph_route_materialization_wait", True
            if (
                graph_plan is not None
                and getattr(graph_plan, "status", None) == PLAN_READY
            ):
                record_miss = getattr(
                    self, "_record_graph_route_materialization_miss", None
                )
                if callable(record_miss) and record_miss(
                    graph_plan,
                    snapshot_epoch,
                    "selected_graph_obligation_not_executable_in_snapshot",
                ):
                    return None, "graph_route_next_action", True
                retain_lease = getattr(self, "_retain_graph_route_plan_lease", None)
                if retain_lease is not None:
                    retain_lease(
                        graph_plan,
                        "selected_graph_obligation_not_executable_in_snapshot",
                    )
                signature = graph_plan.signature()
                if signature != getattr(
                    self, "last_graph_materialization_wait_signature", None
                ):
                    self.last_graph_materialization_wait_signature = signature
                    self.publish_status(
                        "graph_route_materialization_wait",
                        status=graph_plan.status,
                        action=graph_plan.action,
                        obligation_kind=graph_plan.obligation_kind,
                        obligation_id=graph_plan.obligation_id,
                        portal_path=list(graph_plan.portal_path),
                        first_portal_id=graph_plan.first_portal_id,
                        reason="selected_graph_obligation_not_executable_in_snapshot",
                    )
                # Do not publish ``frontier_route_unavailable`` here.  The
                # miss is still retryable; that event is a controller
                # ownership boundary and is emitted only by the exhausted
                # invalidation path with an explicit release decision.
                return None, "graph_route_materialization_wait", True
            # A blocked current-place plan may still have a durable ledger
            # fallback.  The graph planner has already excluded the exhausted
            # identity; let the dedicated Portal probe/egress materializers
            # find a different durable identity before reporting a hard hold.
            # Generic score candidates remain blocked, so this does not bypass
            # the graph contract with an unrelated frontier endpoint.
            if (
                graph_plan is not None
                and getattr(graph_plan, "status", None) == "blocked"
                and str(getattr(graph_plan, "reason", "") or "") in {
                    "current_place_obligation_not_materialized",
                    "durable_obligation_has_no_certified_graph_path",
                }
            ):
                signature = graph_plan.signature()
                if signature != getattr(
                    self, "last_graph_blocked_fallback_signature", None
                ):
                    self.last_graph_blocked_fallback_signature = signature
                    self.publish_status(
                        "graph_route_blocked_fallback_to_durable_ledger",
                        action=str(graph_plan.action),
                        reason=str(graph_plan.reason),
                        current_place_id=graph_plan.current_place_id,
                    )
                candidate, selection_mode, wait_for_validation = (
                    self._select_durable_portal_probe_fallback(snapshot)
                )
                durable_probe_checked = True
                if candidate is not None or wait_for_validation:
                    return candidate, selection_mode, wait_for_validation
            else:
                # A complete plan, or a blocked plan caused by an unrelated
                # invariant, remains a hard barrier for this selector pass.
                return None, self.last_frontier_selection_mode, False

        candidate, selection_mode, wait_for_validation = (
            self.terminal_prefetch_novelty_decision(snapshot)
        )
        if candidate is not None or wait_for_validation:
            return candidate, selection_mode, wait_for_validation
        promoted = self.promote_prefetched_frontier(
            snapshot.message,
            snapshot.route_graph.route_steps,
            snapshot.robot_map,
            snapshot.now,
            validation=snapshot.route_graph.validation,
        )
        if promoted is not None:
            candidate = self.active_cell_from_promoted_frontier(
                snapshot.route_graph.validation,
                snapshot.route_graph.route_steps,
                snapshot.message.info.resolution,
                promoted,
            )
            return candidate, "prefetched_successor", False

        candidate = self._choose_frontier_for_new_action(
            snapshot.message,
            snapshot.route_graph,
            snapshot.map_context,
            snapshot.now,
            snapshot.robot_map,
            snapshot.robot_yaw_map,
        )
        if candidate is not None:
            return candidate, self.last_frontier_selection_mode, False
        if (
            candidate is None
            and not self.frontier_validation_budget_exhausted
            and not self.frontier_validation_pending
            and not durable_probe_checked
        ):
            durable_probe = getattr(self, "select_durable_portal_probe", None)
            if durable_probe is not None:
                candidate = durable_probe(snapshot)
                if candidate is not None:
                    probe = candidate[15] if len(candidate) > 15 else None
                    try:
                        probe_id = int(getattr(probe, "probe_id", 0) or 0)
                    except (TypeError, ValueError):
                        probe_id = 0
                    graph_prepare = getattr(
                        self, "_set_graph_route_preference", None
                    )
                    commit = getattr(
                        self, "_commit_graph_route_candidate", None
                    )
                    if probe_id > 0 and callable(graph_prepare):
                        graph_plan = graph_prepare(
                            map_epoch=snapshot_epoch,
                            visible_work_item_ids=(),
                            visible_probe_ids=(probe_id,),
                            visible_portal_ids=(),
                        )
                        if (
                            graph_plan is None
                            or getattr(graph_plan, "status", None) != PLAN_READY
                            or not callable(commit)
                            or not commit(
                                graph_plan,
                                candidate,
                                source="durable_portal_probe_fallback",
                            )
                        ):
                            self.publish_status(
                                "graph_route_materialization_wait",
                                action=(
                                    None
                                    if graph_plan is None
                                    else graph_plan.action
                                ),
                                obligation_kind=(
                                    None
                                    if graph_plan is None
                                    else graph_plan.obligation_kind
                                ),
                                obligation_id=(
                                    None
                                    if graph_plan is None
                                    else graph_plan.obligation_id
                                ),
                                reason=(
                                    "durable_probe_fallback_plan_not_committed"
                                ),
                            )
                            return None, "graph_route_materialization_wait", True
                    self.graph_route_probe_id = None
                    return candidate, (
                        "durable_portal_probe"
                    ), False
            durable_egress = getattr(
                self, "select_durable_portal_egress", None
            )
            if durable_egress is not None:
                candidate = durable_egress(snapshot)
                if candidate is not None:
                    self.graph_route_portal_id = None
                    return candidate, (
                        "durable_portal_egress"
                    ), False
        if (
            candidate is None
            and self.target_region_claim_active
            and not self.frontier_validation_budget_exhausted
            and not self.frontier_validation_pending
        ):
            self._release_exhausted_target_region_claim()
            candidate = self._choose_frontier_for_new_action(
                snapshot.message,
                snapshot.route_graph,
                snapshot.map_context,
                snapshot.now,
                snapshot.robot_map,
                snapshot.robot_yaw_map,
            )
        return candidate, self.last_frontier_selection_mode, False
