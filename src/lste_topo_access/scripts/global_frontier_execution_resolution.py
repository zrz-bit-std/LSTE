"""Terminal and failure resolution for an active frontier route."""

import rospy


class GlobalFrontierExecutionResolutionMixin:
    """Commit a finished route or invalidate it with complete lifecycle data."""

    def observation_standoff_reached(
        self, distance, route_distance, watchdog, portal_transition,
    ):
        """Return whether a local WorkItem route reached its view contract.

        A frontier endpoint is an observation viewpoint, not a docking pose.
        The viewpoint compiler deliberately places it inside the safe route
        mask, so a controller may settle short of the raw unknown cell. Once
        both the geometric goal and validated route are inside the existing
        terminal envelope, a stalled endpoint route is an observation result
        when it owns a normal local WorkItem. Portal probes and recovery routes
        stay on their own physical-evidence contracts.
        """
        if portal_transition:
            return False
        if str(getattr(self, "active_route_kind", "")) != "frontier_endpoint":
            return False
        if str(
            getattr(self, "active_mission_route_kind", "frontier_endpoint")
        ) != "frontier_endpoint":
            return False
        if getattr(self, "active_place_hops", None) not in (0,):
            return False
        if getattr(self, "active_work_item_id", None) is None:
            return False
        # A doorway WorkItem is owned by the PortalProbe transaction once it
        # has been admitted. It must not be resolved by ordinary standoff.
        if getattr(self, "active_portal_probe_id", None) is not None:
            return False
        if bool(getattr(watchdog, "recovery_pending", False)):
            return False
        if bool(getattr(watchdog, "turn_phase_active", False)):
            return False
        if not (
            bool(getattr(watchdog, "stalled", False))
            or bool(getattr(watchdog, "post_turn_stalled", False))
        ):
            return False
        try:
            radius = float(self.endpoint_terminal_wait_radius)
            goal_distance = float(distance)
            path_distance = float(route_distance)
            waypoint_distance = float(
                getattr(watchdog, "waypoint_distance", goal_distance)
            )
        except (TypeError, ValueError):
            return False
        return (
            radius > 0.0
            and goal_distance <= radius
            and path_distance <= radius
            and waypoint_distance <= radius
        )

    def complete_observation_standoff(
        self, x, y, distance, route_distance, now, active_component, watchdog,
    ):
        """Commit a local observation reached at a safe viewpoint."""
        if not self.observation_standoff_reached(
            distance,
            route_distance,
            watchdog,
            portal_transition=False,
        ):
            return False
        observed_region = self.mark_frontier_observed(
            x,
            y,
            component=active_component,
            region_id=self.active_frontier_region_id,
            now=now,
        )
        self.remember_completed_observation_source(observed_region)
        self.publish_status(
            "route_invalidated",
            reason="frontier_observed_at_standoff",
            observation_contract="local_work_standoff",
            route_id=int(self.active_route_id),
            route_kind=self.active_route_kind,
            goal=[round(float(x), 3), round(float(y), 3)],
            observation_distance=round(float(distance), 3),
            path_distance=round(float(route_distance), 3),
            last_progress_signal=self.active_last_progress_signal,
            watchdog_state=(
                "post_turn_stalled"
                if bool(getattr(watchdog, "post_turn_stalled", False))
                else "stalled"
            ),
        )
        self.release_active_frontier(
            discard_prefetch=True,
            preserve_place_departure=bool(self.place_departure.active),
        )
        return True

    def release_if_active_frontier_observed(
        self, message, known_free, unknown, row, col, x, y, robot_map,
        distance, route_distance, active_component,
    ):
        """Release an ordinary endpoint once the current lidar view resolved it."""
        if (
            distance >= self.dead_end_check_distance
            or not self.active_endpoint_observed_from_map(
                message,
                known_free,
                unknown,
                row,
                col,
                robot_map,
                distance,
            )
        ):
            return False

        # Observation completes a frontier action even when a wall-adjacent
        # endpoint cannot make move_base emit a geometric SUCCEEDED terminal.
        observed_region = self.mark_frontier_observed(
            x,
            y,
            component=active_component,
            region_id=self.active_frontier_region_id,
        )
        self.remember_completed_observation_source(observed_region)
        self.publish_status(
            "route_invalidated",
            reason="frontier_observed_at_standoff",
            route_id=int(self.active_route_id),
            route_kind=self.active_route_kind,
            goal=[round(float(x), 3), round(float(y), 3)],
            observer=[round(float(robot_map[0]), 3), round(float(robot_map[1]), 3)],
            observation_distance=round(float(distance), 3),
            path_distance=round(float(route_distance), 3),
            best_path_distance=(
                None if self.active_best_path_distance is None
                else round(float(self.active_best_path_distance), 3)
            ),
            best_goal_distance=(
                None if self.active_best_goal_distance is None
                else round(float(self.active_best_goal_distance), 3)
            ),
            last_progress_signal=self.active_last_progress_signal,
        )
        rospy.loginfo(
            "Global frontier completed observation at standoff "
            "endpoint=(%.2f,%.2f) observer=(%.2f,%.2f) distance=%.2fm",
            x,
            y,
            robot_map[0],
            robot_map[1],
            distance,
        )
        self.release_active_frontier(
            discard_prefetch=True,
            preserve_place_departure=bool(self.place_departure.active),
        )
        return True

    def resolve_active_route_terminal_or_failure(
        self, row, col, x, y, distance, waypoint_reached, route_steps,
        resolution, now, portal_transition, active_component, route_distance,
        watchdog,
    ):
        """Handle terminal ownership, endpoint failure, and lifecycle logs."""
        endpoint_reached = (
            distance <= self.endpoint_terminal_wait_radius and waypoint_reached
        )
        terminal_required = self.mission_endpoint_only or self.persistent_execution
        # A failed MoveBase terminal is stronger evidence than geometric
        # proximity. The base can stop inside the wait envelope after local
        # recovery, but it cannot later emit the terminal for this lease.
        if watchdog.recovery_pending:
            reason = self.active_route_failure_reason(watchdog)
            self.deactivate_failed_active_frontier(
                x,
                y,
                distance,
                now,
                portal_transition,
                active_component,
                route_distance,
                reason,
                watchdog,
            )
            return None

        # A local observation WorkItem owns a safe viewpoint. Give that
        # semantic terminal precedence over the generic stall watchdog once
        # the route has reached its physical standoff envelope. Otherwise a
        # wall-adjacent frontier is misclassified as controller failure and
        # immediately spawns a needless local-egress recovery route.
        complete_standoff = getattr(
            self, "complete_observation_standoff", None
        )
        if callable(complete_standoff) and complete_standoff(
            x,
            y,
            distance,
            route_distance,
            now,
            active_component,
            watchdog,
        ):
            return None

        if watchdog.post_turn_stalled or watchdog.expired:
            reason = self.active_route_failure_reason(watchdog)
            self.deactivate_failed_active_frontier(
                x,
                y,
                distance,
                now,
                portal_transition,
                active_component,
                route_distance,
                reason,
                watchdog,
            )
            return None
        if endpoint_reached and (
            not terminal_required or self.active_terminal_received
        ):
            if not portal_transition:
                observed_region = self.mark_frontier_observed(
                    x,
                    y,
                    component=active_component,
                    region_id=self.active_frontier_region_id,
                )
                self.remember_completed_observation_source(observed_region)
            self.release_active_frontier(
                preserve_place_departure=bool(self.place_departure.active),
            )
            if self.prefetched_frontier is None:
                self.prefetched_goal_map = None
            return None

        if endpoint_reached and terminal_required:
            # Map coverage and endpoint distance do not own the controller
            # transaction. Wait for the bridge's matching TEB terminal.
            rospy.loginfo_throttle(
                2.0,
                "Global frontier reached endpoint geometry; "
                "waiting for matching TEB terminal route_id=%d "
                "distance=%.2fm terminal_wait_radius=%.2fm",
                self.active_route_id,
                distance,
                self.endpoint_terminal_wait_radius,
            )
            return row, col, x, y, route_steps[row, col] * resolution, 0.0

        reason = self.active_route_failure_reason(watchdog)
        self.deactivate_failed_active_frontier(
            x,
            y,
            distance,
            now,
            portal_transition,
            active_component,
            route_distance,
            reason,
            watchdog,
        )
        return None

    def active_route_failure_reason(self, watchdog):
        """Choose the single lifecycle reason for a stalled route lease."""
        if watchdog.recovery_pending:
            return self.recovery_pending_reason or "move_base_terminal_failure"
        if watchdog.post_turn_stalled:
            return "post_turn_no_progress"
        if getattr(watchdog, "portal_edge_expired", False):
            return "portal_edge_deadline"
        return "stall" if watchdog.stalled else "active_timeout"

    def deactivate_failed_active_frontier(
        self, x, y, distance, now, portal_transition, active_component,
        route_distance, reason, watchdog,
    ):
        """Record a failed route lease before allowing another selector pass."""
        # ``settle_active_work_item`` and ``release_active_frontier`` clear the
        # mutable attempt/route fields. Capture the committed graph identity
        # first so an active WorkItem failure can enter the same canonical
        # materialization exclusion path as a pre-dispatch Navfn rejection.
        durable_plan = None
        durable_failure_epoch = getattr(self, "last_map_epoch", None)
        find_plan = getattr(
            self, "_graph_route_plan_for_active_durable_obligation", None
        )
        if callable(find_plan):
            durable_plan = find_plan()
        if durable_failure_epoch is None:
            durable_failure_epoch = getattr(self, "active_route_map_epoch", None)
        settle_probe = getattr(self, "settle_active_portal_probe", None)
        if settle_probe is not None:
            settle_probe("failed", now, reason)
        if not portal_transition:
            settle = getattr(self, "settle_active_work_item", None)
            if settle is not None:
                settle(now, "deferred", reason)
        self.rejected_frontiers.append((now, x, y))
        portal_retry = None
        physical_crossing_committed = bool(
            getattr(self, "active_portal_crossing_observed", False)
        )
        if portal_transition and not physical_crossing_committed:
            ledger = getattr(self, "portal_hypothesis_ledger", None)
            portal_id = int(getattr(self, "last_portal_hypothesis_id", 0) or 0)
            if ledger is not None and portal_id > 0:
                record = getattr(ledger, "get", lambda _id: None)(portal_id)
                is_crossed = (
                    isinstance(record, dict)
                    and str(record.get("state", "")).strip().lower()
                    == "crossed"
                )
                recorder = getattr(ledger, "execution_failed", None)
                if is_crossed:
                    failed = (
                        record
                        if not callable(recorder)
                        else recorder(portal_id, now=now)
                    )
                    failure_event = "portal_execution_failure_observed"
                else:
                    failed = ledger.failed(portal_id, now=now)
                    failure_event = "portal_hypothesis_failed"
                if failed is not None:
                    self.publish_status(
                        failure_event,
                        portal_id=int(failed["id"]),
                        source_place_id=int(failed["source_place_id"]),
                        route_id=int(self.active_route_id),
                        reason=str(reason),
                        failure_count=int(failed.get("failure_count", 0)),
                        state=str(failed.get("state", "")),
                        execution_failure_count=int(
                            failed.get("execution_failure_count", 0)
                        ),
                    )
            portal_retry = self.prepare_portal_transition_retry(reason)
        elif (
            self.active_route_kind == "local_egress"
            and self.active_local_egress_resumes_portal
        ):
            self.discard_pending_portal_retry("egress_failed:%s" % reason)
        local_egress_queued = (
            portal_retry is not None
            and self.prepare_local_egress(
                reason,
                source_region_id=self.active_frontier_region_id,
            )
        )
        if portal_retry is not None and not local_egress_queued:
            self.discard_pending_portal_retry("no_reached_egress_anchor")
        if not portal_transition:
            local_egress_queued = self.prepare_local_egress(
                reason,
                source_region_id=self.active_frontier_region_id,
            )
        # A prefetched successor was scored from the failed route's topology
        # snapshot.  Its route proof and WorkItem ownership are no longer
        # valid after that failure, even when no local egress anchor exists.
        # Never promote stale cache state as a fresh exploration action.
        if getattr(self, "prefetched_frontier", None) is not None:
            self.clear_prefetched_frontier()
        if local_egress_queued and not portal_transition:
            failed_region = self.region_memory.retain_for_local_egress(
                self.active_frontier_region_id,
                now,
                reason,
            )
        else:
            failed_region = self.record_failed_frontier_region(
                x, y, now, reason, active_component,
            )
        rospy.logwarn(
            "Global frontier deferred map=(%.2f,%.2f): %s "
            "distance=%.2fm elapsed=%.1fs; suppress for %.0fs",
            x,
            y,
            reason,
            distance,
            now - self.active_since,
            self.rejected_timeout,
        )
        self.publish_failed_route_invalidation(
            x,
            y,
            distance,
            now,
            route_distance,
            reason,
            watchdog,
            failed_region,
            local_egress_queued,
            portal_retry is not None and local_egress_queued,
        )
        terminal_contract = getattr(self, "active_terminal_contract", None)
        terminal_contract = (
            dict(terminal_contract)
            if isinstance(terminal_contract, dict)
            else None
        )
        # A watchdog failure can invalidate and cancel the route before the
        # typed MoveBase terminal reaches Global Frontier.  The Bridge already
        # has the dispatch-time contract in that case, so request the same
        # exact release/ACK boundary instead of publishing a semantic-only
        # release that leaves the Bridge's transport tombstone pending.
        if terminal_contract is None:
            dispatch_contracts = (
                getattr(self, "_bridge_dispatch_contracts", None) or {}
            )
            candidate = dispatch_contracts.get(
                int(getattr(self, "active_route_id", 0) or 0)
            )
            if isinstance(candidate, dict):
                terminal_contract = dict(candidate)
        await_controller_release_ack = bool(
            getattr(self, "controller_owned_route_failure", False)
            and int(getattr(self, "active_route_id", 0) or 0) > 0
            and isinstance(terminal_contract, dict)
        )
        self.release_active_frontier(
            queue_successor=not await_controller_release_ack,
            await_controller_release_ack=await_controller_release_ack,
            terminal_contract=terminal_contract,
        )
        if await_controller_release_ack:
            self.publish_status(
                "controller_failure_release_pending",
                route_id=int(getattr(self, "last_released_route_id", 0) or 0),
                transaction_id=int(
                    getattr(self, "last_released_route_transaction_id", 0)
                    or 0
                ),
                action_generation=int(
                    getattr(self, "last_released_route_action_generation", 0)
                    or 0
                ),
                lifecycle_transaction_id=int(
                    getattr(
                        self,
                        "last_released_route_lifecycle_transaction_id",
                        0,
                    )
                    or 0
                ),
                graph_transaction_id=int(
                    getattr(
                        self, "last_released_route_graph_transaction_id", 0
                    )
                    or 0
                ),
                map_epoch=getattr(self, "last_released_route_map_epoch", None),
                origin_epoch=getattr(
                    self, "last_released_route_origin_epoch", None
                ),
                projection_epoch=getattr(
                    self, "last_released_route_projection_epoch", None
                ),
                reason=str(reason),
            )
        if durable_plan is not None:
            # The route lease is already released above. The adapter now owns
            # the durable obligation miss budget and can invalidate it without
            # a second controller cancellation or a stale mutable attempt ID.
            record_failure = getattr(
                self,
                "_record_active_durable_materialization_failure",
                None,
            )
            if callable(record_failure):
                record_failure(
                    plan=durable_plan,
                    map_epoch=durable_failure_epoch,
                    reason=str(reason),
                )
        if self.prefetched_frontier is None:
            self.prefetched_goal_map = None

    def record_failed_frontier_region(self, x, y, now, reason, component):
        """Update place memory for a failed observation route when applicable."""
        if self.active_route_kind == "portal_transition":
            return None
        if self.active_route_kind == "local_egress":
            source_region_id = self.local_egress_place_lease.fail()
            if source_region_id is None:
                return None
            failed_region = self.region_memory.fail(
                x,
                y,
                now,
                reason,
                component=component,
                region_id=source_region_id,
            )
            if failed_region is not None and failed_region["state"] == "dormant":
                self.publish_status(
                    "frontier_region_dormant",
                    reason=failed_region["last_reason"],
                    region_id=int(failed_region["id"]),
                    region_visits=int(failed_region["visits"]),
                    region_failures=int(failed_region["failures"]),
                    goal=[round(float(x), 3), round(float(y), 3)],
                )
            return failed_region
        failed_region = self.region_memory.fail(
            x,
            y,
            now,
            reason,
            component=component,
            region_id=self.active_frontier_region_id,
        )
        if failed_region is not None and failed_region["state"] == "dormant":
            self.publish_status(
                "frontier_region_dormant",
                reason=failed_region["last_reason"],
                region_id=int(failed_region["id"]),
                region_visits=int(failed_region["visits"]),
                region_failures=int(failed_region["failures"]),
                goal=[round(float(x), 3), round(float(y), 3)],
            )
        return failed_region

    def publish_failed_route_invalidation(
        self, x, y, distance, now, route_distance, reason, watchdog,
        failed_region, local_egress_queued, portal_retry_queued,
    ):
        """Publish the complete, externally visible record for one failure."""
        # Consumers must explicitly release the old move_base action; this is
        # a mission lifecycle event, not private selector bookkeeping.
        terminal_contract = getattr(self, "active_terminal_contract", None)
        terminal_contract = (
            terminal_contract if isinstance(terminal_contract, dict) else {}
        )

        def identity_int(value, fallback=0):
            try:
                return int(value or fallback)
            except (TypeError, ValueError):
                return int(fallback)

        self.publish_status(
            "route_invalidated",
            reason=reason,
            transaction_id=identity_int(
                terminal_contract.get("transaction_id")
            ),
            action_generation=identity_int(
                terminal_contract.get("action_generation")
            ),
            lifecycle_transaction_id=identity_int(
                terminal_contract.get("lifecycle_transaction_id")
            ),
            graph_transaction_id=identity_int(
                terminal_contract.get("graph_transaction_id")
            ),
            map_epoch=terminal_contract.get("map_epoch"),
            origin_epoch=terminal_contract.get("origin_epoch"),
            projection_epoch=terminal_contract.get("projection_epoch"),
            goal=[round(float(x), 3), round(float(y), 3)],
            distance=round(float(distance), 3),
            elapsed=round(float(now - self.active_since), 3),
            path_distance=round(float(route_distance), 3),
            best_path_distance=(
                None if self.active_best_path_distance is None
                else round(float(self.active_best_path_distance), 3)
            ),
            best_goal_distance=(
                None if self.active_best_goal_distance is None
                else round(float(self.active_best_goal_distance), 3)
            ),
            odom_detour_distance=round(
                float(self.active_best_detour_odom_distance), 3
            ),
            odom_novel_cells=len(self.active_visited_odom_cells),
            last_progress_signal=self.active_last_progress_signal,
            post_turn_elapsed=(
                None if not watchdog.post_turn_goal_matches
                else round(float(watchdog.post_turn_elapsed), 3)
            ),
            post_turn_odom_translation=(
                None if watchdog.post_turn_translation is None
                else round(float(self.active_turn_completed_translation), 3)
            ),
            post_turn_launched=bool(self.active_turn_completed_launched),
            recovery_behavior=(
                self.recovery_pending_behavior if watchdog.recovery_pending else None
            ),
            recovery_reason=(
                self.recovery_pending_reason if watchdog.recovery_pending else None
            ),
            region_id=(None if failed_region is None else int(failed_region["id"])),
            region_state=(None if failed_region is None else failed_region["state"]),
            region_failures=(
                None if failed_region is None else int(failed_region["failures"])
            ),
            local_egress_queued=bool(local_egress_queued),
            portal_retry_queued=bool(portal_retry_queued),
        )
