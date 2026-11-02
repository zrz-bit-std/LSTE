"""Terminal and failure resolution for an active frontier route."""

import rospy


PORTAL_AUTH_RETRY_MIN_SECONDS = 8.0
PORTAL_AUTH_RETRY_MAX_SECONDS = 12.0


def portal_authentication_retry_budget(active_timeout):
    # Allow a fresh lease for a transient fault, but bound the whole doorway
    # phase well below the former 60–90 second retry window.
    try:
        timeout = float(active_timeout)
    except (TypeError, ValueError):
        timeout = PORTAL_AUTH_RETRY_MIN_SECONDS
    return min(
        PORTAL_AUTH_RETRY_MAX_SECONDS,
        max(PORTAL_AUTH_RETRY_MIN_SECONDS, 0.5 * timeout),
    )


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
        # Authentication entry/turn/return routes share the ordinary endpoint
        # kind, but their terminal contract belongs to the doorway state machine.
        if str(getattr(self, "active_portal_authentication_phase", "") or ""):
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
        auth_phase = str(getattr(
            self, "active_portal_authentication_phase", "") or "")
        auth = getattr(self, "active_portal_authentication", lambda: None)()
        auth_turn_watchdog_grace = False
        auth_turning = auth is not None and auth.state == "turning"
        if auth_turning:
            pose = getattr(self, "pose_odom", None)
            if pose is not None:
                auth.update_rotation(float(pose.theta))
            stage_key = (int(auth.branch_id), "turning")
            if getattr(self, "portal_auth_watchdog_stage_key", None) != stage_key:
                # Entry and turn share the junction transaction, but rotation
                # is a fresh controller stage with almost no XY progress.
                # Reset its route deadline rather than inheriting the entry
                # lease's elapsed time.
                self.portal_auth_watchdog_stage_key = stage_key
                self.portal_auth_watchdog_turn_started_at = now
                self.active_since = now
                self.active_progress_time = now
                self.active_best_distance = float(distance)
                self.active_best_goal_distance = float(distance)
            turn_started = float(getattr(
                self, "portal_auth_watchdog_turn_started_at", now))
            auth_turn_watchdog_grace = (
                float(now) - turn_started < portal_authentication_retry_budget(self.active_timeout)
            )
        elif getattr(self, "portal_auth_watchdog_stage_key", None) is not None:
            self.portal_auth_watchdog_stage_key = None
            self.portal_auth_watchdog_turn_started_at = None
        auth_entry_approach_pending = bool(
            auth_phase == "entering" and auth is not None
            and auth.state == "entering"
            and getattr(self, "active_portal_authentication_entry_approach", False)
        )
        endpoint_tolerance = (
            max(float(self.endpoint_terminal_wait_radius), 0.55)
            if auth_entry_approach_pending
            else float(self.endpoint_terminal_wait_radius)
        )
        endpoint_reached = distance <= endpoint_tolerance and waypoint_reached
        if auth_turning:
            # XY arrival alone is not a completed turn connector. Keep the
            # locked-yaw route live until the authentication pose samples are
            # stable inside the angular tolerance.
            endpoint_reached = endpoint_reached and bool(auth.rotation_ready)
        auth_return_contract = bool(
            auth_phase == "returning" and auth is not None
            and auth.state in ("completed", "failed")
        )
        if auth_return_contract:
            # update_return commits only inside the source node's return radius.
            # That physical contract is wider than the generic waypoint radius.
            endpoint_reached = True
        terminal_required = self.mission_endpoint_only or self.persistent_execution
        # The turn contract is established by stable measured yaw at the
        # locked target. Persistent MoveBase may omit a terminal for the
        # stationary connector, so do not hold the doorway ledger on it.
        auth_turn_contract = bool(auth_turning and auth is not None
                                  and auth.rotation_ready)
        auth_entry_approach_contract = bool(
            auth_phase == "entering" and auth is not None
            and auth.state == "entering" and endpoint_reached
            and getattr(self, "active_portal_authentication_entry_approach", False)
        )
        # A failed MoveBase terminal is stronger evidence than geometric
        # proximity. The base can stop inside the wait envelope after local
        # recovery, but it cannot later emit the terminal for this lease.
        if watchdog.recovery_pending and not auth_turn_watchdog_grace:
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

        if ((watchdog.post_turn_stalled or watchdog.expired)
                and not auth_turn_watchdog_grace):
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
            or auth_turn_contract or auth_return_contract
            or auth_entry_approach_contract
        ):
            auth_phase = str(getattr(
                self, "active_portal_authentication_phase", "") or ""
            )
            auth_stage_only = bool(auth_phase)
            if auth_phase:
                auth = getattr(self, "active_portal_authentication", lambda: None)()
                if auth is None:
                    self.active_portal_authentication_phase = ""
                    auth_stage_only = False
                else:
                    pose = getattr(self, "pose_odom", None)
                    pose_xy = None if pose is None else (float(pose.x), float(pose.y))
                    pose_yaw = None if pose is None else float(pose.theta)
                    if auth.state == "failed":
                        stage_complete = True
                    elif auth_phase == "entering":
                        if auth.state == "entering":
                            auth.update_entry(pose_xy)
                            if auth.state == "inside":
                                auth.entry_approach_goal_odom = None
                                self.active_portal_authentication_entry_approach = False
                            if auth.state != "inside":
                                if getattr(self, "active_portal_authentication_entry_approach", False):
                                    self.active_portal_authentication_entry_approach = False
                                    auth.entry_approach_goal_odom = None
                                    self.publish_status(
                                        "portal_authentication_entry_approach_terminal",
                                        node_id=int(auth.node_id), branch_id=int(auth.branch_id),
                                        pose=None if pose_xy is None else list(pose_xy),
                                    )
                                else:
                                    auth.request_failure_return(
                                        "entry_terminal_before_fully_inside",
                                        pose_xy=pose_xy,
                                    )
                        # After full entry, wait for a fresh SLAM frontier scan
                        # before releasing the goal or accepting the branch.
                        stage_complete = auth.state in ("turning", "returning", "failed")
                    elif auth_phase == "turning":
                        if auth.rotation_ready and pose_yaw is not None:
                            auth.finish_turn(pose_yaw)
                        stage_complete = auth.state in ("returning", "failed")
                    elif auth_phase == "returning":
                        if auth.state == "returning":
                            auth.update_return(pose_xy)
                        stage_complete = auth.state in ("completed", "failed")
                    else:
                        stage_complete = False
                    if auth.state in ("completed", "failed"):
                        next_branch = self.finish_portal_authentication_branch(now=now)
                        self.publish_status(
                            "portal_authentication_route_terminal",
                            node_id=int(auth.node_id), branch_id=int(auth.branch_id),
                            route_id=int(self.active_route_id), phase=auth_phase,
                            state=auth.state, failure_reason=auth.failure_reason,
                            next_branch_id=None if next_branch is None
                            else int(next_branch.branch_id),
                        )
                    if not stage_complete:
                        return (row, col, x, y,
                                float(route_steps[row, col]) * resolution, 0.0)
                    self.active_portal_authentication_phase = ""
            if not portal_transition and not auth_stage_only:
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

        if auth_turning and auth_turn_watchdog_grace:
            # A turn connector intentionally has no XY progress. Keep its
            # lease alive while the locked-yaw controller settles; otherwise
            # the generic watchdog deactivates it on every planning cycle and
            # repeatedly restarts the same doorway turn.
            return (row, col, x, y,
                    float(route_steps[row, col]) * resolution, 0.0)

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
        auth_phase = str(getattr(
            self, "active_portal_authentication_phase", "") or ""
        )
        auth = getattr(self, "active_portal_authentication", lambda: None)()
        if auth is not None and auth.state == "entering":
            entry_key = int(auth.branch_id)
            if getattr(self, "portal_auth_entry_retry_branch_id", None) != entry_key:
                self.portal_auth_entry_retry_branch_id = entry_key
                self.portal_auth_entry_started_at = float(now)
            entry_started = float(getattr(
                self, "portal_auth_entry_started_at", now))
            entry_budget = portal_authentication_retry_budget(self.active_timeout)
            if float(now) - entry_started < entry_budget:
                self.publish_status(
                    "portal_authentication_entry_route_retry",
                    node_id=int(auth.node_id), branch_id=entry_key,
                    reason=str(reason), retry_budget_remaining=round(
                        entry_budget - (float(now) - entry_started), 2),
                )
                # Keep the same candidate and source node; a fresh controller
                # lease retries the same gate without counting a second entry.
                self.active_portal_authentication_phase = ""
                auth_phase = ""
        if auth is not None and auth.state == "turning":
            turn_started = getattr(
                self, "portal_auth_watchdog_turn_started_at", None)
            if turn_started is None:
                turn_started = float(now)
                self.portal_auth_watchdog_turn_started_at = turn_started
            turn_budget = portal_authentication_retry_budget(self.active_timeout)
            if float(now) - float(turn_started) < turn_budget:
                self.publish_status(
                    "portal_authentication_turn_route_retry",
                    node_id=int(auth.node_id), branch_id=int(auth.branch_id),
                    reason=str(reason), retry_budget_remaining=round(
                        turn_budget - (float(now) - float(turn_started)), 2),
                )
                # Retire the failed controller lease, keep the locked turn
                # target, and let GP activate a fresh connector for this branch.
                self.active_portal_authentication_phase = ""
                auth_phase = ""
            else:
                auth._event("doorway_navigation_phase_budget_expired",
                    phase="turning", timeout=float(turn_budget),
                    elapsed=round(float(now)-float(turn_started), 3),
                    reason=str(reason), route_id=int(getattr(self, "active_route_id", 0)))
        if auth is not None and auth.state == "returning":
            return_key = int(auth.branch_id)
            if getattr(self, "portal_auth_return_retry_branch_id", None) != return_key:
                self.portal_auth_return_retry_branch_id = return_key
                self.portal_auth_return_started_at = float(now)
            return_started = float(getattr(self, "portal_auth_return_started_at", now))
            return_budget = portal_authentication_retry_budget(self.active_timeout)
            if float(now) - return_started < return_budget:
                # A transient controller failure is not a false doorway. Keep
                # the same source node and retry returning under a fresh lease.
                self.publish_status(
                    "portal_authentication_return_route_retry",
                    node_id=int(auth.node_id), branch_id=return_key,
                    reason=str(reason), retry_budget_remaining=round(
                        return_budget - (float(now) - return_started), 2),
                )
                self.active_portal_authentication_phase = ""
                auth_phase = ""
        if (auth is not None and auth.state == "returning"
                and auth_phase == "returning"):
            # A controller terminal cannot fail/finish a branch while the robot
            # is still away from its source node. Retire this lease and let the
            # GP adapter select a bounded, Navfn-reachable source-side fallback.
            auth.note_return_route_failure(reason)
            self.publish_status(
                "portal_authentication_return_route_failed",
                node_id=int(auth.node_id), branch_id=int(auth.branch_id),
                reason=str(reason), source_xy=list(auth.source_xy or ()),
                retry_count=int(getattr(auth, "return_controller_failure_count", 0)),
            )
            self.active_portal_authentication_phase = ""
            auth_phase = ""
        if auth_phase:
            auth = getattr(self, "active_portal_authentication", lambda: None)()
            if (auth is not None and auth.state in ("turning", "returning")
                    and auth_phase != auth.state):
                # A terminal from the previous stage can arrive after the
                # authentication ledger has already advanced. Retire that
                # stale lease and let the selector activate the current stage.
                self.publish_status(
                    "portal_authentication_stale_stage_failure_ignored",
                    node_id=int(auth.node_id), branch_id=int(auth.branch_id),
                    failed_phase=auth_phase, current_phase=auth.state,
                    reason=str(reason),
                )
                self.active_portal_authentication_phase = ""
                auth_phase = ""
            if auth is not None and auth_phase and auth.state not in ("completed", "failed"):
                failure = "navigation_route_failed:%s" % str(reason)
                pose = getattr(self, "pose_odom", None)
                pose_xy = None if pose is None else (float(pose.x), float(pose.y))
                if auth_phase != "returning" and auth.request_failure_return(
                        failure, pose_xy=pose_xy):
                    if auth.state == "returning":
                        self.active_portal_authentication_phase = "returning"
                        self.publish_status(
                            "portal_authentication_failure_returning",
                            node_id=int(auth.node_id), branch_id=int(auth.branch_id),
                            reason=failure, source_xy=list(auth.source_xy),
                        )
                    else:
                        self.finish_portal_authentication_branch(now=now)
                        self.active_portal_authentication_phase = ""
                else:
                    auth.fail(failure)
                    self.finish_portal_authentication_branch(now=now)
                    self.active_portal_authentication_phase = ""
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
                # Navigation/controller failure does not disprove a doorway.
                # Keep the authenticated candidate discoverable for retry;
                # only doorway evidence (for example, no frontier after entry)
                # may transition it to failed.
                recorder = getattr(ledger, "execution_failed", None)
                map_epoch = getattr(self, "last_map_epoch", None)
                if map_epoch is None:
                    map_epoch = getattr(self, "topology_component_epoch", None)
                failed = (
                    record
                    if not callable(recorder)
                    else recorder(portal_id, now=now, map_epoch=map_epoch)
                )
                failure_event = "portal_execution_failure_observed"
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
