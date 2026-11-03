"""Watchdogs governing the lifecycle of an active frontier route."""

import math

import rospy

from global_frontier_models import ActiveRouteWatchdogState
from global_frontier_route_lease import route_lease_failure_decision


class GlobalFrontierExecutionWatchdogMixin:
    """Decide whether active-route progress remains healthy."""

    def evaluate_active_route_watchdogs(
        self, x, y, robot_map, now, portal_transition, active_component,
        active_information,
    ):
        """Evaluate turn, recovery, region, and timeout watchdogs."""
        # A turn can be owned directly by the TEB adapter for endpoint, portal,
        # and local-egress actions; it is not always represented by the legacy
        # ``frontier_turn_connector`` route kind.  Treat a matching supervisor
        # TURNING state as physical progress.  Without this boundary, a valid
        # in-place rotation is judged by the translational stall clock and the
        # route is cancelled while the base is still acquiring its heading.
        supervisor_state = str(
            getattr(self, "turn_supervisor_state", "")
        ).strip().upper()
        supervisor_route_kind = str(
            getattr(self, "turn_supervisor_route_kind", "")
        ).strip().lower()
        supervisor_seen = float(
            getattr(self, "turn_supervisor_status_monotonic", 0.0) or 0.0
        )
        supervisor_fresh = (
            supervisor_seen > 0.0
            and now - supervisor_seen
            <= float(getattr(self, "turn_supervisor_status_timeout", 3.0) or 3.0)
        )
        supervisor_turn_active = (
            supervisor_state == "TURNING"
            and supervisor_fresh
            and self.active_route_kind in (
                "frontier_turn_connector",
                "frontier_endpoint",
                "portal_transition",
                "local_egress",
            )
            and (
                not supervisor_route_kind
                or supervisor_route_kind == self.active_route_kind
            )
        )
        turn_phase_active = (
            (
                self.active_route_kind == "frontier_turn_connector"
                and not self.turn_connector_released
            )
            or supervisor_turn_active
        )
        recovery_pending = self.recovery_pending_route_id == self.active_route_id
        if turn_phase_active:
            # An in-place rotation is progress for the translational clock,
            # but only for as long as a turn can plausibly last.  Without this
            # bound a single latched TURNING phase pins active_progress_time
            # forever, which disables every stall watchdog at once.
            since = getattr(self, "turn_phase_since", None)
            if since is None:
                self.turn_phase_since = float(now)
                since = self.turn_phase_since
            limit = float(getattr(self, "turn_phase_max_seconds", 20.0) or 0.0)
            if limit > 0.0 and float(now) - float(since) > limit:
                turn_phase_active = False
                key = int(getattr(self, "active_route_id", 0) or 0)
                if int(getattr(self, "expired_turn_phase_route_id", 0) or 0) != key:
                    self.expired_turn_phase_route_id = key
                    publish = getattr(self, "publish_status", None)
                    if callable(publish):
                        publish(
                            "turn_phase_progress_expired",
                            route_id=key,
                            turn_phase_age_seconds=round(float(now) - float(since), 3),
                            limit_seconds=limit,
                            supervisor_state=str(
                                getattr(self, "turn_supervisor_state", "")
                            ),
                        )
            else:
                self.active_progress_time = now
        else:
            self.turn_phase_since = None

        (
            post_turn_goal_matches,
            post_turn_elapsed,
            post_turn_translation,
        ) = self.update_post_turn_watchdog(x, y, now)
        post_turn_stalled = (
            not turn_phase_active
            and self.post_turn_stall_timeout is not None
            and post_turn_goal_matches
            and not self.active_turn_completed_launched
            and post_turn_elapsed >= self.post_turn_stall_timeout
            and now - self.active_progress_time >= self.post_turn_stall_timeout
        )
        region_stagnant = self.active_region_is_stagnant(
            x,
            y,
            robot_map,
            now,
            portal_transition,
            turn_phase_active,
            recovery_pending,
            active_component,
            active_information,
        )
        # Observation stagnation describes a place-level fact, not an action
        # failure. In particular, it must not terminate or replace the
        # still-active move_base transaction. The terminal lifecycle closes
        # the place only after the bridge proves that this exact route ended.
        stalled = (
            recovery_pending
            or post_turn_stalled
            or (
                not turn_phase_active
                and now - self.active_progress_time > self.stall_timeout
            )
        )
        goal_xy = None
        active_frontier = getattr(self, "active_frontier", None)
        if active_frontier is not None and len(active_frontier) >= 4:
            goal_xy = (float(active_frontier[2]), float(active_frontier[3]))
        stagnation_since = None
        if stalled and goal_xy is not None:
            tracked = getattr(self, "goal_stagnation_xy", None)
            same = bool(
                tracked is not None
                and math.hypot(goal_xy[0] - tracked[0], goal_xy[1] - tracked[1])
                <= 0.30
            )
            if not same:
                self.goal_stagnation_xy = goal_xy
                self.goal_stagnation_count = 1
                self.goal_stagnation_since = float(now)
            else:
                self.goal_stagnation_count = int(
                    getattr(self, "goal_stagnation_count", 0) or 0) + 1
            stagnation_since = float(
                getattr(self, "goal_stagnation_since", now) or now)
        else:
            self.goal_stagnation_xy = None
            self.goal_stagnation_count = 0
            self.goal_stagnation_since = 0.0
        waypoint_distance = self.active_waypoint_distance(robot_map)
        auth_approach_pending = bool(
            getattr(self, "active_portal_authentication_phase", "") == "entering"
            and getattr(self, "active_portal_authentication_entry_approach", False)
        )
        waypoint_release_radius = (
            max(float(self.waypoint_release_radius), 0.50)
            if auth_approach_pending
            else float(self.waypoint_release_radius)
        )
        waypoint_reached = (
            self.active_last_waypoint_map is None
            or waypoint_distance <= waypoint_release_radius
        )
        # A portal route represents one graph edge, not an open-ended local
        # search. Its lease starts at the physical doorway, rather than at a
        # remote route-selection point. This separates corridor approach from
        # the actual crossing without adding a controller-specific timeout.
        portal_edge_started_at = getattr(
            self, "active_portal_gate_approached_at", None,
        )
        portal_edge_expired = (
            portal_transition
            and portal_edge_started_at is not None
            and now - portal_edge_started_at > self.active_timeout
        )
        active_timeout = (
            now - self.active_since > self.active_timeout and stalled
        )
        lease_decision = route_lease_failure_decision(
            controller_owns_failure=bool(
                getattr(self, "controller_owned_route_failure", False)
            ),
            recovery_pending=recovery_pending,
            route_stalled=stalled,
            post_turn_stalled=post_turn_stalled,
            portal_edge_expired=portal_edge_expired,
            active_timeout=active_timeout,
            # Measure from the first stagnation of this goal: every
            # re-dispatch resets active_progress_time (5-10 s apart in
            # 20261103_014424..053045), so the 12 s bound never accumulated.
            stalled_for=float(
                now - float(
                    stagnation_since
                    if stagnation_since is not None
                    else getattr(self, "active_progress_time", now))
            ),
            controller_terminal_limit=float(
                getattr(self, "controller_terminal_wait_seconds", 12.0) or 0.0
            ),
        )
        expired = lease_decision.release
        if (
            stalled
            and goal_xy is not None
            # A doorway authentication owns its own physical budget; its route
            # is intentionally slow while the base enters and turns.
            and not str(getattr(self, "active_portal_authentication_phase", "") or "")
            and int(getattr(self, "goal_stagnation_count", 0) or 0)
            >= int(getattr(self, "goal_stagnation_limit", 3) or 0)
            and float(now) - float(
                getattr(self, "goal_stagnation_since", now) or now)
            >= float(getattr(self, "goal_stagnation_seconds", 25.0) or 0.0)
            and int(getattr(self, "goal_stagnation_abandoned_for", 0) or 0)
            != int(getattr(self, "active_route_id", 0) or 0)
        ):
            self.goal_stagnation_abandoned_for = int(
                getattr(self, "active_route_id", 0) or 0)
            self.abandon_stagnant_goal(now, goal_xy)
            expired = True
        if (
            bool(getattr(self, "controller_owned_route_failure", False))
            and stalled
            and not recovery_pending
            and int(getattr(self, "last_route_stagnation_reported_route_id", 0))
            != int(self.active_route_id)
        ):
            self.last_route_stagnation_reported_route_id = int(self.active_route_id)
            publish = getattr(self, "publish_status", None)
            if callable(publish):
                publish(
                    "route_stagnant_observed",
                    goal_xy=(
                        None if goal_xy is None
                        else [round(float(v), 3) for v in goal_xy]
                    ),
                    stagnation_count=int(
                        getattr(self, "goal_stagnation_count", 0) or 0),
                    route_id=int(self.active_route_id),
                    route_kind=str(self.active_route_kind),
                    authority=lease_decision.authority,
                    reason=lease_decision.reason,
                    active_elapsed=round(float(now - self.active_since), 3),
                    last_progress_signal=str(
                        getattr(self, "active_last_progress_signal", "none")
                    ),
                    # These fields are the diagnosis for "dispatched and then
                    # nothing": which gate was holding the route still.
                    progress_age_seconds=round(
                        float(now) - float(getattr(self, "active_progress_time", now)),
                        3,
                    ),
                    turn_phase_active=bool(turn_phase_active),
                    turn_supervisor_state=str(
                        getattr(self, "turn_supervisor_state", "")
                    ),
                    turn_phase_age_seconds=(
                        None
                        if getattr(self, "turn_phase_since", None) is None
                        else round(
                            float(now) - float(self.turn_phase_since), 3
                        )
                    ),
                    dispatch_announced=bool(
                        getattr(self, "active_work_item_dispatch_announced", False)
                    ),
                    lease_release_pending=bool(
                        getattr(self, "pending_controller_lease_release", None)
                    ),
                )
            rospy.logwarn(
                "Global frontier observed route stagnation without releasing "
                "controller-owned lease route_id=%d",
                self.active_route_id,
            )
        return ActiveRouteWatchdogState(
            turn_phase_active=turn_phase_active,
            recovery_pending=recovery_pending,
            post_turn_goal_matches=post_turn_goal_matches,
            post_turn_elapsed=post_turn_elapsed,
            post_turn_translation=post_turn_translation,
            post_turn_stalled=post_turn_stalled,
            region_stagnant=region_stagnant,
            stalled=stalled,
            waypoint_distance=waypoint_distance,
            waypoint_reached=waypoint_reached,
            portal_edge_expired=portal_edge_expired,
            expired=expired,
        )

    def abandon_stagnant_goal(self, now, goal_xy):
        """Give up on a goal that cannot produce motion, with evidence.

        Runs 20261103_014424/_043447/_053045 dispatched the same goal 8-40
        times; the controller owned the terminal, every re-dispatch reset the
        progress clock, and retry_viewpoint re-proposed the same unreachable
        viewpoint.  After goal_stagnation_limit consecutive stagnations the work
        item is resolved as unreachable and the region is marked failed, so the
        selector has to pick a different obligation.
        """
        item_id = getattr(self, "active_work_item_id", None)
        attempt_id = getattr(self, "active_work_item_attempt_id", None)
        ledger = getattr(self, "place_work_items", None)
        if item_id is not None and ledger is not None:
            abandon = getattr(ledger, "abandon_unreachable", None)
            if callable(abandon):
                abandon(int(item_id), now, "viewpoint_unreachable",
                        attempt_id=attempt_id)
        component = getattr(self, "active_frontier_component", None)
        record = getattr(self, "record_failed_frontier_region", None)
        if callable(record) and goal_xy is not None:
            try:
                record(float(goal_xy[0]), float(goal_xy[1]), now,
                       "goal_stagnation_budget", component)
            except TypeError:
                record(float(goal_xy[0]), float(goal_xy[1]), now,
                       "goal_stagnation_budget")
        publish = getattr(self, "publish_status", None)
        if callable(publish):
            publish(
                "route_unreachable_abandoned",
                goal_xy=(None if goal_xy is None
                         else [round(float(v), 3) for v in goal_xy]),
                stagnation_count=int(
                    getattr(self, "goal_stagnation_count", 0) or 0),
                work_item_id=None if item_id is None else int(item_id),
                reason="goal_stagnation_budget",
            )
        rospy.logwarn(
            "Abandoning goal %s after %d stagnations; work_item=%s",
            None if goal_xy is None
            else [round(float(v), 2) for v in goal_xy],
            int(getattr(self, "goal_stagnation_count", 0) or 0),
            item_id,
        )
        release = getattr(self, "release_active_frontier", None)
        if callable(release):
            release(discard_prefetch=True)
        return True

    def update_post_turn_watchdog(self, x, y, now):
        """Track whether an execution handoff actually launched translation."""
        goal_matches = (
            self.active_turn_completed_route_id == self.active_route_id
            and self.active_turn_completed_goal is not None
            and math.hypot(
                self.active_turn_completed_goal[0] - x,
                self.active_turn_completed_goal[1] - y,
            ) <= 0.10
        )
        elapsed = (
            0.0 if not goal_matches
            else max(0.0, now - self.active_turn_completed_wall)
        )
        translation = None
        if (
            goal_matches
            and self.active_turn_completed_odom_xy is not None
            and self.pose_odom is not None
        ):
            translation = math.hypot(
                float(self.pose_odom.x) - self.active_turn_completed_odom_xy[0],
                float(self.pose_odom.y) - self.active_turn_completed_odom_xy[1],
            )
            self.active_turn_completed_translation = max(
                self.active_turn_completed_translation,
                translation,
            )
            if (
                not self.active_turn_completed_launched
                and self.active_turn_completed_translation >= self.progress_epsilon
            ):
                # A map->odom correction can increase map-space goal distance
                # while the base follows a doorway detour.  Odom launch is the
                # invariant proof that this route is alive.
                self.active_turn_completed_launched = True
                self.active_progress_time = now
                self.active_last_progress_signal = "post_turn_odom_launch"
                self.publish_status(
                    "post_turn_progress_watchdog_released",
                    route_id=int(self.active_route_id),
                    goal=[round(float(x), 3), round(float(y), 3)],
                    odom_translation=round(
                        float(self.active_turn_completed_translation), 3
                    ),
                )
        return goal_matches, elapsed, translation

    def active_region_is_stagnant(
        self, x, y, robot_map, now, portal_transition, turn_phase_active,
        recovery_pending, active_component, active_information,
    ):
        """Report no-gain dwell without changing the route or place state.

        A planning tick can observe a finished room while TEB still owns the
        endpoint action. This method is deliberately read-only: turning a
        place dormant here would make the next timer choose a successor before
        the active action has emitted its matching terminal. Terminal-time
        observation handling owns the eventual place closure.
        """
        if (
            portal_transition
            or turn_phase_active
            or recovery_pending
            # Direct target evidence explicitly owns its structural place.
            # Ordinary coverage completion must not override that higher
            # level mission claim merely because detector frames are sparse.
            or self.target_region_claim_active
            or self.active_observation_session_started_at is None
        ):
            return False
        region = self.region_memory.stagnant(
            x,
            y,
            robot_map[0],
            robot_map[1],
            now,
            component=active_component,
            region_id=self.active_frontier_region_id,
        )
        if region is None:
            return False
        rospy.loginfo_throttle(
            5.0,
            "Global frontier observed no information gain for %.1fs in "
            "region id=%d; retaining active route_id=%d until its terminal",
            now - region["last_gain"],
            region["id"],
            self.active_route_id,
        )
        return True

    def active_waypoint_distance(self, robot_map):
        """Return the distance to the bridge's latest short waypoint."""
        if self.active_last_waypoint_map is None:
            return float("inf")
        return math.hypot(
            self.active_last_waypoint_map[0] - robot_map[0],
            self.active_last_waypoint_map[1] - robot_map[1],
        )

    def active_frontier_observation_ready(self, endpoint_distance):
        """Require physical arrival before starting frontier observation.

        MoveBase may terminate an endpoint action anywhere inside the local
        planner's xy goal tolerance. Keep the frontier waypoint threshold at
        least that large, or a successful controller arrival just outside the
        smaller handoff radius can leave the durable WorkItem pending forever.
        """
        arrival_radius = max(
            float(self.waypoint_release_radius),
            float(getattr(self, "teb_xy_goal_tolerance", 0.0)),
        )
        return (
            self.active_last_waypoint_map is not None
            and self.active_route_kind != "frontier_turn_connector"
            and float(endpoint_distance) <= arrival_radius
        )
