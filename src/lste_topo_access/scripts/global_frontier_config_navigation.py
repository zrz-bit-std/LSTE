#!/usr/bin/env python3

"""Navigation and execution configuration for the frontier explorer."""

import rospy


class GlobalFrontierNavigationConfigurationMixin:
    """Load constraints and timings for Navfn/TEB route execution."""

    def _load_navigation_parameters(self, gp):
        """Resolve route validation, handoff, and progress-watchdog settings."""
        self.costmap_max_age = max(0.5, float(gp("~costmap_max_age", 3.0)))
        # ``clearance`` is the route clearance, not merely a preference.  It
        # must cover the robot footprint and TEB's minimum obstacle distance;
        # otherwise this node can find a connected BFS route that Navfn/TEB
        # correctly rejects.  The production launch passes 0.52 m
        # (0.30 m footprint radius + 0.22 m TEB clearance).
        self.clearance = max(0.05, float(gp("~clearance", 0.52)))
        # ``fallback_clearance`` identifies known-free observation topology at
        # an unknown boundary. It can connect a recovery route through a
        # narrow passage, but never authorizes the endpoint itself: every
        # published endpoint remains in ``strict_free`` below.
        self.frontier_clearance = max(
            0.15,
            min(
                self.clearance,
                float(gp("~fallback_clearance", max(0.20, self.clearance - 0.10))),
            ),
        )
        # The goal is an approach point in the safe route mask.  It may sit
        # short of the actual unknown-boundary cell; the lidar then reveals the
        # next part of the map without driving the footprint into a doorway.
        self.frontier_approach_distance = max(
            0.20, float(gp("~frontier_approach_distance", 1.0))
        )
        self.min_path_distance = max(0.2, float(gp("~min_path_distance", 1.2)))
        observation_recovery = gp("~navfn_observation_recovery_enabled", True)
        self.navfn_observation_recovery_enabled = str(
            observation_recovery
        ).strip().lower() in ("1", "true", "yes", "on")
        # Selection mode is attached to the next route transaction and status
        # record. It is diagnostic only; it never changes TEB's constraints.
        self.last_frontier_selection_mode = "strict_clearance"
        # A bounded Navfn validation pass may reject a batch while other
        # candidates remain. This is distinct from genuine map exhaustion.
        self.frontier_validation_budget_exhausted = False
        # A live service can still be initializing its costmap. This is also
        # distinct from exhaustion and must remain retryable on the next map
        # cycle.
        self.frontier_validation_pending = False
        # The frontier is a mission endpoint.  ``lookahead_distance`` and the
        # rolling segment distance remain compatibility values for the legacy
        # experiment, but the production endpoint contract lets Navfn/TEB own
        # the complete known-free path in one action.
        self.lookahead_distance = max(0.2, float(gp("~lookahead_distance", 2.0)))
        self.waypoint_release_radius = max(
            0.2, float(gp("~waypoint_release_radius", 0.40))
        )
        # Leave room for the vehicle's current command-goal tolerance when a
        # route segment is handed off in-place.  The resulting horizon is
        # derived from the existing route/action contract: with the production
        # 4 m TEB reinitialization distance and a 1.2 m release radius it is
        # about 2.8 m, rather than a separately tuned controller constant.
        self.route_segment_distance = max(
            0.8,
            self.lookahead_distance - self.waypoint_release_radius,
        )
        # Exploration owns a frontier mission, while Navfn/TEB own the live
        # path to that mission.  Publishing a short rolling waypoint here
        # creates an action boundary in the middle of a perfectly valid path
        # and forces the executor to brake, replace its band, and start again.
        # The production contract therefore publishes the selected frontier
        # endpoint.  The legacy rolling-horizon behavior remains available
        # only for explicitly isolated experiments.
        endpoint_only = gp("~mission_endpoint_only", True)
        self.mission_endpoint_only = str(endpoint_only).strip().lower() in (
            "1", "true", "yes", "on",
        )
        # The persistent executor keeps one MoveBase action alive while its
        # pluginized Navfn/TEB pair consumes new plans. In that architecture
        # the frontier node, rather than an action terminal callback, advances
        # a validated prefetched successor. It is opt-in until its complete
        # search behavior has been compared with endpoint-action execution.
        self.persistent_execution = str(
            gp("~persistent_execution", False)
        ).strip().lower() in ("1", "true", "yes", "on")
        # In the persistent architecture one move_base action is a mission
        # lease. TEB/move_base own the authoritative failure terminal; the
        # global layer may still report stagnation, but it cannot manufacture
        # a failure from elapsed time. Legacy endpoint execution keeps its
        # historical watchdog authority for comparison runs.
        self.route_failure_authority = (
            "controller_terminal"
            if self.persistent_execution
            else "global_watchdog"
        )
        self.controller_owned_route_failure = self.persistent_execution
        # Compute the next exploration branch before the current endpoint is
        # reached.  It remains an internal cache until the validated early
        # handoff window below; this prevents map refreshes from replacing a
        # healthy route before a safe continuation is known.
        self.prefetch_distance = max(
            self.waypoint_release_radius * 2.0,
            float(gp("~prefetch_distance", 1.8)),
        )
        # Promote a validated pending branch before the current endpoint is
        # reached.  The old endpoint remains a safe approach point; waiting
        # for move_base's SUCCEEDED callback first inserts a zero-velocity gap
        # between otherwise connected exploration routes.  Keep this window
        # below the prefetch radius so a branch is selected and validated for
        # at least one map cycle before it can become the active goal.
        self.early_handoff_distance = max(
            self.waypoint_release_radius,
            min(
                self.prefetch_distance,
                float(gp("~early_handoff_distance", 1.35)),
            ),
        )
        self.active_timeout = max(2.0, float(gp("~active_timeout", 18.0)))
        self.stall_timeout = max(2.0, float(gp("~stall_timeout", 8.0)))
        # Bounded retry for a doorway attempt whose manoeuvre failed to execute
        # (controller handoff, physical stall, unrecoverable return).  A verdict
        # about the doorway itself is never retried.  Zero restores the old
        # one-strike behaviour for A/B runs.
        self.portal_authentication_retry_limit = max(
            0, int(gp("~portal_authentication_retry_limit", 2))
        )
        # How long a freshly dispatched authentication route may wait for the
        # controller to take it over before the *route* is failed (retryable)
        # instead of the doorway.
        self.portal_authentication_handoff_budget = max(
            5.0, float(gp("~portal_authentication_handoff_budget", 30.0))
        )
        # Quiet window before the same doorway attempt is queued again.
        self.portal_authentication_retry_cooldown = max(
            0.0, float(gp("~portal_authentication_retry_cooldown", 5.0))
        )
        # Room semantics are attributed only while the camera still looks into
        # the doorway's room.  The U-turn direction follows the inside-frontier
        # split, so the sweep up to ``room_view_arc_deg`` on that side is real
        # room content; past it the camera crosses the door plane and starts
        # looking back down the corridor.
        self.room_view_core_deg = max(
            0.0, float(gp("~room_view_core_deg", 45.0))
        )
        self.room_view_arc_deg = max(
            self.room_view_core_deg, float(gp("~room_view_arc_deg", 90.0))
        )
        # During ``entering`` only sample once the robot has reached the gate
        # plane (signed depth along the doorway normal).
        self.room_view_min_depth = float(
            gp("~room_view_min_depth", -0.3)
        )
        # Dwell at the inside point before the U-turn: the camera faces the
        # room, so a short stationary window gives the detector stable frames.
        # Kept well below the 8-12 s authentication progress budget.
        self.room_view_settle_seconds = max(
            0.0, min(5.0, float(gp("~room_view_settle_seconds", 1.0)))
        )
        # A target observation obligation must expire once its visual track
        # dies, otherwise it owns the Place forever (env04 east-lab parking:
        # one hit at sim 2660, then 340 s of silence with no live track).
        self.target_observation_timeout = max(
            0.0, float(gp("~target_observation_timeout", 30.0))
        )
        # A cooperative preempt only invalidates one in-flight pass.  A flag
        # whose terminal never reached the lifecycle must not silence the
        # planner (runs 20261112_220714 / 20261113_154448 / 183353).
        self.planning_preempt_max_stale_seconds = max(
            0.0, float(gp("~planning_preempt_max_stale_seconds", 5.0))
        )
        # A selected endpoint that reveals zero new map information and whose
        # region has produced no gain for this long is a dead end.  The normal
        # no-gain dwell retains the active route until its terminal, but the
        # stale-route handshake can reject the command forever so no terminal
        # ever arrives (env04 south-corridor 300 s stall).  After this timeout
        # the planner fails the region and forces a different obligation.
        self.zero_info_dead_end_timeout = max(
            5.0, float(gp("~zero_info_dead_end_timeout", 30.0))
        )
        # A route the GoalManager keeps discarding as stale can never move;
        # settle its doorway branch after this long instead of waiting for the
        # stationary watchdog (env04 south-corridor 300 s stall).
        self.portal_authentication_stale_route_grace = max(
            5.0, float(gp("~portal_authentication_stale_route_grace", 15.0))
        )
        # Consecutive stagnations of the same goal before it is abandoned.
        self.goal_stagnation_limit = max(
            1, int(gp("~goal_stagnation_limit", 3))
        )
        self.goal_stagnation_seconds = max(
            0.0, float(gp("~goal_stagnation_seconds", 25.0))
        )
        # Once a committed pre-route turn has completed, a route that never
        # physically launches is usually an unusable local plan. This is a
        # *launch* watchdog, not a shorter version of the general progress
        # watchdog: after measurable odom translation, the normal watchdog
        # remains responsible for deciding whether a moving route is healthy.
        # Set <= 0 to disable this specialized lifecycle rule for comparisons.
        raw_post_turn_stall_timeout = float(
            gp("~post_turn_stall_timeout", 6.0)
        )
        self.post_turn_stall_timeout = (
            None if raw_post_turn_stall_timeout <= 0.0 else max(
                1.0, min(self.stall_timeout, raw_post_turn_stall_timeout)
            )
        )
        # SLAM can temporarily disconnect an active frontier while integrating
        # a scan. Keep the last safe endpoint briefly instead of switching
        # branches on every map update.
        # A controller-owned failure may hold the disconnected route only
        # until its exact lease ACK arrives.  The failure resolution that
        # requests that ACK runs after the release, so an unbounded hold
        # deadlocks the planner.  Measured ACK latency is ~0.5 s; the lifecycle
        # sync timeout is 10 s.
        self.controller_failure_hold_seconds = max(
            0.0, float(gp("~controller_failure_hold_seconds", 20.0))
        )
        # The controller is the failure authority in persistent mode, but it
        # must deliver its terminal.  Fall back to the global watchdog after
        # this long without route progress (10 s lifecycle sync + 20 s release
        # hold fit comfortably inside it, the stall watchdog is 180 s).
        self.controller_terminal_wait_seconds = max(
            0.0, float(gp("~controller_terminal_wait_seconds", 12.0))
        )
        # A turn phase may pin the translational progress clock, but only for
        # a plausible in-place rotation.  The supervisor's own no-progress
        # release is capped at 12 s.
        self.turn_phase_max_seconds = max(
            0.0, float(gp("~turn_phase_max_seconds", 20.0))
        )
        # A TURNING status older than this is a dead latch, not an active turn.
        self.turn_supervisor_status_timeout = max(
            0.0, float(gp("~turn_supervisor_status_timeout", 3.0))
        )
        # A doorway can mature after its junction's observation period closed;
        # re-open that junction at most this many times per run.
        self.late_candidate_reopen_limit = max(
            0, int(gp("~late_candidate_reopen_limit", 2))
        )
        self.late_candidate_stable_grace = max(
            0.0, float(gp("~late_candidate_stable_grace", 20.0))
        )
        self.unreachable_grace = max(
            0.0, float(gp("~unreachable_grace", 20.0))
        )
        self.progress_epsilon = max(0.02, float(gp("~progress_epsilon", 0.12)))
        # The frontier node owns map-level route identity while move_base owns
        # the physical terminal.  Do not use a second, unrelated 0.8 m magic
        # number to retire that identity.  Online SLAM can shift map->odom by
        # several cells during arrival, so the exploration layer waits for the
        # matching move_base terminal throughout a small transform envelope
        # around TEB's real XY tolerance.  This is deliberately a *hold*, not
        # a new completion condition: only the bridge terminal advances the
        # frontier route.
        configured_goal_tolerance = gp("~teb_xy_goal_tolerance", None)
        if configured_goal_tolerance is None:
            # Keep the legacy runtime lookup for callers outside the launch
            # file. Production online_slam_frontier.launch always provides
            # the private value, avoiding a start-order dependency.
            configured_goal_tolerance = rospy.get_param(
                "/move_base/TebLocalPlannerROS/xy_goal_tolerance", 0.50
            )
        teb_xy_goal_tolerance = max(0.05, float(configured_goal_tolerance))
        # Portal targets are physical proofs. Preserve the controller success
        # radius explicitly so their endpoint builder can stay beyond it.
        self.teb_xy_goal_tolerance = teb_xy_goal_tolerance
        self.endpoint_terminal_wait_radius = max(
            0.8, teb_xy_goal_tolerance + 0.50
        )
        # Route/goal distance is measured in the online-SLAM map and can grow
        # during a legitimate detour. Use small odom-space coverage cells as
        # a second, map-correction-independent progress proof: entering a new
        # cell renews the watchdog, while a local loop eventually revisits its
        # cells and still times out.
        self.odom_novel_cell_size = max(
            0.20, float(gp("~odom_novel_cell_size", 0.40))
        )
        # A frontier can be several metres away while the collision-aware
        # controller is deliberately slow in a narrow corridor. Keep a failed
        # route out of candidate selection long enough to explore a different
        # branch instead of alternating between the same two boundaries.
        self.rejected_timeout = max(5.0, float(gp("~rejected_timeout", 180.0)))
        # Place observation budgets remain simulation-time policy. This is a
        # last-resort liveness bound for the pathological case where the Place
        # still lacks its first observation, no observation WorkItem can be
        # dispatched, and the robot has not moved in wall time either. Without
        # it a slow simulator can hold the graph until the mission watchdog
        # declares a failure that no policy timer was able to reach.
        self.place_wall_fallback_seconds = max(
            10.0, float(gp("~place_wall_fallback_seconds", 45.0))
        )
        self.place_wall_progress_radius = max(
            0.05, float(gp("~place_wall_progress_radius", 0.35))
        )
        # A predecessor release ACK is normally immediate. When the bridge
        # restamped the controller identity on adoption, or already cleaned
        # the route up, the exact ACK never arrives and successor planning is
        # gated forever. Bound that wait in wall time; the route's own health
        # budgets stay in simulation time.
        self.controller_release_ack_wall_seconds = max(
            10.0, float(gp("~controller_release_ack_wall_seconds", 30.0))
        )
