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
