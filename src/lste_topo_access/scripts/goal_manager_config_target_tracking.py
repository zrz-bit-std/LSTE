"""Visual target-tracking configuration for the Goal Manager."""

import math

import rospy


class GoalManagerTargetTrackingConfigMixin:
    """Load detector evidence and short-horizon target-follow parameters."""

    def _load_target_tracking_parameters(self, gp):
        """Configure visual observations before route and completion policy."""
        self.pass_period = float(gp("~pass_period", 5.0))
        self.sus_c_period = float(gp("~sus_c_period", 3.0))
        self.locked_period = float(gp("~locked_period", 2.0))
        self.sus_a_period = float(gp("~sus_a_period", 2.0))
        self.sus_b_period = float(gp("~sus_b_period", 2.0))
        self.forward_dist = float(gp("~forward_dist", 10.0))
        self.goal_dist_det = float(gp("~goal_dist_det", 5.0))
        self.follow_target_step_distance = max(
            0.2, float(gp("~follow_target_step_distance", 1.5))
        )
        target_scan_clip = gp("~follow_target_use_scan_clip", False)
        self.follow_target_use_scan_clip = str(target_scan_clip).strip().lower() in (
            "1", "true", "yes", "on",
        )
        self.follow_context_step_distance = max(
            0.2, float(gp("~follow_context_step_distance", 1.5))
        )
        self.frontier_topic = gp("~frontier_topic", "/lste/gp_frontier_dir")
        self.frontiers_topic = gp("~frontiers_topic", "/lste/gp_frontiers")
        self.depth_topic = gp("~depth_topic", "/kinect/hd/image_depth_rect")
        self.image_topic = gp("~image_topic", "/kinect/hd/image_color_rect")
        self.use_depth = bool(gp("~use_depth", False))
        self.follow_target_lost_timeout = float(
            gp("~follow_target_lost_timeout", 10.0)
        )
        self.target_reacquire_duration = max(
            0.0, float(gp("~target_reacquire_duration", 6.0))
        )
        self.target_reacquire_distance = max(
            0.2, float(gp("~target_reacquire_distance", 1.0))
        )
        self.target_reacquire_max_attempts = max(
            0, int(gp("~target_reacquire_max_attempts", 1))
        )
        self.follow_ctx_lost_timeout = float(gp("~follow_ctx_lost_timeout", 5.0))
        self.context_state_hold_time = max(
            0.0, float(gp("~context_state_hold_time", 4.0))
        )
        self.context_follow_cooldown = max(
            0.0, float(gp("~context_follow_cooldown", 30.0))
        )
        # A context pair that produced no target evidence must stay out of the
        # next plans long enough for the robot to make real progress.  This is
        # separate from the short retry cooldown above.
        self.context_follow_exhausted_cooldown = max(
            self.context_follow_cooldown,
            float(gp("~context_follow_exhausted_cooldown", 120.0)),
        )
        # A context hint is a geometric direction, not a proven route.  When
        # Navfn cannot connect it, keep the pair out of the next plans long
        # enough to try another hint or a map frontier, but much shorter than
        # the "inspected and found nothing" cooldown so a corrected map can
        # offer it again.
        self.context_route_blocked_cooldown = max(
            self.context_follow_cooldown,
            float(gp("~context_route_blocked_cooldown", 30.0)),
        )
        memory = getattr(self, "ctx_pair_exhaustion", None)
        if memory is not None:
            memory.cooldown = self.context_follow_exhausted_cooldown
        self.context_observe_trigger_seconds = max(
            0.5, float(gp("~context_observe_trigger_seconds", 4.0))
        )
        self.context_observe_duration = max(
            1.0, float(gp("~context_observe_duration", 10.0))
        )
        self.context_observe_hold_seconds = max(
            0.2, float(gp("~context_observe_hold_seconds", 2.0))
        )
        sweep = gp("~context_observe_sweep_deg", "0,30,-30,60,-60")
        if isinstance(sweep, str):
            sweep = [value for value in sweep.split(",") if value.strip()]
        try:
            sweep = tuple(float(value) for value in sweep)
        except (TypeError, ValueError):
            sweep = (0.0, 30.0, -30.0, 60.0, -60.0)
        policy = getattr(self, "ctx_observation", None)
        if policy is not None:
            policy.trigger_seconds = self.context_observe_trigger_seconds
            policy.observe_seconds = self.context_observe_duration
            policy.hold_seconds = self.context_observe_hold_seconds
            if sweep:
                policy.sweep_deg = sweep
        self.follow_min_update_period = float(gp("~follow_min_update_period", 0.3))
        self.follow_heading_alpha = float(gp("~follow_heading_alpha", 0.5))
        self.target_bbox_alpha = min(
            1.0, max(0.05, float(gp("~target_bbox_alpha", 0.35)))
        )
        self.target_heading_alpha = min(
            1.0, max(0.05, float(gp("~target_heading_alpha", 0.30)))
        )
        self.target_max_heading_step = math.radians(max(
            1.0, float(gp("~target_max_heading_step_deg", 25.0))
        ))
        self.target_goal_reached_radius = max(
            0.20, float(gp("~target_goal_reached_radius", 0.75))
        )
        self.target_goal_heading_update_threshold = math.radians(max(
            1.0, float(gp("~target_goal_heading_update_threshold_deg", 35.0))
        ))
        target_approach_strategy = str(
            gp("~target_approach_strategy", "reachable_viewpoint_ladder")
        ).strip().lower()
        if target_approach_strategy not in (
            "legacy_ray", "reachable_viewpoint_ladder",
        ):
            rospy.logwarn(
                "Unsupported target approach strategy %r; using reachable_viewpoint_ladder.",
                target_approach_strategy,
            )
            target_approach_strategy = "reachable_viewpoint_ladder"
        self.target_approach_strategy = target_approach_strategy
