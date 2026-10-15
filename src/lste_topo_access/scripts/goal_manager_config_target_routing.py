"""Map-route validation and handoff configuration for visual targets."""

import math


class GoalManagerTargetRoutingConfigMixin:
    """Load the policy that admits a visual observation as a navigation goal."""

    def _load_target_routing_parameters(self, gp):
        """Configure Navfn validation and continuity-aware target handoff."""
        target_route_validation = gp("~target_route_validation", True)
        self.target_route_validation = str(target_route_validation).strip().lower() in (
            "1", "true", "yes", "on",
        )
        self.target_route_validation_service = gp(
            "~target_route_validation_service", "/move_base/NavfnROS/make_plan"
        )
        self.target_route_validation_timeout = max(
            0.01, float(gp("~target_route_validation_timeout", 0.05))
        )
        self.target_route_validation_tolerance = max(
            0.0, float(gp("~target_route_validation_tolerance", 0.20))
        )
        self.target_route_validation_period = max(
            1.0, float(gp("~target_route_validation_period", 1.0))
        )
        route_continuity = gp("~target_route_continuity_enabled", True)
        self.target_route_continuity_enabled = str(route_continuity).strip().lower() in (
            "1", "true", "yes", "on",
        )
        self.target_route_continuity_entry_lookahead = max(
            0.05, float(gp("~target_route_continuity_entry_lookahead", 0.35))
        )
        self.target_route_continuity_smooth_heading = math.radians(max(
            1.0, float(gp("~target_route_continuity_smooth_heading_deg", 45.0))
        ))
        self.target_route_continuity_takeover_heading = math.radians(max(
            float(gp("~target_route_continuity_smooth_heading_deg", 45.0)),
            float(gp("~target_route_continuity_takeover_heading_deg", 70.0)),
        ))
        self.target_route_continuity_distance_tradeoff = math.radians(max(
            0.0, float(gp("~target_route_continuity_distance_tradeoff_deg", 20.0))
        ))
        defer_takeover = gp("~target_route_continuity_defer_sharp_takeover", True)
        self.target_route_continuity_defer_sharp_takeover = str(
            defer_takeover
        ).strip().lower() in ("1", "true", "yes", "on")
        # Legacy short-horizon prefetch bound. It is retained for compatibility
        # but never ends a confirmed TargetApproachTransaction.
        self.target_cache_max_advances = max(
            0, int(gp("~target_cache_max_advances", 2))
        )
        continuous_handoff = gp("~target_continuous_handoff_enabled", True)
        self.target_continuous_handoff_enabled = str(continuous_handoff).strip().lower() in (
            "1", "true", "yes", "on",
        )
        self.target_continuous_handoff_distance = max(
            self.target_goal_reached_radius + 0.05,
            float(gp("~target_continuous_handoff_distance", 0.95)),
        )
        self.target_continuous_handoff_min_new_frames = max(
            1, int(gp("~target_continuous_handoff_min_new_frames", 2))
        )
        self.follow_locked_done_time = float(gp("~follow_locked_done_time", 10.0))
        self.follow_goal_publish_period = max(
            0.05, float(gp("~follow_goal_publish_period", 0.3))
        )
