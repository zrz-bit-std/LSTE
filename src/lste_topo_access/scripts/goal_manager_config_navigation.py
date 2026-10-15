"""Global-goal and frontier integration configuration for Goal Manager."""


class GoalManagerNavigationConfigMixin:
    """Load exploration source, route handoff, and mission API parameters."""

    def _load_goal_source_parameters(self, gp):
        """Configure the normal brain path or an isolated fixed-goal test."""
        self.global_goal_source = str(
            gp("~global_goal_source", "brain")
        ).strip().lower()
        if self.global_goal_source not in ("brain", "fixed"):
            raise ValueError(
                "~global_goal_source must be 'brain' or 'fixed' "
                "(got %r)" % self.global_goal_source
            )
        self.fixed_goal = (
            float(gp("~fixed_goal_x", 0.0)),
            float(gp("~fixed_goal_y", 0.0)),
            float(gp("~fixed_goal_yaw", 0.0)),
        )
        self.fixed_goal_publish_period = max(
            0.05, float(gp("~fixed_goal_publish_period", 1.0))
        )

    def _load_frontier_integration_parameters(self, gp):
        """Configure online-SLAM frontier route ownership and handoff limits."""
        global_frontier_enabled = gp("~global_frontier_enabled", False)
        self.global_frontier_enabled = str(global_frontier_enabled).strip().lower() in (
            "1", "true", "yes", "on",
        )
        startup_forward_enabled = gp("~startup_forward_enabled", False)
        self.startup_forward_enabled = str(startup_forward_enabled).strip().lower() in (
            "1", "true", "yes", "on",
        )
        self.global_frontier_topic = gp(
            "~global_frontier_topic", "/lste/global_frontier_goal"
        )
        self.global_frontier_command_topic = gp(
            "~global_frontier_command_topic",
            "/lste/global_frontier/route_command",
        )
        command_enabled = gp("~global_frontier_command_enabled", True)
        self.global_frontier_command_enabled = str(command_enabled).strip().lower() in (
            "1", "true", "yes", "on",
        )
        self.global_frontier_status_topic = gp(
            "~global_frontier_status_topic", "/lste/global_frontier/status"
        )
        self.global_frontier_replan_request_topic = gp(
            "~global_frontier_replan_request_topic",
            "/lste/global_frontier/replan_request",
        )
        self.frontier_replan_request_id = 0
        self.frontier_replan_pending_id = 0
        self.frontier_replan_ready_id = 0
        self.teb_goal_terminal_topic = gp(
            "~teb_goal_terminal_topic", "/lste/teb_goal_terminal"
        )
        self.teb_goal_failure_topic = gp(
            "~teb_goal_failure_topic", "/lste/teb_goal_failure"
        )
        self.teb_goal_bridge_status_topic = gp(
            "~teb_goal_bridge_status_topic", "/lste/teb_goal_bridge/status"
        )
        self.global_frontier_max_age = max(
            0.0, float(gp("~global_frontier_max_age", 3.0))
        )
        self.global_frontier_period = max(
            0.1, float(gp("~global_frontier_period", 1.0))
        )
        self.global_frontier_update_radius = max(
            0.2, float(gp("~global_frontier_update_radius", 0.75))
        )
        self.global_frontier_early_handoff_radius = max(
            self.global_frontier_update_radius,
            float(gp("~global_frontier_early_handoff_radius", 0.95)),
        )
        self.global_frontier_jump_distance = max(
            self.global_frontier_update_radius,
            float(gp("~global_frontier_jump_distance", 2.0)),
        )
        self.global_frontier_jump_release_radius = max(
            self.global_frontier_update_radius,
            float(gp("~global_frontier_jump_release_radius", 0.90)),
        )
        self.global_frontier_terminal_hold_timeout = max(
            0.0, float(gp("~global_frontier_terminal_hold_timeout", 20.0))
        )
        self.global_frontier_min_hold_time = max(
            0.0, float(gp("~global_frontier_min_hold_time", 2.5))
        )
        preempt_context = gp("~global_frontier_preempt_context", False)
        self.global_frontier_preempt_context = str(preempt_context).strip().lower() in (
            "1", "true", "yes", "on",
        )

    def _load_mission_interface_parameters(self, gp):
        """Configure controller selection and Goal Manager topic contracts."""
        self.controller_mode = str(gp("~controller_mode", "teb")).strip().lower()
        if self.controller_mode not in ("sappo", "teleop", "teb"):
            self.controller_mode = "teb"
        self.controller_mode_topic = gp(
            "~controller_mode_topic", "/lste/controller_mode"
        )
        self.goal_intent_topic = gp("~goal_intent_topic", "/lste/goal_intent")
        self.goal_command_topic = gp("~goal_command_topic", "/lste/mission_goal")
        self.navigation_hold_topic = gp(
            "~navigation_hold_topic", "/lste/navigation_hold"
        )
        allow_fixed_click_override = gp("~fixed_goal_allow_click_override", False)
        self.fixed_goal_allow_click_override = str(
            allow_fixed_click_override
        ).strip().lower() in ("1", "true", "yes", "on")
        self.fixed_goal_click_topic = gp(
            "~fixed_goal_click_topic", "/move_base/current_goal"
        )
