"""Completion-evidence configuration for visual target pursuit."""


class GoalManagerTargetCompletionConfigMixin:
    """Load evidence thresholds that can finish or hold a target action."""

    def _load_target_completion_parameters(self, gp):
        """Configure close-range completion and multi-frame confirmation."""
        policy = str(gp("~target_done_policy", "visual_target")).strip().lower()
        if policy not in ("visual_target", "target_room_entry", "target_context"):
            policy = "visual_target"
        self.target_done_policy = policy
        # ``target_room_entry`` is a benchmark/diagnostic acceptance policy.
        # The coordinates are supplied by the benchmark overlay; production
        # runs keep the default visual-target policy and never use this mask.
        self.target_room_entry_min_x = float(
            gp("~target_room_entry_min_x", 0.0)
        )
        self.target_room_entry_min_y = float(
            gp("~target_room_entry_min_y", 0.0)
        )
        self.target_room_entry_max_x = float(
            gp("~target_room_entry_max_x", 0.0)
        )
        self.target_room_entry_max_y = float(
            gp("~target_room_entry_max_y", 0.0)
        )
        self.target_room_entry_margin = max(
            0.0, float(gp("~target_room_entry_margin", 0.30))
        )
        self.target_room_entry_min_hold_time = max(
            0.0, float(gp("~target_room_entry_min_hold_time", 0.0))
        )
        require_crossing = gp("~target_room_entry_require_crossing", True)
        self.target_room_entry_require_crossing = str(
            require_crossing
        ).strip().lower() in ("1", "true", "yes", "on")
        self.target_done_min_box_width = float(
            gp("~target_done_min_box_width", 0.06)
        )
        self.target_done_min_box_height = float(
            gp("~target_done_min_box_height", 0.06)
        )
        self.target_done_min_score = float(gp("~target_done_min_score", 0.40))
        self.target_done_min_hold_time = max(
            0.0, float(gp("~target_done_min_hold_time", 2.0))
        )
        self.target_done_min_fresh_hits = max(
            1, int(gp("~target_done_min_fresh_hits", 3))
        )
        require_close = gp("~target_done_require_close_evidence", True)
        self.target_done_require_close_evidence = str(
            require_close
        ).strip().lower() in ("1", "true", "yes", "on")
        require_confirmed_track = gp(
            "~target_done_require_confirmed_track", True
        )
        self.target_done_require_confirmed_track = str(
            require_confirmed_track
        ).strip().lower() in ("1", "true", "yes", "on")
        self.target_done_max_detection_age = max(
            0.0, float(gp("~target_done_max_detection_age", 0.75))
        )
        require_approach = gp("~target_done_require_approach_terminal", True)
        self.target_done_require_approach_terminal = str(
            require_approach
        ).strip().lower() in ("1", "true", "yes", "on")
        self.target_follow_min_score = max(
            0.0, float(gp("~target_follow_min_score", 0.20))
        )
        self.target_follow_min_box_size = max(
            0.0, float(gp("~target_follow_min_box_size", 0.01))
        )
        self.target_follow_candidate_timeout = max(
            0.5, float(gp("~target_follow_candidate_timeout", 8.0))
        )
        self.target_follow_confirm_hits = max(
            1, int(gp("~target_follow_confirm_hits", 2))
        )
        self.target_follow_confirm_window = max(
            self.target_follow_candidate_timeout,
            float(gp("~target_follow_confirm_window", 20.0)),
        )
        self.target_follow_weak_confirm_hits = max(
            self.target_follow_confirm_hits,
            int(gp("~target_follow_weak_confirm_hits", 5)),
        )
        self.target_follow_weak_min_average_score = max(
            self.target_follow_min_score,
            float(gp("~target_follow_weak_min_average_score", 0.24)),
        )
        self.target_follow_spatial_tolerance = max(
            0.02, float(gp("~target_follow_spatial_tolerance", 0.10))
        )
        self.target_observation_hold = max(
            0.0, float(gp("~target_observation_hold", 4.5))
        )
        require_locked = gp("~target_done_require_locked", False)
        self.target_done_require_locked = str(require_locked).strip().lower() in (
            "1", "true", "yes", "on",
        )
