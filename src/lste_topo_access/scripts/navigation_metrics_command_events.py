"""TEB command events and discontinuity classification for navigation telemetry.

This module is observer-only: it records evidence and never publishes a motion command.
"""

import math
import time


class NavigationMetricsCommandEventsMixin:
    def on_teb_cmd(self, message):
        """Track turn-supervisor output before the final safety mux."""
        with self.lock:
            self.last_teb_cmd_wall = time.monotonic()
            previous = self.teb_command
            self.teb_command = message
            self.teb_cmd_messages += 1
            angular = float(message.angular.z)
            previous_angular = float(previous.angular.z)
            current_linear = float(message.linear.x)
            previous_linear = float(previous.linear.x)
            sign = 1 if angular > 0.05 else -1 if angular < -0.05 else 0
            previous_sign = (
                1 if previous_angular > 0.05
                else -1 if previous_angular < -0.05
                else 0
            )
            if sign and self.teb_last_nonzero_angular_sign and sign != self.teb_last_nonzero_angular_sign:
                self.teb_angular_sign_flips += 1
                if (
                    abs(angular) >= self.teb_strong_angular_threshold
                    and abs(previous_angular) >= self.teb_strong_angular_threshold
                ):
                    self.teb_strong_angular_sign_flips += 1
                self._write(
                    "WARN",
                    "teb_supervisor_angular_sign_flip",
                    angular=round(angular, 4),
                    previous_angular=round(previous_angular, 4),
                    count=self.teb_angular_sign_flips,
                    strong_count=self.teb_strong_angular_sign_flips,
                    goal=None if self.goal is None else [round(value, 3) for value in self.goal],
                    teb_status=self.teb_status,
                )
            if (
                sign
                and previous_sign
                and sign != previous_sign
                and current_linear > self.forward_speed_threshold
                and previous_linear > self.forward_speed_threshold
            ):
                self.teb_forward_steering_sign_flips += 1
                self._write(
                    "WARN",
                    "teb_supervisor_forward_steering_sign_flip",
                    angular=round(angular, 4),
                    previous_angular=round(previous_angular, 4),
                    count=self.teb_forward_steering_sign_flips,
                    current_linear=round(current_linear, 4),
                    previous_linear=round(previous_linear, 4),
                    goal=None if self.goal is None else [round(value, 3) for value in self.goal],
                )
            if sign:
                self.teb_last_nonzero_angular_sign = sign
            if previous_linear > 0.05 and current_linear < previous_linear - 0.08:
                is_stop_brake = current_linear <= 0.05
                if is_stop_brake:
                    self.teb_linear_brake_events += 1
                else:
                    self.teb_speed_modulation_events += 1
                self._write(
                    "WARN" if is_stop_brake else "INFO",
                    (
                        "teb_supervisor_linear_brake"
                        if is_stop_brake else "teb_supervisor_speed_modulation"
                    ),
                    count=(
                        self.teb_linear_brake_events
                        if is_stop_brake
                        else self.teb_speed_modulation_events
                    ),
                    previous_linear=round(previous_linear, 4),
                    current_linear=round(current_linear, 4),
                    angular=round(angular, 4),
                    scan_forward_min=None if not math.isfinite(self.scan_forward_minimum) else round(self.scan_forward_minimum, 4),
                    goal=None if self.goal is None else [round(value, 3) for value in self.goal],
                )

    def on_teb_planner_cmd(self, message):
        """Record TEB's raw command separately from the supervisor output."""
        with self.lock:
            self.last_teb_planner_cmd_wall = time.monotonic()
            previous = self.teb_planner_command
            self.teb_planner_command = message
            previous_linear = float(previous.linear.x)
            current_linear = float(message.linear.x)
            current_angular = float(message.angular.z)
            if (
                abs(current_linear) <= 0.001
                and abs(current_angular) <= 0.01
            ):
                invalidate_feedback = getattr(
                    self, "_invalidate_teb_feedback_locked", None
                )
                if callable(invalidate_feedback):
                    invalidate_feedback("planner_zero_command")
            if previous_linear > 0.05 and current_linear < previous_linear - 0.08:
                is_stop_brake = current_linear <= 0.05
                if is_stop_brake:
                    self.teb_planner_linear_brake_events += 1
                else:
                    self.teb_planner_speed_modulation_events += 1
                self._write(
                    "WARN" if is_stop_brake else "INFO",
                    (
                        "teb_planner_linear_brake"
                        if is_stop_brake else "teb_planner_speed_modulation"
                    ),
                    count=(
                        self.teb_planner_linear_brake_events
                        if is_stop_brake
                        else self.teb_planner_speed_modulation_events
                    ),
                    previous_linear=round(previous_linear, 4),
                    current_linear=round(current_linear, 4),
                    angular=round(float(message.angular.z), 4),
                    scan_forward_min=(
                        None if not math.isfinite(self.scan_forward_minimum)
                        else round(self.scan_forward_minimum, 4)
                    ),
                    goal=(
                        None if self.goal is None
                        else [round(value, 3) for value in self.goal]
                    ),
                )

    def _command_discontinuity_reason_locked(self, now):
        """Classify a command gap without influencing navigation control.

        This is an observability boundary: its purpose is to prove whether a
        visible stop belongs to a real topology/action transition, a deliberate
        in-place turn, an obstacle response, or an unexplained clear-space
        interruption before changing the execution architecture.
        """
        if self.task_done:
            return "task_complete", None
        if self.navigation_hold:
            return "navigation_hold", None
        mux_status = self.cmd_vel_mux_status
        if isinstance(mux_status, dict):
            block_reason = str(mux_status.get("block_reason", "none"))
            if block_reason == "navigation_hold":
                return "navigation_hold", None
            if block_reason == "task_complete":
                return "task_complete", None
            output = mux_status.get("output")
            input_command = mux_status.get("input")
            if isinstance(output, dict) and isinstance(input_command, dict):
                try:
                    output_linear = abs(float(output.get("linear_x", 0.0)))
                    input_linear = float(input_command.get("linear_x", 0.0))
                except (TypeError, ValueError):
                    output_linear = float("inf")
                    input_linear = 0.0
                if (
                    str(mux_status.get("governor_reason", ""))
                    == "governor_limited"
                    and input_linear > 0.05
                    and output_linear <= 0.05
                ):
                    return "mux_governor_stop", None
        event, age = self._recent_lifecycle_event(
            self.lifecycle_event_wall,
            ("turn_turn_started", "turn_turning", "turn_turn_completed"),
            now,
        )
        if event is not None:
            return "explicit_turn", age
        # TEB may deliberately rotate in place to acquire a newly exposed
        # Navfn route tangent even when the turn supervisor did not create a
        # dedicated connector phase.  The raw mux command is then a zero
        # linear velocity, but its selected trajectory explicitly requests
        # angular motion.  This is a route-geometry reorientation, not a
        # clear-path safety brake.
        selected_velocity = None
        if isinstance(self.teb_feedback_state, dict):
            selected_velocity = self.teb_feedback_state.get("selected_velocity")
        if isinstance(selected_velocity, dict):
            try:
                selected_linear = abs(float(selected_velocity.get("linear_x", 0.0)))
                selected_angular = abs(float(selected_velocity.get("angular_z", 0.0)))
            except (TypeError, ValueError):
                selected_linear = float("inf")
                selected_angular = 0.0
            if selected_linear <= 0.05 and selected_angular >= 0.10:
                return "teb_reorientation", None
        event, age = self._recent_lifecycle_event(
            self.lifecycle_event_wall,
            (
                "terminal",
                "move_base_succeeded",
                # Persistent TEB emits its endpoint terminal on the dedicated
                # terminal topic while the action lease remains alive until
                # the successor route is admitted.  The zero command can
                # therefore arrive after that event, without a generic
                # move_base SUCCEEDED callback in between.
                "persistent_execution_terminal",
                "endpoint_action_terminal",
            ),
            now,
        )
        if event is not None:
            return "action_terminal", age
        event, age = self._recent_lifecycle_event(
            self.lifecycle_event_wall,
            ("cancel", "frontier_route_invalidated", "handoff_requested"),
            now,
        )
        if event is not None:
            return "route_recovery", age
        event, age = self._recent_lifecycle_event(
            self.lifecycle_event_wall,
            ("move_base_recovery",),
            now,
            window=2.0,
        )
        if event is not None:
            return "planner_recovery", age
        event, age = self._recent_lifecycle_event(
            self.lifecycle_event_wall,
            ("dispatch",),
            now,
            window=0.45,
        )
        if event is not None:
            return "action_dispatch", age
        terminal_reorientation = self.pending_terminal_native_reorientation
        if (
            isinstance(terminal_reorientation, dict)
            and self.goal_source == "global_slam_frontier"
            and self.goal is not None
            and terminal_reorientation.get("goal") is not None
            and now - float(terminal_reorientation.get("armed_wall", now)) <= 30.0
        ):
            terminal_goal = terminal_reorientation["goal"]
            if math.hypot(
                float(self.goal[0]) - float(terminal_goal[0]),
                float(self.goal[1]) - float(terminal_goal[1]),
            ) <= 0.15:
                pose_for_goal = self._pose_xy_in_frame_locked(self.goal_frame)
                if pose_for_goal is not None:
                    remaining = math.hypot(
                        float(self.goal[0]) - float(pose_for_goal[0]),
                        float(self.goal[1]) - float(pose_for_goal[1]),
                    )
                    if remaining <= 1.20:
                        return "terminal_native_teb_reorientation", remaining
        # Gmapping/costmap updates can make the raw TEB command publisher
        # emit one zero sample while the selected TEB trajectory remains
        # forward. This is a sub-control-cycle synchronization gap, not an
        # obstacle brake. Require a fresh selected trajectory, an active
        # action, and sufficient distance from the terminal envelope so a
        # late feedback sample can never hide a genuine endpoint stop.
        selected_velocity = None
        if isinstance(self.teb_feedback_state, dict):
            selected_velocity = self.teb_feedback_state.get("selected_velocity")
        feedback_age = (
            None
            if self.teb_feedback_wall is None
            else max(0.0, now - self.teb_feedback_wall)
        )
        pose_for_goal = self._pose_xy_in_frame_locked(self.goal_frame)
        goal_distance = None
        if pose_for_goal is not None and self.goal is not None:
            goal_distance = math.hypot(
                float(self.goal[0]) - float(pose_for_goal[0]),
                float(self.goal[1]) - float(pose_for_goal[1]),
            )
        if isinstance(selected_velocity, dict):
            try:
                selected_forward = float(selected_velocity.get("linear_x", 0.0))
            except (TypeError, ValueError):
                selected_forward = 0.0
            if (
                self.bridge_active
                and self.teb_status == "trajectory_valid"
                and feedback_age is not None
                and feedback_age <= self.teb_control_cycle_gap_max_feedback_age
                and selected_forward >= self.forward_speed_threshold
                and goal_distance is not None
                and goal_distance >= self.teb_control_cycle_gap_min_goal_distance
            ):
                return "teb_control_cycle_gap", feedback_age
        if (
            self.goal_last_change_wall is not None
            and now - self.goal_last_change_wall <= 0.8
        ):
            return "goal_transition", now - self.goal_last_change_wall
        if (
            math.isfinite(self.scan_minimum)
            and self.scan_minimum <= self.discontinuity_obstacle_clearance
        ):
            return "near_obstacle", None
        if math.isfinite(self.scan_forward_minimum):
            return "unexplained_clear_path", None
        return "unknown_clearance", None

    @staticmethod
    def _increment_reason(counter, reason):
        counter[reason] = int(counter.get(reason, 0)) + 1
