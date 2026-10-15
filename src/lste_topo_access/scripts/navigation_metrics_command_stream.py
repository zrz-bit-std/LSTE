"""Final actuator command quality accounting for navigation telemetry.

This module is observer-only: it records evidence and never publishes a motion command.
"""

import json
import math
import time


class NavigationMetricsCommandStreamMixin:
    def on_cmd(self, message):
        with self.lock:
            previous = self.command
            self.command = message
            self.cmd_messages += 1
            angular = float(message.angular.z)
            previous_angular = float(previous.angular.z)
            current_linear = float(message.linear.x)
            previous_linear = float(previous.linear.x)
            # Forward steering energy: integrate |angular| only while the robot
            # is actually travelling forward, so pure in-place turns at walls
            # do not pollute the straight-line wobble measurement.
            now_wall = time.monotonic()
            straight_path_state = self._straight_path_state_locked(now_wall)
            if self.last_cmd_wall is not None:
                dt = max(0.0, now_wall - self.last_cmd_wall)
                if current_linear > self.forward_speed_threshold:
                    self.forward_distance += current_linear * dt
                    self.forward_angular_energy += abs(angular) * dt
                    if straight_path_state["is_straight"]:
                        self.straight_path_distance += current_linear * dt
                        self.straight_path_angular_energy += abs(angular) * dt
                        self.straight_path_samples += 1
            self.last_cmd_wall = now_wall
            sign = 1 if angular > 0.05 else -1 if angular < -0.05 else 0
            previous_sign = 1 if previous_angular > 0.05 else -1 if previous_angular < -0.05 else 0
            if sign and self.last_nonzero_angular_sign and sign != self.last_nonzero_angular_sign:
                self.angular_sign_flips += 1
                if (
                    abs(angular) >= self.teb_strong_angular_threshold
                    and abs(previous_angular) >= self.teb_strong_angular_threshold
                ):
                    self.strong_angular_sign_flips += 1
                self._write(
                    "WARN",
                    "angular_sign_flip",
                    angular=round(angular, 4),
                    previous_angular=round(previous_angular, 4),
                    count=self.angular_sign_flips,
                    strong_count=self.strong_angular_sign_flips,
                )
            if (
                sign
                and previous_sign
                and sign != previous_sign
                and current_linear > self.forward_speed_threshold
                and previous_linear > self.forward_speed_threshold
            ):
                self.forward_steering_sign_flips += 1
                self._write(
                    "WARN",
                    "forward_steering_sign_flip",
                    angular=round(angular, 4),
                    previous_angular=round(previous_angular, 4),
                    count=self.forward_steering_sign_flips,
                    current_linear=round(current_linear, 4),
                    previous_linear=round(previous_linear, 4),
                    goal=None if self.goal is None else [round(value, 3) for value in self.goal],
                )
            if not straight_path_state["is_straight"]:
                # A direction change across a bend is intentional route
                # tracking, never evidence of corridor wobble.
                self.straight_path_last_nonzero_sign = 0
                self.straight_path_last_nonzero_sign_wall = 0.0
            elif sign and current_linear > self.forward_speed_threshold:
                prior_sign = self.straight_path_last_nonzero_sign
                prior_age = now_wall - self.straight_path_last_nonzero_sign_wall
                if (
                    prior_sign
                    and prior_sign != sign
                    and prior_age <= self.straight_path_sign_flip_window
                ):
                    self.straight_path_steering_sign_flips += 1
                    self._write(
                        "WARN",
                        "straight_path_wobble_sign_flip",
                        angular=round(angular, 4),
                        previous_angular=round(previous_angular, 4),
                        count=self.straight_path_steering_sign_flips,
                        previous_straight_sign=prior_sign,
                        sign_gap_seconds=round(prior_age, 4),
                        current_linear=round(current_linear, 4),
                        plan_curvature_rad=straight_path_state["plan_curvature_rad"],
                        heading_error_rad=straight_path_state["heading_error_rad"],
                        local_plan_curvature_rad=straight_path_state[
                            "local_plan_curvature_rad"
                        ],
                        local_heading_error_rad=straight_path_state[
                            "local_heading_error_rad"
                        ],
                        goal=None if self.goal is None else [
                            round(value, 3) for value in self.goal
                        ],
                    )
                self.straight_path_last_nonzero_sign = sign
                self.straight_path_last_nonzero_sign_wall = now_wall
            if sign:
                self.last_nonzero_angular_sign = sign
            if previous_linear > 0.05 and current_linear < previous_linear - 0.08:
                is_stop_brake = current_linear <= 0.05
                if not is_stop_brake:
                    self.speed_modulation_events += 1
                    self._write(
                        "INFO",
                        "speed_modulation",
                        count=self.speed_modulation_events,
                        previous_linear=round(previous_linear, 4),
                        current_linear=round(current_linear, 4),
                        angular=round(angular, 4),
                        delta=round(current_linear - previous_linear, 4),
                        scan_forward_min=(
                            None
                            if not math.isfinite(self.scan_forward_minimum)
                            else round(self.scan_forward_minimum, 4)
                        ),
                        pose=(
                            None
                            if self.pose is None
                            else [round(value, 3) for value in self.pose]
                        ),
                        goal=(
                            None
                            if self.goal is None
                            else [round(value, 3) for value in self.goal]
                        ),
                    )
                else:
                    self.linear_brake_events += 1
                    self.hard_stop_events += 1
                if is_stop_brake:
                    forward_clear = self.scan_forward_minimum
                    if not math.isfinite(forward_clear):
                        self.brake_events_unknown_clearance += 1
                    elif forward_clear >= 0.60:
                        self.brake_events_clear += 1
                    else:
                        self.brake_events_near += 1
                    now = time.monotonic()
                    transition_age = (
                        None if self.last_goal_transition_wall is None
                        else max(0.0, now - self.last_goal_transition_wall)
                    )
                    transition_kind = (
                        None
                        if transition_age is None or transition_age > 0.50
                        else self.goal_transition_kind
                    )
                    if transition_kind is not None:
                        self.transition_brake_events[transition_kind] += 1
                    brake_reason, lifecycle_age = self._command_discontinuity_reason_locked(now)
                    self._increment_reason(self.brake_reason_counts, brake_reason)
                    discontinuity = self._remember_discontinuity_locked(
                        "linear_brake", brake_reason, now
                    )
                    if now - self.last_brake_wall >= 0.15:
                        self.last_brake_wall = now
                        self._write(
                            "WARN",
                            "linear_brake",
                            count=self.linear_brake_events,
                            discontinuity_id=int(discontinuity["id"]),
                            previous_linear=round(previous_linear, 4),
                            current_linear=round(current_linear, 4),
                            angular=round(angular, 4),
                            delta=round(current_linear - previous_linear, 4),
                            controller_source=self.controller_source,
                            controller_reason=self.controller_reason,
                            scan_forward_min=None if not math.isfinite(self.scan_forward_minimum) else round(self.scan_forward_minimum, 4),
                            scan_min=None if not math.isfinite(self.scan_minimum) else round(self.scan_minimum, 4),
                            obstacle_clearance_threshold=round(
                                self.discontinuity_obstacle_clearance, 4
                            ),
                            pose=None if self.pose is None else [round(value, 3) for value in self.pose],
                            goal=None if self.goal is None else [round(value, 3) for value in self.goal],
                            reason=brake_reason,
                            lifecycle_age_seconds=(
                                None if lifecycle_age is None else round(lifecycle_age, 4)
                            ),
                            move_base_status=self.last_move_base_status,
                            bridge_event=self.bridge_last_event,
                            route_transition_kind=transition_kind,
                            route_transition_age_seconds=(
                                None if transition_kind is None else round(transition_age, 4)
                            ),
                        )
            was_turn_only = abs(previous_linear) <= 0.01 and abs(previous_angular) > 0.05
            is_turn_only = abs(current_linear) <= 0.01 and abs(angular) > 0.05
            if is_turn_only and not was_turn_only:
                self.turn_only_events += 1
                self.turn_only_start_wall = time.monotonic()
                self._write(
                    "INFO",
                    "turn_only_start",
                    count=self.turn_only_events,
                    angular=round(angular, 4),
                    controller_source=self.controller_source,
                    controller_reason=self.controller_reason,
                    scan_forward_min=None if not math.isfinite(self.scan_forward_minimum) else round(self.scan_forward_minimum, 4),
                    scan_min=None if not math.isfinite(self.scan_minimum) else round(self.scan_minimum, 4),
                )
            elif not is_turn_only and was_turn_only and self.turn_only_start_wall is not None:
                duration = time.monotonic() - self.turn_only_start_wall
                self.turn_only_duration_total += duration
                self._write(
                    "INFO",
                    "turn_only_end",
                    duration_seconds=round(duration, 3),
                    total_duration_seconds=round(self.turn_only_duration_total, 3),
                )
                self.turn_only_start_wall = None
            was_moving = abs(float(previous.linear.x)) > 0.05 or abs(previous_angular) > 0.05
            is_zero = abs(float(message.linear.x)) <= 0.01 and abs(angular) <= 0.01
            if was_moving and is_zero:
                self.stop_events += 1
                self.zero_start_wall = time.monotonic()
                stop_reason, lifecycle_age = self._command_discontinuity_reason_locked(
                    self.zero_start_wall
                )
                self._increment_reason(self.stop_reason_counts, stop_reason)
                discontinuity = self._remember_discontinuity_locked(
                    "command_stop", stop_reason, self.zero_start_wall
                )
                self._write(
                    "WARN",
                    "command_stop",
                    count=self.stop_events,
                    discontinuity_id=int(discontinuity["id"]),
                    controller_mode=self.controller_mode,
                    controller_source=self.controller_source,
                    controller_reason=self.controller_reason,
                    teb_status=self.teb_status,
                    bridge_event=self.bridge_last_event,
                    reason=stop_reason,
                    lifecycle_age_seconds=(
                        None if lifecycle_age is None else round(lifecycle_age, 4)
                    ),
                    move_base_status=(
                        None
                        if self.move_base_feedback_state is None
                        else self.move_base_feedback_state.get("status_name")
                    ),
                )
            elif not is_zero and self.zero_start_wall is not None:
                duration = time.monotonic() - self.zero_start_wall
                self.zero_duration_total += duration
                self.stop_duration_count += 1
                self.last_stop_duration = duration
                self.max_stop_duration = max(self.max_stop_duration, duration)
                self._write(
                    "INFO",
                    "stop_end",
                    duration_seconds=round(duration, 3),
                    total_duration_seconds=round(self.zero_duration_total, 3),
                    max_duration_seconds=round(self.max_stop_duration, 3),
                )
                self.zero_start_wall = None

    def on_cmd_vel_mux_status(self, message):
        """Record the controller/safety decision that produced /cmd_vel.

        This callback never changes a command. It only gives the metrics
        layer direct causality for final actuator limits and can revise the
        preceding /cmd_vel discontinuity when ROS delivers the two topics in
        opposite callback order.
        """
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        with self.lock:
            now = time.monotonic()
            self.cmd_vel_mux_status = payload
            self.cmd_vel_mux_status_wall = now
            block_reason = str(payload.get("block_reason", "none"))
            governor_reason = str(payload.get("governor_reason", "none"))
            output = payload.get("output")
            input_command = payload.get("input")
            output_linear = None
            input_linear = None
            if isinstance(output, dict):
                try:
                    output_linear = float(output.get("linear_x", 0.0))
                except (TypeError, ValueError):
                    pass
            if isinstance(input_command, dict):
                try:
                    input_linear = float(input_command.get("linear_x", 0.0))
                except (TypeError, ValueError):
                    pass
            reason = "none"
            if block_reason in ("navigation_hold", "task_complete"):
                reason = block_reason
                self.mux_forced_zero_events += 1
                self.lifecycle_event_wall["mux_" + reason] = now
            elif (
                governor_reason == "governor_limited"
                and input_linear is not None
                and input_linear > 0.05
            ):
                reason = "mux_governor_stop" if (
                    output_linear is not None and abs(output_linear) <= 0.05
                ) else "mux_governor_limited"
                self.mux_governor_limited_events += 1
                self.lifecycle_event_wall[reason] = now
            self._increment_reason(self.mux_status_reason_counts, reason)
            if reason != "none":
                self._reclassify_recent_discontinuities_locked(
                    now,
                    "cmd_vel_mux_status",
                    replacement_reason=reason,
                    window=0.35,
                )
                self._write(
                    "WARN" if reason != "mux_governor_limited" else "INFO",
                    "cmd_vel_mux_intervention",
                    reason=reason,
                    source=payload.get("source"),
                    selected_mode=payload.get("selected_mode"),
                    input=input_command,
                    output=output,
                    governor_linear_limit=payload.get("governor_linear_limit"),
                    scan_forward_min=payload.get("scan_forward_min"),
                    scan_age_seconds=payload.get("scan_age_seconds"),
                    filter_reason=payload.get("filter_reason"),
                )
