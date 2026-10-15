"""TEB trajectory and obstacle telemetry for the navigation observer."""

import math
import time


class NavigationMetricsTebFeedbackMixin:
    """Observe selected TEB trajectories without changing controller behavior."""

    @staticmethod
    def _contract_int(value, default=0):
        try:
            return int(value or 0)
        except (TypeError, ValueError):
            return int(default)

    @classmethod
    def _planner_contract_identity(cls, message):
        """Decode the identity-bearing planner frame for passive validation."""
        return {
            "transaction_id": cls._contract_int(
                getattr(message, "transaction_id", 0)
            ),
            "route_id": cls._contract_int(getattr(message, "route_id", 0)),
            "graph_transaction_id": cls._contract_int(
                getattr(message, "graph_transaction_id", 0)
            ),
            "map_epoch": cls._contract_int(getattr(message, "map_epoch", 0)),
            "origin_epoch": cls._contract_int(
                getattr(message, "origin_epoch", 0)
            ),
            "projection_epoch": cls._contract_int(
                getattr(message, "projection_epoch", 0)
            ),
            "lifecycle_transaction_id": cls._contract_int(
                getattr(message, "lifecycle_transaction_id", "")
            ),
            "action_generation": cls._contract_int(
                getattr(message, "action_generation", 0)
            ),
            "planner_sequence": cls._contract_int(
                getattr(message, "planner_sequence", 0)
            ),
            "state": cls._contract_int(getattr(message, "state", 0)),
            "producer": str(getattr(message, "producer", "") or ""),
        }

    def _planner_contract_reference_locked(self, identity):
        route_id = int(identity.get("route_id", 0) or 0)
        if route_id > 0:
            return getattr(self, "bridge_dispatch_contracts", {}).get(route_id)
        return getattr(self, "bridge_latest_dispatch_contract", None)

    def _validate_planner_contract_locked(self, identity):
        """Validate planner identity against the bridge dispatch contract."""
        if identity["state"] not in (0, 1, 2):
            return "unknown_planner_contract_state"
        if identity["lifecycle_transaction_id"] <= 0:
            return "missing_lifecycle_transaction_id"
        if identity["action_generation"] <= 0:
            return "missing_action_generation"
        if identity["state"] == 1 and identity["route_id"] <= 0:
            return "missing_route_id"
        expected = self._planner_contract_reference_locked(identity)
        if expected is None:
            return "pending_bridge_dispatch"
        for field in (
            "transaction_id",
            "route_id",
            "lifecycle_transaction_id",
            "graph_transaction_id",
            "origin_epoch",
            "projection_epoch",
        ):
            expected_value = self._contract_int(expected.get(field))
            if expected_value > 0 and identity[field] != expected_value:
                return "planner_%s_mismatch" % field
        expected_map_epoch = expected.get("map_epoch")
        if expected_map_epoch is not None:
            if identity["map_epoch"] != self._contract_int(expected_map_epoch):
                return "planner_map_epoch_mismatch"
        expected_generation = self._contract_int(
            expected.get("action_generation")
        )
        if (
            identity["state"] == 2
            and identity["producer"] == "teb_goal_bridge"
        ):
            if identity["action_generation"] < expected_generation:
                return "planner_action_generation_regressed"
        elif expected_generation > 0 and identity["action_generation"] != expected_generation:
            return "planner_action_generation_mismatch"
        return ""

    def _record_planner_contract_locked(self, message, reconciled=False):
        identity = self._planner_contract_identity(message)
        producer = identity["producer"] or "unknown"
        previous_identity = getattr(self, "planner_contract_identity", None)
        previous_state = self._contract_int(
            getattr(self, "planner_contract_state", 0)
        )
        previous_validation = str(
            getattr(self, "planner_contract_validation", "") or ""
        )
        identity_fields = (
            "transaction_id",
            "route_id",
            "graph_transaction_id",
            "map_epoch",
            "origin_epoch",
            "projection_epoch",
            "lifecycle_transaction_id",
            "action_generation",
        )
        identity_changed = not isinstance(previous_identity, dict) or any(
            identity.get(field, 0) != previous_identity.get(field, 0)
            for field in identity_fields
        )
        contract_boundary = bool(
            identity_changed
            or previous_state != 1
            or previous_validation != "accepted"
        )
        previous_sequence = self.planner_contract_sequence_by_producer.get(
            producer, 0
        )
        sequence = identity["planner_sequence"]
        if not reconciled and sequence <= previous_sequence:
            validation = "stale_planner_sequence"
        else:
            self.planner_contract_sequence_by_producer[producer] = max(
                previous_sequence, sequence
            )
            validation = self._validate_planner_contract_locked(identity)
        command = getattr(message, "command", None)
        if command is None:
            command = getattr(self, "teb_planner_command", None)
        command_pair = [
            round(float(getattr(getattr(command, "linear", None), "x", 0.0)), 4),
            round(float(getattr(getattr(command, "angular", None), "z", 0.0)), 4),
        ]
        if not validation and identity["state"] == 1 and (
            abs(command_pair[0]) <= 0.001 and abs(command_pair[1]) <= 0.01
        ):
            validation = "active_contract_zero_command"
        elif not validation and identity["state"] != 1 and (
            abs(command_pair[0]) > 0.001 or abs(command_pair[1]) > 0.01
        ):
            validation = "boundary_contract_nonzero_command"
        state = identity["state"]
        if validation == "pending_bridge_dispatch":
            self.pending_planner_contract = message
        elif validation:
            self.planner_contract_rejections += 1
            self.pending_planner_contract = None
        else:
            self.pending_planner_contract = None
            self.planner_contract_authorized += 1
            self.teb_planner_command = command
            self.planner_contract_identity = dict(identity)
            self.planner_contract_state = state
            self.planner_contract_validation = "accepted"
            self.planner_contract_reason = str(
                getattr(message, "reason", "") or ""
            )
            if state == 1:
                if contract_boundary:
                    # The typed contract is the only identity-bearing
                    # boundary. A same-route contract sample must not erase a
                    # valid identity-free FeedbackMsg on every planner tick.
                    self.teb_feedback_valid = False
                    self.teb_feedback_invalid_reason = "awaiting_teb_feedback"
                self.teb_feedback_requires_fresh_planner_command = False
            else:
                self.teb_planner_command.linear.x = 0.0
                self.teb_planner_command.angular.z = 0.0
                self._invalidate_teb_feedback_locked(
                    "planner_contract_%s"
                    % (getattr(message, "reason", "boundary") or "boundary")
                )
        self.planner_contract_identity = dict(identity)
        self.planner_contract_state = state
        self.planner_contract_validation = (
            "pending_bridge_dispatch"
            if validation == "pending_bridge_dispatch"
            else "rejected" if validation else "accepted"
        )
        self.planner_contract_reason = str(
            getattr(message, "reason", "") or validation or ""
        )
        self._write(
            "WARN" if validation and validation != "pending_bridge_dispatch" else "INFO",
            "planner_command_contract",
            validation=validation or "accepted",
            reconciled=bool(reconciled),
            contract_identity=identity,
            command=command_pair,
            state=state,
            reason=str(getattr(message, "reason", "") or ""),
        )

    def _reconcile_pending_planner_contract_locked(self):
        pending = getattr(self, "pending_planner_contract", None)
        if pending is None:
            return
        identity = self._planner_contract_identity(pending)
        if self._validate_planner_contract_locked(identity) == "pending_bridge_dispatch":
            return
        self._record_planner_contract_locked(pending, reconciled=True)

    def on_planner_command_contract(self, message):
        """Observe and validate the typed planner command stream."""
        with self.lock:
            self._record_planner_contract_locked(message)

    _TEB_BOUNDARY_REASONS = frozenset(
        {
            "new_action_dispatch",
            "route_owner_changed",
            "persistent_endpoint_terminal",
            "execution_terminal",
            "controller_lease_released",
            "action_lease_cleared",
            "bridge_inactive",
            "hard_reset",
            "task_done",
        }
    )

    def _invalidate_teb_feedback_locked(self, reason, clear_planner=False):
        """Mark identity-free feedback unusable until a planner command returns."""
        reason = str(reason or "control_pipeline_boundary")
        previous_valid = bool(getattr(self, "teb_feedback_valid", False))
        previous_reason = str(
            getattr(self, "teb_feedback_invalid_reason", "") or ""
        )
        previous_requires = bool(
            getattr(
                self,
                "teb_feedback_requires_fresh_planner_command",
                False,
            )
        )
        if (
            reason == "planner_zero_command"
            and previous_requires
            and previous_reason in self._TEB_BOUNDARY_REASONS
        ):
            # Preserve the stronger terminal/owner boundary when its trailing
            # zero command is delivered on the separate planner topic.
            return
        previous_state = getattr(self, "teb_feedback_state", None)
        changed = bool(
            previous_valid
            or previous_reason != reason
            or not bool(
                getattr(
                    self,
                    "teb_feedback_requires_fresh_planner_command",
                    False,
                )
            )
        )
        self.teb_feedback_valid = False
        self.teb_feedback_invalid_reason = reason
        self.teb_feedback_requires_fresh_planner_command = True
        self.teb_feedback_identity = None
        self.teb_feedback_invalidation_count = int(
            getattr(self, "teb_feedback_invalidation_count", 0) or 0
        ) + 1
        if isinstance(previous_state, dict):
            state = dict(previous_state)
            state["valid"] = False
            state["invalid_reason"] = reason
            self.teb_feedback_state = state
        if clear_planner:
            command = getattr(self, "teb_planner_command", None)
            if command is not None:
                try:
                    command.linear.x = 0.0
                    command.angular.z = 0.0
                except AttributeError:
                    pass
            self.last_teb_planner_cmd_wall = None
        self.teb_status = "invalidated"
        if changed:
            writer = getattr(self, "_write", None)
            if callable(writer):
                writer(
                    "WARN",
                    "teb_feedback_invalidated",
                    reason=reason,
                    clear_planner=bool(clear_planner),
                    previous_valid=previous_valid,
                    invalidation_count=int(
                        self.teb_feedback_invalidation_count
                    ),
                )

    @staticmethod
    def _point_segment_distance(px, py, ax, ay, bx, by):
        """Return Euclidean distance from one 2-D point to a finite segment."""
        dx = bx - ax
        dy = by - ay
        denominator = dx * dx + dy * dy
        if denominator <= 1e-12:
            return math.hypot(px - ax, py - ay)
        ratio = ((px - ax) * dx + (py - ay) * dy) / denominator
        ratio = max(0.0, min(1.0, ratio))
        return math.hypot(px - (ax + ratio * dx), py - (ay + ratio * dy))

    def _teb_obstacle_snapshot_locked(self, message):
        """Summarize nearby converter obstacles in the feedback frame.

        Feedback obstacle polygons use the same frame as the trajectory. The
        wheel odometry pose is normally ``odom`` too; retain the frame in the
        event so a future frame mismatch is explicit instead of silently
        becoming a misleading distance.
        """
        pose = self.pose
        if pose is None:
            return {"frame": message.header.frame_id or None, "nearest": None}
        px, py = float(pose[0]), float(pose[1])
        nearest = None
        summaries = []
        for obstacle in list(message.obstacles_msg.obstacles):
            points = list(obstacle.polygon.points)
            boundary_distance = float("inf")
            if len(points) == 1:
                boundary_distance = math.hypot(
                    px - float(points[0].x), py - float(points[0].y)
                )
            elif len(points) >= 2:
                for first, second in zip(points, points[1:] + points[:1]):
                    boundary_distance = min(
                        boundary_distance,
                        self._point_segment_distance(
                            px,
                            py,
                            float(first.x),
                            float(first.y),
                            float(second.x),
                            float(second.y),
                        ),
                    )
            effective_distance = max(0.0, boundary_distance - float(obstacle.radius))
            xs = [float(point.x) for point in points]
            ys = [float(point.y) for point in points]
            summary = {
                "id": int(obstacle.id),
                "points": len(points),
                "radius": round(float(obstacle.radius), 4),
                "boundary_distance": (
                    None if not math.isfinite(boundary_distance)
                    else round(boundary_distance, 4)
                ),
                "effective_distance": (
                    None if not math.isfinite(effective_distance)
                    else round(effective_distance, 4)
                ),
                "bounds": (
                    None if not xs else [
                        round(min(xs), 3), round(min(ys), 3),
                        round(max(xs), 3), round(max(ys), 3),
                    ]
                ),
            }
            summaries.append(summary)
            if math.isfinite(effective_distance) and (
                nearest is None
                or effective_distance < nearest["effective_distance"]
            ):
                nearest = summary
        summaries.sort(
            key=lambda item: float("inf")
            if item["effective_distance"] is None else item["effective_distance"]
        )
        return {
            "frame": message.header.frame_id or None,
            "nearest": nearest,
            "nearest_three": summaries[:3],
        }

    def on_teb_feedback(self, message):
        """Record the selected TEB trajectory and sustained zero-velocity evidence."""
        with self.lock:
            now = time.monotonic()
            if not bool(getattr(self, "bridge_active", False)):
                self._invalidate_teb_feedback_locked("bridge_inactive")
                return
            if bool(
                getattr(
                    self,
                    "teb_feedback_requires_fresh_planner_command",
                    False,
                )
            ):
                # FeedbackMsg has no route identity. A sample received before
                # the current planner command may belong to the predecessor.
                return
            trajectories = list(message.trajectories)
            selected_index = int(message.selected_trajectory_idx)
            selected = None
            if 0 <= selected_index < len(trajectories):
                selected = trajectories[selected_index]
            first = selected.trajectory[0] if selected is not None and selected.trajectory else None
            selected_velocity = None if first is None else {
                "linear_x": round(float(first.velocity.linear.x), 4),
                "angular_z": round(float(first.velocity.angular.z), 4),
            }
            obstacle_count = len(message.obstacles_msg.obstacles)
            self.teb_feedback_state = {
                "trajectories": len(trajectories),
                "selected_index": selected_index,
                "selected_points": 0 if selected is None else len(selected.trajectory),
                "selected_velocity": selected_velocity,
                "obstacles": obstacle_count,
                "valid": True,
                "invalid_reason": "",
            }
            self.teb_feedback_wall = now
            self.teb_feedback_valid = True
            self.teb_feedback_invalid_reason = ""
            self.teb_feedback_requires_fresh_planner_command = False
            self.teb_feedback_identity = getattr(
                self, "teb_feedback_owner_identity", None
            )
            if selected is None:
                self.teb_status = "no_selected_trajectory"
            elif first is None:
                self.teb_status = "selected_trajectory_empty"
            elif abs(float(first.velocity.linear.x)) <= 0.002 and abs(float(first.velocity.angular.z)) <= 0.01:
                self.teb_status = "selected_command_near_zero"
            else:
                self.teb_status = "trajectory_valid"
            near_zero_linear = (
                first is not None
                and abs(float(first.velocity.linear.x)) <= 0.01
            )
            turn_in_progress = (
                isinstance(self.teb_turn_supervisor_status, dict)
                and str(
                    self.teb_turn_supervisor_status.get("state", "")
                ).strip().upper() == "TURNING"
            )
            clear_forward = (
                math.isfinite(self.scan_forward_minimum)
                and self.scan_forward_minimum > self.discontinuity_obstacle_clearance
            )
            if (
                self.bridge_active
                and not turn_in_progress
                and near_zero_linear
                and clear_forward
            ):
                if self.teb_zero_velocity_start_wall is None:
                    self.teb_zero_velocity_start_wall = now
                plateau_seconds = now - self.teb_zero_velocity_start_wall
                if (
                    plateau_seconds >= self.teb_zero_velocity_snapshot_min_duration
                    and self.teb_zero_velocity_snapshot_wall
                    < self.teb_zero_velocity_start_wall
                ):
                    self.teb_zero_velocity_snapshot_wall = now
                    selected_start = None
                    selected_endpoint = None
                    if selected is not None and selected.trajectory:
                        selected_start = selected.trajectory[0].pose.position
                        selected_endpoint = selected.trajectory[-1].pose.position
                    self._write(
                        "WARN",
                        "teb_zero_velocity_snapshot",
                        plateau_seconds=round(plateau_seconds, 3),
                        pose=None if self.pose is None else [
                            round(float(value), 4) for value in self.pose
                        ],
                        goal=None if self.goal is None else [
                            round(float(value), 4) for value in self.goal
                        ],
                        goal_frame=self.goal_frame,
                        scan_forward_min=round(
                            float(self.scan_forward_minimum), 4
                        ),
                        selected_velocity=selected_velocity,
                        selected_start=(
                            None if selected_start is None else [
                                round(float(selected_start.x), 4),
                                round(float(selected_start.y), 4),
                            ]
                        ),
                        selected_endpoint=(
                            None if selected_endpoint is None else [
                                round(float(selected_endpoint.x), 4),
                                round(float(selected_endpoint.y), 4),
                            ]
                        ),
                        teb_local_plan=self.teb_local_plan_stats,
                        teb_global_plan=self.teb_global_plan_stats,
                        obstacles=self._teb_obstacle_snapshot_locked(message),
                    )
            else:
                self.teb_zero_velocity_start_wall = None
            if selected is None or now - self.last_teb_feedback_log_wall >= 1.0:
                self.last_teb_feedback_log_wall = now
                self._write(
                    "WARN" if selected is None else "INFO",
                    "teb_feedback",
                    status=self.teb_status,
                    **self.teb_feedback_state,
                )
