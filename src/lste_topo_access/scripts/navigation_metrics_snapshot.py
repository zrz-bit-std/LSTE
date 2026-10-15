"""Periodic snapshot construction and orderly metrics shutdown."""

import math
import time

import rospy


class NavigationMetricsSnapshotMixin:
    """Render one consistent telemetry snapshot from observer state."""

    def _route_identity_snapshot(self):
        """Return only the active graph identity for periodic run snapshots.

        Failure samples already retain the complete cross-topic route context,
        but the ordinary 0.5 Hz snapshot used by a timeout report did not.  A
        compact projection keeps timeout/interrupt diagnosis self-contained
        without copying the large frontier status payload into every sample.
        """
        context_reader = getattr(self, "_failure_route_context_locked", None)
        if not callable(context_reader):
            return {}
        try:
            context = context_reader()
        except Exception:
            return {}
        route = context.get("route") if isinstance(context, dict) else None
        if not isinstance(route, dict):
            return {}
        fields = (
            "route_id", "active_route_id", "route_kind", "active_route_kind",
            "mission_route_kind", "place_id", "source_place_id",
            "graph_transaction_id", "active_graph_transaction_id",
            "latest_graph_transaction_id",
            "destination_place_id", "work_item_id", "attempt_id",
            "transaction_id", "region_id", "region_state", "action",
            "graph_action", "obligation_kind", "obligation_id",
            "portal_id", "first_portal_id", "portal_path", "target_place_id",
            "goal", "reason", "materialization_reason", "map_epoch",
            "rejected_map_epoch", "last_rejection_reason",
            "controller_pending", "terminal_received", "target_failure_latched",
        )
        return {
            key: route[key]
            for key in fields
            if key in route and route[key] is not None
        }

    def _snapshot(self):
        pose = self.pose
        goal = self.goal
        distance = float("nan")
        pose_for_goal = self._pose_xy_in_frame_locked(self.goal_frame)
        if pose_for_goal is not None and goal is not None:
            distance = math.hypot(goal[0] - pose_for_goal[0], goal[1] - pose_for_goal[1])
        average_goal_delta = (
            self.goal_delta_sum / max(1, self.goal_messages - 1)
        )
        elapsed_wall = max(0.001, time.monotonic() - self.start_wall)
        stop_rate_per_minute = self.stop_events * 60.0 / elapsed_wall
        goal_change_rate_per_minute = self.goal_changes * 60.0 / elapsed_wall
        brake_rate_per_minute = self.linear_brake_events * 60.0 / elapsed_wall
        average_stop_duration = (
            self.zero_duration_total / self.stop_duration_count
            if self.stop_duration_count else 0.0
        )
        return {
            "execution_architecture": self.execution_architecture,
            "persistent_execution": self.persistent_execution,
            "failure_evidence": self._failure_snapshot_state(),
            # Keep timeout/interrupt reports attributable to a physical graph
            # action without embedding all cross-topic status payloads here.
            "route_identity": self._route_identity_snapshot(),
            "ros_time": round(rospy.Time.now().to_sec(), 3),
            "pose": None if pose is None else [round(float(value), 4) for value in pose],
            "goal": None if goal is None else [round(float(value), 4) for value in goal],
            "distance_to_goal": None if not math.isfinite(distance) else round(distance, 4),
            "pose_frame": "odom",
            "goal_frame": self.goal_frame,
            "distance_transform_failures": self.distance_transform_failures,
            "path_length": round(self.path_length, 4),
            "cmd": [round(float(self.command.linear.x), 4), round(float(self.command.angular.z), 4)],
            "cmd_vel_mux": self.cmd_vel_mux_status,
            "mux_governor_limited_events": self.mux_governor_limited_events,
            "mux_forced_zero_events": self.mux_forced_zero_events,
            "mux_status_reason_counts": self.mux_status_reason_counts,
            "teb_cmd": [round(float(self.teb_command.linear.x), 4), round(float(self.teb_command.angular.z), 4)],
            "teb_planner_cmd": [
                round(float(self.teb_planner_command.linear.x), 4),
                round(float(self.teb_planner_command.angular.z), 4),
            ],
            "teb_turn_supervisor": self.teb_turn_supervisor_status,
            "teb_turn_supervisor_events": self.teb_turn_supervisor_events,
            "teb_turn_supervisor_last_event": self.teb_turn_supervisor_last_event,
            "teb_trajectory_continuity_events": self.teb_trajectory_continuity_events,
            "scan_min": None if not math.isfinite(self.scan_minimum) else round(self.scan_minimum, 4),
            "scan_forward_min": None if not math.isfinite(self.scan_forward_minimum) else round(self.scan_forward_minimum, 4),
            "scan_left_min": None if not math.isfinite(self.scan_left_minimum) else round(self.scan_left_minimum, 4),
            "scan_right_min": None if not math.isfinite(self.scan_right_minimum) else round(self.scan_right_minimum, 4),
            "controller_mode": self.controller_mode,
            "controller_status": self.controller_status,
            "controller_source": self.controller_source,
            "controller_reason": self.controller_reason,
            "controller_requested": [round(value, 4) if math.isfinite(value) else None for value in self.controller_requested],
            "controller_action": [round(value, 4) if math.isfinite(value) else None for value in self.controller_action],
            "controller_predicted_clearance": None if not math.isfinite(self.controller_predicted_clearance) else round(self.controller_predicted_clearance, 4),
            "teb_status": self.teb_status if self.controller_mode == "teb" else "not_applicable",
            "teb_feedback": self.teb_feedback_state if self.controller_mode == "teb" else None,
            "teb_feedback_valid": bool(
                getattr(self, "teb_feedback_valid", False)
            ),
            "teb_feedback_invalid_reason": str(
                getattr(self, "teb_feedback_invalid_reason", "no_feedback")
                or "no_feedback"
            ),
            "teb_feedback_requires_fresh_planner_command": bool(
                getattr(
                    self,
                    "teb_feedback_requires_fresh_planner_command",
                    True,
                )
            ),
            "teb_feedback_identity": getattr(
                self, "teb_feedback_identity", None
            ),
            "teb_feedback_invalidation_count": int(
                getattr(self, "teb_feedback_invalidation_count", 0) or 0
            ),
            "planner_command_contract": {
                "identity": getattr(self, "planner_contract_identity", None),
                "state": getattr(self, "planner_contract_state", None),
                "validation": getattr(
                    self, "planner_contract_validation", "unobserved"
                ),
                "reason": getattr(self, "planner_contract_reason", "startup"),
                "authorized": int(
                    getattr(self, "planner_contract_authorized", 0) or 0
                ),
                "rejections": int(
                    getattr(self, "planner_contract_rejections", 0) or 0
                ),
                "pending": bool(
                    getattr(self, "pending_planner_contract", None) is not None
                ),
            },
            "frontier_execution_terminal_contract": {
                "identity": getattr(self, "terminal_contract_identity", None),
                "validation": getattr(
                    self, "terminal_contract_validation", "unobserved"
                ),
                "count": int(
                    getattr(self, "terminal_contract_count", 0) or 0
                ),
                "rejections": int(
                    getattr(self, "terminal_contract_rejections", 0) or 0
                ),
                "pending": len(
                    getattr(self, "pending_terminal_contracts", ())
                ),
            },
            "move_base_feedback": self.move_base_feedback_state,
            "recovery": self.recovery_state,
            "move_base_recovery_events": self.move_base_recovery_events,
            "terminal_to_dispatch_count": self.terminal_to_dispatch_count,
            "terminal_to_dispatch_last_seconds": (
                None
                if self.terminal_to_dispatch_last is None
                else round(self.terminal_to_dispatch_last, 4)
            ),
            "terminal_to_dispatch_mean_seconds": round(
                self.terminal_to_dispatch_total
                / max(1, self.terminal_to_dispatch_count),
                4,
            ),
            "terminal_to_dispatch_max_seconds": round(
                self.terminal_to_dispatch_max, 4
            ),
            "global_costmap": self.global_costmap_stats,
            "local_costmap": self.local_costmap_stats,
            "navfn_plan": self.navfn_plan_stats,
            "global_planner_plan": self.global_planner_plan_stats,
            "teb_global_plan": self.teb_global_plan_stats,
            "teb_local_plan": self.teb_local_plan_stats,
            "state": self.state,
            "goal_source": self.goal_source,
            "goal_transaction_id": self.goal_transaction_id,
            "mission_goal_messages": self.mission_goal_messages,
            "goal_diagnostic": self.goal_diagnostic,
            "task_done": self.task_done,
            "navigation_hold": self.navigation_hold,
            "navigation_hold_events": self.navigation_hold_events,
            "navigation_hold_duration_seconds": round(
                self.navigation_hold_duration_total
                + (
                    0.0 if self.navigation_hold_start_wall is None
                    else max(0.0, time.monotonic() - self.navigation_hold_start_wall)
                ),
                3,
            ),
            "goal_messages": self.goal_messages,
            "goal_changes": self.goal_changes,
            "goal_delta_mean": round(average_goal_delta, 4),
            "goal_delta_max": round(self.goal_delta_max, 4),
            "move_base_dispatches": self.dispatch_count,
            "move_base_unique_goal_ids": len(self.move_base_goal_ids),
            "move_base_preemptions": self.preemptions,
            "move_base_frontier_observation_preemptions": (
                self.frontier_observation_preemptions
            ),
            "move_base_frontier_terminal_settle_preemptions": (
                self.frontier_terminal_settle_preemptions
            ),
            "move_base_frontier_continuous_prefetch_preemptions": (
                self.frontier_continuous_prefetch_preemptions
            ),
            "move_base_frontier_segment_preemptions": self.frontier_segment_preemptions,
            "move_base_target_segment_preemptions": self.target_segment_preemptions,
            "move_base_target_terminal_observation_preemptions": (
                self.target_terminal_observation_preemptions
            ),
            "move_base_priority_preemptions": self.priority_preemptions,
            "move_base_task_done_preemptions": self.task_done_preemptions,
            "move_base_pending_hard_reset_preemptions": int(
                getattr(self, "pending_hard_reset_preemptions", 0) or 0
            ),
            "move_base_route_recovery_preemptions": self.route_recovery_preemptions,
            "move_base_route_recovery_preemption_reasons": dict(
                self.route_recovery_preemption_reasons
            ),
            "move_base_unexpected_preemptions": self.unexpected_preemptions,
            "move_base_aborts": self.aborts,
            "move_base_successes": self.successes,
            "angular_sign_flips": self.angular_sign_flips,
            "strong_angular_sign_flips": self.strong_angular_sign_flips,
            "forward_steering_sign_flips": self.forward_steering_sign_flips,
            "teb_angular_sign_flips": self.teb_angular_sign_flips,
            "teb_strong_angular_sign_flips": self.teb_strong_angular_sign_flips,
            "teb_forward_steering_sign_flips": self.teb_forward_steering_sign_flips,
            # ``teb_cmd`` is after the turn supervisor; ``teb_planner_cmd``
            # is the unmodified move_base/TEB stream. `/cmd_vel` remains the
            # actual actuator request after the safety mux.
            "teb_supervisor_linear_brake_events": self.teb_linear_brake_events,
            "teb_supervisor_speed_modulation_events": self.teb_speed_modulation_events,
            "teb_planner_linear_brake_events": self.teb_planner_linear_brake_events,
            "teb_planner_speed_modulation_events": (
                self.teb_planner_speed_modulation_events
            ),
            # Compatibility aliases for logs/analyzers written before the
            # three command channels were explicitly named.
            "teb_linear_brake_events": self.teb_linear_brake_events,
            "teb_speed_modulation_events": self.teb_speed_modulation_events,
            "forward_distance_m": round(self.forward_distance, 3),
            "forward_angular_energy_rad": round(self.forward_angular_energy, 4),
            "forward_angular_energy_per_m": round(
                self.forward_angular_energy / max(0.01, self.forward_distance), 4
            ),
            # Unlike forward_angular_energy_per_m, these values exclude both
            # planned bends and heading-acquisition turns. They are the
            # primary evidence for actual left/right wobble on a straight
            # corridor or open path.
            "straight_path_distance_m": round(self.straight_path_distance, 3),
            "straight_path_angular_energy_rad": round(
                self.straight_path_angular_energy, 4
            ),
            "straight_path_angular_energy_per_m": round(
                self.straight_path_angular_energy
                / max(0.01, self.straight_path_distance),
                4,
            ),
            "straight_path_steering_sign_flips": (
                self.straight_path_steering_sign_flips
            ),
            "straight_path_samples": self.straight_path_samples,
            "straight_path_tracking": self.straight_path_state,
            "straight_path_lookahead_m": round(self.straight_path_lookahead, 3),
            "straight_path_max_curvature_rad": round(
                self.straight_path_max_curvature, 4
            ),
            "straight_path_max_heading_error_rad": round(
                self.straight_path_max_heading_error, 4
            ),
            "forward_speed_threshold": round(self.forward_speed_threshold, 4),
            "brake_events_clear": self.brake_events_clear,
            "brake_events_near": self.brake_events_near,
            "brake_events_unknown_clearance": self.brake_events_unknown_clearance,
            # Raw clearance describes sensor state at a speed drop. Final
            # lifecycle attribution distinguishes an expected action endpoint
            # from an actual clear-path interruption.
            "endpoint_terminal_brake_events": (
                self.brake_reason_counts.get("action_terminal", 0)
                + self.brake_reason_counts.get("endpoint_action_terminal", 0)
                + self.brake_reason_counts.get("persistent_endpoint_terminal", 0)
            ),
            "action_terminal_brake_events": self.brake_reason_counts.get("action_terminal", 0),
            "unexplained_clear_path_brake_events": self.brake_reason_counts.get(
                "unexplained_clear_path", 0
            ),
            "stop_events": self.stop_events,
            "stop_rate_per_minute": round(stop_rate_per_minute, 3),
            "stop_duration_count": self.stop_duration_count,
            "average_stop_duration_seconds": round(average_stop_duration, 3),
            "max_stop_duration_seconds": round(self.max_stop_duration, 3),
            "last_stop_duration_seconds": (
                None if self.last_stop_duration is None
                else round(self.last_stop_duration, 3)
            ),
            "linear_brake_events": self.linear_brake_events,
            "linear_brake_rate_per_minute": round(brake_rate_per_minute, 3),
            "speed_modulation_events": self.speed_modulation_events,
            "brake_reason_counts": dict(self.brake_reason_counts),
            "stop_reason_counts": dict(self.stop_reason_counts),
            "goal_change_rate_per_minute": round(goal_change_rate_per_minute, 3),
            "goal_transition_counts": {
                "prefix_continuation": self.continuous_goal_transitions,
                "endpoint_divergence": self.divergent_goal_transitions,
                "terminal_prefetched_successor": self.terminal_goal_transitions,
                "unknown": self.unknown_goal_transitions,
            },
            "goal_transition_brake_events": dict(self.transition_brake_events),
            "persistent_plan_received": self.persistent_plan_received,
            "persistent_plan_equivalent_retained": (
                self.persistent_plan_equivalent_retained
            ),
            "persistent_plan_installed": self.persistent_plan_installed,
            "persistent_plan_last_event": self.persistent_plan_last_event,
            "persistent_plan_route_version": self.persistent_plan_route_version,
            "persistent_plan_geometry_hash": self.persistent_plan_geometry_hash,
            "turn_only_events": self.turn_only_events,
            "turn_only_duration_seconds": round(self.turn_only_duration_total, 3),
            "safety_override_events": self.safety_override_events,
            "safety_intervention_samples": self.safety_intervention_samples,
            "hard_stop_events": self.hard_stop_events,
            "controller_action_delta": round(self.controller_action_delta, 4),
            "controller_status_changes": self.controller_status_changes,
            "teb_bridge_events": self.bridge_events,
            "teb_bridge_deferred_goal_updates": self.bridge_deferred_goal_updates,
            "teb_bridge_dispatches": self.bridge_dispatches,
            "teb_bridge_terminal_events": self.bridge_terminal_events,
            "teb_bridge_endpoint_detections": self.bridge_endpoint_detections,
            "teb_bridge_successor_dispatches": self.bridge_successor_dispatches,
            "teb_bridge_priority_handoffs": self.bridge_priority_handoffs,
            "teb_bridge_target_retries": self.bridge_target_retries,
            "teb_bridge_target_segment_handoffs": self.bridge_target_segment_handoffs,
            "teb_bridge_frontier_segment_handoffs": self.bridge_frontier_segment_handoffs,
            "teb_bridge_frontier_observation_completions": self.bridge_frontier_observation_completions,
            "teb_bridge_frontier_terminal_settle_completions": (
                self.bridge_frontier_terminal_settle_completions
            ),
            "teb_bridge_frontier_continuous_prefetch_handoffs": self.bridge_frontier_continuous_prefetch_handoffs,
            "teb_bridge_frontier_continuous_prefetch_fallbacks": self.bridge_frontier_continuous_prefetch_fallbacks,
            "teb_bridge_frontier_prefetch_requires_turn": self.bridge_frontier_prefetch_requires_turn,
            "teb_bridge_persistent_lookahead_handoffs": self.bridge_persistent_lookahead_handoffs,
            "teb_bridge_persistent_curve_handoffs": self.bridge_persistent_curve_handoffs,
            "teb_bridge_persistent_lookahead_admission_deferred": (
                self.bridge_persistent_lookahead_admission_deferred
            ),
            "teb_bridge_persistent_lookahead_admission_reasons": dict(
                self.bridge_persistent_lookahead_admission_reasons
            ),
            "teb_bridge_frontier_sharp_replacements": self.bridge_frontier_sharp_replacements,
            "teb_bridge_goal_replacements": self.bridge_goal_replacements,
            "teb_bridge_priority_goal_replacements": self.bridge_priority_goal_replacements,
            "teb_bridge_target_goal_replacements": self.bridge_target_goal_replacements,
            "teb_bridge_active": self.bridge_active,
            "teb_bridge_last_event": self.bridge_last_event,
            "teb_bridge_active_intent_source": self.bridge_active_intent_source,
            "teb_bridge_latest_intent_source": self.bridge_latest_intent_source,
            "zero_duration_seconds": round(self.zero_duration_total, 3),
            "detector_messages": self.detector_messages,
            "target_messages": self.target_messages,
            "target_first_seen_seconds": None if self.target_first_seen_ros is None else round(self.target_first_seen_ros - self.start_ros, 3),
            "target_follow_confirmed_seconds": None if self.target_follow_confirmed_ros is None else round(self.target_follow_confirmed_ros - self.start_ros, 3),
            "target_close_confirmation_started_seconds": None if self.target_close_confirmation_started_ros is None else round(self.target_close_confirmation_started_ros - self.start_ros, 3),
            "target_close_confirmed_seconds": None if self.target_close_confirmed_ros is None else round(self.target_close_confirmed_ros - self.start_ros, 3),
            "legacy_state_locked_seconds": None if self.legacy_state_locked_ros is None else round(self.legacy_state_locked_ros - self.start_ros, 3),
            "target_lock_seconds": None if self.target_lock_ros is None else round(self.target_lock_ros - self.start_ros, 3),
            "task_done_seconds": None if self.task_done_ros is None else round(self.task_done_ros - self.start_ros, 3),
            "task_done_completion_evidence": self.task_done_completion_evidence,
            "task_done_policy": (
                (self.run_context.get("target_thresholds") or {}).get("done_policy")
                if isinstance(getattr(self, "run_context", None), dict)
                else None
            ),
            "target_lifecycle_sessions_observed": len(self.target_lifecycle_sessions),
            "target_lifecycle_completed_session": (
                None
                if self.target_lifecycle_last_completed_key is None
                else list(self.target_lifecycle_last_completed_key)
            ),
            "target_segments_committed": self.target_segments_committed,
            "target_continuous_handoffs_prepared": self.target_continuous_handoffs_prepared,
            "target_goal_changes": self.target_goal_changes,
            "target_route_accepts": self.target_route_accepts,
            "target_route_rejections": self.target_route_rejections,
            "target_route_deferrals": self.target_route_deferrals,
            "target_route_holds": self.target_route_holds,
            "target_route_semantic_replans": self.target_route_semantic_replans,
            "target_route_failures": self.target_route_failures,
            "target_route_releases": self.target_route_releases,
            "target_approach_terminals": self.target_approach_terminals,
            "min_scan_clearance": None if not math.isfinite(self.min_clearance) else round(self.min_clearance, 4),
            "discontinuity_obstacle_clearance": round(
                self.discontinuity_obstacle_clearance, 4
            ),
            "target_geometric_evaluation": self._target_eval_snapshot_locked(),
            "map": self.map_stats,
            "benchmark_coverage": self.benchmark_coverage,
            "benchmark_collision_truth": self.benchmark_collision_truth,
        }

    def on_sample(self, _event):
        with self.lock:
            self.sample_count += 1
            self._write("INFO", "sample", sample=self.sample_count, **self._snapshot())

    def close(self):
        with self.lock:
            # The benchmark launcher records an explicit terminal reason before
            # asking ROS to kill this observer.  Promote a timed-out final
            # state before the normal force-close so no-active routes are not
            # downgraded to a runner-only trial diagnostic.
            termination_reason = ""
            try:
                termination_reason = str(
                    rospy.get_param(
                        "/lste_navigation_metrics/termination_reason", ""
                    )
                    or ""
                ).strip()
            except Exception:
                termination_reason = ""
            if termination_reason in {
                "trial_timeout",
                "slice_timeout",
                "diagnostic_slice_timeout",
            }:
                self._close_boundary_failure_locked(
                    {
                        "event": "trial_boundary",
                        "reason": termination_reason,
                        "termination_reason": termination_reason,
                    },
                    sample=self._failure_append_sample_locked(time.monotonic()),
                    force=True,
                )
            # Flush an active episode before the compact run summary closes;
            # otherwise a process killed during the post-window would lose the
            # only correlated evidence for its failure.
            self._finish_failure_episode_locked("run_stop", force=True)
            self._target_eval_end_episode_locked("run_stop")
            self._target_eval_end_depth_episode_locked("run_stop")
            if self.navigation_hold_start_wall is not None:
                self.navigation_hold_duration_total += max(
                    0.0, time.monotonic() - self.navigation_hold_start_wall
                )
                self.navigation_hold_start_wall = None
            if self.zero_start_wall is not None:
                duration = time.monotonic() - self.zero_start_wall
                self.zero_duration_total += duration
                self.stop_duration_count += 1
                self.last_stop_duration = duration
                self.max_stop_duration = max(self.max_stop_duration, duration)
                self.zero_start_wall = None
            if self.turn_only_start_wall is not None:
                self.turn_only_duration_total += time.monotonic() - self.turn_only_start_wall
                self.turn_only_start_wall = None
            self._write(
                "INFO",
                "run_stop",
                wall_duration_seconds=round(time.monotonic() - self.start_wall, 3),
                summary=self._snapshot(),
                status_counts=self.status_counts,
            )
            try:
                self.stream.close()
            except (AttributeError, ValueError):
                pass
            self._close_failure_evidence_log()
