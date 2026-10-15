"""Turn-phase lifecycle for the TEB execution adapter."""

import json
import math

import rospy
from std_msgs.msg import String

from clock_provider import now_for

from teb_turn_supervisor_contract import (
    STATE_PASS_THROUGH,
    STATE_TURNING,
    TURN_ROUTE_KIND,
    normalize_angle,
)


class TebTurnSupervisorTurnLifecycleMixin:
    """Own the explicit in-place turn transaction, from admission to release."""

    def publish_status_locked(self, event, **fields):
        now = now_for(self)
        pose = self.pose
        error = None
        if pose is not None and self.turn_target_yaw is not None:
            error = normalize_angle(self.turn_target_yaw - pose.theta)
        route_id = int(
            getattr(self, "active_action_route_id", 0) or 0
        ) or int(getattr(self, "latest_route_id", 0) or 0)
        action_generation = int(
            getattr(self, "active_action_generation", 0) or 0
        )
        transaction_id = int(
            getattr(self, "active_action_goal_transaction_id", 0) or 0
        )
        lifecycle_transaction_id = int(
            self._lifecycle_transaction_id_locked()
            if callable(getattr(self, "_lifecycle_transaction_id_locked", None))
            else getattr(
                getattr(self, "lifecycle_manager", None),
                "current_transaction_id",
                0,
            )
            or 0
        )
        graph_transaction_id = int(
            getattr(self, "active_action_graph_transaction_id", 0) or 0
        ) or int(getattr(self, "latest_graph_transaction_id", 0) or 0)
        map_epoch = getattr(self, "active_action_map_epoch", None)
        if map_epoch is None:
            map_epoch = getattr(self, "latest_map_epoch", None)
        origin_epoch = getattr(self, "active_action_origin_epoch", None)
        if origin_epoch is None:
            origin_epoch = getattr(self, "latest_origin_epoch", None)
        projection_epoch = getattr(self, "active_action_projection_epoch", None)
        if projection_epoch is None:
            projection_epoch = getattr(self, "latest_projection_epoch", None)
        output = getattr(self, "last_output_command", None)
        output_linear = float(
            getattr(getattr(output, "linear", None), "x", 0.0) or 0.0
        )
        output_angular = float(
            getattr(getattr(output, "angular", None), "z", 0.0) or 0.0
        )
        payload = {
            "event": str(event),
            "route_id": route_id,
            "action_generation": action_generation,
            "transaction_id": transaction_id,
            "lifecycle_transaction_id": lifecycle_transaction_id,
            "graph_transaction_id": graph_transaction_id,
            "map_epoch": map_epoch,
            "origin_epoch": origin_epoch,
            "projection_epoch": projection_epoch,
            "dedupe_key": (
                [route_id, action_generation]
                if action_generation > 0 else None
            ),
            "state": self.state,
            "mode": self.mode,
            "route_kind": self.latest_route_kind,
            "active_action": bool(self.active_action),
            "active_action_transaction_id": int(
                getattr(self, "active_action_transaction_id", 0) or 0
            ),
            "active_action_lifecycle_transaction_id": int(
                getattr(self, "active_action_lifecycle_transaction_id", 0)
                or 0
            ),
            "active_action_goal_transaction_id": int(
                getattr(self, "active_action_goal_transaction_id", 0) or 0
            ),
            "active_action_generation": int(
                getattr(self, "active_action_generation", 0) or 0
            ),
            "navigation_hold": bool(self.navigation_hold),
            "active_route_kind": self.active_action_route_kind,
            "active_route_id": int(
                getattr(self, "active_action_route_id", 0) or 0
            ),
            "turn_phase": str(getattr(self, "turn_phase", "") or ""),
            "active_map_epoch": getattr(
                self, "active_action_map_epoch", None
            ),
            "active_origin_epoch": getattr(
                self, "active_action_origin_epoch", None
            ),
            "active_projection_epoch": getattr(
                self, "active_action_projection_epoch", None
            ),
            "active_graph_transaction_id": int(
                getattr(self, "active_action_graph_transaction_id", 0) or 0
            ),
            "intent_source": self.latest_intent_source,
            "intent_priority": int(self.latest_intent_priority),
            "goal": None
            if self.turn_goal_xy is None
            else [round(value, 3) for value in self.turn_goal_xy],
            "target_yaw": None
            if self.turn_target_yaw is None
            else round(float(self.turn_target_yaw), 4),
            "pose": None
            if pose is None
            else [
                round(float(pose.x), 3),
                round(float(pose.y), 3),
                round(float(pose.theta), 4),
            ],
            "yaw_error": None if error is None else round(float(error), 4),
            "turn_velocity": round(float(self.turn_velocity), 4),
            "scan_minimum": (
                None if not math.isfinite(self.scan_minimum)
                else round(float(self.scan_minimum), 4)
            ),
            "turn_min_clearance": round(float(self.turn_min_clearance), 4),
            "planner_command": [
                round(float(self.latest_planner_command.linear.x), 4),
                round(float(self.latest_planner_command.angular.z), 4),
            ],
            "planner_command_age_seconds": (
                None
                if self.latest_planner_command_wall <= 0.0
                else round(
                    max(0.0, now - self.latest_planner_command_wall), 4
                )
            ),
            "planner_command_sequence": int(
                getattr(self, "planner_command_sequence", 0) or 0
            ),
            "planner_command_transaction_id": int(
                getattr(self, "planner_command_transaction_id", 0) or 0
            ),
            "planner_command_route_id": int(
                getattr(self, "planner_command_route_id", 0) or 0
            ),
            "planner_command_map_epoch": getattr(
                self, "planner_command_map_epoch", None
            ),
            "planner_command_origin_epoch": getattr(
                self, "planner_command_origin_epoch", None
            ),
            "planner_command_projection_epoch": getattr(
                self, "planner_command_projection_epoch", None
            ),
            "planner_command_graph_transaction_id": int(
                getattr(self, "planner_command_graph_transaction_id", 0) or 0
            ),
            "planner_command_action_generation": int(
                getattr(self, "planner_command_action_generation", 0) or 0
            ),
            "planner_contract_sequence": int(
                getattr(self, "planner_contract_sequence", 0) or 0
            ),
            "planner_contract_state": int(
                getattr(self, "planner_contract_state", 0) or 0
            ),
            "planner_contract_valid": bool(
                getattr(self, "planner_contract_valid", False)
            ),
            "planner_contract_identity": getattr(
                self, "planner_contract_identity", None
            ),
            "planner_contract_pending": (
                None
                if not isinstance(
                    getattr(self, "planner_contract_pending", None), dict
                )
                else {
                    "identity": getattr(
                        self, "planner_contract_pending", {}
                    ).get("identity"),
                    "state": int(
                        getattr(self, "planner_contract_pending", {}).get(
                            "state", 0
                        )
                        or 0
                    ),
                    "age_seconds": round(
                        max(
                            0.0,
                            now
                            - float(
                                getattr(
                                    self, "planner_contract_pending", {}
                                ).get("received_wall", now)
                            ),
                        ),
                        3,
                    ),
                }
            ),
            "trajectory_feedback_valid": bool(
                getattr(self, "trajectory_feedback_valid", False)
            ),
            "trajectory_feedback_invalid_reason": str(
                getattr(
                    self, "trajectory_feedback_invalid_reason", "no_feedback"
                )
                or "no_feedback"
            ),
            "trajectory_feedback_identity": getattr(
                self, "trajectory_feedback_identity", None
            ),
            "trajectory_feedback_invalidation_count": int(
                getattr(self, "trajectory_feedback_invalidation_count", 0) or 0
            ),
            "output_sequence": int(
                getattr(self, "output_sequence", 0) or 0
            ),
            "output_transaction_id": int(
                getattr(self, "last_output_transaction_id", 0) or 0
            ),
            "output_decision": str(
                getattr(self, "last_output_decision", "unknown")
                or "unknown"
            ),
            # ``planner_command`` below is an input cache. Keep the actual
            # command emitted by this timer separately so failure snapshots do
            # not mistake TEB input for Supervisor output.
            "output_command": [
                round(output_linear, 4), round(output_angular, 4)
            ],
            "output_command_age_seconds": (
                None
                if float(getattr(self, "last_output_wall", 0.0) or 0.0) <= 0.0
                else round(
                    max(
                        0.0,
                        now - float(getattr(self, "last_output_wall", 0.0)),
                    ),
                    4,
                )
            ),
            "turns": int(self.turn_count),
            "completed_turns": int(self.turn_completed_count),
            "released_turns": int(self.turn_released_count),
            "turn_abs_rotation": round(
                float(getattr(self, "turn_abs_rotation", 0.0) or 0.0), 4
            ),
            "turn_rotation_cap": round(
                float(getattr(self, "turn_rotation_cap", 0.0) or 0.0), 4
            ),
            "turn_odom_progress_age_seconds": (
                None
                if float(
                    getattr(self, "turn_last_odom_progress_wall", 0.0) or 0.0
                ) <= 0.0
                else round(
                    max(
                        0.0,
                        now
                        - float(
                            getattr(
                                self, "turn_last_odom_progress_wall", 0.0
                            )
                        ),
                    ),
                    4,
                )
            ),
            "turn_no_progress_timeout_seconds": round(
                float(getattr(self, "turn_no_progress_timeout", 0.0) or 0.0),
                4,
            ),
            "turn_odom_sample_count": int(
                getattr(self, "turn_odom_sample_count", 0) or 0
            ),
        }
        payload.update(fields)
        try:
            self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        except (TypeError, ValueError):
            rospy.logwarn_throttle(5.0, "TEB turn supervisor status serialization failed")

    def _observe_turn_odom_pose_locked(self, pose, now):
        """Accumulate actual yaw progress for the current turn contract."""
        if (
            self.state != STATE_TURNING
            or self.turn_target_yaw is None
            or self.turn_action_identity is None
            or self.turn_action_identity
            != getattr(self, "active_action_identity", None)
        ):
            return
        try:
            theta = float(pose.theta)
        except (AttributeError, TypeError, ValueError):
            return
        previous_theta = getattr(self, "turn_last_odom_theta", None)
        self.turn_last_odom_sample_wall = float(now)
        self.turn_odom_sample_count = int(
            getattr(self, "turn_odom_sample_count", 0) or 0
        ) + 1
        if previous_theta is None:
            self.turn_last_odom_theta = theta
            if self.turn_last_odom_progress_wall <= 0.0:
                self.turn_last_odom_progress_wall = float(now)
            return
        delta = abs(normalize_angle(theta - float(previous_theta)))
        self.turn_last_odom_theta = theta
        # Keep the swept-angle budget faithful to odom while ignoring only
        # numerical noise.  A larger delta also refreshes the no-progress
        # clock, which is the health signal used for a physically blocked turn.
        if delta > 1.0e-4:
            self.turn_abs_rotation += delta
        if delta >= max(
            0.005,
            float(getattr(self, "turn_odom_progress_epsilon", 0.01) or 0.01),
        ):
            self.turn_last_odom_progress_wall = float(now)

    def _release_turn_locked(self, reason, completed=False):
        previous_odom_rotation = float(
            getattr(self, "turn_abs_rotation", 0.0) or 0.0
        )
        previous_odom_progress_age = None
        last_progress = float(
            getattr(self, "turn_last_odom_progress_wall", 0.0) or 0.0
        )
        if last_progress > 0.0:
            previous_odom_progress_age = max(0.0, now_for(self) - last_progress)
        previous_odom_samples = int(
            getattr(self, "turn_odom_sample_count", 0) or 0
        )
        if self.state != STATE_TURNING:
            self.turn_target_yaw = None
            self.turn_phase = ""
            self.turn_goal_xy = None
            self.turn_velocity = 0.0
            self.turn_last_odom_theta = None
            self.turn_last_odom_sample_wall = 0.0
            self.turn_last_odom_progress_wall = 0.0
            self.turn_odom_sample_count = 0
            return
        previous_key = self.turn_key
        previous_phase = str(getattr(self, "turn_phase", "") or "")
        previous_action_identity = self.turn_action_identity
        previous_error = None
        if self.pose is not None and self.turn_target_yaw is not None:
            previous_error = normalize_angle(self.turn_target_yaw - self.pose.theta)
        if completed:
            self.completed_turn_key = previous_key
            if (
                previous_phase == "stalled_route_reorientation"
                and previous_action_identity is not None
            ):
                # Do not suppress a second reorientation before the first one
                # has actually reached its stricter completion boundary.
                self.stalled_route_completed_identity = (
                    previous_action_identity
                )
            self.turn_completed_count += 1
            self.turn_settle_until_wall = (
                now_for(self) + self.turn_settle_duration
                if self.turn_settle_duration > 0.0 else 0.0
            )
            self.turn_settle_yaw = (
                self.pose.theta if self.pose is not None else self.turn_target_yaw
            )
        else:
            self.turn_settle_until_wall = 0.0
            self.turn_settle_yaw = None
        self.turn_released_count += 1
        self.state = STATE_PASS_THROUGH
        self.turn_target_yaw = None
        self.turn_phase = ""
        self.turn_goal_xy = None
        self.turn_key = None
        self.turn_action_identity = None
        self.turn_velocity = 0.0
        self.turn_last_odom_theta = None
        self.turn_last_odom_sample_wall = 0.0
        self.turn_last_odom_progress_wall = 0.0
        self.turn_odom_sample_count = 0
        self.publish_status_locked(
            "turn_completed" if completed else "turn_released",
            reason=str(reason),
            previous_key=previous_key,
            previous_yaw_error=(
                None if previous_error is None else round(float(previous_error), 4)
            ),
            odom_abs_rotation=round(previous_odom_rotation, 4),
            odom_progress_age_seconds=(
                None
                if previous_odom_progress_age is None
                else round(previous_odom_progress_age, 4)
            ),
            odom_sample_count=previous_odom_samples,
            no_progress_timeout_seconds=round(
                float(getattr(self, "turn_no_progress_timeout", 0.0) or 0.0),
                4,
            ),
        )
        rospy.loginfo(
            "TEB turn supervisor released turn: reason=%s completed=%s",
            reason,
            completed,
        )

    def _turn_clearance_reason_locked(self, now):
        """Return why the adapter must not command an in-place rotation."""
        if self.scan_monotonic <= 0.0:
            return "scan_unavailable"
        if now - self.scan_monotonic > self.turn_scan_timeout:
            return "scan_stale"
        if self.scan_minimum < self.turn_min_clearance:
            return "obstacle_inside_circular_footprint"
        return ""

    def _stalled_route_reorientation_locked(self, action_identity):
        """Return a Navfn heading only for a persistent clear-path TEB stall."""
        if (
            not self.stalled_route_reorientation_enabled
            or not (
                self._is_frontier_endpoint_action_locked()
                or self._is_portal_transition_action_locked()
            )
            or action_identity is None
            or action_identity == self.stalled_route_completed_identity
            or action_identity == getattr(
                self, "stalled_route_reorientation_attempted_identity", None
            )
            or self.pose is None
        ):
            self.stalled_route_candidate_identity = None
            self.stalled_route_candidate_since_wall = 0.0
            self.stalled_route_ready_identity = None
            return None
        goal_distance = self._goal_distance_in_pose_frame_locked(
            self.active_action_goal
        )
        feedback_age = max(0.0, now_for(self) - self.latest_trajectory_command_wall)
        feedback_timeout = self._trajectory_feedback_timeout_locked()
        linear = float(self.latest_trajectory_command.linear.x)
        angular = float(self.latest_trajectory_command.angular.z)
        planner_linear = float(self.latest_planner_command.linear.x)
        planner_command_age = (
            None
            if self.latest_planner_command_wall <= 0.0
            else max(0.0, now_for(self) - self.latest_planner_command_wall)
        )
        actuator_clamp = getattr(
            self, "_current_forward_only_clamp_locked", lambda _identity: None
        )(action_identity)
        target_yaw = self._initial_navfn_yaw_in_pose_frame_locked()
        heading_error = (
            None if target_yaw is None
            else normalize_angle(target_yaw - self.pose.theta)
        )
        # With forward-only execution TEB may emit its small internal reverse
        # entry sample while it is already close to an endpoint.  The mux
        # removes that linear component, leaving only a weak angular command;
        # the old one-metre gate then let the global watchdog cancel a valid
        # route before the base could acquire the Navfn tangent.  Treat that
        # explicit reverse-entry signal as a short-route reorientation case.
        # It still requires a fresh Navfn tangent, a meaningful heading error,
        # and the normal one-shot action identity guard below. No reverse
        # velocity is ever forwarded to the actuator.
        reverse_entry = (
            (
                linear < -0.002
                and abs(linear) <= self.stalled_route_reorientation_linear
            )
            or (
                planner_linear < -0.002
                and abs(planner_linear)
                <= self.stalled_route_reorientation_linear
            )
            or actuator_clamp is not None
        )
        feedback_current = bool(
            getattr(
                self,
                "_trajectory_feedback_is_current_locked",
                lambda _identity: True,
            )(action_identity)
        )
        planner_current = bool(
            self.latest_planner_command_wall > 0.0
            and planner_command_age is not None
            and planner_command_age <= self.planner_command_timeout
        )
        # FeedbackMsg has no route identity and can still contain the previous
        # trajectory while a new planner contract is crossing the boundary.
        # A fresh, identity-bearing reverse-entry contract is stronger evidence
        # for this one bounded turn than that stale feedback.  This keeps the
        # mux's forward-only safety rule intact while preventing the supervisor
        # from forwarding a command it already knows the actuator will clamp.
        feedback_ready = bool(
            (feedback_current and feedback_age <= feedback_timeout)
            or (reverse_entry and planner_current)
            or actuator_clamp is not None
        )
        is_near_zero = bool(
            reverse_entry
            or (
                abs(linear) <= self.stalled_route_reorientation_linear
                and abs(angular) <= self.stalled_route_reorientation_angular
            )
        )
        minimum_goal_distance = (
            0.0 if reverse_entry
            else self.stalled_route_reorientation_min_distance
        )
        # Once the mux has already rejected the reverse component, a small
        # remaining heading error is still a real control-boundary mismatch:
        # TEB is asking for an entry sample that cannot produce translation.
        # Let the existing bounded turn transaction consume that error instead
        # of leaving the route in a long-lived near-zero command state.  The
        # turn still requires a fresh identity-bearing planner command, fresh
        # scan clearance, and the rotation cap below; this does not synthesize
        # or forward a reverse velocity.
        minimum_heading_error = (
            0.0 if reverse_entry
            else self.stalled_route_reorientation_heading
        )
        eligible = (
            goal_distance is not None
            and goal_distance >= minimum_goal_distance
            and feedback_ready
            and is_near_zero
            and heading_error is not None
            and abs(heading_error) >= minimum_heading_error
        )
        if not eligible:
            self.stalled_route_candidate_identity = None
            self.stalled_route_candidate_since_wall = 0.0
            self.stalled_route_ready_identity = None
            return None
        now = now_for(self)
        if self.stalled_route_candidate_identity != action_identity:
            self.stalled_route_candidate_identity = action_identity
            self.stalled_route_candidate_since_wall = now
            return None
        if now - self.stalled_route_candidate_since_wall < self.stalled_route_reorientation_delay:
            return None
        if self.stalled_route_ready_identity != action_identity:
            self.stalled_route_ready_identity = action_identity
            self.publish_status_locked(
                "stalled_route_reorientation_ready",
                action_goal_distance=round(float(goal_distance), 4),
                feedback_age_seconds=round(float(feedback_age), 4),
                selected_command=[round(linear, 4), round(angular, 4)],
                planner_command=[round(planner_linear, 4), round(
                    float(self.latest_planner_command.angular.z), 4
                )],
                planner_command_age_seconds=(
                    None
                    if planner_command_age is None
                    else round(float(planner_command_age), 4)
                ),
                feedback_current=feedback_current,
                heading_error_deg=round(math.degrees(heading_error), 2),
                actuator_boundary=(
                    None
                    if actuator_clamp is None
                    else dict(actuator_clamp)
                ),
                reason=(
                    "forward_only_actuator_clamp"
                    if actuator_clamp is not None
                    else (
                        "forward_only_reverse_entry"
                        if reverse_entry else "near_zero_route_command"
                    )
                ),
            )
            rospy.logwarn(
                "TEB selected a persistent near-zero route command; rotating once "
                "toward Navfn tangent: endpoint_distance=%.2fm heading_error=%.1fdeg "
                "selected=(%.3f,%.3f)",
                goal_distance,
                math.degrees(heading_error),
                linear,
                angular,
            )
        return target_yaw, heading_error

    def _activate_turn_locked(self):
        if (
            self.mode != self.active_mode
            or self.task_done
            or self.navigation_hold
            or not self._is_managed_action_locked()
            or self.active_action_goal is None
            or not self._intent_matches_goal_locked(
                self.active_action_source_goal or self.active_action_goal
            )
        ):
            return False
        endpoint_alignment = (
            self.endpoint_alignment_enabled
            and self._is_frontier_endpoint_action_locked()
        )
        portal_alignment = self._is_portal_transition_action_locked()
        local_egress_alignment = self._is_local_egress_action_locked()
        action_identity = self.active_action_identity or self._active_action_key_locked()
        target_yaw = None
        heading_error = None
        turn_phase = None
        if (
            (endpoint_alignment or portal_alignment)
            and self.pre_turn_checked_identity != action_identity
        ):
            target_yaw = self._initial_navfn_yaw_in_pose_frame_locked()
            if target_yaw is None or self.pose is None:
                return False
            heading_error = normalize_angle(target_yaw - self.pose.theta)
            if abs(heading_error) <= self.pre_route_turn_threshold:
                self.pre_turn_checked_identity = action_identity
                target_yaw = None
            else:
                turn_phase = (
                    "portal_entry_alignment"
                    if portal_alignment else "pre_route_alignment"
                )
        if local_egress_alignment and self.pre_turn_checked_identity != action_identity:
            # A recovery anchor may lie behind the base.  Its goal orientation
            # is the route's own egress intent, so complete the turn before a
            # forward-only mux can discard TEB's tiny reverse entry sample.
            target_yaw = self._goal_yaw_in_pose_frame_locked(
                self.active_action_goal
            )
            if target_yaw is None or self.pose is None:
                return False
            heading_error = normalize_angle(target_yaw - self.pose.theta)
            if abs(heading_error) <= self.yaw_goal_tolerance:
                self.pre_turn_checked_identity = action_identity
                target_yaw = None
            else:
                turn_phase = "local_egress_alignment"
        if target_yaw is None and self.active_action_route_kind == TURN_ROUTE_KIND:
            target_yaw = self._goal_yaw_in_pose_frame_locked(self.active_action_goal)
            if target_yaw is None:
                return False
            turn_phase = "legacy_connector"
        if target_yaw is None and (
            self._is_frontier_endpoint_action_locked()
            or self._is_portal_transition_action_locked()
        ):
            stalled_alignment = self._stalled_route_reorientation_locked(
                action_identity
            )
            if stalled_alignment is not None:
                target_yaw, heading_error = stalled_alignment
                turn_phase = "stalled_route_reorientation"
        if target_yaw is None:
            return False
        key = self._turn_key_for_goal_locked(self.active_action_goal, target_yaw)
        if key is None or key == self.completed_turn_key:
            return False
        if self._completed_turn_matches_locked(
            self.active_action_goal, target_yaw
        ):
            return False
        if self.state == STATE_TURNING:
            # Navfn may refine the tangent while the same persistent action is
            # rotating. Keep the current turn transaction; replacing it from
            # every tangent update creates repeated in-place spins.
            if (
                self.turn_action_identity == action_identity
                and self.turn_key is not None
            ):
                return True
            return False
        now = now_for(self)
        if self.turn_pending_identity != key:
            self.turn_pending_identity = key
            self.turn_pending_since_wall = now
            return False
        confirm_duration = (
            0.0
            if turn_phase in ("legacy_connector", "stalled_route_reorientation")
            else self.turn_start_confirm_duration
        )
        if now - self.turn_pending_since_wall < confirm_duration:
            return False
        clearance_reason = self._turn_clearance_reason_locked(now)
        if clearance_reason:
            if endpoint_alignment or portal_alignment or local_egress_alignment:
                self.pre_turn_checked_identity = action_identity
            self.completed_turn_key = key
            self.turn_pending_identity = None
            self.turn_pending_since_wall = 0.0
            self.publish_status_locked(
                "turn_skipped",
                reason=clearance_reason,
                scan_minimum=(
                    None if not math.isfinite(self.scan_minimum)
                    else round(float(self.scan_minimum), 4)
                ),
            )
            rospy.logwarn(
                "TEB turn supervisor skipped unsafe in-place turn: reason=%s "
                "scan_min=%s required=%.2fm",
                clearance_reason,
                "n/a" if not math.isfinite(self.scan_minimum)
                else "%.2f" % self.scan_minimum,
                self.turn_min_clearance,
            )
            return False
        if endpoint_alignment and turn_phase == "pre_route_alignment":
            self.pre_turn_checked_identity = action_identity
        self.state = STATE_TURNING
        self.turn_key = key
        self.turn_action_identity = action_identity
        if turn_phase == "stalled_route_reorientation":
            self.stalled_route_reorientation_attempted_identity = action_identity
        self.turn_target_yaw = target_yaw
        self.turn_phase = str(turn_phase or "")
        self.turn_goal_xy = self._goal_xy(self.active_action_goal)
        self.turn_velocity = 0.0
        self.turn_started_wall = now
        if heading_error is not None:
            initial_abs_error = abs(heading_error)
        else:
            initial_abs_error = (
                abs(normalize_angle(target_yaw - self.pose.theta))
                if self.pose is not None else math.pi
            )
        self.turn_initial_abs_error = initial_abs_error
        self.turn_abs_rotation = 0.0
        self.turn_rotation_cap = max(
            initial_abs_error * self.turn_rotation_cap_factor,
            math.radians(120.0),
        )
        self.turn_odom_progress_epsilon = max(
            0.005, min(0.02, 0.10 * float(self.yaw_goal_tolerance))
        )
        self.turn_last_odom_theta = (
            float(self.pose.theta) if self.pose is not None else None
        )
        self.turn_last_odom_sample_wall = now
        self.turn_last_odom_progress_wall = now
        self.turn_odom_sample_count = 0
        # This is derived from the already configured angular budget.  It is
        # a liveness bound, not a second velocity parameter: if no odom yaw
        # progress arrives within the time needed to consume the allowed
        # swept-angle budget, release the turn to the route failure FSM.
        max_vel_theta = max(
            float(getattr(self, "max_vel_theta", 0.65) or 0.65),
            0.05,
        )
        self.turn_no_progress_timeout = max(
            2.0,
            min(
                12.0,
                self.turn_rotation_cap / max_vel_theta,
            ),
        )
        self.turn_count += 1
        self.publish_status_locked(
            "turn_started",
            source_goal=[round(value, 3) for value in self.turn_goal_xy],
            turn_phase=turn_phase,
            plan_heading=round(float(target_yaw), 4),
        )
        rospy.loginfo(
            "TEB turn supervisor started %s: target=(%.2f,%.2f) yaw=%.1fdeg "
            "source=%s",
            turn_phase.replace("_", " "),
            self.turn_goal_xy[0],
            self.turn_goal_xy[1],
            math.degrees(target_yaw),
            self.latest_intent_source,
        )
        return True

    def _completed_turn_matches_locked(self, goal, target_yaw):
        """Treat small map/yaw updates as the same completed turn.

        Persistent frontier execution republishes an equivalent connector while
        SLAM and Navfn refine its pose. Exact tuple identity made the supervisor
        restart the same in-place turn on every refinement, creating an
        artificial spin loop. Reuse the existing action position epsilon and
        yaw tolerance as the lifecycle identity envelope.
        """
        completed = self.completed_turn_key
        if completed is None or goal is None or target_yaw is None:
            return False
        try:
            goal_xy = self._goal_xy(goal)
            position_delta = math.hypot(
                float(goal_xy[0]) - float(completed[0]),
                float(goal_xy[1]) - float(completed[1]),
            )
            yaw_delta = abs(
                normalize_angle(float(target_yaw) - float(completed[2]))
            )
        except (IndexError, TypeError, ValueError):
            return False
        return (
            position_delta
            <= max(0.05, 0.5 * float(self.turn_min_clearance))
            and yaw_delta <= float(self.yaw_goal_tolerance)
        )
