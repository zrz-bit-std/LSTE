"""Command continuity and timer-driven output for the TEB turn supervisor."""

import copy
import json
import math

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from clock_provider import now_for

from teb_turn_supervisor_contract import (
    STATE_PASS_THROUGH,
    STATE_TURNING,
    normalize_angle,
)


class TebTurnSupervisorControlMixin:
    """Forward TEB commands, with narrowly bounded execution-phase handling."""

    def _recovery_command_is_current_locked(self, now):
        """Return true only for a fresh recovery command on this active route."""
        if not (
            getattr(self, "recovery_active", False)
            and getattr(self, "active_action", False)
            and getattr(self, "recovery_identity", None)
            == getattr(self, "active_action_identity", None)
        ):
            return False
        command_wall = float(
            getattr(self, "latest_recovery_command_wall", 0.0) or 0.0
        )
        if command_wall <= 0.0 or now - command_wall > float(
            getattr(self, "recovery_command_timeout", 0.50)
        ):
            return False
        command = getattr(self, "latest_recovery_command", None)
        if command is None:
            return False
        return (
            abs(float(command.linear.x)) > 0.001
            or abs(float(command.angular.z)) > 0.01
        )

    def _trajectory_feedback_is_current_locked(self, action_identity=None):
        """Check feedback validity without breaking legacy lightweight fixtures."""
        if not hasattr(self, "trajectory_feedback_valid"):
            return True
        if not bool(self.trajectory_feedback_valid):
            return False
        if action_identity is None:
            action_identity = getattr(self, "active_action_identity", None)
        return getattr(self, "trajectory_feedback_identity", None) == action_identity

    def _lifecycle_transaction_id_locked(self):
        if bool(getattr(self, "active_action", False)):
            active_action_lifecycle = int(
                getattr(self, "active_action_lifecycle_transaction_id", 0)
                or getattr(self, "active_action_transaction_id", 0)
                or 0
            )
            if active_action_lifecycle > 0:
                return active_action_lifecycle
        return int(
            getattr(
                getattr(self, "lifecycle_manager", None),
                "current_transaction_id",
                0,
            )
            or 0
        )

    def _planner_command_rejection_reason_locked(self, now):
        """Return the causal reason a raw planner command cannot be forwarded."""
        if getattr(self, "require_planner_command_contract", False):
            if not bool(getattr(self, "planner_contract_valid", False)):
                return "planner_contract_missing_or_invalid"
            if int(
                getattr(self, "planner_contract_state", 0) or 0
            ) != 1:
                return "planner_contract_boundary"
            # During a persistent handoff the old planner frame can remain
            # valid for one or more timer ticks after Bridge has cleared the
            # predecessor action and before it announces the successor as
            # active.  The command must still be zero, but comparing that old
            # frame with the successor lifecycle would misclassify this
            # expected boundary as a stale-identity fault.
            if not bool(getattr(self, "active_action", False)):
                pending = getattr(self, "planner_contract_pending", None)
                active_route = int(
                    getattr(self, "active_action_route_id", 0) or 0
                )
                planner_identity = getattr(
                    self, "planner_contract_identity", None
                )
                planner_route = int(
                    planner_identity.get("route_id", 0) or 0
                ) if isinstance(planner_identity, dict) else 0
                active_generation = int(
                    getattr(self, "active_action_generation", 0) or 0
                )
                planner_generation = int(
                    planner_identity.get("action_generation", 0) or 0
                ) if isinstance(planner_identity, dict) else 0
                if (
                    isinstance(pending, dict)
                    or active_route > planner_route
                    or active_generation > planner_generation
                ):
                    return "bridge_action_alignment_pending"
                return "bridge_action_inactive"
            mismatch = self._planner_contract_mismatch_locked(
                getattr(self, "planner_contract_identity", {})
            )
            if mismatch:
                return mismatch
            if self.planner_contract_wall <= 0.0:
                return "planner_contract_missing"
            if now - self.planner_contract_wall > self.planner_command_timeout:
                return "planner_contract_stale"
        current_transaction_id = self._lifecycle_transaction_id_locked()
        planner_transaction_id = int(
            getattr(self, "planner_command_transaction_id", 0) or 0
        )
        if current_transaction_id <= 0:
            return "no_lifecycle_transaction"
        if self.latest_planner_command_wall <= 0.0:
            return "no_planner_command"
        if now - self.latest_planner_command_wall > self.planner_command_timeout:
            return "planner_command_stale"
        if planner_transaction_id != current_transaction_id:
            return "planner_transaction_mismatch"
        active_transaction_id = int(
            getattr(self, "active_action_transaction_id", 0) or 0
        )
        if active_transaction_id > 0 and active_transaction_id != current_transaction_id:
            return "active_transaction_mismatch"
        active_graph_transaction_id = int(
            getattr(self, "active_action_graph_transaction_id", 0) or 0
        )
        planner_graph_transaction_id = int(
            getattr(self, "planner_command_graph_transaction_id", 0) or 0
        )
        if (
            active_graph_transaction_id > 0
            and planner_graph_transaction_id != active_graph_transaction_id
        ):
            return "planner_graph_transaction_mismatch"
        active_route_id = int(getattr(self, "active_action_route_id", 0) or 0)
        planner_route_id = int(getattr(self, "planner_command_route_id", 0) or 0)
        if active_route_id > 0 and planner_route_id != active_route_id:
            return "planner_route_id_mismatch"
        active_map_epoch = getattr(self, "active_action_map_epoch", None)
        planner_map_epoch = getattr(self, "planner_command_map_epoch", None)
        if (
            active_map_epoch is not None
            and planner_map_epoch != active_map_epoch
        ):
            return "planner_map_epoch_mismatch"
        if not self.active_action and (
            abs(float(self.latest_planner_command.linear.x)) > 0.001
            or abs(float(self.latest_planner_command.angular.z)) > 0.001
        ):
            return "no_active_action"
        if (
            self.active_action_identity is not None
            and self.planner_command_action_identity is not None
            and self.active_action_identity != self.planner_command_action_identity
        ):
            return "planner_route_identity_mismatch"
        return ""

    def _publish_command_contract(self, command, transaction_id, decision):
        """Publish the exact command consumed by the mux with its identity."""
        publisher = getattr(self, "command_contract_pub", None)
        if publisher is None:
            return
        active_map_epoch = getattr(self, "active_action_map_epoch", None)
        planner_map_epoch = getattr(self, "planner_command_map_epoch", None)
        # A bridge terminal/handoff status can clear the supervisor's active
        # cache one callback before the planner command cache is cleared. The
        # planner cache still belongs to the same route identity (validated by
        # ``_planner_identity_error``), so preserve that epoch in the wire
        # contract instead of sending a command the mux must reject as
        # unauthenticated.
        command_map_epoch = (
            active_map_epoch
            if active_map_epoch is not None
            else planner_map_epoch
        )
        # An explicit turn is owned by this adapter, not by the last raw TEB
        # sample. A route boundary normally leaves the planner contract in a
        # ZERO/STALE state, but the turn itself still has to cross the mux as
        # a current, authenticated command. Ordinary pass-through commands
        # retain the planner-produced identity unchanged.
        supervisor_owned_turn = decision in (
            "turn", "turn_settle", "move_base_recovery"
        ) and (
            abs(float(command.linear.x)) > 0.001
            or abs(float(command.angular.z)) > 0.01
        ) and bool(self.active_action)
        effective_planner_transaction_id = int(
            getattr(self, "planner_command_transaction_id", 0) or 0
        )
        effective_planner_route_id = int(
            getattr(self, "planner_command_route_id", 0) or 0
        )
        effective_planner_graph_transaction_id = int(
            getattr(self, "planner_command_graph_transaction_id", 0) or 0
        )
        effective_planner_action_generation = int(
            getattr(self, "planner_command_action_generation", 0) or 0
        )
        effective_planner_map_epoch = planner_map_epoch
        effective_planner_origin_epoch = getattr(
            self, "planner_command_origin_epoch", None
        )
        effective_planner_projection_epoch = getattr(
            self, "planner_command_projection_epoch", None
        )
        if supervisor_owned_turn:
            effective_planner_transaction_id = int(transaction_id)
            effective_planner_route_id = int(
                getattr(self, "active_action_route_id", 0) or 0
            )
            effective_planner_graph_transaction_id = int(
                getattr(self, "active_action_graph_transaction_id", 0) or 0
            )
            effective_planner_action_generation = int(
                getattr(self, "active_action_generation", 0) or 0
            )
            effective_planner_map_epoch = active_map_epoch
            effective_planner_origin_epoch = getattr(
                self, "active_action_origin_epoch", None
            )
            effective_planner_projection_epoch = getattr(
                self, "active_action_projection_epoch", None
            )
        payload = {
            "event": "teb_command",
            "transaction_id": int(transaction_id),
            "lifecycle_transaction_id": int(transaction_id),
            "mission_transaction_id": int(effective_planner_transaction_id),
            "command_sequence": int(getattr(self, "output_sequence", 0) or 0),
            "planner_command_sequence": int(
                getattr(self, "planner_command_sequence", 0) or 0
            ),
            "planner_contract_sequence": int(
                getattr(self, "planner_contract_sequence", 0) or 0
            ),
            "planner_action_generation": int(
                effective_planner_action_generation
            ),
            "active_action_generation": int(
                getattr(self, "active_action_generation", 0) or 0
            ),
            "planner_contract_state": int(
                getattr(self, "planner_contract_state", 0) or 0
            ),
            "planner_contract_valid": bool(
                getattr(self, "planner_contract_valid", False)
            ),
            "planner_transaction_id": int(
                effective_planner_transaction_id
            ),
            "active_action_transaction_id": int(
                getattr(self, "active_action_transaction_id", 0) or 0
            ),
            "active_action_lifecycle_transaction_id": int(
                getattr(self, "active_action_lifecycle_transaction_id", 0)
                or 0
            ),
            "graph_transaction_id": int(
                getattr(self, "active_action_graph_transaction_id", 0) or 0
            ),
            "planner_graph_transaction_id": int(
                effective_planner_graph_transaction_id
            ),
            "active_action_graph_transaction_id": int(
                getattr(self, "active_action_graph_transaction_id", 0) or 0
            ),
            "route_id": int(getattr(self, "active_action_route_id", 0) or 0),
            "planner_route_id": int(
                effective_planner_route_id
            ),
            "active_action_route_id": int(
                getattr(self, "active_action_route_id", 0) or 0
            ),
            "map_epoch": command_map_epoch,
            "planner_map_epoch": effective_planner_map_epoch,
            "origin_epoch": effective_planner_origin_epoch,
            "projection_epoch": effective_planner_projection_epoch,
            "planner_origin_epoch": effective_planner_origin_epoch,
            "planner_projection_epoch": effective_planner_projection_epoch,
            "active_action_map_epoch": getattr(
                self, "active_action_map_epoch", None
            ),
            "active_action_origin_epoch": getattr(
                self, "active_action_origin_epoch", None
            ),
            "active_action_projection_epoch": getattr(
                self, "active_action_projection_epoch", None
            ),
            "active_action": bool(self.active_action),
            "state": str(self.state),
            "decision": str(decision),
            "linear_x": round(float(command.linear.x), 6),
            "angular_z": round(float(command.angular.z), 6),
        }
        publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _trajectory_feedback_timeout_locked(self):
        """Return bounded freshness based on the observed feedback cadence."""
        if self.trajectory_feedback_period_ema is None:
            return self.trajectory_feedback_timeout
        return min(
            self.trajectory_feedback_timeout_cap,
            max(
                self.trajectory_feedback_timeout,
                self.trajectory_feedback_period_scale
                * self.trajectory_feedback_period_ema,
            ),
        )

    def _trajectory_continuity_command_locked(self, now, planner_command_stale=False):
        """Forward one fresh, selected TEB command across a transient raw gap.

        The adapter never creates a velocity.  It can only pass through TEB's
        selected forward sample for a short bounded interval, outside terminal
        and explicit-turn envelopes.
        """
        raw = self.latest_planner_command
        raw_linear_is_zero = abs(float(raw.linear.x)) <= 0.01
        raw_is_zero = raw_linear_is_zero and abs(float(raw.angular.z)) <= 0.01
        raw_is_reverse_only = (
            float(raw.linear.x) < 0.0
            and abs(float(raw.angular.z)) <= 0.01
        )
        # PersistentTebLocalPlanner can return a small reverse/near-zero
        # command for the current control cycle even while the selected TEB
        # trajectory still contains a valid forward sample.  With the
        # forward-only actuator boundary that command becomes zero translation
        # and the route can stall indefinitely.  The selected sample is usable
        # only when it has been re-armed by the current identity-bearing
        # planner contract and is still fresh; it is never synthesized here.
        feedback_age = max(0.0, now - self.latest_trajectory_command_wall)
        feedback_current = self._trajectory_feedback_is_current_locked(
            self.active_action_identity
        )
        selected_forward = bool(
            getattr(self, "trajectory_feedback_valid", False)
            and feedback_current
            and feedback_age <= self._trajectory_feedback_timeout_locked()
            and float(self.latest_trajectory_command.linear.x)
            >= self.trajectory_continuity_min_forward
        )
        forward_only_command_gap = bool(
            raw_linear_is_zero and selected_forward
        )
        transient_gap = bool(
            raw_is_zero
            or raw_is_reverse_only
            or planner_command_stale
            or forward_only_command_gap
        )
        sharp_entry_error = (
            self._sharp_navfn_entry_heading_error_locked()
            if transient_gap else None
        )
        # A confirmed target segment remains owned by TEB's local trajectory;
        # a sharp Navfn entry must not turn a transient planner publish gap
        # into a zero command while the selected target trajectory is valid.
        # Graph-owned frontier/portal routes retain the conservative suppression
        # because their entry tangent is an explicit topology contract.
        if (
            sharp_entry_error is not None
            and not self._is_same_target_segment_continuation_locked()
        ):
            action_identity = self.active_action_identity
            if action_identity != self.trajectory_continuity_sharp_entry_identity:
                self.trajectory_continuity_sharp_entry_identity = action_identity
                self.trajectory_continuity_sharp_entry_suppressions += 1
                self.publish_status_locked(
                    "trajectory_continuity_suppressed",
                    reason="sharp_navfn_entry",
                    entry_heading_error_deg=round(
                        math.degrees(sharp_entry_error), 2
                    ),
                    threshold_deg=90.0,
                    raw_command=[
                        round(float(raw.linear.x), 4),
                        round(float(raw.angular.z), 4),
                    ],
                    count=int(self.trajectory_continuity_sharp_entry_suppressions),
                )
            self.trajectory_zero_started_wall = 0.0
            return None
        goal_distance = self._goal_distance_in_pose_frame_locked(
            self.active_action_goal
        )
        feedback_timeout = self._trajectory_feedback_timeout_locked()
        self.trajectory_continuity_goal_distance = goal_distance
        action_or_same_tier_intent = (
            self._intent_matches_goal_locked(
                self.active_action_source_goal or self.active_action_goal
            )
            or (
                self.latest_intent_source == self.active_action_source
                and self.latest_intent_priority == self.active_action_priority
            )
        )
        if not (
            self.trajectory_continuity_enabled
            and self.state == STATE_PASS_THROUGH
            and self._is_continuity_eligible_action_locked()
            and action_or_same_tier_intent
            and not self.task_done
            and not self.navigation_hold
            and transient_gap
            and self._trajectory_feedback_is_current_locked(
                self.active_action_identity
            )
            and feedback_age <= feedback_timeout
            and float(self.latest_trajectory_command.linear.x)
            >= self.trajectory_continuity_min_forward
            and goal_distance is not None
            and goal_distance > self.trajectory_continuity_terminal_radius
        ):
            self.trajectory_zero_started_wall = 0.0
            return None
        if self.trajectory_zero_started_wall <= 0.0:
            self.trajectory_zero_started_wall = now
        if now - self.trajectory_zero_started_wall > self.trajectory_continuity_max_hold:
            return None
        self.trajectory_continuity_events += 1
        self.publish_status_locked(
            "trajectory_continuity",
            continuity_scope=(
                "target_same_track"
                if self._is_same_target_segment_continuation_locked()
                else "frontier_or_connector"
            ),
            gap_kind=(
                "planner_publish_gap"
                if planner_command_stale
                else (
                    "forward_only_command_gap"
                    if forward_only_command_gap
                    else ("reverse_only" if raw_is_reverse_only else "zero")
                )
            ),
            raw_command=[
                round(float(raw.linear.x), 4),
                round(float(raw.angular.z), 4),
            ],
            selected_command=[
                round(float(self.latest_trajectory_command.linear.x), 4),
                round(float(self.latest_trajectory_command.angular.z), 4),
            ],
            feedback_age_seconds=round(feedback_age, 4),
            feedback_timeout_seconds=round(feedback_timeout, 4),
            feedback_period_seconds=(
                None
                if self.trajectory_feedback_period_ema is None
                else round(float(self.trajectory_feedback_period_ema), 4)
            ),
            action_goal_distance=round(float(goal_distance), 4),
            count=int(self.trajectory_continuity_events),
        )
        return copy.deepcopy(self.latest_trajectory_command)

    def _forward_only_reorientation_command_locked(self, now):
        """Keep a clamped route on its current forward TEB trajectory.

        A forward-only mux must remove TEB's small reverse entry sample, but
        that sample can remain the raw planner command after the route has
        already been aligned. A fresh selected TEB trajectory is the only
        valid continuation signal; do not fall back to the command the
        actuator will necessarily clamp. The override is armed only by a
        completed bounded reorientation or an actual mux clamp, and remains
        scoped to the current action identity.
        """
        action_identity = getattr(self, "active_action_identity", None)
        if action_identity is None:
            return None
        if not (
            self.active_action
            and self.state == STATE_PASS_THROUGH
            and not self.task_done
            and not self.navigation_hold
            and (
                self._is_frontier_endpoint_action_locked()
                or self._is_portal_transition_action_locked()
            )
        ):
            return None
        completed_identity = getattr(
            self, "stalled_route_completed_identity", None
        )
        override_identity = getattr(
            self, "forward_only_trajectory_identity", None
        )
        if override_identity is not None and override_identity != action_identity:
            self.forward_only_trajectory_identity = None
            override_identity = None
        clamp = self._current_forward_only_clamp_locked(action_identity)
        if (
            override_identity != action_identity
            and completed_identity != action_identity
            and clamp is None
        ):
            return None
        raw = self.latest_planner_command
        if float(raw.linear.x) >= -0.002:
            return None
        if not self._trajectory_feedback_is_current_locked(action_identity):
            return None
        feedback_age = max(0.0, now - self.latest_trajectory_command_wall)
        feedback_timeout = self._trajectory_feedback_timeout_locked()
        selected = self.latest_trajectory_command
        if (
            feedback_age > feedback_timeout
            or not bool(getattr(self, "trajectory_feedback_valid", False))
            or float(selected.linear.x) < self.trajectory_continuity_min_forward
        ):
            return None
        goal_distance = self._goal_distance_in_pose_frame_locked(
            self.active_action_goal
        )
        if (
            goal_distance is None
            or goal_distance <= self.trajectory_continuity_terminal_radius
        ):
            return None
        armed = override_identity != action_identity
        self.forward_only_trajectory_identity = action_identity
        if armed:
            self.publish_status_locked(
                "forward_only_reorientation",
                raw_command=[
                    round(float(raw.linear.x), 4),
                    round(float(raw.angular.z), 4),
                ],
                selected_command=[
                    round(float(selected.linear.x), 4),
                    round(float(selected.angular.z), 4),
                ],
                feedback_age_seconds=round(feedback_age, 4),
                feedback_timeout_seconds=round(feedback_timeout, 4),
                action_goal_distance=round(float(goal_distance), 4),
                actuator_clamp=clamp,
                reason=(
                    "mux_clamp_forward_trajectory"
                    if clamp is not None
                    else "completed_reorientation_forward_trajectory"
                ),
            )
        return copy.deepcopy(selected)

    def on_mode(self, message):
        mode = message.data.strip().lower()
        if not mode:
            return
        with self.lock:
            previous = self.mode
            self.mode = mode
            if mode != self.active_mode and self.state == STATE_TURNING:
                invalidate_feedback = getattr(
                    self, "_invalidate_trajectory_feedback_locked", None
                )
                if callable(invalidate_feedback):
                    invalidate_feedback(
                        "controller_mode_changed", clear_planner=True
                    )
                self._release_turn_locked(
                    "controller_switched_to_%s" % mode, completed=False
                )
            elif previous != self.active_mode and mode == self.active_mode:
                self._activate_turn_locked()

    def on_task_done(self, message):
        with self.lock:
            self.task_done = bool(message.data)
            if self.task_done:
                invalidate_feedback = getattr(
                    self, "_invalidate_trajectory_feedback_locked", None
                )
                if callable(invalidate_feedback):
                    invalidate_feedback("task_done", clear_planner=True)
                self._release_turn_locked("task_done", completed=False)
            else:
                self.completed_turn_key = None
                self._activate_turn_locked()

    def _set_navigation_hold_locked(self, active):
        active = bool(active)
        self.navigation_hold = active
        if active:
            invalidate_feedback = getattr(
                self, "_invalidate_trajectory_feedback_locked", None
            )
            if callable(invalidate_feedback):
                invalidate_feedback("navigation_hold", clear_planner=True)
            self._release_turn_locked("navigation_hold", completed=False)
        else:
            self._activate_turn_locked()

    def apply_navigation_hold_sample(self, active):
        """Apply the latest non-transactional actuator gate on the timer tick."""
        with self.lock:
            self._set_navigation_hold_locked(active)

    def on_navigation_hold(self, message):
        with self.lock:
            self._set_navigation_hold_locked(message.data)

    def _turn_command_locked(self, dt):
        """Return one acceleration-limited turn command."""
        if self.pose is None or self.turn_target_yaw is None:
            self.turn_velocity = 0.0
            return Twist()
        error = normalize_angle(self.turn_target_yaw - self.pose.theta)
        # A regular TEB endpoint may legitimately finish inside its configured
        # yaw envelope.  A stalled forward-only route is different: the
        # evidence says TEB is still requesting a reverse entry which the mux
        # must remove.  Close that bounded reorientation against the Navfn
        # tangent itself, otherwise a 14-degree residual can be declared done
        # while the next command remains reverse and the route stalls again.
        completion_tolerance = self.yaw_goal_tolerance
        if getattr(self, "turn_phase", "") == "stalled_route_reorientation":
            completion_tolerance = max(
                0.05, min(self.yaw_goal_tolerance, self.yaw_goal_tolerance * 0.25)
            )
        if abs(error) <= completion_tolerance:
            self._release_turn_locked("yaw_tolerance_reached", completed=True)
            return copy.deepcopy(self.latest_planner_command)

        odom_rotation = float(getattr(self, "turn_abs_rotation", 0.0) or 0.0)
        rotation_cap = float(getattr(self, "turn_rotation_cap", 0.0) or 0.0)
        if rotation_cap > 0.0 and odom_rotation >= rotation_cap:
            self.turn_capped_releases += 1
            rospy.logwarn(
                "TEB turn supervisor reached odom rotation budget: "
                "odom_rotation=%.0fdeg cap=%.0fdeg initial_error=%.0fdeg",
                math.degrees(odom_rotation),
                math.degrees(rotation_cap),
                math.degrees(self.turn_initial_abs_error),
            )
            self._release_turn_locked(
                "odom_rotation_budget_exceeded", completed=False
            )
            return Twist()

        last_progress = float(
            getattr(self, "turn_last_odom_progress_wall", 0.0) or 0.0
        )
        no_progress_timeout = float(
            getattr(self, "turn_no_progress_timeout", 0.0) or 0.0
        )
        if (
            last_progress > 0.0
            and no_progress_timeout > 0.0
            and now_for(self) - last_progress >= no_progress_timeout
        ):
            self.turn_no_progress_releases = int(
                getattr(self, "turn_no_progress_releases", 0) or 0
            ) + 1
            progress_age = now_for(self) - last_progress
            self.publish_status_locked(
                "turn_no_odom_progress",
                reason="odom_yaw_stalled",
                odom_abs_rotation=round(odom_rotation, 4),
                odom_progress_age_seconds=round(progress_age, 4),
                no_progress_timeout_seconds=round(no_progress_timeout, 4),
                no_progress_releases=int(self.turn_no_progress_releases),
            )
            rospy.logwarn(
                "TEB turn supervisor found no odom yaw progress: "
                "age=%.2fs budget=%.0fdeg cap=%.0fdeg",
                progress_age,
                math.degrees(odom_rotation),
                math.degrees(rotation_cap),
            )
            self._release_turn_locked(
                "odom_turn_no_progress", completed=False
            )
            return Twist()

        remaining = max(0.0, abs(error) - completion_tolerance)
        desired = min(
            self.max_vel_theta,
            math.sqrt(max(0.0, 2.0 * self.acc_lim_theta * remaining)),
        )
        requested_sign = 1.0 if error > 0.0 else -1.0
        previous = float(self.turn_velocity)
        step = self.acc_lim_theta * max(0.001, min(0.2, float(dt)))
        if previous != 0.0 and math.copysign(1.0, previous) != requested_sign:
            magnitude = max(0.0, abs(previous) - step)
            self.turn_velocity = (
                math.copysign(magnitude, previous) if magnitude else 0.0
            )
        else:
            previous_magnitude = abs(previous)
            if desired >= previous_magnitude:
                magnitude = min(desired, previous_magnitude + step)
            else:
                magnitude = max(desired, previous_magnitude - step)
            self.turn_velocity = requested_sign * magnitude

        command = Twist()
        command.angular.z = self.turn_velocity
        return command

    def on_timer(self, _event):
        now = now_for(self)
        with self.lock:
            command = Twist()
            decision = "inactive"
            if (
                self.mode == self.active_mode
                and not self.task_done
                and not self.navigation_hold
                and self.state == STATE_TURNING
            ):
                clearance_reason = self._turn_clearance_reason_locked(
                    now
                )
                if clearance_reason:
                    self._release_turn_locked(
                        "turn_clearance_lost_%s" % clearance_reason,
                        completed=False,
                    )
                    command = Twist()
                    decision = "turn_clearance_release"
                else:
                    command = self._turn_command_locked(
                        1.0 / self.command_frequency
                    )
                    decision = "turn"
                state = self.state
            elif (
                self._recovery_command_is_current_locked(now)
            ):
                command = copy.deepcopy(self.latest_recovery_command)
                decision = "move_base_recovery"
                state = self.state
            elif (
                getattr(self, "require_planner_command_contract", False)
                and (
                    not bool(getattr(self, "planner_contract_valid", False))
                    or int(getattr(self, "planner_contract_state", 0) or 0) != 1
                )
            ):
                command = Twist()
                decision = "planner_contract_boundary"
                state = self.state
            elif (
                self.mode == self.active_mode
                and not self.task_done
                and not self.navigation_hold
            ):
                rejection_reason = self._planner_command_rejection_reason_locked(now)
                if rejection_reason:
                    command = Twist()
                    decision = rejection_reason
                else:
                    command = copy.deepcopy(self.latest_planner_command)
                    forward_only_reorientation = (
                        self._forward_only_reorientation_command_locked(now)
                    )
                    continuity = forward_only_reorientation
                    if continuity is None:
                        continuity = self._trajectory_continuity_command_locked(
                            now,
                            planner_command_stale=False,
                        )
                    if continuity is not None:
                        command = continuity
                        decision = (
                            "forward_only_reorientation"
                            if forward_only_reorientation is not None
                            else "trajectory_continuity"
                        )
                    else:
                        decision = "pass_through"
                if (
                    self.turn_settle_until_wall > 0.0
                    and self.pose is not None
                    and self.turn_settle_yaw is not None
                ):
                    if now < self.turn_settle_until_wall:
                        yaw_err = normalize_angle(
                            self.turn_settle_yaw - self.pose.theta
                        )
                        command = copy.deepcopy(command)
                        command.angular.z = max(-0.20, min(0.20, yaw_err * 0.6))
                        decision = "turn_settle"
                    else:
                        self.turn_settle_until_wall = 0.0
                        self.turn_settle_yaw = None
                state = self.state
            else:
                command = Twist()
                self.trajectory_zero_started_wall = 0.0
                state = self.state
                if self.task_done:
                    decision = "task_done"
                elif self.navigation_hold:
                    decision = "navigation_hold"
                elif self.mode != self.active_mode:
                    decision = "controller_mode"
            if (
                state == STATE_PASS_THROUGH
                and self._is_managed_action_locked()
                and not self.task_done
                and not self.navigation_hold
            ):
                self._activate_turn_locked()
            transaction_id = self._lifecycle_transaction_id_locked()
            self.last_output_command = copy.deepcopy(command)
            self.last_output_wall = now
            self.output_sequence += 1
            self.last_output_transaction_id = transaction_id
            self.last_output_decision = decision
        self.output_pub.publish(command)
        self._publish_command_contract(command, transaction_id, decision)
        with self.lock:
            if now - self.last_status_wall >= 1.0:
                self.last_status_wall = now
                self.publish_status_locked(
                    "heartbeat",
                    command=[
                        round(float(command.linear.x), 4),
                        round(float(command.angular.z), 4),
                    ],
                )

    def on_shutdown(self):
        try:
            self.output_pub.publish(Twist())
        except Exception:
            pass
