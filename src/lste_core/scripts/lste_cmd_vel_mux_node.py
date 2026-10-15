#!/usr/bin/env python3
import copy
import json
import math
import threading

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from teb_local_planner.msg import FeedbackMsg


class CmdVelMuxNode:
    def __init__(self):
        rospy.init_node("lste_cmd_vel_mux")
        self.mode = str(rospy.get_param("~initial_mode", "teb")).strip().lower()
        if self.mode not in ("sappo", "teleop", "teb"):
            self.mode = "teb"
        forward_only = rospy.get_param("~teb_forward_only", True)
        self.teb_forward_only = str(forward_only).strip().lower() in (
            "1", "true", "yes", "on",
        )
        self.teb_angular_sign_switch_threshold = max(
            0.0, float(rospy.get_param("~teb_angular_sign_switch_threshold", 0.12))
        )
        self.teb_angular_deadband = max(
            0.0, float(rospy.get_param("~teb_angular_deadband", 0.02))
        )
        self.teb_last_angular_sign = 0
        self.teb_last_output_angular = 0.0
        self.teb_micro_hold_cycles = 0
        # A micro reversal is normally TEB quantization around a stable
        # trajectory.  Holding the previous output for a few control cycles
        # avoids turning that noise into a zero-velocity pulse at a corner.
        self.teb_micro_hold_limit = 4
        # TEB can encode the first point of an in-place turn as a tiny reverse
        # velocity with near-zero angular velocity. Keep the planner's
        # upcoming turn intent as an interface-level handoff hint so the mux
        # does not emit a reverse -> zero -> rotate transition.
        self.teb_turn_hint = 0.0
        self.teb_turn_hint_wall = 0.0
        self.teb_turn_transition_count = 0
        self.teb_filter_events = 0
        self.teb_reverse_clamp_count = 0
        self.teb_command_contract_topic = rospy.get_param(
            "~teb_command_contract_topic", "/lste/cmd_vel/teb_contract"
        )
        self.require_teb_command_contract = str(
            rospy.get_param("~require_teb_command_contract", True)
        ).strip().lower() in ("1", "true", "yes", "on")
        self.teb_contract_last_transaction_id = 0
        self.teb_contract_last_sequence = 0
        self.teb_contract_rejection_count = 0
        self.teb_contract_identity = None
        self.task_done = False
        self.navigation_hold = False
        self.hard_reset_hold = False
        self.last_hard_reset_id = ""
        self.last_hard_reset_transaction_id = 0
        self.hard_reset_identity_history = []
        self.lock = threading.Lock()
        self.output_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # This is an observer contract for the real actuator boundary.  It is
        # intentionally separate from /cmd_vel: consumers can attribute a
        # zero or speed cap to the mux without inferring it from a different
        # lidar sector or a later controller callback.
        self.status_pub = rospy.Publisher(
            rospy.get_param("~status_topic", "/lste/cmd_vel_mux/status"),
            String,
            queue_size=50,
        )
        self.status_sequence = 0
        # Forward-collision velocity governor: a hard safety invariant at the
        # sole /cmd_vel publisher.  Whatever TEB, the goal, or the global path
        # command, the commanded forward speed is capped so the robot can
        # always stop within the measured lidar clearance ahead:
        #     v_max = sqrt(2 * a_brake * max(0, clearance - min_gap))
        # This is not a planner tuning knob; it guarantees the vehicle never
        # dives toward a wall it can already see, independent of the goal or
        # the global plan.
        self.scan_topic = rospy.get_param("~scan_topic", "/pro3/rlscan")
        self.governor_decel = max(
            0.05, float(rospy.get_param("~governor_decel", 0.60))
        )
        self.governor_min_gap = max(
            0.05, float(rospy.get_param("~governor_min_gap", 0.45))
        )
        self.governor_max_speed = max(
            0.0, float(rospy.get_param("~governor_max_speed", 0.50))
        )
        self.scan_forward_min = float("inf")
        self.scan_stamp = None
        self.scan_received_time = None
        self.teb_last_filter_reason = "none"
        self.scan_sub = rospy.Subscriber(
            self.scan_topic, LaserScan, self.on_scan, queue_size=1
        )
        self.sappo_sub = rospy.Subscriber(
            "/lste/cmd_vel/sappo", Twist, self.on_sappo_cmd, queue_size=1
        )
        self.teleop_sub = rospy.Subscriber(
            "/lste/cmd_vel/teleop", Twist, self.on_teleop_cmd, queue_size=1
        )
        self.teb_contract_sub = rospy.Subscriber(
            self.teb_command_contract_topic,
            String,
            self.on_teb_command_contract,
            queue_size=10,
        )
        # Keep the raw callback for compatibility with isolated tools, but do
        # not wire it into the production mux. A bare Twist has no lifecycle
        # identity and cannot safely cross a warm-slice route boundary.
        self.teb_sub = None
        if not self.require_teb_command_contract:
            self.teb_sub = rospy.Subscriber(
                "/lste/cmd_vel/teb", Twist, self.on_teb_cmd, queue_size=1
            )
        self.teb_feedback_sub = rospy.Subscriber(
            "/move_base/TebLocalPlannerROS/teb_feedback",
            FeedbackMsg,
            self.on_teb_feedback,
            queue_size=1,
        )
        self.mode_sub = rospy.Subscriber(
            "/lste/controller_mode", String, self.on_mode, queue_size=1
        )
        self.task_done_sub = rospy.Subscriber(
            "/lste/task_done", Bool, self.on_task_done, queue_size=1
        )
        self.navigation_hold_sub = rospy.Subscriber(
            rospy.get_param("~navigation_hold_topic", "/lste/navigation_hold"),
            Bool,
            self.on_navigation_hold,
            queue_size=1,
        )
        self.hard_reset_ack_pub = rospy.Publisher(
            "/lste/experiment/hard_reset_ack", String, queue_size=20
        )
        self.hard_reset_release_sub = rospy.Subscriber(
            "/lste/experiment/hard_reset_release",
            String,
            self.on_hard_reset_release,
            queue_size=5,
        )
        self.hard_reset_sub = rospy.Subscriber(
            "/lste/experiment/hard_reset",
            String,
            self.on_hard_reset,
            queue_size=5,
        )
        rospy.loginfo(
            "Command velocity mux ready: initial_mode=%s teb_forward_only=%s "
            "teb_angular_switch=%.3f teb_angular_deadband=%.3f",
            self.mode,
            self.teb_forward_only,
            self.teb_angular_sign_switch_threshold,
            self.teb_angular_deadband,
        )
        rospy.loginfo(
            "Command velocity mux TEB contract: topic=%s required=%s",
            self.teb_command_contract_topic,
            self.require_teb_command_contract,
        )

    def reset_teb_filter(self):
        self.teb_last_angular_sign = 0
        self.teb_last_output_angular = 0.0
        self.teb_micro_hold_cycles = 0
        self.teb_turn_hint = 0.0
        self.teb_turn_hint_wall = 0.0
        self.teb_last_filter_reason = "reset"

    def filter_teb_angular(self, command, allow_turn_entry_reversal=False):
        """Drop only quantization-scale opposite steering corrections.

        TEB remains responsible for the trajectory and obstacle response. The
        mux only prevents a straight corridor from alternating between tiny
        left/right commands when the map path moves by one cell. A meaningful
        opposite turn still passes as soon as it exceeds the configured
        threshold.
        """
        angular = float(command.angular.z)
        magnitude = abs(angular)
        if self.teb_angular_deadband > 0.0 and magnitude <= self.teb_angular_deadband:
            command.angular.z = 0.0
            self.teb_last_output_angular = 0.0
            self.teb_micro_hold_cycles = 0
            self.teb_last_filter_reason = "angular_deadband"
            return command
        sign = 1 if angular > 0.0 else -1 if angular < 0.0 else 0
        if (
            sign
            and self.teb_last_angular_sign
            and sign != self.teb_last_angular_sign
            and magnitude < self.teb_angular_sign_switch_threshold
            and not allow_turn_entry_reversal
        ):
            self.teb_filter_events += 1
            if (
                self.teb_micro_hold_cycles < self.teb_micro_hold_limit
                and abs(self.teb_last_output_angular) > self.teb_angular_deadband
            ):
                # Preserve the current turn direction briefly.  Returning an
                # immediate zero here makes a turn-only trajectory look like
                # repeated braking, because /cmd_vel is the mux output that
                # the robot actually receives.
                command.angular.z = self.teb_last_output_angular
                self.teb_micro_hold_cycles += 1
                self.teb_last_filter_reason = "angular_micro_reversal_held"
            else:
                command.angular.z = 0.0
                self.teb_last_output_angular = 0.0
                self.teb_micro_hold_cycles = 0
                self.teb_last_filter_reason = "angular_micro_reversal_suppressed"
            rospy.loginfo_throttle(
                2.0,
                "TEB micro-reversal held/suppressed (count=%d raw=%.3f "
                "output=%.3f threshold=%.3f hold=%d/%d)",
                self.teb_filter_events,
                angular,
                command.angular.z,
                self.teb_angular_sign_switch_threshold,
                self.teb_micro_hold_cycles,
                self.teb_micro_hold_limit,
            )
            return command
        if sign:
            self.teb_last_angular_sign = sign
        self.teb_last_output_angular = float(command.angular.z)
        self.teb_micro_hold_cycles = 0
        self.teb_last_filter_reason = "none"
        return command

    def on_mode(self, message):
        mode = message.data.strip().lower()
        if mode not in ("sappo", "teleop", "teb"):
            return
        with self.lock:
            changed = mode != self.mode
            self.mode = mode
        if changed:
            self.reset_teb_filter()
            self.output_pub.publish(Twist())
            self.publish_status(
                source="none",
                input_command=None,
                output_command=Twist(),
                active=False,
                block_reason="controller_mode_switch",
                governor_reason="not_evaluated",
                governor_limit=None,
                filter_reason="reset",
            )
            rospy.loginfo("Command velocity source switched to %s", mode)

    def on_scan(self, message):
        """Track the minimum range in the forward sector for the governor."""
        forward = float("inf")
        angle = float(message.angle_min)
        for i, value in enumerate(message.ranges):
            if math.isfinite(value) and value > 0.01 and abs(angle) <= math.radians(25.0):
                forward = min(forward, float(value))
            angle += float(message.angle_increment)
        with self.lock:
            self.scan_forward_min = forward
            stamp = message.header.stamp.to_sec()
            self.scan_stamp = stamp if stamp > 0.0 else None
            self.scan_received_time = rospy.get_time()

    def _governed_linear(self, linear):
        """Cap forward speed by the braking distance available ahead.

        Uses the standard kinematic stopping bound: from speed v the robot
        stops within v^2 / (2*a).  Enforcing ``v <= sqrt(2*a*(c-gap))`` means
        the robot can always halt before the nearest obstacle it can see, no
        matter what upstream layer commanded the motion.
        """
        if linear <= 0.0:
            return linear, None, "not_forward"
        with self.lock:
            clearance = self.scan_forward_min
        if not math.isfinite(clearance) or clearance <= 0.0:
            return linear, None, "forward_scan_unavailable"
        margin = max(0.0, float(clearance) - self.governor_min_gap)
        max_v = min(
            self.governor_max_speed,
            math.sqrt(2.0 * self.governor_decel * margin),
        )
        if linear > max_v:
            return max_v, max_v, "governor_limited"
        return linear, max_v, "within_governor_limit"

    @staticmethod
    def _command_payload(command):
        if command is None:
            return None
        return {
            "linear_x": round(float(command.linear.x), 5),
            "angular_z": round(float(command.angular.z), 5),
        }

    def publish_status(
        self,
        source,
        input_command,
        output_command,
        active,
        block_reason,
        governor_reason,
        governor_limit,
        filter_reason,
        command_identity=None,
    ):
        """Publish the exact mode/safety decision associated with one output."""
        with self.lock:
            self.status_sequence += 1
            scan_forward_min = self.scan_forward_min
            scan_age = None
            if self.scan_stamp is not None:
                scan_age = max(0.0, rospy.get_time() - self.scan_stamp)
            payload = {
                "event": "actuator_command",
                "sequence": self.status_sequence,
                "source": str(source),
                "selected_mode": self.mode,
                "active": bool(active),
                "task_done": bool(self.task_done),
                "navigation_hold": bool(self.navigation_hold),
                "block_reason": str(block_reason or "none"),
                "input": self._command_payload(input_command),
                "output": self._command_payload(output_command),
                "filter_reason": str(filter_reason or "none"),
                "governor_reason": str(governor_reason or "not_evaluated"),
                "governor_linear_limit": (
                    None if governor_limit is None else round(float(governor_limit), 5)
                ),
                "scan_forward_min": (
                    None if not math.isfinite(scan_forward_min)
                    else round(float(scan_forward_min), 5)
                ),
                "scan_age_seconds": (
                    None if scan_age is None else round(float(scan_age), 5)
                ),
                "command_identity": command_identity,
            }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def forward(
        self,
        source,
        message,
        filter_reason="none",
        input_command=None,
        command_identity=None,
    ):
        if input_command is None:
            input_command = message
        with self.lock:
            active = (
                source == self.mode
                and not self.task_done
                and not self.navigation_hold
                and not self.hard_reset_hold
            )
            if source != self.mode:
                block_reason = "source_not_selected"
            elif self.task_done:
                block_reason = "task_complete"
            elif self.navigation_hold:
                block_reason = "navigation_hold"
            elif self.hard_reset_hold:
                block_reason = "hard_reset_hold"
            else:
                block_reason = "none"
        if not active:
            self.publish_status(
                source=source,
                input_command=input_command,
                output_command=None,
                active=False,
                block_reason=block_reason,
                governor_reason="not_evaluated",
                governor_limit=None,
                filter_reason=filter_reason,
                command_identity=command_identity,
            )
            return
        # Apply the forward-collision governor to every selected source (TEB,
        # SA-PPO, teleop) at the actuator boundary.  ``message`` is copied so
        # the source's own command object is never mutated.
        governed = copy.deepcopy(message)
        governed.linear.x, governor_limit, governor_reason = self._governed_linear(
            float(governed.linear.x)
        )
        self.output_pub.publish(governed)
        self.publish_status(
            source=source,
            input_command=input_command,
            output_command=governed,
            active=True,
            block_reason="none",
            governor_reason=governor_reason,
            governor_limit=governor_limit,
            filter_reason=filter_reason,
            command_identity=command_identity,
        )

    def on_task_done(self, message):
        with self.lock:
            changed = bool(message.data) != self.task_done
            self.task_done = bool(message.data)
            # ``task_done=false`` is a mission-state update, not permission to
            # cross a reset barrier.  Only the explicit reset-release message
            # may clear ``hard_reset_hold`` after all participants are idle.
        if self.task_done:
            # Completion is a safety gate, not a suggestion to the currently
            # selected controller. Stop both autonomous and keyboard commands.
            self.output_pub.publish(Twist())
            self.reset_teb_filter()
            self.publish_status(
                source="none",
                input_command=None,
                output_command=Twist(),
                active=False,
                block_reason="task_complete",
                governor_reason="not_evaluated",
                governor_limit=None,
                filter_reason="reset",
            )
            if changed:
                rospy.loginfo("Command velocity output locked at zero: task complete")
        elif changed:
            rospy.loginfo("Command velocity output unlocked: new task")

    def on_hard_reset(self, message):
        """Zero the actuator boundary until the next task is announced."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict) or payload.get("event") != "hard_reset":
            return
        reset_id = str(payload.get("reset_id", "") or "").strip()
        if not reset_id:
            return
        try:
            reset_transaction_id = int(payload.get("transaction_id", 0) or 0)
        except (TypeError, ValueError):
            reset_transaction_id = 0
        reset_identity = (reset_id, reset_transaction_id)
        duplicate = False
        with self.lock:
            duplicate = reset_identity in self.hard_reset_identity_history
            if not duplicate:
                self.last_hard_reset_id = reset_id
                self.last_hard_reset_transaction_id = reset_transaction_id
                self.hard_reset_identity_history.append(reset_identity)
                del self.hard_reset_identity_history[:-16]
                self.hard_reset_hold = True
                self.task_done = False
                self.navigation_hold = False
                self.teb_contract_last_transaction_id = 0
                self.teb_contract_last_sequence = 0
                self.teb_contract_identity = None
                self.reset_teb_filter()
                self.scan_forward_min = float("inf")
                self.scan_stamp = None
                self.scan_received_time = None
            # The ACK describes the reset snapshot, not the later release
            # state.  A duplicate callback can be delivered after release;
            # reporting the live hold value there would overwrite a valid
            # snapshot in the reset verifier.
            reset_snapshot_hold = True
        if not duplicate:
            self.output_pub.publish(Twist())
            self.publish_status(
                source="none",
                input_command=None,
                output_command=Twist(),
                active=False,
                block_reason="hard_reset",
                governor_reason="not_evaluated",
                governor_limit=None,
                filter_reason="reset",
            )
        self.hard_reset_ack_pub.publish(String(data=json.dumps({
            "event": "hard_reset_ack",
            "node": "lste_cmd_vel_mux",
            "reset_id": reset_id,
            "reason": str(payload.get("reason", "slice_boundary")),
            "reset_transaction_id": reset_transaction_id,
            "state": "IDLE",
            "actuator_zero": reset_snapshot_hold,
            "hard_reset_hold": reset_snapshot_hold,
            "duplicate": duplicate,
            "wall_time": rospy.get_time(),
        }, sort_keys=True)))
        if not duplicate:
            rospy.loginfo(
                "Command velocity mux hard reset complete: reset_id=%s state=IDLE "
                "actuator_zero=true",
                reset_id,
            )

    def on_hard_reset_release(self, message):
        """Release the zero-output barrier after every reset participant is idle."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict) or payload.get("event") != "hard_reset_release":
            return
        reset_id = str(payload.get("reset_id", "") or "").strip()
        if not reset_id or reset_id != self.last_hard_reset_id:
            return
        with self.lock:
            self.hard_reset_hold = False
        self.hard_reset_ack_pub.publish(String(data=json.dumps({
            "event": "hard_reset_release_ack",
            "node": "lste_cmd_vel_mux",
            "reset_id": reset_id,
            "reason": str(payload.get("reason", "slice_boundary")),
            "state": "IDLE",
            "actuator_zero": True,
            "hard_reset_hold": False,
            "wall_time": rospy.get_time(),
        }, sort_keys=True)))
        rospy.loginfo(
            "Command velocity mux hard reset release complete: reset_id=%s "
            "hard_reset_hold=false",
            reset_id,
        )

    def on_navigation_hold(self, message):
        """Gate motion for a mission observation window without changing goal."""
        with self.lock:
            changed = bool(message.data) != self.navigation_hold
            self.navigation_hold = bool(message.data)
        if not changed:
            return
        if self.navigation_hold:
            self.output_pub.publish(Twist())
            self.reset_teb_filter()
            self.publish_status(
                source="none",
                input_command=None,
                output_command=Twist(),
                active=False,
                block_reason="navigation_hold",
                governor_reason="not_evaluated",
                governor_limit=None,
                filter_reason="reset",
            )
        rospy.loginfo(
            "Command velocity observation gate %s",
            "closed" if self.navigation_hold else "opened",
        )

    def on_sappo_cmd(self, message):
        self.forward("sappo", message)

    def on_teleop_cmd(self, message):
        self.forward("teleop", message)

    def _reject_teb_contract(self, reason, identity=None, input_command=None):
        """Stop on an untrusted command instead of executing stale motion."""
        with self.lock:
            self.teb_contract_rejection_count += 1
        rejection_identity = dict(identity or {})
        rejection_identity.update(
            {
                "identity_mismatch": True,
                "rejection_reason": str(reason),
            }
        )
        zero = Twist()
        self.reset_teb_filter()
        self.output_pub.publish(zero)
        self.publish_status(
            source="teb",
            input_command=input_command,
            output_command=zero,
            active=False,
            block_reason=reason,
            governor_reason="not_evaluated",
            governor_limit=None,
            filter_reason="transaction_rejected",
            command_identity=rejection_identity,
        )
        rospy.logwarn_throttle(
            2.0,
            "Rejected TEB command contract: identity_mismatch=true reason=%s "
            "identity=%s count=%d",
            reason,
            identity,
            self.teb_contract_rejection_count,
        )

    def on_teb_command_contract(self, message):
        """Accept only the supervisor command carrying a current transaction."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self._reject_teb_contract("malformed_command_contract")
            return
        if not isinstance(payload, dict) or payload.get("event") != "teb_command":
            self._reject_teb_contract("invalid_command_contract")
            return
        try:
            transaction_id = int(payload.get("transaction_id", 0) or 0)
            command_sequence = int(payload.get("command_sequence", 0) or 0)
            planner_transaction_id = int(
                payload.get("planner_transaction_id", 0) or 0
            )
            active_transaction_id = int(
                payload.get("active_action_transaction_id", 0) or 0
            )
            graph_transaction_id = int(
                payload.get("graph_transaction_id", 0) or 0
            )
            planner_graph_transaction_id = int(
                payload.get("planner_graph_transaction_id", 0) or 0
            )
            active_graph_transaction_id = int(
                payload.get("active_action_graph_transaction_id", 0) or 0
            )
            route_id = int(payload.get("route_id", 0) or 0)
            planner_route_id = int(payload.get("planner_route_id", 0) or 0)
            active_route_id = int(
                payload.get("active_action_route_id", 0) or 0
            )
            planner_action_generation = int(
                payload.get("planner_action_generation", 0) or 0
            )
            active_action_generation = int(
                payload.get("active_action_generation", 0) or 0
            )
            planner_contract_sequence = int(
                payload.get("planner_contract_sequence", 0) or 0
            )
            active_action = bool(payload.get("active_action", False))
            raw_map_epoch = payload.get("map_epoch")
            map_epoch = (
                None
                if raw_map_epoch is None
                else int(raw_map_epoch or 0)
            )
            raw_planner_map_epoch = payload.get("planner_map_epoch")
            planner_map_epoch = (
                None
                if raw_planner_map_epoch is None
                else int(raw_planner_map_epoch or 0)
            )
            raw_active_map_epoch = payload.get("active_action_map_epoch")
            active_map_epoch = (
                None
                if raw_active_map_epoch is None
                else int(raw_active_map_epoch or 0)
            )
            linear_x = float(payload.get("linear_x", 0.0) or 0.0)
            angular_z = float(payload.get("angular_z", 0.0) or 0.0)
        except (TypeError, ValueError):
            self._reject_teb_contract("invalid_command_identity")
            return
        identity = {
            "transaction_id": transaction_id,
            "command_sequence": command_sequence,
            "planner_command_sequence": int(
                payload.get("planner_command_sequence", 0) or 0
            ),
            "planner_transaction_id": planner_transaction_id,
            "active_action_transaction_id": active_transaction_id,
            "graph_transaction_id": graph_transaction_id,
            "planner_graph_transaction_id": planner_graph_transaction_id,
            "active_action_graph_transaction_id": active_graph_transaction_id,
            "route_id": route_id,
            "planner_route_id": planner_route_id,
            "active_action_route_id": active_route_id,
            "planner_action_generation": planner_action_generation,
            "active_action_generation": active_action_generation,
            "planner_contract_sequence": planner_contract_sequence,
            "active_action": active_action,
            "map_epoch": map_epoch,
            "planner_map_epoch": planner_map_epoch,
            "active_action_map_epoch": active_map_epoch,
            "decision": str(payload.get("decision", "unknown") or "unknown"),
            "state": str(payload.get("state", "unknown") or "unknown"),
        }
        input_command = Twist()
        input_command.linear.x = linear_x
        input_command.angular.z = angular_z
        nonzero = abs(linear_x) > 0.001 or abs(angular_z) > 0.001
        with self.lock:
            last_transaction_id = self.teb_contract_last_transaction_id
            last_sequence = self.teb_contract_last_sequence
            if transaction_id <= 0:
                reason = "missing_transaction_id"
            elif transaction_id < last_transaction_id or (
                transaction_id == last_transaction_id
                and command_sequence <= last_sequence
            ):
                reason = "stale_command_transaction"
            elif nonzero and planner_transaction_id != transaction_id:
                reason = "planner_transaction_mismatch"
            elif nonzero and route_id <= 0:
                reason = "missing_route_id"
            elif nonzero and planner_route_id <= 0:
                reason = "missing_planner_route_id"
            elif nonzero and not active_action:
                reason = "no_active_action"
            elif nonzero and planner_action_generation <= 0:
                reason = "missing_action_generation"
            elif nonzero and active_action and active_action_generation <= 0:
                reason = "active_action_generation_missing"
            elif nonzero and active_action and active_transaction_id <= 0:
                reason = "active_action_transaction_missing"
            elif nonzero and active_action and active_route_id <= 0:
                reason = "active_action_route_id_missing"
            elif (
                nonzero
                and active_action_generation > 0
                and planner_action_generation != active_action_generation
            ):
                reason = "action_generation_mismatch"
            elif nonzero and active_action_generation <= 0 and active_action:
                reason = "active_action_generation_missing"
            elif nonzero and active_transaction_id not in (0, transaction_id):
                reason = "active_transaction_mismatch"
            elif (
                nonzero
                and route_id > 0
                and graph_transaction_id <= 0
                and (
                    planner_graph_transaction_id > 0
                    or active_graph_transaction_id > 0
                )
            ):
                reason = "missing_graph_transaction_id"
            elif nonzero and graph_transaction_id != planner_graph_transaction_id:
                reason = "graph_transaction_mismatch"
            elif (
                nonzero
                and active_graph_transaction_id > 0
                and graph_transaction_id != active_graph_transaction_id
            ):
                reason = "active_graph_transaction_mismatch"
            elif nonzero and route_id != planner_route_id:
                reason = "route_id_mismatch"
            elif (
                nonzero
                and active_route_id > 0
                and route_id != active_route_id
            ):
                reason = "active_route_id_mismatch"
            elif nonzero and route_id > 0 and map_epoch is None:
                reason = "missing_map_epoch"
            elif (
                nonzero
                and map_epoch != planner_map_epoch
            ):
                reason = "map_epoch_mismatch"
            elif (
                nonzero
                and active_map_epoch is not None
                and map_epoch != active_map_epoch
            ):
                reason = "active_map_epoch_mismatch"
            else:
                reason = ""
            if not reason:
                self.teb_contract_last_transaction_id = transaction_id
                self.teb_contract_last_sequence = command_sequence
                self.teb_contract_identity = identity
        if reason:
            self._reject_teb_contract(reason, identity, input_command)
            return
        self._process_teb_command(input_command, identity)

    def _process_teb_command(self, message, command_identity=None):
        command = copy.deepcopy(message)
        filter_reason = "none"
        if self.teb_forward_only and message.linear.x < 0.0:
            # TEB enforces a small positive reverse lower bound internally
            # (typically 0.01 m/s), even when the launch file requests a
            # near-zero backwards limit. On this differential-drive base that
            # tiny reverse command competes with the turn optimizer and causes
            # the observed left/right oscillation at a goal behind the robot.
            # Clamping only the mux output preserves TEB's angular command and
            # gives the vehicle an in-place turn without changing the planner.
            command.linear.x = 0.0
            filter_reason = "forward_only_reverse_clamp"
            # A negative first sample is TEB's way of entering a turn when
            # forward motion is infeasible. The next points already contain
            # the turn, so use that fresh planner intent to avoid one zero
            # cycle before rotation starts.
            with self.lock:
                hint_fresh = (
                    self.teb_turn_hint != 0.0
                    and rospy.get_time() - self.teb_turn_hint_wall <= 0.25
                )
                if hint_fresh and abs(command.angular.z) <= self.teb_angular_deadband:
                    command.angular.z = self.teb_turn_hint
                    self.teb_turn_transition_count += 1
                    rospy.loginfo_throttle(
                        2.0,
                        "TEB turn transition kept continuous (count=%d hint=%.3f)",
                        self.teb_turn_transition_count,
                        self.teb_turn_hint,
                    )
            self.teb_reverse_clamp_count += 1
            rospy.logwarn_throttle(
                2.0,
                "TEB reverse command clamped to zero (count=%d raw=%.3f)",
                self.teb_reverse_clamp_count,
                message.linear.x,
            )
        # A negative TEB linear sample is the planner's explicit turn-entry
        # signal when forward-only execution is enabled.  The mux has already
        # removed that reverse component; do not then suppress the accompanying
        # small opposite angular command as if it were corridor quantization.
        command = self.filter_teb_angular(
            command,
            allow_turn_entry_reversal=(
                self.teb_forward_only
                and float(message.linear.x) < 0.0
                and abs(float(command.angular.z)) > self.teb_angular_deadband
            ),
        )
        if self.teb_last_filter_reason != "none":
            filter_reason = self.teb_last_filter_reason
        self.forward(
            "teb",
            command,
            filter_reason=filter_reason,
            input_command=message,
            command_identity=command_identity,
        )

    def on_teb_cmd(self, message):
        """Compatibility callback for isolated legacy launches."""
        if self.require_teb_command_contract:
            self._reject_teb_contract("missing_transaction_identity", input_command=message)
            return
        self._process_teb_command(message, self.teb_contract_identity)

    def on_teb_feedback(self, message):
        """Cache the next angular motion in TEB's selected trajectory.

        Feedback is only an adapter input. TEB remains the sole source of the
        turn direction, and the hint expires immediately when feedback stops.
        """
        selected_index = int(message.selected_trajectory_idx)
        trajectories = list(message.trajectories)
        if selected_index < 0 or selected_index >= len(trajectories):
            with self.lock:
                self.teb_turn_hint = 0.0
                self.teb_turn_hint_wall = 0.0
            return
        points = list(trajectories[selected_index].trajectory)
        hint = 0.0
        for point in points[1:]:
            angular = float(point.velocity.angular.z)
            if abs(angular) > 0.05:
                hint = angular
                break
        with self.lock:
            self.teb_turn_hint = hint
            self.teb_turn_hint_wall = rospy.get_time() if hint else 0.0


if __name__ == "__main__":
    CmdVelMuxNode()
    rospy.spin()
