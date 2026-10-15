"""move_base action and recovery telemetry for the navigation observer."""

import time


STATUS_NAMES = {
    0: "PENDING",
    1: "ACTIVE",
    2: "PREEMPTED",
    3: "SUCCEEDED",
    4: "ABORTED",
    5: "REJECTED",
    6: "PREEMPTING",
    7: "RECALLING",
    8: "RECALLED",
    9: "LOST",
}


class NavigationMetricsMoveBaseMixin:
    """Observe move_base feedback and recoveries without controlling either."""

    def on_move_base_feedback(self, message):
        with self.lock:
            self.last_move_base_feedback_wall = time.monotonic()
            status = message.status
            self.move_base_feedback_state = {
                "goal_id": status.goal_id.id,
                "status": int(status.status),
                "status_name": STATUS_NAMES.get(
                    int(status.status), "STATUS_%d" % int(status.status)
                ),
                "base": [
                    round(float(message.feedback.base_position.pose.position.x), 3),
                    round(float(message.feedback.base_position.pose.position.y), 3),
                ],
                "frame": message.feedback.base_position.header.frame_id or "odom",
            }

    def on_recovery(self, message):
        with self.lock:
            current = {
                "current": int(message.current_recovery_number),
                "total": int(message.total_number_of_recoveries),
                "behavior": message.recovery_behavior_name,
            }
            if current != self.recovery_state:
                self.recovery_state = current
                now = time.monotonic()
                self._failure_record_context_locked("move_base", "recovery", current)
                self.move_base_recovery_events += 1
                self.lifecycle_event_wall["move_base_recovery"] = now
                # Recovery status often arrives just after TEB has emitted its
                # zero command. Reclassify that command as a planner recovery
                # rather than falsely attributing it to a clear-path brake.
                self._reclassify_recent_discontinuities_locked(
                    now,
                    "move_base_recovery",
                    replacement_reason="planner_recovery",
                    window=2.0,
                )
                self._write("WARN", "move_base_recovery", **current)
