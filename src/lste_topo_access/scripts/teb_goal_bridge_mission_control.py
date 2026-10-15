"""Controller and task lifecycle callbacks for the TEB bridge."""

from clock_provider import now_for


class TebGoalBridgeMissionControlMixin:
    def on_mode(self, message):
        mode = message.data.strip().lower()
        if not mode:
            return
        with self.lock:
            previous = self.mode
            self.mode = mode
            if previous == self.active_mode and mode != self.active_mode:
                self.cancel_locked("controller_switched_to_%s" % mode)
            elif mode == self.active_mode and previous != self.active_mode:
                self.dispatch_locked(force=True, reason="controller_switched_to_teb")

    def on_task_done(self, message):
        with self.lock:
            previous = self.task_done
            self.task_done = bool(message.data)
            if self.task_done:
                if self.persistent_execution:
                    # PersistentTebLocalPlanner observes this latch and returns
                    # the one action lease naturally, without a stop/preempt gap.
                    self.publish_bridge_status(
                        "persistent_task_done_waiting_for_action_terminal"
                    )
                else:
                    self.cancel_locked("task_done")
            elif previous and self._is_active_mode():
                self.persistent_installed_target_goal = None
                self.persistent_installed_target_transaction = 0
                self.persistent_target_approach_reported_transaction = 0
                self.dispatch_locked(force=True, reason="new_task")

    def on_navigation_hold(self, message):
        """Apply an intentional observation hold to bridge health accounting."""
        active = bool(message.data)
        with self.lock:
            previous = bool(getattr(self, "navigation_hold", False))
            if active == previous:
                return
            now = now_for(self)
            previous_started = float(
                getattr(self, "navigation_hold_started_monotonic", 0.0)
                or 0.0
            )
            self.navigation_hold = active
            self.navigation_hold_last_change_monotonic = now
            if active:
                self.navigation_hold_started_monotonic = now
                event = "navigation_hold_started"
                hold_duration = None
            else:
                # The no-progress clock must resume from the release edge. A
                # target observation hold is not controller execution time.
                self.navigation_hold_started_monotonic = 0.0
                self.active_progress_monotonic = now
                self.active_motion_progress_monotonic = now
                self.frontier_stale_wait_started_monotonic = 0.0
                event = "navigation_hold_released"
                hold_duration = (
                    None
                    if previous_started <= 0.0
                    else round(max(0.0, now - previous_started), 3)
                )
            self.publish_bridge_status(
                event,
                navigation_hold=active,
                hold_duration=hold_duration,
            )
