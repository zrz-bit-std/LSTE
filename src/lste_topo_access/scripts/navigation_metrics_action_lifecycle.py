"""Action and turn lifecycle telemetry for the navigation observer.

This mixin records action dispatch, actionlib terminal state, and turn
supervisor events. It may reclassify a previously observed zero command after a
matching lifecycle event, but it never feeds data back into navigation.
"""

import json
import math
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


class NavigationMetricsActionLifecycleMixin:
    def on_turn_supervisor_status(self, message):
        """Persist the explicit turn action state in the formal run log."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = {"event": "invalid", "raw": message.data}
        if not isinstance(payload, dict):
            payload = {"event": "invalid", "raw": message.data}
        with self.lock:
            event = str(payload.get("event", "unknown"))
            self.lifecycle_event_wall["turn_%s" % event] = time.monotonic()
            self.teb_turn_supervisor_status = payload
            if event == "trajectory_continuity":
                self.teb_trajectory_continuity_events = max(
                    self.teb_trajectory_continuity_events,
                    int(payload.get("count", 0)),
                )
                # This is a scheduler-quality counter.  A status callback
                # can arrive after an unrelated brake or turn, so it must not
                # rewrite command discontinuity causality by timestamp alone.
            if event != self.teb_turn_supervisor_last_event:
                self.teb_turn_supervisor_events += 1
                self.teb_turn_supervisor_last_event = event
                turn_event = payload.pop("event", event)
                self._write(
                    "INFO" if event not in ("turn_released",) else "WARN",
                    "teb_turn_supervisor_event",
                    turn_event=turn_event,
                    **payload,
                )

    def _record_dispatch(self, xy, transport):
        with self.lock:
            self.dispatch_count += 1
            delta = float("nan")
            if self.dispatch_last_xy is not None:
                delta = math.hypot(xy[0] - self.dispatch_last_xy[0], xy[1] - self.dispatch_last_xy[1])
            self.dispatch_last_xy = xy
            self.subgoal = xy
            self._write(
                "INFO",
                "move_base_dispatch",
                count=self.dispatch_count,
                transport=transport,
                delta_m=None if not math.isfinite(delta) else round(delta, 4),
                goal=[round(xy[0], 3), round(xy[1], 3)],
            )

    def on_action_dispatch(self, message):
        with self.lock:
            target = message.goal.target_pose
            self._record_dispatch(
                (float(target.pose.position.x), float(target.pose.position.y)),
                "move_base_action",
            )
            self._record_terminal_to_dispatch_locked(
                time.monotonic(), "move_base_action"
            )

    def on_dispatch(self, message):
        with self.lock:
            self._record_dispatch(
                (float(message.pose.position.x), float(message.pose.position.y)),
                "move_base_simple_goal",
            )
            self._record_terminal_to_dispatch_locked(
                time.monotonic(), "move_base_simple_goal"
            )

    def _consume_preterminal_persistent_endpoint_preemption_locked(self):
        """Correlate action status that follows an explicit planner terminal hold.

        The bridge cancels the action before the endpoint and terminal status
        topics are necessarily delivered to this observer. The planner
        contract is already an identity-checked terminal boundary at that
        point, so use it as the local predecessor for the same action.
        """
        if not bool(getattr(self, "persistent_execution", False)):
            return False
        reason = str(
            getattr(self, "teb_feedback_invalid_reason", "") or ""
        ).strip().lower()
        if reason not in {
            "planner_contract_endpoint_reached",
            "planner_contract_terminal_hold",
            "planner_contract_persistent_frontier_endpoint_terminal",
        }:
            return False
        if not bool(getattr(self, "bridge_active", False)):
            return False
        source = str(
            getattr(self, "bridge_active_intent_source", "") or ""
        ).strip().lower()
        if source and source != "global_slam_frontier":
            return False
        route = {}
        reader = getattr(self, "_failure_route_context_locked", None)
        if callable(reader):
            context = reader()
            if isinstance(context, dict) and isinstance(context.get("route"), dict):
                route = context["route"]
        route_kind = str(
            route.get("active_route_kind") or route.get("route_kind") or ""
        ).strip().lower()
        if route_kind not in {"frontier_endpoint", "portal_transition", "local_egress"}:
            return False
        self._write(
            "INFO",
            "frontier_observation_preemption",
            reason="planner_contract_terminal_boundary",
            planner_feedback_reason=reason,
            route_id=route.get("active_route_id") or route.get("route_id"),
            route_kind=route_kind,
            correlation_window_seconds=0.0,
        )
        return True

    def _consume_recent_persistent_terminal_preemption_locked(self):
        """Correlate the PREEMPTED emitted by an intentional endpoint terminal."""
        if not bool(getattr(self, "persistent_execution", False)):
            return False
        event_name = "persistent_execution_terminal"
        terminal_wall = float(
            getattr(self, "lifecycle_event_wall", {}).get(event_name, 0.0)
            or 0.0
        )
        # The typed terminal and /move_base/status use independent ROS topic
        # queues. A validated endpoint can therefore trigger the intentional
        # cancel before the terminal topic callback is delivered. The bridge's
        # endpoint_detected status is already identity-checked, so it is a
        # safe predecessor for the same bounded PREEMPTED correlation window.
        endpoint_wall = float(
            getattr(self, "lifecycle_event_wall", {}).get(
                "endpoint_detected", 0.0
            )
            or 0.0
        )
        if endpoint_wall > terminal_wall:
            event_name = "endpoint_detected"
            terminal_wall = endpoint_wall
        if terminal_wall <= 0.0 or time.monotonic() - terminal_wall > 1.0:
            return False
        source = str(
            getattr(self, "bridge_active_intent_source", "") or ""
        ).strip().lower()
        if source and source != "global_slam_frontier":
            return False
        self._write(
            "INFO",
            "frontier_observation_preemption",
            reason=event_name,
            correlation_window_seconds=1.0,
        )
        return True

    @staticmethod
    def _is_expected_goal_replacement_status(status):
        """Recognize actionlib cancellation caused by a successor goal."""
        try:
            code = int(status.status)
        except (AttributeError, TypeError, ValueError):
            return False
        if code not in (8, 9):
            return False
        text = str(getattr(status, "text", "") or "").strip().lower()
        return "another goal" in text and "simple action server" in text

    def on_status(self, message):
        with self.lock:
            for status in message.status_list:
                key = (status.goal_id.id, int(status.status))
                if key in self.status_seen:
                    continue
                self.status_seen.add(key)
                if status.goal_id.id:
                    self.move_base_goal_ids.add(status.goal_id.id)
                code = int(status.status)
                name = STATUS_NAMES.get(code, "STATUS_%d" % code)
                self.last_move_base_status = name
                self._failure_record_context_locked(
                    "move_base",
                    name.lower(),
                    {
                        "goal_id": status.goal_id.id,
                        "status": code,
                        "status_text": status.text or "-",
                    },
                )
                self.lifecycle_event_wall["move_base_%s" % name.lower()] = time.monotonic()
                self.status_counts[name] = self.status_counts.get(name, 0) + 1
                if code == 2:
                    self.preemptions += 1
                    if self._consume_hard_reset_preemption_locked():
                        self._write(
                            "INFO",
                            "hard_reset_preemption",
                            goal_id=status.goal_id.id,
                            reason="warm_slice_hard_reset",
                        )
                    elif self._consume_preterminal_persistent_endpoint_preemption_locked():
                        self.frontier_observation_preemptions += 1
                    elif self._consume_recent_persistent_terminal_preemption_locked():
                        self.frontier_observation_preemptions += 1
                    elif self.pending_task_done_preemptions > 0:
                        self.pending_task_done_preemptions -= 1
                        self.task_done_preemptions += 1
                    elif self.pending_target_terminal_observation_preemptions > 0:
                        self.pending_target_terminal_observation_preemptions -= 1
                        self.target_terminal_observation_preemptions += 1
                    elif self.pending_priority_preemptions > 0:
                        self.pending_priority_preemptions -= 1
                        self.priority_preemptions += 1
                    elif self.pending_target_segment_preemptions > 0:
                        self.pending_target_segment_preemptions -= 1
                        self.target_segment_preemptions += 1
                    elif self.pending_frontier_continuous_prefetch_preemptions > 0:
                        self.pending_frontier_continuous_prefetch_preemptions -= 1
                        self.frontier_continuous_prefetch_preemptions += 1
                    elif self.pending_frontier_segment_preemptions > 0:
                        self.pending_frontier_segment_preemptions -= 1
                        self.frontier_segment_preemptions += 1
                    elif self.pending_frontier_terminal_settle_preemptions > 0:
                        self.pending_frontier_terminal_settle_preemptions -= 1
                        self.frontier_terminal_settle_preemptions += 1
                    elif self.pending_frontier_observation_preemptions > 0:
                        self.pending_frontier_observation_preemptions -= 1
                        self.frontier_observation_preemptions += 1
                    elif (
                        self._consume_expected_route_recovery_preemption_locked()
                        is not None
                    ):
                        self.route_recovery_preemptions += 1
                        self._write(
                            "INFO",
                            "route_recovery_preemption",
                            goal_id=status.goal_id.id,
                            reason="frontier_release_seen_before_move_base_status",
                        )
                    elif self.pending_route_recovery_preemptions > 0:
                        self.pending_route_recovery_preemptions -= 1
                        self.route_recovery_preemptions += 1
                        self._write(
                            "INFO",
                            "route_recovery_preemption",
                            goal_id=status.goal_id.id,
                            reasons=dict(self.route_recovery_preemption_reasons),
                        )
                    elif (
                        self._consume_recent_frontier_release_preemption_locked()
                        is not None
                    ):
                        self.route_recovery_preemptions += 1
                        self._write(
                            "INFO",
                            "route_recovery_preemption",
                            goal_id=status.goal_id.id,
                            reason=(
                                "recent_frontier_release_context"
                            ),
                        )
                    else:
                        self.unexpected_preemptions += 1
                        self._begin_failure_episode_locked(
                            "unexpected_preemption",
                            "move_base",
                            {
                                "goal_id": status.goal_id.id,
                                "status": code,
                                "status_text": status.text or "-",
                            },
                        )
                elif code == 3:
                    self.successes += 1
                elif code in (4, 5, 8, 9):
                    if self._is_expected_goal_replacement_status(status):
                        self.route_recovery_preemptions += 1
                        self._write(
                            "INFO",
                            "route_recovery_preemption",
                            goal_id=status.goal_id.id,
                            reason="move_base_successor_goal_replacement",
                            status_name=name,
                        )
                    else:
                        self.aborts += 1
                        self._begin_failure_episode_locked(
                            "move_base_terminal_failure",
                            "move_base",
                            {
                                "goal_id": status.goal_id.id,
                                "status": code,
                                "status_name": name,
                                "status_text": status.text or "-",
                            },
                        )
                self._write(
                    "INFO" if code in (0, 1, 3) else "WARN",
                    "move_base_status",
                    goal_id=status.goal_id.id,
                    status=code,
                    status_name=name,
                    text=status.text or "-",
                    preemptions=self.preemptions,
                    frontier_observation_preemptions=(
                        self.frontier_observation_preemptions
                    ),
                    frontier_continuous_prefetch_preemptions=(
                        self.frontier_continuous_prefetch_preemptions
                    ),
                    frontier_segment_preemptions=self.frontier_segment_preemptions,
                    target_terminal_observation_preemptions=(
                        self.target_terminal_observation_preemptions
                    ),
                    priority_preemptions=self.priority_preemptions,
                    task_done_preemptions=self.task_done_preemptions,
                    unexpected_preemptions=self.unexpected_preemptions,
                    aborts=self.aborts,
                )
                if code == 3:
                    self._mark_action_terminal_locked(
                        time.monotonic(), "move_base_succeeded"
                    )
                    self._reclassify_recent_discontinuities_locked(
                        time.monotonic(),
                        "move_base_succeeded",
                    )

    def _remember_discontinuity_locked(self, kind, reason, now):
        """Retain a provisional brake/stop cause for terminal correlation."""
        self.discontinuity_sequence += 1
        record = {
            "id": int(self.discontinuity_sequence),
            "kind": str(kind),
            "reason": str(reason),
            "wall": float(now),
        }
        self.recent_discontinuities.append(record)
        cutoff = now - 2.0
        while (
            self.recent_discontinuities
            and self.recent_discontinuities[0]["wall"] < cutoff
        ):
            self.recent_discontinuities.popleft()
        return record

    def _mark_action_terminal_locked(self, now, source):
        """Start one terminal-to-successor timing interval.

        The bridge terminal callback and move_base SUCCEEDED status normally
        arrive within one scheduler tick of each other. Preserve the first
        timestamp, rather than treating their duplicate reports as two action
        boundaries.
        """
        if (
            self.pending_action_terminal_wall is None
            or now - self.pending_action_terminal_wall > 0.75
        ):
            self.pending_action_terminal_wall = now
            self.pending_action_terminal_source = str(source)

    def _record_terminal_to_dispatch_locked(self, now, transport):
        """Record one successful-action handoff, excluding unrelated goals."""
        if self.pending_action_terminal_wall is None:
            return
        delay = now - self.pending_action_terminal_wall
        # A later manual/new mission dispatch is not the successor of this
        # terminal action. Five seconds already exceeds the expected online
        # frontier planning cycle by a wide margin.
        if delay < 0.0 or delay > 5.0:
            self.pending_action_terminal_wall = None
            self.pending_action_terminal_source = ""
            return
        self.terminal_to_dispatch_count += 1
        self.terminal_to_dispatch_total += delay
        self.terminal_to_dispatch_max = max(self.terminal_to_dispatch_max, delay)
        self.terminal_to_dispatch_last = delay
        self._write(
            "INFO",
            "terminal_to_dispatch",
            count=self.terminal_to_dispatch_count,
            delay_seconds=round(delay, 4),
            terminal_source=self.pending_action_terminal_source,
            transport=str(transport),
        )
        self.pending_action_terminal_wall = None
        self.pending_action_terminal_source = ""

    def _reclassify_recent_discontinuities_locked(
        self, now, lifecycle_event, replacement_reason="action_terminal", window=0.75
    ):
        """Correct command-first observations once a terminal status arrives.

        ROS publishes the final zero command and action status from different
        callbacks. Treating their delivery order as physical causality made a
        normal endpoint deceleration look like an unexplained safety brake.
        This only changes telemetry counters; it never affects navigation.
        """
        for record in self.recent_discontinuities:
            delay = now - record["wall"]
            if delay < 0.0 or delay > window:
                continue
            previous = record["reason"]
            if previous == replacement_reason:
                continue
            # A late supervisor feedback message can follow the successful
            # move_base terminal for the *same* command sample.  Arrival is a
            # stronger lifecycle fact than the optional continuity adapter;
            # never rewrite a confirmed endpoint stop into a continuity gap.
            if (
                replacement_reason == "trajectory_continuity"
                and previous == "action_terminal"
            ):
                continue
            counts = (
                self.brake_reason_counts
                if record["kind"] == "linear_brake"
                else self.stop_reason_counts
            )
            if counts.get(previous, 0) > 0:
                counts[previous] -= 1
                if counts[previous] == 0:
                    del counts[previous]
            self._increment_reason(counts, replacement_reason)
            record["reason"] = replacement_reason
            self._write(
                "INFO",
                "discontinuity_reclassified",
                discontinuity_id=int(record["id"]),
                kind=record["kind"],
                previous_reason=previous,
                reason=replacement_reason,
                lifecycle_event=lifecycle_event,
                delay_seconds=round(delay, 4),
            )

    @staticmethod
    def _recent_lifecycle_event(events, names, now, window=0.8):
        """Return the newest named lifecycle event inside ``window`` seconds."""
        newest_name = None
        newest_age = None
        for name in names:
            stamp = events.get(name)
            if stamp is None:
                continue
            age = max(0.0, now - stamp)
            if age <= window and (newest_age is None or age < newest_age):
                newest_name = name
                newest_age = age
        return newest_name, newest_age
