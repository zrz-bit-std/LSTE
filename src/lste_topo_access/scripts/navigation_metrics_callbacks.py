"""State, perception, and map callbacks for ``NavigationMetrics``."""

import json
import time

import rospy


class NavigationMetricsCallbacksMixin:
    """Handle lightweight observer callbacks outside the composition root."""

    def on_state(self, message):
        with self.lock:
            new_state = "%d:%s" % (int(message.state), message.subtype or "-")
            if new_state != self.state:
                previous = self.state
                self.state = new_state
                self._write(
                    "INFO",
                    "state_change",
                    previous=previous,
                    current=new_state,
                    state=int(message.state),
                    subtype=message.subtype or "",
                    goal=None if self.goal is None else [round(value, 3) for value in self.goal],
                    pose=None if self.pose is None else [round(value, 3) for value in self.pose],
                    scores=self.scores,
                )
                if int(message.state) == 2 and self.legacy_state_locked_ros is None:
                    self.legacy_state_locked_ros = rospy.Time.now().to_sec()
                    self._write(
                        "INFO",
                        "legacy_state_locked",
                        latency_seconds=round(
                            self.legacy_state_locked_ros - self.start_ros, 3
                        ),
                        state=new_state,
                    )

    def on_goal_diagnostic(self, message):
        with self.lock:
            try:
                diagnostic = json.loads(message.data)
            except (TypeError, ValueError):
                diagnostic = {"raw": message.data}
            self.goal_diagnostic = diagnostic
            self._failure_record_context_locked("goal", "goal_diagnostic", diagnostic)
            self.goal_source = str(diagnostic.get("source", self.goal_source))
            if self.goal_source.startswith("target_"):
                self.target_goal_changes += 1
            self._write("INFO", "goal_diagnostic", **diagnostic)

    def on_goal_arbitration(self, message):
        """Record mission/execution ownership decisions as first-class events."""
        with self.lock:
            try:
                payload = json.loads(message.data)
            except (TypeError, ValueError):
                payload = {"raw": message.data}
            if not isinstance(payload, dict):
                payload = {"raw": message.data}
            event = str(payload.get("event", "unknown"))
            self._failure_record_context_locked("goal", event, payload)
            self._record_target_lifecycle_locked(event, payload)
            if event in ("target_room_entry_confirmed", "target_task_completed"):
                evidence = str(payload.get("completion_evidence", "") or "").strip()
                if evidence:
                    self.task_done_completion_evidence = evidence
            if event == "target_follow_confirmed":
                if self.target_follow_confirmed_ros is None:
                    self.target_follow_confirmed_ros = rospy.Time.now().to_sec()
                    # ``target_lock`` was previously inferred from the legacy
                    # LSTE state value 2. TEB target following has its own
                    # evidence contract, so retain this field for consumers
                    # while giving it the correct mission-level meaning.
                    self.target_lock_ros = self.target_follow_confirmed_ros
                    self._write(
                        "INFO",
                        "target_follow_confirmed",
                        latency_seconds=round(
                            self.target_follow_confirmed_ros - self.start_ros, 3
                        ),
                        target_track_id=payload.get("target_track_id"),
                        hits=payload.get("hits"),
                        average_score=payload.get("average_score"),
                    )
            elif event == "target_close_confirmation_started":
                if self.target_close_confirmation_started_ros is None:
                    self.target_close_confirmation_started_ros = rospy.Time.now().to_sec()
            elif event == "target_close_confirmed":
                if self.target_close_confirmed_ros is None:
                    self.target_close_confirmed_ros = rospy.Time.now().to_sec()
                    self._write(
                        "INFO",
                        "target_close_confirmed",
                        latency_seconds=round(
                            self.target_close_confirmed_ros - self.start_ros, 3
                        ),
                        target_track_id=payload.get("target_track_id"),
                        hits=payload.get("hits"),
                        hold_seconds=payload.get("hold_seconds"),
                    )
            elif event == "target_segment_committed":
                self.target_segments_committed += 1
            elif event == "target_continuous_handoff_prepared":
                self.target_continuous_handoffs_prepared += 1
            elif event == "target_route_accepted":
                self.target_route_accepts += 1
            elif event == "target_route_rejected":
                self.target_route_rejections += 1
            elif event == "target_route_deferred":
                self.target_route_deferrals += 1
            elif event == "target_route_held":
                self.target_route_holds += 1
            elif event == "target_route_semantic_replan":
                self.target_route_semantic_replans += 1
            elif event == "target_route_released":
                self.target_route_releases += 1
            elif event == "target_approach_terminal":
                self.target_approach_terminals += 1
            if event in (
                "target_route_failed",
                "target_route_semantic_replan_failed",
            ):
                self._begin_failure_episode_locked(
                    "target_route_failed",
                    "goal_manager",
                    payload,
                )
            record = dict(payload)
            record.pop("event", None)
            self._write("INFO" if event != "target_route_rejected" else "WARN",
                        "goal_arbitration", goal_event=event, **record)


    def on_perception_decision(self, message):
        """Persist detector-side acceptance/rejection evidence in this run log."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = {"event": "malformed_perception_decision", "raw": message.data}
        if not isinstance(payload, dict):
            payload = {"event": "malformed_perception_decision", "raw": message.data}
        event = str(payload.pop("event", "unknown"))
        with self.lock:
            if self.pose is not None:
                payload["metrics_pose"] = [round(value, 4) for value in self.pose]
            payload["perception_event"] = event
            level = "WARN" if event in (
                "empty_image", "empty_prompt", "inference_exception", "stale_source_image"
            ) else "INFO"
            self._write(level, "perception_decision", **payload)

    def on_scores(self, message):
        with self.lock:
            self.scores = {
                "total": float(message.s_total),
                "target": float(message.s_target),
                "env": float(message.s_env),
                "ctx": float(message.s_ctx),
                "detected": bool(message.detected),
            }

    def on_task_done(self, message):
        done = bool(message.data)
        with self.lock:
            if done != self.task_done:
                self.task_done = done
                if done:
                    self.task_done_ros = rospy.Time.now().to_sec()
                    # Runtime logging deliberately continues after the task
                    # reaches its terminal state. Preserve that raw evidence,
                    # while writing one immutable benchmark boundary so idle
                    # time cannot dilute rates in offline comparisons.
                    completion_snapshot = {
                        "schema_version": 1,
                        "boundary": "task_done_callback",
                        "boundary_ros_time": round(self.task_done_ros, 3),
                        "boundary_wall_elapsed_seconds": round(
                            time.monotonic() - self.start_wall, 3
                        ),
                        "metrics": self._snapshot(),
                    }
                    self._write(
                        "INFO",
                        "task_completed",
                        latency_seconds=round(self.task_done_ros - self.start_ros, 3),
                        completion_evidence=self.task_done_completion_evidence,
                        task_done_policy=(
                            (self.run_context.get("target_thresholds") or {}).get(
                                "done_policy"
                            )
                            if isinstance(getattr(self, "run_context", None), dict)
                            else None
                        ),
                        target_acquired_latency=(
                            None if self.target_first_seen_ros is None else
                            round(self.target_first_seen_ros - self.start_ros, 3)
                        ),
                        target_lock_latency=(
                            None if self.target_lock_ros is None else
                            round(self.target_lock_ros - self.start_ros, 3)
                        ),
                        completion_snapshot=completion_snapshot,
                    )
                self._write(
                    "INFO",
                    "task_done",
                    value=done,
                    completion_evidence=self.task_done_completion_evidence,
                )

    def on_navigation_hold(self, message):
        with self.lock:
            active = bool(message.data)
            if active == self.navigation_hold:
                return
            now = time.monotonic()
            if active:
                self.navigation_hold_events += 1
                self.navigation_hold_start_wall = now
                self.navigation_hold = True
                self._write(
                    "INFO",
                    "navigation_hold_start",
                    count=self.navigation_hold_events,
                    goal=None if self.goal is None else [round(value, 3) for value in self.goal],
                    pose=None if self.pose is None else [round(value, 3) for value in self.pose],
                )
            else:
                duration = (
                    0.0 if self.navigation_hold_start_wall is None
                    else max(0.0, now - self.navigation_hold_start_wall)
                )
                self.navigation_hold_duration_total += duration
                self.navigation_hold_start_wall = None
                self.navigation_hold = False
                self._write(
                    "INFO",
                    "navigation_hold_end",
                    duration_seconds=round(duration, 3),
                    total_duration_seconds=round(self.navigation_hold_duration_total, 3),
                )

    def on_map(self, message):
        with self.lock:
            self.map_stats = self._grid_stats(message)
            self._update_benchmark_coverage_locked(message)

    def on_global_costmap(self, message):
        with self.lock:
            self.last_global_costmap_wall = time.monotonic()
            self.global_costmap_stats = self._grid_stats(message)
            self.global_costmap_message = message

    def on_local_costmap(self, message):
        with self.lock:
            self.last_local_costmap_wall = time.monotonic()
            self.local_costmap_stats = self._grid_stats(message)
            self.local_costmap_message = message
