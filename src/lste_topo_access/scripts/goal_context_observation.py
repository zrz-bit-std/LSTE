#!/usr/bin/env python3
"""Bounded in-place observation for a stalled context-pair approach.

A context pair is a search hint.  When the approach goal cannot be reached,
the robot still has a useful fallback: stop, sweep the camera, and decide
whether the hint produced target evidence.  If it did not, the caller must
abandon the pair instead of waiting for the external watchdog.
"""

import math


class ContextObservationPolicy:
    """Small pure state machine used by the Goal Manager."""

    def __init__(
        self,
        trigger_seconds=4.0,
        observe_seconds=10.0,
        hold_seconds=2.0,
        reached_radius=0.55,
        progress_epsilon=0.08,
        sweep_deg=(0.0, 30.0, -30.0, 60.0, -60.0),
    ):
        self.trigger_seconds = max(0.5, float(trigger_seconds))
        self.observe_seconds = max(1.0, float(observe_seconds))
        self.hold_seconds = max(0.2, float(hold_seconds))
        self.reached_radius = max(0.05, float(reached_radius))
        self.progress_epsilon = max(0.01, float(progress_epsilon))
        self.sweep_deg = tuple(float(value) for value in sweep_deg)
        self.reset()

    def reset(self):
        self.state = "idle"
        self.last_pose = None
        self.last_progress_at = None
        self.observe_started = None
        self.started_at = None

    def _pose(self, value):
        try:
            return float(value[0]), float(value[1])
        except (TypeError, ValueError, IndexError):
            return None

    def update(
        self,
        now,
        pose_xy,
        goal_xy,
        target_detected=False,
        locked=False,
    ):
        """Return one action: approach / observe / abandon / target."""
        try:
            now = float(now)
        except (TypeError, ValueError):
            now = 0.0
        pose = self._pose(pose_xy)
        goal = self._pose(goal_xy)
        if pose is None or goal is None:
            return {"state": "approach", "distance": None}

        if target_detected or locked:
            self.state = "target"
            return {"state": "target"}

        distance = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
        if self.state in ("idle", "target", "abandon"):
            self.state = "approach"
            self.last_pose = pose
            self.last_progress_at = now
            self.started_at = now
            self.observe_started = None

        if self.state == "approach":
            moved = (
                self.last_pose is None
                or math.hypot(
                    pose[0] - self.last_pose[0],
                    pose[1] - self.last_pose[1],
                ) >= self.progress_epsilon
            )
            if moved:
                self.last_pose = pose
                self.last_progress_at = now
            if (
                distance <= self.reached_radius
                or (
                    self.last_progress_at is not None
                    and now - self.last_progress_at >= self.trigger_seconds
                )
            ):
                self.state = "observe"
                self.observe_started = now
            else:
                return {"state": "approach", "distance": distance}

        if self.state == "observe":
            elapsed = max(0.0, now - float(self.observe_started or now))
            if elapsed >= self.observe_seconds:
                self.state = "abandon"
                return {"state": "abandon", "elapsed": elapsed}
            index = int(elapsed / self.hold_seconds) % len(self.sweep_deg)
            return {
                "state": "observe",
                "yaw_offset": math.radians(self.sweep_deg[index]),
                "elapsed": elapsed,
                "distance": distance,
            }

        if self.state == "abandon":
            return {"state": "abandon"}
        if self.state == "target":
            return {"state": "target"}
        return {"state": "approach", "distance": distance}
