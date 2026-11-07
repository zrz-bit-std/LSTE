#!/usr/bin/env python3
"""Wall-clock release for a Place coverage gate that cannot start.

Place observation budgets stay in simulation time: they gate what the robot
actually sees, and shortening them would change the exploration policy.  A
simulator that runs slower than real time, however, can consume the mission's
wall-clock watchdog while a simulation budget has barely advanced.  A Place
that still lacks its first observation and has no actionable observation
WorkItem then holds the whole graph with no owner.

This helper measures only that pathological wall-clock dwell.  It never
shortens a simulation budget that is still producing physical progress.
"""

import math


class PlaceWallFallback:
    """Bound how long one uncovered Place may hold the graph in wall time."""

    def __init__(self, seconds=45.0, progress_radius=0.35):
        self.seconds = max(5.0, float(seconds))
        self.progress_radius = max(0.05, float(progress_radius))
        self.place_id = None
        self.anchor = None
        self.since = None

    def reset(self):
        self.place_id = None
        self.anchor = None
        self.since = None

    def observe(self, place_id, waiting, pose_xy, now_wall):
        """Return the Place id whose wall budget expired, or ``None``.

        ``waiting`` is true only while the current Place still lacks its first
        observation and no observation WorkItem can be dispatched.  Any
        physical displacement beyond ``progress_radius`` restarts the dwell,
        so a robot that is actually driving never expires this budget.
        """
        try:
            now_wall = float(now_wall)
        except (TypeError, ValueError):
            return None
        if place_id is None or not waiting or pose_xy is None:
            self.reset()
            return None
        try:
            place_id = int(place_id)
            x = float(pose_xy[0])
            y = float(pose_xy[1])
        except (TypeError, ValueError, IndexError):
            self.reset()
            return None
        if place_id != self.place_id or self.anchor is None:
            self.place_id = place_id
            self.anchor = (x, y)
            self.since = now_wall
            return None
        if math.hypot(x - self.anchor[0], y - self.anchor[1]) > self.progress_radius:
            self.anchor = (x, y)
            self.since = now_wall
            return None
        if self.since is None:
            self.since = now_wall
            return None
        if now_wall - self.since >= self.seconds:
            return place_id
        return None


__all__ = ["PlaceWallFallback"]
