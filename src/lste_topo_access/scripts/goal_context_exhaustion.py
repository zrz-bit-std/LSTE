#!/usr/bin/env python3
"""Bounded persistence for unproductive visual context pairs.

A context pair (for example desk + chair) is a search hint, not a navigation
destination.  Once the robot has inspected it and found no target, the same
pair must not be re-selected on every perception tick.  This small pure helper
keeps that decision out of the ROS node so the lifecycle is easy to test.
"""

import time


class ContextPairExhaustion:
    """Remember inspected context pairs for a bounded cooldown."""

    def __init__(self, cooldown=120.0, limit=64):
        self.cooldown = max(1.0, float(cooldown))
        self.limit = max(1, int(limit))
        self._records = {}

    @staticmethod
    def key(task_id, left_label, right_label):
        """Return a stable pair identity within one mission.

        Detector ordering can swap the two context detections between frames.
        The pair is the same physical hint either way, so exhaustion must be
        order-independent; otherwise the same desk/chair pair is re-selected
        forever under alternating left/right order.
        """
        first = str(left_label or "").strip().lower()
        second = str(right_label or "").strip().lower()
        if second < first:
            first, second = second, first
        return (
            str(task_id or "").strip().lower(),
            first,
            second,
        )

    def mark(self, key, now, cooldown=None):
        """Pause one pair until ``now + cooldown``."""
        if key is None:
            return None
        try:
            now = float(now)
        except (TypeError, ValueError):
            return None
        try:
            seconds = self.cooldown if cooldown is None else float(cooldown)
        except (TypeError, ValueError):
            seconds = self.cooldown
        seconds = max(1.0, seconds)
        previous = self._records.get(key)
        count = 1
        if isinstance(previous, tuple) and len(previous) == 2:
            try:
                count = int(previous[1]) + 1
            except (TypeError, ValueError):
                count = 1
        until = now + seconds
        self._records[key] = (until, count)
        while len(self._records) > self.limit:
            oldest = min(
                self._records,
                key=lambda item: self._records[item][0],
            )
            self._records.pop(oldest, None)
        return until

    def is_exhausted(self, key, now):
        """Return whether the pair is still inside its cooldown."""
        if key is None:
            return False
        record = self._records.get(key)
        if record is None:
            return False
        try:
            now = float(now)
        except (TypeError, ValueError):
            return True
        until = float(record[0])
        if until <= now:
            self._records.pop(key, None)
            return False
        return True

    def clear(self, key=None):
        """Clear one pair, or every pair when ``key`` is omitted."""
        if key is None:
            self._records.clear()
            return 0
        return 1 if self._records.pop(key, None) is not None else 0

    def prune(self, now):
        """Drop every expired record."""
        try:
            now = float(now)
        except (TypeError, ValueError):
            return 0
        expired = [
            key for key, record in self._records.items()
            if float(record[0]) <= now
        ]
        for key in expired:
            self._records.pop(key, None)
        return len(expired)

    def snapshot(self):
        return {
            key: {"until": float(value[0]), "count": int(value[1])}
            for key, value in self._records.items()
        }
