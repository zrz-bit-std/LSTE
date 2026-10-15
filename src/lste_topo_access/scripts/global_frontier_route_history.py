"""Reached-position history for deterministic local egress recovery."""

import collections
import math


class ReachedRouteHistory:
    """Keep sparse, physically reached poses for one route lease.

    The history is deliberately not a planned path.  A stalled route may only
    retreat to one of these recorded base positions, which preserves the
    recovery contract even while SLAM updates alter the grid around it.
    """

    def __init__(self, max_points=32):
        self._points = collections.deque(maxlen=max(1, int(max_points)))

    def begin(self, pose):
        """Start a new route lease at the current map-frame pose."""
        self.clear()
        self._points.append(self._normalise(pose))

    def remember(self, pose, minimum_spacing):
        """Record a pose only after meaningful map-frame displacement."""
        point = self._normalise(pose)
        if not self._points or self._distance(point, self._points[-1]) >= float(
            minimum_spacing
        ):
            self._points.append(point)

    def nearest_exit_anchor(self, current_pose, minimum_distance):
        """Return the nearest reached point that exits the current local trap."""
        if current_pose is None:
            return None
        current = self._normalise(current_pose)
        candidates = [
            (self._distance(point, current), point)
            for point in self._points
            if self._distance(point, current) >= float(minimum_distance)
        ]
        if not candidates:
            return None
        return min(candidates, key=lambda item: item[0])[1]

    def clear(self):
        self._points.clear()

    def __len__(self):
        return len(self._points)

    @staticmethod
    def _normalise(pose):
        return float(pose[0]), float(pose[1])

    @staticmethod
    def _distance(first, second):
        return math.hypot(first[0] - second[0], first[1] - second[1])
