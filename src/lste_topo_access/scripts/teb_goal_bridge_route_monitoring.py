"""Navfn and rolling-costmap observations for the TEB goal bridge."""

import math

from clock_provider import now_for


class TebGoalBridgeRouteMonitoringMixin:
    """Track progress along the validated global route and local map."""

    def _path_remaining_distance(self, points, x, y):
        """Return remaining arc length after projecting a pose onto a path."""
        if not points:
            return None
        if len(points) == 1:
            return math.hypot(points[0][0] - x, points[0][1] - y)
        suffix = [0.0] * len(points)
        for index in range(len(points) - 2, -1, -1):
            suffix[index] = suffix[index + 1] + math.hypot(
                points[index + 1][0] - points[index][0],
                points[index + 1][1] - points[index][1],
            )
        nearest_distance_sq = float("inf")
        remaining = None
        for index in range(len(points) - 1):
            ax, ay = points[index]
            bx, by = points[index + 1]
            dx, dy = bx - ax, by - ay
            length = math.hypot(dx, dy)
            if length <= 1e-6:
                continue
            projection = ((x - ax) * dx + (y - ay) * dy) / (length * length)
            projection = min(1.0, max(0.0, projection))
            px, py = ax + projection * dx, ay + projection * dy
            distance_sq = (x - px) ** 2 + (y - py) ** 2
            if distance_sq < nearest_distance_sq:
                nearest_distance_sq = distance_sq
                remaining = (1.0 - projection) * length + suffix[index + 1]
        return remaining

    def _update_navfn_path_progress_locked(self, now):
        if self.active_feedback_pose is None or not self.active_navfn_plan_points:
            return
        x, y, _ = self.active_feedback_pose
        remaining = self._path_remaining_distance(self.active_navfn_plan_points, x, y)
        if remaining is None:
            return
        self.active_navfn_remaining = remaining
        if (
            self.active_navfn_best_remaining is None
            or remaining < self.active_navfn_best_remaining - self.progress_epsilon
        ):
            self.active_navfn_best_remaining = remaining
            self.active_navfn_progress_monotonic = now

    def on_navfn_plan(self, message):
        """Bind the health watchdog to the current action's Navfn route."""
        with self.lock:
            if (
                not self.action_active
                or self.active_goal_global is None
                or len(message.poses) < 2
            ):
                return
            frame = (message.header.frame_id or message.poses[-1].header.frame_id)
            if (frame or "").strip().lstrip("/") != self.global_frame:
                return
            points = [
                (float(pose.pose.position.x), float(pose.pose.position.y))
                for pose in message.poses
            ]
            endpoint = points[-1]
            goal = self.active_goal_global.pose.position
            if math.hypot(endpoint[0] - goal.x, endpoint[1] - goal.y) > 0.50:
                return
            self.active_navfn_plan_points = points
            self.active_navfn_plan_endpoint = [round(endpoint[0], 3), round(endpoint[1], 3)]
            self.active_navfn_remaining = None
            self.active_navfn_best_remaining = None
            self.active_navfn_progress_monotonic = 0.0
            self._update_navfn_path_progress_locked(now_for(self))

    def on_local_costmap(self, message):
        """Keep the latest rolling lidar map for route admission."""
        with self.lock:
            self.local_costmap = message
            self.local_costmap_received_monotonic = now_for(self)

    @staticmethod
    def _local_costmap_cell(message, x, y):
        """Return the row-major cell index for one local-costmap point."""
        resolution = float(message.info.resolution)
        if resolution <= 1e-9:
            return None
        col = int(math.floor((float(x) - message.info.origin.position.x) / resolution))
        row = int(math.floor((float(y) - message.info.origin.position.y) / resolution))
        if row < 0 or col < 0 or row >= int(message.info.height) or col >= int(message.info.width):
            return None
        return row * int(message.info.width) + col
