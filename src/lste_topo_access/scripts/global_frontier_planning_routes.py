"""Small route-geometry helpers shared by selection and prefetching."""

import math

import numpy as np


class GlobalFrontierPlanningRouteMixin:

    def route_headings(self, message, steps, seed, target, robot_xy):
        """Return initial and terminal tangent bearings for one BFS route."""
        path = self.route_path(steps, seed, target)
        if len(path) < 2:
            return None, None
        resolution = float(message.info.resolution)
        anchor_index = min(
            len(path) - 1,
            max(1, int(math.ceil(0.5 / max(resolution, 1e-6)))),
        )
        anchor_x, anchor_y = self.cell_xy(
            message, path[anchor_index][0], path[anchor_index][1],
        )
        if robot_xy is None or math.hypot(anchor_x - robot_xy[0], anchor_y - robot_xy[1]) < 1e-3:
            first_x, first_y = self.cell_xy(message, path[1][0], path[1][1])
            initial = math.atan2(first_y - robot_xy[1], first_x - robot_xy[0]) if robot_xy else None
        else:
            initial = math.atan2(anchor_y - robot_xy[1], anchor_x - robot_xy[0])
        previous_x, previous_y = self.cell_xy(message, path[-2][0], path[-2][1])
        final_x, final_y = self.cell_xy(message, path[-1][0], path[-1][1])
        terminal = math.atan2(final_y - previous_y, final_x - previous_x)
        return initial, terminal

    def _candidate_route_heading_delta(
        self, message, steps, seed, row, col, robot_xy, robot_yaw,
    ):
        """Return the first-route-tangent turn required for one candidate.

        This intentionally falls back to endpoint bearing only when no BFS
        seed is available.  The normal online-SLAM path always supplies a
        seed, so the selection decision is based on an actual connected route
        rather than a line-of-sight assumption.
        """
        endpoint_x, endpoint_y = self.cell_xy(message, row, col)
        fallback = self.heading_delta(endpoint_x, endpoint_y, robot_xy, robot_yaw)
        if seed is None or robot_yaw is None:
            return fallback
        initial, _ = self.route_headings(
            message, steps, seed, (row, col), robot_xy
        )
        if initial is None:
            return fallback
        return abs(self._angle_delta(initial, robot_yaw))

    def prefetched_route_continues_active(
        self, steps, seed, active_cell, pending_cell
    ):
        """Return whether a pending endpoint genuinely extends this route.

        An endpoint that happens to be near the robot is not necessarily the
        continuation of the action currently owned by move_base.  Replacing
        the action before its endpoint is reached is safe only when the new
        deterministic map route contains the full active route as a prefix.
        This is a topological contract, not a heading or distance tuning gate.
        """
        active_path = self.route_path(steps, seed, active_cell)
        pending_path = self.route_path(steps, seed, pending_cell)
        if len(active_path) < 2 or len(pending_path) <= len(active_path):
            return False
        return pending_path[:len(active_path)] == active_path

    def nearest_reachable_cell(self, message, steps, x, y):
        """Reassociate a remembered world-space frontier with a new map grid.

        Gmapping can turn the exact frontier cell into known free space, or
        shift the local cell boundary by one cell, while the route remains
        connected. Reusing the nearest reachable cell avoids treating that
        normal map update as a branch failure.
        """
        if steps is None:
            return None
        resolution = float(message.info.resolution)
        target_col = int((x - message.info.origin.position.x) / resolution)
        target_row = int((y - message.info.origin.position.y) / resolution)
        radius = max(1, int(math.ceil(self.active_reassociation_radius / resolution)))
        r0 = max(0, target_row - radius)
        r1 = min(steps.shape[0], target_row + radius + 1)
        c0 = max(0, target_col - radius)
        c1 = min(steps.shape[1], target_col + radius + 1)
        reachable = np.argwhere(steps[r0:r1, c0:c1] >= 0)
        if reachable.size == 0:
            return None
        reachable[:, 0] += r0
        reachable[:, 1] += c0
        distance = (reachable[:, 0] - target_row) ** 2 + (reachable[:, 1] - target_col) ** 2
        selected = reachable[int(np.argmin(distance))]
        return int(selected[0]), int(selected[1])


