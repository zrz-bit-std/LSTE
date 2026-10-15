"""Shared route reassociation and disconnected-route handling."""

import math

import rospy


class GlobalFrontierExecutionRouteMixin:
    """Keep an active route valid across online-SLAM map updates."""

    def handle_disconnected_active_frontier(
        self, row, col, x, y, robot_map, now,
    ):
        """Hold a short-lived disconnection or abandon it with evidence."""
        if self.active_unreachable_since is None:
            self.active_unreachable_since = now
        held_for = now - self.active_unreachable_since
        waypoint_distance = self.active_waypoint_distance(robot_map)
        if getattr(self, "controller_owned_route_failure", False):
            rospy.logwarn_throttle(
                3.0,
                "Global frontier route is temporarily disconnected; retaining "
                "controller-owned lease route_id=%d held=%.1fs",
                self.active_route_id,
                held_for,
            )
            return (row, col, x, y, 0.0, 0.0), self.active_last_waypoint_map
        if (
            self.active_last_waypoint_map is not None
            and held_for <= self.unreachable_grace
            and waypoint_distance > 0.8
        ):
            rospy.logwarn_throttle(
                3.0,
                "Global frontier temporarily disconnected map=(%.2f,%.2f); "
                "holding last safe waypoint for %.1fs/%.1fs",
                x,
                y,
                held_for,
                self.unreachable_grace,
            )
            return (row, col, x, y, 0.0, 0.0), self.active_last_waypoint_map

        rospy.logwarn(
            "Global frontier abandoned map=(%.2f,%.2f): disconnected for %.1fs",
            x,
            y,
            held_for,
        )
        self.publish_status(
            "route_invalidated",
            reason="disconnected",
            goal=[round(float(x), 3), round(float(y), 3)],
            distance=round(float(math.hypot(x - robot_map[0], y - robot_map[1])), 3),
            elapsed=round(float(held_for), 3),
            path_distance=None,
            best_path_distance=(
                None if self.active_best_path_distance is None
                else round(float(self.active_best_path_distance), 3)
            ),
            best_goal_distance=(
                None if self.active_best_goal_distance is None
                else round(float(self.active_best_goal_distance), 3)
            ),
            last_progress_signal=self.active_last_progress_signal,
        )
        # This is a route/viewpoint failure, not evidence that the physical
        # unknown-side observation boundary was completed. Do this before the
        # release clears the active Attempt identity.
        if self.active_route_kind != "portal_transition":
            settle = getattr(self, "settle_active_work_item", None)
            if settle is not None:
                settle(now, "deferred", "disconnected")
        self.release_active_frontier(discard_prefetch=True)
        return None, None

    def reassociated_active_frontier_cell(self, snapshot, x, y):
        """Return the live route cell only when map and costmap still agree."""
        route_graph = snapshot.route_graph
        reassociated = self.nearest_reachable_cell(
            snapshot.message, route_graph.route_steps, x, y,
        )
        if (
            reassociated is not None
            and route_graph.validation is not None
            and self.candidate_costmap_distance(route_graph.validation, x, y) is None
        ):
            return None
        return reassociated
