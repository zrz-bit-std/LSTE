"""Execution and progress accounting for one local-egress route."""

import math

import rospy

from global_frontier_models import ActiveRouteObservation


class GlobalFrontierExecutionEgressMixin:
    """Return from a blocked branch through a physically reached anchor."""

    def observe_local_egress_route(self, snapshot, row, col, x, y):
        """Track a recovery anchor without mutating frontier/place evidence."""
        route_graph = snapshot.route_graph
        distance = math.hypot(x - snapshot.robot_map[0], y - snapshot.robot_map[1])
        route_distance = float(route_graph.route_steps[row, col]) * float(
            snapshot.message.info.resolution
        )
        if route_graph.validation is not None:
            costmap_distance = self.candidate_costmap_distance(
                route_graph.validation, x, y,
            )
            if costmap_distance is not None:
                route_distance = float(costmap_distance)
        route_progress = (
            self.active_best_path_distance is None
            or route_distance < self.active_best_path_distance - self.progress_epsilon
        )
        goal_progress = (
            self.active_best_goal_distance is None
            or distance < self.active_best_goal_distance - self.progress_epsilon
        )
        if route_progress:
            self.active_best_path_distance = route_distance
        if goal_progress:
            self.active_best_goal_distance = distance
            self.active_best_distance = distance
        self._record_active_route_progress(
            snapshot.now,
            route_progress,
            goal_progress,
            self._active_route_odom_detour_progress(),
            self._entered_novel_active_odom_cell(),
        )
        self._update_active_route_position(snapshot.robot_map)
        return ActiveRouteObservation(
            distance=distance,
            information=0.0,
            component=None,
            route_distance=route_distance,
        )

    def update_local_egress_route(self, snapshot, row, col, x, y):
        """Execute a proven-safe retreat before resuming global exploration."""
        reassociated = self.reassociated_active_frontier_cell(snapshot, x, y)
        if reassociated is None:
            return self.handle_disconnected_active_frontier(
                row, col, x, y, snapshot.robot_map, snapshot.now,
            )
        row, col = reassociated
        self.active_unreachable_since = None
        observation = self.observe_local_egress_route(snapshot, row, col, x, y)
        watchdog = self.evaluate_active_route_watchdogs(
            x, y, snapshot.robot_map, snapshot.now, True, None, 0.0,
        )
        controller_owns_failure = bool(
            getattr(self, "controller_owned_route_failure", False)
        )
        if watchdog.expired or (watchdog.stalled and not controller_owns_failure):
            self.deactivate_failed_active_frontier(
                x,
                y,
                observation.distance,
                snapshot.now,
                True,
                None,
                observation.route_distance,
                "local_egress_" + self.active_route_failure_reason(watchdog),
                watchdog,
            )
            return None, None
        endpoint_reached = (
            observation.distance <= self.endpoint_terminal_wait_radius
            and watchdog.waypoint_reached
        )
        terminal_required = self.mission_endpoint_only or self.persistent_execution
        if endpoint_reached and not terminal_required:
            self.publish_status(
                "local_egress_completed",
                route_id=int(self.active_route_id),
                recovery_goal=[round(float(x), 3), round(float(y), 3)],
            )
            self.release_active_frontier()
            return None, None
        if endpoint_reached:
            rospy.loginfo_throttle(
                2.0,
                "Global frontier reached local egress anchor; waiting for "
                "matching TEB terminal route_id=%d distance=%.2fm",
                self.active_route_id,
                observation.distance,
            )
        return (row, col, x, y, observation.route_distance, 0.0), None
