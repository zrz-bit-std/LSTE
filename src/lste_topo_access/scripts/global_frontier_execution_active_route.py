"""Normal frontier-route execution after a route lease is active."""

import rospy


class GlobalFrontierExecutionActiveRouteMixin:
    """Observe, supervise, and retain one active frontier action."""

    def observe_connected_active_route(
        self, snapshot, row, col, x, y, portal_transition,
    ):
        """Collect map progress and watchdog evidence for a live route."""
        route_graph = snapshot.route_graph
        map_context = snapshot.map_context
        observation = self.observe_active_route(
            snapshot.message,
            route_graph.route_steps,
            route_graph.validation,
            map_context.unknown,
            map_context.components,
            row,
            col,
            x,
            y,
            snapshot.robot_map,
            snapshot.now,
            portal_transition,
        )
        if observation is None:
            # A persistent Portal can be committed and released from the
            # physical crossing observer before this method reaches the
            # watchdog. The next selector tick owns the successor route.
            return None, None
        watchdog = self.evaluate_active_route_watchdogs(
            x,
            y,
            snapshot.robot_map,
            snapshot.now,
            portal_transition,
            observation.component,
            observation.information,
        )
        return observation, watchdog

    def prefetch_or_promote_connected_route(
        self, snapshot, row, col, x, y, observation,
    ):
        """Cache a successor and return it only after a safe early handoff."""
        route_graph = snapshot.route_graph
        map_context = snapshot.map_context
        if not getattr(self, "graph_route_planner_enabled", False):
            # The legacy selector owns successor prefetching. The durable graph
            # method selects its next obligation at the terminal boundary;
            # running both selectors here repeatedly scans the full frontier
            # while the graph lifecycle correctly defers cross-Place handoff.
            self.prefetch_active_route_successor(
                snapshot.message,
                route_graph.strict_steps,
                route_graph.strict_free,
                route_graph.frontier,
                route_graph.frontier_free,
                map_context.unknown,
                map_context.occupied,
                map_context.components,
                route_graph.validation,
                route_graph.route_steps,
                route_graph.seed,
                snapshot.robot_map,
                snapshot.robot_yaw_map,
                snapshot.now,
                row,
                col,
                x,
                y,
                observation.distance,
            )
        early_promoted = self.try_early_prefetch_promotion(
            snapshot.message,
            route_graph.route_steps,
            route_graph.seed,
            snapshot.robot_map,
            snapshot.now,
            route_graph.validation,
            observation.component,
            row,
            col,
            x,
            y,
            observation.distance,
        )
        return (
            None if early_promoted is None
            else self.active_cell_from_promoted_frontier(
                route_graph.validation,
                route_graph.route_steps,
                snapshot.message.info.resolution,
                early_promoted,
            )
        )

    def update_connected_active_frontier(
        self, snapshot, row, col, x, y, portal_transition,
    ):
        """Apply the active-route lifecycle without replacing its action."""
        route_graph = snapshot.route_graph
        map_context = snapshot.map_context
        observation, watchdog = self.observe_connected_active_route(
            snapshot, row, col, x, y, portal_transition,
        )
        if observation is None:
            return None, None
        # No-gain dwell is an observation diagnostic. The action remains the
        # bridge's property until its route-id-matched terminal arrives.
        early_promoted = self.prefetch_or_promote_connected_route(
            snapshot, row, col, x, y, observation,
        )
        if early_promoted is not None:
            return early_promoted, None
        controller_owns_failure = bool(
            getattr(self, "controller_owned_route_failure", False)
        )
        should_keep_route = (
            not watchdog.expired
            and (controller_owns_failure or not watchdog.stalled)
            and (
                observation.distance > self.endpoint_terminal_wait_radius
                or not watchdog.waypoint_reached
            )
        )
        if not should_keep_route:
            return self.resolve_active_route_terminal_or_failure(
                row,
                col,
                x,
                y,
                observation.distance,
                watchdog.waypoint_reached,
                route_graph.route_steps,
                snapshot.message.info.resolution,
                snapshot.now,
                portal_transition,
                observation.component,
                observation.route_distance,
                watchdog,
            ), None
        if self.release_if_active_frontier_observed(
            snapshot.message,
            map_context.known_free,
            map_context.unknown,
            row,
            col,
            x,
            y,
            snapshot.robot_map,
            observation.distance,
            observation.route_distance,
            observation.component,
        ):
            return None, None
        active_cell = (
            row,
            col,
            x,
            y,
            route_graph.route_steps[row, col] * snapshot.message.info.resolution,
            0.0,
        )
        if observation.distance <= self.endpoint_terminal_wait_radius:
            rospy.loginfo_throttle(
                3.0,
                "Global frontier holding completed frontier until short "
                "waypoint is reached distance=%.2fm waypoint_distance=%.2fm "
                "release_radius=%.2fm",
                observation.distance,
                watchdog.waypoint_distance,
                self.waypoint_release_radius,
            )
        return active_cell, None

    def update_active_frontier(self, snapshot):
        """Supervise an existing route without selecting a fresh frontier."""
        if self.active_frontier is None:
            return None, None
        row, col, x, y = self.active_frontier
        if self.active_route_kind == "local_egress":
            return self.update_local_egress_route(snapshot, row, col, x, y)
        reassociated = self.reassociated_active_frontier_cell(snapshot, x, y)
        if reassociated is None:
            return self.handle_disconnected_active_frontier(
                row, col, x, y, snapshot.robot_map, snapshot.now,
            )
        self.active_unreachable_since = None
        return self.update_connected_active_frontier(
            snapshot,
            reassociated[0],
            reassociated[1],
            x,
            y,
            self.active_route_kind == "portal_transition",
        )
