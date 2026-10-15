#!/usr/bin/env python3
"""Endpoint and turn-connector geometry for global-frontier commands."""

import math

import rospy


class GlobalFrontierRouteCommandGeometryMixin:
    """Choose only valid route geometry; do not publish or mutate route state."""

    def active_mission_endpoint(self, message, active_cell):
        """Return the stable mission endpoint for the current route lease."""
        if self.active_frontier is not None:
            _, _, x, y = self.active_frontier
            return float(x), float(y)
        return self.cell_xy(message, active_cell[0], active_cell[1])

    def select_route_command_cell(self, message, route_steps, active_cell):
        """Choose an endpoint or legacy rolling connector on the safe route."""
        row, col = active_cell[:2]
        remaining_path = float(route_steps[row, col]) * float(message.info.resolution)
        command_cell = row, col
        route_kind = "frontier_endpoint"
        if (
            not self.mission_endpoint_only
            and remaining_path > self.route_segment_distance
        ):
            horizon_cells = max(
                1,
                int(math.ceil(
                    self.route_segment_distance / message.info.resolution
                )),
            )
            candidate = self.waypoint_on_path(
                route_steps, row, col, horizon_cells
            )
            if candidate is not None:
                command_cell = candidate
                route_kind = "frontier_connector"
        return command_cell, route_kind, remaining_path

    def maybe_insert_turn_connector(
        self,
        message,
        route_steps,
        active_cell,
        mission_xy,
        command_xy,
        initial_heading,
        terminal_heading,
        remaining_path,
        robot_yaw_map,
    ):
        """Return a legacy yaw connector only for its explicit comparison mode."""
        if initial_heading is None or robot_yaw_map is None:
            turn_delta = None
        else:
            turn_delta = abs(self._angle_delta(initial_heading, robot_yaw_map))
        should_insert = (
            self.turn_execution_mode == "legacy_connector"
            and self.mission_endpoint_only
            and turn_delta is not None
            and turn_delta >= self.explicit_turn_connector_threshold
            and remaining_path > self.waypoint_release_radius
        )
        if not should_insert:
            return command_xy, terminal_heading, "frontier_endpoint", False
        connector_steps = max(
            1,
            int(math.ceil(
                self.turn_connector_distance
                / max(float(message.info.resolution), 1e-6)
            )),
        )
        connector_cell = self.waypoint_on_path(
            route_steps, active_cell[0], active_cell[1], connector_steps
        )
        if connector_cell is not None:
            connector_x, connector_y = self.cell_xy(
                message, connector_cell[0], connector_cell[1]
            )
            command_xy = float(connector_x), float(connector_y)
        else:
            # A short valid route does not justify manufacturing a point outside
            # the Navfn-validated route mask.
            command_xy = mission_xy
        rospy.loginfo(
            "Global frontier inserted explicit turn connector "
            "before reverse-facing branch: endpoint=(%.2f,%.2f) "
            "initial_heading=%.1fdeg terminal_heading=%.1fdeg "
            "turn_delta=%.1fdeg path=%.2fm",
            command_xy[0],
            command_xy[1],
            math.degrees(initial_heading),
            math.degrees(terminal_heading)
            if terminal_heading is not None else float("nan"),
            math.degrees(turn_delta),
            remaining_path,
        )
        return command_xy, initial_heading, "frontier_turn_connector", True
