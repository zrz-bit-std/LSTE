#!/usr/bin/env python3
"""Route-lifecycle decisions for a stable global-frontier command."""

import math

import rospy

from global_frontier_models import ActiveRouteCommand
from global_frontier_route_command_geometry import (
    GlobalFrontierRouteCommandGeometryMixin,
)


class GlobalFrontierRouteCommandResolutionMixin(
    GlobalFrontierRouteCommandGeometryMixin,
):
    """Apply command-lease state while preserving mission route semantics."""

    def should_hold_active_waypoint(self, robot_map):
        """Decide whether an already-published command must remain frozen."""
        previous = self.active_last_waypoint_map
        if previous is None:
            return False, float("inf"), False
        previous_distance = math.hypot(
            previous[0] - robot_map[0],
            previous[1] - robot_map[1],
        )
        turn_still_pending = (
            self.active_route_kind == "frontier_turn_connector"
            and not self.turn_connector_released
        )
        # A segmented route advances only after its matching MoveBase success
        # arrives. Replacing its command inside the release radius creates a
        # mid-action preemption and loses the local map observation that the
        # next segment is meant to use.
        connector_terminal_pending = (
            not self.mission_endpoint_only
            and self.active_route_kind not in (
                "frontier_turn_connector",
                "local_egress",
            )
        )
        mission_endpoint_is_frozen = (
            (
                self.mission_endpoint_only
            # A portal is one directed physical edge.  Segmenting it into
            # rolling map waypoints lets SLAM updates move its terminal and
            # turns one doorway crossing into several unrelated actions.
                or self.active_route_kind == "portal_transition"
            )
            and not (
                self.active_route_kind == "frontier_turn_connector"
                and self.turn_connector_released
            )
        )
        should_hold = (
            mission_endpoint_is_frozen
            or connector_terminal_pending
            or previous_distance > self.waypoint_release_radius
            or turn_still_pending
        )
        return should_hold, previous_distance, turn_still_pending

    def resolve_active_route_command(
        self, message, active_cell, held_waypoint_map, route_steps, seed,
        robot_map, robot_yaw_map,
    ):
        """Resolve one stable command without publishing ROS state."""
        mission_xy = self.active_mission_endpoint(message, active_cell)
        remaining_path = (
            float(route_steps[active_cell[0], active_cell[1]])
            * float(message.info.resolution)
        )
        if held_waypoint_map is not None:
            return ActiveRouteCommand(
                mission_xy=mission_xy,
                command_xy=held_waypoint_map,
                command_yaw=self.active_last_waypoint_yaw,
                route_kind=self.active_route_kind,
                remaining_path=remaining_path,
                mission_route_kind=getattr(
                    self, "active_mission_route_kind", self.active_route_kind,
                ),
            )
        should_hold, previous_distance, turn_still_pending = (
            self.should_hold_active_waypoint(robot_map)
        )
        if should_hold:
            rospy.loginfo_throttle(
                3.0,
                "Global frontier holding route command kind=%s "
                "(%.2f,%.2f) distance=%.2fm release_radius=%.2fm "
                "turn_pending=%s",
                self.active_route_kind,
                self.active_last_waypoint_map[0],
                self.active_last_waypoint_map[1],
                previous_distance,
                self.waypoint_release_radius,
                turn_still_pending,
            )
            return ActiveRouteCommand(
                mission_xy=mission_xy,
                command_xy=self.active_last_waypoint_map,
                command_yaw=self.active_last_waypoint_yaw,
                route_kind=self.active_route_kind,
                remaining_path=remaining_path,
                mission_route_kind=getattr(
                    self, "active_mission_route_kind", self.active_route_kind,
                ),
            )
        command_cell, command_kind, remaining_path = self.select_route_command_cell(
            message, route_steps, active_cell
        )
        # ``portal_transition`` is a mission-level contract.  Endpoint versus
        # connector describes geometry only and must not erase that meaning.
        mission_route_kind = getattr(
            self, "active_mission_route_kind", self.active_route_kind,
        )
        preserve_portal_identity = mission_route_kind == "portal_transition"
        route_kind = (
            "portal_transition"
            if preserve_portal_identity
            else command_kind
        )
        if mission_route_kind == "local_egress":
            route_kind = "local_egress"
        initial_heading, terminal_heading = self.route_headings(
            message, route_steps, seed, command_cell, robot_map
        )
        command_x, command_y = self.cell_xy(
            message, command_cell[0], command_cell[1]
        )
        command_xy = float(command_x), float(command_y)
        # Endpoint-only execution normally freezes a route at its mission
        # goal.  A portal has the same contract even with the legacy
        # segmented-frontier setting: one directed doorway traversal cannot
        # be decomposed into changing map-frame subgoals.
        mission_endpoint_is_frozen = (
            self.mission_endpoint_only
            or mission_route_kind == "portal_transition"
        )
        if mission_endpoint_is_frozen:
            command_xy = mission_xy
        command_yaw = terminal_heading
        if (
            command_kind == "frontier_endpoint"
            and not preserve_portal_identity
            and mission_route_kind != "local_egress"
        ):
            command_xy, command_yaw, route_kind, inserted_turn_connector = (
                self.maybe_insert_turn_connector(
                    message,
                    route_steps,
                    active_cell,
                    mission_xy,
                    command_xy,
                    initial_heading,
                    terminal_heading,
                    remaining_path,
                    robot_yaw_map,
                )
            )
        else:
            inserted_turn_connector = False
        self.active_last_waypoint_map = command_xy
        self.active_last_waypoint_yaw = command_yaw
        self.active_route_kind = route_kind
        self.turn_connector_released = not inserted_turn_connector
        rospy.loginfo(
            "Global frontier route command kind=%s command=(%.2f,%.2f) "
            "mission=(%.2f,%.2f) remaining=%.2fm horizon=%.2fm yaw=%s",
            route_kind,
            command_xy[0],
            command_xy[1],
            mission_xy[0],
            mission_xy[1],
            remaining_path,
            self.route_segment_distance,
            "none" if command_yaw is None else "%.1fdeg" % math.degrees(command_yaw),
        )
        return ActiveRouteCommand(
            mission_xy=mission_xy,
            command_xy=command_xy,
            command_yaw=command_yaw,
            route_kind=route_kind,
            remaining_path=remaining_path,
            mission_route_kind=mission_route_kind,
        )
