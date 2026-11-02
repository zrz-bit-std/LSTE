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
        authentication_approach_pending = bool(
            getattr(self, "active_portal_authentication_phase", "") == "entering"
            and getattr(self, "active_portal_authentication_entry_approach", False)
        )
        connector_terminal_pending = (
            not self.mission_endpoint_only
            and self.active_route_kind not in (
                "frontier_turn_connector",
                "local_egress",
            )
            and not authentication_approach_pending
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
        mission_endpoint_is_frozen = (
            mission_endpoint_is_frozen and not authentication_approach_pending
        )
        approach_release_radius = (
            max(float(self.waypoint_release_radius), 0.50)
            if authentication_approach_pending
            else float(self.waypoint_release_radius)
        )
        should_hold = (
            mission_endpoint_is_frozen
            or connector_terminal_pending
            or previous_distance > approach_release_radius
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
        auth = getattr(self, "active_portal_authentication", lambda: None)()
        authentication_turn_pending = bool(
            getattr(self, "active_portal_authentication_phase", "") == "turning"
            and auth is not None and getattr(auth, "target_yaw", None) is not None
        )
        # A frozen exploration endpoint must yield to the doorway-auth turn:
        # the turn keeps one fixed XY anchor, but its yaw is a new action phase.
        authentication_approach_pending = bool(
            getattr(self, "active_portal_authentication_phase", "") == "entering"
            and getattr(self, "active_portal_authentication_entry_approach", False)
        )
        if (held_waypoint_map is not None and not authentication_turn_pending
                and not authentication_approach_pending):
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
        if should_hold and not authentication_turn_pending:
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
        work_normal = getattr(self, "active_work_item_normal_xy", None)
        if getattr(self, "active_work_item_id", None) is not None and work_normal is not None:
            view_yaw = getattr(self, "active_work_item_map_yaw", None)
            if view_yaw is None:
                frame = message.header.frame_id or "map"
                origin_map = self.transform_xy(frame, "odom", 0.0, 0.0)
                normal_map = self.transform_xy(
                    frame, "odom", float(work_normal[0]), float(work_normal[1]),
                )
                if origin_map is not None and normal_map is not None:
                    view_yaw = math.atan2(
                        normal_map[1] - origin_map[1],
                        normal_map[0] - origin_map[0],
                    )
                    self.active_work_item_map_yaw = view_yaw
            if view_yaw is not None:
                command_yaw = float(view_yaw)
        if getattr(self, "active_portal_authentication_phase", "") == "turning":
            auth = getattr(self, "active_portal_authentication", lambda: None)()
            if auth is not None and auth.target_yaw is not None:
                frame = message.header.frame_id or "map"
                inside = auth.inside_xy
                target_odom = (
                    inside[0] + math.cos(auth.target_yaw),
                    inside[1] + math.sin(auth.target_yaw),
                )
                inside_map = self.transform_xy(frame, "odom", *inside)
                target_map = self.transform_xy(frame, "odom", *target_odom)
                if inside_map is not None and target_map is not None:
                    # Keep one map-frame waypoint for this atomic turn. Using
                    # the latest robot pose here changes the action identity on
                    # every SLAM update, so the supervisor cancels the fixed-yaw
                    # turn before it can make progress.
                    command_xy = getattr(auth, "turn_map_xy", None)
                    if command_xy is None:
                        pose = getattr(self, "pose_odom", None)
                        current_map = (None if pose is None else self.transform_xy(
                            frame, "odom", float(pose.x), float(pose.y)))
                        command_xy = tuple(
                            inside_map if current_map is None else current_map
                        )
                        auth.turn_map_xy = command_xy
                    command_yaw = math.atan2(
                        target_map[1] - inside_map[1],
                        target_map[0] - inside_map[0],
                    )
                    route_kind = "frontier_turn_connector"
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
