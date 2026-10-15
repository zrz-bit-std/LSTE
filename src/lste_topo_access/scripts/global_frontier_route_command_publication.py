#!/usr/bin/env python3
"""ROS reporting and publication for resolved global-frontier commands."""

import math

import rospy
from geometry_msgs.msg import PoseStamped

from global_frontier_portal_crossing import portal_execution_goal


class GlobalFrontierRouteCommandPublicationMixin:
    """Publish already-resolved route commands without changing their geometry."""

    def active_route_command_changed(self, command):
        """Compare a command with the last lifecycle status publication."""
        return (
            self.last_status_command_map is None
            or math.hypot(
                command.command_xy[0] - self.last_status_command_map[0],
                command.command_xy[1] - self.last_status_command_map[1],
            ) > 0.05
            or self.active_route_kind != command.route_kind
            or (
                command.command_yaw is not None
                and (
                    self.last_status_command_yaw is None
                    or abs(self._angle_delta(
                        command.command_yaw, self.last_status_command_yaw
                    )) > 0.08
                )
            )
            or self.last_status_mission_map is None
            or math.hypot(
                command.mission_xy[0] - self.last_status_mission_map[0],
                command.mission_xy[1] - self.last_status_mission_map[1],
            ) > 0.20
        )

    def publish_active_route_status(self, command):
        """Publish a route lifecycle update only when the command changed."""
        if not self.active_route_command_changed(command):
            return
        self.publish_status(
            "route_command",
            route_kind=self.active_route_kind,
            mission_route_kind=command.mission_route_kind,
            route_id=int(self.active_route_id),
            command_goal=[
                round(float(command.command_xy[0]), 3),
                round(float(command.command_xy[1]), 3),
            ],
            command_yaw=(
                None
                if command.command_yaw is None
                else round(float(command.command_yaw), 3)
            ),
            mission_goal=[
                round(float(command.mission_xy[0]), 3),
                round(float(command.mission_xy[1]), 3),
            ],
            path_remaining=round(float(command.remaining_path), 3),
        )
        self.last_status_command_map = command.command_xy
        self.last_status_command_yaw = command.command_yaw
        self.last_status_mission_map = command.mission_xy

    def publish_active_route_pose(self, message, command):
        """Serialize a route command as the ROS goal and companion command."""
        map_frame = message.header.frame_id or "map"
        frame_id, execution_xy = portal_execution_goal(
            self.active_route_kind,
            map_frame,
            command.command_xy,
            getattr(self, "active_portal_destination_odom_xy", None),
        )
        # A portal terminal is physical. Its target is frozen in odom at
        # activation, so do not attach a map-frame tangent to that pose. TEB
        # has an unconstrained endpoint yaw for frontier routes anyway.
        execution_yaw = (
            None if frame_id == "odom" else command.command_yaw
        )
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = frame_id
        goal.pose.position.x, goal.pose.position.y = execution_xy
        if execution_yaw is None:
            goal.pose.orientation.w = 1.0
        else:
            goal.pose.orientation.z = math.sin(0.5 * execution_yaw)
            goal.pose.orientation.w = math.cos(0.5 * execution_yaw)
        self.publish_route_command(
            goal.header.frame_id,
            execution_xy[0],
            execution_xy[1],
            execution_yaw,
            command.route_kind,
            command.mission_route_kind,
        )
        self.publisher.publish(goal)
        # WorkItem ownership begins at this execution boundary, not when a
        # frontier candidate was merely selected.  A later connector refresh
        # is harmless because the announcer is idempotent.
        announce_work_item = getattr(self, "announce_active_work_item_dispatch", None)
        if announce_work_item is not None:
            announce_work_item()

    def publish_active_route_command(
        self, message, active_cell, held_waypoint_map, route_steps, seed,
        robot_map, robot_yaw_map,
    ):
        """Resolve, report, and publish the stable command for one route."""
        command = self.resolve_active_route_command(
            message,
            active_cell,
            held_waypoint_map,
            route_steps,
            seed,
            robot_map,
            robot_yaw_map,
        )
        self.publish_active_route_status(command)
        self.publish_active_route_pose(message, command)
