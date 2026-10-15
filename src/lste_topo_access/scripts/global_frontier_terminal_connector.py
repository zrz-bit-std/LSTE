"""Advance a segmented frontier route without completing its mission lease."""

import math

from clock_provider import now_for


class GlobalFrontierTerminalConnectorMixin:
    """Separate intermediate connector terminals from final route terminals."""

    def consume_connector_terminal(self):
        """Release a reached segment while retaining the mission endpoint.

        A segmented route keeps one ``active_route_id`` across several short
        MoveBase actions.  Treating every successful segment as a mission
        terminal would incorrectly record a portal crossing while the base is
        still on the source side of the door.  The connector is complete only
        when its command point is the mission endpoint itself.
        """
        if (
            self.mission_endpoint_only
            or self.active_frontier is None
            or self.active_last_waypoint_map is None
            or self.active_route_kind != "frontier_connector"
            or self.active_route_kind in (
                "frontier_turn_connector",
                "local_egress",
            )
        ):
            return False
        mission_x, mission_y = self.active_frontier[2:4]
        command_x, command_y = self.active_last_waypoint_map
        endpoint_error = math.hypot(
            float(mission_x) - float(command_x),
            float(mission_y) - float(command_y),
        )
        if endpoint_error <= 0.05:
            return False

        reached_command = (float(command_x), float(command_y))
        self.active_last_waypoint_map = None
        self.active_last_waypoint_yaw = None
        self.active_terminal_received = False
        self.active_progress_time = now_for(self)
        self.active_last_progress_signal = "connector_terminal"
        # Force the next timer to rebuild a route from the newly observed
        # local map before it emits the next segment of this same mission.
        self.last_planning_wall = 0.0
        self.publish_status(
            "route_connector_reached",
            route_id=int(self.active_route_id),
            route_kind=self.active_route_kind,
            command_goal=[
                round(reached_command[0], 3),
                round(reached_command[1], 3),
            ],
            mission_goal=[round(float(mission_x), 3), round(float(mission_y), 3)],
            mission_remaining=round(endpoint_error, 3),
        )
        return True
