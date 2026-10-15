"""Physical-progress observation for an active frontier route."""

import math

import rospy

from global_frontier_models import ActiveRouteObservation
from global_frontier_physical_progress import physical_progress_decision
from global_frontier_topology import copy_component_evidence


class GlobalFrontierExecutionObservationMixin:
    """Collect only map, odometry, and local-information progress evidence."""

    def observe_active_route(
        self, message, steps, validation, unknown, components, row, col, x, y,
        robot_map, now, portal_transition,
    ):
        """Update route-progress evidence and return the current map facts."""
        if portal_transition:
            self.observe_portal_gate_approach(now)
            # Portal topology is advanced by continuous physical evidence, not
            # by an arbitrary controller terminal that may arrive late or not
            # at all after the base has already crossed the doorway.
            portal_destination = self.observe_active_portal_crossing(now)
            if portal_destination is not None and self.active_frontier is None:
                # Persistent portal crossing may release the old semantic
                # route in the same serialized tick. Do not continue below
                # with reset route fields or let the old watchdog process a
                # route that has already been committed and released.
                return None
        distance = math.hypot(x - robot_map[0], y - robot_map[1])
        if (
            portal_transition
            and distance <= self.endpoint_terminal_wait_radius
        ):
            transaction = getattr(self, "portal_transaction", None)
            if transaction is not None:
                transaction.destination_standoff(self.active_route_id)
        information = self.frontier_information(unknown, row, col)
        component = None
        if not portal_transition:
            component = self.topology_component_evidence(
                message, components, row, col, steps=steps,
            )
            if component is not None:
                self.active_frontier_component = copy_component_evidence(component)
            observation_ready = self.active_frontier_observation_ready(distance)
            observation_session_started = (
                observation_ready
                and self.active_observation_session_started_at is None
            )
            observed_region = self.region_memory.observe(
                x,
                y,
                information,
                robot_map[0],
                robot_map[1],
                now,
                component=component,
                region_id=self.active_frontier_region_id,
                observation_ready=observation_ready,
                observation_session_started=observation_session_started,
                physical_robot_xy=(
                    None
                    if getattr(self, "pose_odom", None) is None
                    else (float(self.pose_odom.x), float(self.pose_odom.y))
                ),
            )
            if observation_session_started and observed_region is not None:
                self.active_observation_session_started_at = now
                self.publish_status(
                    "frontier_observation_session_started",
                    route_id=int(self.active_route_id),
                    region_id=int(observed_region["id"]),
                    goal=[round(float(x), 3), round(float(y), 3)],
                    observer=[
                        round(float(robot_map[0]), 3),
                        round(float(robot_map[1]), 3),
                    ],
                    observation_distance=round(float(distance), 3),
                )
                rospy.loginfo(
                    "Global frontier began observation session route_id=%d "
                    "region_id=%d distance=%.2fm",
                    self.active_route_id,
                    observed_region["id"],
                    distance,
                )

        route_distance = float(steps[row, col]) * float(message.info.resolution)
        if validation is not None:
            costmap_distance = self.candidate_costmap_distance(validation, x, y)
            if costmap_distance is not None:
                route_distance = costmap_distance

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
            # Kept for old status readers; this is explicitly the physical
            # distance to the current fixed endpoint.
            self.active_best_goal_distance = distance
            self.active_best_distance = distance

        odom_detour_progress = self._active_route_odom_detour_progress()
        odom_coverage_progress = self._entered_novel_active_odom_cell()
        self._record_active_route_progress(
            now,
            route_progress,
            goal_progress,
            odom_detour_progress,
            odom_coverage_progress,
        )
        self._update_active_route_position(robot_map)
        return ActiveRouteObservation(
            distance=distance,
            information=information,
            component=component,
            route_distance=route_distance,
        )

    def observe_portal_gate_approach(self, now):
        """Start the bounded edge action after physically reaching its gate.

        The map frame moves during online SLAM. The gate was projected to
        ``odom`` when this route became active, so the check remains attached
        to one physical doorway. The existing endpoint envelope is reused as
        the gate tolerance; this is a lifecycle boundary, not a new tuning
        parameter.
        """
        if (
            self.active_route_kind != "portal_transition"
            or self.active_portal_gate_approached_at is not None
            or self.active_portal_gate_odom_xy is None
            or self.pose_odom is None
        ):
            return False
        gate_x, gate_y = self.active_portal_gate_odom_xy
        distance = math.hypot(
            float(self.pose_odom.x) - float(gate_x),
            float(self.pose_odom.y) - float(gate_y),
        )
        # ``completed_radius`` already defines the physical footprint around
        # a reached viewpoint. It tolerates the small map/odom residual left
        # by an online SLAM update while remaining smaller than a room-scale
        # route. Do not introduce a second doorway-distance parameter.
        gate_tolerance = max(
            float(self.endpoint_terminal_wait_radius),
            float(self.completed_radius),
        )
        if distance > gate_tolerance:
            return False
        self.active_portal_gate_approached_at = float(now)
        transaction = getattr(self, "portal_transaction", None)
        if transaction is not None:
            transition = transaction.gate_reached(self.active_route_id)
            if transition is not None:
                self.publish_status(
                    "portal_transaction_phase",
                    transaction_id=int(transition.transaction_id),
                    route_id=int(transition.route_id),
                    state=transition.state,
                    transition=transition.last_transition,
                    reason=transition.last_reason,
                )
        self.publish_status(
            "portal_edge_started_at_gate",
            route_id=int(self.active_route_id),
            portal_gate=[round(float(gate_x), 3), round(float(gate_y), 3)],
            gate_distance=round(float(distance), 3),
            gate_tolerance=round(float(gate_tolerance), 3),
        )
        return True

    def _active_route_odom_detour_progress(self):
        """Accept only a new maximum odom excursion during an active route."""
        if self.active_start_odom_xy is None or self.pose_odom is None:
            return False
        distance = math.hypot(
            float(self.pose_odom.x) - self.active_start_odom_xy[0],
            float(self.pose_odom.y) - self.active_start_odom_xy[1],
        )
        if distance <= self.active_best_detour_odom_distance + self.progress_epsilon:
            return False
        self.active_best_detour_odom_distance = distance
        return True

    def _record_active_route_progress(
        self, now, route_progress, goal_progress, odom_detour_progress,
        odom_coverage_progress,
    ):
        """Renew the watchdog only from the physical-progress contract.

        ``route_progress`` and ``goal_progress`` are map-derived diagnostics.
        They may improve solely because SLAM moved the coordinate frame or
        relabelled free space, which was the cause of the long Level 4 stall.
        Keep recording that fact, but never let it hide a stationary base from
        the route watchdog.
        """
        decision = physical_progress_decision(
            route_progress=route_progress,
            goal_progress=goal_progress,
            odom_detour_progress=odom_detour_progress,
            odom_novel_coverage=odom_coverage_progress,
        )
        if not decision.renew_watchdog:
            if decision.signal != "none":
                self.active_last_progress_signal = decision.signal
            return
        self.active_progress_time = now
        self.active_last_progress_signal = decision.signal

    def _update_active_route_position(self, robot_map):
        """Keep the last map position only after a meaningful displacement."""
        self.remember_active_route_position(robot_map)
        if self.active_last_robot_xy is None:
            self.active_last_robot_xy = (robot_map[0], robot_map[1])
        elif math.hypot(
            robot_map[0] - self.active_last_robot_xy[0],
            robot_map[1] - self.active_last_robot_xy[1],
        ) >= self.progress_epsilon:
            self.active_last_robot_xy = (robot_map[0], robot_map[1])
