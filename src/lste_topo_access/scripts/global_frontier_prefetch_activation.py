"""Promote a validated cached successor into the active route lifecycle."""

import math

import rospy

from lifecycle_manager import State
from global_frontier_topology import copy_component_evidence


class GlobalFrontierPrefetchActivationMixin:
    def install_prefetched_frontier(
        self, message, steps, robot_map, now, row, col, x, y,
        preserve_route_id,
    ):
        """Turn one validated cache entry into the next active route lease."""
        predecessor_route_id = int(self.active_route_id)
        previous_frontier = self.active_frontier
        previous_component = self.active_frontier_component
        previous_region_id = self.active_frontier_region_id
        self.active_frontier = (row, col, x, y)
        self.active_frontier_component = copy_component_evidence(
            self.prefetched_frontier_component
        )
        self.active_frontier_region_id = None
        if not preserve_route_id:
            self.active_route_id += 1
        region = None
        if getattr(self, "place_memory_enabled", True):
            region = self.activate_frontier_region(
                x,
                y,
                (
                    0.0
                    if self.prefetched_frontier_information is None
                    else self.prefetched_frontier_information
                ),
                now,
                "prefetched_successor",
                component=self.prefetched_frontier_component,
                physical_place_id=getattr(self, "current_physical_place_id", None),
            )
            if region is None:
                self.active_frontier = previous_frontier
                self.active_frontier_component = previous_component
                self.active_frontier_region_id = previous_region_id
                return None
            self.active_frontier_region_id = int(region["id"])
            self.current_physical_place_id = int(region["id"])
        else:
            # ``frontier`` and ``frontier_distance_dedup`` own only endpoint
            # radius memory.  A cached successor must not mint a hidden Place
            # or WorkItem when it is promoted.
            self.active_frontier_region_id = None
            self.current_physical_place_id = None
        reserve_work = getattr(self, "reserve_prefetched_work_item", None)
        if reserve_work is None:
            self.active_work_item_id = None
            self.active_work_item_attempt_id = None
        else:
            reserve_work(message, x, y, region, now)
        self.active_observation_session_started_at = None
        self.active_transition_kind = (
            "prefix_continuation" if preserve_route_id else "endpoint_divergence"
        )
        self.active_predecessor_route_id = predecessor_route_id
        self.active_transition_distance = (
            None if previous_frontier is None else math.hypot(
                float(previous_frontier[2]) - robot_map[0],
                float(previous_frontier[3]) - robot_map[1],
            )
        )
        self.active_since = now
        self.active_best_distance = math.hypot(x - robot_map[0], y - robot_map[1])
        self.active_best_goal_distance = self.active_best_distance
        self.active_best_path_distance = (
            float(steps[row, col]) * float(message.info.resolution)
            if steps is not None and steps[row, col] >= 0 else None
        )
        self.active_progress_time = now
        self.active_last_progress_signal = "route_and_goal_initialized"
        self.active_last_robot_xy = (robot_map[0], robot_map[1])
        self.begin_active_route_history(robot_map)
        self.active_start_odom_xy = (
            None if self.pose_odom is None
            else (float(self.pose_odom.x), float(self.pose_odom.y))
        )
        self.active_best_detour_odom_distance = 0.0
        self._reset_active_odom_coverage()
        self.active_unreachable_since = None
        lifecycle = getattr(self, "lifecycle_manager", None)
        if not preserve_route_id and lifecycle is not None:
            # Prefetched successors bypass initialize_active_frontier_route.
            # They are still a new route owner, so they must open a fresh
            # lifecycle transaction before the route command crosses ROS.
            lifecycle.begin_transaction(State.DISPATCHED, now=now)
            latest = getattr(lifecycle, "latest_transaction_id", None)
            self.active_route_lifecycle_transaction_id = int(
                latest() if callable(latest) else lifecycle.current_transaction_id
            )
        elif preserve_route_id:
            # A prefix continuation deliberately keeps the predecessor route
            # identity.  If a compatibility composition did not initialize
            # it, use the current lifecycle only as a defensive fallback.
            active_lifecycle = int(
                getattr(self, "active_route_lifecycle_transaction_id", 0) or 0
            )
            if active_lifecycle <= 0 and lifecycle is not None:
                latest = getattr(lifecycle, "latest_transaction_id", None)
                self.active_route_lifecycle_transaction_id = int(
                    latest()
                    if callable(latest)
                    else lifecycle.current_transaction_id
                )

        graph_transaction = getattr(
            self, "graph_route_action_transaction", None
        )
        route_epoch = (
            getattr(graph_transaction, "map_epoch", None)
            if graph_transaction is not None
            else getattr(self, "last_map_epoch", None)
        )
        try:
            self.active_route_map_epoch = max(1, int(route_epoch or 0))
        except (TypeError, ValueError):
            self.active_route_map_epoch = max(
                1, int(getattr(self, "topology_component_epoch", 1) or 1)
            )
        route_origin_epoch = getattr(graph_transaction, "origin_epoch", None)
        route_projection_epoch = getattr(
            graph_transaction, "projection_epoch", None
        )
        if route_origin_epoch is None:
            route_origin_epoch = self.active_route_map_epoch
        if route_projection_epoch is None:
            route_projection_epoch = self.active_route_map_epoch
        # Keep the successor's immutable predecessor contract aligned with the
        # lifecycle identity used by publish_route_command().  Without this
        # record, release handling can fall back to the previous route's
        # contract after a prefetched route takes ownership.
        self.active_route_dispatch_contract = {
            "route_id": int(self.active_route_id),
            "transaction_id": 0,
            "lifecycle_transaction_id": int(
                getattr(self, "active_route_lifecycle_transaction_id", 0) or 0
            ),
            "action_generation": int(
                getattr(self, "active_action_generation", 0) or 0
            ),
            "graph_transaction_id": int(
                getattr(graph_transaction, "graph_transaction_id", 0) or 0
            ),
            "map_epoch": self.active_route_map_epoch,
            "origin_epoch": route_origin_epoch,
            "projection_epoch": route_projection_epoch,
            "route_kind": "frontier_endpoint",
            "mission_route_kind": "frontier_endpoint",
            "source": "global_slam_frontier",
            "priority": 0,
        }
        if not preserve_route_id:
            self.publish_status(
                "frontier_successor_lifecycle_started",
                route_id=int(self.active_route_id),
                predecessor_route_id=int(predecessor_route_id),
                lifecycle_transaction_id=int(
                    self.active_route_lifecycle_transaction_id
                ),
                graph_transaction_id=int(
                    getattr(graph_transaction, "graph_transaction_id", 0) or 0
                ),
                map_epoch=self.active_route_map_epoch,
                origin_epoch=route_origin_epoch,
                projection_epoch=route_projection_epoch,
                reason="prefetched_successor_adopted",
            )
        # The promoted endpoint is a mission commitment. Reset command cache
        # so a distant branch begins with a connected waypoint, not one jump.
        self.active_last_waypoint_map = None
        self.active_last_waypoint_yaw = None
        self.active_mission_route_kind = "frontier_endpoint"
        self.active_route_kind = "frontier_endpoint"
        self.active_terminal_received = False
        self.turn_connector_released = True
        self._clear_active_post_turn_watchdog()
        self.last_status_command_map = None
        self.last_status_command_yaw = None
        self.last_status_mission_map = None
        self.clear_prefetched_frontier()
        return True

    def promote_prefetched_frontier(
        self, message, steps, robot_map, now, validation=None,
        preserve_route_id=False,
    ):
        """Promote a pending branch after the previous frontier is inspected."""
        candidate = self.validated_prefetched_frontier(
            message, steps, robot_map, validation,
        )
        if candidate is None:
            return None
        row, col, x, y = candidate
        if not self.install_prefetched_frontier(
            message,
            steps,
            robot_map,
            now,
            row,
            col,
            x,
            y,
            preserve_route_id,
        ):
            return None
        rospy.loginfo(
            "Global frontier promoted prefetched branch map=(%.2f,%.2f) "
            "distance=%.2fm",
            x,
            y,
            self.active_best_distance,
        )
        return row, col, x, y
