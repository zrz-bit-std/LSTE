#!/usr/bin/env python3
"""Regression tests for segmented global-frontier route terminals."""

from pathlib import Path
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from global_frontier_terminal_connector import GlobalFrontierTerminalConnectorMixin


class ConnectorTerminalLifecycle(GlobalFrontierTerminalConnectorMixin):
    def __init__(
        self, *, endpoint_only=False, route_kind="frontier_connector",
        waypoint=(2.0, 0.0), mission=(8.0, 0.0),
    ):
        self.mission_endpoint_only = endpoint_only
        self.active_frontier = (0, 0, mission[0], mission[1])
        self.active_last_waypoint_map = waypoint
        self.active_last_waypoint_yaw = 1.0
        self.active_route_kind = route_kind
        self.active_terminal_received = True
        self.active_progress_time = 0.0
        self.active_last_progress_signal = "old"
        self.last_planning_wall = 10.0
        self.active_route_id = 7
        self.events = []

    def publish_status(self, event, **data):
        self.events.append((event, data))


class RouteConnectorTerminalTest(unittest.TestCase):
    def test_intermediate_connector_keeps_the_mission_open(self):
        lifecycle = ConnectorTerminalLifecycle()

        self.assertTrue(lifecycle.consume_connector_terminal())
        self.assertIsNone(lifecycle.active_last_waypoint_map)
        self.assertIsNone(lifecycle.active_last_waypoint_yaw)
        self.assertFalse(lifecycle.active_terminal_received)
        self.assertEqual(lifecycle.active_frontier[2:4], (8.0, 0.0))
        self.assertEqual(lifecycle.active_route_kind, "frontier_connector")
        self.assertEqual(lifecycle.events[-1][0], "route_connector_reached")
        self.assertEqual(lifecycle.events[-1][1]["mission_remaining"], 6.0)

    def test_final_endpoint_is_left_for_normal_mission_completion(self):
        lifecycle = ConnectorTerminalLifecycle(waypoint=(8.0, 0.0))

        self.assertFalse(lifecycle.consume_connector_terminal())
        self.assertEqual(lifecycle.active_last_waypoint_map, (8.0, 0.0))
        self.assertTrue(lifecycle.active_terminal_received)
        self.assertEqual(lifecycle.events, [])

    def test_atomic_and_local_egress_routes_never_consume_a_connector_terminal(self):
        atomic = ConnectorTerminalLifecycle(endpoint_only=True)
        egress = ConnectorTerminalLifecycle(route_kind="local_egress")

        self.assertFalse(atomic.consume_connector_terminal())
        self.assertFalse(egress.consume_connector_terminal())
