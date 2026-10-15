#!/usr/bin/env python3
"""Regression coverage for method isolation during prefetch promotion."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_prefetch_activation import GlobalFrontierPrefetchActivationMixin


class GeometryBaselinePrefetchProbe(GlobalFrontierPrefetchActivationMixin):
    """Minimal host proving cache promotion cannot mint a Place ledger."""

    def __init__(self):
        self.place_memory_enabled = False
        self.active_route_id = 4
        self.active_frontier = (0, 0, 0.5, 0.5)
        self.active_frontier_component = None
        self.active_frontier_region_id = 9
        self.current_physical_place_id = 9
        self.prefetched_frontier_component = None
        self.prefetched_frontier_information = 3.0
        self.active_observation_session_started_at = 0.0
        self.active_transition_kind = "initial"
        self.active_predecessor_route_id = 0
        self.active_transition_distance = None
        self.active_since = 0.0
        self.active_best_distance = None
        self.active_best_goal_distance = None
        self.active_best_path_distance = None
        self.active_progress_time = 0.0
        self.active_last_progress_signal = ""
        self.active_last_robot_xy = None
        self.active_start_odom_xy = None
        self.active_best_detour_odom_distance = 0.0
        self.active_unreachable_since = None
        self.active_last_waypoint_map = None
        self.active_last_waypoint_yaw = None
        self.active_route_kind = "frontier_endpoint"
        self.active_terminal_received = True
        self.turn_connector_released = False
        self.last_status_command_map = (1.0, 1.0)
        self.last_status_command_yaw = 0.0
        self.last_status_mission_map = (1.0, 1.0)
        self.pose_odom = None
        self.reservations = []
        self.cleared = False

    def activate_frontier_region(self, *_args, **_kwargs):
        raise AssertionError("geometry baseline must not activate a Place")

    def reserve_prefetched_work_item(self, *args):
        self.reservations.append(args)

    def begin_active_route_history(self, *_args):
        pass

    def _reset_active_odom_coverage(self):
        pass

    def _clear_active_post_turn_watchdog(self):
        pass

    def clear_prefetched_frontier(self):
        self.cleared = True


class FrontierPrefetchMethodBoundaryTest(unittest.TestCase):
    def test_geometry_baseline_promotion_keeps_place_and_workitem_state_empty(self):
        probe = GeometryBaselinePrefetchProbe()
        message = SimpleNamespace(info=SimpleNamespace(resolution=0.1))

        promoted = probe.install_prefetched_frontier(
            message,
            np.zeros((1, 1), dtype=np.int32),
            robot_map=(0.0, 0.0),
            now=5.0,
            row=0,
            col=0,
            x=2.0,
            y=1.0,
            preserve_route_id=False,
        )

        self.assertTrue(promoted)
        self.assertIsNone(probe.active_frontier_region_id)
        self.assertIsNone(probe.current_physical_place_id)
        self.assertEqual(len(probe.reservations), 1)
        self.assertIsNone(probe.reservations[0][3])
        self.assertTrue(probe.cleared)


if __name__ == "__main__":
    unittest.main()
