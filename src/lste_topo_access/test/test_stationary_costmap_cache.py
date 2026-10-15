#!/usr/bin/env python3
"""Red/green coverage for stationary costmap cache retention."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import global_frontier_planning_costmap as costmap_module
from global_frontier_planning_costmap import GlobalFrontierPlanningCostmapMixin
from global_frontier_route_state import GlobalFrontierRouteStateMixin


def grid_message():
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id="map",
            stamp=SimpleNamespace(to_sec=lambda: 0.0),
        ),
        info=SimpleNamespace(width=1, height=1, resolution=1.0),
        data=[0],
    )


class StationaryCostmapOwner(
    GlobalFrontierRouteStateMixin,
    GlobalFrontierPlanningCostmapMixin,
):
    def __init__(self):
        self.costmap_msg = grid_message()
        self.costmap_max_age = 3.0
        self.costmap_last_receive_wall = 0.0
        self.costmap_message_count = 1
        self.cached_costmap_validation = None
        self.cached_costmap_validation_wall = 0.0
        self.decision_wake_scheduler = None
        self.pose_odom = None


class StationaryCostmapCacheTest(unittest.TestCase):
    def test_static_pose_keeps_cache_valid_past_age_limit(self):
        owner = StationaryCostmapOwner()
        pose = SimpleNamespace(x=1.0, y=2.0, theta=0.0)
        owner.apply_pose(pose)
        owner.apply_pose(SimpleNamespace(x=1.0, y=2.0, theta=0.0))

        with patch.object(costmap_module.rospy, "logwarn_throttle", lambda *_args: None):
            self.assertIs(owner.fresh_costmap(), owner.costmap_msg)


if __name__ == "__main__":
    unittest.main()
