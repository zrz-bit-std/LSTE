"""Regression tests for target-as-observation, not target-as-destination."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from goal_manager_target_viewpoint_geometry import build_target_viewpoint  # noqa: E402


class ViewpointManager:
    def __init__(self, hypothesis=None, ray_count=0):
        self.latest_pose = SimpleNamespace(x=0.0, y=0.0)
        self.target_hypothesis_xy = hypothesis
        self.target_hypothesis_ray_count = ray_count
        self.target_hypothesis_navigation_enabled = True
        self.target_goal_reached_radius = 0.75
        self.target_route_validation_tolerance = 0.20

    def target_minimum_viewpoint_distance(self):
        return 0.90


class TargetViewpointGeometryTest(unittest.TestCase):
    def test_static_target_is_approached_to_a_standoff_ring(self):
        geometry = build_target_viewpoint(
            ViewpointManager((0.0, 4.0), ray_count=6),
            1.57,
            1.5,
        )

        self.assertEqual(geometry.mode, "static_target_standoff")
        self.assertAlmostEqual(geometry.x, 0.0, places=6)
        self.assertAlmostEqual(geometry.y, 1.5, places=6)
        self.assertAlmostEqual(geometry.target_standoff, 1.10, places=6)
        self.assertAlmostEqual(geometry.remaining_target_range, 2.5, places=6)
        # The request is a view point, never the estimated object coordinate.
        self.assertLess(geometry.y, 4.0)

    def test_safe_standoff_does_not_create_zero_length_navigation_goal(self):
        geometry = build_target_viewpoint(
            ViewpointManager((0.0, 1.0), ray_count=6),
            1.57,
            1.5,
        )

        self.assertEqual(geometry.mode, "safe_standoff")
        self.assertAlmostEqual(geometry.distance_from_robot, 0.0, places=6)

    def test_unobservable_track_retains_bearing_horizon_fallback(self):
        geometry = build_target_viewpoint(
            ViewpointManager((0.0, 4.0), ray_count=1),
            1.57,
            1.5,
        )

        self.assertEqual(geometry.mode, "bearing_horizon")
        self.assertAlmostEqual(geometry.y, 1.5, places=6)
        self.assertIsNone(geometry.target_range)


if __name__ == "__main__":
    unittest.main()
