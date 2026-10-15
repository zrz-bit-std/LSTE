"""Regression tests for geometric target-track promotion evidence."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from goal_manager_target_geometry import (
    MIN_TRANSLATION_BASELINE_M,
    forward_ray_intersection,
    supports_stable_static_target,
    supports_static_target,
)


class TargetGeometryTest(unittest.TestCase):
    def test_forward_rays_intersect_in_front_of_both_poses(self):
        point = forward_ray_intersection(
            ((0.0, 0.0), (1.0, 1.0)),
            ((2.0, 0.0), (-1.0, 1.0)),
        )
        self.assertAlmostEqual(point[0], 1.0)
        self.assertAlmostEqual(point[1], 1.0)

    def test_parallel_or_behind_rays_are_not_target_evidence(self):
        self.assertIsNone(
            forward_ray_intersection(
                ((0.0, 0.0), (1.0, 0.0)),
                ((1.0, 0.0), (1.0, 0.0)),
            )
        )
        self.assertFalse(supports_static_target([
            ((0.0, 0.0), (1.0, 0.0)),
            ((0.0, 1.0), (1.0, 0.0)),
            ((0.0, 2.0), (1.0, 0.0)),
        ]))

    def test_three_rays_need_two_forward_intersections(self):
        self.assertTrue(supports_static_target([
            ((0.0, 0.0), (1.0, 1.0)),
            ((2.0, 0.0), (-1.0, 1.0)),
            ((0.0, 3.0), (1.0, -2.0)),
        ]))

    def test_two_source_stamped_rays_are_minimal_static_point_evidence(self):
        self.assertTrue(supports_static_target([
            ((0.0, 0.0), (1.0, 1.0)),
            ((2.0, 0.0), (-1.0, 1.0)),
        ]))

    def test_nearly_parallel_forward_rays_do_not_fake_depth(self):
        """Forward motion with a tiny bearing change is not triangulation."""
        from target_ray_hypothesis import estimate_target_point

        estimate = estimate_target_point([
            ((0.0, 0.0), (1.0, 0.0)),
            ((0.5, 0.0), (1.0, 0.01)),
            ((1.0, 0.0), (1.0, 0.02)),
        ])
        self.assertIsNone(estimate)

    def test_well_separated_rays_provide_navigation_depth(self):
        from target_ray_hypothesis import estimate_target_point

        estimate = estimate_target_point([
            ((0.0, 0.0), (1.0, 0.2)),
            ((1.0, 0.0), (-0.2, 1.0)),
        ])
        self.assertIsNotNone(estimate)
        self.assertGreaterEqual(estimate.ray_count, 2)

    def test_weak_track_requires_redundant_translational_geometry(self):
        rays = [
            ((0.0, 0.0), (1.0, 0.0)),
            ((0.0, 1.0), (1.0, -0.2)),
            ((0.3, 0.0), (0.98, 0.20)),
        ]
        self.assertTrue(supports_static_target(rays))
        self.assertTrue(supports_stable_static_target(rays))

        yaw_only = [
            ((0.0, 0.0), (1.0, 0.0)),
            ((0.0, 0.0), (0.98, 0.20)),
            ((0.0, 0.0), (0.93, 0.36)),
        ]
        self.assertFalse(supports_stable_static_target(yaw_only))

        small_baseline = [
            ((0.0, 0.0), (1.0, 0.0)),
            ((MIN_TRANSLATION_BASELINE_M * 0.5, 0.0), (0.98, 0.20)),
            ((MIN_TRANSLATION_BASELINE_M * 0.25, 0.10), (0.94, -0.34)),
        ]
        self.assertFalse(supports_stable_static_target(small_baseline))


if __name__ == "__main__":
    unittest.main()
