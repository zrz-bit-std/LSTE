"""Regression tests for parallax-based target geometry."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from target_ray_hypothesis import estimate_target_point


class TargetRayHypothesisTest(unittest.TestCase):
    def test_intersects_static_forward_rays(self):
        # Target=(5, 2); two observer poses provide real parallax.
        result = estimate_target_point(
            [
                ((0.0, 0.0), (5.0, 2.0)),
                ((1.0, 0.0), (4.0, 2.0)),
                ((0.5, 0.2), (4.5, 1.8)),
            ]
        )
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.point_xy[0], 5.0, places=3)
        self.assertAlmostEqual(result.point_xy[1], 2.0, places=3)
        self.assertLess(result.residual, 1e-5)

    def test_parallel_rays_do_not_invent_depth(self):
        self.assertIsNone(
            estimate_target_point(
                [((0.0, 0.0), (1.0, 0.0)), ((0.01, 0.0), (1.0, 0.0))]
            )
        )

    def test_inconsistent_or_behind_rays_are_rejected(self):
        self.assertIsNone(
            estimate_target_point(
                [((0.0, 0.0), (1.0, 0.0)), ((1.0, 0.0), (-1.0, 0.0))]
            )
        )

    def test_small_baseline_is_not_observable(self):
        self.assertIsNone(
            estimate_target_point(
                [((0.0, 0.0), (1.0, 1.0)), ((0.01, 0.01), (1.0, 1.0))]
            )
        )


if __name__ == "__main__":
    unittest.main()
