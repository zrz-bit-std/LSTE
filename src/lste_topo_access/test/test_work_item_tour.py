"""Behavioral checks for the imported pose-cost optimizer, independent of ROS."""
import itertools
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from global_frontier_work_item_tour import (
    compute_open_tour_time, kinematic_time, optimize_kinematic_open_tsp,
    order_work_items_open_tsp,
)


class WorkItemTourTest(unittest.TestCase):
    def test_pose_cost_includes_translation_and_both_turns(self):
        start = (0.0, 0.0, 0.0)
        self.assertAlmostEqual(kinematic_time(start, (2.0, 0.0, 0.0)), 5.0)
        self.assertAlmostEqual(
            kinematic_time(start, (-1.0, 0.0, 0.0)), 2.5 + 2 * math.pi / 0.8)
        self.assertAlmostEqual(
            kinematic_time(start, (0.0, 0.0, math.pi / 2)), math.pi / 1.6)

    def test_heading_can_prefer_forward_work_over_a_closer_backward_work(self):
        items = [
            {"id": 1, "anchor_xy": (1.0, 0.0), "normal_xy": (1.0, 0.0)},
            {"id": 2, "anchor_xy": (-0.5, 0.0), "normal_xy": (-1.0, 0.0)},
        ]
        self.assertEqual([i["id"] for i in order_work_items_open_tsp((0, 0, 0), items)], [1, 2])
        self.assertEqual([i["id"] for i in order_work_items_open_tsp((0, 0, math.pi), items)], [2, 1])

    def test_optimizer_removes_crossings_without_losing_work_items(self):
        start = (0.0, 0.0, 0.0)
        poses = [(0, 2, 0.5), (3, 0, 0), (0, 4, 1), (3, 4, -1)]
        initial = [0, 1, 2, 3]
        tour = optimize_kinematic_open_tsp(start, poses, initial_tour=initial)
        cost = compute_open_tour_time(start, poses, tour)
        self.assertEqual(sorted(tour), initial)
        self.assertLess(cost, compute_open_tour_time(start, poses, initial))
        optimum = min(compute_open_tour_time(start, poses, p)
                      for p in itertools.permutations(initial))
        self.assertAlmostEqual(cost, optimum)


if __name__ == "__main__":
    unittest.main()
