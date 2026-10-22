import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from global_frontier_portal_authentication import (
    DoorwayAuthentication,
    JunctionDoorwayAuthentication,
)
from gp_exploration_tree import ExplorationTree
from global_frontier_execution_resolution import GlobalFrontierExecutionResolutionMixin


class PortalAuthenticationTest(unittest.TestCase):
    def test_full_entry_locked_turn_and_return(self):
        branch = DoorwayAuthentication(4, 9, (0, 0), (1, 0), .8)
        self.assertTrue(branch.begin((-.5, 0)))
        self.assertAlmostEqual(branch.route_goal()[0],
                               branch.required_depth + branch.entry_goal_margin)
        self.assertGreater(branch.route_goal()[0], branch.required_depth + .55)
        self.assertFalse(branch.update_entry((.3, 0)))
        self.assertTrue(branch.update_entry((.9, 0)))
        self.assertTrue(branch.observe_inside(2, 5, 0.0, [-1.0]))
        locked = branch.target_yaw
        self.assertEqual(branch.turn_direction, "clockwise")
        self.assertAlmostEqual(branch.target_yaw, -(3.141592653589793 - 0.08))
        self.assertFalse(branch.update_rotation(-.5))
        self.assertFalse(branch.update_rotation(.5))
        self.assertAlmostEqual(branch.target_yaw, locked)
        self.assertFalse(branch.update_rotation(locked))
        self.assertFalse(branch.update_rotation(locked))
        self.assertTrue(branch.update_rotation(locked))
        self.assertEqual(branch.state, "turning")
        self.assertTrue(branch.finish_turn(locked))
        self.assertEqual(branch.route_goal(), (-.5, 0))
        self.assertTrue(branch.update_return((-.5, 0)))
        self.assertEqual(branch.state, "completed")

    def test_no_frontier_rejects_candidate(self):
        branch = DoorwayAuthentication(1, 2, (0, 0), (1, 0), .5)
        branch.begin((-1, 0))
        branch.update_entry((1, 0))
        self.assertTrue(branch.observe_inside(0, 0, 0.0, []))
        self.assertEqual(branch.state, "returning")
        self.assertEqual(branch.route_goal(), (-1.0, 0.0))
        self.assertTrue(branch.update_return((-1.0, 0.0)))
        self.assertEqual(branch.state, "failed")
        self.assertEqual(branch.failure_reason, "no_frontier_inside_doorway")

    def test_observation_period_waits_for_stable_branch_set(self):
        first = DoorwayAuthentication(2, 7, (0, 0), (1, 0), .5)
        task = JunctionDoorwayAuthentication(7, settle_seconds=1.0)
        task.add(first, now=10.0)
        self.assertIsNone(task.start_next((0, 0), now=10.9))
        self.assertEqual(task.phase, "watching")
        self.assertEqual(task.start_next((0, 0), now=11.0).branch_id, 2)

    def test_finished_branch_advances_once_and_preserves_failure(self):
        first = DoorwayAuthentication(2, 7, (0, 0), (1, 0), .5)
        second = DoorwayAuthentication(3, 7, (0, 0), (0, 1), .5)
        task = JunctionDoorwayAuthentication(7, (first, second), settle_seconds=0)
        self.assertEqual(task.start_next((-1, 0), now=1).branch_id, 2)
        first.update_entry((1, 0))
        first.observe_inside(1, 0, 0.0, [0.2])
        first.update_rotation(first.target_yaw)
        first.update_rotation(first.target_yaw)
        first.update_rotation(first.target_yaw)
        first.finish_turn(first.target_yaw)
        first.update_return((-1, 0))
        self.assertEqual(task.finish_active((-1, 0), now=2).branch_id, 3)
        second.update_entry((0, 1))
        second.observe_inside(0, 0, 0.0, [])
        self.assertEqual(second.state, "returning")
        self.assertTrue(second.update_return((-1, 0)))
        self.assertTrue(task.all_finished())
        self.assertEqual(task.snapshot()["branches"][1]["failure_reason"],
                         "no_frontier_inside_doorway")

    def test_new_branch_waits_for_observation_window(self):
        first = DoorwayAuthentication(1, 4, (0, 0), (1, 0), .5)
        task = JunctionDoorwayAuthentication(4, (first,), settle_seconds=1.0)
        self.assertEqual(task.start_next((-1, 0), now=2.0).branch_id, 1)
        first.fail("test_failure")
        second = DoorwayAuthentication(2, 4, (0, 0), (0, 1), .5)
        task.add(second, now=2.5)
        self.assertIsNone(task.finish_active((-1, 0), now=2.6))
        self.assertEqual(task.phase, "watching")
        self.assertEqual(task.start_next((-1, 0), now=3.5).branch_id, 2)

    def test_rejected_door_preserves_tree_lineage(self):
        tree = ExplorationTree()
        tree.nodes = {
            1: {"id": 1, "branch_ids": [5]},
            2: {"id": 2, "parent_branch_id": 5},
        }
        tree.branches = {
            5: {"id": 5, "node_id": 1, "destination_node_id": 2,
                "destination_space_id": 8, "state": "active"},
        }
        tree.spaces = {8: {"id": 8, "parent_branch_id": 5, "node_ids": [2]}}
        tree.reject_branch_authentication(5, "no_frontier_inside_doorway")
        self.assertEqual(tree.branches[5]["state"], "authentication_failed")
        self.assertEqual(tree.nodes[2]["parent_branch_id"], 5)
        self.assertEqual(tree.nodes[2]["rejected_by_branch_id"], 5)
        self.assertEqual(tree.spaces[8]["rejected_by_branch_id"], 5)
        self.assertIn(5, tree.nodes[1]["branch_ids"])


if __name__ == "__main__":
    unittest.main()
