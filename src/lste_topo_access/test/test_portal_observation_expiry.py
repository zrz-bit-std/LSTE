"""Bounded conclusion for a portal observation period that cannot start.

Run ``20261031_165652`` froze at sim 298.17: node 5 kept three branches
``[9, 10, 12]`` and stayed in ``phase=watching`` with ``all_finished=false``.
Branch 11 had just been withdrawn because the robot was on the destination
side of its gate (``source_depth = +0.473``), and the remaining branches could
never be entered. While the phase stayed ``watching`` the caller kept retiring
the ordinary route, so the graph dispatched nothing for the next 615 s.

The wait is now bounded: after ``portal_observation_start_timeout`` seconds
with no runnable branch, the remaining branches are failed and the task
completes.
"""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_portal_authentication import (  # noqa: E402
    DoorwayAuthentication,
    JunctionDoorwayAuthentication,
)


def make_branch(branch_id, state="pending"):
    branch = DoorwayAuthentication(branch_id, 5, (0.0, 0.0), (1.0, 0.0))
    branch.state = state
    return branch


def make_task(states):
    task = JunctionDoorwayAuthentication(5, settle_seconds=1.0)
    for branch_id, state in states.items():
        task.add(make_branch(branch_id, state), now=0.0)
    return task


class ObservationExpiryTest(unittest.TestCase):
    def test_watching_timer_starts_on_first_idle_cycle(self):
        task = make_task({9: "pending", 10: "pending", 12: "pending"})
        self.assertEqual(task.expire_unstartable(100.0, 30.0), [])
        self.assertAlmostEqual(task.watching_since, 100.0)

    def test_wait_is_bounded(self):
        task = make_task({9: "pending", 10: "pending", 12: "pending"})
        task.expire_unstartable(100.0, 30.0)
        self.assertEqual(task.expire_unstartable(129.0, 30.0), [])
        expired = task.expire_unstartable(130.0, 30.0)
        self.assertEqual(sorted(expired), [9, 10, 12])
        self.assertEqual(task.phase, "complete")
        self.assertEqual(task.branches, {})

    def test_replay_of_the_sealed_node5_watching_state(self):
        # The exact branch set the run froze on.
        task = make_task({9: "pending", 10: "pending", 12: "pending"})
        self.assertEqual(task.phase, "watching")
        self.assertFalse(task.all_finished())
        task.expire_unstartable(298.17, 30.0)
        self.assertEqual(task.expire_unstartable(328.17, 30.0), [9, 10, 12])
        self.assertTrue(task.all_finished())
        # The caller stops retiring the ordinary route once the phase is not
        # watching/authenticating.
        self.assertNotIn(task.phase, ("watching", "authenticating"))

    def test_active_branch_is_never_expired(self):
        task = make_task({9: "pending", 10: "pending"})
        task.active_branch_id = 9
        self.assertEqual(task.expire_unstartable(0.0, 30.0), [])
        self.assertEqual(task.expire_unstartable(600.0, 30.0), [])
        self.assertIn(9, task.branches)

    def test_finished_task_is_left_alone(self):
        task = make_task({9: "completed", 10: "failed"})
        self.assertTrue(task.all_finished())
        self.assertEqual(task.expire_unstartable(0.0, 30.0), [])
        self.assertEqual(task.expire_unstartable(600.0, 30.0), [])

    def test_started_branch_is_failed_rather_than_withdrawn(self):
        task = make_task({9: "pending", 10: "entering"})
        task.expire_unstartable(0.0, 30.0)
        expired = task.expire_unstartable(31.0, 30.0)
        self.assertEqual(sorted(expired), [9, 10])
        self.assertEqual(task.phase, "complete")
        # The withdrawn branch is gone; the started one is failed in place so
        # its history is still inspectable.
        self.assertEqual(sorted(task.branches), [10])
        self.assertEqual(task.branches[10].state, "failed")
        self.assertEqual(task.branches[10].failure_reason,
                         "observation_geometry_unavailable")

    def test_timeout_is_configurable(self):
        task = make_task({9: "pending"})
        task.expire_unstartable(0.0, 5.0)
        self.assertEqual(task.expire_unstartable(4.0, 5.0), [])
        self.assertEqual(task.expire_unstartable(5.0, 5.0), [9])


class CallerContractTest(unittest.TestCase):
    """The graph must actually call the bound before retiring the route."""

    @classmethod
    def setUpClass(cls):
        cls.source = (SCRIPTS / "gp_branch_frontier.py").read_text(encoding="utf-8")

    def test_caller_expires_before_retiring_the_route(self):
        expire_at = self.source.index("task.expire_unstartable(")
        retire_at = self.source.index(
            'if task.branches and task.phase in ("watching", "authenticating"):')
        self.assertLess(expire_at, retire_at)

    def test_caller_marks_tree_branches_failed(self):
        self.assertIn('tree_branch["authentication_state"] = "failed"', self.source)
        self.assertIn("portal_authentication_observation_expired", self.source)

    def test_timeout_is_a_parameter(self):
        self.assertIn("portal_observation_start_timeout", self.source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
