"""An exhausted doorway return settles that branch; it does not seal the trial.

Runs ``20261101_191225`` (cross, sim 1314) and ``20261101_210304`` (rotated T,
sim 765) both aborted with ``portal_authentication_return_unrecoverable`` after
13-22 minutes of healthy exploration, and lost ``tree``/``auth_all``/
``lifecycle``/``dfs`` in one shot because the runner sealed the whole trial.

The branch is still recorded as failed and the return obligation is still an
explicit fact in the ledger and the tree branch, but the observation period now
advances to the next doorway instead of freezing the mission.
"""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_portal_authentication import DoorwayAuthentication  # noqa: E402
from gp_branch_frontier import GPBranchFrontierMixin  # noqa: E402


class TreeBranch(dict):
    pass


class Fixture(GPBranchFrontierMixin):
    def __init__(self):
        auth = DoorwayAuthentication(13, 6, (0.0, 0.0), (1.0, 0.0))
        auth.begin((-1.0, 0.0))
        auth.request_failure_return(
            "authentication_physical_stall:entering", (1.0, 0.0)
        )
        auth.return_recovery_final_blocked = True
        self.auth = auth
        self.active_portal_authentication = lambda: auth
        self.pose_odom = SimpleNamespace(x=1.0, y=0.0, theta=0.0)
        self.events = []
        self.releases = []
        self.finished = []
        self.publish_status = lambda event, **kw: self.events.append((event, kw))
        self.release_active_frontier = lambda **kw: self.releases.append(kw)
        self.finish_portal_authentication_branch = (
            lambda **kw: self.finished.append(kw)
        )
        self.gp_exploration_tree = SimpleNamespace(
            branches={13: TreeBranch(id=13, state="authentication_failed")}
        )
        self.place_departure = SimpleNamespace(active=False)


class ReturnBudgetSettlementTest(unittest.TestCase):
    def settle(self):
        fixture = Fixture()
        fixture.check_portal_authentication_physical_progress(100.0)
        return fixture

    def test_branch_is_recorded_as_failed(self):
        fixture = self.settle()
        self.assertEqual(fixture.auth.state, "failed")
        self.assertEqual(fixture.auth.failure_reason, "return_unrecoverable")
        self.assertIn(
            "doorway_authentication_failed",
            [event["event"] for event in fixture.auth.events],
        )

    def test_the_return_obligation_stays_explicit(self):
        fixture = self.settle()
        fields = fixture.events[0][1]
        self.assertEqual(fixture.events[0][0],
                         "portal_authentication_return_unrecoverable")
        self.assertTrue(fields["execution_failed"])
        self.assertTrue(fields["return_obligation_pending"])
        self.assertEqual(fields["reason"], "source_side_recovery_budget_exhausted")
        branch = fixture.gp_exploration_tree.branches[13]
        self.assertEqual(branch["authentication_state"], "failed")
        self.assertEqual(branch["authentication_failure_reason"],
                         "return_unrecoverable")
        self.assertTrue(branch["return_obligation_pending"])

    def test_the_observation_period_advances(self):
        fixture = self.settle()
        self.assertEqual(len(fixture.finished), 1)
        self.assertEqual(fixture.finished[0], {"now": 100.0})

    def test_the_controller_lease_is_released_for_a_successor(self):
        fixture = self.settle()
        self.assertEqual(len(fixture.releases), 1)
        # queue_successor stays at its default (True): the planner may pick the
        # next doorway instead of being held on the abandoned return.
        self.assertNotIn("queue_successor", fixture.releases[0])
        self.assertFalse(fixture.releases[0].get("await_controller_release_ack"))

    def test_a_real_last_moment_return_still_wins(self):
        fixture = Fixture()
        fixture.auth.source_xy = (0.0, 0.0)
        fixture.auth.return_gate_cleared = True
        fixture.pose_odom = SimpleNamespace(x=0.0, y=0.0, theta=0.0)
        fixture.check_portal_authentication_physical_progress(100.0)
        self.assertNotEqual(fixture.auth.failure_reason, "return_unrecoverable")
        self.assertEqual(fixture.finished, [])


class RunnerContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (
            SCRIPTS.parents[2]
            / "scripts/tests/targeted_navigation/run_t_junction.sh"
        ).read_text(encoding="utf-8")

    def test_the_runner_no_longer_seals_the_trial_on_this_event(self):
        self.assertNotIn(
            'STOP_REASON="portal_authentication_return_unrecoverable"',
            self.source,
        )
        self.assertNotIn(
            'grep -Eq \'"frontier_event": *"portal_authentication_return_unrecoverable"\'',
            self.source,
        )

    def test_the_place_return_is_settled_instead_of_sealing_the_trial(self):
        # The node now publishes place_dfs_return_settled and keeps the
        # obligation; sealing the trial discarded every other audit's evidence.
        self.assertNotIn(
            'STOP_REASON="place_dfs_return_unrecoverable"', self.source
        )

    def test_scope_recovery_failure_is_still_fatal(self):
        self.assertIn(
            'STOP_REASON="place_scope_recovery_failed_trial"', self.source
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
