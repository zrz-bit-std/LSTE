"""A doorway branch whose source-side return stays blocked must settle.

env04 ``20261112_194955`` / ``20261112_213048``: the authentication route kept
the graph transaction it was created with, the GoalManager discarded the
republications as ``graph_transaction_id_below_high_water``, and the branch
stayed in ``returning`` while no return goal was ever dispatched.  The internal
140 s return budget starts *after* the stationary clock and was pre-empted by
the 300 s stationary watchdog, so the branch never settled and kept owning
navigation.

The branch now timestampes the moment its return retries are declared blocked.
Once that window stays open, the physical-progress owner settles the branch so
the junction stops owning navigation and the observation period can advance.
"""

from pathlib import Path
import sys
import time
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
    def __init__(self, blocked_age=20.0, blocked=True, state="returning"):
        auth = DoorwayAuthentication(13, 6, (0.0, 0.0), (1.0, 0.0))
        auth.begin((-1.0, 0.0))
        auth.request_failure_return(
            "authentication_physical_stall:entering", (1.0, 0.0)
        )
        auth.state = state
        auth.return_recovery_blocked = blocked
        if blocked_age is not None:
            auth.return_recovery_blocked_at = time.monotonic() - blocked_age
        self.auth = auth
        self.active_portal_authentication = lambda: auth
        self.pose_odom = SimpleNamespace(x=1.0, y=0.0, theta=0.0)
        self.active_timeout = 20.0
        self.portal_authentication_stale_route_grace = 15.0
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

    def reopen_late_stable_candidates(self, now):
        return False

    def check(self, now=100.0):
        return self.check_portal_authentication_physical_progress(now)


class BlockedReturnSettlementTest(unittest.TestCase):
    def test_a_settled_blocked_return_fails_the_branch(self):
        fixture = Fixture(blocked_age=20.0)
        self.assertTrue(fixture.check())
        self.assertEqual(fixture.auth.state, "failed")
        self.assertEqual(fixture.auth.failure_reason, "return_unrecoverable")

    def test_the_observation_period_advances(self):
        fixture = Fixture()
        fixture.check()
        self.assertEqual(fixture.finished, [{"now": 100.0}])
        self.assertEqual(len(fixture.releases), 1)

    def test_the_reason_names_the_stale_identity(self):
        fixture = Fixture()
        fixture.check()
        event, fields = fixture.events[0]
        self.assertEqual(event, "portal_authentication_return_unrecoverable")
        self.assertEqual(fields["reason"], "route_identity_stale_rejected")
        self.assertTrue(fields["execution_failed"])
        self.assertTrue(fields["return_obligation_pending"])

    def test_a_fresh_blocked_window_is_left_alone(self):
        fixture = Fixture(blocked_age=5.0)
        self.assertFalse(fixture.check())
        self.assertEqual(fixture.auth.state, "returning")
        self.assertEqual(fixture.finished, [])

    def test_an_unblocked_return_is_left_alone(self):
        fixture = Fixture(blocked_age=None, blocked=False)
        self.assertFalse(fixture.check())
        self.assertEqual(fixture.auth.state, "returning")
        self.assertEqual(fixture.finished, [])

    def test_entering_phase_is_covered(self):
        fixture = Fixture(blocked_age=20.0, state="entering")
        self.assertTrue(fixture.check())
        self.assertEqual(fixture.auth.state, "failed")


class BlockedTimestampTest(unittest.TestCase):
    def test_the_branch_stamps_the_blocked_window_once(self):
        branch = DoorwayAuthentication(19, 4, (0.0, 0.0), (1.0, 0.0))
        branch.begin((-1.0, 0.0))
        branch.state = "returning"
        branch.failure_reason = "authentication_physical_stall:turning"
        now = 0.0
        for step in range(40):
            now += 5.0
            branch.physical_progress_expired((0.4, 0.0), 0.0, now, 10.0)
        self.assertTrue(branch.return_recovery_blocked)
        self.assertIsNotNone(getattr(branch, "return_recovery_blocked_at", None))


if __name__ == "__main__":
    unittest.main()
