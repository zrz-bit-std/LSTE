"""A doorway manoeuvre that never started must not condemn the doorway.

env04 ``20261112_231045`` node 6 branch 15 entered the east doorway and then
published a turn route.  The turn supervisor stayed ``PASS_THROUGH`` for the
whole 28 s window (``planner_contract_boundary``: terminal_hold /
typed_planner_contract_required) so the in-place turn was never executed, while
the physical-progress budget expired 24 s in and the branch was marked
``authentication_failed``.  No Place was created for the east room, the target
claim bound to the corridor Place instead, and its observation obligation never
materialised.

Two rules keep that from happening without weakening any doorway check:

* the motion window only judges a phase the controller actually took over;
* a failure to *execute* keeps the doorway identity for a bounded retry, while a
  verdict about the doorway itself is still rejected on the first attempt.
"""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_portal_authentication import (  # noqa: E402
    DoorwayAuthentication,
    GlobalFrontierPortalAuthenticationMixin,
    JunctionDoorwayAuthentication,
)
from gp_branch_frontier import GPBranchFrontierMixin  # noqa: E402


def _pending_auth():
    """A branch in the state ``JunctionDoorwayAuthentication`` starts from."""
    return DoorwayAuthentication(13, 6, (0.0, 0.0), (1.0, 0.0))


def _auth(state="entering"):
    """A branch already executing one phase of the manoeuvre."""
    auth = _pending_auth()
    auth.begin((-1.0, 0.0))
    auth.state = state
    return auth


class ActionRunningTest(unittest.TestCase):
    """``portal_authentication_action_running`` uses existing signals only."""

    def setUp(self):
        self.host = GPBranchFrontierMixin()
        self.host.turn_supervisor_status_monotonic = 100.0
        self.host.turn_supervisor_state = "PASS_THROUGH"
        self.host.active_route_dispatch_contract = {"action_generation": 0}

    def test_turning_requires_a_healthy_turn_supervisor(self):
        auth = _auth("turning")
        self.assertFalse(
            self.host.portal_authentication_action_running(auth, 101.0))
        self.host.turn_supervisor_state = "TURNING"
        self.assertTrue(
            self.host.portal_authentication_action_running(auth, 101.0))

    def test_turning_ignores_a_dead_heartbeat(self):
        self.host.turn_supervisor_state = "TURNING"
        auth = _auth("turning")
        self.assertFalse(
            self.host.portal_authentication_action_running(auth, 110.0))

    def test_entry_and_return_follow_bridge_adoption(self):
        auth = _auth("entering")
        self.assertFalse(
            self.host.portal_authentication_action_running(auth, 101.0))
        self.host.active_route_dispatch_contract = {"action_generation": 7}
        self.assertTrue(
            self.host.portal_authentication_action_running(auth, 101.0))
        auth.state = "returning"
        self.assertTrue(
            self.host.portal_authentication_action_running(auth, 101.0))

    def test_missing_contract_keeps_the_legacy_motion_judgement(self):
        auth = _auth("entering")
        self.host.active_route_dispatch_contract = None
        self.assertTrue(
            self.host.portal_authentication_action_running(auth, 101.0))

    def test_missing_turn_status_keeps_the_legacy_motion_judgement(self):
        auth = _auth("turning")
        self.host.turn_supervisor_status_monotonic = 0.0
        self.assertTrue(
            self.host.portal_authentication_action_running(auth, 101.0))


class ProgressWindowTest(unittest.TestCase):
    """The motion budget is frozen while the handoff is pending."""

    def test_rearmed_window_does_not_expire_without_motion(self):
        auth = _auth("turning")
        auth.target_yaw = 1.0
        auth.note_action_running(False, 10.0)
        self.assertAlmostEqual(auth._phase_handoff_since, 10.0)
        # The controller never took over for 20 s: the owner keeps rearming.
        auth.rearm_physical_progress((1.0, 0.0), 0.0, 30.0)
        self.assertFalse(
            auth.physical_progress_expired((1.0, 0.0), 0.0, 40.0, 20.0))

    def test_started_phase_still_expires_without_motion(self):
        auth = _auth("turning")
        auth.target_yaw = 1.0
        auth.note_action_running(True, 10.0)
        self.assertFalse(
            auth.physical_progress_expired((1.0, 0.0), 0.0, 10.0, 20.0))
        self.assertTrue(
            auth.physical_progress_expired((1.0, 0.0), 0.0, 40.0, 20.0))

    def test_phase_change_clears_the_started_flag(self):
        auth = _auth("entering")
        auth.note_action_running(True, 10.0)
        self.assertTrue(auth._phase_action_running)
        auth.state = "turning"
        auth.note_action_running(False, 11.0)
        self.assertFalse(auth._phase_action_running)
        self.assertAlmostEqual(auth._phase_handoff_since, 11.0)

    def test_resumed_action_clears_the_handoff_clock(self):
        auth = _auth("turning")
        auth.note_action_running(False, 10.0)
        auth.note_action_running(True, 12.0)
        self.assertIsNone(auth._phase_handoff_since)


class TaskRetryTest(unittest.TestCase):
    """``retry_active`` requeues the same doorway after a cooldown."""

    def test_retry_requeues_the_same_branch_after_the_cooldown(self):
        task = JunctionDoorwayAuthentication(6, settle_seconds=0.0)
        auth = _pending_auth()
        task.add(auth, now=0.0)
        self.assertIs(task.start_next((-1.0, 0.0), now=0.0), auth)
        auth.state = "turning"
        auth.fail("authentication_physical_stall:turning")

        self.assertIsNone(task.retry_active((-1.0, 0.0), now=100.0, cooldown=5.0))
        self.assertIsNone(task.active_branch_id)
        self.assertEqual(task.branches[13].state, "pending")
        self.assertAlmostEqual(task.branches[13].retry_not_before, 105.0)
        self.assertFalse(task.all_finished())
        self.assertIsNone(task.next_pending(now=104.0))
        self.assertIs(task.start_next((-1.0, 0.0), now=106.0),
                      task.branches[13])

    def test_reset_keeps_the_measured_gate_and_identity(self):
        auth = _auth("turning")
        auth.gate_geometry = (1.2, ((0.0, -0.6), (0.0, 0.6)))
        auth.inside_xy = (1.3, 0.0)
        auth.target_yaw = 2.0
        auth.fail("authentication_physical_stall:turning")
        auth.reset_for_retry(200.0, cooldown=5.0)

        self.assertEqual(auth.state, "pending")
        self.assertIsNone(auth.failure_reason)
        self.assertIsNone(auth.target_yaw)
        self.assertIsNone(auth.inside_xy)
        self.assertEqual(auth.gate_geometry[0], 1.2)
        self.assertEqual(auth.gate_xy, (0.0, 0.0))
        self.assertAlmostEqual(auth.retry_not_before, 205.0)


class Harness(GlobalFrontierPortalAuthenticationMixin):
    def __init__(self, limit=2, cooldown=5.0):
        self.events = []
        self.rejected = []
        self.portal_authentication_history = []
        self.portal_authentication_retry_limit = limit
        self.portal_authentication_retry_cooldown = cooldown
        self.portal_hypothesis_ledger = None
        self.region_memory = None
        self.gp_observation_tree = None
        tree = SimpleNamespace(
            branches={13: {"id": 13, "state": "authentication_failed"}},
            nodes={6: {"id": 6, "physical_xy": [-1.0, 0.0]}},
            current_node_id=6,
        )
        tree.reject_branch_authentication = (
            lambda bid, reason: self.rejected.append((bid, reason)))
        self.gp_exploration_tree = tree
        self.junction_portal_authentication = None

    def publish_status(self, name, **kwargs):
        self.events.append((name, kwargs))


def _failed_branch(limit=2):
    host = Harness(limit=limit)
    task = JunctionDoorwayAuthentication(6, settle_seconds=0.0)
    task.add(_pending_auth(), now=0.0)
    host.junction_portal_authentication = task
    task.start_next((-1.0, 0.0), now=0.0)
    auth = task.branches[13]
    auth.fail("authentication_physical_stall:turning")
    return host, task, auth


class RetryClassificationTest(unittest.TestCase):
    def test_only_execution_failures_are_retryable(self):
        host = Harness()
        branch = SimpleNamespace(
            failure_reason="authentication_physical_stall:turning")
        self.assertEqual(
            host.portal_authentication_retry_reason(
                branch, {"authentication_retry_count": 0}),
            "authentication_physical_stall:turning")

    def test_retry_budget_stops_the_loop(self):
        host = Harness()
        branch = SimpleNamespace(
            failure_reason="authentication_physical_stall:entering")
        self.assertIsNone(host.portal_authentication_retry_reason(
            branch, {"authentication_retry_count": 2}))

    def test_doorway_verdicts_are_never_retried(self):
        host = Harness()
        branch = SimpleNamespace(failure_reason="")
        for reason in (
            "no_frontier_inside_doorway",
            "known_free_bypass_within_current_place",
            "measured_section_not_passable",
            "portal_source_side_geometry_unproven",
        ):
            branch.failure_reason = reason
            self.assertIsNone(
                host.portal_authentication_retry_reason(
                    branch, {"authentication_retry_count": 0}),
                reason)

    def test_zero_limit_restores_one_strike(self):
        host = Harness(limit=0)
        branch = SimpleNamespace(
            failure_reason="authentication_physical_stall:turning")
        self.assertIsNone(
            host.portal_authentication_retry_reason(branch, {}))


class FinishBranchRetryTest(unittest.TestCase):
    def test_execution_failure_retries_without_rejecting_the_doorway(self):
        host, task, _auth_obj = _failed_branch()
        host.finish_portal_authentication_branch(now=100.0)

        self.assertEqual(host.rejected, [])
        tree_branch = host.gp_exploration_tree.branches[13]
        self.assertEqual(tree_branch["state"], "retry")
        self.assertEqual(tree_branch["authentication_state"], "retry")
        self.assertEqual(tree_branch["authentication_retry_count"], 1)
        self.assertEqual(task.branches[13].state, "pending")
        self.assertIsNone(task.active_branch_id)
        names = [name for name, _payload in host.events]
        self.assertIn("portal_authentication_branch_retry", names)

    def test_retry_budget_is_bounded(self):
        host, _task, _auth_obj = _failed_branch(limit=1)
        host.gp_exploration_tree.branches[13]["authentication_retry_count"] = 1
        host.finish_portal_authentication_branch(now=100.0)
        self.assertEqual(len(host.rejected), 1)

    def test_doorway_verdict_is_rejected_immediately(self):
        host, task, _auth_obj = _failed_branch()
        task.branches[13].failure_reason = "no_frontier_inside_doorway"
        task.branches[13].state = "failed"
        host.finish_portal_authentication_branch(now=100.0)
        self.assertEqual(len(host.rejected), 1)
        self.assertEqual(host.rejected[0][0], 13)
        self.assertEqual(task.branches[13].state, "failed")


class GateIntegrationTest(unittest.TestCase):
    """The frontier-level gate, replayed as the 231045 timeline."""

    def _fixture(self, state="turning"):
        host = GPBranchFrontierMixin()
        auth = _auth(state)
        auth.target_yaw = 1.0
        host.pose_odom = SimpleNamespace(x=0.1, y=0.0, theta=0.0)
        host.active_timeout = 20.0
        host.portal_authentication_handoff_budget = 30.0
        host.portal_authentication_stale_route_grace = 15.0
        host.active_portal_authentication = lambda: auth
        host.junction_portal_authentication = None
        host.turn_supervisor_status_monotonic = 0.0
        host.turn_supervisor_state = "PASS_THROUGH"
        # Entry/return routes are judged only after bridge adoption; the
        # fixture supplies an adopted contract so the legacy motion budget
        # still applies to those phases.
        host.active_route_dispatch_contract = {"action_generation": 1}
        host.events = []
        host.releases = []
        host.publish_status = lambda name, **kw: host.events.append((name, kw))
        host.release_active_frontier = lambda **kw: host.releases.append(kw)
        return host, auth

    def test_pass_through_does_not_condemn_the_doorway(self):
        host, auth = self._fixture("turning")
        host.turn_supervisor_status_monotonic = 100.0
        # 24 s of PASS_THROUGH, exactly the 231045 window.
        for now in (100.0, 110.0, 124.0):
            self.assertFalse(
                host.check_portal_authentication_physical_progress(now))
            host.turn_supervisor_status_monotonic = now
        self.assertEqual(auth.state, "turning")
        self.assertIsNone(auth.failure_reason)
        self.assertEqual(host.releases, [])
        self.assertEqual(auth._phase_handoff_since, 100.0)

    def test_turn_that_never_starts_becomes_a_retryable_handoff_failure(self):
        host, auth = self._fixture("turning")
        host.turn_supervisor_status_monotonic = 100.0
        host.check_portal_authentication_physical_progress(100.0)
        for now in (120.0, 131.0):
            host.turn_supervisor_status_monotonic = now
            handled = host.check_portal_authentication_physical_progress(now)
        self.assertTrue(handled)
        self.assertEqual(auth.state, "returning")
        self.assertEqual(auth.failure_reason,
                         "authentication_handoff_stall:turning")
        self.assertEqual(len(host.releases), 1)
        names = [name for name, _payload in host.events]
        self.assertIn("portal_authentication_controller_handoff", names)
        # The doorway itself is still retryable, never "authentication_failed".
        ledger = Harness()
        self.assertIsNotNone(ledger.portal_authentication_retry_reason(
            auth, {"authentication_retry_count": 0}))

    def test_started_turn_still_expires_on_real_stall(self):
        host, auth = self._fixture("turning")
        host.turn_supervisor_status_monotonic = 100.0
        host.turn_supervisor_state = "TURNING"
        host.check_portal_authentication_physical_progress(100.0)
        for now in (110.0, 120.0, 130.0):
            host.turn_supervisor_status_monotonic = now
            handled = host.check_portal_authentication_physical_progress(now)
        self.assertTrue(handled)
        self.assertTrue(str(auth.failure_reason).startswith(
            "authentication_physical_stall:"))
        self.assertGreaterEqual(len(host.releases), 1)


if __name__ == "__main__":
    unittest.main()
