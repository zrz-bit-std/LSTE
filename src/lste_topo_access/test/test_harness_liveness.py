"""Harness and liveness fixes for the four failure classes of 2026-11-03.

* a leftover stall watchdog killed the *next* attempt's roslaunch 17 s after
  launch (20261103_062322) because ``terminate`` matched any
  ``t_junction_nav.launch``;
* ``lste_navigation_metrics`` segfaulted twice at ~35-40 min and the runner
  sealed those attempts (20261103_010504, 20261103_035307);
* a killed attempt was recorded as ``launch_failed`` without ever being
  audited, although its bag and metrics were intact;
* a mission that had already settled every durable obligation idled forever
  "awaiting sensor stability" until the watchdog killed it
  (20261102_214754, 20261103_001037).
"""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "src/lste_topo_access/scripts"
TESTS = ROOT / "scripts/tests/targeted_navigation"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
if str(TESTS) not in sys.path:
    sys.path.insert(0, str(TESTS))

from l_stall_watchdog import launch_pids_for_run  # noqa: E402


class WatchdogScopeTest(unittest.TestCase):
    LINES = [
        "1234 /usr/bin/python3 /opt/ros/noetic/bin/roslaunch "
        "/ws/t_junction_nav.launch metrics_run_directory:=/logs/20261103_053045",
        "5678 /usr/bin/python3 /opt/ros/noetic/bin/roslaunch "
        "/ws/t_junction_nav.launch metrics_run_directory:=/logs/20261103_062325",
    ]

    def test_only_the_owning_roslaunch_is_selected(self):
        self.assertEqual(
            launch_pids_for_run(self.LINES, "/logs/20261103_053045"), [1234])
        self.assertEqual(
            launch_pids_for_run(self.LINES, "/logs/20261103_062325"), [5678])

    def test_an_unknown_run_selects_nothing(self):
        self.assertEqual(launch_pids_for_run(self.LINES, "/logs/20260101_000000"), [])

    def test_framework_shutdown_is_ignored(self):
        self.assertEqual(
            launch_pids_for_run(["12 python roslaunch other.launch"], "/logs/x"), [])


class KilledAttemptAuditTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.suite = (TESTS / "run_place_dfs_suite.sh").read_text(encoding="utf-8")
        cls.driver = (TESTS / "run_four_until_pass.sh").read_text(encoding="utf-8")

    def test_the_suite_can_audit_an_existing_root(self):
        self.assertIn("LSTE_AUDIT_ONLY_ROOT", self.suite)
        self.assertIn("if [[ -z \"${LSTE_AUDIT_ONLY_ROOT:-}\" ]]; then", self.suite)

    def test_the_driver_scores_a_killed_recording(self):
        self.assertIn("killed before the audit", self.driver)
        self.assertIn('LSTE_AUDIT_ONLY_ROOT="$root"', self.driver)

    def test_the_driver_sweeps_stale_watchdogs(self):
        self.assertIn('pgrep -f "l_stall_watchdog.py"', self.driver)
        self.assertIn('pkill -TERM -f "l_stall_watchdog.py --suite-root $root"',
                      self.driver)


class MetricsRestartTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launch = (TESTS / "t_junction_nav.launch").read_text(encoding="utf-8")
        cls.readiness = (TESTS / "readiness.sh").read_text(encoding="utf-8")

    def test_the_metrics_writer_is_respawned_with_faulthandler(self):
        block = self.launch.split('name="lste_navigation_metrics"', 1)[1]
        block = block.split("</node>", 1)[0]
        self.assertIn('respawn="true"', block)
        self.assertIn("PYTHONFAULTHANDLER", block)

    def test_the_runner_ignores_a_respawned_metrics_death(self):
        self.assertIn("grep -v 'lste_navigation_metrics'", self.readiness)


class BranchWaitFixture:
    def __init__(self, timeout=30.0):
        from global_frontier_planning_snapshot import (
            GlobalFrontierPlanningSnapshotMixin,
        )

        class Fixture(GlobalFrontierPlanningSnapshotMixin):
            pass

        self.obj = Fixture()
        self.obj.gp_portal_assistance_enabled = True
        self.obj.gp_exploration_tree = SimpleNamespace(
            branches={7: {"id": 7, "state": "pending"}})
        self.obj.frontier_exhausted = False
        self.obj.branch_wait_timeout = timeout
        self.obj.frontier_validation_budget_exhausted = False
        self.obj.frontier_validation_pending = False
        self.obj.portal_transaction = None
        self.obj.graph_completion_state = None
        self.obj.current_physical_place_id = 1
        self.obj.place_memory_enabled = True
        for _name in (
            "last_place_graph_hop_skips",
            "last_work_item_failed_viewpoint_skips",
            "last_portal_probe_viewpoint_rejections",
            "last_graph_policy_rejections",
            "last_portal_sources",
            "last_portal_candidates_seen",
            "last_portal_hard_rejected",
            "last_portal_executable_candidates",
            "last_portal_missing_transition_goals",
            "last_portal_unbound_reprojection_skips",
            "last_observed_place_reentry_skips",
        ):
            setattr(self.obj, _name, 0)
        self.obj.last_graph_route_plan = None
        self.obj.clearance = 1.0
        self.obj.frontier_clearance = 1.0
        self.obj.pose_odom = SimpleNamespace(x=0.0, y=0.0, theta=0.0)
        self.obj.clock = 100.0
        self.obj.time_provider = lambda: self.obj.clock
        self.obj.events = []
        self.obj.publish_status = lambda event, **kw: self.obj.events.append(
            (event, kw))

    def report(self, clock=None, x=None):
        if clock is not None:
            self.obj.clock = float(clock)
        if x is not None:
            self.obj.pose_odom.x = float(x)
        self.obj._report_no_frontier_selection()


class BranchWaitTimeoutTest(unittest.TestCase):
    def test_the_wait_expires_and_declares_exhaustion(self):
        fixture = BranchWaitFixture()
        fixture.report(100.0)
        self.assertFalse(fixture.obj.frontier_exhausted)
        fixture.report(120.0)
        self.assertFalse(fixture.obj.frontier_exhausted)
        fixture.report(131.0)
        self.assertTrue(fixture.obj.frontier_exhausted)
        self.assertEqual(fixture.obj.events[-1][0], "gp_branch_wait_expired")
        self.assertEqual(fixture.obj.events[-1][1]["pending_branch_ids"], [7])

    def test_movement_resets_the_wait(self):
        fixture = BranchWaitFixture()
        fixture.report(100.0)
        fixture.report(120.0, x=1.0)      # the base is still exploring
        fixture.report(131.0)             # only 11 s since the move
        self.assertFalse(fixture.obj.frontier_exhausted)
        fixture.report(152.0)
        self.assertTrue(fixture.obj.frontier_exhausted)

    def test_a_settled_tree_reaches_the_completion_gate(self):
        # Run 20261103_124742: the tree was built, every obligation settled,
        # and then the empty branch list was mistaken for the startup wait.
        from unittest import mock
        fixture = BranchWaitFixture()
        fixture.report(100.0)
        self.assertEqual(fixture.obj.events[-1][0], "gp_branch_waiting")
        fixture.obj.gp_exploration_tree = SimpleNamespace(branches={})
        fixture.obj.events.clear()
        with mock.patch("rospy.logwarn_throttle"), mock.patch("rospy.logwarn"):
            fixture.report(101.0)
        names = [event for event, _payload in fixture.obj.events]
        self.assertNotIn("gp_branch_waiting", names)
        self.assertTrue(fixture.obj.frontier_exhausted)

    def test_no_pending_branches_does_not_expire(self):
        fixture = BranchWaitFixture()
        fixture.obj.gp_exploration_tree = SimpleNamespace(branches={})
        fixture.report(100.0)
        fixture.report(200.0)
        self.assertFalse(fixture.obj.frontier_exhausted)


class BranchWaitConfigTest(unittest.TestCase):
    def test_the_timeout_is_a_parameter(self):
        config = (SCRIPTS / "global_frontier_config_exploration.py").read_text(
            encoding="utf-8")
        self.assertIn("branch_wait_timeout", config)


if __name__ == "__main__":
    unittest.main(verbosity=2)
