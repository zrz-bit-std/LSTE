"""A controller-owned failure must not wait forever for its terminal.

In persistent mode the controller is the failure authority, so the node holds
the route until a typed terminal arrives.  Run ``20261102_050319`` dispatched a
WorkItem at sim 1000.8 and then sat at ``route_stagnant_observed:
controller_owns_failure_terminal`` until the 180 s watchdog killed the attempt.

``controller_owned_route_failure`` is a mode flag (set from
``persistent_execution`` at startup), so the wait is measured from the route's
own stalled duration and falls back to the global watchdog after
``controller_terminal_wait_seconds``.  A terminal that arrives in time keeps
the controller as the authority.
"""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_route_lease import (  # noqa: E402
    AUTHORITY_CONTROLLER_TERMINAL,
    AUTHORITY_GLOBAL_WATCHDOG,
    route_lease_failure_decision,
)


class ControllerTerminalWaitTest(unittest.TestCase):
    def decide(self, **kwargs):
        return route_lease_failure_decision(
            controller_owns_failure=True,
            recovery_pending=kwargs.pop("recovery_pending", False),
            route_stalled=kwargs.pop("stalled", True),
            **kwargs,
        )

    def test_within_the_limit_the_controller_keeps_authority(self):
        decision = self.decide(stalled_for=10.0, controller_terminal_limit=45.0)
        self.assertFalse(decision.release)
        self.assertEqual(decision.reason, "controller_owns_failure_terminal")
        self.assertEqual(decision.authority, AUTHORITY_CONTROLLER_TERMINAL)

    def test_the_wait_is_bounded(self):
        decision = self.decide(stalled_for=45.1, controller_terminal_limit=45.0)
        self.assertTrue(decision.release)
        self.assertEqual(decision.reason, "controller_terminal_timeout")

    def test_a_pending_terminal_still_wins(self):
        decision = self.decide(
            recovery_pending=True,
            stalled_for=1000.0,
            controller_terminal_limit=45.0,
        )
        self.assertTrue(decision.release)
        self.assertEqual(decision.reason, "controller_terminal_failure")

    def test_without_a_limit_the_old_contract_is_unchanged(self):
        decision = self.decide(stalled_for=1000.0)
        self.assertFalse(decision.release)
        self.assertEqual(decision.reason, "controller_owns_failure_terminal")

    def test_non_controller_authority_is_untouched(self):
        decision = route_lease_failure_decision(
            controller_owns_failure=False,
            route_stalled=True,
            stalled_for=1000.0,
            controller_terminal_limit=45.0,
        )
        self.assertTrue(decision.release)
        self.assertEqual(decision.authority, AUTHORITY_GLOBAL_WATCHDOG)
        self.assertEqual(decision.reason, "stall")


class ConfigContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = (SCRIPTS / "global_frontier_config_navigation.py").read_text(
            encoding="utf-8"
        )
        cls.watchdog = (
            SCRIPTS / "global_frontier_execution_watchdogs.py"
        ).read_text(encoding="utf-8")

    def test_the_limit_is_a_parameter(self):
        self.assertIn("controller_terminal_wait_seconds", self.config)
        self.assertIn("controller_terminal_wait_seconds", self.watchdog)

    def test_the_wait_is_measured_from_route_progress(self):
        self.assertIn("active_progress_time", self.watchdog)
        self.assertIn("stalled_for=", self.watchdog)


if __name__ == "__main__":
    unittest.main(verbosity=2)
