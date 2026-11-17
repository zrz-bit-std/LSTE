"""The no-frontier reporting path must attempt a retired-Portal restore."""

from pathlib import Path
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_planning_snapshot import (  # noqa: E402
    GlobalFrontierPlanningSnapshotMixin,
)


class _Host(GlobalFrontierPlanningSnapshotMixin):
    gp_portal_assistance_enabled = False

    def __init__(self):
        self.exit_activation_attempted = False
        self.restore_attempted = False

    def _activate_place_exit_if_ready(self):
        self.exit_activation_attempted = True
        return False

    def _restore_portal_entries_if_exhausted(self):
        self.restore_attempted = True
        return True


class NoFrontierRestoreWiringTest(unittest.TestCase):
    def test_restore_runs_after_exit_activation(self):
        host = _Host()
        host._report_no_frontier_selection()
        self.assertTrue(host.exit_activation_attempted)
        self.assertTrue(host.restore_attempted)


if __name__ == "__main__":
    unittest.main()
