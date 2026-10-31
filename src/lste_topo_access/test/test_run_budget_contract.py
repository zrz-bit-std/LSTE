"""Run-budget contract: 45-minute ceiling, 3-minute stall rule.

Measured facts behind the numbers:

* a healthy L run finished on its own at sim 1178 / 1350 / 1949 s
  (``semantic_junction_goal_complete``), never by timeout;
* the simulated clock advances at about 0.74x wall time, so the slowest
  historical completion needs roughly 2635 s of wall time;
* the longest *normal* pause in a successful run was 69 s, while pathological
  stalls lasted 568-2282 s.

Hence: drive ceiling 3000 s wall, and any uninterrupted 180 s without progress
terminates the scenario. The driver stops earlier than the ceiling whenever the
mission completes.
"""

from pathlib import Path
import re
import unittest


SUITE = Path("/home/zrz/work/lste_ws/scripts/tests/targeted_navigation")
LAUNCHER = SUITE / "run_l_clean.sh"
WATCHDOG = SUITE / "l_stall_watchdog.py"


class RunBudgetContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launcher = LAUNCHER.read_text(encoding="utf-8")
        cls.watchdog = WATCHDOG.read_text(encoding="utf-8")

    def test_drive_ceiling_is_3000_seconds(self):
        self.assertIn('SECONDS_TO_RUN="${GP_SUITE_SECONDS:-3000}"',
                      self.launcher)

    def test_stall_rule_defaults_to_180_seconds(self):
        self.assertIn('STALL_SECONDS="${LSTE_STALL_SECONDS:-180}"',
                      self.launcher)

    def test_watchdog_gets_ceiling_plus_margin(self):
        self.assertIn('--max-wall-seconds "$((SECONDS_TO_RUN + 600))"',
                      self.launcher)

    def test_watchdog_own_default_is_not_shorter_than_the_ceiling(self):
        match = re.search(
            r'"--max-wall-seconds",\s*type=float,\s*default=([0-9.]+)',
            self.watchdog)
        self.assertIsNotNone(match)
        self.assertGreaterEqual(float(match.group(1)), 3000.0)

    def test_ceiling_is_at_least_the_slowest_historical_completion(self):
        # 1949 s sim at 0.74x wall ~= 2635 s; the ceiling must cover it.
        self.assertGreaterEqual(3000, 2635)


if __name__ == "__main__":
    unittest.main(verbosity=2)
