"""Campaign contract: perception LOCKED ends an attempt as success."""

from pathlib import Path
import sys
import unittest


TOOLS = Path(__file__).resolve().parents[3] / "scripts" / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

import audit_search_attempt as audit
import run_ten_attempts as campaign


class LockedSuccessTest(unittest.TestCase):
    def test_state_two_ends_the_attempt(self):
        records = [
            ("sample", {"pose": [0.0, 0.0]}),
            ("state_change", {"state": 1, "current": "1:Sus-B"}),
            ("state_change", {"state": 2, "current": "2:Locked"}),
        ]
        self.assertEqual(campaign.locked_state(records)["current"], "2:Locked")

    def test_suspicious_state_does_not_end_the_attempt(self):
        records = [
            ("state_change", {"state": 0, "current": "0:-"}),
            ("state_change", {"state": 1, "current": "1:Sus-B"}),
        ]
        self.assertIsNone(campaign.locked_state(records))

    def test_task_done_is_not_locked_state(self):
        self.assertIsNone(campaign.locked_state([("task_done", {"value": True})]))

    def test_non_numeric_state_is_ignored(self):
        records = [
            ("state_change", {"state": None}),
            ("state_change", {"state": "locked"}),
        ]
        self.assertIsNone(campaign.locked_state(records))

    def test_audit_counts_recorded_locked_as_success_evidence(self):
        rows = [
            {"kind": "state", "state": 0},
            {"kind": "state", "state": 2},
        ]
        self.assertTrue(audit.state_rows_show_locked(rows))
        self.assertFalse(audit.state_rows_show_locked([{"kind": "state", "state": 1}]))


class TotalLimitTest(unittest.TestCase):
    def test_zero_limit_means_unlimited(self):
        self.assertFalse(campaign.total_limit_exceeded(10_000_000.0, 0))
        self.assertFalse(campaign.total_limit_exceeded(10_000_000.0, -1))

    def test_positive_limit_still_applies(self):
        self.assertTrue(campaign.total_limit_exceeded(11.0, 10.0))
        self.assertFalse(campaign.total_limit_exceeded(9.0, 10.0))


if __name__ == "__main__":
    unittest.main()
