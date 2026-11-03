"""A doorway whose measured section keeps flipping must not be retried forever."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/lste_topo_access/scripts"))

import gp_branch_frontier as frontier  # noqa: E402


class WithdrawnCandidateRetryTest(unittest.TestCase):
    def branch(self, previous, current):
        record = {"physical_gate": current}
        if previous is not None:
            record["authentication_withdrawn_gate"] = previous
        return record

    def test_the_same_gate_does_not_retry(self):
        # 20261103_191849 branch 11: gate moved 0.02 m across 28 withdraw cycles.
        self.assertFalse(frontier.withdrawn_candidate_retryable(
            self.branch([0.13, 18.95], [0.13, 18.93])))

    def test_a_moved_gate_retries(self):
        self.assertTrue(frontier.withdrawn_candidate_retryable(
            self.branch([0.13, 18.93], [0.40, 18.93])))

    def test_a_never_withdrawn_candidate_retries(self):
        self.assertTrue(frontier.withdrawn_candidate_retryable(
            {"physical_gate": [0.13, 18.93]}))


class WithdrawalTerminalTest(unittest.TestCase):
    def test_two_withdrawals_are_not_terminal(self):
        self.assertFalse(frontier.withdrawal_is_terminal(2, 3))

    def test_the_third_withdrawal_is_terminal(self):
        self.assertTrue(frontier.withdrawal_is_terminal(3, 3))

    def test_the_limit_is_at_least_one(self):
        self.assertTrue(frontier.withdrawal_is_terminal(1, 0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
