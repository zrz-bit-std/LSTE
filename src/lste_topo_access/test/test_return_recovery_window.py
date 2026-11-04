"""A stalled return gets a real recovery window, not six rapid retries."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/lste_topo_access/scripts"))

import global_frontier_portal_authentication as auth  # noqa: E402


class ReturnRecoveryWindowTest(unittest.TestCase):
    def test_the_old_three_retries_no_longer_block_after_fifteen_seconds(self):
        # 20261103_210032: six ~5 s retries blocked at ~30 s while the costmap
        # blob was still present (cleared at ~48 s local / ~108 s global).
        self.assertFalse(auth.return_recovery_should_block(3, 15.0, 3, 45.0))

    def test_the_window_opens_after_the_real_costmap_clear(self):
        self.assertTrue(auth.return_recovery_should_block(9, 45.0, 3, 45.0))
        self.assertTrue(auth.return_recovery_should_block(22, 108.0, 6, 140.0) is False)

    def test_the_final_budget_still_needs_its_window(self):
        self.assertFalse(auth.return_recovery_should_block(28, 120.0, 6, 140.0))
        self.assertTrue(auth.return_recovery_should_block(28, 140.0, 6, 140.0))

    def test_the_count_bound_still_applies(self):
        self.assertFalse(auth.return_recovery_should_block(2, 200.0, 3, 45.0))

    def test_invalid_input_never_blocks(self):
        self.assertFalse(auth.return_recovery_should_block(None, None, 3, 45.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
