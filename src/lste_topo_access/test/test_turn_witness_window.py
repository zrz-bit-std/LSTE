"""The turn witness must survive the observed node event-queue latency."""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "scripts/tests/targeted_navigation"))

import audit_portal_turn_directions as turn_audit  # noqa: E402


def yaw_at(t):
    if t <= 7.5:
        return 0.2
    if t <= 8.5:
        return 0.2 - 0.2 * (t - 7.5)
    if t <= 10.0:
        return 0.0
    if t <= 13.0:
        return (t - 10.0) / 3.0 * (math.pi + 0.2)
    return math.pi + 0.2


def build():
    poses = [(6.0 + 0.05 * i, yaw_at(6.0 + 0.05 * i)) for i in range(160)]
    turns = {(1, 2): {
        "initial_yaw": 0.2,
        "target_yaw": math.pi + 0.2,
        "direction": "counterclockwise",
        "start": 10.0,
        "completed": 13.0,
    }}
    return poses, turns


class TurnWitnessWindowTest(unittest.TestCase):
    def test_the_old_1_5_s_lookback_reports_a_missing_witness(self):
        poses, turns = build()
        result = turn_audit.audit_turns(poses, turns, witness_window=1.5)
        self.assertFalse(result["passed"])
        self.assertIn("initial_physical_yaw_witness_missing",
                      [e.get("reason") for e in result["errors"]])

    def test_the_observed_2_5_s_queue_latency_is_covered(self):
        poses, turns = build()
        result = turn_audit.audit_turns(poses, turns)
        self.assertTrue(result["passed"], result.get("errors"))

    def test_the_rate_allowance_still_bounds_the_witness(self):
        # A frozen yaw that no pose ever reached must still fail, however wide
        # the window becomes.
        poses, turns = build()
        turns[(1, 2)]["initial_yaw"] = 2.5
        result = turn_audit.audit_turns(poses, turns)
        self.assertFalse(result["passed"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
