"""Witness latency must not be charged against the physical sweep.

Run ``20261102_000051`` (L attempt 2, 14/15) failed ``turn_raw`` twice for two
different, both-spurious reasons:

* node 12 branch 39: ``initial_physical_yaw_witness_missing`` because the frozen
  yaw (-1.1991) was 0.1427 rad from the nearest raw yaw in the preceding 1.5 s
  window.  The node's pose cache is coalesced through 0.6 s planning cycles, so
  during a 0.65 rad/s sweep it trails the raw odom stream.
* node 6 branch 14: after widening the witness tolerance, the sweep was measured
  from a witness 0.077 rad inside the turn, so a correct 2.72 rad sweep measured
  2.641 and failed the pi-0.13 threshold by 0.021.

The witness is now searched with a 0.15 rad latency allowance, and the sweep is
measured from the *frozen* yaw rather than from the witness, so the two
criteria stay independent.  A short or wrongly signed turn still fails.
"""

import math
from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[3] / "scripts/tests/targeted_navigation"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from audit_portal_turn_directions import (  # noqa: E402
    WITNESS_TOLERANCE, WITNESS_TOLERANCE_MAX, witness_tolerance, audit_turns,
)


PI = math.pi


def sweep_poses(start, initial_yaw, direction, offset, sweep_radians,
                samples=40, rate=0.05):
    """Raw poses rotating from ``initial_yaw + offset`` by ``sweep_radians``."""
    sign = 1.0 if direction == "counterclockwise" else -1.0
    poses = [(start - 0.5, initial_yaw + offset),
             (start, initial_yaw + offset)]
    for index in range(1, samples + 1):
        poses.append((
            start + index * rate,
            initial_yaw + offset + sign * sweep_radians * index / samples,
        ))
    return poses


def approach_poses(start, yaw_at_start, rate, seconds=1.5, step=0.05):
    """A window that is already rotating, like a real turn start."""
    count = int(seconds / step)
    return [(start - seconds + i * step,
             yaw_at_start - (seconds - i * step) * rate)
            for i in range(count + 1)]


def turn(initial_yaw, direction, start, completed):
    sign = 1.0 if direction == "counterclockwise" else -1.0
    return {
        (6, 14): {
            "initial_yaw": initial_yaw,
            "target_yaw": initial_yaw + sign * (PI - 0.08),
            "direction": direction,
            "start": start,
            "completed": completed,
        }
    }


class WitnessLatencyTest(unittest.TestCase):
    def test_a_witness_inside_the_latency_allowance_is_accepted(self):
        poses = sweep_poses(100.0, -0.4711, "counterclockwise", 0.077, 2.72)
        result = audit_turns(poses, turn(-0.4711, "counterclockwise", 100.0, 102.1))
        self.assertTrue(result["passed"], result["errors"])
        self.assertAlmostEqual(result["results"][0]["witness_offset"], 0.077, places=3)

    def test_a_witness_beyond_the_allowance_is_rejected(self):
        poses = sweep_poses(100.0, -1.1991, "counterclockwise",
                            WITNESS_TOLERANCE_MAX + 0.12, 2.72)
        result = audit_turns(poses, turn(-1.1991, "counterclockwise", 100.0, 102.1))
        self.assertFalse(result["passed"])
        self.assertEqual(result["errors"][0]["reason"],
                         "initial_physical_yaw_witness_missing")

    def test_the_measured_worst_case_is_accepted(self):
        # Run 20261102_094214 node 24 branch 69: 0.3344 rad at 0.82 rad/s.
        poses = approach_poses(100.0, -0.4711 + 0.3344, 0.82)
        poses += sweep_poses(100.0, -0.4711, "counterclockwise", 0.3344, 2.72)[2:]
        result = audit_turns(poses, turn(-0.4711, "counterclockwise", 100.0, 102.1))
        self.assertTrue(result["passed"], result["errors"])

    def test_the_allowance_scales_with_the_turn_rate(self):
        slow = witness_tolerance(0.05)
        fast = witness_tolerance(0.82)
        self.assertAlmostEqual(slow, WITNESS_TOLERANCE, places=3)
        self.assertAlmostEqual(fast, 0.492, places=3)
        self.assertLessEqual(witness_tolerance(5.0), WITNESS_TOLERANCE_MAX)

    def test_the_sweep_is_measured_from_the_frozen_yaw(self):
        # 2.641 rad of raw rotation plus a 0.077 rad witness offset is a full
        # sweep; the same rotation without the offset is not.
        with_offset = sweep_poses(100.0, -0.4711, "counterclockwise", 0.077, 2.641)
        result = audit_turns(
            with_offset, turn(-0.4711, "counterclockwise", 100.0, 102.1))
        self.assertTrue(result["passed"], result["errors"])
        self.assertAlmostEqual(result["results"][0]["sweep"], 2.718, places=2)

        without = sweep_poses(100.0, -0.4711, "counterclockwise", 0.0, 2.30)
        result = audit_turns(without, turn(-0.4711, "counterclockwise", 100.0, 102.1))
        self.assertFalse(result["passed"])
        self.assertEqual(result["errors"][0]["reason"],
                         "physical_locked_sweep_not_proven")

    def test_a_clockwise_sweep_is_measured_with_its_own_sign(self):
        poses = sweep_poses(100.0, 2.0, "clockwise", 0.05, PI - 0.08)
        result = audit_turns(poses, turn(2.0, "clockwise", 100.0, 102.1))
        self.assertTrue(result["passed"], result["errors"])
        # directed_rotation is sign-normalised: a correct sweep is ~pi for
        # both directions.
        self.assertGreater(result["results"][0]["directed_rotation"], PI - 0.2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
