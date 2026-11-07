"""Regression tests for the in-place context observation policy."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from goal_context_observation import ContextObservationPolicy  # noqa: E402


class ContextObservationPolicyTest(unittest.TestCase):
    def test_moving_approach_does_not_trigger_observation(self):
        policy = ContextObservationPolicy(trigger_seconds=4.0)
        self.assertEqual(
            policy.update(0.0, (0.0, 0.0), (3.0, 0.0))["state"],
            "approach",
        )
        self.assertEqual(
            policy.update(1.0, (0.2, 0.0), (3.0, 0.0))["state"],
            "approach",
        )
        self.assertEqual(
            policy.update(2.0, (0.4, 0.0), (3.0, 0.0))["state"],
            "approach",
        )

    def test_stalled_approach_switches_to_in_place_observation(self):
        policy = ContextObservationPolicy(trigger_seconds=4.0)
        policy.update(0.0, (0.0, 0.0), (3.0, 0.0))
        action = policy.update(4.1, (0.01, 0.0), (3.0, 0.0))
        self.assertEqual(action["state"], "observe")
        self.assertIn("yaw_offset", action)
        self.assertEqual(policy.state, "observe")

    def test_reached_goal_switches_to_observation(self):
        policy = ContextObservationPolicy(reached_radius=0.55)
        policy.update(0.0, (0.0, 0.0), (1.0, 0.0))
        action = policy.update(0.5, (0.5, 0.0), (1.0, 0.0))
        self.assertEqual(action["state"], "observe")

    def test_observation_sweeps_then_abandons(self):
        policy = ContextObservationPolicy(
            trigger_seconds=1.0,
            observe_seconds=6.0,
            hold_seconds=2.0,
            sweep_deg=(0.0, 30.0, -30.0),
        )
        policy.update(0.0, (0.0, 0.0), (3.0, 0.0))
        first = policy.update(1.1, (0.0, 0.0), (3.0, 0.0))
        second = policy.update(3.2, (0.0, 0.0), (3.0, 0.0))
        self.assertEqual(first["state"], "observe")
        self.assertEqual(second["state"], "observe")
        self.assertNotEqual(first["yaw_offset"], second["yaw_offset"])
        final = policy.update(8.0, (0.0, 0.0), (3.0, 0.0))
        self.assertEqual(final["state"], "abandon")

    def test_target_evidence_wins_over_observation(self):
        policy = ContextObservationPolicy(trigger_seconds=1.0)
        policy.update(0.0, (0.0, 0.0), (3.0, 0.0))
        policy.update(1.1, (0.0, 0.0), (3.0, 0.0))
        self.assertEqual(
            policy.update(
                2.0, (0.0, 0.0), (3.0, 0.0), target_detected=True,
            )["state"],
            "target",
        )

    def test_locked_wins_over_observation(self):
        policy = ContextObservationPolicy()
        self.assertEqual(
            policy.update(
                0.0, (0.0, 0.0), (3.0, 0.0), locked=True,
            )["state"],
            "target",
        )


if __name__ == "__main__":
    unittest.main()
