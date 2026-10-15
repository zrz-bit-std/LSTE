"""Regression for equivalent connector turn identity."""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from teb_turn_supervisor_turn_lifecycle import (  # noqa: E402
    TebTurnSupervisorTurnLifecycleMixin,
)


class TurnIdentityFixture(TebTurnSupervisorTurnLifecycleMixin):
    def __init__(self):
        self.completed_turn_key = (1.0, 2.0, 0.0)
        self.turn_min_clearance = 0.32
        self.yaw_goal_tolerance = 0.35

    @staticmethod
    def _goal_xy(goal):
        return float(goal.x), float(goal.y)


class TebTurnSupervisorIdentityTest(unittest.TestCase):
    def test_map_refinement_does_not_restart_completed_connector(self):
        fixture = TurnIdentityFixture()
        equivalent = SimpleNamespace(x=1.1, y=2.05)
        self.assertTrue(
            fixture._completed_turn_matches_locked(equivalent, 0.2)
        )
        self.assertFalse(
            fixture._completed_turn_matches_locked(equivalent, 0.5)
        )


if __name__ == "__main__":
    unittest.main()
