"""A repeated alignment that never moves the base is a cycle, not progress.

Run 20261102_174505 sat at (-14.97, -3.35) from sim 1612 to 1660.  The turn
supervisor started six turns in 80 s, alternating between
``portal_entry_alignment`` (-141 deg), ``stalled_route_reorientation``
(-93 deg) and the re-planned successors (-139, -93, -129).  Both phases aim at
the Navfn initial tangent of *their own* plan, and every controller stall
produced a new plan with a slightly different tangent, so the base chased a
heading it could never hold.  Zero translation -> the DFS return budget expired
60 s later -> ``place_dfs_return_unrecoverable``.

The supervisor now remembers the goal, the pose anchor and the number of
completed alignments.  Two completions for the same goal with less than 0.15 m
of travel raise ``reorientation_cycle_detected`` (about 8-10 s after the first
alignment) and lock out every further turn for that goal.  A goal that moves
more than 0.5 m is a genuinely different target and clears the lock.
"""

from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from teb_turn_supervisor_turn_lifecycle import (  # noqa: E402
    TebTurnSupervisorTurnLifecycleMixin,
)


class CycleFixture(TebTurnSupervisorTurnLifecycleMixin):
    def __init__(self, goal=(10.0, 0.0)):
        self.pose = SimpleNamespace(x=0.0, y=0.0, theta=0.0)
        self.turn_phase = "portal_entry_alignment"
        self.turn_cycle_goal_xy = None
        self.turn_cycle_pose_anchor_xy = None
        self.turn_cycle_completions = 0
        self.turn_cycle_blocked_goal_xy = None
        self.turn_cycle_reported_goal_xy = None
        self.turn_cycle_translation_m = 0.15
        self.turn_cycle_limit = 2
        self.turn_cycle_goal_tolerance_m = 0.25
        self.turn_cycle_block_radius_m = 0.50
        self.turn_cycle_activation_grace_s = 3.0
        self.turn_cycle_last_completion_wall = 0.0
        self.active_action_goal = goal
        self.clock = 100.0
        self.time_provider = lambda: self.clock
        self.events = []

    def publish_status_locked(self, event, **fields):
        self.events.append((event, fields))

    def _goal_xy(self, goal):
        return (float(goal[0]), float(goal[1]))

    def complete_at(self, x, phase="portal_entry_alignment", goal=(10.0, 0.0)):
        self.pose.x = float(x)
        self.turn_phase = phase
        self._note_turn_completion_for_cycle_locked(list(goal), True)


class CycleDetectionTest(unittest.TestCase):
    def test_one_alignment_is_not_a_cycle(self):
        fixture = CycleFixture()
        fixture.complete_at(0.0, "portal_entry_alignment")
        self.assertEqual(fixture.turn_cycle_completions, 1)
        self.assertIsNone(fixture.turn_cycle_blocked_goal_xy)
        self.assertEqual(fixture.events, [])

    def test_two_alignments_without_travel_is_a_cycle(self):
        fixture = CycleFixture()
        fixture.complete_at(0.0, "portal_entry_alignment")
        fixture.complete_at(0.01, "stalled_route_reorientation")
        self.assertEqual(fixture.turn_cycle_completions, 2)
        self.assertIsNotNone(fixture.turn_cycle_blocked_goal_xy)
        self.assertEqual(
            [event for event, _fields in fixture.events],
            ["reorientation_cycle_detected"],
        )
        fields = fixture.events[0][1]
        self.assertEqual(fields["reason"], "repeated_alignment_without_translation")
        self.assertLess(fields["travel_since_anchor_m"], 0.15)

    def test_real_travel_resets_the_counter(self):
        fixture = CycleFixture()
        fixture.complete_at(0.0, "portal_entry_alignment")
        fixture.complete_at(0.30, "stalled_route_reorientation")
        self.assertEqual(fixture.turn_cycle_completions, 1)
        self.assertIsNone(fixture.turn_cycle_blocked_goal_xy)

    def test_a_new_goal_restarts_the_counter(self):
        fixture = CycleFixture()
        fixture.complete_at(0.0, goal=(10.0, 0.0))
        fixture.complete_at(0.0, goal=(30.0, 0.0))
        self.assertEqual(fixture.turn_cycle_completions, 1)
        self.assertIsNone(fixture.turn_cycle_blocked_goal_xy)

    def test_one_alignment_plus_a_static_grace_blocks_the_next_turn(self):
        # The second turn in run 20261102_174505 took 11 s to finish; the gate
        # must trip from the activation path instead, ~4-6 s after the first.
        fixture = CycleFixture()
        fixture.complete_at(0.0)
        self.assertFalse(fixture._turn_cycle_blocks_action_locked())
        fixture.clock += 4.0
        self.assertTrue(fixture._turn_cycle_blocks_action_locked())
        events = [event for event, _fields in fixture.events]
        self.assertEqual(events[0], "reorientation_cycle_detected")
        self.assertIn("reorientation_cycle_blocked", events)

    def test_translation_inside_the_grace_keeps_the_turn_allowed(self):
        fixture = CycleFixture()
        fixture.complete_at(0.0)
        fixture.pose.x = 0.40          # the base actually moved
        fixture.clock += 4.0
        self.assertFalse(fixture._turn_cycle_blocks_action_locked())

    def test_an_authentication_turn_does_not_count(self):
        fixture = CycleFixture()
        fixture.complete_at(0.0, "portal_authentication_turn")
        fixture.complete_at(0.0, "portal_authentication_turn")
        self.assertEqual(fixture.turn_cycle_completions, 0)
        self.assertIsNone(fixture.turn_cycle_blocked_goal_xy)

    def test_an_uncompleted_release_does_not_count(self):
        fixture = CycleFixture()
        fixture.pose.x = 0.0
        fixture._note_turn_completion_for_cycle_locked([10.0, 0.0], False)
        fixture._note_turn_completion_for_cycle_locked([10.0, 0.0], False)
        self.assertEqual(fixture.turn_cycle_completions, 0)


class HeadingAuthorityTest(unittest.TestCase):
    def blocked(self):
        fixture = CycleFixture()
        fixture.complete_at(0.0)
        fixture.complete_at(0.0)
        return fixture

    def test_the_same_goal_is_locked_out(self):
        fixture = self.blocked()
        self.assertTrue(fixture._turn_cycle_blocks_action_locked())
        names = [event for event, _fields in fixture.events]
        self.assertIn("reorientation_cycle_detected", names)
        self.assertIn("reorientation_cycle_blocked", names)

    def test_the_lockout_is_reported_once(self):
        fixture = self.blocked()
        for _ in range(4):
            fixture._turn_cycle_blocks_action_locked()
        self.assertEqual(
            [event for event, _fields in fixture.events].count(
                "reorientation_cycle_blocked"),
            1,
        )

    def test_a_different_goal_clears_the_lockout(self):
        fixture = self.blocked()
        fixture.active_action_goal = (10.0, 2.0)   # 2 m away: a new target
        self.assertFalse(fixture._turn_cycle_blocks_action_locked())
        self.assertIsNone(fixture.turn_cycle_blocked_goal_xy)
        self.assertEqual(fixture.turn_cycle_completions, 0)

    def test_a_nearby_goal_keeps_the_lockout(self):
        fixture = self.blocked()
        fixture.active_action_goal = (10.2, 0.1)   # same dead end
        self.assertTrue(fixture._turn_cycle_blocks_action_locked())

    def test_an_authentication_turn_is_never_blocked(self):
        fixture = self.blocked()
        fixture.latest_authentication_turn_required = True
        self.assertFalse(fixture._turn_cycle_blocks_action_locked())


class ActivationGateTest(unittest.TestCase):
    def test_activation_is_refused_while_the_cycle_lock_is_held(self):
        source = (
            SCRIPTS / "teb_turn_supervisor_turn_lifecycle.py").read_text(
                encoding="utf-8")
        body = source.split("def _activate_turn_locked", 1)[1]
        self.assertIn("_turn_cycle_blocks_action_locked", body.split(
            "def ", 1)[0])


class ParameterContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        base = SCRIPTS
        cls.params = (base / "teb_turn_supervisor_parameters.py").read_text(
            encoding="utf-8")
        cls.state = (base / "teb_turn_supervisor_state.py").read_text(
            encoding="utf-8")

    def test_the_cycle_thresholds_are_parameters(self):
        for name in ("turn_cycle_translation_m", "turn_cycle_limit",
                     "turn_cycle_goal_tolerance_m", "turn_cycle_block_radius_m"):
            self.assertIn(name, self.params)

    def test_the_cycle_bookkeeping_is_initialised(self):
        for name in ("turn_cycle_goal_xy", "turn_cycle_pose_anchor_xy",
                     "turn_cycle_completions", "turn_cycle_blocked_goal_xy"):
            self.assertIn(name, self.state)

    def test_detection_is_bounded_to_two_alignments(self):
        self.assertIn('gp("~turn_cycle_limit", 2)', self.params)


if __name__ == "__main__":
    unittest.main(verbosity=2)
