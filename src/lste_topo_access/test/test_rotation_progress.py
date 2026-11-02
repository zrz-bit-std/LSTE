"""An in-place alignment is progress, and a re-dispatch must not restart it.

Run 20261102_174505 sat in a pocket at (-14.96, -3.37) with 0.54 m ahead and a
clear 8 m corridor at 167 deg (the only exit).  Getting out needed one ~103 deg
rotation, which was allowed by the forward-only mux (pure rotation) and had
enough clearance (0.54 m > 0.30 m radius).

It never finished that rotation:

* the bridge refreshed its motion-health clock only on XY translation, so after
  ``progress_timeout`` (12 s) of pure rotation the action was cancelled with
  ``controller_stall_no_physical_progress`` (observed at 1624.1, 1639.0 and
  1654.0 -- every 15 s);
* each cancel re-dispatched the route, which changed the action identity and
  released the turn, so the next alignment started from zero.

Two fixes are covered here: rotation counts as motion health, and a portal
alignment continues across a re-dispatch instead of being released.
"""

from contextlib import nullcontext
from types import SimpleNamespace
import math
import sys
from pathlib import Path
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from teb_goal_bridge_teb_runtime import (  # noqa: E402
    TebGoalBridgeTebRuntimeMixin,
)
from teb_turn_supervisor_turn_lifecycle import (  # noqa: E402
    TebTurnSupervisorTurnLifecycleMixin,
)
from teb_turn_supervisor_contract import STATE_TURNING  # noqa: E402


def angle_delta(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


class BridgeFixture(TebGoalBridgeTebRuntimeMixin):
    _angle_delta = staticmethod(angle_delta)

    def __init__(self):
        self.lock = nullcontext()
        self.action_active = True
        self.progress_epsilon = 0.12
        self.progress_yaw_epsilon = 0.12
        self.active_motion_reference = (0.0, 0.0)
        self.active_motion_reference_yaw = None
        self.active_motion_progress_monotonic = 0.0
        self.teb_reorientation_started_monotonic = 0.0
        self.teb_reorientation_reference_yaw = None
        self.odom_yaw = None
        self.odom_pose_monotonic = 0.0
        self.turn_supervisor_state = "PASS_THROUGH"
        self.clock = 100.0
        self.time_provider = lambda: self.clock

    def sample(self, x, y, yaw, clock):
        self.clock = float(clock)
        self.on_pose(SimpleNamespace(x=float(x), y=float(y), theta=float(yaw)))


class RotationProgressTest(unittest.TestCase):
    def test_pure_rotation_refreshes_motion_health(self):
        bridge = BridgeFixture()
        bridge.sample(0.0, 0.0, 0.0, 100.0)
        bridge.sample(0.0, 0.0, 0.20, 101.0)   # 11 deg, no translation
        self.assertEqual(bridge.active_motion_progress_monotonic, 101.0)

    def test_rotation_counts_while_the_turn_supervisor_owns_the_turn(self):
        # This is the observed case: the bridge used to ignore yaw exactly
        # while the supervisor was TURNING, which is when it matters.
        bridge = BridgeFixture()
        bridge.turn_supervisor_state = "TURNING"
        bridge.sample(0.0, 0.0, 0.0, 100.0)
        bridge.sample(0.0, 0.0, 0.20, 105.0)
        self.assertEqual(bridge.active_motion_progress_monotonic, 105.0)

    def test_small_yaw_noise_does_not_refresh(self):
        bridge = BridgeFixture()
        bridge.sample(0.0, 0.0, 0.0, 100.0)
        bridge.sample(0.0, 0.0, 0.05, 101.0)   # 2.9 deg
        self.assertEqual(bridge.active_motion_progress_monotonic, 0.0)

    def test_yaw_is_accumulated_from_the_last_reference(self):
        bridge = BridgeFixture()
        bridge.sample(0.0, 0.0, 0.0, 100.0)
        bridge.sample(0.0, 0.0, 0.05, 101.0)
        bridge.sample(0.0, 0.0, 0.10, 102.0)
        self.assertEqual(bridge.active_motion_progress_monotonic, 0.0)
        bridge.sample(0.0, 0.0, 0.15, 103.0)
        self.assertEqual(bridge.active_motion_progress_monotonic, 103.0)

    def test_translation_still_refreshes(self):
        bridge = BridgeFixture()
        bridge.sample(0.0, 0.0, 0.0, 100.0)
        bridge.sample(0.30, 0.0, 0.0, 101.0)
        self.assertEqual(bridge.active_motion_progress_monotonic, 101.0)

    def test_a_wrapped_yaw_is_measured_across_pi(self):
        bridge = BridgeFixture()
        bridge.sample(0.0, 0.0, math.pi - 0.05, 100.0)
        bridge.sample(0.0, 0.0, -math.pi + 0.05, 101.0)   # 0.10 rad across pi
        self.assertEqual(bridge.active_motion_progress_monotonic, 0.0)
        bridge.sample(0.0, 0.0, -math.pi + 0.15, 102.0)   # 0.20 rad total
        self.assertEqual(bridge.active_motion_progress_monotonic, 102.0)


class SupervisorFixture(TebTurnSupervisorTurnLifecycleMixin):
    _goal_xy = staticmethod(
        lambda message: (
            float(message.pose.position.x), float(message.pose.position.y)
        )
    )

    def __init__(self, route_kind="portal_transition",
                 phase="portal_entry_alignment", refreshed_yaw=1.25):
        self.state = STATE_TURNING
        self.turn_phase = phase
        self.turn_action_identity = "old"
        self.turn_target_yaw = 1.20
        self.turn_goal_xy = (10.0, 0.0)
        self.turn_key = "old-key"
        self.yaw_goal_tolerance = 0.35
        self.stalled_route_turn_handoff_count = 0
        self.stalled_route_reorientation_attempted_identity = None
        self.active_action = True
        self.active_action_source = "global_slam_frontier"
        self.active_action_route_kind = route_kind
        self.active_action_goal = SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=10.0, y=0.0))
        )
        self.active_action_goal_context = {
            "goal_role": "place_observation", "mission_id": "m1"
        }
        self._refreshed_yaw = refreshed_yaw
        self.events = []

    def _initial_navfn_yaw_in_pose_frame_locked(self):
        return self._refreshed_yaw

    def _turn_key_for_goal_locked(self, goal, yaw):
        return "new-key"

    def publish_status_locked(self, event, **fields):
        self.events.append((event, fields))


class HandoffContinuationTest(unittest.TestCase):
    def continue_turn(self, fixture):
        return fixture._continue_stalled_route_turn_for_handoff_locked(
            "new", {"goal_role": "place_observation", "mission_id": "m1"})

    def test_a_portal_alignment_continues_across_a_redispatch(self):
        fixture = SupervisorFixture()
        self.assertTrue(self.continue_turn(fixture))
        self.assertEqual(fixture.turn_action_identity, "new")
        self.assertEqual(fixture.stalled_route_turn_handoff_count, 1)
        self.assertEqual(
            [event for event, _fields in fixture.events],
            ["stalled_route_turn_handoff_continued"],
        )

    def test_a_stalled_route_reorientation_still_continues(self):
        fixture = SupervisorFixture(
            route_kind="frontier_endpoint",
            phase="stalled_route_reorientation",
        )
        self.assertTrue(self.continue_turn(fixture))

    def test_other_route_kinds_are_not_continued(self):
        fixture = SupervisorFixture(route_kind="local_egress")
        self.assertFalse(self.continue_turn(fixture))

    def test_other_turn_phases_are_not_continued(self):
        fixture = SupervisorFixture(phase="portal_authentication_turn")
        self.assertFalse(self.continue_turn(fixture))

    def test_a_different_tangent_is_not_continued(self):
        fixture = SupervisorFixture(refreshed_yaw=1.20 + 0.50)
        self.assertFalse(self.continue_turn(fixture))

    def test_the_handoff_count_is_bounded(self):
        fixture = SupervisorFixture()
        fixture.stalled_route_turn_handoff_count = 8
        self.assertFalse(self.continue_turn(fixture))


class ParameterContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.params = (
            SCRIPTS / "teb_goal_bridge_parameters.py").read_text(encoding="utf-8")
        cls.runtime = (
            SCRIPTS / "teb_goal_bridge_teb_runtime.py").read_text(encoding="utf-8")
        cls.lifecycle = (
            SCRIPTS / "teb_turn_supervisor_turn_lifecycle.py"
        ).read_text(encoding="utf-8")

    def test_the_yaw_progress_threshold_is_a_parameter(self):
        self.assertIn("progress_yaw_epsilon", self.params)
        self.assertIn("progress_yaw_epsilon", self.runtime)

    def test_portal_transition_is_covered_by_the_continuation(self):
        self.assertIn('"portal_transition"', self.lifecycle)
        self.assertIn('"portal_entry_alignment"', self.lifecycle)


if __name__ == "__main__":
    unittest.main(verbosity=2)
