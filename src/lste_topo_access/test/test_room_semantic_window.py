"""Room semantics come only from frames that still look into the room.

env04 ``20261113_035731`` node 5: the west doorway scored +0.18 with the label
``chair`` while the east doorway scored -0.69 with ``fire hydrant``.  The world
has the hydrant on the west side and the chairs (and the yellow cup) on the
east side, so the two rooms had claimed each other's furniture.

The capture window was ``turning`` + ``returning``.  The authentication U-turn
sweeps ~172 degrees and its direction follows the inside-frontier split, so the
first ~90 degrees are real room content, but past the door plane the camera
looks back down the corridor - which is where the opposite room was picked up.

These cases pin the window: keep the inward core and the chosen-side sweep,
drop the look-back arc and every ``returning`` frame, and do not sample the
approach before the robot reaches the gate.
"""
from pathlib import Path
import math
import sys
import unittest
from types import SimpleNamespace

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_portal_authentication import DoorwayAuthentication  # noqa: E402
from global_frontier_event_callbacks import (  # noqa: E402
    GlobalFrontierEventCallbacksMixin,
)

# env04 20261113_035731 node 5 doorways (physical/odom geometry).
WEST_GATE = (11.13, -11.58)
WEST_DEST = (9.35, -11.63)
EAST_GATE = (13.92, -11.57)
EAST_DEST = (15.24, -10.82)
JUNCTION = (12.41, -11.33)


def _auth(gate, destination, direction="", state="turning"):
    auth = DoorwayAuthentication(12, 5, gate, destination)
    auth.begin(JUNCTION)
    auth.state = state
    auth.turn_direction = direction
    auth.chosen_turn_side = (
        "counterclockwise" if direction == "counterclockwise"
        else "clockwise" if direction == "clockwise" else None
    )
    auth.target_yaw = 0.36
    return auth


class ObservesRoomTest(unittest.TestCase):
    """Angle window around the doorway's inward normal."""

    def test_inward_core_is_kept_on_both_sides(self):
        auth = _auth(WEST_GATE, WEST_DEST, "clockwise")
        self.assertTrue(auth.observes_room(math.radians(196.0)))   # driving in, delta +14
        self.assertTrue(auth.observes_room(math.radians(150.0)))   # delta -32

    def test_chosen_side_sweep_is_kept(self):
        auth = _auth(WEST_GATE, WEST_DEST, "clockwise")
        self.assertTrue(auth.observes_room(math.radians(100.0)))   # delta -82, room interior

    def test_look_back_arc_is_dropped(self):
        auth = _auth(WEST_GATE, WEST_DEST, "clockwise")
        # The 035731 west door swept down to 21 deg; from ~90 deg on it looked
        # straight down the corridor at the east room's chairs.
        self.assertFalse(auth.observes_room(math.radians(60.0)))
        self.assertFalse(auth.observes_room(math.radians(21.0)))

    def test_east_door_symmetric_case(self):
        auth = _auth(EAST_GATE, EAST_DEST, "counterclockwise")
        self.assertTrue(auth.observes_room(math.radians(-14.0)))   # delta -44
        self.assertTrue(auth.observes_room(math.radians(60.0)))    # delta +30
        self.assertTrue(auth.observes_room(math.radians(110.0)))   # delta +80
        self.assertFalse(auth.observes_room(math.radians(140.0)))
        self.assertFalse(auth.observes_room(math.radians(158.0)))

    def test_unselected_side_gets_only_the_core(self):
        auth = _auth(EAST_GATE, EAST_DEST, "counterclockwise")
        auth.state = "turning"
        # +80 is inside the arc for CCW; the mirrored -80 must not be.
        self.assertTrue(auth.observes_room(math.radians(110.0)))
        self.assertFalse(auth.observes_room(math.radians(-50.0)))

    def test_returning_is_never_sampled(self):
        auth = _auth(WEST_GATE, WEST_DEST, "clockwise", state="returning")
        self.assertFalse(auth.observes_room(math.radians(180.0)))

    def test_entering_uses_the_core_only(self):
        auth = _auth(WEST_GATE, WEST_DEST, state="entering")
        self.assertTrue(auth.observes_room(math.radians(180.0)))
        self.assertFalse(auth.observes_room(math.radians(90.0)))

    def test_missing_pose_keeps_only_pre_turn_states(self):
        auth = _auth(WEST_GATE, WEST_DEST, "clockwise", state="turning")
        self.assertFalse(auth.observes_room(None))
        auth.state = "entering"
        self.assertTrue(auth.observes_room(None))

    def test_thresholds_are_configurable(self):
        auth = _auth(WEST_GATE, WEST_DEST, "clockwise")
        self.assertFalse(auth.observes_room(math.radians(100.0), core_deg=30.0, arc_deg=60.0))
        self.assertTrue(auth.observes_room(math.radians(180.0), core_deg=30.0, arc_deg=60.0))


class ObservationContextTest(unittest.TestCase):
    """The ingress/apply context gates both capture points identically."""

    def _harness(self, yaw):
        host = GlobalFrontierEventCallbacksMixin()
        host.pose_odom = SimpleNamespace(x=10.9, y=-11.6, theta=math.radians(yaw))
        host.room_view_core_deg = 45.0
        host.room_view_arc_deg = 90.0
        host.room_view_min_depth = -0.3
        return host

    def test_look_into_the_room_is_attributed(self):
        host = self._harness(196.0)
        auth = _auth(WEST_GATE, WEST_DEST, "clockwise")
        context = host.portal_room_observation_context(auth)
        self.assertEqual(context["branch_id"], 12)
        self.assertEqual(context["node_id"], 5)

    def test_look_back_is_not_attributed(self):
        host = self._harness(40.0)
        auth = _auth(WEST_GATE, WEST_DEST, "clockwise")
        self.assertIsNone(host.portal_room_observation_context(auth))

    def test_returning_is_not_attributed(self):
        host = self._harness(180.0)
        auth = _auth(WEST_GATE, WEST_DEST, "clockwise", state="returning")
        self.assertIsNone(host.portal_room_observation_context(auth))

    def test_approach_behind_the_gate_plane_is_not_attributed(self):
        host = self._harness(180.0)
        auth = _auth(WEST_GATE, WEST_DEST, state="entering")
        host.pose_odom = SimpleNamespace(x=12.4, y=-11.5, theta=math.pi)
        self.assertIsNone(host.portal_room_observation_context(auth))

    def test_entry_past_the_gate_plane_is_attributed(self):
        host = self._harness(180.0)
        auth = _auth(WEST_GATE, WEST_DEST, state="entering")
        host.pose_odom = SimpleNamespace(x=10.9, y=-11.6, theta=math.pi)
        self.assertIsNotNone(host.portal_room_observation_context(auth))

    def test_legacy_fixture_without_the_gate_keeps_old_rule(self):
        host = self._harness(0.0)
        stub = SimpleNamespace(node_id=8, branch_id=31, state="turning")
        context = host.portal_room_observation_context(stub)
        self.assertEqual(context["branch_id"], 31)
        stub.state = "returning"
        self.assertIsNotNone(host.portal_room_observation_context(stub))
        stub.state = "entering"
        self.assertIsNone(host.portal_room_observation_context(stub))


class RoomLabelSwapReplayTest(unittest.TestCase):
    """Replay the 035731 sweep: keep the room half, drop the corridor half."""

    def _kept_range(self, auth, start_deg, end_deg, step=2.0):
        kept = []
        angle = start_deg
        while angle >= end_deg if start_deg > end_deg else angle <= end_deg:
            if auth.observes_room(math.radians(angle)):
                kept.append(round(angle, 1))
            angle += -step if start_deg > end_deg else step
        return kept

    def test_west_door_keeps_the_room_sweep_only(self):
        auth = _auth(WEST_GATE, WEST_DEST, "clockwise")
        kept = self._kept_range(auth, 196.0, 20.0)
        self.assertIn(196.0, kept)
        self.assertIn(150.0, kept)
        self.assertLessEqual(min(kept), 92.0)
        self.assertNotIn(60.0, kept)
        self.assertNotIn(20.0, kept)

    def test_east_door_keeps_the_room_sweep_only(self):
        auth = _auth(EAST_GATE, EAST_DEST, "counterclockwise")
        kept = self._kept_range(auth, -14.0, 158.0)
        self.assertIn(-14.0, kept)
        self.assertIn(110.0, kept)
        self.assertGreaterEqual(max(kept), 118.0)
        self.assertLess(max(kept), 125.0)
        self.assertNotIn(140.0, kept)


if __name__ == "__main__":
    unittest.main()
