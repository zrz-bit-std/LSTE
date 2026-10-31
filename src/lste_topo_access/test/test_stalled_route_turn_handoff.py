"""Continuation contract for a stalled-route reorientation across a refresh.

Run ``20261031_172738`` spent 380 s spinning at one endpoint. The supervisor
started a ``stalled_route_reorientation`` and then released it about a second
later with ``bridge_action_identity_changed`` in 27 of 36 attempts.

The refresh was not a new dispatch: ``mission_id`` stayed ``portal_semantic_test``
and the endpoint was unchanged, but the same work item was published under two
goal-role labels within a second -- ``place_observation`` (owner_place_id=2)
and ``geometry_frontier`` (owner_place_id=None) -- while
``graph_transaction_id`` advanced every 1-2 s. The continuation predicate
required ``goal_role == "place_observation"`` and ``owner_place_id > 0`` on
both sides, so it never matched (``stalled_route_turn_handoff_continued`` = 0).

The predicate now compares mission identity and geometry, enforcing the place
ids only when both sides carry them.
"""

from pathlib import Path
from types import SimpleNamespace
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from teb_turn_supervisor_contract import STATE_TURNING  # noqa: E402
from teb_turn_supervisor_turn_lifecycle import (  # noqa: E402
    TebTurnSupervisorTurnLifecycleMixin,
)


def goal(x, y, yaw):
    return SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y),
            orientation=SimpleNamespace(z=yaw),
        )
    )


OBSERVED_PLACE = {
    "goal_role": "place_observation",
    "owner_place_id": 2,
    "source_place_id": 2,
    "mission_id": "portal_semantic_test",
    "portal_crossing_certified": False,
}

OBSERVED_FRONTIER = {
    "goal_role": "geometry_frontier",
    "owner_place_id": None,
    "source_place_id": None,
    "mission_id": "portal_semantic_test",
    "portal_crossing_certified": False,
}


class _Host(TebTurnSupervisorTurnLifecycleMixin):
    def __init__(self, context=None, goal_pose=None, target_yaw=0.5,
                 turn_xy=(1.0, 2.0), navfn_yaw="same"):
        self.state = STATE_TURNING
        self.turn_phase = "stalled_route_reorientation"
        self.stalled_route_turn_handoff_count = 0
        self.active_action = True
        self.active_action_source = "global_slam_frontier"
        self.active_action_route_kind = "frontier_endpoint"
        self.active_action_goal = goal_pose or goal(1.0, 2.0, target_yaw)
        self.active_action_goal_context = dict(context or {})
        self.turn_target_yaw = target_yaw
        self.turn_goal_xy = turn_xy
        self.yaw_goal_tolerance = 0.35
        self.events = []
        # "same" mirrors the tangent the reorientation was started from;
        # None reproduces a costmap that has no Navfn heading yet.
        self._navfn_yaw = target_yaw if navfn_yaw == "same" else navfn_yaw

    # --- helpers used by the predicate, isolated from ROS ---
    def _goal_yaw_in_pose_frame_locked(self, target):
        return float(target.pose.orientation.z)

    def _initial_navfn_yaw_in_pose_frame_locked(self):
        return self._navfn_yaw

    def _goal_xy(self, target):
        return (float(target.pose.position.x), float(target.pose.position.y))

    def _turn_key_for_goal_locked(self, target, yaw):
        return ("turn", round(yaw, 3))

    def publish_status_locked(self, event, **fields):
        self.events.append((event, fields))


class HandoffContinuationTest(unittest.TestCase):
    def test_observed_role_flip_continues_the_turn(self):
        host = _Host(OBSERVED_FRONTIER)
        self.assertTrue(host._continue_stalled_route_turn_for_handoff_locked(
            "new-identity", OBSERVED_PLACE))
        self.assertEqual(host.stalled_route_turn_handoff_count, 1)
        self.assertEqual(host.turn_action_identity, "new-identity")
        self.assertEqual(host.events[-1][0], "stalled_route_turn_handoff_continued")

    def test_handoff_compares_the_navfn_tangent_not_the_endpoint_yaw(self):
        # Reproduces the sealed runs: the endpoint's yaw is ~0 while the turn
        # target is the Navfn tangent at -1.56 rad.  Comparing the endpoint yaw
        # gave a 1.56 rad gap and refused every handoff.
        host = _Host(OBSERVED_FRONTIER,
                     goal_pose=goal(1.0, 2.0, 0.0), target_yaw=-1.56,
                     navfn_yaw=-1.56)
        self.assertTrue(host._continue_stalled_route_turn_for_handoff_locked(
            "new-identity", OBSERVED_PLACE))
        self.assertEqual(host.stalled_route_turn_handoff_count, 1)

    def test_endpoint_yaw_alone_no_longer_blocks_the_handoff(self):
        # Same as above but asserting the old reference really would fail:
        # the endpoint yaw differs from the tangent by more than the tolerance.
        host = _Host(OBSERVED_FRONTIER,
                     goal_pose=goal(1.0, 2.0, 0.0), target_yaw=-1.56,
                     navfn_yaw=-1.56)
        gap = abs(host._goal_yaw_in_pose_frame_locked(host.active_action_goal)
                  - host.turn_target_yaw)
        self.assertGreater(gap, host.yaw_goal_tolerance)

    def test_handoff_falls_back_to_goal_yaw_without_a_navfn_tangent(self):
        host = _Host(OBSERVED_FRONTIER, target_yaw=0.5, navfn_yaw=None)
        self.assertTrue(host._continue_stalled_route_turn_for_handoff_locked(
            "id2", OBSERVED_PLACE))

    def test_moved_navfn_tangent_is_still_refused(self):
        host = _Host(OBSERVED_FRONTIER, target_yaw=0.5, navfn_yaw=2.4)
        self.assertFalse(host._continue_stalled_route_turn_for_handoff_locked(
            "id2", OBSERVED_PLACE))

    def test_place_to_place_handoff_still_works(self):
        host = _Host(dict(OBSERVED_PLACE))
        self.assertTrue(host._continue_stalled_route_turn_for_handoff_locked(
            "id2", OBSERVED_PLACE))

    def test_different_mission_is_refused(self):
        changed = dict(OBSERVED_FRONTIER, mission_id="another_mission")
        host = _Host(changed)
        self.assertFalse(host._continue_stalled_route_turn_for_handoff_locked(
            "id2", OBSERVED_PLACE))

    def test_different_known_place_is_refused(self):
        changed = dict(OBSERVED_PLACE, owner_place_id=3)
        host = _Host(changed)
        self.assertFalse(host._continue_stalled_route_turn_for_handoff_locked(
            "id2", OBSERVED_PLACE))

    def test_different_known_source_place_is_refused(self):
        changed = dict(OBSERVED_PLACE, source_place_id=9)
        host = _Host(changed)
        self.assertFalse(host._continue_stalled_route_turn_for_handoff_locked(
            "id2", OBSERVED_PLACE))

    def test_endpoint_beyond_two_metres_is_refused(self):
        host = _Host(OBSERVED_FRONTIER, turn_xy=(9.0, 9.0))
        self.assertFalse(host._continue_stalled_route_turn_for_handoff_locked(
            "id2", OBSERVED_PLACE))

    def test_handoff_cap_is_enforced(self):
        host = _Host(OBSERVED_FRONTIER)
        host.stalled_route_turn_handoff_count = 8
        self.assertFalse(host._continue_stalled_route_turn_for_handoff_locked(
            "id2", OBSERVED_PLACE))

    def test_certified_portal_is_refused(self):
        changed = dict(OBSERVED_FRONTIER, portal_crossing_certified=True)
        host = _Host(changed)
        self.assertFalse(host._continue_stalled_route_turn_for_handoff_locked(
            "id2", OBSERVED_PLACE))

    def test_non_stalled_phase_is_refused(self):
        host = _Host(OBSERVED_FRONTIER)
        host.turn_phase = "authentication_turn"
        self.assertFalse(host._continue_stalled_route_turn_for_handoff_locked(
            "id2", OBSERVED_PLACE))

    def test_unknown_role_is_refused(self):
        changed = dict(OBSERVED_FRONTIER, goal_role="portal_transition")
        host = _Host(changed)
        self.assertFalse(host._continue_stalled_route_turn_for_handoff_locked(
            "id2", OBSERVED_PLACE))


if __name__ == "__main__":
    unittest.main(verbosity=2)
