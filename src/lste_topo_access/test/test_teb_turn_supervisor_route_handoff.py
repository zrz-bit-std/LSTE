#!/usr/bin/env python3
"""Regression for preserving bounded stalled-route turns over Place refreshes."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from teb_turn_supervisor_contract import STATE_TURNING  # noqa: E402
from teb_turn_supervisor_routes import TebTurnSupervisorRoutesMixin  # noqa: E402
from teb_turn_supervisor_turn_lifecycle import (  # noqa: E402
    TebTurnSupervisorTurnLifecycleMixin,
)


def pose_stamped(x, y, yaw):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="odom", stamp=0.0),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y),
            orientation=SimpleNamespace(
                x=0.0, y=0.0, z=math.sin(yaw/2.0), w=math.cos(yaw/2.0),
            ),
        ),
    )


class HandoffFixture(TebTurnSupervisorRoutesMixin, TebTurnSupervisorTurnLifecycleMixin):
    def __init__(self):
        self.state = STATE_TURNING
        self.turn_phase = "stalled_route_reorientation"
        self.turn_action_identity = (31, 12)
        self.active_action_identity = (32, 13)
        self.active_action = True
        self.active_action_source = "global_slam_frontier"
        self.active_action_route_kind = "frontier_endpoint"
        self.active_action_goal_context = {
            "goal_role": "place_observation",
            "owner_place_id": 3,
            "source_place_id": 3,
            "portal_crossing_certified": False,
        }
        self.active_action_goal = pose_stamped(0.4, -1.0, 1.62)
        self.turn_target_yaw = 1.55
        self.turn_goal_xy = (0.0, -1.0)
        self.turn_key = (0.0, -1.0, 1.55)
        self.stalled_route_reorientation_attempted_identity = self.turn_action_identity
        self.stalled_route_turn_handoff_count = 0
        self.yaw_goal_tolerance = 0.35
        self.pose_frame = "odom"
        self.events = []

    def publish_status_locked(self, event, **fields):
        self.events.append((event, fields))


class TebTurnSupervisorRouteHandoffTest(unittest.TestCase):
    def test_same_place_nearby_endpoint_rebinds_turn_without_resetting_budget(self):
        fixture = HandoffFixture()
        old_identity = fixture.turn_action_identity
        fixture.turn_started_wall = 100.0
        fixture.turn_last_odom_progress_wall = 101.0
        fixture.turn_abs_rotation = 0.4
        self.assertTrue(fixture._continue_stalled_route_turn_for_handoff_locked(
            fixture.active_action_identity,
            dict(fixture.active_action_goal_context),
        ))
        self.assertEqual(fixture.turn_action_identity, fixture.active_action_identity)
        self.assertNotEqual(fixture.turn_action_identity, old_identity)
        self.assertEqual(fixture.turn_goal_xy, (0.4, -1.0))
        self.assertEqual(fixture.turn_started_wall, 100.0)
        self.assertEqual(fixture.turn_last_odom_progress_wall, 101.0)
        self.assertEqual(fixture.turn_abs_rotation, 0.4)
        self.assertEqual(fixture.stalled_route_turn_handoff_count, 1)
        self.assertEqual(
            fixture.stalled_route_reorientation_attempted_identity,
            fixture.active_action_identity,
        )
        self.assertEqual(fixture.events[-1][0], "stalled_route_turn_handoff_continued")

    def test_handoff_rejects_place_change_wrong_heading_and_exhausted_cap(self):
        fixture = HandoffFixture()
        changed_place = dict(fixture.active_action_goal_context, owner_place_id=4)
        self.assertFalse(fixture._continue_stalled_route_turn_for_handoff_locked(
            fixture.active_action_identity, changed_place,
        ))
        wrong_heading = dict(fixture.active_action_goal_context)
        fixture.active_action_goal = pose_stamped(0.4, -1.0, 2.4)
        self.assertFalse(fixture._continue_stalled_route_turn_for_handoff_locked(
            fixture.active_action_identity, wrong_heading,
        ))
        fixture.active_action_goal = pose_stamped(0.4, -1.0, 1.62)
        fixture.stalled_route_turn_handoff_count = 8
        self.assertFalse(fixture._continue_stalled_route_turn_for_handoff_locked(
            fixture.active_action_identity,
            dict(fixture.active_action_goal_context),
        ))


if __name__ == "__main__":
    unittest.main()
