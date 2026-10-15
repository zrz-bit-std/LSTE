#!/usr/bin/env python3
"""Regression for atomic heading acquisition on a recovery egress route."""

import math
from pathlib import Path
import time
from types import SimpleNamespace
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from teb_turn_supervisor_contract import (  # noqa: E402
    FRONTIER_SOURCE,
    STATE_PASS_THROUGH,
    STATE_TURNING,
)
from teb_turn_supervisor_routes import TebTurnSupervisorRoutesMixin  # noqa: E402
from teb_turn_supervisor_turn_lifecycle import (  # noqa: E402
    TebTurnSupervisorTurnLifecycleMixin,
)


def pose(x, y, yaw):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="odom", stamp=0.0),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y),
            orientation=SimpleNamespace(
                x=0.0,
                y=0.0,
                z=math.sin(yaw / 2.0),
                w=math.cos(yaw / 2.0),
            ),
        ),
    )


class LocalEgressFixture(
    TebTurnSupervisorRoutesMixin,
    TebTurnSupervisorTurnLifecycleMixin,
):
    def __init__(self):
        self.mode = "teb"
        self.active_mode = "teb"
        self.task_done = False
        self.navigation_hold = False
        self.active_action = True
        self.active_action_route_kind = "local_egress"
        self.active_action_source = FRONTIER_SOURCE
        self.active_action_goal = pose(0.0, -2.0, -math.pi / 2.0)
        self.active_action_source_goal = self.active_action_goal
        self.latest_intent_goal = (0.0, -2.0)
        self.latest_intent_source = FRONTIER_SOURCE
        self.latest_intent_priority = 0
        self.active_action_identity = self._active_action_key_locked()
        self.pose_frame = "odom"
        self.pose = SimpleNamespace(x=0.0, y=0.0, theta=0.0)
        self.tf_listener = None
        self.state = STATE_PASS_THROUGH
        self.pre_turn_checked_identity = None
        self.completed_turn_key = None
        self.turn_pending_identity = None
        self.turn_pending_since_wall = 0.0
        self.turn_start_confirm_duration = 0.0
        self.endpoint_alignment_enabled = False
        self.yaw_goal_tolerance = 0.35
        self.pre_route_turn_threshold = math.radians(75.0)
        self.turn_min_clearance = 0.32
        self.turn_scan_timeout = 0.5
        self.scan_minimum = 2.0
        self.scan_monotonic = time.monotonic()
        self.turn_count = 0
        self.turn_completed_count = 0
        self.turn_released_count = 0
        self.turn_settle_duration = 0.0
        self.turn_rotation_cap_factor = 1.6
        self.events = []

    def publish_status_locked(self, event, **fields):
        self.events.append((str(event), fields))


class TebTurnSupervisorLocalEgressTest(unittest.TestCase):
    def test_egress_heading_is_admitted_as_an_atomic_turn(self):
        fixture = LocalEgressFixture()

        self.assertFalse(fixture._activate_turn_locked())
        self.assertTrue(fixture._activate_turn_locked())
        self.assertEqual(fixture.state, STATE_TURNING)
        self.assertEqual(fixture.events[-1][0], "turn_started")
        self.assertEqual(
            fixture.events[-1][1]["turn_phase"],
            "local_egress_alignment",
        )
        self.assertAlmostEqual(fixture.turn_target_yaw, -math.pi / 2.0)


if __name__ == "__main__":
    unittest.main()
