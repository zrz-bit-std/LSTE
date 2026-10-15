#!/usr/bin/env python3
"""Regression for short endpoint routes with TEB reverse-entry samples."""

import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from teb_turn_supervisor_control import TebTurnSupervisorControlMixin  # noqa: E402
from teb_turn_supervisor_contract import FRONTIER_SOURCE, STATE_PASS_THROUGH  # noqa: E402
from teb_turn_supervisor_routes import TebTurnSupervisorRoutesMixin  # noqa: E402
from teb_turn_supervisor_turn_lifecycle import (  # noqa: E402
    TebTurnSupervisorTurnLifecycleMixin,
)


def pose_stamped(x, y, yaw=0.0):
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


def twist(linear, angular):
    return SimpleNamespace(
        linear=SimpleNamespace(x=linear),
        angular=SimpleNamespace(z=angular),
    )


class ShortReverseEntryFixture(
    TebTurnSupervisorControlMixin,
    TebTurnSupervisorRoutesMixin,
    TebTurnSupervisorTurnLifecycleMixin,
):
    def __init__(self):
        self.mode = "teb"
        self.active_mode = "teb"
        self.task_done = False
        self.navigation_hold = False
        self.active_action = True
        self.active_action_route_kind = "frontier_endpoint"
        self.active_action_source = FRONTIER_SOURCE
        self.active_action_goal = pose_stamped(0.5, 0.0)
        self.active_action_source_goal = self.active_action_goal
        self.latest_intent_goal = (0.5, 0.0)
        self.latest_intent_source = FRONTIER_SOURCE
        self.latest_intent_priority = 0
        self.active_action_identity = self._active_action_key_locked()
        self.pose_frame = "odom"
        self.pose = SimpleNamespace(x=0.0, y=0.0, theta=0.0)
        self.tf_listener = None
        self.latest_navfn_plan = SimpleNamespace(
            header=SimpleNamespace(frame_id="odom"),
            poses=[pose_stamped(0.0, 0.0), pose_stamped(0.0, 0.2)],
        )
        self.latest_trajectory_command = twist(-0.01, 0.05)
        self.latest_trajectory_command_wall = time.monotonic()
        self.trajectory_feedback_period_ema = None
        self.trajectory_feedback_timeout = 0.35
        self.trajectory_feedback_timeout_cap = 0.60
        self.trajectory_feedback_period_scale = 1.50
        self.stalled_route_reorientation_enabled = True
        self.stalled_route_reorientation_delay = 0.80
        self.stalled_route_reorientation_heading = math.radians(45.0)
        self.stalled_route_reorientation_linear = 0.03
        self.stalled_route_reorientation_angular = 0.15
        self.stalled_route_reorientation_min_distance = 1.00
        self.yaw_goal_tolerance = 0.35
        self.stalled_route_completed_identity = None
        self.stalled_route_candidate_identity = None
        self.stalled_route_candidate_since_wall = 0.0
        self.stalled_route_ready_identity = None
        self.events = []

    def publish_status_locked(self, event, **fields):
        self.events.append((event, fields))


class TebTurnSupervisorShortReverseEntryTest(unittest.TestCase):
    def test_reverse_entry_allows_one_short_route_reorientation(self):
        fixture = ShortReverseEntryFixture()

        self.assertIsNone(
            fixture._stalled_route_reorientation_locked(
                fixture.active_action_identity
            )
        )
        fixture.stalled_route_candidate_since_wall = time.monotonic() - 1.0

        alignment = fixture._stalled_route_reorientation_locked(
            fixture.active_action_identity
        )

        self.assertIsNotNone(alignment)
        self.assertAlmostEqual(alignment[0], math.pi / 2.0)
        self.assertEqual(
            fixture.events[-1][1]["reason"], "forward_only_reverse_entry"
        )

    def test_short_route_plain_zero_command_keeps_the_original_gate(self):
        fixture = ShortReverseEntryFixture()
        fixture.latest_trajectory_command = twist(0.0, 0.05)

        self.assertIsNone(
            fixture._stalled_route_reorientation_locked(
                fixture.active_action_identity
            )
        )
        self.assertEqual(fixture.events, [])


if __name__ == "__main__":
    unittest.main()
