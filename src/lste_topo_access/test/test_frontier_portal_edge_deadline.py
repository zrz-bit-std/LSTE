#!/usr/bin/env python3
"""Regression tests for the bounded place-to-place portal transaction."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_route_lease import route_lease_failure_decision  # noqa: E402

WATCHDOG_SOURCE = SCRIPTS / "global_frontier_execution_watchdogs.py"
RESOLUTION_SOURCE = SCRIPTS / "global_frontier_execution_resolution.py"
OBSERVATION_SOURCE = SCRIPTS / "global_frontier_execution_observation.py"


def load_method(source, class_name, method_name, namespace):
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    owner = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    method = next(
        node for node in owner.body
        if isinstance(node, ast.FunctionDef) and node.name == method_name
    )
    exec(
        compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"),
        namespace,
    )
    return namespace[method_name]


class WatchdogState:
    def __init__(self, **fields):
        self.__dict__.update(fields)


EVALUATE = load_method(
    WATCHDOG_SOURCE,
    "GlobalFrontierExecutionWatchdogMixin",
    "evaluate_active_route_watchdogs",
    {
        "ActiveRouteWatchdogState": WatchdogState,
        "route_lease_failure_decision": route_lease_failure_decision,
        "rospy": SimpleNamespace(logwarn=lambda *_args: None),
    },
)
FAILURE_REASON = load_method(
    RESOLUTION_SOURCE,
    "GlobalFrontierExecutionResolutionMixin",
    "active_route_failure_reason",
    {},
)
GATE_APPROACH = load_method(
    OBSERVATION_SOURCE,
    "GlobalFrontierExecutionObservationMixin",
    "observe_portal_gate_approach",
    {"math": math},
)


class PortalWatchdogExplorer:
    evaluate_active_route_watchdogs = EVALUATE
    active_route_failure_reason = FAILURE_REASON

    def __init__(self, route_kind):
        self.active_route_kind = route_kind
        self.active_route_id = 4
        self.recovery_pending_route_id = 0
        self.post_turn_stall_timeout = 6.0
        self.active_turn_completed_launched = False
        self.active_progress_time = 45.5
        self.stall_timeout = 12.0
        self.active_since = 0.0
        self.active_timeout = 45.0
        self.active_portal_gate_approached_at = 0.0
        self.active_last_waypoint_map = (8.0, 1.0)
        self.waypoint_release_radius = 0.4

    @staticmethod
    def update_post_turn_watchdog(_x, _y, _now):
        return False, 0.0, None

    @staticmethod
    def active_region_is_stagnant(*_args):
        return False

    @staticmethod
    def active_waypoint_distance(_robot_map):
        return 3.0


class ControllerOwnedWatchdogExplorer(PortalWatchdogExplorer):
    def __init__(self):
        super().__init__("frontier_endpoint")
        self.controller_owned_route_failure = True
        self.last_route_stagnation_reported_route_id = 0
        self.events = []

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


class SupervisorTurningWatchdogExplorer(PortalWatchdogExplorer):
    """Expose the direct TEB endpoint-alignment execution phase."""

    def __init__(self, route_kind="frontier_endpoint"):
        super().__init__(route_kind)
        self.turn_supervisor_state = "TURNING"
        self.turn_supervisor_route_kind = route_kind
        self.turn_supervisor_turn_phase = "pre_route_alignment"


class PortalGateObserver:
    observe_portal_gate_approach = GATE_APPROACH

    def __init__(self):
        self.active_route_kind = "portal_transition"
        self.active_portal_gate_approached_at = None
        self.active_portal_gate_odom_xy = (5.0, 4.0)
        self.pose_odom = SimpleNamespace(x=3.6, y=4.0)
        self.endpoint_terminal_wait_radius = 0.8
        self.completed_radius = 1.25
        self.active_route_id = 9
        self.events = []

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


class FrontierPortalEdgeDeadlineTest(unittest.TestCase):
    def test_persistent_stall_waits_for_controller_terminal(self):
        explorer = ControllerOwnedWatchdogExplorer()

        state = explorer.evaluate_active_route_watchdogs(
            8.0,
            1.0,
            (4.0, 1.0),
            now=60.0,
            portal_transition=False,
            active_component=None,
            active_information=0.0,
        )

        self.assertTrue(state.stalled)
        self.assertFalse(state.expired)
        self.assertEqual(explorer.events[0][0], "route_stagnant_observed")

        explorer.recovery_pending_route_id = explorer.active_route_id
        terminal_state = explorer.evaluate_active_route_watchdogs(
            8.0,
            1.0,
            (4.0, 1.0),
            now=61.0,
            portal_transition=False,
            active_component=None,
            active_information=0.0,
        )

        self.assertTrue(terminal_state.expired)

    def test_portal_edge_cannot_be_renewed_by_progress_inside_a_room(self):
        explorer = PortalWatchdogExplorer("portal_transition")

        state = explorer.evaluate_active_route_watchdogs(
            8.0,
            1.0,
            (4.0, 1.0),
            now=46.0,
            portal_transition=True,
            active_component=None,
            active_information=0.0,
        )

        self.assertFalse(state.stalled)
        self.assertTrue(state.portal_edge_expired)
        self.assertTrue(state.expired)
        self.assertEqual(
            explorer.active_route_failure_reason(state),
            "portal_edge_deadline",
        )

    def test_ordinary_frontier_keeps_existing_progress_based_timeout(self):
        explorer = PortalWatchdogExplorer("frontier_endpoint")

        state = explorer.evaluate_active_route_watchdogs(
            8.0,
            1.0,
            (4.0, 1.0),
            now=46.0,
            portal_transition=False,
            active_component=None,
            active_information=0.0,
        )

        self.assertFalse(state.portal_edge_expired)
        self.assertFalse(state.expired)

    def test_direct_teb_endpoint_turn_counts_as_active_physical_progress(self):
        explorer = SupervisorTurningWatchdogExplorer()
        explorer.active_progress_time = 45.5
        explorer.active_since = 0.0
        explorer.stall_timeout = 2.0
        explorer.active_timeout = 10.0

        state = explorer.evaluate_active_route_watchdogs(
            8.0,
            1.0,
            (4.0, 1.0),
            now=60.0,
            portal_transition=False,
            active_component=None,
            active_information=0.0,
        )

        self.assertTrue(state.turn_phase_active)
        self.assertFalse(state.stalled)
        self.assertFalse(state.expired)

    def test_stale_turn_status_does_not_cover_a_local_egress_route(self):
        explorer = SupervisorTurningWatchdogExplorer("frontier_endpoint")
        explorer.active_route_kind = "local_egress"
        explorer.active_progress_time = 0.0
        explorer.active_since = 0.0
        explorer.stall_timeout = 2.0
        explorer.active_timeout = 10.0

        state = explorer.evaluate_active_route_watchdogs(
            8.0,
            1.0,
            (4.0, 1.0),
            now=5.0,
            portal_transition=False,
            active_component=None,
            active_information=0.0,
        )

        self.assertFalse(state.turn_phase_active)
        self.assertTrue(state.stalled)

    def test_corridor_approach_does_not_consume_the_portal_edge_lease(self):
        explorer = PortalWatchdogExplorer("portal_transition")
        explorer.active_portal_gate_approached_at = None

        state = explorer.evaluate_active_route_watchdogs(
            8.0,
            1.0,
            (4.0, 1.0),
            now=46.0,
            portal_transition=True,
            active_component=None,
            active_information=0.0,
        )

        self.assertFalse(state.stalled)
        self.assertFalse(state.portal_edge_expired)
        self.assertFalse(state.expired)

    def test_gate_proximity_starts_the_bounded_edge_lease(self):
        observer = PortalGateObserver()

        self.assertFalse(observer.observe_portal_gate_approach(now=12.0))
        self.assertIsNone(observer.active_portal_gate_approached_at)

        observer.pose_odom = SimpleNamespace(x=4.3, y=4.0)
        self.assertTrue(observer.observe_portal_gate_approach(now=13.5))
        self.assertEqual(observer.active_portal_gate_approached_at, 13.5)
        self.assertEqual(observer.events[-1][0], "portal_edge_started_at_gate")


if __name__ == "__main__":
    unittest.main()
