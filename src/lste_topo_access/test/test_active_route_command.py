#!/usr/bin/env python3
"""Unit tests for the active-frontier command decomposition.

The ROS node remains intentionally thin around these predicates: deciding
whether a command stays frozen is pure route-lifecycle logic and must remain
testable without a ROS master.
"""

import ast
import math
from pathlib import Path
import sys
import unittest

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
SOURCE = SCRIPTS_DIR / "lste_global_frontier_node.py"
ACTIVATION_SOURCE = SCRIPTS_DIR / "global_frontier_activation.py"
RESOLUTION_SOURCE = SCRIPTS_DIR / "global_frontier_route_command_resolution.py"
GEOMETRY_SOURCE = SCRIPTS_DIR / "global_frontier_route_command_geometry.py"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from global_frontier_models import ActiveRouteCommand, SelectedFrontier


def load_hold_predicate():
    tree = ast.parse(
        RESOLUTION_SOURCE.read_text(encoding="utf-8"), filename=str(RESOLUTION_SOURCE)
    )
    command_mixin = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierRouteCommandResolutionMixin"
    )
    predicate = next(
        node for node in command_mixin.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "should_hold_active_waypoint"
    )
    namespace = {"math": math}
    exec(
        compile(
            ast.Module(body=[predicate], type_ignores=[]),
            str(RESOLUTION_SOURCE),
            "exec",
        ),
        namespace,
    )
    return namespace["should_hold_active_waypoint"]


def load_selected_frontier_parser():
    tree = ast.parse(
        ACTIVATION_SOURCE.read_text(encoding="utf-8"),
        filename=str(ACTIVATION_SOURCE),
    )
    activation = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierActivationMixin"
    )
    selected = [
        node for node in activation.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "selected_frontier_from_cell"
    ]
    namespace = {"SelectedFrontier": SelectedFrontier}
    exec(
        compile(
            ast.Module(body=selected, type_ignores=[]),
            str(ACTIVATION_SOURCE),
            "exec",
        ),
        namespace,
    )
    parser = namespace["selected_frontier_from_cell"]
    return parser.__func__ if isinstance(parser, staticmethod) else parser


def load_lifecycle_preparation():
    tree = ast.parse(
        ACTIVATION_SOURCE.read_text(encoding="utf-8"),
        filename=str(ACTIVATION_SOURCE),
    )
    activation = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierActivationMixin"
    )
    method = next(
        node for node in activation.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "prepare_selected_frontier_lifecycle"
    )
    namespace = {}
    exec(
        compile(
            ast.Module(body=[method], type_ignores=[]),
            str(ACTIVATION_SOURCE),
            "exec",
        ),
        namespace,
    )
    return namespace["prepare_selected_frontier_lifecycle"]


def load_command_cell_selector():
    tree = ast.parse(
        GEOMETRY_SOURCE.read_text(encoding="utf-8"), filename=str(GEOMETRY_SOURCE)
    )
    command_mixin = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierRouteCommandGeometryMixin"
    )
    selector = next(
        node for node in command_mixin.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "select_route_command_cell"
    )
    namespace = {"math": math}
    exec(
        compile(
            ast.Module(body=[selector], type_ignores=[]),
            str(GEOMETRY_SOURCE),
            "exec",
        ),
        namespace,
    )
    return namespace["select_route_command_cell"]


def load_command_resolver():
    tree = ast.parse(
        RESOLUTION_SOURCE.read_text(encoding="utf-8"), filename=str(RESOLUTION_SOURCE)
    )
    command_mixin = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierRouteCommandResolutionMixin"
    )
    resolver = next(
        node for node in command_mixin.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "resolve_active_route_command"
    )
    namespace = {
        "ActiveRouteCommand": ActiveRouteCommand,
        "math": math,
        "rospy": type("Ros", (), {"loginfo": staticmethod(lambda *_args: None)}),
    }
    exec(
        compile(
            ast.Module(body=[resolver], type_ignores=[]),
            str(RESOLUTION_SOURCE),
            "exec",
        ),
        namespace,
    )
    return namespace["resolve_active_route_command"]


class CommandLifecycle:
    should_hold_active_waypoint = load_hold_predicate()
    select_route_command_cell = load_command_cell_selector()

    def __init__(
        self,
        *,
        waypoint=(4.0, 1.0),
        route_kind="frontier_endpoint",
        released=True,
        endpoint_only=True,
        release_radius=0.4,
    ):
        self.active_last_waypoint_map = waypoint
        self.active_route_kind = route_kind
        self.turn_connector_released = released
        self.mission_endpoint_only = endpoint_only
        self.waypoint_release_radius = release_radius


class LocalEgressCommandLifecycle(CommandLifecycle):
    resolve_active_route_command = load_command_resolver()

    def __init__(self):
        super().__init__(waypoint=None, route_kind="local_egress")
        self.active_last_waypoint_yaw = None
        self.route_segment_distance = 2.0
        self.turn_execution_mode = "endpoint_action"

    @staticmethod
    def active_mission_endpoint(_message, active_cell):
        return float(active_cell[2]), float(active_cell[3])

    @staticmethod
    def select_route_command_cell(_message, _steps, active_cell):
        return (active_cell[0], active_cell[1]), "frontier_endpoint", 1.0

    @staticmethod
    def route_headings(*_args):
        return 0.0, 0.0

    @staticmethod
    def cell_xy(_message, _row, _col):
        return 4.0, 5.0

    @staticmethod
    def should_hold_active_waypoint(_robot_map):
        return False, float("inf"), False


class PortalTransitionCommandLifecycle(LocalEgressCommandLifecycle):
    """A portal uses endpoint geometry but must retain portal semantics."""

    def __init__(self):
        CommandLifecycle.__init__(
            self,
            waypoint=None,
            route_kind="portal_transition",
        )
        self.active_last_waypoint_yaw = None
        self.route_segment_distance = 2.0
        self.turn_execution_mode = "endpoint_action"


class LifecyclePreparation:
    prepare_selected_frontier_lifecycle = load_lifecycle_preparation()

    def __init__(self):
        self.place_graph_waiting_for_portal = True
        self.calls = []

    @staticmethod
    def route_crosses_place_boundary(place_hops):
        return place_hops is not None and int(place_hops) >= 1

    def prepare_place_departure(self, *_args):
        self.calls.append("structural")
        return None

    def prepare_observation_place_departure(self, *_args):
        self.calls.append("observation")
        return None

    def activate_frontier_region(self, *_args, **_kwargs):
        self.calls.append("activate")
        return {"id": 1}


class ActiveRouteCommandTest(unittest.TestCase):
    def test_endpoint_lease_stays_frozen_inside_release_radius(self):
        lifecycle = CommandLifecycle()

        should_hold, distance, turn_pending = lifecycle.should_hold_active_waypoint(
            (3.9, 1.0)
        )

        self.assertTrue(should_hold)
        self.assertLess(distance, lifecycle.waypoint_release_radius)
        self.assertFalse(turn_pending)

    def test_released_turn_connector_can_advance_at_release_radius(self):
        lifecycle = CommandLifecycle(
            route_kind="frontier_turn_connector",
            released=True,
        )

        should_hold, _distance, turn_pending = lifecycle.should_hold_active_waypoint(
            (3.9, 1.0)
        )

        self.assertFalse(should_hold)
        self.assertFalse(turn_pending)

    def test_pending_turn_connector_stays_frozen_even_when_close(self):
        lifecycle = CommandLifecycle(
            route_kind="frontier_turn_connector",
            released=False,
        )

        should_hold, _distance, turn_pending = lifecycle.should_hold_active_waypoint(
            (3.9, 1.0)
        )

        self.assertTrue(should_hold)
        self.assertTrue(turn_pending)

    def test_segmented_portal_stays_frozen_until_its_action_terminal(self):
        lifecycle = CommandLifecycle(
            route_kind="portal_transition",
            endpoint_only=False,
        )

        should_hold, _distance, turn_pending = lifecycle.should_hold_active_waypoint(
            (3.9, 1.0)
        )

        self.assertTrue(should_hold)
        self.assertFalse(turn_pending)

    def test_selected_frontier_parses_optional_route_metadata_by_position(self):
        parser = load_selected_frontier_parser()
        selection = parser((3, 4, 7.5, 8.5, 12.0, 9.0, 2.0, 4.0, {"label": 8}, 1))

        self.assertEqual((selection.row, selection.col), (3, 4))
        self.assertEqual(selection.route_kind, "frontier_endpoint")
        self.assertEqual(selection.place_hops, 1)
        self.assertEqual(selection.component, {"label": 8})

    def test_command_cell_selector_accepts_a_full_frontier_tuple(self):
        lifecycle = CommandLifecycle(endpoint_only=True)
        message = type("Message", (), {"info": type("Info", (), {"resolution": 0.1})})()
        steps = np.zeros((4, 5), dtype=np.int32)
        steps[2, 3] = 17

        cell, route_kind, remaining = lifecycle.select_route_command_cell(
            message,
            steps,
            (2, 3, 1.25, 2.75, 1.7, 42.0),
        )

        self.assertEqual(cell, (2, 3))
        self.assertEqual(route_kind, "frontier_endpoint")
        self.assertAlmostEqual(remaining, 1.7)

    def test_local_egress_route_kind_survives_command_resolution(self):
        lifecycle = LocalEgressCommandLifecycle()
        message = type("Message", (), {"info": type("Info", (), {"resolution": 0.1})})()
        steps = np.zeros((4, 5), dtype=np.int32)

        command = lifecycle.resolve_active_route_command(
            message,
            (1, 2, 4.0, 5.0),
            None,
            steps,
            (0, 0),
            (0.0, 0.0),
            0.0,
        )

        self.assertEqual(command.route_kind, "local_egress")
        self.assertEqual(lifecycle.active_route_kind, "local_egress")

    def test_portal_route_kind_survives_endpoint_command_resolution(self):
        lifecycle = PortalTransitionCommandLifecycle()
        message = type("Message", (), {"info": type("Info", (), {"resolution": 0.1})})()
        steps = np.zeros((4, 5), dtype=np.int32)

        command = lifecycle.resolve_active_route_command(
            message,
            (1, 2, 4.0, 5.0),
            None,
            steps,
            (0, 0),
            (0.0, 0.0),
            0.0,
        )

        self.assertEqual(command.route_kind, "portal_transition")
        self.assertEqual(lifecycle.active_route_kind, "portal_transition")

    def test_local_endpoint_expands_coverage_without_preparing_departure(self):
        lifecycle = LifecyclePreparation()
        local = SelectedFrontier(
            row=1,
            col=2,
            x=3.0,
            y=4.0,
            path_distance=2.0,
            information=10.0,
            structure=1.0,
            score=2.0,
            component={"label": 1},
            place_hops=0,
            route_kind="frontier_endpoint",
        )

        region, departed = lifecycle.prepare_selected_frontier_lifecycle(
            None, None, None, (0.0, 0.0), 0.0, local, "strict_clearance",
        )

        self.assertEqual(region, {"id": 1})
        self.assertIsNone(departed)
        self.assertEqual(lifecycle.calls, ["activate"])


if __name__ == "__main__":
    unittest.main()
