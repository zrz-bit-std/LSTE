#!/usr/bin/env python3
"""Regression tests for action ownership across frontier observation states."""

import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"


def load_method(filename, class_name, method_name, namespace=None):
    source = SCRIPTS_DIR / filename
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    owner = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    method = next(
        node for node in owner.body
        if isinstance(node, ast.FunctionDef) and node.name == method_name
    )
    namespace = {} if namespace is None else namespace
    exec(
        compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"),
        namespace,
    )
    return namespace[method_name]


READ_ONLY_STAGNATION = load_method(
    "global_frontier_execution_watchdogs.py",
    "GlobalFrontierExecutionWatchdogMixin",
    "active_region_is_stagnant",
    {"rospy": SimpleNamespace(loginfo_throttle=lambda *_args: None)},
)
ACTIVE_ROUTE_UPDATE = load_method(
    "global_frontier_execution_active_route.py",
    "GlobalFrontierExecutionActiveRouteMixin",
    "update_connected_active_frontier",
    {"rospy": SimpleNamespace(loginfo_throttle=lambda *_args: None)},
)
TERMINAL_PLACE_CLOSURE = load_method(
    "global_frontier_terminal_observation.py",
    "GlobalFrontierTerminalObservationMixin",
    "close_stagnant_place_after_terminal",
    {"rospy": SimpleNamespace(get_time=lambda: 40.0, loginfo=lambda *_args: None)},
)


class ReadOnlyStagnationExplorer:
    active_region_is_stagnant = READ_ONLY_STAGNATION

    def __init__(self):
        self.target_region_claim_active = False
        self.active_observation_session_started_at = 10.0
        self.active_frontier_region_id = 8
        self.active_route_id = 42
        self.region_memory = SimpleNamespace(
            stagnant=lambda *_args, **_kwargs: {
                "id": 8,
                "last_gain": 12.0,
            },
            dormant=lambda *_args, **_kwargs: self.fail_if_called(),
        )

    @staticmethod
    def fail_if_called():
        raise AssertionError("watchdog may not change a place lifecycle")


class ActiveRouteOwnershipExplorer:
    update_connected_active_frontier = ACTIVE_ROUTE_UPDATE

    def __init__(self):
        self.endpoint_terminal_wait_radius = 0.85
        self.resolve_calls = []
        self.prefetch_calls = 0

    def observe_connected_active_route(self, *_args):
        return (
            SimpleNamespace(distance=0.50, component=None, route_distance=0.50),
            SimpleNamespace(
                region_stagnant=True,
                stalled=False,
                expired=False,
                waypoint_reached=True,
            ),
        )

    def prefetch_or_promote_connected_route(self, *_args):
        self.prefetch_calls += 1
        return None

    def resolve_active_route_terminal_or_failure(self, *args):
        self.resolve_calls.append(args)
        return (args[0], args[1], args[2], args[3], 0.0, 0.0)

    @staticmethod
    def release_if_active_frontier_observed(*_args):
        raise AssertionError("an active route must not be released before terminal")


class TerminalPlaceClosureExplorer:
    close_stagnant_place_after_terminal = TERMINAL_PLACE_CLOSURE

    def __init__(self):
        self.active_route_kind = "frontier_endpoint"
        self.target_region_claim_active = False
        self.active_observation_session_started_at = 10.0
        self.active_last_robot_xy = (4.0, 5.0)
        self.active_frontier_component = {"label": 3}
        self.active_frontier_region_id = 8
        self.active_route_id = 42
        self.status = []
        self.region = {
            "id": 8,
            "visits": 2,
            "completions": 0,
            "information": 14.0,
            "last_gain": 12.0,
        }
        self.region_memory = SimpleNamespace(
            stagnant=self.stagnant,
            dormant=self.dormant,
        )
        self.dormant_calls = 0

    def stagnant(self, *_args, **_kwargs):
        return self.region

    def dormant(self, region, now, reason):
        self.dormant_calls += 1
        self.assertEqual(region, self.region)
        self.assertEqual(now, 40.0)
        self.assertEqual(reason, "no_information_progress")
        return True

    def publish_status(self, event, **fields):
        self.status.append((event, fields))

    def assertEqual(self, left, right):
        if left != right:
            raise AssertionError("%r != %r" % (left, right))


class FrontierActionOwnershipTest(unittest.TestCase):
    def test_watchdog_does_not_make_a_place_dormant(self):
        explorer = ReadOnlyStagnationExplorer()

        self.assertTrue(
            explorer.active_region_is_stagnant(
                4.0, 5.0, (4.0, 5.0), 40.0, False, False, False, None, 14.0,
            )
        )

    def test_near_endpoint_with_an_active_action_cannot_publish_a_successor(self):
        explorer = ActiveRouteOwnershipExplorer()
        snapshot = SimpleNamespace(
            route_graph=SimpleNamespace(route_steps=[[0]], validation=None),
            map_context=SimpleNamespace(known_free=None, unknown=None),
            message=SimpleNamespace(info=SimpleNamespace(resolution=0.1)),
            now=40.0,
        )

        active, replacement = explorer.update_connected_active_frontier(
            snapshot, 2, 3, 4.0, 5.0, False,
        )

        self.assertEqual(active, (2, 3, 4.0, 5.0, 0.0, 0.0))
        self.assertIsNone(replacement)
        self.assertEqual(explorer.prefetch_calls, 1)
        self.assertEqual(len(explorer.resolve_calls), 1)

    def test_verified_terminal_is_the_place_closure_boundary(self):
        explorer = TerminalPlaceClosureExplorer()

        region = explorer.close_stagnant_place_after_terminal((2, 3, 4.0, 5.0))

        self.assertEqual(region, explorer.region)
        self.assertEqual(explorer.dormant_calls, 1)
        self.assertEqual(explorer.status[0][0], "frontier_region_dormant")


if __name__ == "__main__":
    unittest.main()
