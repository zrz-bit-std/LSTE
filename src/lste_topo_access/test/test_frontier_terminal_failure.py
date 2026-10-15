#!/usr/bin/env python3
"""Regression tests for failed frontier-action lifecycle precedence."""

import ast
from pathlib import Path
import unittest
from types import SimpleNamespace


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
EXECUTION_SOURCE = SCRIPTS_DIR / "global_frontier_execution_resolution.py"


def load_terminal_resolver():
    tree = ast.parse(
        EXECUTION_SOURCE.read_text(encoding="utf-8"),
        filename=str(EXECUTION_SOURCE),
    )
    explorer = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierExecutionResolutionMixin"
    )
    resolver = next(
        node for node in explorer.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "resolve_active_route_terminal_or_failure"
    )
    namespace = {"rospy": SimpleNamespace(loginfo_throttle=lambda *_args: None)}
    exec(
        compile(
            ast.Module(body=[resolver], type_ignores=[]),
            str(EXECUTION_SOURCE),
            "exec",
        ),
        namespace,
    )
    return namespace["resolve_active_route_terminal_or_failure"]


def load_failed_route_deactivator():
    tree = ast.parse(
        EXECUTION_SOURCE.read_text(encoding="utf-8"),
        filename=str(EXECUTION_SOURCE),
    )
    explorer = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierExecutionResolutionMixin"
    )
    method = next(
        node for node in explorer.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "deactivate_failed_active_frontier"
    )
    namespace = {
        "rospy": SimpleNamespace(logwarn=lambda *_args: None),
    }
    exec(
        compile(
            ast.Module(body=[method], type_ignores=[]),
            str(EXECUTION_SOURCE),
            "exec",
        ),
        namespace,
    )
    return namespace["deactivate_failed_active_frontier"]


def load_observation_standoff_contract():
    tree = ast.parse(
        EXECUTION_SOURCE.read_text(encoding="utf-8"),
        filename=str(EXECUTION_SOURCE),
    )
    explorer = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierExecutionResolutionMixin"
    )
    method = next(
        node for node in explorer.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "observation_standoff_reached"
    )
    namespace = {}
    exec(
        compile(
            ast.Module(body=[method], type_ignores=[]),
            str(EXECUTION_SOURCE),
            "exec",
        ),
        namespace,
    )
    return namespace["observation_standoff_reached"]


def load_observation_standoff_completion():
    tree = ast.parse(
        EXECUTION_SOURCE.read_text(encoding="utf-8"),
        filename=str(EXECUTION_SOURCE),
    )
    explorer = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierExecutionResolutionMixin"
    )
    method = next(
        node for node in explorer.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "complete_observation_standoff"
    )
    namespace = {}
    exec(
        compile(
            ast.Module(body=[method], type_ignores=[]),
            str(EXECUTION_SOURCE),
            "exec",
        ),
        namespace,
    )
    return namespace["complete_observation_standoff"]


class TerminalFailureExplorer:
    resolve_active_route_terminal_or_failure = load_terminal_resolver()

    def __init__(self):
        self.endpoint_terminal_wait_radius = 0.85
        self.mission_endpoint_only = True
        self.persistent_execution = False
        self.active_terminal_received = False
        self.active_frontier_region_id = 7
        self.active_route_id = 1
        self.deactivations = []

    @staticmethod
    def active_route_failure_reason(watchdog):
        if watchdog.recovery_pending:
            return "move_base_aborted"
        if watchdog.region_stagnant:
            return "region_information_stagnant"
        if watchdog.post_turn_stalled:
            return "post_turn_no_progress"
        return "stall"

    def deactivate_failed_active_frontier(self, *args):
        self.deactivations.append(args)

    def mark_frontier_observed(self, *_args, **_kwargs):
        raise AssertionError("a failed action must not mark the endpoint observed")

    def release_active_frontier(self):
        raise AssertionError("a failed action must not take the success path")


class FailedRouteCacheExplorer:
    deactivate_failed_active_frontier = load_failed_route_deactivator()

    def __init__(self):
        self.active_route_kind = "frontier_endpoint"
        self.active_local_egress_resumes_portal = False
        self.pending_portal_retry = None
        self.active_frontier_region_id = 7
        self.active_since = 1.0
        self.rejected_timeout = 180.0
        self.rejected_frontiers = []
        self.prefetched_frontier = (1, 2, 3.0, 4.0)
        self.cleared = 0

    def prepare_local_egress(self, *_args, **_kwargs):
        return False

    def record_failed_frontier_region(self, *_args, **_kwargs):
        return None

    def publish_failed_route_invalidation(self, *_args, **_kwargs):
        pass

    def release_active_frontier(self):
        pass

    def clear_prefetched_frontier(self):
        self.cleared += 1
        self.prefetched_frontier = None


class ObservationStandoffExplorer:
    observation_standoff_reached = load_observation_standoff_contract()

    def __init__(self):
        self.endpoint_terminal_wait_radius = 1.0
        self.active_route_kind = "frontier_endpoint"
        self.active_mission_route_kind = "frontier_endpoint"
        self.active_place_hops = 0
        self.active_work_item_id = 2
        self.active_portal_probe_id = None


class StandoffResolutionExplorer:
    resolve_active_route_terminal_or_failure = load_terminal_resolver()
    complete_observation_standoff = load_observation_standoff_completion()
    observation_standoff_reached = load_observation_standoff_contract()

    def __init__(self):
        self.endpoint_terminal_wait_radius = 1.0
        self.mission_endpoint_only = True
        self.persistent_execution = False
        self.active_route_kind = "frontier_endpoint"
        self.active_mission_route_kind = "frontier_endpoint"
        self.active_place_hops = 0
        self.active_work_item_id = 2
        self.active_portal_probe_id = None
        self.active_route_id = 6
        self.active_frontier_region_id = 1
        self.active_last_progress_signal = "map_progress_without_physical_motion"
        self.place_departure = SimpleNamespace(active=False)
        self.status = []
        self.completed = False
        self.released = False

    def mark_frontier_observed(self, *_args, **_kwargs):
        return {"id": 1}

    def remember_completed_observation_source(self, _region):
        self.completed = True

    def publish_status(self, event, **fields):
        self.status.append((event, fields))

    def release_active_frontier(self, **_kwargs):
        self.released = True


class FrontierTerminalFailureTest(unittest.TestCase):
    def test_local_work_standoff_is_a_semantic_terminal(self):
        explorer = ObservationStandoffExplorer()
        watchdog = SimpleNamespace(
            recovery_pending=False,
            turn_phase_active=False,
            stalled=True,
            post_turn_stalled=False,
            waypoint_distance=0.72,
        )

        self.assertTrue(
            explorer.observation_standoff_reached(
                0.658, 0.80, watchdog, portal_transition=False,
            )
        )

    def test_standoff_contract_never_owns_portal_or_unstarted_routes(self):
        explorer = ObservationStandoffExplorer()
        watchdog = SimpleNamespace(
            recovery_pending=False,
            turn_phase_active=False,
            stalled=True,
            post_turn_stalled=False,
            waypoint_distance=0.72,
        )

        explorer.active_portal_probe_id = 7
        self.assertFalse(
            explorer.observation_standoff_reached(
                0.658, 0.80, watchdog, portal_transition=False,
            )
        )
        explorer.active_portal_probe_id = None
        self.assertFalse(
            explorer.observation_standoff_reached(
                0.658, 0.80, watchdog, portal_transition=True,
            )
        )
        explorer.active_work_item_id = None
        self.assertFalse(
            explorer.observation_standoff_reached(
                0.658, 0.80, watchdog, portal_transition=False,
            )
        )

    def test_standoff_contract_does_not_mask_active_turn_or_recovery(self):
        explorer = ObservationStandoffExplorer()
        for field in ("turn_phase_active", "recovery_pending"):
            watchdog = SimpleNamespace(
                recovery_pending=False,
                turn_phase_active=False,
                stalled=True,
                post_turn_stalled=False,
                waypoint_distance=0.72,
            )
            setattr(watchdog, field, True)
            self.assertFalse(
                explorer.observation_standoff_reached(
                    0.658, 0.80, watchdog, portal_transition=False,
                )
            )

    def test_stalled_local_work_completes_without_local_egress(self):
        explorer = StandoffResolutionExplorer()
        watchdog = SimpleNamespace(
            recovery_pending=False,
            turn_phase_active=False,
            stalled=True,
            post_turn_stalled=False,
            expired=False,
            waypoint_distance=0.72,
        )

        result = explorer.resolve_active_route_terminal_or_failure(
            row=24,
            col=31,
            x=2.65,
            y=3.05,
            distance=0.658,
            waypoint_reached=False,
            route_steps={(24, 31): 8},
            resolution=0.10,
            now=40.0,
            portal_transition=False,
            active_component={"label": 1},
            route_distance=0.80,
            watchdog=watchdog,
        )

        self.assertIsNone(result)
        self.assertTrue(explorer.completed)
        self.assertTrue(explorer.released)
        self.assertEqual(explorer.status[-1][0], "route_invalidated")
        self.assertEqual(
            explorer.status[-1][1]["reason"],
            "frontier_observed_at_standoff",
        )
        self.assertEqual(
            explorer.status[-1][1]["observation_contract"],
            "local_work_standoff",
        )
    def test_failed_terminal_beats_endpoint_proximity(self):
        explorer = TerminalFailureExplorer()
        watchdog = SimpleNamespace(recovery_pending=True)

        result = explorer.resolve_active_route_terminal_or_failure(
            row=4,
            col=5,
            x=18.45,
            y=12.45,
            distance=0.80,
            waypoint_reached=True,
            route_steps={(4, 5): 7},
            resolution=0.10,
            now=10.0,
            portal_transition=False,
            active_component={"label": 1},
            route_distance=0.90,
            watchdog=watchdog,
        )

        self.assertIsNone(result)
        self.assertEqual(len(explorer.deactivations), 1)
        self.assertEqual(explorer.deactivations[0][7], "move_base_aborted")

    def test_stagnant_place_waits_for_the_matching_action_terminal(self):
        explorer = TerminalFailureExplorer()
        watchdog = SimpleNamespace(
            recovery_pending=False,
            region_stagnant=True,
            post_turn_stalled=False,
            stalled=False,
            expired=False,
        )

        result = explorer.resolve_active_route_terminal_or_failure(
            row=4,
            col=5,
            x=18.45,
            y=12.45,
            distance=0.20,
            waypoint_reached=True,
            route_steps={(4, 5): 7},
            resolution=0.10,
            now=10.0,
            portal_transition=False,
            active_component={"label": 1},
            route_distance=0.20,
            watchdog=watchdog,
        )

        self.assertEqual(result[:4], (4, 5, 18.45, 12.45))
        self.assertAlmostEqual(result[4], 0.7)
        self.assertEqual(result[5], 0.0)
        self.assertEqual(explorer.deactivations, [])

    def test_failed_route_invalidates_prefetched_successor(self):
        explorer = FailedRouteCacheExplorer()

        explorer.deactivate_failed_active_frontier(
            3.0,
            4.0,
            0.4,
            10.0,
            False,
            {"label": 1},
            0.5,
            "stall",
            SimpleNamespace(
                recovery_pending=False,
                post_turn_stalled=False,
                post_turn_goal_matches=False,
                post_turn_elapsed=0.0,
                post_turn_translation=None,
            ),
        )

        self.assertEqual(explorer.cleared, 1)
        self.assertIsNone(explorer.prefetched_frontier)


if __name__ == "__main__":
    unittest.main()
