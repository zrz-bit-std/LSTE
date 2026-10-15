"""Regression tests for the one-egress, one-retry portal edge contract."""

import ast
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
RECOVERY_SOURCE = SCRIPTS / "global_frontier_portal_recovery.py"
RESOLUTION_SOURCE = SCRIPTS / "global_frontier_execution_resolution.py"
ACTIVATION_SOURCE = SCRIPTS / "global_frontier_activation.py"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_models import PortalTransitionRetry


def load_methods(source, class_name, names, namespace):
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    owner = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    methods = [
        node for node in owner.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    exec(
        compile(ast.Module(body=methods, type_ignores=[]), str(source), "exec"),
        namespace,
    )
    return namespace


RECOVERY_METHODS = load_methods(
    RECOVERY_SOURCE,
    "GlobalFrontierPortalRecoveryMixin",
    {
        "prepare_portal_transition_retry",
        "discard_pending_portal_retry",
        "select_pending_portal_retry",
    },
    {"PortalTransitionRetry": PortalTransitionRetry},
)
FAILURE_METHODS = load_methods(
    RESOLUTION_SOURCE,
    "GlobalFrontierExecutionResolutionMixin",
    {"deactivate_failed_active_frontier"},
    {"rospy": SimpleNamespace(logwarn=lambda *_args, **_kwargs: None)},
)
ACTIVATION_METHODS = load_methods(
    ACTIVATION_SOURCE,
    "GlobalFrontierActivationMixin",
    {"initialize_active_frontier_route"},
    {"math": math, "copy_component_evidence": lambda value: value},
)


class PortalRecoveryExplorer:
    prepare_portal_transition_retry = (
        RECOVERY_METHODS["prepare_portal_transition_retry"]
    )
    discard_pending_portal_retry = RECOVERY_METHODS["discard_pending_portal_retry"]
    select_pending_portal_retry = RECOVERY_METHODS["select_pending_portal_retry"]

    def __init__(self):
        self.active_route_kind = "portal_transition"
        self.active_frontier = (10, 11, 8.0, 4.0)
        self.active_route_id = 19
        self.active_portal_retry = False
        self.active_portal_gate_xy = (7.5, 4.0)
        self.pending_portal_retry = None
        self.place_departure = SimpleNamespace(region_id=7)
        self.events = []

    def publish_status(self, event, **fields):
        self.events.append((event, fields))

    @staticmethod
    def nearest_reachable_cell(_message, _steps, _x, _y):
        return 12, 13

    @staticmethod
    def candidate_costmap_distance(_validation, _x, _y):
        return 2.4

    @staticmethod
    def navfn_goal_reachable(_robot_map, _goal, _frame):
        return True

    @staticmethod
    def component_at_map_position(*_args):
        return {"epoch": 11, "label": 5, "cells": 80}


class PortalFailureExplorer:
    deactivate_failed_active_frontier = (
        FAILURE_METHODS["deactivate_failed_active_frontier"]
    )

    def __init__(self):
        self.rejected_frontiers = []
        self.active_route_kind = "portal_transition"
        self.active_frontier_region_id = 7
        self.active_local_egress_resumes_portal = False
        self.active_portal_retry = False
        self.active_since = 3.0
        self.rejected_timeout = 180.0
        self.prefetched_frontier = None
        self.prefetched_goal_map = (1.0, 1.0)
        self.prepared = []
        self.published = []
        self.released = False

    def prepare_portal_transition_retry(self, reason):
        self.prepared.append(("portal", reason))
        return object()

    def prepare_local_egress(self, reason, source_region_id):
        self.prepared.append(("egress", reason, source_region_id))
        return True

    def discard_pending_portal_retry(self, reason):
        self.prepared.append(("discard", reason))

    @staticmethod
    def record_failed_frontier_region(*_args):
        return None

    def clear_prefetched_frontier(self):
        self.prepared.append(("clear_prefetch",))

    def publish_failed_route_invalidation(self, *args):
        self.published.append(args)

    def release_active_frontier(self):
        self.released = True


class ActivationExplorer:
    initialize_active_frontier_route = (
        ACTIVATION_METHODS["initialize_active_frontier_route"]
    )

    def __init__(self):
        self.frontier_exhausted = True
        self.active_frontier = None
        self.active_frontier_component = None
        self.active_frontier_region_id = None
        self.active_local_egress_resumes_portal = False
        self.local_egress_place_lease = SimpleNamespace(activate=lambda: False)
        self.active_observation_session_started_at = 1.0
        self.place_departure = SimpleNamespace(active=True)
        self.active_route_id = 3
        self.recovery_pending_route_id = 99
        self.recovery_pending_behavior = "old"
        self.recovery_pending_reason = "old"
        self.active_since = 0.0
        self.active_best_distance = None
        self.active_best_goal_distance = None
        self.active_best_path_distance = None
        self.active_progress_time = 0.0
        self.active_last_progress_signal = "old"
        self.active_last_robot_xy = None
        self.pose_odom = SimpleNamespace(x=1.0, y=2.0)
        self.active_best_detour_odom_distance = 1.0
        self.active_unreachable_since = 1.0
        self.active_last_waypoint_map = (9.0, 9.0)
        self.active_last_waypoint_yaw = 0.1
        self.active_route_kind = "frontier_endpoint"
        self.active_portal_retry = True
        self.active_terminal_received = True
        self.turn_connector_released = False
        self.last_status_command_map = (9.0, 9.0)
        self.last_status_command_yaw = 0.1
        self.last_status_mission_map = (9.0, 9.0)
        self.history_begins = []

    def _clear_active_place_departure(self):
        raise AssertionError("active departure should be retained")

    def _reset_active_odom_coverage(self):
        pass

    def begin_active_route_history(self, robot_map):
        self.history_begins.append(robot_map)


class PortalTransitionRecoveryTest(unittest.TestCase):
    def snapshot(self):
        return SimpleNamespace(
            message=SimpleNamespace(
                header=SimpleNamespace(frame_id="map"),
                info=SimpleNamespace(resolution=0.1),
            ),
            robot_map=(3.0, 4.0),
            map_context=SimpleNamespace(components=object(), known_free=object()),
            route_graph=SimpleNamespace(
                route_steps=[[0]], validation=object(),
            ),
        )

    def test_first_portal_failure_reserves_exact_edge_for_one_retry(self):
        explorer = PortalRecoveryExplorer()

        retry = explorer.prepare_portal_transition_retry("stall")

        self.assertIsNotNone(retry)
        self.assertEqual(retry.failed_route_id, 19)
        self.assertEqual(retry.source_region_id, 7)
        self.assertEqual(retry.goal_xy, (8.0, 4.0))
        self.assertEqual(retry.portal_gate_xy, (7.5, 4.0))
        self.assertEqual(explorer.events[-1][0], "portal_transition_recovery_prepared")

    def test_retry_is_a_freshly_validated_portal_action(self):
        explorer = PortalRecoveryExplorer()
        explorer.prepare_portal_transition_retry("stall")

        candidate, mode, waiting = explorer.select_pending_portal_retry(
            self.snapshot()
        )

        self.assertFalse(waiting)
        self.assertEqual(mode, "portal_recovery_retry")
        self.assertEqual(candidate[0:4], (12, 13, 8.0, 4.0))
        self.assertEqual(candidate[9:11], (1, "portal_transition"))
        self.assertEqual(candidate[11], (7.5, 4.0))
        self.assertIsNone(explorer.pending_portal_retry)
        self.assertEqual(explorer.events[-1][0], "portal_transition_recovery_selected")

    def test_retried_portal_cannot_start_another_retry_loop(self):
        explorer = PortalRecoveryExplorer()
        explorer.active_portal_retry = True

        self.assertIsNone(explorer.prepare_portal_transition_retry("stall"))
        self.assertIsNone(explorer.pending_portal_retry)

    def test_portal_failure_queues_egress_instead_of_unrelated_goal_selection(self):
        explorer = PortalFailureExplorer()
        watchdog = SimpleNamespace(
            post_turn_goal_matches=False,
            post_turn_elapsed=0.0,
            post_turn_translation=None,
            recovery_pending=False,
        )

        explorer.deactivate_failed_active_frontier(
            x=8.0,
            y=4.0,
            distance=3.0,
            now=11.0,
            portal_transition=True,
            active_component=None,
            route_distance=4.2,
            reason="stall",
            watchdog=watchdog,
        )

        self.assertEqual(explorer.prepared[0], ("portal", "stall"))
        self.assertEqual(explorer.prepared[1], ("egress", "stall", 7))
        self.assertTrue(explorer.released)
        self.assertTrue(explorer.published[0][-1])

    def test_state_initializer_is_independent_of_selection_mode(self):
        explorer = ActivationExplorer()
        selection = SimpleNamespace(
            row=1,
            col=2,
            x=4.0,
            y=5.0,
            component=None,
            route_kind="portal_transition",
            path_distance=3.0,
        )

        explorer.initialize_active_frontier_route(
            selection, region=None, now=8.0, robot_map=(1.0, 2.0)
        )

        self.assertEqual(explorer.active_route_kind, "portal_transition")
        self.assertFalse(explorer.active_portal_retry)
        self.assertEqual(explorer.history_begins, [(1.0, 2.0)])


if __name__ == "__main__":
    unittest.main()
