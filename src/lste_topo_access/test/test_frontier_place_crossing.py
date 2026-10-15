#!/usr/bin/env python3
"""Regression tests for route-kind-independent place departure closure."""

import ast
import math
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
VALIDATION_SOURCE = SCRIPTS_DIR / "global_frontier_terminal_validation.py"
RECORDING_SOURCE = SCRIPTS_DIR / "global_frontier_terminal_recording.py"
REPLAN_SOURCE = SCRIPTS_DIR / "global_frontier_terminal_replan.py"
OBSERVATION_SOURCE = SCRIPTS_DIR / "global_frontier_observation_coverage.py"


def load_methods():
    sources = (
        (VALIDATION_SOURCE, "GlobalFrontierTerminalValidationMixin", {
            "matching_execution_terminal",
        }),
        (RECORDING_SOURCE, "GlobalFrontierTerminalRecordingMixin", {
            "record_execution_terminal",
        }),
        (REPLAN_SOURCE, "GlobalFrontierTerminalReplanMixin", {
            "on_execution_terminal",
            "_arm_terminal_drain_timer",
            "_process_execution_terminal_locked",
            "_drain_execution_terminals_locked",
            "on_terminal_drain_timer",
        }),
        (OBSERVATION_SOURCE, "GlobalFrontierObservationCoverageMixin", {
            "route_crosses_place_boundary",
        }),
    )
    selected = []
    for source, class_name, names in sources:
        tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
        owner = next(
            node for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == class_name
        )
        selected.extend(
            node for node in owner.body
            if isinstance(node, ast.FunctionDef) and node.name in names
        )
    namespace = {
        "math": math,
        "rospy": SimpleNamespace(
            loginfo_throttle=lambda *_args, **_kwargs: None,
            logwarn_throttle=lambda *_args, **_kwargs: None,
            Timer=lambda *_args, **_kwargs: object(),
            Duration=lambda value: value,
            is_shutdown=lambda: False,
        ),
    }
    exec(
        compile(
            ast.Module(body=selected, type_ignores=[]),
            str(REPLAN_SOURCE),
            "exec",
        ),
        namespace,
    )
    return namespace


METHODS = load_methods()


class CrossingExplorer:
    matching_execution_terminal = METHODS["matching_execution_terminal"]
    record_execution_terminal = METHODS["record_execution_terminal"]
    on_execution_terminal = METHODS["on_execution_terminal"]
    _arm_terminal_drain_timer = METHODS["_arm_terminal_drain_timer"]
    _process_execution_terminal_locked = METHODS["_process_execution_terminal_locked"]
    _drain_execution_terminals_locked = METHODS["_drain_execution_terminals_locked"]
    on_terminal_drain_timer = METHODS["on_terminal_drain_timer"]
    route_crosses_place_boundary = METHODS["route_crosses_place_boundary"]

    def __init__(self):
        self.planning_lock = threading.RLock()
        self.terminal_ingress_lock = threading.Lock()
        self.pending_execution_terminals = __import__(
            "collections"
        ).deque(maxlen=16)
        self.terminal_ingress_dropped = 0
        self.terminal_drain_timer = None
        self.task_done = False
        self.waypoint_release_radius = 1.20
        self.active_last_waypoint_map = (13.0, 10.0)
        self.map_msg = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
        )
        self.active_frontier = (10, 10, 13.0, 10.0)
        self.active_route_id = 1
        self.active_route_kind = "frontier_endpoint"
        self.active_place_hops = 1
        self.active_frontier_component = {"label": 1}
        self.active_frontier_region_id = 5
        self.active_portal_gate_xy = (12.0, 10.0)
        self.active_portal_gate_odom_xy = (2.0, -1.0)
        self.active_portal_destination_odom_xy = (3.0, -1.0)
        self.pose_odom = SimpleNamespace(x=3.0, y=-1.0)
        self.place_departure = SimpleNamespace(
            active=True,
            region_id=5,
            basis="structural_transition",
            anchor_map=(12.0, 10.0),
        )
        self.marked = []
        self.observation_sources = []
        self.commits = 0
        self.status = []
        self.portal_entries = []
        self.region_memory = SimpleNamespace(
            record_entry_portal=self.record_entry_portal,
        )

    def record_entry_portal(self, *args, **kwargs):
        self.portal_entries.append((args, kwargs))
        return {"id": args[0]}

    def mark_frontier_observed(self, *args, **kwargs):
        self.marked.append((args, kwargs))

    def remember_completed_observation_source(self, region):
        self.observation_sources.append(region)

    @staticmethod
    def close_stagnant_place_after_terminal(_completed):
        # Place closure is exercised independently; this test isolates the
        # departure transaction that follows a normal successful terminal.
        return None

    def commit_active_place_departure(self):
        self.commits += 1

    def publish_status(self, event, **fields):
        self.status.append((event, fields))

    @staticmethod
    def promote_prefetched_terminal(_message):
        # Stop after testing the terminal edge; successor selection is out of
        # scope for this crossing-lifecycle contract.
        return True


class FrontierPlaceCrossingTest(unittest.TestCase):
    def test_one_hop_is_a_place_boundary_even_for_endpoint_route_kind(self):
        self.assertTrue(CrossingExplorer.route_crosses_place_boundary(1))
        self.assertTrue(CrossingExplorer.route_crosses_place_boundary(2))
        self.assertFalse(CrossingExplorer.route_crosses_place_boundary(0))
        self.assertFalse(CrossingExplorer.route_crosses_place_boundary(None))
        self.assertFalse(CrossingExplorer.route_crosses_place_boundary("invalid"))

    def test_successful_endpoint_keeps_departure_pending_for_physical_proof(self):
        explorer = CrossingExplorer()
        terminal = SimpleNamespace(
            route_id=1,
            route_kind="frontier_endpoint",
            goal=SimpleNamespace(
                header=SimpleNamespace(frame_id="map"),
                pose=SimpleNamespace(position=SimpleNamespace(x=13.0, y=10.0)),
            ),
        )

        explorer.on_execution_terminal(terminal)
        with explorer.planning_lock:
            explorer._drain_execution_terminals_locked()

        self.assertEqual(len(explorer.marked), 1)
        self.assertEqual(len(explorer.observation_sources), 1)
        self.assertEqual(len(explorer.portal_entries), 1)
        self.assertEqual(explorer.portal_entries[0][0][:3], (5, (12.0, 10.0), (13.0, 10.0)))
        self.assertEqual(explorer.marked[0][1]["physical_xy"], (3.0, -1.0))
        self.assertEqual(explorer.commits, 0)
        self.assertEqual(
            explorer.status[-1][0], "frontier_place_departure_terminal_pending"
        )

    def test_nearby_stale_terminal_cannot_close_a_new_route(self):
        explorer = CrossingExplorer()
        # This reproduces the storage-room failure: the old goal is only
        # 0.9 m from the new endpoint, so geometry alone would accept it.
        terminal = SimpleNamespace(
            route_id=2,
            route_kind="frontier_endpoint",
            goal=SimpleNamespace(
                header=SimpleNamespace(frame_id="map"),
                pose=SimpleNamespace(position=SimpleNamespace(x=13.9, y=10.0)),
            ),
        )

        explorer.on_execution_terminal(terminal)
        with explorer.planning_lock:
            explorer._drain_execution_terminals_locked()

        self.assertEqual(explorer.marked, [])
        self.assertEqual(explorer.observation_sources, [])
        self.assertEqual(explorer.status, [])

    def test_portal_terminal_uses_its_frozen_odom_endpoint(self):
        explorer = CrossingExplorer()
        explorer.active_route_kind = "portal_transition"
        terminal = SimpleNamespace(
            route_id=1,
            route_kind="portal_transition",
            goal=SimpleNamespace(
                header=SimpleNamespace(frame_id="odom"),
                pose=SimpleNamespace(position=SimpleNamespace(x=3.0, y=-1.0)),
            ),
        )

        completed, terminal_delta = explorer.matching_execution_terminal(terminal)

        self.assertEqual(completed, explorer.active_frontier)
        self.assertAlmostEqual(terminal_delta, 0.0)


if __name__ == "__main__":
    unittest.main()
