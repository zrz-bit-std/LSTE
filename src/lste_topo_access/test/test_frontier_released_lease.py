#!/usr/bin/env python3
"""Regression tests for delayed controller terminals after graph release."""

import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
import sys


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
SOURCE = SCRIPTS_DIR / "global_frontier_event_callbacks.py"
sys.path.insert(0, str(SCRIPTS_DIR))
from global_frontier_route_state import GlobalFrontierRouteStateMixin  # noqa: E402


def _load_callback():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    owner = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierEventCallbacksMixin"
    )
    method = next(
        node for node in owner.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "on_bridge_status"
    )
    namespace = {
        "json": json,
        "time": __import__("time"),
        "now_for": lambda _owner: 100.0,
        "rospy": SimpleNamespace(
            logwarn=lambda *_args, **_kwargs: None,
        ),
    }
    exec(
        compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"),
        namespace,
    )
    return namespace["on_bridge_status"]


class _PortalLedger:
    def __init__(self):
        self.failure_calls = 0

    def get(self, _portal_id):
        return {"id": 5, "state": "crossed"}

    def execution_failed(self, *_args, **_kwargs):
        self.failure_calls += 1
        return {"id": 5, "source_place_id": 3, "state": "failed"}

    def failed(self, *_args, **_kwargs):
        self.failure_calls += 1
        return {"id": 5, "source_place_id": 3, "state": "failed"}


class _Explorer:
    on_bridge_status = _load_callback()

    def __init__(self):
        self.task_done = False
        self.active_frontier = None
        self.active_route_id = 16
        self.last_released_route_id = 16
        self.last_released_route_kind = "portal_transition"
        self.last_released_route_terminal_received = True
        self.last_released_route_controller_pending = True
        self.last_portal_hypothesis_id = 5
        self.portal_hypothesis_ledger = _PortalLedger()
        self.recovery_pending_route_id = 0
        self.recovery_pending_behavior = ""
        self.recovery_pending_reason = ""
        self.recovery_pending_wall = 0.0
        self.last_planning_wall = 42.0
        self.status = []

    def publish_status(self, event, **fields):
        self.status.append((event, fields))

    def _remember_bridge_dispatch_contract(self, _payload):
        return None

    def _bridge_status_contract(self, payload):
        return {
            "route_id": int(payload.get("active_route_id", 0)),
            "route_kind": payload.get("active_route_kind", ""),
            "mission_route_kind": payload.get("active_mission_route_kind", ""),
            "action_generation": 1,
        }

    def _bridge_terminal_matches_dispatch(self, _payload, _contract):
        return True, "matched"


class _ReleaseOwner(GlobalFrontierRouteStateMixin):
    def __init__(self):
        self.active_route_kind = "frontier_endpoint"
        self.active_route_id = 56
        self.active_terminal_received = True
        self.active_terminal_contract = {
            "route_id": 56, "transaction_id": 181, "action_generation": 0,
            "graph_transaction_id": 181, "reason": "terminal_boundary",
        }
        self._bridge_dispatch_contracts = {
            56: {
                "route_id": 56, "transaction_id": 56, "action_generation": 99,
                "graph_transaction_id": 181,
                "lifecycle_transaction_id": 2584865440244308403995277696943906430,
                "map_epoch": 264, "origin_epoch": 264, "projection_epoch": 264,
                "route_kind": "frontier_endpoint",
            }
        }
        self.active_route_dispatch_contract = None
        self.controller_owned_route_failure = False
        self.place_departure = SimpleNamespace(active=False)
        self.active_frontier = (0, 0, 1.0, 2.0)
        self.active_visited_odom_cells = set()
        self.status = []
        self.pending_contract = None
        self.pending_controller_lease_releases = {}

    def publish_status(self, event, **fields):
        self.status.append((event, fields))

    def _controller_release_ack_matches(self, _contract):
        return False

    def _remember_controller_release_contract(self, contract):
        self.pending_contract = dict(contract)
        self.pending_controller_lease_releases[contract["route_id"]] = dict(contract)
        self.awaiting_controller_lease_release_ack = True

    def _clear_graph_route_plan_lease(self, _reason):
        pass

    def clear_active_route_history(self):
        pass


class ReleasedControllerLeaseTest(unittest.TestCase):
    def test_unstarted_semantic_route_never_waits_for_transport_ack(self):
        policy = GlobalFrontierRouteStateMixin.controller_release_requires_ack
        self.assertFalse(policy(47, 0, False))
        self.assertFalse(policy(0, 91, False))

    def test_terminal_semantic_identity_cannot_overwrite_bridge_dispatch_identity(self):
        merge = GlobalFrontierRouteStateMixin.merge_route_release_contract
        exact = {
            "route_id": 56, "transaction_id": 56, "action_generation": 99,
            "graph_transaction_id": 181, "lifecycle_transaction_id": 2584865440244308403995277696943906430,
            "map_epoch": 264, "route_kind": "frontier_endpoint",
        }
        stale_semantic = {
            "route_id": 56, "transaction_id": 181, "action_generation": 0,
            "graph_transaction_id": 181, "reason": "terminal_boundary",
        }
        merged = merge(56, exact, None, stale_semantic, None)
        self.assertEqual(merged["route_id"], 56)
        self.assertEqual(merged["transaction_id"], 56)
        self.assertEqual(merged["action_generation"], 99)
        self.assertEqual(merged["graph_transaction_id"], 181)
        self.assertEqual(merged["reason"], "terminal_boundary")

    def test_release_publishes_exact_bridge_identity_after_stale_terminal(self):
        owner = _ReleaseOwner()
        self.assertTrue(owner.release_active_frontier(queue_successor=False, preserve_place_departure=True))
        released = next(fields for event, fields in owner.status
                        if event == "controller_lease_released")
        self.assertEqual(released["route_id"], 56)
        self.assertEqual(released["transaction_id"], 56)
        self.assertEqual(released["action_generation"], 99)
        self.assertEqual(owner.pending_contract["transaction_id"], 56)
        self.assertEqual(owner.pending_contract["action_generation"], 99)
        self.assertEqual(released["release_phase"], "awaiting_transport_ack")

    def test_release_merge_ignores_a_different_route_terminal(self):
        merge = GlobalFrontierRouteStateMixin.merge_route_release_contract
        exact = {"route_id": 56, "transaction_id": 56, "action_generation": 99}
        successor = {"route_id": 57, "transaction_id": 57, "action_generation": 99}
        merged = merge(56, exact, None, successor, None)
        self.assertEqual(merged["route_id"], 56)
        self.assertEqual(merged["transaction_id"], 56)

    def test_dispatched_route_and_observed_terminal_still_require_ack(self):
        policy = GlobalFrontierRouteStateMixin.controller_release_requires_ack
        self.assertTrue(policy(47, 75, False))
        self.assertTrue(policy(47, 0, True))

    def test_delayed_failure_is_cleanup_after_logical_terminal(self):
        explorer = _Explorer()
        payload = {
            "event": "terminal",
            "active_route_id": 16,
            "active_intent_source": "global_slam_frontier",
            "active_route_kind": "portal_transition",
            "active_mission_route_kind": "portal_transition",
            "status": 4,
            "status_text": "ABORTED",
        }

        explorer.on_bridge_status(SimpleNamespace(data=json.dumps(payload)))

        self.assertFalse(explorer.last_released_route_controller_pending)
        self.assertEqual(explorer.last_planning_wall, 0.0)
        self.assertEqual(explorer.portal_hypothesis_ledger.failure_calls, 0)
        self.assertEqual(explorer.status[-1][0], "execution_terminal_failure")
        self.assertEqual(
            explorer.status[-1][1]["lease_phase"],
            "post_terminal_controller_cleanup",
        )


if __name__ == "__main__":
    unittest.main()
