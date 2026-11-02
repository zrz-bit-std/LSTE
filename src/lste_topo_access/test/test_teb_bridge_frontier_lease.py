#!/usr/bin/env python3
"""Regression tests for graph-to-controller frontier lease handoff."""

import ast
import json
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
STATUS_SOURCE = SCRIPTS / "teb_goal_bridge_frontier_status.py"
HEALTH_SOURCE = SCRIPTS / "teb_goal_bridge_action_health.py"


class GoalStatusStub:
    PREEMPTED = 2


def load_class(source, class_name, namespace):
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    owner = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    exec(
        compile(ast.Module(body=[owner], type_ignores=[]), str(source), "exec"),
        namespace,
    )
    return namespace[class_name]


STATUS_MIXIN = load_class(
    STATUS_SOURCE,
    "TebGoalBridgeFrontierStatusMixin",
    {"json": json, "math": __import__("math"),
     "rospy": SimpleNamespace(logwarn=lambda *_args, **_kwargs: None)},
)
TERMINAL_SOURCE = SCRIPTS / "teb_goal_bridge_action_terminal.py"
TERMINAL_MIXIN = load_class(
    TERMINAL_SOURCE,
    "TebGoalBridgeActionTerminalMixin",
    {
        "GoalStatus": SimpleNamespace(SUCCEEDED=3),
        "TURN_ROUTE_KIND": "frontier_turn_connector",
        "now_for": lambda _owner: 12.0,
        "rospy": SimpleNamespace(loginfo=lambda *_args, **_kwargs: None),
    },
)


HEALTH_MIXIN = load_class(
    HEALTH_SOURCE,
    "TebGoalBridgeActionHealthMixin",
    {
        "time": SimpleNamespace(monotonic=lambda: 100.0),
        "now_for": lambda _owner: 100.0,
        "GoalStatus": GoalStatusStub,
        "rospy": SimpleNamespace(logwarn=lambda *_args, **_kwargs: None),
    },
)


class FakeActionClient:
    def __init__(self):
        self.cancel_count = 0

    def cancel_goal(self):
        self.cancel_count += 1


class FakeBridge(STATUS_MIXIN, HEALTH_MIXIN):
    def __init__(self):
        self.lock = threading.RLock()
        self.persistent_execution = False
        self.task_done = False
        self.action_active = True
        self.active_intent_source = "global_slam_frontier"
        self.active_intent_priority = 0
        self.active_route_kind = "portal_transition"
        self.active_route_id = 17
        self.latest_intent_source = "global_slam_frontier"
        self.latest_intent_priority = 0
        self.latest_route_id = 17
        self.latest_goal = object()
        self.last_dispatched_goal = SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0),
            )
        )
        self.frontier_lease_released_route_id = 0
        self.frontier_lease_released_reason = ""
        self.frontier_observation_completion_pending = None
        self.frontier_continuous_prefetch_handoff_pending = None
        self.handoff_requested = False
        self.action_generation = 4
        self.last_result_status = None
        self.last_result_monotonic = 0.0
        self.persistent_target_pending_transaction = 0
        self.action_client = FakeActionClient()
        self.statuses = []
        self.health_cleared = False
        self.terminations = []

    def _commit_termination(self, event, **fields):
        self.terminations.append((event, fields))
        return {"committed": True}

    def _is_active_mode(self):
        return True

    def _clear_target_failure_locked(self, _reason):
        pass

    def _clear_failed_route_lease_locked(self):
        pass

    def _clear_action_health_locked(self):
        self.health_cleared = True
        self.active_route_id = 0
        self.active_intent_source = "unknown"
        self.active_intent_priority = 0
        self.active_route_kind = ""

    def publish_bridge_status(self, event, **fields):
        self.statuses.append((event, fields))


class TebBridgeFrontierLeaseTest(unittest.TestCase):
    def test_unavailable_active_route_keeps_controller_until_terminal(self):
        bridge = FakeBridge()

        bridge.on_frontier_status(SimpleNamespace(data=json.dumps({
            "event": "frontier_route_unavailable",
            "route_id": 17,
            "reason": "durable_identity_not_in_current_frontier_snapshot",
        })))

        self.assertEqual(bridge.action_client.cancel_count, 0)
        self.assertFalse(bridge.health_cleared)
        self.assertEqual(bridge.frontier_lease_released_route_id, 0)
        self.assertEqual(bridge.latest_route_id, 17)
        self.assertIsNotNone(bridge.latest_goal)
        self.assertEqual(
            [event for event, _fields in bridge.statuses],
            ["frontier_route_unavailable_deferred"],
        )
        self.assertEqual(
            bridge.statuses[-1][1]["controller_lease"],
            "held_until_terminal",
        )

    def test_unavailable_idle_route_releases_controller_lease(self):
        bridge = FakeBridge()
        bridge.action_active = False

        bridge.on_frontier_status(SimpleNamespace(data=json.dumps({
            "event": "frontier_route_unavailable",
            "route_id": 17,
            "reason": "durable_identity_not_in_current_frontier_snapshot",
        })))

        self.assertEqual(bridge.action_client.cancel_count, 0)
        self.assertTrue(bridge.health_cleared)
        self.assertEqual(bridge.frontier_lease_released_route_id, 17)
        self.assertEqual(bridge.latest_route_id, 0)
        self.assertIsNone(bridge.latest_goal)
        self.assertEqual(
            [event for event, _fields in bridge.statuses],
            ["controller_lease_released", "frontier_route_unavailable"],
        )
        self.assertEqual(
            bridge.statuses[-1][1]["released_route_id"],
            17,
        )

    def test_endpoint_terminal_allows_release_while_action_transport_is_active(self):
        """A persistent endpoint terminal is an explicit lease boundary."""
        bridge = FakeBridge()

        bridge.on_frontier_status(SimpleNamespace(data=json.dumps({
            "event": "frontier_route_unavailable",
            "route_id": 17,
            "reason": "selected_graph_obligation_not_executable_in_snapshot",
            "released_controller_route": {
                "route_id": 17,
                "controller_pending": True,
                "terminal_received": True,
            },
        })))

        # The execution terminal was already received, so this semantic
        # release must not send a redundant transport cancel.
        self.assertEqual(bridge.action_client.cancel_count, 0)
        self.assertTrue(bridge.health_cleared)
        self.assertEqual(bridge.frontier_lease_released_route_id, 17)
        self.assertEqual(
            [event for event, _fields in bridge.statuses],
            ["controller_lease_released", "frontier_route_unavailable"],
        )
        self.assertIn(
            "endpoint_terminal_",
            bridge.statuses[-1][1]["reason"],
        )


class TurnTerminalRetentionTest(unittest.TestCase):
    def test_successful_xy_terminal_is_retained_before_turn_status_arrives(self):
        class Bridge(TERMINAL_MIXIN):
            def __init__(self):
                self.turn_supervisor_state = "PASS_THROUGH"
                self.statuses = []
                self.watchdog = []
                self.move_base_terminal_pending = False

            def publish_bridge_status(self, event, **fields):
                self.statuses.append((event, fields))

            def _arm_route_lease_watchdog_locked(self, **fields):
                self.watchdog.append(fields)

        bridge = Bridge()
        retained = bridge._retain_turn_terminal_until_yaw_complete_locked(
            3,
            {"route_kind": "frontier_turn_connector", "route_id": 4},
        )

        self.assertTrue(retained)
        self.assertTrue(bridge.move_base_terminal_pending)
        self.assertEqual(bridge.statuses[0][0], "turn_execution_terminal")
        self.assertEqual(bridge.statuses[0][1]["status_text"], "SUCCEEDED_XY_WAITING_YAW")
        self.assertEqual(
            bridge.watchdog[0]["reason"],
            "turn_execution_terminal_waiting_yaw",
        )


class SemanticReleaseRebindTest(unittest.TestCase):
    def make_bridge(self, contract_route_id=29):
        bridge = FakeBridge()
        bridge.active_route_id = contract_route_id
        bridge.latest_route_id = contract_route_id
        bridge.active_route_kind = "frontier_endpoint"
        bridge.active_intent_source = "global_slam_frontier"
        bridge.active_action_contract = {
            "route_id": contract_route_id,
            "route_kind": "frontier_endpoint",
            "source": "global_slam_frontier",
            "generation": 41,
            "action_generation": 41,
            "transaction_id": 9,
            "lifecycle_transaction_id": 12,
            "graph_transaction_id": 7,
            "map_epoch": 3,
            "origin_epoch": 2,
            "projection_epoch": 5,
        }
        bridge.last_dispatch_identity = None
        bridge.released_contracts = []
        bridge._release_frontier_controller_lease_locked = (
            lambda route_id, reason, release_contract=None:
            bridge.released_contracts.append(
                (route_id, reason, dict(release_contract or {}))
            ) or True
        )
        bridge._action_identity_key = lambda route_id, generation: (
            route_id, generation
        )
        bridge._released_action_key_seen_locked = lambda _key: False
        return bridge

    def test_semantic_only_release_rebinds_to_exact_active_dispatch(self):
        bridge = self.make_bridge()
        accepted = bridge._apply_controller_lease_release_locked({
            "event": "controller_lease_release",
            "route_id": 29,
            "controller_pending": False,
            "release_phase": "semantic_only",
            "released_controller_route": {
                "route_id": 29,
                "route_kind": "frontier_endpoint",
                "action_generation": 0,
                "transaction_id": 0,
                "lifecycle_transaction_id": 12,
                "graph_transaction_id": 7,
            },
        })
        self.assertTrue(accepted)
        self.assertEqual(len(bridge.released_contracts), 1)
        route_id, _reason, contract = bridge.released_contracts[0]
        self.assertEqual(route_id, 29)
        self.assertEqual(contract["action_generation"], 41)
        self.assertEqual(contract["transaction_id"], 9)
        self.assertEqual(contract["lifecycle_transaction_id"], 12)
        self.assertEqual(contract["graph_transaction_id"], 7)
        ack = [fields for event, fields in bridge.statuses
               if event == "lease_release_ack"]
        self.assertEqual(len(ack), 1)
        self.assertEqual(ack[0]["route_id"], 29)
        self.assertEqual(ack[0]["action_generation"], 41)

    def test_semantic_only_release_never_binds_to_another_route(self):
        bridge = self.make_bridge(contract_route_id=28)
        accepted = bridge._apply_controller_lease_release_locked({
            "event": "controller_lease_release",
            "route_id": 29,
            "controller_pending": False,
            "release_phase": "semantic_only",
            "released_controller_route": {"route_id": 29},
        })
        self.assertFalse(accepted)
        self.assertEqual(bridge.released_contracts, [])
        self.assertFalse(any(event == "lease_release_ack"
                             for event, _fields in bridge.statuses))


if __name__ == "__main__":
    unittest.main()


class EarlyReleaseAckReplayTest(unittest.TestCase):
    def identity(self):
        return dict(route_id=26,route_kind='frontier_turn_connector',transaction_id=27,
                    graph_transaction_id=25,lifecycle_transaction_id=100,action_generation=41,
                    map_epoch=143,origin_epoch=143,projection_epoch=141)

    def bridge(self):
        bridge=FakeBridge();bridge.active_route_id=27;bridge.latest_route_id=27
        bridge.pending_successor_release_ack=self.identity()
        bridge.action_generation=44
        return bridge

    def request(self,identity):
        return dict(released_controller_route=identity,controller_pending=True,
                    release_phase='awaiting_transport_ack')

    def test_exact_early_ack_replay_never_cancels_newer_successor(self):
        bridge=self.bridge()
        self.assertTrue(bridge._apply_controller_lease_release_locked(self.request(self.identity())))
        self.assertEqual(bridge.active_route_id,27)
        self.assertEqual(bridge.action_client.cancel_count,0)
        self.assertEqual(bridge.terminations,[])
        ack=next(fields for event,fields in bridge.statuses if event=='lease_release_ack')
        self.assertEqual(ack['released_controller_route'],self.identity())
        self.assertEqual(ack['projection_epoch'],141)

    def test_wrong_identity_is_not_replayed(self):
        for key,value in (('route_id',27),('lifecycle_transaction_id',101),('transaction_id',28),('action_generation',42),('graph_transaction_id',26),
                          ('projection_epoch',142),('origin_epoch',144)):
            bridge=self.bridge();request=self.identity();request[key]=value
            with self.subTest(field=key):
                self.assertFalse(bridge._apply_controller_lease_release_locked(self.request(request)))
                self.assertFalse(any(event=='lease_release_ack' for event,_ in bridge.statuses))
                self.assertEqual(bridge.action_client.cancel_count,0)

    def test_watchdog_keeps_distinct_origin_and_projection_after_action_clear(self):
        bridge=self.bridge();contract=dict(self.identity(),generation=41,source='global_slam_frontier')
        identity=bridge._route_lease_watchdog_identity_locked(contract)
        self.assertEqual(identity['origin_epoch'],143)
        self.assertEqual(identity['projection_epoch'],141)
