#!/usr/bin/env python3
"""Regression tests for dispatch-time TEB action identity."""

import ast
import copy
from pathlib import Path
import sys
import threading
from types import MappingProxyType, SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
ACTION_CLIENT_SOURCE = SCRIPTS / "teb_goal_bridge_action_client.py"
ACTION_TERMINAL_SOURCE = SCRIPTS / "teb_goal_bridge_action_terminal.py"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from teb_goal_bridge_route_monitoring import (  # noqa: E402
    TebGoalBridgeRouteMonitoringMixin,
)
from teb_goal_bridge_mission_runtime import (  # noqa: E402
    TebGoalBridgeMissionRuntimeMixin,
)


class GoalStatusStub:
    ABORTED = 4
    SUCCEEDED = 3

    @staticmethod
    def to_string(status):
        return {
            GoalStatusStub.ABORTED: "ABORTED",
            GoalStatusStub.SUCCEEDED: "SUCCEEDED",
        }.get(status, "UNKNOWN")


class TerminalMessageStub:
    def __init__(self):
        self.route_id = 0
        self.action_generation = 0
        self.route_kind = ""
        self.goal = None


class PublisherSpy:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def load_class(source_path, class_name, namespace):
    tree = ast.parse(source_path.read_text(encoding="utf-8"), filename=str(source_path))
    owner = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    exec(
        compile(
            ast.Module(body=[owner], type_ignores=[]),
            str(source_path),
            "exec",
        ),
        namespace,
    )
    return namespace[class_name]


ACTION_CLIENT_MIXIN = load_class(
    ACTION_CLIENT_SOURCE,
    "TebGoalBridgeActionClientMixin",
    {
        "copy": copy,
        "time": SimpleNamespace(monotonic=lambda: 100.0),
        "now_for": lambda _owner: 100.0,
        "MappingProxyType": MappingProxyType,
        "rospy": SimpleNamespace(
            Duration=lambda seconds: seconds,
            loginfo=lambda *_args, **_kwargs: None,
            loginfo_throttle=lambda *_args, **_kwargs: None,
        ),
        "MoveBaseGoal": object,
    },
)
ACTION_TERMINAL_MIXIN = load_class(
    ACTION_TERMINAL_SOURCE,
    "TebGoalBridgeActionTerminalMixin",
    {
        "copy": copy,
        "time": SimpleNamespace(monotonic=lambda: 100.0),
        "now_for": lambda _owner: 100.0,
        "rospy": SimpleNamespace(
            Time=SimpleNamespace(now=lambda: 123.0),
            loginfo=lambda *_args, **_kwargs: None,
            logwarn=lambda *_args, **_kwargs: None,
            logwarn_throttle=lambda *_args, **_kwargs: None,
        ),
        "GoalStatus": GoalStatusStub,
        "FrontierExecutionTerminal": TerminalMessageStub,
        "TURN_ROUTE_KIND": "frontier_turn_connector",
    },
)


class ContractBridge(
    ACTION_TERMINAL_MIXIN,
    ACTION_CLIENT_MIXIN,
    TebGoalBridgeRouteMonitoringMixin,
    TebGoalBridgeMissionRuntimeMixin,
):
    def __init__(self):
        self.lock = threading.RLock()
        self.action_generation = 0
        self.active_action_contract = None
        self.latest_goal_transaction_id = 15
        self.latest_intent_source = "global_slam_frontier"
        self.latest_intent_priority = 0
        self.latest_route_kind = "frontier_endpoint"
        self.latest_mission_route_kind = "frontier_endpoint"
        self.latest_route_id = 15
        self.latest_target_epoch = 4
        self.latest_target_track_id = "old-track"
        self.latest_target_viewpoint_candidate_id = "old-candidate"
        self.latest_target_viewpoint_attempt_id = "old-attempt"
        self.latest_goal_context = {"route": "old"}
        self.global_frame = "map"
        self.latest_goal = None
        self.active_intent_source = "global_slam_frontier"
        self.active_intent_priority = 0
        self.active_route_kind = "frontier_endpoint"
        self.active_mission_route_kind = "frontier_endpoint"
        self.active_route_id = 15
        self.active_target_epoch = 4
        self.active_target_track_id = "old-track"
        self.active_target_viewpoint_candidate_id = "old-candidate"
        self.active_target_viewpoint_attempt_id = "old-attempt"
        self.active_goal_context = {"route": "old"}
        self.action_active = False
        self.task_done = False
        self.persistent_execution = False
        self.turn_supervisor_state = "PASS_THROUGH"
        self.handoff_requested = False
        self.frontier_observation_completion_pending = None
        self.frontier_observation_completion_radius = 0.3
        self.frontier_observation_completion_count = 0
        self.last_dispatched_goal = None
        self.active_feedback_pose = None
        self.active_feedback_frame = ""
        self.last_feedback_pose_global = None
        self.feedback_transform_failures = 0
        self.active_navfn_plan_points = []
        self.active_navfn_plan_endpoint = None
        self.active_navfn_remaining = None
        self.active_navfn_best_remaining = None
        self.active_navfn_progress_monotonic = 0.0
        self.progress_epsilon = 0.01
        self.last_terminal_goal = None
        self.last_result_status = None
        self.last_result_monotonic = 0.0
        self.terminal_count = 0
        self.dispatch_count = 0
        self.target_failure_latched = False
        self.statuses = []
        self.terminal_pub = PublisherSpy()
        self.terminal_contract_pub = PublisherSpy()
        self.terminal_dispatches = 0
        self.failed_route_contract = None

    @staticmethod
    def pose(frame, x, y):
        return SimpleNamespace(
            header=SimpleNamespace(frame_id=frame, stamp=0.0),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y),
                orientation=SimpleNamespace(z=0.0, w=1.0),
            ),
        )

    def _clear_failed_route_lease_locked(self):
        pass

    def _reset_teb_reorientation_locked(self):
        pass

    def _clear_target_failure_locked(self, _reason):
        pass

    @staticmethod
    def _goal_in_global_frame(goal):
        return goal

    def _clear_action_health_locked(self):
        self.active_action_contract = None
        self.active_route_id = 0
        self.active_route_kind = ""
        self.active_intent_source = "unknown"
        self.active_intent_priority = 0

    def _remember_failed_route_lease_locked(self, action_contract=None):
        self.failed_route_contract = action_contract

    def schedule_terminal_dispatch_locked(self):
        self.terminal_dispatches += 1

    def publish_bridge_status(self, event, **fields):
        self.statuses.append((event, fields))


class TebBridgeActionContractTest(unittest.TestCase):
    def _dispatch_route(self):
        bridge = ContractBridge()
        source_goal = bridge.pose("odom", 1.0, 2.0)
        generation = bridge._begin_action_lifecycle_locked(
            source_goal,
            bridge.pose("map", 3.0, 4.0),
        )
        return bridge, source_goal, generation

    def _adopt_successor(self, bridge):
        successor_goal = bridge.pose("odom", 9.0, 10.0)
        bridge.latest_intent_source = "global_slam_frontier"
        bridge.latest_route_kind = "portal_transition"
        bridge.latest_mission_route_kind = "cross_portal"
        bridge.latest_route_id = 16
        bridge.latest_target_epoch = 9
        bridge.latest_target_track_id = "new-track"
        bridge.latest_target_viewpoint_candidate_id = "new-candidate"
        bridge.latest_target_viewpoint_attempt_id = "new-attempt"
        bridge.latest_goal_context = {"route": "new"}
        bridge.active_intent_source = "global_slam_frontier"
        bridge.active_route_kind = "portal_transition"
        bridge.active_mission_route_kind = "cross_portal"
        bridge.active_route_id = 16
        bridge.active_target_epoch = 9
        bridge.active_target_track_id = "new-track"
        bridge.active_target_viewpoint_candidate_id = "new-candidate"
        bridge.active_target_viewpoint_attempt_id = "new-attempt"
        bridge.active_goal_context = {"route": "new"}
        bridge.last_dispatched_goal = copy.deepcopy(successor_goal)

    @staticmethod
    def _navfn_message(bridge, endpoint_x, endpoint_y):
        poses = [
            bridge.pose("map", 0.0, 0.0),
            bridge.pose("map", endpoint_x * 0.5, endpoint_y * 0.5),
            bridge.pose("map", endpoint_x, endpoint_y),
        ]
        return SimpleNamespace(
            header=SimpleNamespace(frame_id="map"), poses=poses
        )

    def test_replacement_discards_old_feedback_before_navfn_callback(self):
        """A new plan cannot project the previous action's pose."""
        bridge, _source_goal, _generation = self._dispatch_route()
        bridge.active_feedback_pose = (1.0, 0.0, 0.0)
        bridge.active_feedback_frame = "map"
        bridge.last_feedback_pose_global = bridge.pose("map", 1.0, 0.0)
        bridge.feedback_transform_failures = 3

        bridge.latest_route_id = 16
        bridge.latest_route_kind = "frontier_endpoint"
        bridge.latest_mission_route_kind = "frontier_endpoint"
        bridge.latest_goal_context = {"route": "successor"}
        bridge._begin_action_lifecycle_locked(
            bridge.pose("odom", 4.0, 0.0),
            bridge.pose("map", 4.0, 0.0),
        )

        self.assertIsNone(bridge.active_feedback_pose)
        self.assertEqual(bridge.active_feedback_frame, "")
        self.assertIsNone(bridge.last_feedback_pose_global)
        self.assertEqual(bridge.feedback_transform_failures, 0)

        bridge.on_navfn_plan(self._navfn_message(bridge, 4.0, 0.0))

        self.assertEqual(bridge.active_navfn_plan_endpoint, [4.0, 0.0])
        self.assertIsNone(bridge.active_navfn_remaining)

    def test_success_callback_keeps_dispatch_identity_after_successor_adoption(self):
        bridge, source_goal, generation = self._dispatch_route()
        contract = bridge.active_action_contract
        self._adopt_successor(bridge)

        bridge.on_done(generation, GoalStatusStub.SUCCEEDED, None)

        terminal = bridge.terminal_contract_pub.messages[-1]
        self.assertEqual(terminal.route_id, 15)
        self.assertEqual(terminal.action_generation, generation)
        self.assertEqual(terminal.route_kind, "frontier_endpoint")
        self.assertEqual(terminal.goal.pose.position.x, source_goal.pose.position.x)
        self.assertEqual(terminal.goal.pose.position.y, source_goal.pose.position.y)
        terminal_status = bridge.statuses[-1][1]
        self.assertEqual(terminal_status["route_id"], 15)
        self.assertEqual(terminal_status["action_generation"], generation)
        self.assertEqual(terminal_status["target_epoch"], 4)
        self.assertEqual(terminal_status["target_track_id"], "old-track")
        self.assertEqual(
            terminal_status["target_viewpoint_candidate_id"], "old-candidate"
        )
        self.assertEqual(
            terminal_status["target_viewpoint_attempt_id"], "old-attempt"
        )
        self.assertEqual(bridge.terminal_dispatches, 1)
        self.assertIsNone(bridge.active_action_contract)
        with self.assertRaises(TypeError):
            contract["route_id"] = 16

    def test_failure_callback_remembers_dispatch_route_for_replacement_guard(self):
        bridge, _source_goal, generation = self._dispatch_route()
        self._adopt_successor(bridge)

        bridge.on_done(generation, GoalStatusStub.ABORTED, None)

        self.assertIsNotNone(bridge.failed_route_contract)
        self.assertEqual(bridge.failed_route_contract["route_id"], 15)
        self.assertEqual(bridge.failed_route_contract["generation"], generation)
        terminal_status = bridge.statuses[-1][1]
        self.assertEqual(terminal_status["route_id"], 15)
        self.assertEqual(terminal_status["route_kind"], "frontier_endpoint")

    def test_persistent_stream_refreshes_contract_for_adopted_route(self):
        bridge, _source_goal, generation = self._dispatch_route()
        bridge.persistent_execution = True
        bridge.latest_goal = bridge.pose("map", 9.0, 10.0)
        bridge.latest_goal_transaction_id = 16
        bridge.latest_route_id = 16
        bridge.latest_route_kind = "frontier_endpoint"
        bridge.latest_mission_route_kind = "portal_probe"
        bridge.latest_goal_context = {"route": "probe"}
        bridge.latest_target_epoch = 0
        bridge.latest_target_track_id = ""
        bridge.latest_target_viewpoint_candidate_id = ""
        bridge.latest_target_viewpoint_attempt_id = ""

        bridge._adopt_persistent_mission_goal_locked()

        contract = bridge.active_action_contract
        self.assertEqual(contract["generation"], generation)
        self.assertEqual(contract["route_id"], 16)
        self.assertEqual(contract["mission_route_kind"], "portal_probe")
        self.assertEqual(contract["source_goal"].pose.position.x, 9.0)
        self.assertEqual(contract["execution_goal"].pose.position.y, 10.0)
        self.assertEqual(bridge.active_mission_route_kind, "portal_probe")


if __name__ == "__main__":
    unittest.main()
