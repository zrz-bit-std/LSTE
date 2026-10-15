#!/usr/bin/env python3
"""Regression for releasing a failed persistent target-plan lease."""

import ast
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace
import threading
import time
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


class StringStub:
    def __init__(self, data=""):
        self.data = data


class GoalStatusStub:
    PREEMPTED = 2


def load_method(source_path, class_name, method_name):
    tree = ast.parse(source_path.read_text(encoding="utf-8"), filename=str(source_path))
    owner = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    method = next(
        node
        for node in owner.body
        if isinstance(node, ast.FunctionDef) and node.name == method_name
    )
    namespace = {
        "copy": copy,
        "json": json,
        "math": math,
        "time": time,
        "now_for": lambda _owner: 42.0,
        "String": StringStub,
        "GoalStatus": GoalStatusStub,
        "normalize_goal_context": lambda value: value or {},
        "rospy": SimpleNamespace(
            Time=SimpleNamespace(
                now=lambda: SimpleNamespace(to_sec=lambda: 42.0),
            ),
            logwarn_throttle=lambda *_args, **_kwargs: None,
            loginfo_throttle=lambda *_args, **_kwargs: None,
        ),
        "PoseStamped": lambda: SimpleNamespace(
            header=SimpleNamespace(stamp=None, frame_id="", seq=0),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(z=0.0, w=1.0),
            ),
        ),
    }
    exec(
        compile(ast.Module(body=[method], type_ignores=[]), str(source_path), "exec"),
        namespace,
    )
    return namespace[method_name]


ON_PLAN_RESULT = load_method(
    SCRIPTS / "teb_goal_bridge_persistent_target_result.py",
    "TebGoalBridgePersistentTargetResultMixin",
    "on_persistent_target_plan_result",
)
ON_GOAL_COMMAND = load_method(
    SCRIPTS / "teb_goal_bridge_mission_input.py",
    "TebGoalBridgeMissionInputMixin",
    "on_goal_command",
)
ON_INTENT = load_method(
    SCRIPTS / "teb_goal_bridge_intent.py",
    "TebGoalBridgeIntentMixin",
    "on_intent",
)
RELEASE_TARGET = load_method(
    SCRIPTS / "teb_goal_bridge_action_health.py",
    "TebGoalBridgeActionHealthMixin",
    "_release_failed_target_controller_lease_locked",
)


class PublisherSpy:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class PersistentTargetFailureFixture:
    on_persistent_target_plan_result = ON_PLAN_RESULT
    on_goal_command = ON_GOAL_COMMAND
    on_intent = ON_INTENT
    _release_failed_target_controller_lease_locked = RELEASE_TARGET

    def __init__(self):
        self.lock = threading.RLock()
        self.persistent_execution = True
        self.task_done = False
        self.latest_goal = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=4.0, y=5.0),
            ),
        )
        self.latest_goal_transaction_id = 7
        self.persistent_target_pending_transaction = 7
        self.persistent_installed_target_transaction = 0
        self.persistent_target_request_transaction = 7
        self.persistent_target_terminal_boundary_transaction = 0
        self.persistent_target_pending_goal = self.latest_goal
        self.persistent_installed_target_goal = None
        self.latest_intent_priority = 2
        self.active_intent_priority = 2
        self.latest_intent_source = "target_parallax"
        self.latest_route_kind = "target_approach"
        self.latest_mission_route_kind = "target_approach"
        self.latest_route_id = 44
        self.active_route_id = 44
        self.active_target_track_id = "yellow-cup:1"
        self.latest_target_epoch = 3
        self.latest_target_track_id = "yellow-cup:1"
        self.latest_target_viewpoint_candidate_id = "view-1"
        self.latest_target_viewpoint_attempt_id = "attempt-1"
        self.latest_goal_context = {"work_item_id": 85}
        self.action_active = True
        self.target_failure_latched = False
        self.target_failure_goal = None
        self.target_failure_epoch = 0
        self.target_failure_track_id = ""
        self.target_failure_generation = 0
        self.target_failure_count = 0
        self.action_generation = 11
        self.global_frame = "map"
        self.handoff_requested = True
        self.cancel_calls = []
        self.action_client = SimpleNamespace(
            cancel_goal=lambda: self.cancel_calls.append("cancel_goal")
        )
        self.clear_calls = []
        self.statuses = []
        self.target_failure_pub = PublisherSpy()
        self.use_goal_command = True
        self.require_intent = False
        self.mode = "teb"
        self.active_mode = "teb"
        self.position_epsilon = 0.05
        self.global_frame = "map"
        self.frontier_portal_wait = False
        self.frontier_portal_wait_route_id = 0
        self.target_requests = []

    def _goal_from_persistent_result(self, _payload):
        return None

    def _clear_persistent_target_request_locked(self, reason, force=False):
        self.clear_calls.append((str(reason), bool(force)))
        self.persistent_target_request_transaction = 0
        return True

    def publish_bridge_status(self, event, **fields):
        self.statuses.append((str(event), fields))

    def _normalize_goal(self, goal):
        return goal

    def _frontier_route_is_stale_locked(self, _source, _priority, _route_id):
        return False

    def _frontier_route_released_locked(self, _source, _priority, _route_id):
        return False

    def _accept_newer_frontier_route_locked(self, _source, _priority, _route_id):
        return True

    def _is_active_mode(self):
        return False

    def _request_persistent_target_locked(self, reason):
        self.target_requests.append(str(reason))
        return True

    def _publish_persistent_mission_goal_locked(self, _reason):
        return True

    def _adopt_persistent_mission_goal_locked(self):
        return True

    def _target_failure_blocks_identity_locked(
        self,
        priority,
        target_track_id="",
        target_epoch=0,
        source="unknown",
        transaction_id=0,
    ):
        # Use the production method loaded from the same source file. Keeping
        # this explicit makes the fixture fail if the guard is removed.
        return TARGET_FAILURE_GUARD(
            self,
            priority,
            target_track_id,
            target_epoch,
            source,
            transaction_id,
        )


TARGET_FAILURE_GUARD = load_method(
    SCRIPTS / "teb_goal_bridge_intent.py",
    "TebGoalBridgeIntentMixin",
    "_target_failure_blocks_identity_locked",
)


class PersistentTargetFailureReleaseTest(unittest.TestCase):
    def test_navfn_failure_releases_target_lease_for_frontier(self):
        bridge = PersistentTargetFailureFixture()
        bridge.on_persistent_target_plan_result(
            SimpleNamespace(
                data=json.dumps({"event": "target_plan_failed", "transaction_id": 7})
            )
        )

        self.assertEqual(bridge.clear_calls, [("target_plan_failed", True)])
        self.assertIsNone(bridge.latest_goal)
        self.assertEqual(bridge.latest_intent_priority, 0)
        self.assertEqual(bridge.latest_intent_source, "waiting_global_slam_frontier")
        self.assertEqual(bridge.persistent_target_pending_transaction, 0)
        self.assertEqual(bridge.persistent_installed_target_transaction, 0)
        # Keep the semantic terminal transaction for late result correlation;
        # the separate lease tombstone rejects queued target messages.
        self.assertEqual(bridge.persistent_target_terminal_boundary_transaction, 7)
        self.assertEqual(bridge.target_lease_tombstone_transaction_id, 7)
        self.assertFalse(bridge.action_active)
        self.assertEqual(bridge.cancel_calls, ["cancel_goal"])
        self.assertTrue(bridge.target_failure_latched)
        self.assertEqual(bridge.target_failure_track_id, "yellow-cup:1")

        failure = json.loads(bridge.target_failure_pub.messages[0].data)
        self.assertEqual(failure["status"], "NAVFN_NO_PATH")
        self.assertEqual(failure["target_track_id"], "yellow-cup:1")
        released = [
            fields for event, fields in bridge.statuses
            if event == "target_controller_lease_released"
        ]
        self.assertEqual(len(released), 1)
        self.assertEqual(released[0]["next_owner"], "global_slam_frontier")
        plan_failed = [
            fields for event, fields in bridge.statuses
            if event == "persistent_target_plan_failed"
        ]
        self.assertEqual(plan_failed[0]["controller_lease"], "released")
        self.assertEqual(plan_failed[0]["target_track_id"], "yellow-cup:1")

    def test_stale_target_overlay_cannot_reclaim_priority_after_frontier_replan(self):
        """A newer transport id must not resurrect the failed target lease."""
        bridge = PersistentTargetFailureFixture()
        bridge.on_persistent_target_plan_result(
            SimpleNamespace(
                data=json.dumps({"event": "target_plan_failed", "transaction_id": 7})
            )
        )

        # This is the ownership contract the old Level 4 run violated: the
        # target failure must request an explicit room-claim release before a
        # frontier replan, rather than keeping target_room_claim=true.
        self.assertTrue(bridge.target_failure_latched)
        self.assertEqual(bridge.target_failure_track_id, "yellow-cup:1")
        release_event = next(
            fields for event, fields in bridge.statuses
            if event == "target_controller_lease_released"
        )
        self.assertEqual(release_event["next_owner"], "global_slam_frontier")

        # The new frontier route is admitted while the failed target latch is
        # still held. Physical frontier progress is the boundary that clears
        # that latch; merely receiving a replacement does not.
        bridge.on_goal_command(
            SimpleNamespace(
                data=json.dumps(
                    {
                        "event": "mission_goal",
                        "transaction_id": 8,
                        "priority": 0,
                        "source": "global_slam_frontier",
                        "route_id": 45,
                        "route_kind": "frontier_endpoint",
                        "goal": [9.0, 10.0],
                        "frame_id": "map",
                    }
                )
            )
        )
        self.assertEqual(bridge.latest_intent_priority, 0)
        self.assertEqual(bridge.latest_route_id, 45)

        # A queued old target overlay can carry a larger transaction number,
        # so the transaction-order guard alone is insufficient.
        bridge.on_goal_command(
            SimpleNamespace(
                data=json.dumps(
                    {
                        "event": "mission_goal",
                        "transaction_id": 9,
                        "priority": 2,
                        "source": "target_terminal_advance",
                        "route_id": 0,
                        "route_kind": "target_approach",
                        # A larger transport/observation epoch bypasses the
                        # numeric tombstone and must still be rejected while
                        # the failed semantic track remains latched.
                        "target_epoch": 4,
                        "target_track_id": "yellow-cup:1",
                        "goal": [4.0, 5.0],
                        "frame_id": "map",
                    }
                )
            )
        )
        self.assertEqual(bridge.latest_intent_priority, 0)
        self.assertEqual(bridge.latest_route_id, 45)
        self.assertEqual(bridge.latest_goal_transaction_id, 8)
        self.assertEqual(bridge.target_requests, [])
        ignored = [
            fields for event, fields in bridge.statuses
            if event == "target_intent_ignored"
        ]
        self.assertEqual(len(ignored), 1)
        self.assertEqual(ignored[0]["reason"], "failed_target_latched")

    def test_stale_target_intent_is_rejected_when_legacy_intent_topic_is_used(self):
        bridge = PersistentTargetFailureFixture()
        bridge.use_goal_command = False
        bridge.on_persistent_target_plan_result(
            SimpleNamespace(
                data=json.dumps({"event": "target_plan_failed", "transaction_id": 7})
            )
        )

        bridge.on_intent(
            SimpleNamespace(
                data=json.dumps(
                    {
                        "source": "target_terminal_advance",
                        "priority": 2,
                        "transaction_id": 9,
                        "target_epoch": 4,
                        "target_track_id": "yellow-cup:1",
                        "goal": [4.0, 5.0],
                    }
                )
            )
        )
        self.assertEqual(bridge.latest_intent_priority, 0)
        self.assertEqual(bridge.latest_intent_source, "waiting_global_slam_frontier")
        ignored = [
            fields for event, fields in bridge.statuses
            if event == "target_intent_ignored"
        ]
        self.assertEqual(len(ignored), 1)

    def test_stale_terminal_observation_cannot_reclaim_failed_track(self):
        bridge = PersistentTargetFailureFixture()
        bridge.on_persistent_target_plan_result(
            SimpleNamespace(
                data=json.dumps({"event": "target_plan_failed", "transaction_id": 7})
            )
        )

        bridge.on_goal_command(
            SimpleNamespace(
                data=json.dumps(
                    {
                        "event": "target_terminal_observation",
                        "transaction_id": 9,
                        "target_epoch": 4,
                        "target_track_id": "yellow-cup:1",
                    }
                )
            )
        )
        self.assertEqual(bridge.latest_intent_priority, 0)
        ignored = [
            fields
            for event, fields in bridge.statuses
            if event == "target_intent_ignored"
        ]
        self.assertTrue(ignored)
        self.assertEqual(ignored[-1]["reason"], "failed_target_latched")


if __name__ == "__main__":
    unittest.main()
