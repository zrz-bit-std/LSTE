#!/usr/bin/env python3
"""Regression tests for target-room observation ownership."""

import ast
import copy
import json
import unittest
from pathlib import Path
from types import SimpleNamespace

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
import sys

if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from goal_manager_target_follow import GoalManagerTargetFollowMixin
from goal_manager_target_completion import GoalManagerTargetCompletionMixin
from goal_manager_frontier import GoalManagerFrontierMixin
from goal_manager_target_recovery import GoalManagerTargetRecoveryMixin
from goal_manager_target_segments import GoalManagerTargetSegmentsMixin
from goal_manager_target_utils import GoalManagerTargetUtilsMixin


def load_teb_goal_failure_method():
    """Load the ROS callback body without importing its ROS message package."""
    source = SCRIPTS_DIR / "goal_manager_teb_callbacks.py"
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    callback_class = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GoalManagerTebCallbacksMixin"
    )
    method = next(
        node
        for node in callback_class.body
        if isinstance(node, ast.FunctionDef) and node.name == "on_teb_goal_failure"
    )
    namespace = {
        "String": lambda data="": SimpleNamespace(data=data),
        "copy": copy,
        "json": json,
        "EXPLORE_SUS_C_MODE": "explore_sus_c_mode",
    }
    exec(
        compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"),
        namespace,
    )
    return namespace["on_teb_goal_failure"]


METHODS = {
    "goal_from_target_follow": GoalManagerTargetFollowMixin.goal_from_target_follow,
    "target_close_completion_eligible": (
        GoalManagerTargetCompletionMixin.target_close_completion_eligible
    ),
    "target_detection_is_close": GoalManagerTargetCompletionMixin.target_detection_is_close,
    "target_close_confirmation_grace": (
        GoalManagerTargetCompletionMixin.target_close_confirmation_grace
    ),
    "target_close_score_threshold": GoalManagerTargetCompletionMixin.target_close_score_threshold,
    "target_close_confirmation_active": (
        GoalManagerTargetCompletionMixin.target_close_confirmation_active
    ),
    "target_terminal_observation_eligible": (
        GoalManagerTargetCompletionMixin.target_terminal_observation_eligible
    ),
    "target_evidence_timeout": GoalManagerTargetUtilsMixin.target_evidence_timeout,
    "target_candidate_information_active": (
        GoalManagerTargetUtilsMixin.target_candidate_information_active
    ),
    "target_pursuit_hint_distance": GoalManagerTargetSegmentsMixin.target_pursuit_hint_distance,
    "clear_target_memory": GoalManagerTargetCompletionMixin.clear_target_memory,
    "release_target_room_claim": GoalManagerFrontierMixin.release_target_room_claim,
    "on_teb_goal_failure": load_teb_goal_failure_method(),
}


class TargetCloseEvidenceTest(unittest.TestCase):
    class GoalManagerStub(GoalManagerTargetFollowMixin):
        target_close_confirmation_grace = METHODS[
            "target_close_confirmation_grace"
        ]
        target_close_completion_eligible = METHODS[
            "target_close_completion_eligible"
        ]
        target_detection_is_close = METHODS["target_detection_is_close"]
        target_hypothesis_at_safe_standoff = (
            GoalManagerTargetCompletionMixin.target_hypothesis_at_safe_standoff
        )
        target_close_score_threshold = METHODS["target_close_score_threshold"]
        target_close_confirmation_active = METHODS[
            "target_close_confirmation_active"
        ]
        target_terminal_observation_eligible = METHODS[
            "target_terminal_observation_eligible"
        ]
        target_evidence_timeout = METHODS["target_evidence_timeout"]
        target_pursuit_hint_distance = METHODS["target_pursuit_hint_distance"]
        clear_target_memory = METHODS["clear_target_memory"]
        release_target_room_claim = METHODS["release_target_room_claim"]
        release_failed_target_route = (
            GoalManagerTargetRecoveryMixin.release_failed_target_route
        )

        def __init__(self):
            self.target_done_max_detection_age = 0.75
            self.target_observation_hold = 4.5
            self.target_follow_candidate_timeout = 8.0
            self.follow_target_lost_timeout = 12.0
            self.target_follow_confirm_window = 20.0
            self.target_done_min_score = 0.40
            self.target_done_min_box_width = 0.05
            self.target_done_min_box_height = 0.05
            self.target_done_require_approach_terminal = False
            self.target_follow_min_score = 0.20
            self.target_follow_min_box_size = 0.01
            self.target_follow_confirmed = True
            self.target_completed_segments = 1
            self.target_track_id = "yellow_cup:yellow cup:1"
            self.target_approach_track_id = self.target_track_id
            self.target_close_since = 10.0
            self.target_close_last_seen = 10.0
            self.target_close_hits = 1
            self.target_close_wait_reported = False
            self.target_done_min_fresh_hits = 3
            self.target_blocked = False
            self.target_route_continuity_deferred = False
            self.target_last_goal = object()
            self.target_reacquire_goal = None
            self.target_reacquire_started = None
            self.target_reacquire_attempts = 0
            self.target_candidate_room_claim_requested = False
            self.target_candidate_last_seen = None
            self.target_candidate = None
            self.last_goal_source = ""
            self.latest_pose = object()
            self.target_goal_reached_radius = 0.75
            self.controller_mode = "keyboard"
            self.target_segment_terminal_ready = False
            self.target_terminal_close_candidate_seen = False
            self.target_observation_epoch = 3
            self.target_terminal_reobserve_min_epoch = 2
            self.target_terminal_reobserve_pending = False
            self.target_last_heading = None
            self.target_terminal_blind_advances = 0
            self.target_cache_max_advances = 2
            self.follow_target_step_distance = 1.5
            self.target_execution_state = "TARGET_CANDIDATE"
            self.goal_source = ""
            self.navigation_holds = []
            self.events = []
            self.release_calls = 0
            self.global_frontier_enabled = True
            self.replan_requests = []

        def goal_robot_distance(self, _goal):
            return 0.0

        def can_prepare_target_continuous_handoff(self, _distance):
            return False

        def set_navigation_hold(self, active, reason):
            self.navigation_holds.append((bool(active), str(reason)))

        def publish_goal_arbitration(self, event, **fields):
            self.events.append((event, fields))

        def release_target_follow_to_frontier(self, _now):
            self.release_calls += 1
            return object()

        def fresh_global_frontier_goal(self, _now):
            return None

        def request_global_frontier_replan(self, reason, **fields):
            self.replan_requests.append((reason, fields))
            return len(self.replan_requests)

    def test_confirmed_direct_track_uses_follow_score_at_close_range(self):
        manager = self.GoalManagerStub()
        self.assertEqual(manager.target_close_score_threshold(), 0.20)
        manager.target_follow_confirmed = False
        self.assertEqual(manager.target_close_score_threshold(), 0.40)

    def test_strong_unconfirmed_candidate_can_request_information_view(self):
        manager = self.GoalManagerStub()
        manager.target_follow_confirmed = False
        manager.target_candidate_room_claim_requested = True
        manager.target_candidate_last_seen = 10.0
        manager.target_follow_candidate_timeout = 8.0
        manager.target_candidate = SimpleNamespace(score=0.45, w=0.03, h=0.04)

        self.assertTrue(
            METHODS["target_candidate_information_active"](manager, 11.0)
        )
        self.assertFalse(
            METHODS["target_candidate_information_active"](manager, 18.0)
        )

    def test_direct_candidate_uses_its_own_timeout_without_monitor_context(self):
        manager = self.GoalManagerStub()
        manager.target_follow_confirmed = False
        manager.effective_mode = "catch_ctx_mode"
        self.assertEqual(manager.target_evidence_timeout(), 8.0)
        manager.target_follow_confirmed = True
        self.assertEqual(manager.target_evidence_timeout(), 12.0)

    def test_target_pursuit_hint_extends_the_existing_short_step_budget(self):
        manager = self.GoalManagerStub()
        self.assertEqual(manager.target_pursuit_hint_distance(), 4.5)
        manager.target_cache_max_advances = 0
        self.assertEqual(manager.target_pursuit_hint_distance(), 1.5)

    def test_confirmed_direct_close_track_can_complete_without_blind_last_step(self):
        manager = self.GoalManagerStub()
        manager.target_completed_segments = 0
        manager.target_approach_track_id = ""
        self.assertTrue(manager.target_close_completion_eligible())
        manager.target_done_require_approach_terminal = True
        self.assertFalse(manager.target_close_completion_eligible())

    def test_approach_terminal_remains_eligible_in_strict_mode(self):
        manager = self.GoalManagerStub()
        manager.target_done_require_approach_terminal = True
        self.assertTrue(manager.target_close_completion_eligible())

    def test_terminal_track_continuity_can_start_at_validated_endpoint(self):
        manager = self.GoalManagerStub()
        manager.target_segment_terminal_ready = True
        detection = SimpleNamespace(score=0.50, w=0.032, h=0.049)

        self.assertTrue(
            manager.target_terminal_observation_eligible(detection)
        )

    def test_uncalibrated_ray_point_cannot_complete_a_small_target(self):
        """Low-residual diagnostic geometry must not create a false task_done."""
        manager = self.GoalManagerStub()
        manager.target_hypothesis_xy = (0.0, 1.0)
        manager.target_hypothesis_ray_count = 6
        manager.target_hypothesis_navigation_enabled = False
        manager.target_minimum_viewpoint_distance = lambda: 0.9
        detection = SimpleNamespace(score=0.8, w=0.02, h=0.02)
        self.assertFalse(manager.target_detection_is_close(detection))

    def test_close_vote_keeps_target_room_observation_until_grace_expires(self):
        manager = self.GoalManagerStub()
        self.assertTrue(manager.target_close_confirmation_active(29.99))
        self.assertFalse(manager.target_close_confirmation_active(30.00))

    def test_direct_close_track_keeps_room_without_an_approach_terminal(self):
        manager = self.GoalManagerStub()
        manager.target_completed_segments = 0
        manager.target_approach_track_id = ""
        self.assertTrue(manager.target_close_confirmation_active(29.99))
        self.assertFalse(manager.target_close_confirmation_active(30.00))

    def test_close_vote_cannot_release_to_frontier_before_confirmation(self):
        manager = self.GoalManagerStub()
        goal_from_target_follow = METHODS["goal_from_target_follow"]
        result = goal_from_target_follow(manager, 11.0)
        self.assertIsNone(result)
        self.assertEqual(manager.release_calls, 0)
        self.assertEqual(manager.goal_source, "target_reobserving")
        self.assertEqual(manager.navigation_holds[-1], (True, "await_close_confirmation"))
        self.assertEqual(manager.events[-1][0], "target_close_confirmation_waiting")
        self.assertIsNone(goal_from_target_follow(manager, 11.1))
        self.assertEqual(
            [event for event, _fields in manager.events].count(
                "target_close_confirmation_waiting"
            ),
            1,
        )

    def test_pending_terminal_close_vote_prevents_another_viewpoint_advance(self):
        manager = self.GoalManagerStub()
        manager.target_segment_terminal_ready = True
        manager.controller_mode = "teb"
        manager.target_observation_loss_certified = lambda: False
        manager._await_required_terminal_observations = lambda _now: False
        advances = []
        manager._try_blind_target_terminal_advance = lambda now: (
            advances.append(now) or (True, object())
        )
        manager._hold_close_target_confirmation = lambda _now: True
        manager._handle_blocked_terminal_target_route = lambda _now: (False, None)

        result = GoalManagerTargetFollowMixin._handle_target_segment_terminal(
            manager, 12.0
        )

        self.assertIsNone(result)
        self.assertEqual(advances, [])

    def test_direct_close_vote_cannot_release_to_frontier_without_monitor_context(self):
        manager = self.GoalManagerStub()
        manager.target_completed_segments = 0
        manager.target_approach_track_id = ""
        manager.effective_mode = "catch_ctx_mode"
        goal_from_target_follow = METHODS["goal_from_target_follow"]
        result = goal_from_target_follow(manager, 11.0)
        self.assertIsNone(result)
        self.assertEqual(manager.release_calls, 0)
        self.assertEqual(manager.goal_source, "target_reobserving")
        self.assertEqual(manager.navigation_holds[-1], (True, "await_close_confirmation"))

    def test_single_direct_candidate_waits_for_claimed_place_replan(self):
        manager = self.GoalManagerStub()
        manager.target_follow_confirmed = False
        manager.target_candidate_room_claim_requested = True
        goal_from_target_follow = METHODS["goal_from_target_follow"]

        self.assertIsNone(goal_from_target_follow(manager, 11.0))
        self.assertEqual(manager.goal_source, "target_candidate_room_replan")

    def test_single_direct_candidate_accepts_only_the_claimed_frontier_update(self):
        manager = self.GoalManagerStub()
        manager.target_follow_confirmed = False
        manager.target_candidate_room_claim_requested = True
        claimed_frontier = object()
        manager.fresh_global_frontier_goal = lambda _now: claimed_frontier
        goal_from_target_follow = METHODS["goal_from_target_follow"]

        self.assertIs(goal_from_target_follow(manager, 11.0), claimed_frontier)
        self.assertEqual(manager.goal_source, "target_candidate_room_search")

    def test_expired_target_track_releases_its_room_claim(self):
        manager = self.GoalManagerStub()
        manager.target_track_id = "yellow_cup:yellow cup:7"
        manager.target_candidate_room_claim_requested = True

        manager.clear_target_memory()

        self.assertEqual(manager.replan_requests[0][0], "target_room_claim_release")
        self.assertTrue(
            manager.replan_requests[0][1]["target_room_claim_release"]
        )
        self.assertEqual(
            manager.replan_requests[0][1]["target_track_id"],
            "yellow_cup:yellow cup:7",
        )

    def test_aborted_target_route_releases_ownership_before_frontier_replan(self):
        """A controller failure clears target ownership before map recovery."""
        manager = self.GoalManagerStub()
        manager.target_last_goal = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            pose=SimpleNamespace(position=SimpleNamespace(x=21.9, y=14.9)),
        )
        # The bridge reports failures only for a currently committed intent.
        manager.last_goal = manager.target_last_goal
        manager.target_track_id = "yellow_cup:yellow cup:1"
        manager.target_observation_epoch = 7
        manager.target_failure_count = 0
        manager.target_reacquire_max_attempts = 1
        manager.target_reacquire_attempts = 0
        manager.target_blocked = False
        manager.target_blocked_goal = None
        manager.target_blocked_since = None
        manager.target_blocked_reason = ""
        manager.target_candidate_room_claim_requested = True
        manager.last_goal_source = "target_follow"
        manager.effective_mode = "catch_target_mode"
        manager.next_update_time = 12.0
        manager.controller_mode = "teb"
        manager.events = []
        manager.replan_requests = []
        manager.access_modes = []
        manager.make_goal_pose = lambda values, _yaw: SimpleNamespace(
            header=SimpleNamespace(frame_id=""),
            pose=SimpleNamespace(position=SimpleNamespace(x=values[0], y=values[1])),
        )
        manager.pose_distance = lambda _first, _second: 0.0
        manager.publish_goal_arbitration = lambda event, **fields: manager.events.append(
            (event, fields)
        )
        manager.target_semantic_hint_map = lambda **_kwargs: (21.6, 17.6)

        def request_global_frontier_replan(reason, **fields):
            manager.replan_requests.append((reason, fields))
            return len(manager.replan_requests)

        manager.request_global_frontier_replan = request_global_frontier_replan
        manager.pub_access_mode = SimpleNamespace(
            publish=lambda message: manager.access_modes.append(message.data)
        )

        class FakeTime:
            @staticmethod
            def now():
                return SimpleNamespace(to_sec=lambda: 42.0)

        class FakeRospy:
            Time = FakeTime

            @staticmethod
            def logwarn(*_args, **_kwargs):
                pass

            @staticmethod
            def loginfo(*_args, **_kwargs):
                pass

        method = METHODS["on_teb_goal_failure"]
        globals_before = method.__globals__.get("rospy")
        method.__globals__["rospy"] = FakeRospy
        try:
            method(
                manager,
                SimpleNamespace(
                    data=json.dumps(
                        {
                            "goal": [21.9, 14.9],
                            "goal_frame": "map",
                            "target_track_id": manager.target_track_id,
                            "reason": "navfn_empty_plan",
                            "status": "ABORTED",
                        }
                    )
                ),
            )
        finally:
            if globals_before is None:
                method.__globals__.pop("rospy", None)
            else:
                method.__globals__["rospy"] = globals_before

        self.assertEqual(manager.access_modes, ["explore_sus_c_mode"])
        self.assertFalse(manager.target_blocked)
        self.assertIsNone(manager.target_blocked_goal)
        self.assertIsNone(manager.target_last_goal)
        self.assertFalse(manager.target_candidate_room_claim_requested)
        self.assertEqual(
            manager.target_execution_state, "TARGET_ROUTE_FAILED_RELEASED"
        )
        self.assertEqual(manager.goal_source, "waiting_global_slam_frontier")
        self.assertEqual(len(manager.replan_requests), 1)
        self.assertEqual(manager.replan_requests[0][0], "target_room_claim_release")
        self.assertTrue(
            manager.replan_requests[0][1]["target_room_claim_release"]
        )
        self.assertEqual(
            manager.replan_requests[0][1]["target_room_claim_release_reason"],
            "target_route_failed",
        )
        self.assertFalse(manager.replan_requests[0][1].get("target_room_claim"))
        self.assertFalse(manager.replan_requests[0][1].get("target_pursuit"))
        self.assertEqual(
            manager.replan_requests[0][1]["target_track_id"],
            "yellow_cup:yellow cup:1",
        )


if __name__ == "__main__":
    unittest.main()
