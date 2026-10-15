"""Regression tests for non-blocking Navfn endpoint validation."""

from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import global_frontier_planning_navfn as navfn_module  # noqa: E402
from global_frontier_planning_navfn import (  # noqa: E402
    GlobalFrontierPlanningNavfnMixin,
)


class _SlowNavfnExplorer(GlobalFrontierPlanningNavfnMixin):
    def __init__(self):
        self.navfn_plan_validation = True
        self.persistent_execution = False
        self.navfn_make_plan_service = "/fake_navfn"
        self.navfn_make_plan_timeout = 0.05
        self.navfn_startup_probe_distance = 0.4
        self.navfn_empty_warmup_seconds = 0.0
        self.navfn_first_response_wall = None
        self.navfn_first_nonempty_response_wall = None
        self.navfn_last_validation_state = "unavailable"
        self.navfn_last_plan_endpoint = None
        self.map_msg = SimpleNamespace(
            info=SimpleNamespace(resolution=0.10),
        )
        self.calls = 0
        self.call_started = threading.Event()
        self.call_finished = threading.Event()

    def navfn_service(self, _request):
        self.calls += 1
        self.call_started.set()
        time.sleep(0.25)
        self.call_finished.set()
        return SimpleNamespace(plan=SimpleNamespace(poses=[]))


class _ReadinessExplorer(GlobalFrontierPlanningNavfnMixin):
    """Small fixture for the startup contract without ROS messages."""

    def __init__(self, persistent_execution):
        self.navigation_stack_ready = False
        self.navigation_readiness_state = "waiting_for_map_tf_costmap_navfn"
        self.persistent_execution = persistent_execution
        self.costmap_validation = object()
        self.probe_calls = 0
        self.validation_calls = 0
        self.statuses = []

    def cached_costmap_steps(self, _robot_map, _now):
        return self.costmap_validation

    def startup_probe_goal(self, _validation, _map_frame):
        self.probe_calls += 1
        return (0.4, 0.0)

    def navfn_goal_reachable(self, _robot_map, _goal, _map_frame):
        self.validation_calls += 1
        return True

    def publish_status(self, event, **fields):
        self.statuses.append((event, fields))


class NavfnAsyncValidationTest(unittest.TestCase):
    def setUp(self):
        self.wait_for_service = navfn_module.rospy.wait_for_service
        self.time_type = navfn_module.rospy.Time
        self.is_shutdown = navfn_module.rospy.is_shutdown
        self.warn_throttle = navfn_module.rospy.logwarn_throttle
        self.info_throttle = navfn_module.rospy.loginfo_throttle
        self.warn = navfn_module.rospy.logwarn
        self.info = navfn_module.rospy.loginfo
        navfn_module.rospy.wait_for_service = lambda *_args, **_kwargs: None
        navfn_module.rospy.Time = SimpleNamespace(now=lambda: 0)
        navfn_module.rospy.is_shutdown = lambda: False
        navfn_module.rospy.logwarn_throttle = lambda *_args, **_kwargs: None
        navfn_module.rospy.loginfo_throttle = lambda *_args, **_kwargs: None
        navfn_module.rospy.logwarn = lambda *_args, **_kwargs: None
        navfn_module.rospy.loginfo = lambda *_args, **_kwargs: None

    def tearDown(self):
        navfn_module.rospy.wait_for_service = self.wait_for_service
        navfn_module.rospy.Time = self.time_type
        navfn_module.rospy.is_shutdown = self.is_shutdown
        navfn_module.rospy.logwarn_throttle = self.warn_throttle
        navfn_module.rospy.loginfo_throttle = self.info_throttle
        navfn_module.rospy.logwarn = self.warn
        navfn_module.rospy.loginfo = self.info

    def test_slow_service_does_not_block_planning_call(self):
        explorer = _SlowNavfnExplorer()
        started = time.monotonic()
        result = explorer.navfn_goal_reachable((0.0, 0.0), (2.0, 0.0), "map")
        elapsed = time.monotonic() - started

        self.assertIsNone(result)
        self.assertLess(elapsed, 0.10)
        self.assertTrue(explorer.call_started.wait(0.20))
        self.assertTrue(explorer.call_finished.wait(1.0))

    def test_same_pending_identity_is_submitted_once(self):
        explorer = _SlowNavfnExplorer()
        first = explorer.navfn_goal_reachable((0.0, 0.0), (2.0, 0.0), "map")
        second = explorer.navfn_goal_reachable((0.01, 0.01), (2.0, 0.0), "map")

        self.assertIsNone(first)
        self.assertIsNone(second)
        self.assertTrue(explorer.call_finished.wait(1.0))
        self.assertEqual(explorer.calls, 1)

    def test_empty_result_preserves_unreachable_contract(self):
        explorer = _SlowNavfnExplorer()
        explorer.navfn_goal_reachable((0.0, 0.0), (2.0, 0.0), "map")
        self.assertTrue(explorer.call_finished.wait(1.0))

        result = None
        deadline = time.monotonic() + 1.0
        while result is None and time.monotonic() < deadline:
            result = explorer.navfn_goal_reachable(
                (0.0, 0.0), (2.0, 0.0), "map"
            )
            if result is None:
                time.sleep(0.01)
        self.assertFalse(result)
        self.assertEqual(explorer.navfn_last_validation_state, "bootstrap_timeout")

    def test_completed_result_survives_a_slow_planning_boundary(self):
        explorer = _SlowNavfnExplorer()
        explorer.navfn_goal_reachable((0.0, 0.0), (2.0, 0.0), "map")
        self.assertTrue(explorer.call_finished.wait(1.0))
        # A real frontier snapshot can take longer than the service call. The
        # completed fact must remain consumable after that boundary.
        time.sleep(0.10)
        result = explorer.navfn_goal_reachable((0.0, 0.0), (2.0, 0.0), "map")
        self.assertFalse(result)

    def test_worker_completion_wakes_event_driven_planning(self):
        """A completed validation must wake a static-map event loop."""
        explorer = _SlowNavfnExplorer()
        requested = []
        explorer.decision_wake_scheduler = SimpleNamespace(
            request=lambda reason: requested.append(reason)
        )

        explorer.navfn_goal_reachable((0.0, 0.0), (2.0, 0.0), "map")
        self.assertTrue(explorer.call_finished.wait(1.0))

        self.assertIn("navfn_validation_completed", requested)

    def test_rejected_lifecycle_result_does_not_strand_pending_identity(self):
        """A full lifecycle inbox must allow a Navfn identity to retry."""
        explorer = _SlowNavfnExplorer()
        explorer.lifecycle_manager = SimpleNamespace(
            current_transaction_id=1,
            enqueue_type=lambda *_args, **_kwargs: False,
        )

        explorer.navfn_goal_reachable((0.0, 0.0), (2.0, 0.0), "map")
        self.assertTrue(explorer.call_finished.wait(1.0))

        key = explorer._navfn_validation_key((0.0, 0.0), (2.0, 0.0), "map")
        self.assertNotIn(key, explorer._navfn_validation_pending)

    def test_persistent_stream_uses_costmap_readiness_until_first_mission(self):
        explorer = _ReadinessExplorer(persistent_execution=True)

        self.assertTrue(
            explorer.navigation_stack_is_ready((0.0, 0.0), "map", 1.0)
        )
        self.assertTrue(explorer.navigation_stack_ready)
        self.assertEqual(explorer.navigation_readiness_state, "ready_persistent_stream")
        self.assertEqual(explorer.probe_calls, 0)
        self.assertEqual(explorer.validation_calls, 0)
        self.assertEqual(explorer.statuses[-1][0], "navigation_readiness")
        self.assertEqual(explorer.statuses[-1][1]["navfn_probe"], "deferred")

    def test_endpoint_action_keeps_navfn_startup_probe_contract(self):
        explorer = _ReadinessExplorer(persistent_execution=False)

        self.assertTrue(
            explorer.navigation_stack_is_ready((0.0, 0.0), "map", 1.0)
        )
        self.assertEqual(explorer.probe_calls, 1)
        self.assertEqual(explorer.validation_calls, 1)
        self.assertEqual(explorer.navigation_readiness_state, "ready")

    def test_consumed_empty_result_requests_event_driven_retry(self):
        explorer = _SlowNavfnExplorer()
        explorer.navfn_empty_warmup_seconds = 1.0
        requested = []
        explorer.decision_wake_scheduler = SimpleNamespace(
            request=lambda reason: requested.append(reason)
        )

        explorer.navfn_goal_reachable((0.0, 0.0), (2.0, 0.0), "map")
        self.assertTrue(explorer.call_finished.wait(1.0))
        self.assertIsNone(
            explorer.navfn_goal_reachable((0.0, 0.0), (2.0, 0.0), "map")
        )
        self.assertIn("navfn_validation_retry", requested)


if __name__ == "__main__":
    unittest.main()
