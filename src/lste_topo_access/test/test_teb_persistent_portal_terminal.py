#!/usr/bin/env python3
"""Regression test for portal completion in the persistent action lease."""

from pathlib import Path
import sys
import threading
import unittest

from geometry_msgs.msg import PoseStamped


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import teb_goal_bridge_persistent_frontier_endpoint as endpoint_module


endpoint_module.rospy.loginfo = lambda *_args, **_kwargs: None


class FakePersistentBridge(
    endpoint_module.TebGoalBridgePersistentFrontierEndpointMixin
):
    def __init__(self):
        self.lock = threading.RLock()
        self.persistent_execution = True
        self.task_done = False
        self.action_active = True
        self.active_intent_source = "global_slam_frontier"
        self.active_intent_priority = 0
        self.active_route_kind = "portal_transition"
        self.active_route_id = 7
        self.latest_intent_source = "global_slam_frontier"
        self.latest_intent_priority = 0
        self.latest_route_kind = "portal_transition"
        self.latest_route_id = 7
        self.last_dispatched_goal = self._pose("odom", 4.0, 5.0)
        self.active_goal_global = self._pose("odom", 4.0, 5.0)
        self.prefetched_frontier_goal = None
        self.prefetched_frontier_route_id = 0
        self.persistent_frontier_endpoint_terminal_routes = set()
        self.terminal_count = 0
        self.position_epsilon = 0.05
        self.persistent_frontier_admission_endpoint_epsilon = 0.20
        self.active_navfn_plan_endpoint = None
        self.terminals = []
        self.statuses = []

    @staticmethod
    def _pose(frame, x, y):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    @staticmethod
    def _goal_in_global_frame(pose):
        return pose

    def _publish_execution_terminal_locked(self, goal):
        self.terminals.append(goal)
        return goal

    def _clear_target_failure_locked(self, _reason):
        return None

    def publish_bridge_status(self, event, **fields):
        self.statuses.append((event, fields))


class PersistentPortalTerminalTest(unittest.TestCase):
    def test_portal_endpoint_report_releases_place_graph_terminal(self):
        bridge = FakePersistentBridge()
        report = bridge._pose("odom", 4.0, 5.0)

        bridge.on_persistent_frontier_endpoint_reached(report)

        self.assertEqual(len(bridge.terminals), 1)
        self.assertIn(7, bridge.persistent_frontier_endpoint_terminal_routes)
        self.assertEqual(
            bridge.statuses[-1][0],
            "persistent_portal_transition_terminal",
        )

    def test_portal_accepts_map_reprojection_drift_of_old_canonical_goal(self):
        bridge = FakePersistentBridge()
        bridge.last_dispatched_goal = bridge._pose("map", 3.7, 5.0)
        bridge.active_goal_global = bridge._pose("map", 4.0, 5.0)
        report = bridge._pose("map", 4.0, 5.0)

        bridge.on_persistent_frontier_endpoint_reached(report)

        self.assertEqual(len(bridge.terminals), 1)
        self.assertEqual(bridge.statuses[-1][0], "persistent_portal_transition_terminal")

    def test_local_egress_endpoint_report_closes_persistent_lease(self):
        bridge = FakePersistentBridge()
        bridge.active_route_kind = "local_egress"
        bridge.latest_route_kind = "local_egress"
        report = bridge._pose("odom", 4.0, 5.0)

        bridge.on_persistent_frontier_endpoint_reached(report)

        self.assertEqual(len(bridge.terminals), 1)
        self.assertIn(7, bridge.persistent_frontier_endpoint_terminal_routes)
        self.assertEqual(
            bridge.statuses[-1][0],
            "persistent_local_egress_terminal",
        )

    def test_frontier_accepts_current_navfn_endpoint_after_map_update(self):
        bridge = FakePersistentBridge()
        bridge.active_route_kind = "frontier_endpoint"
        bridge.latest_route_kind = "frontier_endpoint"
        # The command goal moved by 0.224 m after a map update.  The currently
        # installed Navfn path, which produced the terminal report, ends at
        # the reported point and is therefore the stronger route identity.
        bridge.last_dispatched_goal = bridge._pose("map", 5.85, 19.25)
        bridge.active_goal_global = bridge._pose("map", 5.85, 19.25)
        bridge.active_navfn_plan_endpoint = [5.65, 19.35]
        report = bridge._pose("map", 5.65, 19.35)

        bridge.on_persistent_frontier_endpoint_reached(report)

        self.assertEqual(len(bridge.terminals), 1)
        self.assertEqual(
            bridge.statuses[-1][0],
            "persistent_frontier_endpoint_terminal",
        )
        self.assertEqual(
            bridge.statuses[-1][1]["endpoint_match_basis"],
            "active_navfn_plan_endpoint",
        )


if __name__ == "__main__":
    unittest.main()
