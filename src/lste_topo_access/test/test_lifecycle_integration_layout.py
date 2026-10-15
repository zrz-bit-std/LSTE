#!/usr/bin/env python3
"""Structural guards for the single-threaded ROS lifecycle boundary."""

import ast
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"


def class_node(path, name):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    return next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == name
    )


class LifecycleIntegrationLayoutTest(unittest.TestCase):
    def test_both_nodes_compose_the_lifecycle_mixin(self):
        frontier = (SCRIPTS / "lste_global_frontier_node.py").read_text(
            encoding="utf-8"
        )
        goal = (SCRIPTS / "lste_goal_manager.py").read_text(encoding="utf-8")
        self.assertIn("GlobalFrontierLifecycleMixin", frontier)
        self.assertIn("GoalManagerLifecycleMixin", goal)
        self.assertIn("_initialize_lifecycle_manager()", frontier)
        self.assertIn("_initialize_lifecycle_manager()", goal)
        bridge = (SCRIPTS / "lste_teb_goal_bridge.py").read_text(encoding="utf-8")
        turn = (SCRIPTS / "lste_teb_turn_supervisor.py").read_text(encoding="utf-8")
        self.assertIn("TebGoalBridgeLifecycleMixin", bridge)
        self.assertIn("TebTurnSupervisorLifecycleMixin", turn)
        self.assertIn("_initialize_lifecycle_manager()", bridge)
        self.assertIn("_initialize_lifecycle_manager()", turn)

    def test_ros_ingress_methods_only_enqueue(self):
        path = SCRIPTS / "global_frontier_lifecycle.py"
        owner = class_node(path, "GlobalFrontierLifecycleMixin")
        callback_names = {
            "on_map",
            "on_costmap",
            "on_costmap_update",
            "on_pose",
            "on_scan",
            "on_task",
            "on_detections",
            "on_goal_arbitration",
            "on_task_done",
            "on_bridge_status",
            "on_replan_request",
            "on_execution_terminal",
        }
        methods = {
            node.name: node
            for node in owner.body
            if isinstance(node, ast.FunctionDef)
        }
        for name in callback_names:
            self.assertIn(name, methods)
            calls = [
                node
                for node in ast.walk(methods[name])
                if isinstance(node, ast.Call)
            ]
            self.assertTrue(
                any(
                    isinstance(call.func, ast.Attribute)
                    and call.func.attr == "_enqueue_lifecycle_event"
                    for call in calls
                ),
                name,
            )

    def test_timer_is_the_only_compute_entry(self):
        for filename, owner_name in (
            ("global_frontier_lifecycle.py", "GlobalFrontierLifecycleMixin"),
            ("goal_manager_lifecycle.py", "GoalManagerLifecycleMixin"),
        ):
            owner = class_node(SCRIPTS / filename, owner_name)
            timer = next(
                node
                for node in owner.body
                if isinstance(node, ast.FunctionDef) and node.name == "on_timer"
            )
            self.assertTrue(
                any(
                    isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == "tick"
                    for node in ast.walk(timer)
                ),
                filename,
            )

    def test_execution_adapter_ingress_is_queued(self):
        for filename, owner_name, callback_names in (
            (
                "teb_goal_bridge_lifecycle.py",
                "TebGoalBridgeLifecycleMixin",
                {"on_goal", "on_goal_command", "on_intent", "on_done", "on_feedback"},
            ),
            (
                "teb_turn_supervisor_lifecycle.py",
                "TebTurnSupervisorLifecycleMixin",
                {"on_intent", "on_bridge_status", "on_goal", "on_pose", "on_timer"},
            ),
        ):
            owner = class_node(SCRIPTS / filename, owner_name)
            methods = {
                node.name: node
                for node in owner.body
                if isinstance(node, ast.FunctionDef)
            }
            for name in callback_names - {"on_timer"}:
                self.assertIn(name, methods)
                self.assertTrue(
                    any(
                        isinstance(node, ast.Call)
                        and isinstance(node.func, ast.Attribute)
                        and node.func.attr == "_enqueue_bridge_event"
                        or isinstance(node, ast.Call)
                        and isinstance(node.func, ast.Attribute)
                        and node.func.attr == "_enqueue_turn_event"
                        for node in ast.walk(methods[name])
                    ),
                    "%s:%s" % (filename, name),
                )

    def test_install_space_contains_lifecycle_modules(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        for filename in (
            "scripts/lifecycle_manager.py",
            "scripts/global_frontier_lifecycle.py",
            "scripts/goal_manager_lifecycle.py",
            "scripts/teb_goal_bridge_lifecycle.py",
            "scripts/teb_turn_supervisor_lifecycle.py",
        ):
            self.assertIn(filename, cmake)


if __name__ == "__main__":
    unittest.main()
