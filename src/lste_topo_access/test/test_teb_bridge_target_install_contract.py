"""Keep an installed target lease through its approach terminal."""

import ast
import copy
from pathlib import Path
import unittest
from types import SimpleNamespace


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
SOURCE = SCRIPTS / "teb_goal_bridge_persistent_target.py"


def load_install_method():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    owner = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "TebGoalBridgePersistentTargetMixin"
    )
    method = next(
        node
        for node in owner.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_install_persistent_target_locked"
    )
    namespace = {"copy": copy}
    exec(
        compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"),
        namespace,
    )
    return namespace["_install_persistent_target_locked"]


INSTALL_TARGET = load_install_method()


def pose(x, y):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        pose=SimpleNamespace(position=SimpleNamespace(x=float(x), y=float(y))),
    )


class TargetInstallFixture:
    _install_persistent_target_locked = INSTALL_TARGET

    def __init__(self):
        self.persistent_execution = True
        self.latest_goal = pose(4.0, 5.0)
        self.latest_intent_priority = 2
        self.latest_goal_transaction_id = 3
        self.persistent_target_pending_transaction = 3
        self.persistent_target_pending_goal = pose(4.0, 5.0)
        self.persistent_target_request_transaction = 3
        self.persistent_installed_target_goal = None
        self.persistent_installed_target_transaction = 0
        self.target_failure_latched = False
        self.action_active = False
        self.clear_calls = []
        self.approvals = []
        self.statuses = []

    def _goal_in_global_frame(self, goal):
        return goal

    @staticmethod
    def _same_goal(first, second):
        return (
            first is not None
            and second is not None
            and first.pose.position.x == second.pose.position.x
            and first.pose.position.y == second.pose.position.y
        )

    def _publish_persistent_mission_goal_locked(self, reason):
        self.approvals.append(str(reason))

    def _clear_persistent_target_request_locked(self, reason, force=False):
        self.clear_calls.append((str(reason), bool(force)))

    def publish_bridge_status(self, event, **fields):
        self.statuses.append((str(event), fields))


class TargetInstallContractTest(unittest.TestCase):
    def test_approved_target_is_not_cleared_before_approach_terminal(self):
        fixture = TargetInstallFixture()

        fixture._install_persistent_target_locked(3, pose(4.0, 5.0))

        self.assertEqual(fixture.clear_calls, [])
        self.assertEqual(fixture.persistent_target_request_transaction, 0)
        self.assertEqual(fixture.persistent_target_pending_transaction, 0)
        self.assertEqual(fixture.persistent_installed_target_transaction, 3)
        self.assertEqual(fixture.approvals, ["target_plan_installed"])
        self.assertEqual(
            fixture.statuses[-1][0], "persistent_target_plan_installed"
        )


if __name__ == "__main__":
    unittest.main()
