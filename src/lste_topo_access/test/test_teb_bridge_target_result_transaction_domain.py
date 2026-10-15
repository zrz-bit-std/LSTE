"""Keep semantic target-result IDs separate from bridge lifecycle IDs."""

import ast
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
SOURCE = SCRIPTS / "teb_goal_bridge_lifecycle.py"


def load_target_result_callback():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    owner = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "TebGoalBridgeLifecycleMixin"
    )
    method = next(
        node
        for node in owner.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "on_persistent_target_plan_result"
    )
    namespace = {
        "EventType": SimpleNamespace(BRIDGE_TARGET_RESULT="BRIDGE_TARGET_RESULT"),
    }
    exec(
        compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"),
        namespace,
    )
    return namespace["on_persistent_target_plan_result"]


ON_TARGET_RESULT = load_target_result_callback()


class TargetResultIngressFixture:
    on_persistent_target_plan_result = ON_TARGET_RESULT

    def __init__(self):
        self.lifecycle_manager = SimpleNamespace(current_transaction_id=9001)
        self.calls = []

    def _enqueue_bridge_event(self, event_type, payload=None, transaction_id=None):
        self.calls.append((event_type, payload, transaction_id))
        return True


class TargetResultTransactionDomainTest(unittest.TestCase):
    def test_semantic_target_id_does_not_become_lifecycle_event_id(self):
        fixture = TargetResultIngressFixture()
        result = SimpleNamespace(data='{"event":"target_plan_installed","transaction_id":3}')

        fixture.on_persistent_target_plan_result(result)

        self.assertEqual(len(fixture.calls), 1)
        self.assertEqual(fixture.calls[0][2], 9001)
        self.assertNotEqual(fixture.calls[0][2], 3)


if __name__ == "__main__":
    unittest.main()
