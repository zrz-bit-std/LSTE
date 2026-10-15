#!/usr/bin/env python3
"""Regression tests for distinct left/right semantic context detections."""

import ast
import types
import unittest
from pathlib import Path


def load_has_ctx_pair():
    source = Path(__file__).resolve().parents[1] / "scripts/lste_state_node.py"
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    manager = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "StateNode"
    )
    method = next(
        node for node in manager.body
        if isinstance(node, ast.FunctionDef) and node.name == "has_ctx_pair"
    )
    namespace = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)
    return namespace["has_ctx_pair"]


HAS_CTX_PAIR = load_has_ctx_pair()


def detection(label):
    return types.SimpleNamespace(label=label)


class StateContextPairTest(unittest.TestCase):
    def make_node(self, labels):
        return types.SimpleNamespace(
            latest_task_msg=types.SimpleNamespace(
                ctx_left="monitor", ctx_right="monitor"
            ),
            latest_dets=types.SimpleNamespace(
                env_dets=[detection(label) for label in labels], target_dets=[]
            ),
        )

    def test_one_monitor_cannot_fill_both_context_slots(self):
        self.assertFalse(HAS_CTX_PAIR(self.make_node(["monitor"])))

    def test_two_monitor_detections_form_a_context_pair(self):
        self.assertTrue(HAS_CTX_PAIR(self.make_node(["monitor", "monitor"])))


if __name__ == "__main__":
    unittest.main()
