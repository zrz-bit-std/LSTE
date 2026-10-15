#!/usr/bin/env python3
"""Regression tests for terminal-time observation-place replanning."""

import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
SOURCE = SCRIPTS_DIR / "global_frontier_terminal_prefetch.py"


def load_terminal_prefetch_policy():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    owner = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierTerminalPrefetchMixin"
    )
    method = next(
        node for node in owner.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "promote_prefetched_terminal"
    )
    namespace = {"rospy": SimpleNamespace(loginfo=lambda *_args: None)}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace["promote_prefetched_terminal"]


class TerminalPolicyExplorer:
    promote_prefetched_terminal = load_terminal_prefetch_policy()

    def __init__(self):
        self.prefetched_frontier = (4, 7, 12.25, 6.75)
        self.map_msg = SimpleNamespace()
        self.status = []
        self.cleared = 0

    def publish_status(self, event, **fields):
        self.status.append((event, fields))

    def clear_prefetched_frontier(self):
        self.cleared += 1
        self.prefetched_frontier = None


class FrontierTerminalReplanTest(unittest.TestCase):
    def test_terminal_discards_old_branch_before_successor_selection(self):
        explorer = TerminalPolicyExplorer()

        self.assertFalse(explorer.promote_prefetched_terminal(object()))
        self.assertEqual(explorer.cleared, 1)
        self.assertEqual(
            explorer.status,
            [
                (
                    "frontier_prefetch_discarded",
                    {
                        "reason": "terminal_requires_fresh_observation_place_decision",
                        "pending_goal": [12.25, 6.75],
                    },
                )
            ],
        )


if __name__ == "__main__":
    unittest.main()
