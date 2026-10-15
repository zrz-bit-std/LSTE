#!/usr/bin/env python3
"""Prevent the isolated SA-PPO controller test from becoming a monolith."""

import ast
from pathlib import Path
import unittest


TEST_DIR = Path(__file__).resolve().parent


class SappoRunnerLayoutTest(unittest.TestCase):
    def test_public_entrypoint_only_delegates_to_runner(self):
        path = TEST_DIR / "sappo_test.py"
        source = path.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(path))

        self.assertLessEqual(len(source.splitlines()), 30)
        self.assertIn("from sappo_runner import main", source)
        self.assertFalse(
            [node for node in tree.body if isinstance(node, ast.ClassDef)],
            "controller logic belongs in focused sibling modules",
        )

    def test_safety_modules_have_explicit_responsibilities(self):
        expected = {
            "sappo_runner.py": 300,
            "sappo_safety_primitives.py": 500,
            "sappo_mppi_guard.py": 350,
            "sappo_grid_guard.py": 700,
            "sappo_grid_boundary.py": 700,
        }
        for filename, maximum_lines in expected.items():
            path = TEST_DIR / filename
            self.assertTrue(path.is_file(), filename)
            source = path.read_text(encoding="utf-8")
            ast.parse(source, filename=str(path))
            self.assertLessEqual(len(source.splitlines()), maximum_lines, filename)


if __name__ == "__main__":
    unittest.main()
