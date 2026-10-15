#!/usr/bin/env python3
"""Regression for preserving TEB's explicit turn-entry angular sample."""

import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "scripts/lste_cmd_vel_mux_node.py"


def load_filter_method():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    owner = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    method = next(
        node
        for node in owner.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "filter_teb_angular"
    )
    namespace = {
        "rospy": SimpleNamespace(loginfo_throttle=lambda *_args: None),
    }
    exec(
        compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"),
        namespace,
    )
    return namespace["filter_teb_angular"]


FILTER = load_filter_method()


class MuxFilterFixture:
    filter_teb_angular = FILTER

    def __init__(self):
        self.teb_angular_deadband = 0.02
        self.teb_angular_sign_switch_threshold = 0.12
        self.teb_last_angular_sign = 1
        self.teb_last_output_angular = 0.08
        self.teb_micro_hold_cycles = 0
        self.teb_micro_hold_limit = 4
        self.teb_filter_events = 0
        self.teb_last_filter_reason = "none"


class CmdVelMuxTurnEntryTest(unittest.TestCase):
    @staticmethod
    def command(angular):
        return SimpleNamespace(angular=SimpleNamespace(z=angular))

    def test_normal_micro_reversal_is_still_suppressed(self):
        fixture = MuxFilterFixture()
        result = fixture.filter_teb_angular(self.command(-0.05))
        self.assertAlmostEqual(result.angular.z, 0.08)
        self.assertEqual(fixture.teb_last_filter_reason, "angular_micro_reversal_held")

    def test_turn_entry_reversal_reaches_actuator_boundary(self):
        fixture = MuxFilterFixture()
        result = fixture.filter_teb_angular(
            self.command(-0.05), allow_turn_entry_reversal=True
        )
        self.assertAlmostEqual(result.angular.z, -0.05)
        self.assertEqual(fixture.teb_last_filter_reason, "none")
        self.assertEqual(fixture.teb_last_angular_sign, -1)


if __name__ == "__main__":
    unittest.main()
