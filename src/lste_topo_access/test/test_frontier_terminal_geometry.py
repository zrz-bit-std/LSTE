"""Focused frame/endpoint contract for frontier terminal validation."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

SOURCE = Path(__file__).resolve().parents[1] / "scripts" / "global_frontier_terminal_validation.py"


def load_geometry_contract():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    owner = next(node for node in tree.body if isinstance(node, ast.ClassDef)
                 and node.name == "GlobalFrontierTerminalValidationMixin")
    method = next(node for node in owner.body if isinstance(node, ast.FunctionDef)
                  and node.name == "_terminal_goal_geometry_contract")
    namespace = {"math": __import__("math")}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace["_terminal_goal_geometry_contract"]


class FrontierTerminalGeometryTest(unittest.TestCase):
    geometry = load_geometry_contract()

    def test_authentication_turn_uses_frozen_odom_anchor(self):
        auth = SimpleNamespace(state="turning", turn_odom_xy=(1.25, -2.5))
        self.assertEqual(
            self.geometry("frontier_turn_connector", "/map", (10, 20), None,
                          "turning", auth),
            ("odom", (1.25, -2.5)),
        )

    def test_ordinary_connector_stays_in_map(self):
        auth = SimpleNamespace(state="idle", turn_odom_xy=None)
        self.assertEqual(
            self.geometry("frontier_turn_connector", "/map", (10, 20), None,
                          "", auth),
            ("map", (10.0, 20.0)),
        )

    def test_portal_transition_keeps_frozen_odom_destination(self):
        self.assertEqual(
            self.geometry("portal_transition", "map", (10, 20), (3, 4), "", None),
            ("odom", (3.0, 4.0)),
        )

    def test_missing_or_nonfinite_turn_anchor_is_rejected(self):
        for auth in (SimpleNamespace(state="turning", turn_odom_xy=None),
                     SimpleNamespace(state="turning", turn_odom_xy=(float("nan"), 1))):
            self.assertIsNone(
                self.geometry("frontier_turn_connector", "map", (10, 20), None,
                              "turning", auth)
            )


if __name__ == "__main__":
    unittest.main()
