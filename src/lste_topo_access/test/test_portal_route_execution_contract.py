"""Guard the non-segmented execution contract for portal edges."""

import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
SOURCE = SCRIPTS / "global_frontier_route_command_resolution.py"


def load_method(name):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
    owner = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierRouteCommandResolutionMixin"
    )
    method = next(
        node for node in owner.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )
    namespace = {"math": math}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace[name]


SHOULD_HOLD = load_method("should_hold_active_waypoint")


class PortalRouteExecutionContractTest(unittest.TestCase):
    def test_portal_keeps_one_endpoint_action_even_in_legacy_segment_mode(self):
        explorer = SimpleNamespace(
            active_last_waypoint_map=(8.0, 4.0),
            active_route_kind="portal_transition",
            mission_endpoint_only=False,
            turn_connector_released=True,
            waypoint_release_radius=0.4,
        )

        held, distance, turn_pending = SHOULD_HOLD(explorer, (8.0, 4.0))

        self.assertTrue(held)
        self.assertAlmostEqual(distance, 0.0)
        self.assertFalse(turn_pending)


if __name__ == "__main__":
    unittest.main()
