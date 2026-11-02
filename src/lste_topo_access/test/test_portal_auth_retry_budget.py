#!/usr/bin/env python3
"""Doorway authentication retries have a short, bounded total stage budget."""

import ast
from pathlib import Path
import unittest
import sys
import math


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
SOURCE = SCRIPTS / "global_frontier_execution_resolution.py"
sys.path.insert(0, str(SCRIPTS))
from global_frontier_portal_authentication import DoorwayAuthentication

TREE = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
FUNCTION = next(
    node for node in TREE.body
    if isinstance(node, ast.FunctionDef)
    and node.name == "portal_authentication_retry_budget"
)
_NAMESPACE = {"PORTAL_AUTH_RETRY_MIN_SECONDS": 8.0, "PORTAL_AUTH_RETRY_MAX_SECONDS": 12.0}
exec(compile(ast.Module(body=[FUNCTION], type_ignores=[]), str(SOURCE), "exec"), _NAMESPACE)
retry_budget = _NAMESPACE["portal_authentication_retry_budget"]


class PortalAuthenticationRetryBudgetTest(unittest.TestCase):
    def test_budget_is_short_and_capped_for_lab_route_timeouts(self):
        self.assertEqual(retry_budget(45.0), 12.0)
        self.assertEqual(retry_budget(18.0), 9.0)
        self.assertEqual(retry_budget(4.0), 8.0)
        self.assertEqual(retry_budget(1000.0), 12.0)

    def test_entry_turn_and_return_all_use_the_shared_budget(self):
        owner = next(
            node for node in TREE.body
            if isinstance(node, ast.ClassDef)
            and node.name == "GlobalFrontierExecutionResolutionMixin"
        )
        methods = [
            node for node in owner.body
            if isinstance(node, ast.FunctionDef)
            and node.name in {
                "resolve_active_route_terminal_or_failure",
                "deactivate_failed_active_frontier",
            }
        ]
        budget_calls = [
            node for method in methods
            for node in ast.walk(method)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "portal_authentication_retry_budget"
        ]
        self.assertEqual(len(budget_calls), 4)


class PortalPhysicalProgressTest(unittest.TestCase):
    def doorway(self):
        auth = DoorwayAuthentication(1, 1, (0, 0), (1, 0))
        auth.begin((-1, 0))
        return auth

    def test_stationary_entry_expires_despite_controller_success_reissues(self):
        auth = self.doorway()
        self.assertFalse(auth.physical_progress_expired((-1, 0), 0, 0, 12))
        for now in (2, 4, 6, 8, 10):
            # New controller goals do not change this branch-owned clock.
            self.assertFalse(auth.physical_progress_expired((-1, 0), math.pi, now, 12))
        self.assertTrue(auth.physical_progress_expired((-1, 0), math.pi, 12, 12))
        self.assertEqual(auth.events[-1]['event'], 'doorway_physical_progress_expired')

    def test_actual_translation_keeps_a_long_healthy_approach_alive(self):
        auth = self.doorway()
        for now, x in ((0, -1), (10, -.8), (20, -.6), (30, -.4)):
            self.assertFalse(auth.physical_progress_expired((x, 0), 0, now, 12))

    def test_locked_turn_progress_refreshes_clock_but_turning_away_does_not(self):
        auth = self.doorway()
        auth.state = 'turning'
        auth.target_yaw = math.pi / 2
        for now, yaw in ((0, 0), (10, .5), (20, 1.0)):
            self.assertFalse(auth.physical_progress_expired((0, 0), yaw, now, 12))
        self.assertTrue(auth.physical_progress_expired((0, 0), .5, 32, 12))

    def test_near_source_failure_still_requires_crossing_back_from_destination_side(self):
        auth=DoorwayAuthentication(1,1,(0.,0.),(1.,0.))
        auth.begin((-.2,0.))
        auth.request_failure_return('authentication_physical_stall:entering',(.2,0.))
        self.assertEqual(auth.state,'returning')
        self.assertFalse(auth.update_return((.2,0.)))
        self.assertTrue(auth.update_return((-.2,0.)))
        self.assertEqual(auth.state,'failed')
        self.assertEqual(auth.events[-1]['event'],'returned_to_junction_node_after_failure')

    def test_failure_return_gets_its_own_progress_clock(self):
        auth = self.doorway()
        auth.physical_progress_expired((1, 0), 0, 0, 12)
        self.assertTrue(auth.physical_progress_expired((1, 0), 0, 12, 12))
        auth.request_failure_return('authentication_physical_stall:entering', (1, 0))
        self.assertEqual(auth.state, 'returning')
        self.assertFalse(auth.physical_progress_expired((1, 0), 0, 12, 12))
        self.assertFalse(auth.physical_progress_expired((.7, 0), 0, 22, 12))


if __name__ == "__main__":
    unittest.main()
