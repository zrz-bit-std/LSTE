"""Pure integration tests for the frontier completion/recovery boundary."""

import ast
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_completion_gate import evaluate_completion  # noqa: E402
from global_frontier_completion_recovery import (  # noqa: E402
    GraphCompletionStateMachine,
)


def load_report_method():
    source_path = SCRIPTS / "global_frontier_planning_snapshot.py"
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    owner = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "GlobalFrontierPlanningSnapshotMixin"
    )
    method = next(
        node for node in owner.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_report_no_frontier_selection"
    )
    namespace = {
        "evaluate_completion": evaluate_completion,
        "rospy": SimpleNamespace(
            logwarn=lambda *_args: None,
            logwarn_throttle=lambda *_args: None,
        ),
    }
    exec(
        compile(
            ast.Module(body=[method], type_ignores=[]),
            str(source_path),
            "exec",
        ),
        namespace,
    )
    return namespace["_report_no_frontier_selection"]


REPORT_NO_SELECTION = load_report_method()


class RegionMemory:
    regions = [{"id": 1}]


class CountLedger:
    def __init__(self, count):
        self.count = count

    def unresolved_count(self, _place_id):
        return self.count


class EmptyHypotheses:
    def unbound_for_source(self, _place_id):
        return []


class EmptyTargetWork:
    def pending_place_ids(self):
        return ()


class CompletionGateExplorer:
    _report_no_frontier_selection = REPORT_NO_SELECTION

    def __init__(self):
        self.place_memory_enabled = True
        self.current_physical_place_id = 1
        self.region_memory = RegionMemory()
        self.place_work_items = CountLedger(1)
        self.portal_probe_ledger = CountLedger(0)
        self.portal_hypothesis_ledger = EmptyHypotheses()
        self.target_observation_work = EmptyTargetWork()
        self.frontier_validation_budget_exhausted = False
        self.frontier_validation_pending = False
        self.last_place_graph_hop_skips = 0
        self.last_portal_sources = 0
        self.last_portal_missing_transition_goals = 0
        self.last_portal_unbound_reprojection_skips = 0
        self.last_work_item_failed_viewpoint_skips = 0
        self.last_portal_probe_viewpoint_rejections = 0
        self.last_graph_policy_rejections = 0
        self.pending_local_egress = None
        self.pending_portal_retry = None
        self.graph_completion_state = GraphCompletionStateMachine()
        self.last_completion_gate_signature = None
        self.frontier_exhausted = False
        self.place_graph_waiting_for_portal = False
        self.last_observed_place_reentry_skips = 0
        self.clearance = 0.77
        self.frontier_clearance = 0.62
        self.status = []

    def publish_status(self, event, **fields):
        self.status.append((event, fields))


class CompletionGateIntegrationTest(unittest.TestCase):
    def test_unresolved_no_viewpoint_requests_recovery_then_blocks(self):
        explorer = CompletionGateExplorer()

        explorer._report_no_frontier_selection()
        self.assertEqual(explorer.status[-1][0], "graph_recovery_requested")
        self.assertFalse(explorer.frontier_exhausted)

        explorer._report_no_frontier_selection()
        self.assertEqual(explorer.status[-1][0], "graph_exploration_blocked")
        self.assertFalse(explorer.frontier_exhausted)
        self.assertNotIn(
            "frontier_exhausted",
            [event for event, _fields in explorer.status],
        )

    def test_settled_graph_still_uses_frontier_exhausted_terminal(self):
        explorer = CompletionGateExplorer()
        explorer.place_work_items = CountLedger(0)

        explorer._report_no_frontier_selection()

        self.assertEqual(explorer.status[-1][0], "frontier_exhausted")
        self.assertTrue(explorer.frontier_exhausted)

    def test_remote_structural_boundary_is_not_claimed_complete(self):
        explorer = CompletionGateExplorer()
        explorer.place_work_items = CountLedger(0)
        explorer.last_place_graph_hop_skips = 3

        explorer._report_no_frontier_selection()

        self.assertEqual(explorer.status[-1][0], "graph_recovery_requested")
        self.assertFalse(explorer.frontier_exhausted)

    def test_rejected_portal_sources_do_not_count_as_graph_transit(self):
        explorer = CompletionGateExplorer()
        explorer.place_work_items = CountLedger(0)
        explorer.last_place_graph_hop_skips = 1
        explorer.last_portal_sources = 3
        explorer.last_portal_candidates_seen = 3
        explorer.last_portal_hard_rejected = 3
        explorer.last_portal_executable_candidates = 0

        explorer._report_no_frontier_selection()

        event, fields = explorer.status[-1]
        self.assertEqual(event, "graph_recovery_requested")
        self.assertFalse(fields["graph_transit_available"])
        self.assertEqual(fields["portal_executable_candidates"], 0)


if __name__ == "__main__":
    unittest.main()
