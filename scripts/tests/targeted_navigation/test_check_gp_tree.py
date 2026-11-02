import json
import tempfile
import unittest
from pathlib import Path
from check_gp_tree import check


class GpTreeAuditTest(unittest.TestCase):
    def fixture(self):
        branches = [
            dict(id=20, node_id=9, heading=3.14159, stable=True, state="authenticated",
                 passability="open", map_geometry={"width": 2.7, "bounds": [[-1, 0], [-1, 2]]},
                 canonical_portal_id=3, room_place_id=4),
            dict(id=21, node_id=9, heading=-0.1, stable=True, state="authentication_failed",
                 authentication_state="failed", authentication_failure_reason="known_free_bypass_within_current_place",
                 room_invalidated=True, passability="open",
                 map_geometry={"width": 2.8, "bounds": [[1, -1], [1, 1]]}),
            dict(id=22, node_id=9, heading=1.5708, stable=True, state="authenticated",
                 passability="open", map_geometry={"width": 2.9, "bounds": [[0, 1], [2, 1]]},
                 canonical_portal_id=4, room_place_id=5),
        ]
        prior = dict(nodes=[dict(id=9, junction_confirmed=True, physical_xy=[0, 0])],
                     branches=[dict(id=99, node_id=9, state="pending", stable=False)], spaces=[])
        latest = dict(nodes=[dict(id=9, junction_confirmed=True, physical_xy=[0, 0])],
                      branches=branches, spaces=[])
        branch_evidence = dict(branch_id=21, node_id=9,
            failure_reason="known_free_bypass_within_current_place",
            source_side_proven=True, return_gate_cleared=True,
            events=[dict(event=name) for name in ("doorway_entry_started", "vehicle_fully_inside",
                "inside_frontier_observed", "room_facing_turn_complete", "returned_to_junction_node",
                "portal_candidate_rejected")])
        branches[1]["authentication_evidence"] = branch_evidence
        branches[1]["authentication_failure_reason"] = "known_free_bypass_within_current_place"
        rejection = dict(frontier_event="portal_candidate_rejected", reason="known_free_bypass_within_current_place",
            spatial_evidence=dict(source_label=7, destination_label=7, entry_label=7),
            portal_authentication=dict(branches=[branch_evidence]))
        portals = dict(records=[dict(id=3, state="crossed", source_place_id=3,
                                     destination_place_id=4, crossing_count=1),
                               dict(id=4, state="crossed", source_place_id=3,
                                     destination_place_id=5, crossing_count=1)])
        return [dict(frontier_event="gp_branch_tree", tree=prior),
                dict(frontier_event="gp_branch_tree", tree=latest, portal_hypotheses=portals),
                rejection]

    def run_audit(self, events):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "metrics.log"
            path.write_text("".join("event=global_frontier_event data=" + json.dumps(e) + "\n"
                                    for e in events))
            return check(path, [0, 1.5708, 3.14159], [0, 0], 2.0)

    def test_safe_same_place_branch_counts_as_resolved_without_child_place(self):
        result = self.run_audit(self.fixture())
        self.assertTrue(result["passed"], result["checks"])
        self.assertEqual(result["details"]["resolved_same_place_branches"], [21])
        self.assertTrue(result["checks"]["same_place_continuations_have_no_child_place"])

    def test_missing_positive_same_place_evidence_does_not_accept_branch(self):
        events = self.fixture()
        events[-1]["spatial_evidence"]["destination_label"] = 0
        result = self.run_audit(events)
        self.assertFalse(result["checks"]["expected_directions_present"])


if __name__ == "__main__":
    unittest.main()
