"""The doorway audit must accept a resolved candidate only on full evidence."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "scripts/tests/targeted_navigation"))

import check_portal_authentication as audit  # noqa: E402


def interior_goal_snapshot(**overrides):
    source = [-18.949408524654533, -1.1952832106902822]
    pose = [-18.638621412337077, -0.8838910918647773]
    snapshot = {
        "failure_reason": "no_reachable_interior_entry_goal",
        "inside_xy": None,
        "source_side_proven": True,
        "return_gate_cleared": False,
        "source_xy": source,
        "events": [
            {"event": "doorway_entry_started", "source_xy": source},
            {"event": "doorway_interior_plan_unavailable",
             "feasible_interior_goal_count": 0, "map_stamp": 1357.529,
             "minimum_depth": 2.35, "navigation_clearance": 0.52},
            {"event": "doorway_failure_settled_at_source",
             "reason": "no_reachable_interior_entry_goal",
             "phase": "entering", "pose_xy": pose, "source_xy": source,
             "signed_depth": -3.8940874254368705, "distance": 0.44},
            {"event": "doorway_authentication_failed",
             "reason": "no_reachable_interior_entry_goal"},
        ],
    }
    snapshot.update(overrides)
    return snapshot


class SettledAtSourceWithoutInteriorGoalTest(unittest.TestCase):
    def test_the_115704_branch_is_evidence_complete(self):
        self.assertTrue(audit.settled_at_source_without_interior_goal(
            interior_goal_snapshot()))

    def test_entering_the_doorway_requires_a_return(self):
        snapshot = interior_goal_snapshot()
        snapshot["events"] = snapshot["events"] + [
            {"event": "vehicle_fully_inside"}]
        self.assertFalse(audit.settled_at_source_without_interior_goal(snapshot))

    def test_inside_evidence_requires_a_return(self):
        self.assertFalse(audit.settled_at_source_without_interior_goal(
            interior_goal_snapshot(inside_xy=[-15.0, -3.0])))

    def test_a_reachable_interior_goal_is_not_a_settle(self):
        snapshot = interior_goal_snapshot()
        snapshot["events"][1]["feasible_interior_goal_count"] = 1
        self.assertFalse(audit.settled_at_source_without_interior_goal(snapshot))

    def test_settling_away_from_the_source_is_not_a_pass(self):
        snapshot = interior_goal_snapshot()
        snapshot["events"][2]["pose_xy"] = [-16.5, -3.0]
        snapshot["events"][2]["distance"] = 3.4
        self.assertFalse(audit.settled_at_source_without_interior_goal(snapshot))

    def test_missing_source_side_proof_is_not_a_pass(self):
        self.assertFalse(audit.settled_at_source_without_interior_goal(
            interior_goal_snapshot(source_side_proven=False)))

    def test_a_different_failure_reason_is_not_accepted(self):
        self.assertFalse(audit.settled_at_source_without_interior_goal(
            interior_goal_snapshot(failure_reason="authentication_physical_stall:entering")))


class SamePlaceRejectionEvidenceTest(unittest.TestCase):
    def evidence(self, reason, spatial, **overrides):
        flags = dict(success_sequence=True, target_locked=True,
                     returned_failure=True, lineage_preserved=True)
        flags.update(overrides)
        return audit.same_place_rejection_evidence(
            reason, spatial,
            {"source_side_proven": True, "return_gate_cleared": True},
            **flags)

    def test_the_124741_entry_side_branch_is_evidence_complete(self):
        self.assertTrue(self.evidence(
            "visited_entry_side_of_current_place",
            {"source_label": 1, "destination_label": 16, "entry_label": 16}))

    def test_the_classic_same_place_rejection_still_passes(self):
        self.assertTrue(self.evidence(
            "known_free_bypass_within_current_place",
            {"source_label": 4, "destination_label": 4, "entry_label": 4}))

    def test_entry_side_needs_the_entry_label(self):
        self.assertFalse(self.evidence(
            "visited_entry_side_of_current_place",
            {"source_label": 1, "destination_label": 16, "entry_label": 3}))

    def test_a_candidate_that_merges_with_its_own_source_is_not_entry_side(self):
        self.assertFalse(self.evidence(
            "visited_entry_side_of_current_place",
            {"source_label": 16, "destination_label": 16, "entry_label": 16}))

    def test_without_a_return_the_cycle_is_not_proven(self):
        self.assertFalse(self.evidence(
            "visited_entry_side_of_current_place",
            {"source_label": 1, "destination_label": 16, "entry_label": 16},
            returned_failure=False))

    def test_without_the_interior_cycle_it_is_not_proven(self):
        self.assertFalse(self.evidence(
            "visited_entry_side_of_current_place",
            {"source_label": 1, "destination_label": 16, "entry_label": 16},
            success_sequence=False))

    def test_an_unknown_reason_is_never_accepted(self):
        self.assertFalse(self.evidence(
            "something_else",
            {"source_label": 1, "destination_label": 16, "entry_label": 16}))

    def test_missing_spatial_evidence_is_never_accepted(self):
        self.assertFalse(self.evidence(
            "visited_entry_side_of_current_place", None))


class StableUnstartedCandidateTest(unittest.TestCase):
    def branch(self, **overrides):
        record = {
            "stable": True,
            "passability": "open",
            "physical_gate": [0.1, 1.0],
            "connection": None,
            "state": "pending",
            "authentication_state": "pending",
        }
        record.update(overrides)
        return record

    def test_an_undecided_stable_candidate_is_counted(self):
        self.assertTrue(audit.stable_unstarted_candidate(self.branch()))

    def test_a_branch_never_touched_is_still_a_candidate(self):
        self.assertTrue(audit.stable_unstarted_candidate(
            self.branch(authentication_state=None)))

    def test_an_analytically_rejected_branch_is_not_a_candidate(self):
        # 20261103_141641 node 8 branches 26/27/29.
        self.assertFalse(audit.stable_unstarted_candidate(
            self.branch(authentication_state="failed",
                        state="authentication_failed")))

    def test_a_withdrawn_section_is_not_a_candidate(self):
        self.assertFalse(audit.stable_unstarted_candidate(
            self.branch(authentication_state="waiting_passable_section")))

    def test_a_completed_branch_is_not_a_candidate(self):
        self.assertFalse(audit.stable_unstarted_candidate(
            self.branch(authentication_state="completed")))

    def test_a_blocked_section_is_not_a_candidate(self):
        self.assertFalse(audit.stable_unstarted_candidate(
            self.branch(passability="blocked")))

    def test_a_superseded_or_return_branch_is_not_a_candidate(self):
        self.assertFalse(audit.stable_unstarted_candidate(
            self.branch(state="superseded")))
        self.assertFalse(audit.stable_unstarted_candidate(
            self.branch(connection="return")))

    def test_a_branch_without_a_physical_gate_is_not_a_candidate(self):
        self.assertFalse(audit.stable_unstarted_candidate(
            self.branch(physical_gate=None)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
