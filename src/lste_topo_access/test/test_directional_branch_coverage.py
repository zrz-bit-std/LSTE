"""Focused tests for persistent directional branch coverage."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_directional_branch_coverage import (  # noqa: E402
    BRANCH_COMPLETED,
    BRANCH_OPEN,
    BRANCH_TRANSIT,
    DirectionalBranchCoverage,
    DirectionalBranchKey,
    EVIDENCE_DESTINATION_VIEW,
    EVIDENCE_CROSSING_VERIFIED,
)


class DirectionalBranchCoverageTest(unittest.TestCase):
    def setUp(self):
        self.ledger = DirectionalBranchCoverage()
        self.branch = DirectionalBranchKey(2, "portal-7")

    def test_slam_projection_change_keeps_branch_and_work_item_identity(self):
        first = self.ledger.observe_frontier(
            self.branch,
            (1.0, 2.0),
            direction_xy=(1.0, 0.0),
            map_epoch=10,
            now=1.0,
        )
        second = self.ledger.observe_frontier(
            self.branch,
            (24.0, -8.0),
            direction_xy=(0.98, 0.04),
            map_epoch=11,
            now=2.0,
        )

        self.assertEqual(first.key, second.key)
        self.assertEqual(first.work_item.id, second.work_item.id)
        self.assertEqual(second.state, BRANCH_OPEN)
        self.assertEqual(second.observation_count, 2)
        self.assertEqual(second.frontier_xy, (24.0, -8.0))
        self.assertEqual(len(self.ledger.pending_work_items()), 1)

    def test_completed_branch_never_recreates_work_item_after_map_updates(self):
        self.ledger.observe_frontier(self.branch, (1.0, 2.0), map_epoch=1)
        completed = self.ledger.complete_branch(
            self.branch,
            evidence_kind=EVIDENCE_DESTINATION_VIEW,
            event_id="view-1",
        )
        self.assertEqual(completed.state, BRANCH_COMPLETED)
        self.assertIsNone(completed.work_item)
        stable_id = completed.stable_work_item_id

        refreshed = self.ledger.observe_frontier(
            self.branch,
            (-40.0, 17.0),
            map_epoch=2,
        )
        self.assertEqual(refreshed.state, BRANCH_COMPLETED)
        self.assertIsNone(self.ledger.work_item_for(self.branch))
        self.assertEqual(refreshed.stable_work_item_id, stable_id)
        self.assertEqual(self.ledger.pending_work_items(), ())

    def test_transit_is_routeable_but_has_no_observation_work_item(self):
        self.ledger.observe_frontier(self.branch, (3.0, 4.0))
        transit = self.ledger.mark_transit(
            self.branch,
            8,
            event_id="crossing-1",
        )

        self.assertEqual(transit.state, BRANCH_TRANSIT)
        self.assertTrue(transit.transit)
        self.assertTrue(self.ledger.allows_transit(self.branch))
        self.assertEqual(transit.destination_place_id, 8)
        self.assertIsNone(transit.work_item)
        self.assertEqual(
            [fact.kind for fact in transit.evidence],
            [EVIDENCE_CROSSING_VERIFIED],
        )

        # Replaying a later positive completion fact must not turn a transit
        # edge back into an observation task.
        replayed = self.ledger.complete_branch(
            self.branch,
            evidence_kind=EVIDENCE_DESTINATION_VIEW,
            event_id="view-after-crossing",
        )
        self.assertEqual(replayed.state, BRANCH_TRANSIT)
        self.assertEqual(replayed.destination_place_id, 8)
        self.assertIsNone(replayed.work_item)

    def test_only_contradictory_evidence_reopens_same_work_item_lineage(self):
        self.ledger.observe_frontier(self.branch, (1.0, 2.0))
        original = self.ledger.work_item_for(self.branch)
        completed = self.ledger.complete_branch(
            self.branch,
            event_id="coverage-1",
        )

        # Ordinary evidence and a changed SLAM projection do not reopen it.
        unchanged = self.ledger.record_evidence(
            self.branch,
            EVIDENCE_DESTINATION_VIEW,
            event_id="view-1",
        )
        self.ledger.observe_frontier(self.branch, (100.0, 100.0), map_epoch=9)
        self.assertEqual(unchanged.state, BRANCH_COMPLETED)
        self.assertIsNone(self.ledger.work_item_for(self.branch))

        reopened = self.ledger.record_evidence(
            self.branch,
            "doorway_contradiction",
            contradictory=True,
            event_id="contradiction-1",
        )
        self.assertEqual(reopened.state, BRANCH_OPEN)
        self.assertEqual(reopened.reopen_count, 1)
        self.assertEqual(reopened.work_item.id, original.id)
        self.assertEqual(reopened.work_item.generation, original.generation + 1)
        self.assertEqual(reopened.stable_work_item_id, completed.stable_work_item_id)
        self.assertEqual(len(self.ledger.pending_work_items()), 1)

    def test_replayed_contradiction_event_is_idempotent(self):
        self.ledger.observe_frontier(self.branch, (1.0, 2.0))
        self.ledger.complete_branch(self.branch, event_id="coverage-1")
        first = self.ledger.record_evidence(
            self.branch,
            "contradiction",
            contradictory=True,
            event_id="contradiction-1",
        )
        second = self.ledger.record_evidence(
            self.branch,
            "contradiction",
            contradictory=True,
            event_id="contradiction-1",
            now=20.0,
        )
        self.assertEqual(second, first)
        self.assertEqual(second.reopen_count, 1)
        self.assertEqual(second.evidence_revision, first.evidence_revision)

    def test_stale_crossing_event_cannot_reenable_transit_after_contradiction(self):
        self.ledger.observe_frontier(self.branch, (1.0, 2.0))
        self.ledger.mark_transit(self.branch, 8, event_id="crossing-1")
        self.ledger.record_evidence(
            self.branch,
            "crossing_invalidated",
            contradictory=True,
            event_id="contradiction-1",
        )

        stale = self.ledger.mark_transit(
            self.branch,
            8,
            event_id="crossing-1",
        )
        self.assertEqual(stale.state, BRANCH_OPEN)
        self.assertFalse(stale.transit)
        self.assertIsNotNone(stale.work_item)

    def test_distinct_physical_branch_ids_do_not_merge_by_coordinate(self):
        other = DirectionalBranchKey(2, "portal-8")
        left = self.ledger.observe_frontier(self.branch, (1.0, 2.0))
        right = self.ledger.observe_frontier(other, (1.0, 2.0))

        self.assertNotEqual(left.key, right.key)
        self.assertNotEqual(left.work_item.id, right.work_item.id)
        self.assertEqual(len(self.ledger.pending_work_items(2)), 2)


if __name__ == "__main__":
    unittest.main()
