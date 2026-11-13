"""Turn-into-room needs evidence; a tie never buys a U-turn.

env04 has two junctions that must behave differently:

* mid junction (~11.8, -6.7), north-to-south: the west doorway is the hydrant
  room (score <= 0.0 in 16 runs) and sits ~90 deg to the right.  Expectation is
  to keep going south instead of turning into it.
* lower junction (~12.1, -11.2): the east doorway is the lab (chairs/desks, the
  yellow cup lives there) at 0.182-0.731, and ends up ~178 deg behind the robot;
  it must still be entered, while the west hydrant room (-0.69) must not.

The rules pinned here are:

* straight ahead (<= soft limit) needs no evidence;
* turning beyond the soft limit into a room needs at least one positive
  environment term (0.15 = 1 of the 4 room terms = 0.182);
* a >115 deg turn additionally needs the same score *and* a 0.40 margin over
  the runner-up, so ties and "less negative" rooms never buy a U-turn.
"""
from pathlib import Path
import math
import sys
import unittest
from types import SimpleNamespace

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_portal_selection import (  # noqa: E402
    GlobalFrontierPortalSelectionMixin,
)


class TurnEvidenceTest(unittest.TestCase):
    """``portal_turn_needs_positive_room`` on the observed envelope."""

    def setUp(self):
        self.host = GlobalFrontierPortalSelectionMixin()
        self.host.room_turn_soft_limit_deg = 60.0
        self.host.heading_semantic_override_min_score = 0.15

    def test_side_room_without_positive_evidence_is_skipped(self):
        # mid junction: west room is -0.69 / 0.0 and ~90 deg to the right
        self.assertTrue(self.host.portal_turn_needs_positive_room(
            math.radians(90.0), -0.69))
        self.assertTrue(self.host.portal_turn_needs_positive_room(
            math.radians(90.0), 0.0))
        self.assertTrue(self.host.portal_turn_needs_positive_room(
            math.radians(90.0), None))

    def test_side_room_with_positive_evidence_may_be_entered(self):
        self.assertFalse(self.host.portal_turn_needs_positive_room(
            math.radians(90.0), 0.182))

    def test_straight_ahead_needs_no_evidence(self):
        # mid junction south arm: straight on, negative score is fine
        self.assertFalse(self.host.portal_turn_needs_positive_room(
            math.radians(10.0), -0.69))
        self.assertFalse(self.host.portal_turn_needs_positive_room(
            math.radians(60.0), -1.0))

    def test_behind_room_is_judged_by_the_same_evidence_rule(self):
        self.assertFalse(self.host.portal_turn_needs_positive_room(
            math.radians(178.0), 0.182))
        self.assertTrue(self.host.portal_turn_needs_positive_room(
            math.radians(178.0), -0.69))

    def test_soft_limit_is_configurable(self):
        self.host.room_turn_soft_limit_deg = 30.0
        self.assertTrue(self.host.portal_turn_needs_positive_room(
            math.radians(40.0), 0.0))
        self.assertFalse(self.host.portal_turn_needs_positive_room(
            math.radians(20.0), 0.0))

    def test_missing_delta_never_blocks(self):
        self.assertFalse(self.host.portal_turn_needs_positive_room(None, -0.69))


class MarginOverrideTest(unittest.TestCase):
    """``_portal_semantic_heading_override`` needs score AND margin."""

    def _host(self, score, suppressed=False):
        host = GlobalFrontierPortalSelectionMixin()
        host.heading_semantic_override_enabled = True
        host.task_semantic_value_enabled = True
        host.heading_semantic_override_max_attempts = 2
        host.heading_semantic_override_min_score = 0.15
        host.heading_semantic_override_min_margin = 0.40
        host.heading_semantic_override_hold_seconds = 8.0
        host.events = []
        host.publish_status = lambda name, **kw: host.events.append((name, kw))
        record = {
            "id": 7, "destination_place_id": 6,
            "semantic_score": score, "heading_override_count": 0,
            "exploration_suppressed": suppressed,
        }
        host.portal_hypothesis_ledger = SimpleNamespace(
            get=lambda portal_id: record)
        return host, SimpleNamespace(hypothesis_id=7)

    def test_clear_winner_is_authorized(self):
        host, source = self._host(0.182)
        result = host._portal_semantic_heading_override(
            source, math.radians(178.0), 100.0, runner_up_score=-0.69)
        self.assertIsNotNone(result)
        names = [name for name, _fields in host.events]
        self.assertIn("portal_heading_margin", names)

    def test_tie_is_refused(self):
        host, source = self._host(0.182)
        self.assertIsNone(host._portal_semantic_heading_override(
            source, math.radians(178.0), 100.0, runner_up_score=0.0))

    def test_weak_score_is_refused(self):
        host, source = self._host(0.0)
        self.assertIsNone(host._portal_semantic_heading_override(
            source, math.radians(178.0), 100.0, runner_up_score=-0.69))

    def test_single_candidate_uses_the_floor_only(self):
        host, source = self._host(0.182)
        self.assertIsNotNone(host._portal_semantic_heading_override(
            source, math.radians(178.0), 100.0, runner_up_score=None))


class RoomScoreRankingTest(unittest.TestCase):
    def test_ranking_skips_unbound_and_suppressed_rooms(self):
        records = [
            {"id": 1, "source_place_id": 5, "destination_place_id": 9,
             "semantic_score": 0.182},
            {"id": 2, "source_place_id": 5, "destination_place_id": 10,
             "semantic_score": -0.69},
            {"id": 3, "source_place_id": 5, "destination_place_id": None,
             "semantic_score": 0.731},
            {"id": 4, "source_place_id": 5, "destination_place_id": 11,
             "semantic_score": 0.731, "exploration_suppressed": True},
            {"id": 5, "source_place_id": 6, "destination_place_id": 12,
             "semantic_score": 0.971},
        ]
        host = GlobalFrontierPortalSelectionMixin()
        host.portal_hypothesis_ledger = SimpleNamespace(snapshot=lambda: records)
        best, second = host._portal_room_score_ranking(5)
        self.assertAlmostEqual(best, 0.182)
        self.assertAlmostEqual(second, -0.69)


class InsideDwellTest(unittest.TestCase):
    """The 1.0 s inside dwell gates the U-turn, not the sampling."""

    def test_dwell_waits_before_turning(self):
        from gp_branch_frontier import GPBranchFrontierMixin
        from global_frontier_portal_authentication import DoorwayAuthentication

        host = GPBranchFrontierMixin()
        host.room_view_settle_seconds = 1.0
        auth = DoorwayAuthentication(13, 6, (0.0, 0.0), (1.0, 0.0))
        auth.begin((-1.0, 0.0))
        auth.state = "inside"

        auth.inside_since = 10.0
        self.assertFalse(host.portal_authentication_inside_settled(auth, 10.5))
        self.assertFalse(host.portal_authentication_inside_settled(auth, 10.99))
        self.assertTrue(host.portal_authentication_inside_settled(auth, 11.0))

    def test_unstamped_inside_state_is_not_stalled(self):
        from gp_branch_frontier import GPBranchFrontierMixin
        from global_frontier_portal_authentication import DoorwayAuthentication

        host = GPBranchFrontierMixin()
        host.room_view_settle_seconds = 1.0
        auth = DoorwayAuthentication(13, 6, (0.0, 0.0), (1.0, 0.0))
        auth.begin((-1.0, 0.0))
        auth.state = "inside"
        auth.inside_xy = (1.5, 0.0)
        self.assertTrue(host.portal_authentication_inside_settled(auth, 10.0))

    def test_zero_settle_disables_the_dwell(self):
        from gp_branch_frontier import GPBranchFrontierMixin
        from global_frontier_portal_authentication import DoorwayAuthentication

        host = GPBranchFrontierMixin()
        host.room_view_settle_seconds = 0.0
        auth = DoorwayAuthentication(13, 6, (0.0, 0.0), (1.0, 0.0))
        auth.begin((-1.0, 0.0))
        auth.state = "inside"
        self.assertTrue(host.portal_authentication_inside_settled(auth, 10.0))

    def test_retry_clears_the_dwell_stamp(self):
        from global_frontier_portal_authentication import DoorwayAuthentication

        auth = DoorwayAuthentication(13, 6, (0.0, 0.0), (1.0, 0.0))
        auth.begin((-1.0, 0.0))
        auth.state = "inside"
        auth.inside_since = 42.0
        auth.reset_for_retry(50.0, cooldown=5.0)
        self.assertIsNone(auth.inside_since)


if __name__ == "__main__":
    unittest.main()
