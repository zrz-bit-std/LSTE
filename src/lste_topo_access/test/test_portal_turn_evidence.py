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


class NegativeRoomRetirementTest(unittest.TestCase):
    """An inspected negative doorway must not keep a Place unfinished."""

    class _WorkItems:
        def __init__(self):
            self.calls = []

        def abandon_portal_entry(self, portal_id, now, reason,
                                 resolution="return_unrecoverable",
                                 item_id=None):
            self.calls.append((portal_id, now, reason, resolution, item_id))
            return {"id": 24 if item_id is None else item_id}

    def _host(self):
        host = GlobalFrontierPortalSelectionMixin()
        host.place_work_items = self._WorkItems()
        host.events = []
        host.publish_status = lambda name, **kw: host.events.append((name, kw))
        return host

    def test_inspected_negative_doorway_is_retired(self):
        host = self._host()
        record = {
            "id": 5, "gp_authentication_state": "completed",
            "semantic_entry_work_item_id": 24,
        }
        retired = host._retire_negative_room_entry(
            record, 5, -0.538, 100.0)
        self.assertEqual(retired, {"id": 24})
        self.assertTrue(record["exploration_suppressed"])
        self.assertEqual(record["policy_skip_reason"], "negative_room_turn")
        self.assertEqual(host.place_work_items.calls, [
            (5, 100.0, "negative_room_turn",
             "policy_skipped_negative_room", 24),
        ])
        names = [name for name, _fields in host.events]
        self.assertIn("portal_entry_policy_skipped", names)

    def test_unknown_inside_stays_deferred_only(self):
        host = self._host()
        record = {"id": 6, "gp_authentication_state": "entering"}
        self.assertIsNone(host._retire_negative_room_entry(
            record, 6, -0.538, 100.0))
        self.assertFalse(record.get("exploration_suppressed", False))
        self.assertEqual(host.place_work_items.calls, [])
        self.assertEqual(host.events, [])


if __name__ == "__main__":
    unittest.main()


class ForwardAlternativesTest(unittest.TestCase):
    """A negative doorway yields only to a straighter or better doorway."""

    def _host(self, deltas, scores=None, rejected=()):
        from types import SimpleNamespace

        scores = scores or {}
        host = GlobalFrontierPortalSelectionMixin()
        host.room_turn_soft_limit_deg = 60.0
        host.heading_semantic_override_min_score = 0.15
        host.completed_radius = 0.5
        host.frontier_is_rejected = lambda x, y, now: (x, y) in rejected
        host.cell_xy = lambda _message, row, _col: (float(row), 0.0)
        host._route_heading_penalty = (
            lambda _request, endpoint, _xy: (math.radians(deltas[endpoint[0]]), 0.0)
        )
        records = {
            key: {"semantic_score": score}
            for key, score in scores.items()
        }
        host.portal_hypothesis_ledger = SimpleNamespace(
            get=lambda key: records.get(int(key)))
        sources = {
            key: SimpleNamespace(
                hypothesis_id=key, endpoint_row=key, endpoint_col=0,
                identity_row=key, identity_col=0)
            for key in deltas
        }
        request = SimpleNamespace(
            message=None, excluded=None, validation=None, now=1.0,
            steps=[[0.0]],
        )
        return host, request, sources

    def test_corridor_ahead_makes_a_side_room_skippable(self):
        host, request, sources = self._host({1: 10.0, 2: 90.0})
        straight, positive = host._portal_forward_alternatives(
            request, sources)
        self.assertTrue(straight)
        self.assertFalse(positive)

    def test_last_way_forward_has_no_alternative(self):
        # Run 20261113_181729: the only way south was a 74 deg door at -0.538.
        host, request, sources = self._host({2: 74.0}, scores={2: -0.538})
        self.assertEqual(
            host._portal_forward_alternatives(request, sources),
            (False, False))

    def test_positive_room_makes_a_negative_room_skippable(self):
        # Env04 lower T: east lab 0.182 vs west hydrant -0.69, both ~90 deg.
        host, request, sources = self._host(
            {1: 90.0, 2: 90.0}, scores={1: 0.182, 2: -0.69})
        straight, positive = host._portal_forward_alternatives(
            request, sources)
        self.assertFalse(straight)
        self.assertTrue(positive)

    def test_negative_neighbours_are_not_an_alternative(self):
        host, request, sources = self._host(
            {1: 90.0, 2: 90.0}, scores={1: -0.538, 2: -0.69})
        self.assertEqual(
            host._portal_forward_alternatives(request, sources),
            (False, False))

    def test_excluded_straight_doorway_does_not_count(self):
        host, request, sources = self._host(
            {1: 10.0, 2: 90.0}, rejected={(1.0, 0.0)})
        straight, positive = host._portal_forward_alternatives(
            request, sources)
        self.assertFalse(straight)
        self.assertFalse(positive)

    def test_suppressed_positive_room_does_not_count(self):
        from types import SimpleNamespace

        host, request, sources = self._host(
            {1: 90.0, 2: 90.0}, scores={1: 0.182, 2: -0.69})
        records = {
            1: {"semantic_score": 0.182, "exploration_suppressed": True},
            2: {"semantic_score": -0.69, "exploration_suppressed": False},
        }
        host.portal_hypothesis_ledger = SimpleNamespace(
            get=lambda key: records.get(int(key)))
        self.assertEqual(
            host._portal_forward_alternatives(request, sources),
            (False, False))

    def test_missing_geometry_is_ignored(self):
        host, request, sources = self._host({2: 90.0})
        host.cell_xy = lambda *args: (None, None)
        self.assertEqual(
            host._portal_forward_alternatives(request, sources),
            (False, False))


class DeferralCallContractTest(unittest.TestCase):
    """Every ``_note_portal_deferral`` call must match its signature.

    The turn gate originally passed ``semantic_score=`` to the deferral note,
    which only accepts ``heading_delta_deg``/``detail``.  The resulting
    TypeError killed the frontier timer on every cycle and silently parked the
    robot, so this contract is checked statically and once at runtime.
    """

    def test_all_deferral_call_sites_match_the_signature(self):
        import ast
        import inspect

        source_path = SCRIPTS / "global_frontier_portal_selection.py"
        tree = ast.parse(source_path.read_text())
        signature = inspect.signature(
            GlobalFrontierPortalSelectionMixin._note_portal_deferral)
        allowed = set(signature.parameters) - {"self"}
        self.assertIn("detail", allowed)

        offenders = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            name = getattr(func, "attr", None) or getattr(func, "id", None)
            if name != "_note_portal_deferral":
                continue
            for keyword in node.keywords:
                if keyword.arg is not None and keyword.arg not in allowed:
                    offenders.append(
                        (getattr(node, "lineno", 0), keyword.arg))
        self.assertEqual(offenders, [])

    def test_negative_room_turn_note_accepts_the_score_detail(self):
        from global_frontier_portal_belief import PortalHypothesisLedger

        host = GlobalFrontierPortalSelectionMixin()
        host.portal_hypothesis_ledger = PortalHypothesisLedger()
        host.gp_exploration_tree = None
        note = host._note_portal_deferral(
            7, "negative_room_turn", 1.0,
            heading_delta_deg=90.0,
            detail={"semantic_score": -0.69})
        self.assertEqual(note, (7, "negative_room_turn"))
