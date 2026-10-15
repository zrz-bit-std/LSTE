"""Regression for completing a small target after its viewpoint portfolio."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from goal_manager_goal_arbitration import GoalManagerGoalArbitrationMixin  # noqa: E402
from goal_manager_target_completion import GoalManagerTargetCompletionMixin  # noqa: E402
from goal_manager_target_approach_transaction import TargetApproachTransaction  # noqa: E402
from goal_manager_target_viewpoint_attempt_ledger import VIEWPOINT_OBSERVED  # noqa: E402


class Stamp:
    def __init__(self, seconds):
        self.seconds = float(seconds)
        self.secs = int(seconds)
        self.nsecs = int((self.seconds - self.secs) * 1e9)

    def to_sec(self):
        return self.seconds


class PortfolioFixture(
    GoalManagerGoalArbitrationMixin,
    GoalManagerTargetCompletionMixin,
):
    def __init__(self):
        self.task_done_published = False
        self.target_done_require_locked = False
        self.current_state = 0
        self.target_done_require_approach_terminal = True
        self.target_follow_confirmed = True
        self.target_track_id = "yellow_cup:yellow cup:1"
        self.target_approach_track_id = self.target_track_id
        self.target_completed_segments = 3
        self.target_terminal_reobserve_pending = False
        self.target_terminal_reobserve_min_epoch = 0
        self.target_observation_epoch = 8
        self.target_done_max_detection_age = 1.0
        self.target_follow_candidate_timeout = 8.0
        self.target_follow_min_score = 0.2
        self.target_follow_min_box_size = 0.01
        self.target_done_min_box_width = 0.06
        self.target_done_min_box_height = 0.06
        self.target_done_min_score = 0.4
        self.target_close_since = 10.0
        self.target_close_hits = 1
        self.latest_dets = SimpleNamespace(
            header=SimpleNamespace(stamp=Stamp(19.8), seq=8)
        )
        self.target_detection_for_track = lambda _message: SimpleNamespace(
            score=0.31, w=0.019, h=0.024
        )
        self.target_approach_transaction = TargetApproachTransaction()
        self.target_approach_transaction.begin(self.target_track_id, 1.0, 1)
        ledger = self.target_approach_transaction.viewpoint_ledger
        for index in range(4):
            candidate = ledger.ensure_candidate(index)
            candidate.route_status = VIEWPOINT_OBSERVED
        self.events = []
        self.published = []
        self.revalidation_calls = []
        self.pub_task_done = SimpleNamespace(
            publish=lambda message: self.published.append(message)
        )

    def publish_goal_arbitration(self, event, **fields):
        self.events.append((event, fields))

    def set_navigation_hold(self, active, reason):
        self.events.append(("navigation_hold", {"active": active, "reason": reason}))

    def release_target_follow_to_frontier(self, now, **kwargs):
        self.revalidation_calls.append((now, kwargs))
        return None


class TargetViewpointPortfolioCompletionTest(unittest.TestCase):
    def test_small_target_requires_revalidation_after_all_validated_views(self):
        fixture = PortfolioFixture()

        self.assertTrue(fixture.target_viewpoint_portfolio_complete(20.0))
        fixture.maybe_publish_task_done(20.0)

        self.assertFalse(fixture.task_done_published)
        self.assertEqual(len(fixture.published), 0)
        self.assertEqual(len(fixture.revalidation_calls), 1)
        self.assertTrue(
            fixture.revalidation_calls[0][1]["preserve_obligation"]
        )
        self.assertTrue(fixture.target_portfolio_revalidation_requested)
        self.assertIn(
            "target_viewpoint_portfolio_requires_revalidation",
            [event for event, _ in fixture.events],
        )

    def test_open_viewpoint_does_not_complete_portfolio(self):
        fixture = PortfolioFixture()
        fixture.target_approach_transaction.viewpoint_ledger.get(
            fixture.target_approach_transaction.viewpoint_ledger.candidate_id(3)
        ).route_status = "route_pending"

        self.assertFalse(fixture.target_viewpoint_portfolio_complete(20.0))


if __name__ == "__main__":
    unittest.main()
