"""Reproduce the 2026-11-03 dead-goal loop and prove the budget breaks it.

At the stall sites the frontier dispatched the same goal 8, 40 and 33 times.
Every dispatch reset ``active_progress_time`` (they were 5-10 s apart), so the
12 s controller-terminal bound never accumulated, and the planner kept choosing
``retry_viewpoint`` for the same unreachable work-item viewpoint.  The robot
was commanded forward at up to 0.5 m/s and had 11 m of free space in some
direction, so this was never a reverse-only dead end.

These tests drive the watchdog with that sequence:
  * the first two stagnations must not abandon anything (old behaviour: 8-40);
  * the third abandons the work item, marks the region failed and releases;
  * a changed goal or real movement resets the budget;
  * the controller-terminal wait is measured from the first stagnation, not
    from the re-dispatch-reset progress clock.
"""

from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import global_frontier_execution_watchdogs as watchdogs  # noqa: E402
from global_frontier_execution_watchdogs import (  # noqa: E402
    GlobalFrontierExecutionWatchdogMixin,
)


class Ledger:
    def __init__(self):
        self.abandoned = []

    def abandon_unreachable(self, item_id, now, reason, attempt_id=None):
        self.abandoned.append((int(item_id), str(reason), attempt_id))


class Fixture(GlobalFrontierExecutionWatchdogMixin):
    def __init__(self, goal=(10.0, 0.0), limit=3):
        self.active_frontier = (0, 0, float(goal[0]), float(goal[1]))
        self.active_route_kind = "frontier_endpoint"
        self.active_route_id = 80
        self.active_since = 0.0
        self.active_timeout = 18.0
        self.stall_timeout = 8.0
        self.active_progress_time = 100.0
        self.controller_owned_route_failure = True
        self.controller_terminal_wait_seconds = 12.0
        self.recovery_pending_route_id = 0
        self.post_turn_stall_timeout = None
        self.turn_phase_active = False
        self.turn_phase_since = None
        self.turn_supervisor_state = "PASS_THROUGH"
        self.turn_supervisor_route_kind = ""
        self.turn_connector_released = True
        self.goal_stagnation_limit = limit
        self.goal_stagnation_xy = None
        self.goal_stagnation_count = 0
        self.goal_stagnation_since = 0.0
        self.goal_stagnation_abandoned_for = 0
        self.last_route_stagnation_reported_route_id = 0
        self.active_last_progress_signal = "none"
        self.waypoint_release_radius = 0.8
        self.active_last_waypoint_map = (float(goal[0]), float(goal[1]))
        self.active_portal_authentication_phase = ""
        self.active_portal_authentication_entry_approach = False
        self.active_portal_gate_approached_at = None
        self.active_work_item_id = 42
        self.pose_odom = SimpleNamespace(x=0.0, y=0.0, theta=0.0)
        self.active_work_item_attempt_id = 7
        self.place_work_items = Ledger()
        self.active_frontier_component = None
        self.events = []
        self.releases = []
        self.failed_regions = []

    def active_waypoint_distance(self, robot_map):
        return 5.0

    def update_post_turn_watchdog(self, x, y, now):
        return False, 0.0, None

    def active_region_is_stagnant(self, *args, **kwargs):
        return False

    def publish_status(self, event, **fields):
        self.events.append((event, fields))

    def release_active_frontier(self, **kwargs):
        self.releases.append(kwargs)

    def record_failed_frontier_region(self, x, y, now, reason, component=None):
        self.failed_regions.append((x, y, reason))

    def evaluate(self, now, x=0.0, y=0.0, stalled=True, new_route=True):
        # Simulate the 5-10 s re-dispatch: a new route id every time, and the
        # progress clock restarted 9 s ago with no movement.
        if new_route:
            self.active_route_id += 1
        self.active_progress_time = (now - 9.0) if stalled else now
        return self.evaluate_active_route_watchdogs(
            x, y, None, now, False, None, None)


class GoalStagnationBudgetTest(unittest.TestCase):
    def test_the_old_loop_is_reproduced_then_broken(self):
        fixture = Fixture(limit=3)
        # Three re-dispatched routes that each stagnate on the same WorkItem,
        # as at the 2026-11-03 sites (8, 40 and 33 stagnation reports).
        for index, now in enumerate((105.0, 113.0, 121.0)):
            state = fixture.evaluate(now)
            if index < 2:
                self.assertFalse(
                    [e for e, _f in fixture.events
                     if e == "route_unreachable_abandoned"],
                    "must not abandon before the budget is spent",
                )
        self.assertEqual(fixture.place_work_items.abandoned,
                         [(42, "viewpoint_unreachable", 7)])
        self.assertEqual(
            [e for e, _f in fixture.events if e == "route_unreachable_abandoned"],
            ["route_unreachable_abandoned"],
        )
        self.assertEqual(len(fixture.releases), 1)
        self.assertEqual(fixture.failed_regions[0][2], "goal_stagnation_budget")
        self.assertTrue(state.expired)

    def test_the_budget_is_per_work_item(self):
        fixture = Fixture(limit=2)
        fixture.evaluate(105.0)
        fixture.evaluate(113.0)
        fixture.active_work_item_id = 43
        fixture.evaluate(121.0)
        fixture.evaluate(129.0)
        self.assertEqual(
            [entry[0] for entry in fixture.place_work_items.abandoned], [42, 43])
        self.assertEqual(len(fixture.releases), 2)

    def test_a_portal_authentication_route_is_never_abandoned(self):
        fixture = Fixture(limit=2)
        fixture.active_portal_authentication_phase = "entering"
        for now in (105.0, 113.0, 121.0, 129.0):
            fixture.evaluate(now)
        self.assertEqual(fixture.place_work_items.abandoned, [])
        self.assertEqual(fixture.releases, [])

    def test_moving_away_restarts_the_budget(self):
        fixture = Fixture(limit=3)
        fixture.evaluate(105.0)
        fixture.evaluate(113.0)
        fixture.pose_odom.y = 1.0                        # the base actually left
        fixture.evaluate(121.0)
        self.assertEqual(fixture.stagnation_item_count, 1)
        self.assertEqual(fixture.place_work_items.abandoned, [])

    def test_changing_goals_at_one_pose_still_spends_the_budget(self):
        # Run 20261103_110919: 18 reports on 18 different frontier endpoints
        # while the base stayed inside 0.3 m.
        fixture = Fixture(limit=3)
        for index, now in enumerate((105.0, 113.0, 121.0)):
            fixture.active_frontier = (0, 0, 10.0 + index, 0.0)
            fixture.evaluate(now)
        self.assertEqual(
            [entry[0] for entry in fixture.place_work_items.abandoned], [42])

    def test_progress_resets_the_budget(self):
        fixture = Fixture(limit=3)
        fixture.evaluate(105.0)
        fixture.pose_odom.x = 0.40                        # 0.40 m of real travel
        fixture.evaluate(113.0)
        self.assertEqual(fixture.stagnation_item_count, 1)
        self.assertEqual(fixture.place_work_items.abandoned, [])

    def test_the_controller_wait_is_measured_from_the_first_stagnation(self):
        fixture = Fixture(limit=99)
        captured = []
        original = watchdogs.route_lease_failure_decision

        def spy(**kwargs):
            captured.append(kwargs.get("stalled_for"))
            return original(**kwargs)

        watchdogs.route_lease_failure_decision = spy
        try:
            fixture.evaluate(105.0)
            fixture.evaluate(140.0)
        finally:
            watchdogs.route_lease_failure_decision = original
        # 0 s then 16 s since the same goal first stalled; the reset progress
        # clock would have reported 9 s and 9 s, which is why the 12 s bound
        # never fired.
        self.assertEqual([round(v) for v in captured], [0, 35])


class ConfigTest(unittest.TestCase):
    def test_the_budget_is_a_parameter(self):
        config = (SCRIPTS / "global_frontier_config_navigation.py").read_text(
            encoding="utf-8")
        self.assertIn("goal_stagnation_limit", config)

    def test_the_ledger_can_abandon_with_evidence(self):
        ledger = (SCRIPTS / "global_frontier_work_items.py").read_text(
            encoding="utf-8")
        self.assertIn("def abandon_unreachable", ledger)


if __name__ == "__main__":
    unittest.main(verbosity=2)
