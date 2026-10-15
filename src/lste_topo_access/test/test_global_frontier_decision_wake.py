"""Regression tests for the fast/slow decision wake boundary."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_decision_wake import DecisionWakeScheduler  # noqa: E402


class DecisionWakeSchedulerTest(unittest.TestCase):
    def test_startup_has_one_wake(self):
        scheduler = DecisionWakeScheduler()

        wake = scheduler.claim()

        self.assertIsNotNone(wake)
        self.assertEqual(wake.reasons, ("startup",))
        self.assertIsNone(scheduler.claim())

    def test_identical_map_publications_do_not_wake_again(self):
        scheduler = DecisionWakeScheduler()
        scheduler.claim()

        self.assertTrue(scheduler.observe_map(fingerprint="map-a"))
        self.assertFalse(scheduler.observe_map(fingerprint="map-a"))
        wake = scheduler.claim()

        self.assertEqual(wake.reasons, ("map_available",))

    def test_changed_map_is_a_new_fact(self):
        scheduler = DecisionWakeScheduler()
        scheduler.observe_map(fingerprint="map-a")
        scheduler.claim()

        self.assertTrue(scheduler.observe_map(fingerprint="map-b"))
        self.assertEqual(scheduler.claim().reasons, ("map_changed",))

    def test_live_route_blocks_claim_but_terminal_releases_it(self):
        scheduler = DecisionWakeScheduler()
        scheduler.claim()
        scheduler.start_route(11)
        scheduler.request("map_changed")

        self.assertIsNone(scheduler.claim(active_route_id=11))
        scheduler.finish_route(11, "execution_terminal_failure")
        wake = scheduler.claim()

        self.assertEqual(
            wake.reasons,
            ("map_changed", "execution_terminal_failure"),
        )

    def test_stale_terminal_cannot_wake_new_route(self):
        scheduler = DecisionWakeScheduler()
        scheduler.start_route(12)

        self.assertFalse(
            scheduler.observe_status(
                "execution_terminal_failure", {"route_id": 11}
            )
        )
        self.assertIsNone(scheduler.claim())
        self.assertEqual(scheduler.active_route_id, 12)

    def test_route_command_consumes_pending_reasons(self):
        scheduler = DecisionWakeScheduler()
        scheduler.request("map_changed")
        scheduler.observe_status("route_command", {"route_id": 9})

        self.assertFalse(scheduler.pending)
        self.assertEqual(scheduler.active_route_id, 9)

    def test_task_done_halts_future_wakes(self):
        scheduler = DecisionWakeScheduler()
        scheduler.observe_status("task_done")

        self.assertTrue(scheduler.halted)
        self.assertIsNone(scheduler.claim())
        self.assertFalse(scheduler.request("map_changed"))

    def test_reasons_are_stable_and_deduplicated(self):
        scheduler = DecisionWakeScheduler()
        scheduler.claim()
        scheduler.request("portal_probe_started")
        scheduler.request("map_changed")
        scheduler.request("portal_probe_started")

        self.assertEqual(
            scheduler.claim().reasons,
            ("portal_probe_started", "map_changed"),
        )

    def test_repeated_lifecycle_fact_does_not_rewake_after_claim(self):
        scheduler = DecisionWakeScheduler()
        fields = {
            "route_id": 4,
            "reason": "durable_identity_not_in_current_frontier_snapshot",
        }

        self.assertTrue(
            scheduler.observe_status("frontier_route_unavailable", fields)
        )
        scheduler.claim()
        self.assertFalse(
            scheduler.observe_status("frontier_route_unavailable", dict(fields))
        )
        self.assertIsNone(scheduler.claim())

    def test_new_route_identity_is_a_new_lifecycle_fact(self):
        scheduler = DecisionWakeScheduler()
        self.assertTrue(
            scheduler.observe_status(
                "frontier_route_unavailable", {"route_id": 4}
            )
        )
        scheduler.claim()
        self.assertTrue(
            scheduler.observe_status(
                "frontier_route_unavailable", {"route_id": 5}
            )
        )


if __name__ == "__main__":
    unittest.main()
