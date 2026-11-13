"""Bounded target-claim ownership and stale planning preempts.

Runs 20261109_004523, 20261110_232536, 20261112_201037/220714/231045 and
20261113_154448/183353 all parked inside the east target room: the target had
been seen once, its visual track died, ``TargetObservationWork`` stayed
unresolved, and that obligation blocked the Place exit and every candidate
outside the claimed component.  Four runs additionally went silent because an
execution terminal left ``planning_preempt_requested`` set, which makes
``bfs()`` abort every snapshot before selection.

These tests pin the bounds:

* work whose target has not been seen for ``target_observation_timeout`` is
  released (not completed), so new evidence can reopen it;
* a stale ``target_supported`` belief no longer blocks leaving the room;
* the region-claim wall clock starts at bind time and is checked before the
  selector's early wait returns;
* a preempt older than ``planning_preempt_max_stale_seconds`` is cleared, and a
  rejected terminal clears it immediately.
"""
from pathlib import Path
from types import SimpleNamespace
import sys
import time
import unittest
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_observation_place_state import (  # noqa: E402
    GlobalFrontierObservationPlaceStateMixin,
)
from global_frontier_planning_runtime import (  # noqa: E402
    GlobalFrontierPlanningRuntimeMixin,
)
from global_frontier_planning_snapshot import (  # noqa: E402
    GlobalFrontierPlanningSnapshotMixin,
)
from global_frontier_planning_selection import (  # noqa: E402
    GlobalFrontierPlanningSelectionMixin,
)
from global_frontier_target_observation_work import (  # noqa: E402
    TargetObservationWorkLedger,
)
from global_frontier_terminal_validation import (  # noqa: E402
    GlobalFrontierTerminalValidationMixin,
)


class _PlaceHost(
    GlobalFrontierObservationPlaceStateMixin,
    GlobalFrontierPlanningSnapshotMixin,
):
    """Mixin surface used by the claim/observation gates."""

    def __init__(self):
        self.events = []
        self.publish_status = lambda name, **fields: self.events.append(
            (name, fields))


class StaleTargetWorkExpiryTest(unittest.TestCase):
    def _host(self, live_tracks=()):
        host = _PlaceHost()
        host.target_observation_work = TargetObservationWorkLedger("task-v1")
        host.target_observation_work.observe_target(
            "task-v1", 8, labels=("yellow cup",),
            track_id="yellow_cup:yellow cup:1", now=100.0,
        )
        host.target_belief = SimpleNamespace(
            evidence=lambda place_id: {
                "state": "target_supported", "track_ids": list(live_tracks),
                "last_observed_at": 100.0,
            })
        host.target_observation_timeout = 30.0
        return host

    def test_expired_work_is_released_and_announced(self):
        host = self._host()
        self.assertEqual(host.expire_stale_target_observation_work(129.0), [])
        self.assertTrue(host.target_observation_work.has_pending(8))

        released = host.expire_stale_target_observation_work(130.0)
        self.assertEqual([item["place_id"] for item in released], [8])
        self.assertFalse(host.target_observation_work.has_pending(8))
        self.assertEqual(host.events[-1][0], "target_observation_work_released")
        self.assertEqual(host.events[-1][1]["place_id"], 8)
        self.assertEqual(
            host.events[-1][1]["reason"], "observation_timeout")

    def test_live_track_keeps_the_obligation(self):
        host = self._host(live_tracks=("yellow_cup:yellow cup:1",))
        self.assertEqual(
            host.expire_stale_target_observation_work(999.0), [])
        self.assertTrue(host.target_observation_work.has_pending(8))

    def test_zero_timeout_disables_the_release(self):
        host = self._host()
        host.target_observation_timeout = 0.0
        self.assertEqual(
            host.expire_stale_target_observation_work(999.0), [])
        self.assertTrue(host.target_observation_work.has_pending(8))


class PlaceExitBeliefFreshnessTest(unittest.TestCase):
    def _host(self, record):
        host = _PlaceHost()
        host.place_work_items = SimpleNamespace(
            unresolved_observation_count=lambda place_id, **kwargs: 0,
            actionable_frontier_observation_count=lambda place_id: 0,
        )
        host.target_region_claim_active = False
        host.target_belief = SimpleNamespace(evidence=lambda place_id: record)
        host.target_observation_timeout = 30.0
        host.active_portal_authentication = lambda: None
        host.junction_portal_authentication = None
        return host

    def test_stale_single_hit_no_longer_blocks_the_exit(self):
        host = self._host({
            "state": "target_supported", "track_ids": [],
            "last_observed_at": 2660.4,
        })
        self.assertTrue(host._place_exit_activation_ready(8, now=3000.0))

    def test_a_recent_sighting_still_blocks_the_exit(self):
        host = self._host({
            "state": "target_supported", "track_ids": [],
            "last_observed_at": 2660.4,
        })
        self.assertFalse(host._place_exit_activation_ready(8, now=2670.0))

    def test_a_live_track_still_blocks_the_exit(self):
        host = self._host({
            "state": "target_supported",
            "track_ids": ["yellow_cup:yellow cup:1"],
            "last_observed_at": 1.0,
        })
        self.assertFalse(host._place_exit_activation_ready(8, now=9999.0))

    def test_no_target_evidence_allows_the_exit(self):
        host = self._host({"state": "context_supported", "track_ids": []})
        self.assertTrue(host._place_exit_activation_ready(8, now=9999.0))


class TargetRegionClaimClockTest(unittest.TestCase):
    def _host(self):
        host = _PlaceHost()
        host.target_region_claim_active = True
        host.target_region_claim_wall_timeout = 60.0
        host.target_region_claim_wait_since_wall = None
        host.target_region_claim_bound_wall = 1000.0
        host.target_region_claim_component = {"epoch": 3, "label": 1}
        host.target_region_claim_track_id = "yellow_cup:yellow cup:1"
        host.target_region_claim_request_id = 1
        host.target_region_claim_bound_reported = True
        host.target_region_claim_waiting_reported = False
        host.target_reinspection_pending = True
        host.current_physical_place_id = 8
        host.target_observation_work = TargetObservationWorkLedger("task-v1")
        host.target_observation_work.observe_target(
            "task-v1", 8, labels=("yellow cup",), now=2660.0,
        )
        return host

    def test_wall_expiry_is_measured_from_the_bind(self):
        host = self._host()
        self.assertFalse(host._target_region_claim_wall_expired(1059.9))
        self.assertTrue(host._target_region_claim_wall_expired(1060.0))

    def test_inactive_claim_never_expires(self):
        host = self._host()
        host.target_region_claim_active = False
        self.assertFalse(host._target_region_claim_wall_expired(9999.0))

    def test_first_release_call_after_the_bind_deadline_releases(self):
        host = self._host()
        with patch("time.monotonic", return_value=1061.0):
            host._release_exhausted_target_region_claim()
        self.assertFalse(host.target_region_claim_active)
        names = [name for name, _fields in host.events]
        self.assertIn("target_region_claim_wall_timeout", names)
        self.assertIn("target_region_claim_released", names)

    def test_waiting_before_the_deadline_keeps_the_claim(self):
        host = self._host()
        with patch("time.monotonic", return_value=1020.0):
            host._release_exhausted_target_region_claim()
        self.assertTrue(host.target_region_claim_active)
        names = [name for name, _fields in host.events]
        self.assertIn("target_region_claim_waiting_for_obligation", names)


class EarlyClaimReleaseTest(unittest.TestCase):
    """The release must run before the selector's early wait returns."""

    def _host(self):
        host = GlobalFrontierPlanningSelectionMixin()
        host.releases = 0
        host.target_region_claim_active = True
        host._target_region_claim_wall_expired = lambda now_wall=None: True

        def release():
            host.releases += 1

        host._release_exhausted_target_region_claim = release
        host.portal_authentication_owns_navigation = lambda: False
        host.portal_transaction = SimpleNamespace(
            active=True, state="throat",
            snapshot=lambda: SimpleNamespace(
                transaction_id=9, route_id=53, state="throat"),
        )
        host.pending_local_egress = None
        host.pending_portal_retry = None
        host.portal_transaction_route_rebind_pending = False
        host.events = []
        host.publish_status = lambda name, **fields: host.events.append(
            (name, fields))
        return host

    def test_release_runs_before_the_portal_transaction_wait(self):
        host = self._host()
        snapshot = SimpleNamespace(map_context=None)
        candidate, mode, wait = host.select_next_active_frontier(snapshot)
        self.assertIsNone(candidate)
        self.assertEqual(mode, "portal_transaction_wait")
        self.assertEqual(host.releases, 1)
        self.assertEqual(host.events[0][0], "portal_transaction_selection_blocked")


class PlanningPreemptExpiryTest(unittest.TestCase):
    def _host(self):
        host = GlobalFrontierPlanningRuntimeMixin()
        host.planning_preempt_requested = True
        host.planning_preempt_requested_wall = 100.0
        host.planning_preempt_reason = "execution_terminal"
        host.planning_preempt_max_stale_seconds = 5.0
        host.events = []
        host.publish_status = lambda name, **fields: host.events.append(
            (name, fields))
        return host

    def test_a_young_preempt_is_kept(self):
        host = self._host()
        self.assertFalse(host.expire_stale_planning_preempt(104.9))
        self.assertTrue(host.planning_preempt_requested)

    def test_a_stale_preempt_is_cleared_and_announced(self):
        host = self._host()
        self.assertTrue(host.expire_stale_planning_preempt(105.0))
        self.assertFalse(host.planning_preempt_requested)
        self.assertIsNone(host.planning_preempt_requested_wall)
        self.assertEqual(host.events[-1][0], "planning_preempt_expired")
        self.assertEqual(host.events[-1][1]["reason"], "execution_terminal")

    def test_request_stamps_the_wall_clock(self):
        host = self._host()
        host.planning_preempt_requested = False
        host.planning_preempt_requested_wall = None
        before = time.monotonic()
        host.request_planning_preempt("execution_terminal")
        self.assertGreaterEqual(host.planning_preempt_requested_wall, before)
        host.clear_planning_preempt()
        self.assertIsNone(host.planning_preempt_requested_wall)

    def test_missing_stamp_is_initialized_not_expired(self):
        host = self._host()
        host.planning_preempt_requested_wall = None
        self.assertFalse(host.expire_stale_planning_preempt(9999.0))
        self.assertIsNotNone(host.planning_preempt_requested_wall)
        self.assertTrue(host.planning_preempt_requested)


class RejectedTerminalClearsPreemptTest(unittest.TestCase):
    def test_rejection_clears_the_cooperative_flag(self):
        host = GlobalFrontierTerminalValidationMixin()
        cleared = []
        host.planning_preempt_requested = True
        host.clear_planning_preempt = lambda: cleared.append(True)
        host.events = []
        host.publish_status = lambda name, **fields: host.events.append(
            (name, fields))
        with patch(
                "global_frontier_terminal_validation.rospy.loginfo_throttle"):
            host._reject_terminal(SimpleNamespace(), "route_mismatch")
        self.assertEqual(cleared, [True])
        self.assertEqual(host.events[-1][0], "stale_terminal_rejected")


if __name__ == "__main__":
    unittest.main()
