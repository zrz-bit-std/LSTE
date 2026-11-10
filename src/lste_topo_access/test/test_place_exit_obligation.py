#!/usr/bin/env python3
"""Regression tests for the child-owned doorway exit obligation."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_planning_snapshot import (  # noqa: E402
    GlobalFrontierPlanningSnapshotMixin,
)
from global_frontier_work_items import PlaceWorkItemLedger  # noqa: E402


class ExitActivationProbe(GlobalFrontierPlanningSnapshotMixin):
    def __init__(self):
        self.place_work_items = PlaceWorkItemLedger(0.7)
        self.current_physical_place_id = 2
        self.current_place_scope_pending_frontier_count = 0
        self.target_region_claim_active = False
        self.target_observation_work_pending = False
        self.events = []
        self.clock = 10.0
        self.time_provider = lambda: self.clock

    def publish_status(self, event, **fields):
        self.events.append((event, fields))

    def active_portal_authentication(self):
        return None

    def make_exit(self):
        return self.place_work_items.ensure_place_exit(
            2, 7, 1, (6.6, 1.9), (6.2, 3.0), now=1.0, generation=1,
        )


class PlaceExitActivationTest(unittest.TestCase):
    def test_room_work_keeps_the_exit_dormant(self):
        probe = ExitActivationProbe()
        probe.make_exit()
        probe.place_work_items._new_item(
            2, frozenset(), 1.0, xy=(1.0, 1.0), reason="room_work",
        )

        for _ in range(3):
            probe.clock += 1.0
            self.assertFalse(probe._activate_place_exit_if_ready())

        item = probe.place_work_items.place_exit_item(2)
        self.assertEqual(item["exit_state"], "dormant")

    def test_two_complete_cycles_activate_the_exit(self):
        probe = ExitActivationProbe()
        probe.make_exit()

        probe.clock = 10.0
        self.assertFalse(probe._activate_place_exit_if_ready())
        probe.clock = 11.0
        self.assertTrue(probe._activate_place_exit_if_ready())

        item = probe.place_work_items.place_exit_item(2)
        self.assertEqual(item["exit_state"], "ready")
        self.assertTrue(any(event == "place_exit_ready"
                            for event, _fields in probe.events))

    def test_activation_hysteresis_resets_when_work_reappears(self):
        probe = ExitActivationProbe()
        probe.make_exit()

        probe.clock = 10.0
        self.assertFalse(probe._activate_place_exit_if_ready())
        probe.place_work_items._new_item(
            2, frozenset(), 11.0, xy=(1.0, 1.0), reason="room_work",
        )
        probe.clock = 11.0
        self.assertFalse(probe._activate_place_exit_if_ready())
        self.assertEqual(
            probe.place_work_items.place_exit_item(2)["exit_state"], "dormant",
        )

    def test_target_chase_vetoes_the_exit(self):
        probe = ExitActivationProbe()
        probe.make_exit()
        probe.target_region_claim_active = True

        for _ in range(3):
            probe.clock += 1.0
            self.assertFalse(probe._activate_place_exit_if_ready())

        self.assertEqual(
            probe.place_work_items.place_exit_item(2)["exit_state"], "dormant",
        )

    def test_pending_frontier_vetoes_the_exit(self):
        probe = ExitActivationProbe()
        probe.make_exit()
        probe.current_place_scope_pending_frontier_count = 3

        for _ in range(3):
            probe.clock += 1.0
            self.assertFalse(probe._activate_place_exit_if_ready())

        self.assertEqual(
            probe.place_work_items.place_exit_item(2)["exit_state"], "dormant",
        )

    def test_stalled_dispatch_retries_then_blocks(self):
        probe = ExitActivationProbe()
        exit_id = probe.make_exit()
        probe.place_work_items.activate_place_exit(exit_id, now=10.0)

        limit = 3
        probe.place_exit_max_dispatches = limit
        probe.place_exit_dispatch_timeout = 60.0
        for attempt in range(limit):
            probe.place_work_items.mark_place_exit_dispatched(
                exit_id, now=probe.clock,
            )
            probe.clock += 61.0
            probe._activate_place_exit_if_ready()

        item = probe.place_work_items.place_exit_item(2)
        self.assertEqual(item["exit_state"], "blocked")
        self.assertTrue(any(event == "place_exit_blocked"
                            for event, _fields in probe.events))


    def test_ready_exit_that_never_materializes_is_blocked(self):
        probe = ExitActivationProbe()
        exit_id = probe.make_exit()
        probe.place_work_items.activate_place_exit(exit_id, now=10.0)
        probe.place_exit_ready_timeout = 120.0

        probe.clock = 200.0
        probe._activate_place_exit_if_ready()

        item = probe.place_work_items.place_exit_item(2)
        self.assertEqual(item["exit_state"], "blocked")
        self.assertTrue(any(event == "place_exit_blocked"
                            for event, _fields in probe.events))


if __name__ == "__main__":
    unittest.main()
