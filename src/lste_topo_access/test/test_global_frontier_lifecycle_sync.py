#!/usr/bin/env python3
"""Regression tests for quiet-topic synchronization after route completion."""

from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_lifecycle import GlobalFrontierLifecycleMixin
from lifecycle_manager import EventType, LifecycleManager, State


def grid_message():
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id="map",
            stamp=SimpleNamespace(to_sec=lambda: 1.0),
        ),
        info=SimpleNamespace(width=1, height=1, resolution=1.0),
        data=[0],
    )


class GlobalFrontierLifecycleSyncTest(unittest.TestCase):
    def _owner(self, *, cached=True, timeout=10.0):
        owner = GlobalFrontierLifecycleMixin()
        now = time.monotonic()
        owner.map_msg = grid_message() if cached else None
        owner.costmap_msg = grid_message() if cached else None
        owner.costmap_last_receive_wall = now if cached else 0.0
        owner.costmap_max_age = 3.0
        owner.costmap_message_count = 0
        owner.cached_costmap_validation = None
        owner.cached_costmap_validation_wall = 0.0
        owner.decision_wake_scheduler = None
        owner._lifecycle_sync_seen = set()
        owner._lifecycle_sync_started_transaction = 0
        owner.status_events = []
        owner.publish_status = lambda name, **fields: owner.status_events.append(
            (name, fields)
        )
        owner.lifecycle_manager = LifecycleManager(
            event_handler=owner._handle_lifecycle_event,
            transition_handler=owner._on_lifecycle_transition,
            timeout_handler=owner._on_lifecycle_timeout,
            timeouts={State.SYNCING: timeout},
            time_fn=lambda: now,
        )
        return owner, now

    def test_valid_cache_is_reused_and_sync_completes_without_new_publish(self):
        owner, now = self._owner(cached=True)
        transaction_id = owner.lifecycle_manager.begin_transaction(
            State.EXECUTION_DONE,
            now=now,
        )
        owner.lifecycle_manager.enqueue_type(
            EventType.SYNC_REQUESTED,
            transaction_id=transaction_id,
        )

        result = owner.lifecycle_manager.tick(now=now)

        self.assertEqual(result.state, State.IDLE)
        self.assertIn(
            "lifecycle_sync_snapshot_reused",
            [name for name, _fields in owner.status_events],
        )

    def test_missing_cache_stays_syncing_and_times_out(self):
        owner, now = self._owner(cached=False, timeout=1.0)
        transaction_id = owner.lifecycle_manager.begin_transaction(
            State.EXECUTION_DONE,
            now=now,
        )
        owner.lifecycle_manager.enqueue_type(
            EventType.SYNC_REQUESTED,
            transaction_id=transaction_id,
        )

        first = owner.lifecycle_manager.tick(now=now)
        second = owner.lifecycle_manager.tick(now=now + 1.1)

        self.assertEqual(first.state, State.SYNCING)
        self.assertTrue(second.timed_out)
        self.assertEqual(second.state, State.FAILED)
        self.assertIn(
            "lifecycle_sync_waiting_for_publish",
            [name for name, _fields in owner.status_events],
        )

    def test_failed_transition_releases_frontier_lease_before_reset(self):
        owner, _now = self._owner(cached=False)
        owner.active_frontier = object()
        owner.last_planning_wall = 42.0
        released = []
        owner.release_active_frontier = lambda **fields: released.append(fields)

        owner._on_lifecycle_transition(State.DISPATCHED, State.FAILED, None)

        self.assertEqual(released, [{"discard_prefetch": True}])
        self.assertEqual(owner.last_planning_wall, 0.0)


if __name__ == "__main__":
    unittest.main()
