#!/usr/bin/env python3
"""Focused tests for the single-threaded lifecycle contract."""

from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import patch


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from lifecycle_manager import EventType, LifecycleManager, State


class LifecycleManagerTest(unittest.TestCase):
    def test_uuid1_time_component_preserves_transaction_order_at_rollover(self):
        class FakeUuid:
            def __init__(self, timestamp, identity):
                self.time = timestamp
                self.int = identity

        with patch(
            "lifecycle_manager.uuid.uuid1",
            side_effect=(
                FakeUuid((1 << 32) - 1, (1 << 64) - 1),
                FakeUuid(1 << 32, 0),
            ),
        ):
            previous = LifecycleManager._new_transaction_id()
            current = LifecycleManager._new_transaction_id()

        self.assertGreater(current, previous)

    def test_uuid1_clock_rollback_does_not_make_a_new_transaction_stale(self):
        class FakeUuid:
            def __init__(self, timestamp, identity):
                self.time = timestamp
                self.int = identity

        with patch(
            "lifecycle_manager.uuid.uuid1",
            side_effect=(
                FakeUuid(100, (1 << 64) - 1),
                FakeUuid(99, 0),
            ),
        ):
            previous = LifecycleManager._new_transaction_id()
            current = LifecycleManager._new_transaction_id()

        self.assertGreater(current, previous)

    def test_callbacks_can_enqueue_but_only_tick_changes_state(self):
        manager = LifecycleManager()
        transaction_id = manager.begin_transaction()

        manager.enqueue_type(EventType.NAV_DISPATCHED, transaction_id=transaction_id)
        self.assertEqual(manager.current_state, State.IDLE)

        result = manager.tick(now=1.0)
        self.assertEqual(result.processed, 1)
        self.assertEqual(manager.current_state, State.DISPATCHED)

    def test_stale_events_are_discarded_after_new_transaction(self):
        manager = LifecycleManager()
        old_transaction = manager.begin_transaction()
        manager.begin_transaction()
        manager.enqueue_type(EventType.NAV_FAILED, transaction_id=old_transaction)

        result = manager.tick(now=2.0)
        self.assertEqual(result.discarded_stale, 1)
        self.assertEqual(manager.current_state, State.IDLE)

    def test_navigation_hold_state_survives_a_transaction_boundary(self):
        observed = []
        manager = LifecycleManager(
            event_handler=lambda event: observed.append(event.payload),
        )
        old_transaction = manager.begin_transaction()
        manager.enqueue_type(
            EventType.BRIDGE_NAVIGATION_HOLD,
            True,
            transaction_id=old_transaction,
        )
        manager.begin_transaction()
        manager.enqueue_type(
            EventType.BRIDGE_NAVIGATION_HOLD,
            False,
            transaction_id=old_transaction,
        )

        result = manager.tick(now=2.5)

        self.assertEqual(result.processed, 1)
        self.assertEqual(result.discarded_stale, 0)
        self.assertEqual(observed, [False])

    def test_events_are_serialized_even_when_producers_race(self):
        manager = LifecycleManager()
        transaction_id = manager.begin_transaction()
        barrier = threading.Barrier(3)

        def producer(event_type):
            barrier.wait()
            manager.enqueue_type(event_type, transaction_id=transaction_id)

        first = threading.Thread(target=producer, args=(EventType.NAV_DISPATCHED,))
        second = threading.Thread(target=producer, args=(EventType.TASK_COMPLETED,))
        first.start()
        second.start()
        barrier.wait()
        first.join()
        second.join()

        result = manager.tick(now=3.0)
        self.assertEqual(result.processed, 2)
        self.assertEqual(manager.current_state, State.COMPLETED)

    def test_timeout_is_checked_inside_tick(self):
        manager = LifecycleManager(timeouts={State.DISPATCHED: 2.0})
        transaction_id = manager.begin_transaction(now=10.0)
        manager.enqueue_type(EventType.NAV_DISPATCHED, transaction_id=transaction_id)
        manager.tick(now=10.0)
        self.assertEqual(manager.current_state, State.DISPATCHED)

        result = manager.tick(now=12.1)
        self.assertTrue(result.timed_out)
        self.assertEqual(manager.current_state, State.FAILED)

    def test_failed_timeout_resets_to_a_clean_transaction_on_next_tick(self):
        manager = LifecycleManager(timeouts={State.DISPATCHED: 2.0})
        transaction_id = manager.begin_transaction(now=10.0)
        manager.enqueue_type(EventType.NAV_DISPATCHED, transaction_id=transaction_id)
        manager.tick(now=10.0)
        manager.tick(now=12.1)

        # A delayed event from the failed transaction must not revive dirty
        # state; the next owner tick creates a new clean transaction first.
        manager.enqueue_type(EventType.NAV_DISPATCHED, transaction_id=transaction_id)
        reset_result = manager.tick(now=12.2)

        self.assertEqual(reset_result.state, State.IDLE)
        self.assertGreater(manager.current_transaction_id, transaction_id)

        fresh_transaction = manager.current_transaction_id
        manager.enqueue_type(
            EventType.NAV_DISPATCHED,
            transaction_id=fresh_transaction,
        )
        resumed = manager.tick(now=12.3)
        self.assertEqual(resumed.state, State.DISPATCHED)

    def test_handler_and_transition_side_effects_run_on_tick_thread(self):
        calls = []

        def handler(event):
            calls.append(("event", event.type, threading.get_ident()))
            return State.DISPATCHED

        def transition(previous, current, event):
            calls.append(("transition", previous, current, threading.get_ident()))

        manager = LifecycleManager(
            event_handler=handler,
            transition_handler=transition,
        )
        transaction_id = manager.begin_transaction()
        producer_thread_id = []

        def producer():
            producer_thread_id.append(threading.get_ident())
            manager.enqueue_type(EventType.TASK_STARTED, transaction_id=transaction_id)

        thread = threading.Thread(target=producer)
        thread.start()
        thread.join()
        tick_thread_id = threading.get_ident()
        manager.tick(now=4.0)

        self.assertNotEqual(producer_thread_id[0], tick_thread_id)
        self.assertEqual({item[-1] for item in calls}, {tick_thread_id})
        self.assertEqual(manager.current_state, State.DISPATCHED)

    def test_compute_handler_runs_inside_the_tick_ownership_window(self):
        manager = LifecycleManager()
        manager.begin_transaction()
        observed = []

        def compute(_now):
            observed.append(manager._tick_active)
            manager.begin_transaction(State.DISPATCHED, now=5.0)

        manager.tick(now=5.0, compute_handler=compute)

        self.assertEqual(observed, [True])
        self.assertEqual(manager.current_state, State.DISPATCHED)

    def test_newer_transaction_event_is_adopted_instead_of_dropped(self):
        manager = LifecycleManager()
        current = manager.begin_transaction()
        newer = current + 1
        manager.enqueue_type(EventType.NAV_DISPATCHED, transaction_id=newer)

        result = manager.tick(now=6.0)

        self.assertEqual(result.adopted_future, 1)
        self.assertEqual(result.discarded_stale, 0)
        self.assertEqual(manager.current_transaction_id, newer)
        self.assertEqual(manager.current_state, State.DISPATCHED)

    def test_newer_transaction_supersedes_older_events_already_in_the_inbox(self):
        observed = []
        manager = LifecycleManager(
            event_handler=lambda event: observed.append(event.payload),
        )
        current = manager.begin_transaction()
        newer = current + 1
        manager.enqueue_type(EventType.POSE_UPDATED, "old", current)
        manager.enqueue_type(EventType.NAV_DISPATCHED, "new", newer)

        result = manager.tick(now=6.5)

        self.assertEqual(result.adopted_future, 1)
        self.assertNotIn("old", observed)
        self.assertIn("new", observed)

    def test_transaction_boundary_cannot_be_mutated_after_tick(self):
        manager = LifecycleManager()
        manager.begin_transaction()
        manager.tick(now=7.0)

        with self.assertRaises(RuntimeError):
            manager.begin_transaction()

    def test_high_rate_pose_samples_keep_only_the_latest_event(self):
        observed = []
        manager = LifecycleManager(
            event_handler=lambda event: observed.append(event),
            max_inbox_size=8,
        )
        transaction_id = manager.begin_transaction()

        for value in range(10000):
            self.assertTrue(
                manager.enqueue_type(
                    EventType.POSE_UPDATED,
                    value,
                    transaction_id=transaction_id,
                )
            )

        self.assertEqual(manager.inbox.qsize(), 1)
        result = manager.tick(now=8.0)

        self.assertEqual(result.processed, 1)
        self.assertEqual(observed[-1].payload, 9999)
        self.assertGreater(manager.coalesced_events, 0)

    def test_causal_event_is_not_evicted_by_distinct_sample_streams(self):
        manager = LifecycleManager(max_inbox_size=2)
        transaction_id = manager.begin_transaction()

        self.assertTrue(
            manager.enqueue_type(EventType.POSE_UPDATED, "pose", transaction_id)
        )
        manager.enqueue_type(EventType.SCAN_UPDATED, "scan", transaction_id)

        self.assertTrue(
            manager.enqueue_type(EventType.NAV_FAILED, transaction_id=transaction_id)
        )

    def test_costmap_delta_burst_requests_full_resync(self):
        observed = []
        manager = LifecycleManager(
            event_handler=lambda event: observed.append(event),
            max_inbox_size=8,
        )
        transaction_id = manager.begin_transaction()

        for value in range(10000):
            self.assertTrue(
                manager.enqueue_type(
                    EventType.COSTMAP_UPDATED,
                    ("delta", value),
                    transaction_id=transaction_id,
                )
            )

        self.assertEqual(manager.inbox.qsize(), 1)
        result = manager.tick(now=9.0)

        self.assertEqual(result.processed, 1)
        self.assertEqual(observed[-1].payload, ("resync", None))

    def test_tick_budget_leaves_remaining_events_for_a_later_tick(self):
        manager = LifecycleManager(max_inbox_size=32, max_events_per_tick=3)
        transaction_id = manager.begin_transaction()
        for _ in range(10):
            self.assertTrue(
                manager.enqueue_type(
                    EventType.BRIDGE_WAKE,
                    transaction_id=transaction_id,
                )
            )

        result = manager.tick(now=10.0)

        self.assertEqual(result.processed, 3)
        self.assertEqual(manager.inbox.qsize(), 7)

    def test_causal_terminal_events_are_not_coalesced(self):
        observed = []
        manager = LifecycleManager(
            event_handler=lambda event: observed.append(event.type),
        )
        transaction_id = manager.begin_transaction()
        manager.enqueue_type(EventType.NAV_REACHED, transaction_id=transaction_id)
        manager.enqueue_type(EventType.NAV_FAILED, transaction_id=transaction_id)

        result = manager.tick(now=11.0)

        self.assertEqual(result.processed, 2)
        self.assertEqual(
            observed,
            [EventType.NAV_REACHED, EventType.NAV_FAILED],
        )


if __name__ == "__main__":
    unittest.main()
