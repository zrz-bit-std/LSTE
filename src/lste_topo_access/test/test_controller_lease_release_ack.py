#!/usr/bin/env python3
"""Regression tests for the controller-lease release ACK handshake."""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest import mock


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

_ROSPY_STUBBED = False
try:  # Prefer the real ROS runtime when the test environment provides it.
    import rospy  # noqa: F401
except Exception:  # pragma: no cover - headless unit-test environment
    _ROSPY_STUBBED = True
    sys.modules["rospy"] = SimpleNamespace(
        loginfo=lambda *_args, **_kwargs: None,
        logwarn=lambda *_args, **_kwargs: None,
        logerr=lambda *_args, **_kwargs: None,
        is_shutdown=lambda: True,
        Timer=lambda *_args, **_kwargs: None,
        Duration=lambda *_args, **_kwargs: None,
        get_param=lambda *_args, **_kwargs: None,
        ServiceProxy=lambda *_args, **_kwargs: None,
        Subscriber=lambda *_args, **_kwargs: None,
        Publisher=lambda *_args, **_kwargs: None,
    )

from global_frontier_terminal_replan import (  # noqa: E402
    GlobalFrontierTerminalReplanMixin,
)
from global_frontier_planning_runtime import (  # noqa: E402
    GlobalFrontierPlanningRuntimeMixin,
)

# Import-time only. Leaving a stub ``rospy`` behind would break every later
# module in the same pytest process.
if _ROSPY_STUBBED:
    sys.modules.pop("rospy", None)


LIFECYCLE = 2585005248705681157859272160041197236


def _contract(route_id=61, transaction_id=65, action_generation=121,
              terminal_received=False):
    """The predecessor contract this node holds before any ACK arrives."""
    return {
        "route_id": route_id,
        "route_kind": "frontier_endpoint",
        "transaction_id": transaction_id,
        "action_generation": action_generation,
        "lifecycle_transaction_id": LIFECYCLE,
        "graph_transaction_id": 106,
        "map_epoch": 813,
        "origin_epoch": 813,
        "projection_epoch": 813,
        "terminal_received": terminal_received,
        "controller_pending": True,
    }


def _ack(route_id=61, transaction_id=71, action_generation=125,
         controller_lease="released"):
    """The bridge watchdog ACK, carrying the post-adoption identity."""
    released = {
        "route_id": route_id,
        "route_kind": "frontier_endpoint",
        "transaction_id": transaction_id,
        "action_generation": action_generation,
        "lifecycle_transaction_id": LIFECYCLE,
        "graph_transaction_id": 106,
        "map_epoch": 813,
        "origin_epoch": 813,
        "projection_epoch": 813,
        "terminal_received": True,
        "controller_pending": True,
    }
    return {
        "released_route_id": route_id,
        "released_route_kind": "frontier_endpoint",
        "route_id": route_id,
        "transaction_id": transaction_id,
        "action_generation": action_generation,
        "lifecycle_transaction_id": LIFECYCLE,
        "graph_transaction_id": 106,
        "map_epoch": 813,
        "origin_epoch": 813,
        "projection_epoch": 813,
        "controller_lease": controller_lease,
        "release_phase": "route_lease_watchdog_expired",
        "released_controller_route": released,
    }


class _Owner(GlobalFrontierTerminalReplanMixin):
    def __init__(self):
        self.status = []
        self.pending_controller_lease_releases = {}
        self.pending_controller_lease_release = None
        self.pending_controller_release_wait_since_wall = {}
        self.awaiting_controller_lease_release_ack = False
        self.last_controller_lease_release_ack = None
        self.last_released_route_id = 0
        self.last_released_route_controller_pending = False
        self.last_planning_wall = 42.0
        self.active_route_id = 0
        self.active_frontier = None
        self.lifecycle_manager = SimpleNamespace(
            current_transaction_id=0,
            enqueue_type=lambda *_args, **_kwargs: None,
        )
        self.immediate_plan_timer = None

    def publish_status(self, event, **fields):
        self.status.append((event, dict(fields)))

    def events(self):
        return [event for event, _fields in self.status]

    def field_of(self, event):
        for name, fields in self.status:
            if name == event:
                return fields
        raise AssertionError("%s not published: %s" % (event, self.events()))


class _CycleOwner(GlobalFrontierPlanningRuntimeMixin, _Owner):
    def _clear_planning_preempt_if_drained(self):
        return False

    def _drain_execution_terminals_locked(self):
        """Terminal ingress is covered elsewhere; this test owns the ACK gate."""
        return None


class ControllerLeaseReleaseAckTest(unittest.TestCase):
    def _owner_with_pending(self, **kwargs):
        owner = _Owner()
        owner.active_route_id = int(kwargs.get("route_id", 61))
        owner._remember_controller_release_contract(_contract(**kwargs))
        return owner

    # ---- A: adoption restamp -------------------------------------------
    def test_watchdog_ack_with_restamped_identity_closes_the_route(self):
        owner = self._owner_with_pending()
        accepted = owner._accept_controller_lease_release_ack(_ack())
        self.assertFalse(accepted)
        self.assertFalse(owner.awaiting_controller_lease_release_ack)
        self.assertEqual(owner.pending_controller_lease_releases, {})
        self.assertIn(
            "controller_lease_release_ack_identity_adopted", owner.events()
        )
        self.assertIn("controller_lease_release_acknowledged", owner.events())
        adopted = owner.field_of("controller_lease_release_ack_identity_adopted")
        self.assertEqual(
            adopted["adopted_fields"], ["transaction_id", "action_generation"]
        )
        self.assertEqual(adopted["expected_identity"]["transaction_id"], 65)
        self.assertEqual(adopted["received_identity"]["transaction_id"], 71)
        self.assertEqual(
            owner.last_controller_lease_release_ack["transaction_id"], 71
        )
        self.assertEqual(
            owner.last_controller_lease_release_ack["action_generation"], 125
        )

    def test_regressing_controller_identity_stays_stale(self):
        owner = self._owner_with_pending()
        owner._accept_controller_lease_release_ack(
            _ack(transaction_id=60, action_generation=119)
        )
        self.assertIn("stale_lease_release_ack", owner.events())
        stale = owner.field_of("stale_lease_release_ack")
        self.assertEqual(
            stale["reason"], "lease_release_ack_transaction_id_mismatch"
        )
        self.assertIn(61, owner.pending_controller_lease_releases)
        self.assertTrue(owner.awaiting_controller_lease_release_ack)

    def test_terminal_backed_contract_keeps_exact_identity(self):
        owner = self._owner_with_pending(terminal_received=True)
        owner._accept_controller_lease_release_ack(_ack())
        self.assertIn("stale_lease_release_ack", owner.events())
        self.assertNotIn(
            "controller_lease_release_ack_identity_adopted", owner.events()
        )
        self.assertIn(61, owner.pending_controller_lease_releases)

    def test_ack_for_another_route_is_still_rejected(self):
        owner = self._owner_with_pending()
        owner._accept_controller_lease_release_ack(_ack(route_id=62))
        stale = owner.field_of("stale_lease_release_ack")
        self.assertEqual(stale["reason"], "no_pending_controller_release")
        self.assertIn(61, owner.pending_controller_lease_releases)

    # ---- B: wall-time escape valve -------------------------------------
    def test_pending_release_expires_after_the_wall_budget(self):
        owner = self._owner_with_pending()
        self.assertEqual(owner.expire_stale_controller_release_wait(100.0), [])
        self.assertEqual(
            owner.expire_stale_controller_release_wait(129.9), []
        )
        self.assertEqual(owner.expire_stale_controller_release_wait(130.0), [61])
        self.assertFalse(owner.awaiting_controller_lease_release_ack)
        self.assertEqual(owner.pending_controller_lease_releases, {})
        self.assertEqual(
            owner.pending_controller_release_wait_since_wall, {}
        )
        desynced = owner.field_of("controller_lease_release_desynced")
        self.assertEqual(desynced["reason"], "controller_release_ack_wall_timeout")
        self.assertEqual(desynced["route_id"], 61)
        self.assertEqual(desynced["budget_seconds"], 30.0)
        self.assertEqual(desynced["wall_seconds"], 30.0)

    def test_configured_budget_is_honoured(self):
        owner = self._owner_with_pending()
        owner.controller_release_ack_wall_seconds = 12.0
        self.assertEqual(owner.expire_stale_controller_release_wait(100.0), [])
        self.assertEqual(owner.expire_stale_controller_release_wait(111.9), [])
        self.assertEqual(owner.expire_stale_controller_release_wait(112.0), [61])

    def test_matching_ack_clears_the_wall_anchor(self):
        owner = self._owner_with_pending()
        self.assertEqual(owner.expire_stale_controller_release_wait(100.0), [])
        owner._accept_controller_lease_release_ack(_ack())
        self.assertEqual(owner.expire_stale_controller_release_wait(200.0), [])
        self.assertEqual(
            owner.pending_controller_release_wait_since_wall, {}
        )
        self.assertNotIn("controller_lease_release_desynced", owner.events())

    def test_expiry_unblocks_the_successor_planning_cycle(self):
        owner = _CycleOwner()
        owner.active_route_id = 61
        owner._remember_controller_release_contract(_contract())
        _token, skip = owner._prepare_planning_cycle()
        self.assertTrue(skip)
        self.assertIn("controller_lease_release_wait", owner.events())
        # The bridge cleaned the route up and will never send another ACK.
        owner.pending_controller_release_wait_since_wall = {61: 100.0}
        import global_frontier_terminal_replan as replan_module

        with mock.patch.object(
            replan_module.time, "monotonic", return_value=131.0
        ):
            _token, skip = owner._prepare_planning_cycle()
        self.assertFalse(skip)
        self.assertFalse(owner.awaiting_controller_lease_release_ack)
        self.assertEqual(owner.pending_controller_lease_releases, {})


if __name__ == "__main__":
    unittest.main()
