"""A controller-owned disconnected route must not be held forever.

Six of the eight stalled attempts in the last round spent 174-187 s printing

    Global frontier route is temporarily disconnected; retaining
    controller-owned lease route_id=.. held=178.3s

The handler returned the last safe waypoint for as long as
``controller_owned_route_failure`` stayed true, but the failure resolution that
publishes the exact lease ACK only runs *after* the route is released.  The
hold is now bounded by ``controller_failure_hold_seconds``; on expiry it falls
through to the existing abandon path, which retires the lease and lets the
planner pick a successor.
"""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

# The handler logs before any node exists; a bare import of rospy raises
# ROSInitException from logwarn without init_node().
sys.modules.setdefault(
    "rospy",
    SimpleNamespace(
        loginfo=lambda *_a, **_k: None,
        logwarn=lambda *_a, **_k: None,
        logwarn_throttle=lambda *_a, **_k: None,
        logerr=lambda *_a, **_k: None,
    ),
)

from global_frontier_execution_routes import (  # noqa: E402
    GlobalFrontierExecutionRouteMixin,
)


class Fixture(GlobalFrontierExecutionRouteMixin):
    def __init__(self, controller_failure=True, reason=""):
        self.active_unreachable_since = None
        self.controller_owned_route_failure = controller_failure
        self.controller_failure_hold_seconds = 20.0
        self.unreachable_grace = 5.0
        self.active_route_id = 52
        self.active_route_kind = "frontier_endpoint"
        self.active_last_waypoint_map = (1, 2, 3.0, 4.0)
        self.active_best_path_distance = None
        self.active_best_goal_distance = None
        self.active_last_progress_signal = None
        self.recovery_pending_reason = reason
        self.events = []
        self.released = []
        self.settled = []

    def active_waypoint_distance(self, robot_map):
        return 2.0

    def publish_status(self, event, **fields):
        self.events.append((event, fields))

    def release_active_frontier(self, **kwargs):
        self.released.append(kwargs)

    def settle_active_work_item(self, now, state, reason):
        self.settled.append((state, reason))


def route(held_from=0.0, now=0.0):
    return (0, 0, 5.0, 6.0)


class HoldBoundTest(unittest.TestCase):
    def test_hold_is_returned_inside_the_bound(self):
        fixture = Fixture()
        waypoint = fixture.handle_disconnected_active_frontier(
            0, 0, 5.0, 6.0, (0.0, 0.0), 0.0
        )
        self.assertIsNotNone(waypoint[1])
        held = fixture.handle_disconnected_active_frontier(
            0, 0, 5.0, 6.0, (0.0, 0.0), 19.9
        )
        self.assertIsNotNone(held[1])
        self.assertEqual(fixture.released, [])
        self.assertEqual(fixture.events, [])

    def test_the_hold_expires_and_the_route_is_released(self):
        fixture = Fixture(reason="controller_stall_no_physical_progress")
        fixture.handle_disconnected_active_frontier(0, 0, 5.0, 6.0, (0.0, 0.0), 0.0)
        self.assertEqual(
            fixture.handle_disconnected_active_frontier(
                0, 0, 5.0, 6.0, (0.0, 0.0), 20.1
            ),
            (None, None),
        )
        self.assertEqual(len(fixture.released), 1)
        self.assertEqual(
            [event for event, _ in fixture.events], ["route_invalidated"]
        )
        self.assertEqual(
            fixture.events[0][1]["reason"], "controller_stall_no_physical_progress"
        )
        self.assertEqual(fixture.settled[0][1], "disconnected")

    def test_the_bound_is_configurable(self):
        fixture = Fixture()
        fixture.controller_failure_hold_seconds = 2.0
        fixture.handle_disconnected_active_frontier(0, 0, 5.0, 6.0, (0.0, 0.0), 0.0)
        self.assertIsNotNone(
            fixture.handle_disconnected_active_frontier(
                0, 0, 5.0, 6.0, (0.0, 0.0), 1.9
            )[1]
        )
        # Past the controller bound the ordinary grace must not re-hold it.
        self.assertEqual(
            fixture.handle_disconnected_active_frontier(
                0, 0, 5.0, 6.0, (0.0, 0.0), 2.1
            ),
            (None, None),
        )
        self.assertEqual(len(fixture.released), 1)

    def test_a_zero_bound_never_holds(self):
        fixture = Fixture()
        fixture.controller_failure_hold_seconds = 0.0
        fixture.handle_disconnected_active_frontier(0, 0, 5.0, 6.0, (0.0, 0.0), 0.0)
        self.assertEqual(
            fixture.handle_disconnected_active_frontier(
                0, 0, 5.0, 6.0, (0.0, 0.0), 0.1
            ),
            (None, None),
        )


class OrdinaryDisconnectionTest(unittest.TestCase):
    """The non-controller path keeps its own short grace."""

    def test_ordinary_disconnection_still_uses_unreachable_grace(self):
        fixture = Fixture(controller_failure=False)
        fixture.handle_disconnected_active_frontier(0, 0, 5.0, 6.0, (0.0, 0.0), 0.0)
        # Grace is 5 s, so 20 s later it must abandon, not hold.
        self.assertEqual(
            fixture.handle_disconnected_active_frontier(
                0, 0, 5.0, 6.0, (0.0, 0.0), 20.0
            ),
            (None, None),
        )
        self.assertEqual(fixture.events[0][1]["reason"], "disconnected")


class SourceContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (SCRIPTS / "global_frontier_execution_routes.py").read_text(
            encoding="utf-8"
        )

    def test_controller_hold_is_bounded_before_the_abandon_path(self):
        self.assertIn("controller_failure_hold_seconds", self.source)
        hold = self.source.index("hold_limit = float(")
        abandon = self.source.index("self.release_active_frontier(discard_prefetch=True)")
        self.assertLess(hold, abandon)


if __name__ == "__main__":
    unittest.main(verbosity=2)
