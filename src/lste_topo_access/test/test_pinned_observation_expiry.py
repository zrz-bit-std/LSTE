"""A pinned portal observation period must expire even out of Place scope.

Run ``20261101_002732`` froze for 180 s at the north-room endpoint.  Node 17's
observation task kept two branches in ``pending`` with
``active_branch_id = None`` and ``phase = watching``; nothing could start them
because the branches had no source pose.  ``portal_authentication_node_to_pin``
then pinned ``tree.current_node_id`` to 17 for as long as the phase stayed
watching, so the tree never advanced and the graph dispatched nothing.

The bounded expiry existed, but it lived after the Place-ownership guards in
``_sync_portal_authentication``.  Once the active room no longer owned the
pinned node those guards returned early, so the expiry never ran.  The expiry
now runs first, from ``_expire_stranded_portal_authentication``.
"""

from pathlib import Path
import sys
import unittest
from types import SimpleNamespace


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_portal_authentication import (  # noqa: E402
    DoorwayAuthentication,
    JunctionDoorwayAuthentication,
)
from gp_branch_frontier import GPBranchFrontierMixin  # noqa: E402


TIMEOUT = 30.0


def pinned_task(branch_ids=(46, 47)):
    task = JunctionDoorwayAuthentication(17, settle_seconds=1.0)
    for branch_id in branch_ids:
        branch = DoorwayAuthentication(branch_id, 17, (0.0, 0.0), (1.0, 0.0))
        task.add(branch, now=0.0)
    task.phase = "watching"
    return task


class Fixture(GPBranchFrontierMixin):
    def __init__(self, task, node_id=17, active_room_node=12):
        self.junction_portal_authentication = task
        self.portal_authentication_by_node = {17: task}
        self.portal_transaction = None
        self.place_work_items = None
        self.current_physical_place_id = 4
        self.current_place_scope_id = 4
        self.active_semantic_room = {"node_id": active_room_node}
        self.active_frontier = None
        self.awaiting_controller_lease_release_ack = False
        self.pending_controller_lease_releases = {}
        self.portal_authentication_history = []
        self.active_portal_authentication = lambda: None
        self.events = []
        self.tree = SimpleNamespace(
            current_node_id=node_id,
            nodes={
                node_id: {"id": node_id, "physical_xy": None,
                          "branch_ids": [46, 47]},
            },
            branches={46: {"id": 46}, 47: {"id": 47}},
        )

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


class PinnedObservationExpiryTest(unittest.TestCase):
    def test_out_of_scope_pinned_task_is_expired(self):
        task = pinned_task()
        fixture = Fixture(task)
        # The Place no longer owns node 17, so the ownership guards return
        # early; the expiry must still run and conclude the task.
        fixture._sync_portal_authentication(fixture.tree, None, None, 100.0)
        self.assertEqual(task.phase, "watching")
        fixture._sync_portal_authentication(
            fixture.tree, None, None, 100.0 + TIMEOUT + 1.0
        )
        self.assertTrue(task.all_finished(), task.snapshot())
        self.assertEqual(task.phase, "complete")
        self.assertNotIn(task.phase, ("watching", "authenticating"))

    def test_expiry_publishes_the_bounded_conclusion(self):
        task = pinned_task()
        fixture = Fixture(task)
        fixture._expire_stranded_portal_authentication(fixture.tree, None, 10.0)
        fixture._expire_stranded_portal_authentication(
            fixture.tree, None, 10.0 + TIMEOUT + 1.0
        )
        events = [event for event, _fields in fixture.events]
        self.assertIn("portal_authentication_observation_expired", events)
        fields = dict(fixture.events[-1][1])
        self.assertEqual(fields["node_id"], 17)
        self.assertEqual(sorted(fields["branch_ids"]), [46, 47])
        self.assertEqual(fields["reason"], "pinned_observation_unstartable")

    def test_tree_branches_are_marked_failed(self):
        task = pinned_task()
        fixture = Fixture(task)
        fixture._expire_stranded_portal_authentication(fixture.tree, None, 0.0)
        fixture._expire_stranded_portal_authentication(
            fixture.tree, None, TIMEOUT + 1.0
        )
        for branch_id in (46, 47):
            record = fixture.tree.branches[branch_id]
            self.assertEqual(record["authentication_state"], "failed")
            self.assertEqual(
                record["authentication_failure_reason"],
                "observation_geometry_unavailable",
            )

    def test_active_branch_is_never_expired(self):
        task = pinned_task()
        task.active_branch_id = 46
        fixture = Fixture(task)
        self.assertEqual(
            fixture._expire_stranded_portal_authentication(
                fixture.tree, None, 10_000.0
            ),
            [],
        )
        self.assertIn(46, task.branches)

    def test_finished_task_is_left_alone(self):
        task = pinned_task()
        for branch in task.branches.values():
            branch.state = "failed"
        fixture = Fixture(task)
        self.assertEqual(
            fixture._expire_stranded_portal_authentication(
                fixture.tree, None, 10_000.0
            ),
            [],
        )
        self.assertEqual(fixture.events, [])


class PerCycleExpiryTest(unittest.TestCase):
    """The planning cycle must conclude a pinned task, not only discovery."""

    def test_physical_progress_check_expires_a_watching_task(self):
        task = pinned_task()
        fixture = Fixture(task)
        fixture.pose_odom = SimpleNamespace(x=0.0, y=0.0, theta=0.0)
        fixture.map_msg = None
        # No active branch: active_portal_authentication() returns None, so the
        # old early-return skipped the expiry entirely.
        self.assertFalse(fixture.check_portal_authentication_physical_progress(100.0))
        self.assertEqual(task.phase, "watching")
        self.assertTrue(
            fixture.check_portal_authentication_physical_progress(100.0 + TIMEOUT + 1.0)
        )
        self.assertTrue(task.all_finished())
        self.assertEqual(task.phase, "complete")
        self.assertIn(
            "portal_authentication_observation_expired",
            [event for event, _fields in fixture.events],
        )


class CallerOrderingTest(unittest.TestCase):
    """The expiry must precede the scope guards that used to skip it."""

    @classmethod
    def setUpClass(cls):
        cls.source = (SCRIPTS / "gp_branch_frontier.py").read_text(
            encoding="utf-8"
        )

    def test_expiry_runs_before_the_place_ownership_guard(self):
        expire_at = self.source.index(
            "self._expire_stranded_portal_authentication(tree, message, now)"
        )
        guard_at = self.source.index("        if active_room is not None:")
        self.assertLess(expire_at, guard_at)

    def test_expiry_is_bounded_by_the_observation_timeout(self):
        self.assertIn("portal_observation_start_timeout", self.source)
        self.assertIn("expire_unstartable(now, timeout)", self.source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
