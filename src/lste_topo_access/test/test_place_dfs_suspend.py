"""A physical return must move the DFS cursor, not freeze it.

env04 ``20261113_024720``: Place 2 was entered, its local WorkItem 14 stayed
unresolved, and the robot crossed back into Place 1.  ``_commit_semantic_room_
return`` correctly refused to *close* the room, but it also left the DFS stack
at ``[1, 2]``.  Every later ``enter(source, ...)`` then failed on
``stack[-1] != source`` while the planner kept crossing new doorways, so Places
3..6 existed only as observed destinations ("obs") and were never committed.

``PlaceDfsTraversal.suspend`` moves the cursor back to the parent while keeping
the room unfinished: its WorkItems, the pending resume and the Portal are
untouched, so it can be entered again to finish the work.
"""
from pathlib import Path
import sys
import unittest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(TEST_DIR := str(Path(__file__).resolve().parent)) not in sys.path:
    sys.path.insert(0, TEST_DIR)
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_place_dfs import PlaceDfsTraversal  # noqa: E402
from global_frontier_portal_arrival_commit import (  # noqa: E402
    GlobalFrontierPortalArrivalCommitMixin,
)
from global_frontier_work_items import PlaceWorkItemLedger  # noqa: E402


class SuspendTest(unittest.TestCase):
    """``suspend`` only moves the cursor for the exact physical edge."""

    def _dfs(self):
        dfs = PlaceDfsTraversal()
        dfs.bootstrap(1)
        self.assertTrue(dfs.enter(1, 2, 2))
        self.assertEqual(dfs.stack, [1, 2])
        return dfs

    def test_suspend_pops_the_child_without_completing_it(self):
        dfs = self._dfs()
        self.assertTrue(dfs.suspend(2, 1, 2))
        self.assertEqual(dfs.stack, [1])
        self.assertEqual(dfs.nodes[2]["state"], "suspended")
        self.assertNotEqual(dfs.nodes[2]["state"], "completed")
        self.assertEqual(dfs.nodes[2]["physical_exit"], "incomplete_return")

    def test_suspended_room_can_be_entered_again(self):
        dfs = self._dfs()
        self.assertTrue(dfs.suspend(2, 1, 2))
        self.assertTrue(dfs.enter(1, 2, 2))
        self.assertEqual(dfs.stack, [1, 2])

    def test_wrong_parent_or_portal_is_refused(self):
        dfs = self._dfs()
        self.assertFalse(dfs.suspend(2, 9, 2))
        self.assertFalse(dfs.suspend(2, 1, 9))
        self.assertEqual(dfs.stack, [1, 2])
        self.assertEqual(dfs.nodes[2]["state"], "exploring")

    def test_non_top_and_empty_stacks_are_refused(self):
        dfs = self._dfs()
        self.assertTrue(dfs.enter(2, 3, 7))
        self.assertEqual(dfs.stack, [1, 2, 3])
        self.assertFalse(dfs.suspend(2, 1, 2))
        self.assertEqual(dfs.stack, [1, 2, 3])
        empty = PlaceDfsTraversal()
        self.assertFalse(empty.suspend(1))


class IncompleteReturnHarness(GlobalFrontierPortalArrivalCommitMixin):
    def __init__(self):
        self.place_dfs = PlaceDfsTraversal()
        self.place_work_items = PlaceWorkItemLedger(merge_radius=0.5)
        self.active_semantic_rooms = []
        self.active_semantic_room = None
        self.events = []
        self.pending_graph_resume_place_id = None
        self.pending_graph_resume_portal_id = None
        self.pending_graph_resume_destination_place_id = None

    def publish_status(self, name, **fields):
        self.events.append((name, fields))


class IncompleteReturnTest(unittest.TestCase):
    """The reported run: return with an open WorkItem must not freeze the DFS."""

    def _fixture(self):
        host = IncompleteReturnHarness()
        host.place_dfs.bootstrap(1)
        self.assertTrue(host.place_dfs.enter(1, 2, 2))
        item = host.place_work_items._new_item(2, (), 0.0)
        room = {
            "node_id": 7, "branch_id": 4, "portal_id": 2,
            "place_id": 2, "source_place_id": 1,
        }
        host.active_semantic_rooms = [room]
        host.active_semantic_room = room
        return host, room, item

    def test_return_moves_the_cursor_and_keeps_the_work_open(self):
        host, room, item = self._fixture()
        host._commit_semantic_room_return(
            room, 2, 1, 5.0, None, None, host.active_semantic_rooms,
        )

        self.assertEqual(host.pending_graph_resume_place_id, 2)
        self.assertEqual(host.place_dfs.stack, [1])
        self.assertEqual(host.place_dfs.nodes[2]["state"], "suspended")
        self.assertEqual(host.active_semantic_rooms, [])
        self.assertIsNone(host.active_semantic_room)
        # Nothing about the room is closed or suppressed by the move.
        self.assertEqual(host.place_work_items.unresolved_ids(2), [item["id"]])
        names = [name for name, _fields in host.events]
        self.assertIn("semantic_room_return_incomplete", names)
        self.assertIn("place_dfs_suspended_incomplete", names)
        self.assertNotIn("semantic_room_closed", names)
        self.assertNotIn("place_dfs_returned", names)

    def test_a_later_doorway_can_now_be_committed(self):
        host, room, _item = self._fixture()
        host._commit_semantic_room_return(
            room, 2, 1, 5.0, None, None, host.active_semantic_rooms,
        )
        # Placement 3 is the next physical crossing from Place 1 (024720 shape).
        self.assertTrue(host.place_dfs.enter(1, 3, 1))
        self.assertEqual(host.place_dfs.stack, [1, 3])

    def test_wrong_return_edge_leaves_everything_untouched(self):
        host, room, _item = self._fixture()
        host._commit_semantic_room_return(
            room, 99, 1, 5.0, None, None, host.active_semantic_rooms,
        )
        self.assertEqual(host.place_dfs.stack, [1, 2])
        self.assertEqual(host.active_semantic_rooms, [room])
        self.assertIs(host.active_semantic_room, room)
        names = [name for name, _fields in host.events]
        self.assertNotIn("place_dfs_suspended_incomplete", names)


class RejectedReturnTest(unittest.TestCase):
    """env04 20261113_154448: the room was done but the DFS was not "returning".

    The physical return arrived 10 s after entry, before the door-owned exit was
    dispatched, so ``return_to_parent`` refused (node state "exploring") and the
    cursor stayed on the child.  The east doorway afterwards could not be
    committed and became another "observed, not committed" Place.
    """

    def _fixture(self):
        host = IncompleteReturnHarness()
        host.place_dfs.bootstrap(1)
        self.assertTrue(host.place_dfs.enter(1, 2, 2))
        room = {
            "node_id": 7, "branch_id": 4, "portal_id": 2,
            "place_id": 2, "source_place_id": 1,
        }
        host.active_semantic_rooms = [room]
        host.active_semantic_room = room
        return host, room

    def test_rejected_return_still_moves_the_cursor(self):
        host, room = self._fixture()
        self.assertEqual(host.place_dfs.nodes[2]["state"], "exploring")
        host._commit_semantic_room_return(
            room, 2, 1, 5.0, None, None, host.active_semantic_rooms,
        )
        self.assertEqual(host.place_dfs.stack, [1])
        self.assertEqual(host.place_dfs.nodes[2]["state"], "suspended")
        self.assertEqual(host.active_semantic_rooms, [])
        self.assertIsNone(host.active_semantic_room)
        names = [name for name, _fields in host.events]
        self.assertIn("place_dfs_suspended_return_rejected", names)
        self.assertNotIn("place_dfs_return_rejected", names)
        self.assertNotIn("semantic_room_closed", names)

    def test_wrong_edge_is_still_rejected(self):
        host, room = self._fixture()
        host._commit_semantic_room_return(
            room, 99, 1, 5.0, None, None, host.active_semantic_rooms,
        )
        self.assertEqual(host.place_dfs.stack, [1, 2])
        self.assertEqual(host.active_semantic_rooms, [room])
        self.assertIs(host.active_semantic_room, room)
        names = [name for name, _fields in host.events]
        self.assertIn("place_dfs_return_rejected", names)
        self.assertNotIn("place_dfs_suspended_return_rejected", names)

    def test_next_doorway_commits_after_the_repair(self):
        host, room = self._fixture()
        host._commit_semantic_room_return(
            room, 2, 1, 5.0, None, None, host.active_semantic_rooms,
        )
        # 20261113_154448: portal 7 into the east room was crossed 10 s later.
        self.assertTrue(host.place_dfs.enter(1, 3, 7))
        self.assertEqual(host.place_dfs.stack, [1, 3])


if __name__ == "__main__":
    unittest.main()
