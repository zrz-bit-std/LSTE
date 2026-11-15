#!/usr/bin/env python3
"""Source-Place identity checks for Portal routing and DFS entry."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_activation import (  # noqa: E402
    portal_source_place_mismatch,
    rebind_stale_portal_departure,
)
from global_frontier_place_dfs import PlaceDfsTraversal  # noqa: E402
from global_frontier_portal_arrival_commit import (  # noqa: E402
    resolve_portal_dfs_entry,
)


class PortalSourcePlaceMismatchTest(unittest.TestCase):
    def test_stale_departure_source_is_rejected(self):
        self.assertTrue(portal_source_place_mismatch(3, 2, 3))

    def test_matching_sources_are_allowed(self):
        self.assertFalse(portal_source_place_mismatch(3, 3, 3))

    def test_portal_ledger_mismatch_is_rejected(self):
        self.assertTrue(portal_source_place_mismatch(3, 3, 4))

    def test_reverse_crossing_from_recorded_destination_is_allowed(self):
        self.assertFalse(portal_source_place_mismatch(2, 2, 1, 2))

    def test_portal_without_current_place_endpoint_is_rejected(self):
        self.assertTrue(portal_source_place_mismatch(3, 3, 1, 2))

    def test_stale_departure_inside_portal_endpoints_is_still_rejected(self):
        self.assertTrue(portal_source_place_mismatch(3, 2, 3, 4))

    def test_missing_departure_falls_back_to_current_place(self):
        self.assertFalse(portal_source_place_mismatch(3, None, 3))

    def test_unknown_current_place_is_not_judged(self):
        self.assertFalse(portal_source_place_mismatch(None, 2, 3))

    def test_unknown_sources_are_not_judged(self):
        self.assertFalse(portal_source_place_mismatch(3, None, None))

    def test_zero_valued_sources_are_ignored(self):
        self.assertFalse(portal_source_place_mismatch(3, 0, 0))


class StalePortalDepartureRebindTest(unittest.TestCase):
    class Explorer:
        def __init__(self):
            self.calls = []
            self.place_departure = SimpleNamespace(region_id=2)

        def prepare_covered_place_transit(self, robot_map, place_hops):
            self.calls.append((robot_map, place_hops))
            self.place_departure.region_id = 3
            return {"id": 3}

    def test_stale_departure_rebinds_to_current_portal_endpoint(self):
        explorer = self.Explorer()

        source = rebind_stale_portal_departure(
            explorer, 3, 2, 3, 5, (1.0, 2.0), 1,
        )

        self.assertEqual(source, 3)
        self.assertEqual(explorer.place_departure.region_id, 3)
        self.assertEqual(explorer.calls, [((1.0, 2.0), 1)])

    def test_current_place_outside_portal_endpoints_stays_rejected(self):
        explorer = self.Explorer()

        source = rebind_stale_portal_departure(
            explorer, 3, 2, 4, 5, (1.0, 2.0), 1,
        )

        self.assertEqual(source, 2)
        self.assertEqual(explorer.calls, [])


class PortalDfsEntryResolutionTest(unittest.TestCase):
    def make_dfs(self):
        dfs = PlaceDfsTraversal()
        self.assertTrue(dfs.bootstrap(1))
        self.assertTrue(dfs.enter(1, 3, 2))
        return dfs

    def test_stale_arrival_source_uses_active_room(self):
        dfs = self.make_dfs()

        entered, parent = resolve_portal_dfs_entry(
            dfs, 2, 4, 4, 3, 3,
        )

        self.assertTrue(entered)
        self.assertEqual(parent, 3)
        self.assertEqual(dfs.snapshot()["stack"], [1, 3, 4])

    def test_direct_parent_is_used_when_available(self):
        dfs = self.make_dfs()

        entered, parent = resolve_portal_dfs_entry(
            dfs, 3, 4, 4, 3, 3,
        )

        self.assertTrue(entered)
        self.assertEqual(parent, 3)
        self.assertEqual(dfs.snapshot()["stack"], [1, 3, 4])

    def test_mismatched_active_room_does_not_enter(self):
        dfs = self.make_dfs()

        entered, parent = resolve_portal_dfs_entry(
            dfs, 2, 4, 4, 2, 3,
        )

        self.assertFalse(entered)
        self.assertEqual(parent, 2)
        self.assertEqual(dfs.snapshot()["stack"], [1, 3])

    def test_reverse_crossing_uses_the_active_destination_room(self):
        dfs = PlaceDfsTraversal()
        self.assertTrue(dfs.bootstrap(1))
        self.assertTrue(dfs.enter(1, 2, 3))

        entered, parent = resolve_portal_dfs_entry(
            dfs, 1, 3, 3, 2, 1, 2,
        )

        self.assertTrue(entered)
        self.assertEqual(parent, 2)
        self.assertEqual(dfs.snapshot()["stack"], [1, 2, 3])

    def test_missing_dfs_still_reports_the_source(self):
        entered, parent = resolve_portal_dfs_entry(
            None, 2, 4, 4, 3, 3,
        )

        self.assertTrue(entered)
        self.assertEqual(parent, 2)


if __name__ == "__main__":
    unittest.main()
