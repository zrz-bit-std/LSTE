#!/usr/bin/env python3
"""Regression tests for global-frontier status context."""

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_reporting import GlobalFrontierReportingMixin


class _RegionMemory:
    def __init__(self, regions):
        self.regions = regions

    def by_id(self, region_id):
        return self.regions.get(region_id)


class GlobalFrontierReportingTest(unittest.TestCase):
    def test_active_region_report_uses_durable_memory_identity(self):
        reporter = GlobalFrontierReportingMixin()
        reporter.active_frontier_region_id = 7
        reporter.region_memory = _RegionMemory({7: {"state": "open"}})

        self.assertEqual(reporter.active_region_report(), (7, "open"))

    def test_active_region_report_handles_unassigned_and_retired_regions(self):
        reporter = GlobalFrontierReportingMixin()
        reporter.region_memory = _RegionMemory({})
        reporter.active_frontier_region_id = None
        self.assertEqual(reporter.active_region_report(), (None, None))

        reporter.active_frontier_region_id = 11
        self.assertEqual(reporter.active_region_report(), (11, None))

    def test_goal_context_marks_place_owned_observation_work(self):
        reporter = GlobalFrontierReportingMixin()
        reporter.place_memory_enabled = True
        reporter.exploration_method = "place_portal_workitem"
        reporter.active_route_kind = "frontier_connector"
        reporter.active_frontier_region_id = 7
        reporter.current_physical_place_id = 7
        reporter.current_task_id = "yellow_cup"
        reporter.current_mission_id = "yellow_cup"
        reporter.current_task_version = "version-a"
        reporter.active_work_item_id = 13
        reporter.place_departure = type("Departure", (), {"active": False})()

        context = reporter.active_goal_context()

        self.assertEqual(context["goal_role"], "place_observation_work")
        self.assertEqual(context["owner_place_id"], 7)
        self.assertEqual(context["source_place_id"], 7)
        self.assertEqual(context["work_item_id"], 13)
        self.assertEqual(context["task_id"], "yellow_cup")
        self.assertEqual(context["task_version"], "version-a")

    def test_goal_context_keeps_geometry_baseline_free_of_place_state(self):
        reporter = GlobalFrontierReportingMixin()
        reporter.place_memory_enabled = False
        reporter.exploration_method = "frontier"
        reporter.active_route_kind = "frontier_endpoint"
        reporter.active_frontier_region_id = 7
        reporter.current_physical_place_id = 7
        reporter.active_work_item_id = 13

        context = reporter.active_goal_context()

        self.assertEqual(context["goal_role"], "geometry_frontier")
        self.assertEqual(context["exploration_method"], "frontier")
        self.assertIsNone(context["owner_place_id"])
        self.assertIsNone(context["source_place_id"])
        self.assertIsNone(context["work_item_id"])

    def test_goal_context_marks_a_doorway_as_a_certified_transition(self):
        reporter = GlobalFrontierReportingMixin()
        reporter.place_memory_enabled = True
        reporter.exploration_method = "place_portal_workitem"
        reporter.active_route_kind = "portal_transition"
        reporter.active_frontier_region_id = None
        reporter.current_physical_place_id = 3
        reporter.active_work_item_id = None
        reporter.active_portal_gate_odom_xy = (1.23456, -2.34567)
        reporter.place_departure = type(
            "Departure", (), {"active": True, "region_id": 3, "transit_only": False}
        )()

        context = reporter.active_goal_context()

        self.assertEqual(context["goal_role"], "certified_portal_crossing")
        self.assertEqual(context["source_place_id"], 3)
        self.assertTrue(context["portal_crossing_certified"])
        self.assertEqual(context["portal_gate_odom"], [1.2346, -2.3457])


if __name__ == "__main__":
    unittest.main()
