#!/usr/bin/env python3
"""Regression tests for global-frontier status context."""

from pathlib import Path
from unittest.mock import patch
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from global_frontier_reporting import GlobalFrontierReportingMixin
from global_frontier_event_callbacks import GlobalFrontierEventCallbacksMixin


class _RegionMemory:
    def __init__(self, regions):
        self.regions = regions

    def by_id(self, region_id):
        return self.regions.get(region_id)


class GlobalFrontierReportingTest(unittest.TestCase):
    def test_validated_terminal_advances_pending_release_mission_transaction(self):
        callbacks = GlobalFrontierEventCallbacksMixin()
        callbacks.active_route_id = 23
        payload = dict(event="dispatch", route_id=23, action_generation=31,
            transaction_id=23, route_kind="frontier_turn_connector",
            mission_route_kind="frontier_endpoint", source="global_slam_frontier",
            priority=0, lifecycle_transaction_id=55, graph_transaction_id=23,
            map_epoch=112, origin_epoch=112, projection_epoch=112)
        expected = callbacks._remember_bridge_dispatch_contract(payload)
        callbacks.pending_controller_lease_release = dict(expected, route_kind="frontier_endpoint")
        callbacks.last_released_route_id = 23
        callbacks.last_released_route_transaction_id = 23
        terminal = callbacks._bridge_status_contract(dict(payload, event="terminal", transaction_id=24))
        self.assertEqual(callbacks._bridge_terminal_matches_dispatch({},terminal), (True,"matched"))
        self.assertEqual(callbacks.pending_controller_lease_release["transaction_id"],24)
        self.assertEqual(callbacks.last_released_route_transaction_id,24)
        self.assertEqual(callbacks.active_route_dispatch_contract["transaction_id"],24)

    def test_wrong_epoch_terminal_cannot_advance_any_transaction(self):
        callbacks = GlobalFrontierEventCallbacksMixin()
        callbacks.active_route_id = 23
        payload = dict(event="dispatch", route_id=23, action_generation=31,
            transaction_id=23, route_kind="frontier_turn_connector",
            mission_route_kind="frontier_endpoint", source="global_slam_frontier",
            priority=0, lifecycle_transaction_id=55, graph_transaction_id=23,
            map_epoch=112, origin_epoch=112, projection_epoch=112)
        callbacks._remember_bridge_dispatch_contract(payload)
        callbacks.pending_controller_lease_release = dict(callbacks.active_route_dispatch_contract)
        for field in ("route_id","action_generation","lifecycle_transaction_id",
                      "graph_transaction_id","map_epoch","origin_epoch","projection_epoch"):
            terminal = callbacks._bridge_status_contract(dict(payload, event="terminal", transaction_id=24,
                                                               **{field:999}))
            self.assertFalse(callbacks._bridge_terminal_matches_dispatch({},terminal)[0])
            self.assertEqual(callbacks._bridge_dispatch_contracts[23]["transaction_id"],23)
            self.assertEqual(callbacks.pending_controller_lease_release["transaction_id"],23)

    def test_delayed_adoption_advances_same_pending_release_only(self):
        callbacks = GlobalFrontierEventCallbacksMixin()
        callbacks.active_route_id = 23
        payload = dict(event="dispatch", route_id=23, action_generation=31,
            transaction_id=23, route_kind="frontier_endpoint", source="global_slam_frontier",
            priority=0, lifecycle_transaction_id=55, graph_transaction_id=23,
            map_epoch=112, origin_epoch=112, projection_epoch=112)
        expected=callbacks._remember_bridge_dispatch_contract(payload)
        callbacks.pending_controller_lease_release=dict(expected)
        callbacks.last_released_route_id=23
        callbacks._remember_bridge_dispatch_contract(dict(payload,event="persistent_mission_path_adopted",transaction_id=24))
        self.assertEqual(callbacks.pending_controller_lease_release["transaction_id"],24)
        callbacks.pending_controller_lease_release["route_id"]=22
        callbacks._remember_bridge_dispatch_contract(dict(payload,transaction_id=25))
        self.assertEqual(callbacks.pending_controller_lease_release["transaction_id"],24)

    def test_turn_connector_dispatch_keeps_exact_release_identity(self):
        class Callbacks(GlobalFrontierEventCallbacksMixin):
            pass

        callbacks = Callbacks()
        callbacks.active_route_id = 3
        callbacks.active_route_dispatch_contract = {
            "route_id": 3,
            "route_kind": "frontier_turn_connector",
            "graph_transaction_id": 3,
            "map_epoch": 23,
            "origin_epoch": 23,
            "projection_epoch": 1,
        }
        payload = {
            "event": "dispatch",
            "route_id": 3,
            "action_generation": 5,
            "transaction_id": 3,
            "route_kind": "frontier_turn_connector",
            "source": "global_slam_frontier",
            "priority": 0,
            "lifecycle_transaction_id": 55,
            "graph_transaction_id": 3,
            "map_epoch": 23,
            "origin_epoch": 23,
            "projection_epoch": 1,
        }

        contract = callbacks._remember_bridge_dispatch_contract(payload)

        self.assertEqual(contract["action_generation"], 5)
        self.assertEqual(contract["projection_epoch"], 1)
        self.assertEqual(callbacks.active_route_dispatch_contract["graph_transaction_id"], 3)

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

    def test_gp_authentication_route_gets_canonical_transaction_identity(self):
        class Publisher:
            def publish(self, message):
                self.message = message

        class Reporter(GlobalFrontierReportingMixin, GlobalFrontierEventCallbacksMixin):
            def active_portal_authentication(self):
                return object()

            def active_goal_context(self, route_kind=None, graph_transaction_id=None):
                return {"graph_transaction_id": int(graph_transaction_id or 0)}

        reporter = Reporter()
        reporter.graph_route_action_transaction = None
        reporter.last_map_epoch = 9
        reporter.active_route_map_epoch = 7
        reporter.last_graph_snapshot_epoch = 191
        reporter.topology_component_epoch = 7
        reporter.active_route_id = 12
        reporter.active_action_generation = 2
        reporter.active_route_kind = "frontier_endpoint"
        reporter.active_transition_kind = ""
        reporter.active_predecessor_route_id = 0
        reporter.active_route_dispatch_contract = None
        reporter.active_transition_distance = None
        reporter.active_portal_probe_phase = ""
        reporter.route_lifecycle_transaction_id = lambda: 55
        reporter.lifecycle_transaction_id = lambda: 55
        reporter.command_publisher = Publisher()

        route_rospy = GlobalFrontierReportingMixin.publish_route_command.__globals__["rospy"]
        with patch.object(route_rospy, "Time", create=True) as clock:
            clock.now.return_value.to_sec.return_value = 1.0
            reporter.publish_route_command("map", 1.0, 2.0, None, "frontier_endpoint")
        import json
        payload = json.loads(reporter.command_publisher.message.data)

        self.assertEqual(payload["graph_transaction_id"], 12)
        self.assertEqual(payload["transaction_id"], 12)
        self.assertEqual(payload["goal_context"]["graph_transaction_id"], 12)
        self.assertEqual(payload["map_epoch"], 7)
        self.assertEqual(payload["origin_epoch"], 7)
        self.assertEqual(payload["projection_epoch"], 7)

        # Real failure: WI graph planning consumed an extra sequence. Activation
        # owns graph53 while the physical route is12. The wire and the local
        # release/adoption contract must agree before Bridge callbacks arrive.
        reporter.active_route_dispatch_contract = {
            "route_id":12,"graph_transaction_id":53,"transaction_id":0,
            "predecessor_contract":{"route_id":11},
        }
        with patch.object(route_rospy, "Time", create=True) as clock:
            clock.now.return_value.to_sec.return_value = 2.0
            reporter.publish_route_command("map",1.,2.,None,"frontier_endpoint")
        payload=json.loads(reporter.command_publisher.message.data)
        self.assertEqual(payload["graph_transaction_id"],53)
        self.assertEqual(payload["transaction_id"],53)
        self.assertEqual(payload["goal_context"]["graph_transaction_id"],53)
        for key in ("route_id","graph_transaction_id","transaction_id","map_epoch","projection_epoch"):
            self.assertEqual(reporter.active_route_dispatch_contract[key],payload[key])
        adopted=reporter._remember_bridge_dispatch_contract(dict(payload,event="dispatch",action_generation=77,source="global_slam_frontier",intent_priority=0))
        self.assertIsNotNone(adopted)
        self.assertEqual(reporter.active_route_dispatch_contract["action_generation"],77)
        self.assertEqual(reporter.active_route_dispatch_contract["predecessor_contract"],{"route_id":11})

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


    def test_gp_portal_assistance_ignores_legacy_probe_for_place_close(self):
        class _ProbeLedger:
            def unresolved_count(self, _place_id):
                return 1

        reporter = GlobalFrontierReportingMixin()
        reporter.current_physical_place_id = 1
        reporter.region_memory = _RegionMemory({
            1: {"id": 1, "state": "open", "endpoint_observations": 1},
        })
        reporter.place_work_items = None
        reporter.portal_probe_ledger = _ProbeLedger()
        reporter.gp_portal_assistance_enabled = True

        progress = reporter.place_progress_report()

        self.assertEqual(progress["unresolved_portals"], 0)
        self.assertTrue(progress["local_observation_complete"])

        reporter.gp_portal_assistance_enabled = False
        self.assertEqual(
            reporter.place_progress_report()["unresolved_portals"], 1
        )


if __name__ == "__main__":
    unittest.main()
