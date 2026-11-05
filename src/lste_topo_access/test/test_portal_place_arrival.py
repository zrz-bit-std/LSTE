"""Regression tests for durable place identity after a portal crossing."""

from pathlib import Path
import sys
import numpy as np
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

rospy = sys.modules.setdefault(
    "rospy",
    SimpleNamespace(loginfo=lambda *_args, **_kwargs: None, logwarn=lambda *_args, **_kwargs: None),
)
if not hasattr(rospy, "loginfo"):
    rospy.loginfo = lambda *_args, **_kwargs: None
if not hasattr(rospy, "logwarn"):
    rospy.logwarn = lambda *_args, **_kwargs: None

from global_frontier_place_memory import FrontierRegionMemory
from global_frontier_work_items import WORK_RESOLVED, PlaceWorkItemLedger
from global_frontier_models import PendingPortalArrival
from global_frontier_portal_transaction import PortalTransaction
from global_frontier_portal_belief import PortalHypothesisLedger
from global_frontier_portal_lifecycle import GlobalFrontierPortalLifecycleMixin
from global_frontier_portal_arrival_commit import (
    GlobalFrontierPortalArrivalCommitMixin,
)
from global_frontier_portal_authentication import (
    destination_returns_into_source_place,
)
from global_frontier_work_items import PlaceWorkItemLedger


class PortalExplorer(GlobalFrontierPortalLifecycleMixin):
    def __init__(self):
        self.region_memory = FrontierRegionMemory(
            radius=2.5,
            information_delta=8,
            stagnation_timeout=12,
            failure_limit=2,
            limit=16,
        )
        self.active_route_id = 19
        self.active_route_kind = "portal_transition"
        self.active_frontier = (10, 20, 8.0, 4.0)
        self.active_portal_retry = False
        self.active_portal_gate_xy = (7.2, 4.0)
        self.active_start_odom_xy = (5.0, 4.0)
        self.active_portal_gate_odom_xy = (7.2, 4.0)
        self.active_portal_destination_odom_xy = (8.5, 4.0)
        self.active_portal_gate_approached_at = 2.0
        self.pose_odom = SimpleNamespace(x=8.0, y=4.0)
        self.waypoint_release_radius = 0.4
        self.endpoint_terminal_wait_radius = 0.4
        self.clearance = 0.52
        self.active_frontier_component = {
            "epoch": 12,
            "label": 4,
            "cells": 80,
            "center_x": 8.0,
            "center_y": 4.0,
            "min_x": 7.0,
            "max_x": 9.0,
            "min_y": 3.0,
            "max_y": 5.0,
        }
        self.pending_portal_arrivals = []
        self.pending_portal_retry = None
        self.place_departure = SimpleNamespace(region_id=None)
        self.events = []
        self.current_place_scope_valid = True
        self._place_partition_key = ('old_raster',)
        self.scope_states = []

    def publish_status(self, event, **fields):
        self.events.append((event, fields))
        self.scope_states.append((event, self.current_place_scope_valid))


class PortalPlaceArrivalTest(unittest.TestCase):
    def test_handoff_retires_scope_but_preserves_exact_tf_and_unfinished_work(self):
        explorer=PortalExplorer()
        projector=lambda x,y:(x,y)
        explorer._place_partition_projector=projector
        explorer._place_partition_tf_key=('map',5.)
        explorer.place_work_items=object()
        ledger=explorer.place_work_items
        explorer.current_place_scope_frontier_work_evidence={'pending_count':0}
        explorer._invalidate_place_scope_for_handoff(7)
        self.assertFalse(explorer.current_place_scope_valid)
        self.assertIsNone(explorer._place_partition_key)
        self.assertEqual(explorer.place_entry_rehydration_pending,7)
        self.assertIsNone(explorer.current_place_scope_frontier_work_evidence)
        self.assertIs(explorer.place_work_items,ledger)
        self.assertIs(explorer._place_partition_projector,projector)
        self.assertEqual(explorer._place_partition_tf_key,('map',5.))

    def test_failed_projection_return_keeps_unresolved_work_and_does_not_close(self):
        from global_frontier_place_dfs import PlaceDfsTraversal
        from global_frontier_work_items import PlaceWorkItemLedger
        explorer=PortalExplorer();explorer.place_dfs=PlaceDfsTraversal()
        explorer.place_dfs.enter(1,3,12)
        explorer.place_dfs.update_projection_recovery(3,False,0.,(4.,0.))
        explorer.place_dfs.update_projection_recovery(3,False,30.,(4.,0.))
        explorer.place_work_items=PlaceWorkItemLedger(merge_radius=.5)
        item=explorer.place_work_items._new_item(3,(),0.)
        room=dict(node_id=7,branch_id=4,portal_id=12,place_id=3,source_place_id=1)
        explorer.active_semantic_rooms=[room];explorer.active_semantic_room=room
        explorer._commit_semantic_room_return(room,12,1,5.,None,None,explorer.active_semantic_rooms)
        self.assertEqual(explorer.place_work_items.unresolved_ids(3),[item['id']])
        self.assertEqual(explorer.place_dfs.nodes[3]['state'],'failed')
        self.assertFalse(explorer.current_place_scope_valid)
        self.assertIsNone(explorer._place_partition_key)
        self.assertEqual(explorer.place_entry_rehydration_pending,1)
        self.assertEqual(explorer.place_dfs.stack,[1])
        self.assertEqual(explorer.events[-1][0],'place_scope_recovery_returned')
        self.assertFalse(explorer.events[-1][1]['place_closed'])
        self.assertNotIn('semantic_room_closed',[name for name,_ in explorer.events])

    def test_return_waits_for_unprojected_place_work(self):
        explorer = PortalExplorer()
        from global_frontier_work_items import PlaceWorkItemLedger
        explorer.place_work_items = PlaceWorkItemLedger(merge_radius=0.5)
        item = explorer.place_work_items._new_item(3, (), 0.0)
        room = {"node_id": 7, "branch_id": 4, "portal_id": 12,
                "place_id": 3, "source_place_id": 1}
        explorer.active_semantic_rooms = [room]
        explorer.active_semantic_room = room
        explorer.pending_graph_resume_place_id = None
        explorer.pending_graph_resume_portal_id = None
        explorer.pending_graph_resume_destination_place_id = None

        explorer._commit_semantic_room_return(room, 99, 1, 5.0, None, None,
                                              explorer.active_semantic_rooms)

        self.assertEqual(explorer.pending_graph_resume_place_id, 3)
        self.assertEqual(explorer.place_work_items.unresolved_ids(3), [item["id"]])
        self.assertIs(explorer.active_semantic_room, room)
        self.assertEqual(explorer.events[-1][0], "semantic_room_return_incomplete")

        explorer.place_work_items.resolve(item["id"], 6.0, reason="observed")
        explorer._commit_semantic_room_return(room, 99, 1, 7.0, None, None,
                                              explorer.active_semantic_rooms)
        self.assertIsNone(explorer.pending_graph_resume_place_id)
        self.assertEqual(explorer.place_work_items.unresolved_ids(3), [])
        self.assertIsNone(explorer.active_semantic_room)
        self.assertEqual(explorer.active_semantic_rooms, [])
        self.assertEqual(explorer.events[-1][0], "semantic_room_closed")

    def test_return_waits_while_scoped_work_attempt_is_active(self):
        from global_frontier_work_items import PlaceWorkItemLedger
        explorer = PortalExplorer()
        explorer.place_work_items = PlaceWorkItemLedger(merge_radius=0.5)
        item = explorer.place_work_items._new_item(3, (), 0.0, anchor_xy=[1.0, 0.0])
        explorer.place_work_items.dispatch_existing(
            item["id"], 1.0, viewpoint_xy=[0.5, 0.0],
        )
        room = {"node_id": 7, "branch_id": 4, "portal_id": 12,
                "place_id": 3, "source_place_id": 1}
        explorer.active_semantic_rooms = [room]
        explorer.active_semantic_room = room
        explorer.pending_graph_resume_place_id = None
        explorer.pending_graph_resume_portal_id = None
        explorer.pending_graph_resume_destination_place_id = None

        explorer._commit_semantic_room_return(
            room, 99, 1, 5.0, None, None, explorer.active_semantic_rooms,
        )

        self.assertEqual(explorer.pending_graph_resume_place_id, 3)
        self.assertEqual(explorer.place_work_items.unresolved_ids(3), [item["id"]])
        self.assertIs(explorer.active_semantic_room, room)
        self.assertEqual(explorer.events[-1][0], "semantic_room_return_incomplete")

    def test_return_cannot_close_place_while_any_direction_has_unfinished_work(self):
        from global_frontier_work_items import PlaceWorkItemLedger
        from global_frontier_room_scope import build_branch_scope
        explorer = PortalExplorer()
        explorer.place_work_items = PlaceWorkItemLedger(merge_radius=0.5)
        room = {"node_id": 7, "branch_id": 4, "portal_id": 12,
                "place_id": 3, "source_place_id": 1}
        branches = [
            {"id": 4, "map_geometry": {"direction": [1.0, 0.0]}},
            {"id": 5, "map_geometry": {"direction": [0.0, 1.0]}},
        ]
        room["branch_scope"] = build_branch_scope([0.0, 0.0], branches, 4)
        selected = explorer.place_work_items._new_item(
            3, (), 0.0, anchor_xy=[4.0, 0.2],
        )
        sibling = explorer.place_work_items._new_item(
            3, (), 0.0, anchor_xy=[0.2, 4.0],
        )
        explorer.place_work_items.resolve(selected["id"], 1.0, reason="observed")
        explorer.active_semantic_rooms = [room]
        explorer.active_semantic_room = room
        explorer.pending_graph_resume_place_id = None
        explorer.pending_graph_resume_portal_id = None
        explorer.pending_graph_resume_destination_place_id = None

        explorer._commit_semantic_room_return(
            room, 12, 1, 5.0, None, None, explorer.active_semantic_rooms,
        )

        self.assertEqual(explorer.events[-1][0], "semantic_room_return_incomplete")
        self.assertEqual(explorer.place_work_items.unresolved_ids(1), [])
        self.assertEqual(explorer.place_work_items.unresolved_ids(3), [sibling["id"]])
        self.assertEqual(explorer.active_semantic_rooms, [room])
        self.assertEqual(explorer.pending_graph_resume_place_id, 3)

    def test_returning_to_incomplete_place_resumes_without_pushing_duplicate_room(self):
        explorer = PortalExplorer()
        room = {"node_id": 7, "branch_id": 4, "portal_id": 12,
                "place_id": 3, "source_place_id": 1}
        explorer.active_semantic_rooms = [room]
        explorer.active_semantic_room = room

        resumed = explorer._resume_semantic_room_arrival(room, 3, 99)

        self.assertTrue(resumed)
        self.assertEqual(explorer.active_semantic_rooms, [room])
        self.assertIs(explorer.active_semantic_room, room)
        self.assertEqual(explorer.events[-1][0], "semantic_room_exploration_resumed")

    def test_completed_nested_room_pops_to_outer_room_and_suppresses_its_entrance(self):
        explorer = PortalExplorer()
        from global_frontier_work_items import PlaceWorkItemLedger
        from global_frontier_portal_belief import PortalHypothesisLedger
        explorer.place_work_items = PlaceWorkItemLedger(merge_radius=0.5)
        outer = {"node_id": 7, "branch_id": 4, "portal_id": 12,
                 "place_id": 3, "source_place_id": 1}
        portals = PortalHypothesisLedger()
        portal, _ = portals.certify(3, (1, 0), (2, 0), now=1.0)
        inner = {"node_id": 9, "branch_id": 5, "portal_id": portal["id"],
                 "place_id": 4, "source_place_id": 3}
        explorer.active_semantic_rooms = [outer, inner]
        explorer.active_semantic_room = inner
        explorer._commit_semantic_room_return(inner, 99, 3, 5.0, portals,
                                              None, explorer.active_semantic_rooms)
        self.assertIs(explorer.active_semantic_room, outer)
        self.assertEqual(explorer.active_semantic_rooms, [outer])
        self.assertTrue(portals.get(portal["id"])["exploration_suppressed"])
        self.assertEqual(explorer.events[-1][0], "semantic_room_closed")

    def test_terminal_binds_next_selection_to_the_arrived_place(self):
        explorer = PortalExplorer()

        region = explorer.record_portal_place_arrival(
            (10, 20, 8.0, 4.0), now=3.0
        )

        self.assertIsNotNone(region)
        self.assertEqual(region["entered_at"], 3.0)
        self.assertEqual(region["observation_started_at"], 3.0)
        self.assertEqual(region["endpoint_observations"], 1)
        self.assertEqual(region["portal_arrivals"], 1)
        self.assertEqual(
            region["entry_portals"],
            [{
                "gate": [7.2, 4.0],
                "inside": [8.0, 4.0],
                "physical_gate": [7.2, 4.0],
                "physical_inside": [8.0, 4.0],
                "normal": [1.0, 0.0],
            }],
        )
        self.assertEqual(explorer.events[-1][0], "portal_place_entered")
        self.assertTrue(any(
            event == "portal_place_entry_observed"
            for event, _fields in explorer.events
        ))
        tier, matched = explorer.region_memory.candidate_tier(
            8.1, 4.0, 20.0, component=explorer.active_frontier_component
        )
        self.assertEqual(tier, "revisit")
        self.assertEqual(matched["id"], region["id"])

    def test_missing_terminal_component_is_committed_from_a_fresh_snapshot(self):
        explorer = PortalExplorer()
        explorer.active_frontier_component = None
        component = {
            "epoch": 13,
            "label": 7,
            "cells": 80,
            "center_x": 8.0,
            "center_y": 4.0,
            "min_x": 7.0,
            "max_x": 9.0,
            "min_y": 3.0,
            "max_y": 5.0,
        }
        explorer.component_at_map_position = lambda *_args: component

        self.assertIsNone(
            explorer.record_portal_place_arrival((10, 20, 8.0, 4.0), now=3.0)
        )
        self.assertEqual(explorer.events[-1][0], "portal_place_arrival_pending")
        self.assertEqual(len(explorer.pending_portal_arrivals), 1)

        entered = explorer.retry_pending_portal_arrivals(
            None, None, None, now=4.0
        )

        self.assertEqual(len(entered), 1)
        self.assertEqual(len(explorer.pending_portal_arrivals), 0)
        self.assertEqual(entered[0]["portal_arrivals"], 1)
        self.assertEqual(explorer.events[-1][0], "portal_place_entered")
        self.assertEqual(
            explorer.events[-1][1]["evidence_source"], "fresh_map_snapshot"
        )

    def test_source_component_does_not_reject_a_pending_destination_arrival(self):
        explorer = PortalExplorer()
        source, _tier = explorer.region_memory.enter(
            5.0,
            4.0,
            now=1.0,
            component=explorer.active_frontier_component,
        )
        explorer.region_memory.endpoint_observed(
            5.0,
            4.0,
            now=2.0,
            component=explorer.active_frontier_component,
            region_id=source["id"],
        )
        explorer.place_departure.region_id = source["id"]
        explorer.active_frontier_component = None
        explorer.component_at_map_position = lambda *_args: {
            **source["component"],
        }

        explorer.record_portal_place_arrival((10, 20, 8.0, 4.0), now=3.0)
        entered = explorer.retry_pending_portal_arrivals(
            None, None, None, now=4.0
        )

        self.assertEqual(entered, [])
        self.assertEqual(len(explorer.pending_portal_arrivals), 1)
        self.assertFalse(any(event == "portal_place_arrival_rejected"
                             for event, _fields in explorer.events))

    def test_verified_crossing_creates_destination_when_core_stays_connected(self):
        explorer = PortalExplorer()
        source, _tier = explorer.region_memory.enter(
            5.0,
            4.0,
            now=1.0,
            component=explorer.active_frontier_component,
        )
        explorer.region_memory.endpoint_observed(
            5.0,
            4.0,
            now=2.0,
            component=explorer.active_frontier_component,
            region_id=source["id"],
        )
        explorer.place_departure.region_id = source["id"]
        explorer.portal_hypothesis_ledger = PortalHypothesisLedger()
        edge, _created = explorer.portal_hypothesis_ledger.certify(
            source["id"],
            explorer.active_portal_gate_odom_xy,
            explorer.active_portal_destination_odom_xy,
            now=2.0,
        )
        explorer.last_portal_hypothesis_id = edge["id"]
        transaction = PortalTransaction()
        transaction.start(
            explorer.active_route_id,
            portal_id=edge["id"],
            source_place_id=source["id"],
            gate_xy=explorer.active_portal_gate_odom_xy,
            destination_xy=explorer.active_portal_destination_odom_xy,
            source_side_proven=True,
        )
        transaction.crossing_verified(explorer.active_route_id)
        explorer.portal_transaction = transaction

        entered = explorer.record_portal_place_arrival(
            (10, 20, 8.0, 4.0), now=3.0,
        )

        self.assertIsNotNone(entered)
        self.assertFalse(explorer.current_place_scope_valid)
        self.assertIsNone(explorer._place_partition_key)
        self.assertTrue(all(not valid for event,valid in explorer.scope_states
                            if event in ('portal_hypothesis_crossed','place_dfs_entered')))
        self.assertNotEqual(entered["id"], source["id"])
        self.assertEqual(entered["portal_arrivals"], 1)
        self.assertEqual(entered["entry_portals"][0]["portal_id"], edge["id"])
        self.assertEqual(entered["entry_portals"][0]["source_place_id"], source["id"])
        self.assertEqual(explorer.current_physical_place_id, entered["id"])
        self.assertEqual(edge["destination_place_id"], entered["id"])
        self.assertEqual(
            explorer.events[-1][0], "portal_place_entered",
        )

    def test_verified_crossing_materializes_destination_without_component(self):
        """A physical edge must not wait for an unavailable SLAM label."""
        explorer = PortalExplorer()
        source, _tier = explorer.region_memory.enter(
            5.0,
            4.0,
            now=1.0,
            component=explorer.active_frontier_component,
        )
        explorer.region_memory.endpoint_observed(
            5.0,
            4.0,
            now=2.0,
            component=explorer.active_frontier_component,
            region_id=source["id"],
        )
        explorer.place_departure.region_id = source["id"]
        explorer.active_frontier_component = None
        explorer.portal_hypothesis_ledger = PortalHypothesisLedger()
        edge, _created = explorer.portal_hypothesis_ledger.certify(
            source["id"],
            explorer.active_portal_gate_odom_xy,
            explorer.active_portal_destination_odom_xy,
            now=2.0,
        )
        explorer.last_portal_hypothesis_id = edge["id"]
        transaction = PortalTransaction()
        transaction.start(
            explorer.active_route_id,
            portal_id=edge["id"],
            source_place_id=source["id"],
            gate_xy=explorer.active_portal_gate_odom_xy,
            destination_xy=explorer.active_portal_destination_odom_xy,
            source_side_proven=True,
        )
        transaction.crossing_verified(explorer.active_route_id)
        explorer.portal_transaction = transaction

        entered = explorer.record_portal_place_arrival(
            (10, 20, 8.0, 4.0), now=3.0,
        )

        self.assertIsNotNone(entered)
        self.assertFalse(explorer.current_place_scope_valid)
        self.assertIsNone(explorer._place_partition_key)
        self.assertTrue(all(not valid for event,valid in explorer.scope_states
                            if event in ('portal_hypothesis_crossed','place_dfs_entered')))
        self.assertNotEqual(entered["id"], source["id"])
        self.assertEqual(explorer.current_physical_place_id, entered["id"])
        self.assertEqual(edge["destination_place_id"], entered["id"])
        self.assertEqual(
            explorer.events[-1][1]["evidence_source"],
            "portal_identity_without_component",
        )

    def test_pending_room_reentry_reuses_the_existing_place_identity(self):
        explorer = PortalExplorer()
        source_component = {
            "epoch": 21, "label": 1, "cells": 80,
            "center_x": 5.0, "center_y": 4.0,
            "min_x": 4.0, "max_x": 6.0,
            "min_y": 3.0, "max_y": 5.0,
        }
        room_component = {
            **source_component, "label": 2, "center_x": 8.0,
            "min_x": 7.0, "max_x": 9.0,
        }
        source, _ = explorer.region_memory.activate(
            5.0, 4.0, 10.0, now=1.0, component=source_component,
        )
        room, _ = explorer.region_memory.activate(
            8.0, 4.0, 10.0, now=2.0, component=room_component,
        )
        explorer.portal_hypothesis_ledger = PortalHypothesisLedger()
        edge, _ = explorer.portal_hypothesis_ledger.certify(
            source["id"], (7.2, 4.0), (8.5, 4.0), now=2.0,
        )
        explorer.portal_hypothesis_ledger.crossed(edge["id"], now=2.5)
        explorer.portal_hypothesis_ledger.bind_destination(
            edge["id"], room["id"], now=2.5,
        )
        explorer.last_portal_hypothesis_id = edge["id"]
        explorer.portal_transaction = PortalTransaction()
        explorer.portal_transaction.start(
            explorer.active_route_id,
            portal_id=edge["id"],
            source_place_id=source["id"],
            gate_xy=(7.2, 4.0),
            destination_xy=(8.5, 4.0),
            source_side_proven=True,
        )
        explorer.portal_transaction.crossing_verified(explorer.active_route_id)
        active_room = {
            "node_id": 4, "branch_id": 8, "portal_id": edge["id"],
            "place_id": room["id"], "source_place_id": source["id"],
        }
        explorer.active_semantic_room = active_room
        explorer.active_semantic_rooms = [active_room]
        explorer.pending_graph_resume_place_id = room["id"]
        explorer.pending_graph_resume_portal_id = edge["id"]
        explorer.pending_graph_resume_destination_place_id = source["id"]
        explorer.region_memory.endpoint_observed(
            8.0, 4.0, now=2.8, component=room_component,
            region_id=room["id"],
        )
        arrival = PendingPortalArrival(
            route_id=explorer.active_route_id,
            goal_xy=(8.0, 4.0),
            terminal_time=3.0,
            source_region_id=source["id"],
            portal_gate_xy=(7.2, 4.0),
            physical_goal_xy=(8.0, 4.0),
            physical_portal_gate_xy=(7.2, 4.0),
        )

        entered = explorer.commit_portal_arrival(
            arrival, room_component, 3.0, "route_terminal",
        )

        self.assertIsNotNone(entered)
        self.assertFalse(explorer.current_place_scope_valid)
        self.assertIsNone(explorer._place_partition_key)
        self.assertTrue(all(not valid for event,valid in explorer.scope_states
                            if event in ('portal_hypothesis_crossed','place_dfs_entered')))
        self.assertEqual(entered["id"], room["id"])
        self.assertEqual(len(explorer.region_memory.regions), 2)
        self.assertEqual(explorer.current_physical_place_id, room["id"])
        self.assertTrue(any(
            event == "semantic_room_exploration_resumed"
            for event, _fields in explorer.events
        ))

    def test_verified_arrival_in_covered_place_is_recorded_without_reopening_it(self):
        explorer = PortalExplorer()
        destination = explorer.active_frontier_component
        region, _tier = explorer.region_memory.enter(
            8.0, 4.0, now=1.0, component=destination,
        )
        explorer.region_memory.observe(
            8.0,
            4.0,
            10.0,
            robot_x=8.0,
            robot_y=4.0,
            now=2.0,
            component=destination,
            region_id=region["id"],
            observation_ready=True,
        )
        explorer.region_memory.dormant(
            region, now=3.0, reason="physical_departure",
        )

        arrived = explorer.record_portal_place_arrival(
            (10, 20, 8.0, 4.0), now=4.0,
        )

        self.assertIs(arrived, region)
        self.assertEqual(region["state"], "dormant")
        self.assertEqual(region["covered_arrivals"], 1)
        self.assertEqual(region["last_reason"], "portal_arrival_already_covered")
        self.assertEqual(explorer.events[-1][0], "portal_place_covered_arrival")

    def test_reverse_portal_arrival_reuses_the_edge_source_place(self):
        explorer = PortalExplorer()
        source_component = {
            "epoch": 20,
            "label": 1,
            "cells": 80,
            "center_x": 5.0,
            "center_y": 4.0,
            "min_x": 4.0,
            "max_x": 6.0,
            "min_y": 3.0,
            "max_y": 5.0,
        }
        destination_component = {
            **source_component,
            "label": 2,
            "center_x": 8.0,
        }
        source, _ = explorer.region_memory.activate(
            5.0, 4.0, 10.0, now=1.0, component=source_component,
        )
        destination, _ = explorer.region_memory.activate(
            8.0, 4.0, 10.0, now=2.0, component=destination_component,
        )
        explorer.region_memory.endpoint_observed(
            5.0, 4.0, now=1.5, component=source_component,
            region_id=source["id"],
        )
        explorer.region_memory.endpoint_observed(
            8.0, 4.0, now=2.5, component=destination_component,
            region_id=destination["id"],
        )
        explorer.place_departure.region_id = destination["id"]
        explorer.active_frontier_component = source_component
        explorer.last_portal_hypothesis_id = 1
        explorer.portal_hypothesis_ledger = PortalHypothesisLedger()
        edge, _ = explorer.portal_hypothesis_ledger.certify(
            source["id"], (7.2, 4.0), (8.5, 4.0), now=2.0,
        )
        explorer.portal_hypothesis_ledger.crossed(edge["id"], now=2.5)
        explorer.portal_hypothesis_ledger.bind_destination(
            edge["id"], destination["id"], now=2.5,
        )

        arrived = explorer.record_portal_place_arrival(
            (10, 20, 8.0, 4.0), now=3.0,
        )

        self.assertIs(arrived, source)
        self.assertEqual(arrived["id"], source["id"])
        self.assertEqual(len(explorer.region_memory.regions), 2)
        # Reverse transit reuses the already-bound directed edge; it must not
        # overwrite the original destination with the Place being returned to.
        self.assertEqual(edge["destination_place_id"], destination["id"])
        self.assertFalse(any(
            event == "portal_hypothesis_destination_bound"
            for event, _fields in explorer.events
        ))

    def test_goal_circle_success_at_the_gate_never_enters_the_destination(self):
        explorer = PortalExplorer()
        # The action endpoint is beyond the door, but TEB may report success
        # anywhere in its XY tolerance circle. This pose is still at the gate.
        explorer.pose_odom = SimpleNamespace(x=7.3, y=4.0)

        entered = explorer.record_portal_place_arrival(
            (10, 20, 8.0, 4.0), now=3.0,
        )

        self.assertIsNone(entered)
        self.assertEqual(explorer.pending_portal_arrivals, [])
        self.assertIsNotNone(explorer.pending_portal_retry)
        self.assertEqual(explorer.events[-1][0], "portal_crossing_unconfirmed")
        self.assertEqual(
            explorer.events[-1][1]["reason"],
            "physical_destination_side_not_reached",
        )

    def test_stale_arrival_is_rejected_before_place_memory_mutation(self):
        explorer = PortalExplorer()
        transaction = PortalTransaction()
        transaction.start(
            20,
            portal_id=9,
            source_place_id=3,
            gate_xy=(7.2, 4.0),
            destination_xy=(8.5, 4.0),
        )
        transaction.crossing_verified(20)
        explorer.portal_transaction = transaction
        explorer.last_portal_hypothesis_id = 9

        arrival = PendingPortalArrival(
            route_id=19,
            goal_xy=(8.0, 4.0),
            terminal_time=4.0,
            source_region_id=3,
            portal_gate_xy=(7.2, 4.0),
            physical_goal_xy=(8.0, 4.0),
            physical_portal_gate_xy=(7.2, 4.0),
        )
        before = len(explorer.region_memory.regions)

        self.assertIsNone(
            explorer.commit_portal_arrival(
                arrival,
                explorer.active_frontier_component,
                now=4.0,
                evidence_source="stale_terminal",
            )
        )
        self.assertEqual(len(explorer.region_memory.regions), before)
        self.assertTrue(explorer.current_place_scope_valid)
        self.assertEqual(explorer._place_partition_key, ('old_raster',))
        self.assertEqual(
            explorer.events[-1][1]["reason"],
            "stale_or_mismatched_portal_transaction",
        )

    def test_live_odom_crossing_commits_before_the_controller_terminal(self):
        """A completed graph edge cannot be retried after a late TEB stall."""
        explorer = PortalExplorer()

        region = explorer.observe_active_portal_crossing(now=3.0)

        self.assertIsNotNone(region)
        self.assertTrue(explorer.active_portal_crossing_observed)
        self.assertEqual(region["portal_arrivals"], 1)
        self.assertEqual(explorer.events[-1][0], "portal_place_entered")
        self.assertEqual(
            explorer.events[-1][1]["evidence_source"], "continuous_odom"
        )
        self.assertIsNone(explorer.observe_active_portal_crossing(now=4.0))
        self.assertEqual(region["portal_arrivals"], 1)

    def test_live_odom_check_does_not_prepare_a_retry_before_crossing(self):
        """Approaching a gate is not a failed edge and should remain quiet."""
        explorer = PortalExplorer()
        explorer.pose_odom = SimpleNamespace(x=7.3, y=4.0)

        self.assertIsNone(explorer.observe_active_portal_crossing(now=3.0))
        self.assertIsNone(explorer.pending_portal_retry)
        self.assertEqual(explorer.events, [])

    def test_self_loop_rejection_closes_transaction_and_departure(self):
        """A same-Place portal result must not strand successor planning."""
        explorer = PortalExplorer()
        source, _tier = explorer.region_memory.enter(
            5.0,
            4.0,
            now=1.0,
            component=explorer.active_frontier_component,
        )
        explorer.region_memory.endpoint_observed(
            5.0,
            4.0,
            now=2.0,
            component=explorer.active_frontier_component,
            region_id=source["id"],
        )
        class Departure:
            active = True
            region_id = source["id"]

            def clear(self):
                self.active = False

        explorer.place_departure = Departure()
        explorer._clear_active_place_departure = explorer.place_departure.clear
        explorer.portal_hypothesis_ledger = PortalHypothesisLedger()
        edge, _created = explorer.portal_hypothesis_ledger.certify(
            source["id"],
            explorer.active_portal_gate_odom_xy,
            explorer.active_portal_destination_odom_xy,
            now=2.0,
        )
        explorer.last_portal_hypothesis_id = edge["id"]
        transaction = PortalTransaction()
        transaction.start(
            explorer.active_route_id,
            portal_id=edge["id"],
            source_place_id=source["id"],
            gate_xy=explorer.active_portal_gate_odom_xy,
            destination_xy=explorer.active_portal_destination_odom_xy,
            source_side_proven=True,
        )
        transaction.crossing_verified(explorer.active_route_id)
        explorer.portal_transaction = transaction

        # Force the destination resolver to return the source Place, matching
        # the false self-loop seen in the Level 4 run when no new component was
        # available at the portal endpoint.
        explorer.region_memory.enter_new_portal_destination = (
            lambda *_args, **_kwargs: (source, "covered_arrival")
        )
        arrival = PendingPortalArrival(
            route_id=explorer.active_route_id,
            goal_xy=(8.0, 4.0),
            terminal_time=3.0,
            source_region_id=source["id"],
            portal_gate_xy=explorer.active_portal_gate_xy,
            physical_goal_xy=(8.0, 4.0),
            physical_portal_gate_xy=explorer.active_portal_gate_odom_xy,
        )

        self.assertIsNone(
            explorer.commit_portal_arrival(
                arrival,
                None,
                now=3.0,
                evidence_source="portal_identity_without_component",
            )
        )
        self.assertFalse(explorer.portal_transaction.active)
        self.assertTrue(explorer.active_portal_crossing_rejected)
        self.assertFalse(explorer.place_departure.active)
        self.assertEqual(
            explorer.portal_hypothesis_ledger.get(edge["id"])["state"],
            "failed",
        )
        self.assertTrue(any(
            event == "portal_transaction_aborted" for event, _fields in explorer.events
        ))
        # The rejected route cannot reopen the same arrival on a later pose or
        # terminal callback.
        event_count = len(explorer.events)
        self.assertIsNone(
            explorer.record_portal_place_arrival(
                (10, 20, 8.0, 4.0), now=4.0,
            )
        )
        self.assertEqual(len(explorer.events), event_count)


if __name__ == "__main__":
    unittest.main()


class SemanticEntryPlaceCommitTest(unittest.TestCase):
    def test_crossing_into_the_destination_resolves_the_semantic_entry(self):
        explorer = PortalExplorer()
        source, _tier = explorer.region_memory.enter(
            5.0, 4.0, now=1.0, component=explorer.active_frontier_component,
        )
        explorer.region_memory.endpoint_observed(
            5.0, 4.0, now=2.0,
            component=explorer.active_frontier_component,
            region_id=source["id"],
        )
        explorer.place_departure.region_id = source["id"]
        explorer.portal_hypothesis_ledger = PortalHypothesisLedger()
        edge, _created = explorer.portal_hypothesis_ledger.certify(
            source["id"],
            explorer.active_portal_gate_odom_xy,
            explorer.active_portal_destination_odom_xy,
            now=2.0,
        )
        explorer.last_portal_hypothesis_id = edge["id"]
        transaction = PortalTransaction()
        transaction.start(
            explorer.active_route_id,
            portal_id=edge["id"],
            source_place_id=source["id"],
            gate_xy=explorer.active_portal_gate_odom_xy,
            destination_xy=explorer.active_portal_destination_odom_xy,
            source_side_proven=True,
        )
        transaction.crossing_verified(explorer.active_route_id)
        explorer.portal_transaction = transaction
        explorer.place_work_items = PlaceWorkItemLedger(0.5, 0.5)
        entry = explorer.place_work_items.ensure_portal_entry(
            edge["id"], source["id"], (7.2, 4.0), 1, 0.95, 2.0,
        )
        explorer.place_work_items.dispatch_existing(
            entry, 2.5, viewpoint_xy=(7.2, 4.0),
        )
        self.assertEqual(
            explorer.place_work_items._items[entry]["state"], "unresolved",
        )

        entered = explorer.record_portal_place_arrival(
            (10, 20, 8.0, 4.0), now=3.0,
        )

        self.assertIsNotNone(entered)
        item = explorer.place_work_items._items[entry]
        self.assertEqual(item["state"], WORK_RESOLVED)
        self.assertIsNone(explorer.place_work_items.active_attempt_id(entry))
        self.assertEqual(
            item["last_reason"], "portal_destination_place_committed",
        )
        resolved = [
            event for event, fields in explorer.events
            if event == "portal_entry_work_item_resolved"
        ]
        self.assertTrue(resolved)


class DestinationSelfLoopTest(unittest.TestCase):
    def _memory(self):
        memory = FrontierRegionMemory(
            radius=3.0,
            information_delta=8,
            stagnation_timeout=12,
            failure_limit=2,
            limit=16,
        )
        source, _tier = memory.enter(3.4, 0.0, now=1.0)
        return memory, source

    def test_destination_back_inside_the_source_place_is_a_self_loop(self):
        memory, source = self._memory()
        node = SimpleNamespace()  # no live Place partition: identity fallback
        self.assertTrue(
            destination_returns_into_source_place(
                node, (4.082, -0.646), source["id"], region_memory=memory,
            )
        )
        self.assertFalse(
            destination_returns_into_source_place(
                node, (9.523, -0.097), source["id"], region_memory=memory,
            )
        )

    def test_live_mask_alone_does_not_suppress_a_new_room(self):
        node = SimpleNamespace(
            current_place_scope_mask=np.ones((20, 20), dtype=bool),
            current_place_scope_message=SimpleNamespace(
                info=SimpleNamespace(
                    resolution=1.0,
                    origin=SimpleNamespace(
                        position=SimpleNamespace(x=0.0, y=0.0),
                    ),
                ),
            ),
            current_place_scope_valid=True,
            current_place_scope_id=1,
            current_physical_place_id=1,
        )
        # Connectivity through an open doorway is not exploration evidence.
        self.assertFalse(
            destination_returns_into_source_place(node, (4.0, 0.5), 1)
        )

    def test_visited_path_separates_return_from_new_room(self):
        memory, source = self._memory()
        for now, xy in (
            (1.0, (3.4, 0.0)),
            (2.0, (4.0, 0.1)),
            (3.0, (5.0, 0.2)),
        ):
            memory.record_visited_pose(
                source["id"], odom_xy=xy, now=now,
            )
        node = SimpleNamespace(
            current_place_scope_mask=np.ones((20, 20), dtype=bool),
            current_place_scope_message=SimpleNamespace(
                info=SimpleNamespace(
                    resolution=1.0,
                    origin=SimpleNamespace(
                        position=SimpleNamespace(x=0.0, y=0.0),
                    ),
                ),
            ),
            current_place_scope_valid=True,
            current_place_scope_id=1,
            current_physical_place_id=1,
        )
        # The west return is on the prior path even though the live mask also
        # contains it through the open doorway.
        self.assertTrue(
            destination_returns_into_source_place(
                node, (3.85, -0.635), source["id"],
                region_memory=memory, before_time=10.0,
            )
        )
        # The upper room is connected by an open door but was not visited
        # before this branch started.
        self.assertFalse(
            destination_returns_into_source_place(
                node, (6.472, 3.374), source["id"],
                region_memory=memory, before_time=10.0,
            )
        )
        # The current branch's own entry path must not count as history.
        self.assertFalse(
            destination_returns_into_source_place(
                node, (3.85, -0.635), source["id"],
                region_memory=memory, before_time=0.5,
            )
        )


class PromotionHost(GlobalFrontierPortalArrivalCommitMixin):
    def __init__(self):
        self.region_memory = FrontierRegionMemory(
            radius=3.0,
            information_delta=8,
            stagnation_timeout=12,
            failure_limit=2,
            limit=16,
        )
        self.portal_hypothesis_ledger = PortalHypothesisLedger()
        self.place_work_items = PlaceWorkItemLedger(0.5, 0.5)
        self.events = []
        self.place_dfs = SimpleNamespace(
            nodes={
                1: {"place_id": 1, "state": "exploring"},
                2: {"place_id": 2, "state": "exploring"},
                3: {"place_id": 3, "state": "exploring"},
            },
            stack=[1],
        )

    def publish_status(self, event, **fields):
        self.events.append((event, fields))


class NextSemanticPortalPromotionTest(unittest.TestCase):
    def _host(self):
        host = PromotionHost()
        source, _tier = host.region_memory.enter(3.4, 0.0, now=1.0)
        east, _tier = host.region_memory.enter(9.5, -0.1, now=1.0)
        upper, _tier = host.region_memory.enter(6.75, 4.0, now=1.0)
        loop, _created = host.portal_hypothesis_ledger.certify(
            source["id"], (5.44, -0.22), (4.08, -0.65), now=2.0,
        )
        loop["destination_place_id"] = upper["id"]
        loop["semantic_priority"] = 2
        loop["semantic_score"] = -0.69
        loop["exploration_suppressed"] = True
        east_edge, _created = host.portal_hypothesis_ledger.certify(
            source["id"], (8.12, -0.11), (9.52, -0.10), now=2.0,
        )
        east_edge["destination_place_id"] = east["id"]
        east_edge["semantic_priority"] = 3
        east_edge["semantic_score"] = -0.69
        return host, source, east, upper, loop, east_edge

    def test_next_deferred_portal_becomes_an_entry_work_item(self):
        host, source, east, upper, loop, east_edge = self._host()

        entry = host._promote_next_semantic_portal(source["id"], now=5.0)

        self.assertIsNotNone(entry)
        item = host.place_work_items._items[entry]
        self.assertEqual(item["kind"], "portal_entry")
        self.assertEqual(item["portal_id"], east_edge["id"])
        self.assertEqual(item["place_id"], source["id"])
        self.assertEqual((item["x"], item["y"]), (8.12, -0.11))
        self.assertIsNone(
            host._promote_next_semantic_portal(source["id"], now=6.0)
        )
        self.assertEqual(
            host.events[-1][0], "portal_entry_work_item_promoted",
        )

    def test_completed_destination_is_not_promoted(self):
        host, source, east, upper, loop, east_edge = self._host()
        host.place_dfs.nodes[east["id"]]["state"] = "completed"
        self.assertIsNone(
            host._promote_next_semantic_portal(source["id"], now=5.0)
        )
