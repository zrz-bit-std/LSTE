import math
#!/usr/bin/env python3
import unittest
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import patch
import sys

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from gp_branch_frontier import (
    GPBranchFrontierMixin, select_reachable_interior_goal,
    select_reachable_approach_goal, reproject_locked_entry_goal,
    advance_goal_along_portal_axis,
    select_reachable_source_side_return_goal,
    select_reachable_return_sidestep_goal,
)
from gp_observation_tree import JunctionObservationTree
from global_frontier_planning_selection import GlobalFrontierPlanningSelectionMixin
from global_frontier_work_items import (
    GlobalFrontierWorkItemLifecycleMixin, PlaceWorkItemLedger,
)


class PortalAuthenticationNodePinTest(unittest.TestCase):
    def test_measured_admission_rejects_sector_only_or_invalid_sections(self):
        from global_frontier_place_partition import measured_authentication_gate_geometry
        for geometry in ({}, {'width':2.}, {'width':2.,'physical_bounds':[[0.,0.],[0.,0.]]},
                         {'width':2.,'physical_bounds':[[0.,0.],[float('nan'),1.]]}):
            with self.subTest(geometry=geometry):
                self.assertIsNone(measured_authentication_gate_geometry({'reach':3.,'angular_width':1.,'map_geometry':geometry}))

    def test_watching_task_pins_junction_during_quiet_settle_window(self):
        fixture = GPBranchFrontierMixin()
        fixture.active_portal_authentication = lambda: None
        fixture.junction_portal_authentication = SimpleNamespace(
            node_id=12, phase="watching", all_finished=lambda: False,
        )
        self.assertEqual(fixture.portal_authentication_node_to_pin(), 12)

    def test_authentication_motion_does_not_add_room_lobes_as_sibling_arms(self):
        fixture = GPBranchFrontierMixin()
        fixture.junction_portal_authentication = SimpleNamespace(
            node_id=12, active_branch_id=31, phase="authenticating",
        )
        tree = SimpleNamespace(current_node_id=12)
        proposals = [{"heading": 1.0}, {"heading": -1.0}]
        self.assertEqual(
            fixture.tree_observations_for_authentication(tree, proposals), (),
        )

    def test_pending_observation_keeps_source_when_viewpoint_matches_another_node(self):
        from gp_exploration_tree import ExplorationTree
        fixture = GPBranchFrontierMixin()
        fixture.active_portal_authentication = lambda: None
        tree = ExplorationTree()
        source = tree._new_node((0, 0), (0, 0), 0)
        other = tree._new_node((4, 0), (4, 0), 1)
        source['junction_confirmed'] = True
        fixture.junction_portal_authentication = SimpleNamespace(
            node_id=source['id'], phase="watching", all_finished=lambda: False,
        )
        proposals = [dict(heading=h, reach=3, physical_gate=None)
                     for h in (0, 2.0, 4.0)]
        tree.observe((4, 0), (4, 0), proposals, 2, 2, 1,
                     line_is_free=lambda a, b: True,
                     pinned_node_id=fixture.portal_authentication_node_to_pin())
        self.assertEqual(tree.current_node_id, source['id'])
        self.assertEqual(source['physical_xy'], [0, 0])
        self.assertEqual(len(tree.nodes), 2)
        self.assertEqual(other['branch_ids'], [])
        fixture.junction_portal_authentication.phase = "complete"
        tree.observe((4, 0), (4, 0), proposals, 3, 3, 1,
                     line_is_free=lambda a, b: True,
                     pinned_node_id=fixture.portal_authentication_node_to_pin())
        self.assertEqual(tree.current_node_id, other['id'])

    def test_completed_task_releases_junction_pin(self):
        fixture = GPBranchFrontierMixin()
        fixture.active_portal_authentication = lambda: None
        fixture.junction_portal_authentication = SimpleNamespace(
            node_id=12, phase="complete", all_finished=lambda: True,
        )
        self.assertIsNone(fixture.portal_authentication_node_to_pin())


class PortalCrossingObservationOwnershipTest(unittest.TestCase):
    def test_new_observation_does_not_abort_certified_crossing_at_gate(self):
        fixture=GPBranchFrontierMixin()
        fixture.portal_transaction=SimpleNamespace(active=True)
        fixture.portal_authentication_by_node={}
        fixture.junction_portal_authentication=None
        # No tree access or task replacement is allowed before physical commit.
        fixture._sync_portal_authentication(None,None,None,10.0)
        self.assertEqual(fixture.portal_authentication_by_node,{})
        self.assertIsNone(fixture.junction_portal_authentication)


class ReachableInteriorGoalTest(unittest.TestCase):
    def test_approach_goal_extends_past_reachable_anchor_on_portal_axis(self):
        self.assertEqual(
            advance_goal_along_portal_axis((0.5, 0.2), (0, 0), (1, 0), 0.45),
            (0.95, 0.2),
        )

    def test_locked_odom_waypoint_is_reprojected_after_slam_map_correction(self):
        locked_odom = (2.0, 1.0)
        first = reproject_locked_entry_goal(lambda target, source, x, y: (x + 1.0, y), "map", locked_odom)
        corrected = reproject_locked_entry_goal(lambda target, source, x, y: (x + 1.3, y - 0.2), "map", locked_odom)
        self.assertEqual(first, (3.0, 1.0))
        self.assertEqual(corrected, (3.3, 0.8))
        self.assertIsNone(reproject_locked_entry_goal(lambda *args: None, "map", None))

    def test_chooses_reachable_point_past_entry_depth(self):
        points = [(0.2, 0.0), (0.95, 0.0), (1.0, 0.0)]
        self.assertEqual(
            select_reachable_interior_goal(points, (0, 0), (1, 0), (1, 0), 0.9), 2,
        )

    def test_uses_nearest_available_interior_point(self):
        points = [(0.2, 0.0), (0.96, 0.0), (1.3, 0.0)]
        self.assertEqual(
            select_reachable_interior_goal(points, (0, 0), (1, 0), (1, 0), 0.9), 1,
        )

    def test_selects_deepest_reachable_approach_with_forward_progress(self):
        points = [(0.2, 0.0), (0.8, 0.1), (0.7, 0.0)]
        self.assertEqual(select_reachable_approach_goal(points, (0, 0), (1, 0), (0.1, 0)), 1)

    def test_prefers_centered_reachable_approach_over_deeper_lateral_cell(self):
        points = [(1.3, 0.65), (0.95, 0.05), (0.8, 0.0)]
        self.assertEqual(
            select_reachable_approach_goal(points, (0, 0), (1, 0), (0, 0), max_lateral=0.3),
            1,
        )

    def test_rejects_approach_without_forward_progress(self):
        self.assertIsNone(select_reachable_approach_goal([(0.1, 0)], (0, 0), (1, 0), (0.1, 0)))

    def test_remote_room_is_not_an_interior_or_approach_fallback(self):
        # Reproduce a north-facing gate at x16 selecting the north arm at x0.
        points=[(0.85,4.95),(16.2,20.0)]
        gate=(16.2,1.74);destination=(16.4,4.14)
        self.assertIsNone(select_reachable_interior_goal(
            points,gate,destination,(16.3,2.74),0.95))
        self.assertIsNone(select_reachable_approach_goal(
            points,gate,destination,(16.0,-1.25)))

    def test_rejects_gate_side_fallback(self):
        points = [(0.2, 0.0), (0.5, 0.0)]
        self.assertIsNone(
            select_reachable_interior_goal(points, (0, 0), (1, 0), (1, 0), 0.9),
        )



class ReturnSidestepGoalTest(unittest.TestCase):
    def test_selects_reachable_lateral_waypoint_without_advancing_through_gate(self):
        points = [(0.0, 0.0), (0.0, 0.9), (0.1, -0.95), (0.8, 0.8)]
        costs = [0.0, 1.0, 1.2, 0.5]
        self.assertEqual(
            select_reachable_return_sidestep_goal(
                points, costs, (0.0, 0.0), (0.0, 0.0), (1.0, 0.0),
                minimum_lateral=0.55),
            1,
        )

    def test_avoids_failed_approach_segment_and_destination_ward_targets(self):
        points = [(0.8, 0.8), (0.1, 0.9), (0.0, -0.9)]
        costs = [0.1, 1.0, 1.2]
        self.assertEqual(
            select_reachable_return_sidestep_goal(
                points, costs, (0.0, 0.0), (0.0, 0.0), (1.0, 0.0),
                blocked_approach_points=[(0.35, 0.0)],
                blocked_approach_radius=0.34),
            2,
        )

    def test_rejects_sidestep_when_no_reachable_local_lateral_point_exists(self):
        self.assertIsNone(
            select_reachable_return_sidestep_goal(
                [(0.0, 0.0), (0.4, 0.1), (2.0, 0.9)], [0.0, 0.2, 0.3],
                (0.0, 0.0), (0.0, 0.0), (1.0, 0.0)),
        )



class WorkItemObservationPreemptionTest(unittest.TestCase):
    def test_suspended_work_item_resumes_with_same_identity_after_observation(self):
        class Fixture(GlobalFrontierWorkItemLifecycleMixin):
            def __init__(self):
                self.place_work_items = PlaceWorkItemLedger(merge_radius=0.35)
                self.events = []
                self.suspended_work_item_id = None
                self.active_work_item_id = None
                self.active_work_item_attempt_id = None
                self.active_work_item_place_id = None
                self.active_work_item_goal = None
                self.active_work_item_route_kind = None
                self.active_work_item_dispatch_announced = False

            def publish_status(self, event, **data):
                self.events.append((event, data))

        fixture = Fixture()
        item_id = fixture.place_work_items.dispatch(8, (1.0, 2.0), 1.0, map_epoch=3)
        fixture.active_work_item_id = item_id
        fixture.active_work_item_attempt_id = fixture.place_work_items.active_attempt_id(item_id)
        fixture.active_work_item_place_id = 8
        fixture.active_work_item_goal = (1.0, 2.0)
        fixture.active_work_item_route_kind = "frontier_endpoint"

        fixture.suspend_active_work_item(2.0, "portal_junction_observation_period")
        record = next(item for item in fixture.place_work_items.snapshot() if item["id"] == item_id)
        self.assertEqual(record["state"], "unresolved")
        self.assertEqual(record["attempts"][-1]["state"], "suspended")
        self.assertEqual(fixture.suspended_work_item_id, item_id)

        selection = SimpleNamespace(
            route_kind="frontier_endpoint", place_hops=0, x=1.0, y=2.0,
            work_item_id=item_id, component={"epoch": 4},
        )
        resumed = fixture.dispatch_active_work_item(
            SimpleNamespace(header=SimpleNamespace(frame_id="map")),
            selection, {"id": 8}, 3.0,
        )
        record = next(item for item in fixture.place_work_items.snapshot() if item["id"] == item_id)
        self.assertEqual(resumed, item_id)
        self.assertEqual(record["attempts"][-1]["state"], "active")
        self.assertIsNone(fixture.suspended_work_item_id)
        self.assertIn("work_item_resumed", [event for event, _ in fixture.events])


class GPObservationWorkItemFreezeTest(unittest.TestCase):
    def test_nested_authentication_waits_for_current_slam_place_scope(self):
        class Tree:
            current_node_id = 7
            nodes = {7: {"id": 7, "junction_confirmed": True, "branch_ids": [71]}}
            branches = {71: {
                "id": 71, "node_id": 7, "state": "pending", "stable": True,
                "passability": "open", "physical_gate": (2.0, 0.0), "heading": 0.0,
            }}

        class Fixture(GPBranchFrontierMixin):
            active_semantic_room = {"place_id": 8, "node_id": 1}
            current_physical_place_id = 8
            portal_authentication_by_node = {}
            junction_portal_authentication = None

        fixture = Fixture()
        fixture._sync_portal_authentication(Tree(), None, None, 1.0)
        self.assertFalse(getattr(fixture, "semantic_room_suppressed_junction_ids", set()))
        self.assertEqual(fixture.portal_authentication_by_node, {})

        self.assertNotIn(7, getattr(fixture, "semantic_room_suppressed_junction_ids", set()))

    def test_active_room_suppression_uses_active_region_during_route_handoff(self):
        class Tree:
            current_node_id = 9
            nodes = {9: {"id": 9, "junction_confirmed": True, "branch_ids": [91]}}
            branches = {91: {"id": 91, "node_id": 9, "state": "pending",
                             "stable": True, "passability": "open",
                             "physical_gate": (2.0, 0.0), "heading": 0.0}}

        class Fixture(GPBranchFrontierMixin):
            active_semantic_room = {"place_id": 8, "node_id": 1}
            current_physical_place_id = None
            active_frontier_region_id = 8
            portal_authentication_by_node = {}
            junction_portal_authentication = None

        fixture = Fixture()
        fixture._sync_portal_authentication(Tree(), None, None, 1.0)
        self.assertFalse(getattr(fixture, "semantic_room_suppressed_junction_ids", set()))
        self.assertEqual(fixture.portal_authentication_by_node, {})

    def test_sensor_only_fan_waits_without_preempting_until_sections_measured(self):
        class Tree:
            current_node_id = 2
            nodes = {2: {"id": 2, "junction": True, "junction_confirmed": False,
                         "branch_ids": [31, 32, 33], "physical_xy": (0.0, 0.0),
                         "parent_branch_id": None}}
            branches = {bid: {"id": bid, "node_id": 2, "state": "pending",
                              "stable": True,
                              "passability": "unknown" if bid == 31 else "open",
                              "physical_gate": (2.0*math.cos(heading),2.0*math.sin(heading)),
                              "heading": heading,
                              "map_geometry": {"direction": [
                                  math.cos(heading), math.sin(heading)]}}
                        for bid, heading in zip((31, 32, 33),
                            (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0))}

        class Fixture(GPBranchFrontierMixin):
            def __init__(self):
                self.gp_exploration_tree = Tree()
                self.gp_observation_tree = None
                self.portal_authentication_by_node = {}
                self.junction_portal_authentication = None
                self.active_work_item_id = None
                self.events = []
            def publish_status(self, event, **payload):
                self.events.append((event, payload))

        fixture = Fixture()
        fixture._sync_portal_authentication(Tree(), None, None, 10.0)
        task = fixture.portal_authentication_by_node[2]
        self.assertEqual(set(task.branches), set())
        self.assertEqual(set(Tree.branches), {31,32,33})
        self.assertFalse(any(event == "portal_authentication_preempted_graph_route"
                             for event, _ in fixture.events))
        for branch in Tree.branches.values():
            heading=branch['heading'];gx,gy=branch['physical_gate']
            branch['map_geometry'].update(width=2.,physical_bounds=[
                [gx-math.sin(heading),gy+math.cos(heading)],
                [gx+math.sin(heading),gy-math.cos(heading)]])
        fixture._sync_portal_authentication(Tree(), None, None, 11.0)
        self.assertEqual(set(task.branches), {31,32,33})
        self.assertTrue(all(a.gate_geometry[1] is not None for a in task.branches.values()))

    def test_source_side_geometry_wait_remeasures_without_cancelling_active_auth(self):
        class Tree:
            current_node_id=2
            nodes={2:dict(id=2,junction=True,junction_confirmed=True,
                          branch_ids=[31,32,33],physical_xy=(0.,0.),parent_branch_id=None)}
            branches={}
        for bid,heading in zip((31,32,33),(0.,2.*math.pi/3.,4.*math.pi/3.)):
            gx,gy=-math.cos(heading),-math.sin(heading)
            Tree.branches[bid]=dict(id=bid,node_id=2,state='pending',stable=True,
                passability='open',heading=heading,physical_gate=(gx,gy),
                map_geometry=dict(width=2.,physical_bounds=[
                    [gx-math.sin(heading),gy+math.cos(heading)],
                    [gx+math.sin(heading),gy-math.cos(heading)]]))
        class Fixture(GPBranchFrontierMixin):
            def __init__(self):
                self.gp_exploration_tree=Tree();self.gp_observation_tree=None
                self.portal_authentication_by_node={};self.junction_portal_authentication=None
                self.active_work_item_id=None;self.events=[]
            def publish_status(self,event,**payload):self.events.append((event,payload))
        f=Fixture();f._sync_portal_authentication(Tree(),None,None,10.)
        task=f.portal_authentication_by_node[2]
        self.assertEqual(task.branches,{})
        self.assertEqual(set(Tree.branches),{31,32,33})
        self.assertFalse(any(e=='portal_authentication_preempted_graph_route' for e,_ in f.events))
        self.assertEqual(sum(e=='portal_authentication_waiting_source_side_geometry' for e,_ in f.events),3)
        f._sync_portal_authentication(Tree(),None,None,11.)
        self.assertEqual(sum(e=='portal_authentication_waiting_source_side_geometry' for e,_ in f.events),3)
        for source in (None,(float('nan'),0.),(-1.,0.)):
            Tree.nodes[2]['physical_xy']=source
            f._sync_portal_authentication(Tree(),None,None,11.5)
            self.assertEqual(task.branches,{})
        Tree.nodes[2]['physical_xy']=(0.,0.)
        branch=Tree.branches[31];branch['physical_gate']=(1.,0.)
        branch['map_geometry']['physical_bounds']=[[1.,-1.],[1.,1.]]
        f._sync_portal_authentication(Tree(),None,None,12.)
        self.assertIn(31,task.branches)
        # Changed pending sections can be withdrawn and later remeasured.
        branch['physical_gate']=(-1.,0.);branch['map_geometry']['physical_bounds']=[[-1.,-1.],[-1.,1.]]
        f._sync_portal_authentication(Tree(),None,None,13.)
        self.assertNotIn(31,task.branches)
        branch['physical_gate']=(1.,0.);branch['map_geometry']['physical_bounds']=[[1.,-1.],[1.,1.]]
        f._sync_portal_authentication(Tree(),None,None,14.)
        auth=task.branches[31];auth.begin((0.,0.));task.active_branch_id=31
        frozen=(auth.gate_xy,auth.destination_xy,auth.state)
        branch['physical_gate']=(-1.,0.);branch['map_geometry']['physical_bounds']=[[-1.,-1.],[-1.,1.]]
        f._sync_portal_authentication(Tree(),None,None,15.)
        self.assertIs(task.branches[31],auth)
        self.assertEqual((auth.gate_xy,auth.destination_xy,auth.state),frozen)

    def test_collapsed_stable_fan_does_not_start_authentication(self):
        class Tree:
            current_node_id = 2
            nodes = {2: {"id": 2, "junction": True, "junction_confirmed": False,
                         "branch_ids": [31, 32, 33], "physical_xy": (0.0, 0.0),
                         "parent_branch_id": None}}
            branches = {bid: {"id": bid, "node_id": 2, "state": "pending",
                              "stable": True, "passability": "open",
                              "physical_gate": (1.0, 0.0), "heading": heading}
                        for bid, heading in zip((31, 32, 33), (-0.24, 0.50, math.pi))}

        class Fixture(GPBranchFrontierMixin):
            portal_authentication_by_node = {}
            junction_portal_authentication = None
            active_semantic_room = None
            current_physical_place_id = 1
            active_frontier_region_id = 1

        fixture = Fixture()
        fixture._sync_portal_authentication(Tree(), None, None, 10.0)
        self.assertEqual(fixture.portal_authentication_by_node, {})

    def test_auth_mask_and_physical_start_wait_until_old_ordinary_lease_ack(self):
        class Tree:
            current_node_id=1
            nodes={1:dict(id=1,junction_confirmed=True,branch_ids=[9],parent_branch_id=None,physical_xy=(0.,0.))}
            branches={9:dict(id=9,node_id=1,state='pending',stable=True,passability='open',
                physical_gate=(1.,0.),heading=0.,map_geometry=dict(width=2.,physical_bounds=[[1.,-1.],[1.,1.]]))}
        class Fixture(GPBranchFrontierMixin):
            def __init__(self):
                self.gp_exploration_tree=Tree();self.gp_observation_tree=None
                self.portal_authentication_by_node={};self.junction_portal_authentication=None
                self.active_frontier=(0,0,.5,.5);self.active_work_item_id=None
                self.awaiting_controller_lease_release_ack=False;self.pending_controller_lease_releases={}
                self.mask_modes=[];self.place_navigation_constraints=SimpleNamespace(set_mode=lambda node,bounded:self.mask_modes.append(bounded))
                self.events=[];self.releases=[]
            def release_active_frontier(self,**fields):
                self.releases.append(fields);self.active_frontier=None
                self.awaiting_controller_lease_release_ack=True
                self.pending_controller_lease_releases={'old_owned_transaction':object()}
            def publish_status(self,event,**fields):self.events.append((event,fields))
        f=Fixture();f._sync_portal_authentication(Tree(),None,None,2.)
        auth=f.junction_portal_authentication.branches[9]
        f._sync_portal_authentication(Tree(),None,None,10.)
        self.assertEqual(f.mask_modes,[])
        self.assertEqual(auth.state,'pending');self.assertIsNone(auth.source_xy)
        self.assertFalse(any(e=='portal_authentication_branch_started' for e,_ in f.events))
        self.assertTrue(f.releases[0]['await_controller_release_ack'])
        self.assertEqual(len(f.releases),1)
        f.awaiting_controller_lease_release_ack=False
        f._sync_portal_authentication(Tree(),None,None,11.)
        self.assertEqual(f.mask_modes,[]);self.assertEqual(auth.state,'pending')
        # Existing exact-identity ACK handler removes the owned pending record.
        f.pending_controller_lease_releases={}
        f._sync_portal_authentication(Tree(),None,None,12.)
        self.assertEqual(f.mask_modes,[False]);self.assertEqual(auth.state,'entering')
        self.assertTrue(any(e=='portal_authentication_branch_started'for e,_ in f.events))

    def test_confirmed_junction_suspends_active_work_item(self):
        class Tree:
            current_node_id = 1
            nodes = {1: {
                "id": 1, "junction_confirmed": True, "branch_ids": [9],
                "parent_branch_id": None, "physical_xy": (0.0, 0.0),
            }}
            branches = {9: {
                "id": 9, "node_id": 1, "state": "pending", "stable": True,
                "passability": "open", "physical_gate": (1.0, 0.0),
                "heading": 0.0,
                "map_geometry": {"width": 2., "physical_bounds": [[1.,-1.],[1.,1.]]},
            }}

        class Fixture(GPBranchFrontierMixin, GlobalFrontierWorkItemLifecycleMixin):
            def __init__(self):
                self.gp_exploration_tree = Tree()
                self.gp_observation_tree = JunctionObservationTree()
                self.portal_authentication_by_node = {}
                self.junction_portal_authentication = None
                self.active_frontier = (1, 2, 0.5, 0.5)
                self.active_portal_authentication_phase = ""
                self.graph_route_plan_lease_active = True
                self.last_graph_route_plan = object()
                self.last_graph_route_plan_signature = object()
                self.graph_route_action_transaction = object()
                self.graph_route_probe_id = 77
                self.active_portal_probe_id = None
                self.released_frontiers = 0
                self._clear_graph_route_plan_lease = lambda _reason="": setattr(
                    self, "graph_route_plan_lease_active", False
                )
                self.release_active_frontier = lambda **_kwargs: setattr(
                    self, "released_frontiers", self.released_frontiers + 1
                )
                self.active_work_item_id = None
                self.active_work_item_attempt_id = None
                self.active_work_item_place_id = None
                self.active_work_item_goal = None
                self.active_work_item_route_kind = None
                self.active_work_item_dispatch_announced = False
                self.suspended_work_item_id = None
                self.place_work_items = PlaceWorkItemLedger(merge_radius=0.35)
                self.events = []

            def publish_status(self, event, **data):
                self.events.append((event, data))

        fixture = Fixture()
        item_id = fixture.place_work_items.dispatch(1, (0.5, 0.5), 1.0, map_epoch=1)
        fixture.active_work_item_id = item_id
        fixture.active_work_item_attempt_id = fixture.place_work_items.active_attempt_id(item_id)
        fixture.active_work_item_place_id = 1
        fixture.active_work_item_goal = (0.5, 0.5)
        fixture.active_work_item_route_kind = "frontier_endpoint"

        with patch("gp_branch_frontier.rospy.get_param", return_value=0.30, create=True):
            fixture._sync_portal_authentication(Tree(), None, None, 2.0)

        item = next(record for record in fixture.place_work_items.snapshot() if record["id"] == item_id)
        self.assertEqual(item["state"], "unresolved")
        self.assertEqual(item["attempts"][-1]["state"], "suspended")
        self.assertEqual(fixture.suspended_work_item_id, item_id)
        self.assertIsNone(fixture.active_work_item_id)
        self.assertIn("portal_authentication_preempted_graph_route", [event for event, _ in fixture.events])
        self.assertIsNone(fixture.last_graph_route_plan)
        self.assertIsNone(fixture.graph_route_action_transaction)
        self.assertIsNone(fixture.graph_route_probe_id)
        self.assertFalse(fixture.graph_route_plan_lease_active)
        self.assertEqual(fixture.released_frontiers, 1)
        self.assertEqual(fixture.junction_portal_authentication.phase, "watching")
        self.assertIsNone(fixture.junction_portal_authentication.active_branch_id)


class GPAuthenticationRoutePriorityTest(unittest.TestCase):
    class Fixture(GlobalFrontierPlanningSelectionMixin):
        def __init__(self, candidate):
            self.candidate = candidate
            self.pending_local_egress = {"source_route_id": 41}
            self.pending_portal_retry = object()
            self.local_egress_place_lease = SimpleNamespace(cleared=False)
            self.local_egress_place_lease.clear = lambda: setattr(
                self.local_egress_place_lease, "cleared", True
            )
            self.egress_selected = False
            self.events = []

        def portal_authentication_owns_navigation(self):
            return True

        def select_pending_local_egress(self, _snapshot):
            self.egress_selected = True
            raise AssertionError("generic egress must not preempt GP auth")

        def _choose_frontier_for_new_action(self, *_args):
            return self.candidate

        def publish_status(self, event, **fields):
            self.events.append((event, fields))

    @staticmethod
    def snapshot():
        return SimpleNamespace(
            message=object(), route_graph=object(), map_context=object(),
            now=1.0, robot_map=(0.0, 0.0), robot_yaw_map=0.0,
        )

    def test_gp_auth_target_preempts_queued_generic_egress(self):
        candidate = object()
        fixture = self.Fixture(candidate)

        selected, mode, wait = fixture.select_next_active_frontier(self.snapshot())

        self.assertIs(selected, candidate)
        self.assertEqual(mode, "gp_portal_authentication")
        self.assertFalse(wait)
        self.assertFalse(fixture.egress_selected)
        self.assertIsNone(fixture.pending_local_egress)
        self.assertTrue(fixture.local_egress_place_lease.cleared)
        self.assertIsNotNone(fixture.pending_portal_retry)

    def test_gp_watch_wait_keeps_generic_egress_from_stealing_route(self):
        fixture = self.Fixture(None)

        selected, mode, wait = fixture.select_next_active_frontier(self.snapshot())

        self.assertIsNone(selected)
        self.assertEqual(mode, "gp_portal_authentication_wait")
        self.assertTrue(wait)
        self.assertFalse(fixture.egress_selected)
        self.assertIsNone(fixture.pending_local_egress)


class GPAuthenticationPhaseSyncTest(unittest.TestCase):
    def test_state_machine_turning_phase_is_published_to_route_resolver(self):
        fixture = GPBranchFrontierMixin()
        fixture.active_portal_authentication_phase = "entering"

        phase = fixture.sync_portal_authentication_phase(
            SimpleNamespace(state="turning")
        )

        self.assertEqual(phase, "turning")
        self.assertEqual(fixture.active_portal_authentication_phase, "turning")

    def test_terminal_auth_state_clears_route_phase(self):
        fixture = GPBranchFrontierMixin()
        fixture.active_portal_authentication_phase = "returning"

        phase = fixture.sync_portal_authentication_phase(
            SimpleNamespace(state="completed")
        )

        self.assertEqual(phase, "")
        self.assertEqual(fixture.active_portal_authentication_phase, "")


class GPAuthenticationTurnSupervisorHandoffTest(unittest.TestCase):
    class Auth:
        state = "turning"
        node_id = 2
        branch_id = 3
        turn_direction = "counterclockwise"
        target_yaw = 1.0

        def update_rotation(self, yaw, stable_samples=3):
            self.last_yaw = yaw
            self.last_stable_samples = stable_samples
            return abs(yaw - self.target_yaw) <= 0.40

        def finish_turn(self, yaw):
            if abs(yaw - self.target_yaw) > 0.40:
                return False
            self.state = "returning"
            return True

    class Fixture(GPBranchFrontierMixin):
        def __init__(self):
            self.auth = GPAuthenticationTurnSupervisorHandoffTest.Auth()
            self.junction_portal_authentication = SimpleNamespace(
                active_branch_id=3, branches={3: self.auth},
            )
            self.pose_odom = SimpleNamespace(theta=0.68, x=0.2, y=0.1)
            self.active_route_id = 17
            self.active_portal_authentication_phase = "turning"
            self.place_departure = SimpleNamespace(active=False)
            self.events = []
            self.released = []

        def active_portal_authentication(self):
            return self.auth

        def publish_status(self, event, **fields):
            self.events.append((event, fields))

        def release_active_frontier(self, **kwargs):
            self.released.append(kwargs)

    def test_supervisor_turn_complete_advances_matching_auth_route(self):
        fixture = self.Fixture()
        completed = fixture.complete_portal_turn_from_supervisor({
            "event": "turn_completed", "route_id": 17,
            "active_route_kind": "frontier_turn_connector",
        })
        self.assertTrue(completed)
        self.assertEqual(fixture.auth.state, "returning")
        self.assertEqual(fixture.auth.last_stable_samples, 1)
        self.assertEqual(fixture.active_portal_authentication_phase, "returning")
        self.assertTrue(fixture.events[-1][1]["supervisor_acknowledged"])
        self.assertEqual(len(fixture.released), 1)

    def test_completion_event_pose_survives_coalesced_pose_cache(self):
        fixture = self.Fixture()
        fixture.pose_odom.theta = -0.6
        self.assertTrue(fixture.complete_portal_turn_from_supervisor({
            "route_id": 17, "active_route_kind": "frontier_turn_connector",
            "pose": [0.2, 0.1, 0.68],
        }))
        self.assertEqual(fixture.auth.last_yaw, 0.68)
        self.assertEqual(fixture.events[-1][1]["final_yaw"], 0.68)

    def test_wrong_or_invalid_event_yaw_cannot_complete_authentication(self):
        for yaw in (-0.6, float("nan")):
            fixture = self.Fixture()
            self.assertFalse(fixture.complete_portal_turn_from_supervisor({
                "route_id": 17, "active_route_kind": "frontier_turn_connector",
                "pose": [0.2, 0.1, yaw],
            }))
            self.assertEqual(fixture.auth.state, "turning")
            self.assertEqual(fixture.released, [])

    def test_stale_turn_completion_cannot_advance_auth_route(self):
        fixture = self.Fixture()
        self.assertFalse(fixture.complete_portal_turn_from_supervisor({
            "event": "turn_completed", "route_id": 16,
            "active_route_kind": "frontier_turn_connector",
        }))
        self.assertFalse(fixture.complete_portal_turn_from_supervisor({
            "event": "turn_completed", "route_id": 17,
            "active_route_kind": "frontier_endpoint",
        }))
        self.assertEqual(fixture.auth.state, "turning")
        self.assertEqual(fixture.released, [])




class PortalWidthHandoffTest(unittest.TestCase):
    def fixture(self,branch):
        from global_frontier_portal_belief import PortalHypothesisLedger
        from global_frontier_portal_authentication import DoorwayAuthentication
        fixture=GPBranchFrontierMixin()
        fixture.portal_hypothesis_ledger=PortalHypothesisLedger()
        fixture.current_physical_place_id=1
        fixture.gp_exploration_tree=SimpleNamespace(branches={1:branch})
        fixture.publish_status=lambda *args,**kwargs:None
        auth=DoorwayAuthentication(1,2,(1.,0.),(3.,0.));auth.state='completed';auth.inside_xy=(2.,0.)
        return fixture,auth

    def test_unmapped_gp_width_is_kept_in_canonical_portal_record(self):
        branch=dict(id=1,node_id=2,map_geometry=dict(width=None),reach=3.,angular_width=1.)
        fixture,auth=self.fixture(branch)
        result=fixture._register_authenticated_gp_portal(auth,10.)
        self.assertAlmostEqual(result['opening_width'],6*math.sin(.5))

    def test_canonical_place_seed_uses_measured_inside_not_gp_ray_endpoint(self):
        fixture,auth=self.fixture(dict(id=1,node_id=2,map_geometry=dict(width=2.)))
        auth.destination_xy=(3.,0.)  # A predicted endpoint can be beyond a wall.
        auth.inside_xy=(1.8,.2)
        result=fixture._register_authenticated_gp_portal(auth,10.)
        self.assertEqual(result['physical_destination'],[1.8,.2])
        self.assertEqual(result['gp_predicted_destination'],[3.,0.])
        self.assertEqual(result['gp_inside_witness'],[1.8,.2])
        self.assertIsNone(result['destination_place_id'])

    def test_registered_bounds_use_entry_frozen_section_after_branch_updates(self):
        branch=dict(id=1,node_id=2,map_geometry=dict(width=2.,physical_bounds=[[1.,-1.],[1.,1.]]))
        fixture,auth=self.fixture(branch)
        auth.state='pending'
        auth.refresh_pending_geometry((1.,0.),(3.,0.),2.,branch['map_geometry']['physical_bounds'])
        auth.begin((0.,0.));auth.state='completed';auth.inside_xy=(2.,0.)
        branch['map_geometry'].update(width=3.,physical_bounds=[[2.,-1.5],[2.,1.5]])
        self.assertFalse(auth.refresh_pending_geometry((2.,0.),(4.,0.),3.,branch['map_geometry']['physical_bounds']))
        result=fixture._register_authenticated_gp_portal(auth,10.)
        self.assertEqual(result['physical_gate'],[1.,0.])
        self.assertEqual(result['physical_bounds'],[[1.,-1.],[1.,1.]])
        self.assertEqual(result['opening_width'],2.)
        self.assertEqual(result['gp_authentication_required_depth'],auth.required_depth)

    def test_missing_inside_witness_cannot_certify_a_place_seed(self):
        fixture,auth=self.fixture(dict(id=1,node_id=2,map_geometry=dict(width=2.)))
        auth.inside_xy=None
        self.assertIsNone(fixture._register_authenticated_gp_portal(auth,10.))
        self.assertEqual(auth.failure_reason,'inside_witness_unavailable')
        self.assertEqual(fixture.portal_hypothesis_ledger.snapshot(),[])

    def test_nested_portal_requires_distinct_slam_destination_proof(self):
        from unittest.mock import patch
        for evidence in (None, dict(reject=False,reason='slam_evidence_incomplete',
                                    source_label=0,destination_label=2)):
            with self.subTest(evidence=evidence):
                fixture,auth=self.fixture(dict(id=1,node_id=2,map_geometry=dict(width=2.)))
                fixture.active_semantic_room=dict(place_id=2)
                with patch('global_frontier_place_portal_validation.authenticated_place_portal_evidence',
                           return_value=evidence):
                    self.assertIsNone(fixture._register_authenticated_gp_portal(auth,10.))
                self.assertEqual(auth.state,'failed')
                self.assertEqual(auth.failure_reason,'slam_place_boundary_unproven')
                self.assertEqual(fixture.portal_hypothesis_ledger.snapshot(),[])

    def test_no_width_evidence_cannot_create_an_unbounded_portal(self):
        fixture,auth=self.fixture(dict(id=1,node_id=2,map_geometry=dict(width=None)))
        self.assertIsNone(fixture._register_authenticated_gp_portal(auth,10.))
        self.assertEqual(auth.state,'failed')
        self.assertEqual(fixture.portal_hypothesis_ledger.snapshot(),[])

class AuthenticationUnrecoverableReturnTest(unittest.TestCase):
    def fixture(self, returned=False):
        from global_frontier_portal_authentication import DoorwayAuthentication
        fixture=GPBranchFrontierMixin()
        auth=DoorwayAuthentication(1,1,(0.,0.),(1.,0.))
        auth.begin((-1.,0.))
        auth.request_failure_return('authentication_physical_stall:entering',(1.,0.))
        auth.return_recovery_final_blocked=True
        fixture.active_portal_authentication=lambda:auth
        fixture.pose_odom=SimpleNamespace(x=-1. if returned else 1.,y=0.,theta=0.)
        fixture.events=[];fixture.releases=[];fixture.finished=[]
        fixture.publish_status=lambda event,**kw:fixture.events.append((event,kw))
        fixture.release_active_frontier=lambda **kw:fixture.releases.append(kw)
        fixture.finish_portal_authentication_branch=lambda **kw:fixture.finished.append(kw)
        return fixture,auth

    def test_exhausted_recovery_fails_the_branch_once_and_advances(self):
        # The branch is still recorded as failed and the return obligation is
        # still an explicit ledger fact, but the observation period advances
        # instead of freezing the mission (run 20261101_191225 lost 13 minutes
        # of healthy exploration to the old "seal the trial" behaviour).
        fixture,auth=self.fixture()
        fixture._update_portal_authentication_from_map(None,None,None,[],0.,100.)
        self.assertEqual(auth.state,'failed')
        self.assertEqual(auth.failure_reason,'return_unrecoverable')
        self.assertEqual(len(fixture.finished),1)
        self.assertEqual(len(fixture.releases),1)
        self.assertEqual([event for event,kw in fixture.events],
                         ['portal_authentication_return_unrecoverable'])
        self.assertTrue(fixture.events[0][1]['return_obligation_pending'])
        # Re-entering the pipeline must not report the same branch twice.
        fixture._update_portal_authentication_from_map(None,None,None,[],0.,110.)
        self.assertEqual(
            [event for event,kw in fixture.events].count(
                'portal_authentication_return_unrecoverable'), 1)

    def test_actual_return_wins_over_exhausted_recovery_flag(self):
        fixture,auth=self.fixture(returned=True)
        fixture._update_portal_authentication_from_map(None,None,None,[],0.,100.)
        self.assertEqual(auth.state,'failed')
        self.assertIn('returned_to_junction_node_after_failure',[e['event'] for e in auth.events])
        self.assertNotIn('portal_authentication_return_unrecoverable',[e for e,kw in fixture.events])
        self.assertEqual(len(fixture.finished),1)


class AuthenticationControllerReleaseBarrierTest(unittest.TestCase):
    def test_phase_successor_waits_for_exact_predecessor_release_ack(self):
        fixture=GPBranchFrontierMixin()
        fixture.active_portal_authentication=lambda:object()
        compiled=[]
        fixture._portal_authentication_route_candidate=lambda *a:compiled.append(a) or 'candidate'
        fixture.awaiting_controller_lease_release_ack=True
        self.assertIsNone(fixture.choose_gp_branch_frontier(None,None,0.,None,None,None))
        self.assertEqual(compiled,[])
        fixture.awaiting_controller_lease_release_ack=False
        self.assertEqual(fixture.choose_gp_branch_frontier(None,None,0.,None,None,None),'candidate')
        self.assertEqual(len(compiled),1)


class InsideMapEvidenceRetryTest(unittest.TestCase):
    def test_missing_inside_map_keeps_full_body_witness_for_next_map(self):
        import numpy as np
        from global_frontier_portal_authentication import DoorwayAuthentication
        fixture=GPBranchFrontierMixin();auth=DoorwayAuthentication(1,2,(1.,0.),(3.,0.))
        auth.state='inside';auth.source_xy=(0.,0.);auth.inside_xy=(2.,0.)
        fixture.active_portal_authentication=lambda:auth
        fixture.pose_odom=SimpleNamespace(x=2.,y=0.,theta=0.)
        context=SimpleNamespace(known_free=np.ones((3,3),dtype=bool),unknown=np.zeros((3,3),dtype=bool))
        with patch('global_frontier_place_portal_validation.authentication_inside_frontiers',return_value=(None,None)) as evidence:
            for now in (1.,2.):fixture._update_portal_authentication_from_map(None,None,context,[],0.,now)
        self.assertEqual(evidence.call_count,2)
        self.assertEqual(auth.state,'inside');self.assertEqual(auth.inside_xy,(2.,0.))
        self.assertIsNone(auth.failure_reason)


class FullInsideTurnHandoffTest(unittest.TestCase):
    def test_observed_inside_releases_entry_before_new_turn_lease(self):
        import numpy as np
        from global_frontier_portal_authentication import DoorwayAuthentication
        fixture=GPBranchFrontierMixin();auth=DoorwayAuthentication(1,2,(1.,0.),(3.,0.))
        auth.state='inside';auth.source_xy=(0.,0.);auth.inside_xy=(2.,0.)
        fixture.active_portal_authentication=lambda:auth
        fixture.pose_odom=SimpleNamespace(x=2.,y=0.,theta=0.)
        fixture.transform_xy=lambda *args: (_ for _ in ()).throw(AssertionError('latest TF used for inside evidence'))
        fixture._place_partition_tf_key=('map',1.)
        fixture._place_partition_projector=lambda x,y:(x,y)
        fixture.publish_status=lambda *a,**k:None
        releases=[]
        def release(**fields):
            releases.append(fields);fixture.awaiting_controller_lease_release_ack=True
        fixture.release_active_frontier=release
        context=SimpleNamespace(known_free=np.ones((5,5),dtype=bool),unknown=np.zeros((5,5),dtype=bool))
        message=SimpleNamespace(header=SimpleNamespace(frame_id='map',stamp=SimpleNamespace(to_sec=lambda:1.)),
             info=SimpleNamespace(resolution=1.,origin=SimpleNamespace(position=SimpleNamespace(x=0.,y=0.))))
        tree=SimpleNamespace(branches={1:dict(heading=0.)})
        mask=np.zeros((5,5),dtype=bool);mask[2,2]=True
        with patch('global_frontier_place_portal_validation.authentication_inside_frontiers',return_value=(mask,{'gate_map_xy':[1.,0.],'heading_map':0.})):
            fixture._update_portal_authentication_from_map(tree,message,context,[],0.,1.)
        self.assertEqual(auth.state,'turning')
        self.assertEqual(len(releases),1)
        self.assertTrue(releases[0]['await_controller_release_ack'])
        self.assertFalse(releases[0]['queue_successor'])
        compiled=[]
        fixture._portal_authentication_route_candidate=lambda *args:compiled.append(args) or 'turn'
        self.assertIsNone(fixture.choose_gp_branch_frontier(None,None,2.,None,None,None))
        self.assertEqual(compiled,[])
        fixture.awaiting_controller_lease_release_ack=False
        self.assertEqual(fixture.choose_gp_branch_frontier(None,None,3.,None,None,None),'turn')


class EmptyInsideReturnHandoffTest(unittest.TestCase):
    def test_empty_inside_releases_entry_endpoint_and_returns_to_original_node(self):
        import numpy as np
        from global_frontier_portal_authentication import DoorwayAuthentication
        fixture=GPBranchFrontierMixin();auth=DoorwayAuthentication(1,2,(1.,0.),(3.,0.))
        auth.state='inside';auth.source_xy=(0.,0.);auth.inside_xy=(2.,0.)
        fixture.active_portal_authentication=lambda:auth
        fixture.pose_odom=SimpleNamespace(x=2.,y=0.,theta=0.)
        fixture.transform_xy=lambda *args: (_ for _ in ()).throw(AssertionError('latest TF used for inside evidence'))
        fixture._place_partition_tf_key=('map',1.)
        fixture._place_partition_projector=lambda x,y:(x,y)
        fixture.publish_status=lambda *a,**k:None
        released=[];fixture.release_active_frontier=lambda **k:released.append(k)
        context=SimpleNamespace(known_free=np.ones((5,5),dtype=bool),unknown=np.zeros((5,5),dtype=bool))
        message=SimpleNamespace(header=SimpleNamespace(frame_id='map',stamp=SimpleNamespace(to_sec=lambda:1.)))
        tree=SimpleNamespace(branches={1:dict(heading=0.)})
        with patch('global_frontier_place_portal_validation.authentication_inside_frontiers',return_value=(np.zeros((5,5),dtype=bool),{'gate_map_xy':[1.,0.],'heading_map':0.})):
            fixture._update_portal_authentication_from_map(tree,message,context,[],0.,1.)
        self.assertEqual(auth.state,'returning')
        self.assertEqual(auth.failure_reason,'no_frontier_inside_doorway')
        self.assertEqual(auth.route_goal(),auth.source_xy)
        self.assertEqual(len(released),1)
        self.assertTrue(released[0]['await_controller_release_ack'])
        self.assertFalse(released[0]['queue_successor'])



class UnreachableReturnPlaneTest(unittest.TestCase):
    def test_unreachable_plane_retires_candidate_and_returns_to_real_node(self):
        import numpy as np
        from global_frontier_portal_authentication import DoorwayAuthentication
        fixture=GPBranchFrontierMixin()
        fixture.transform_xy=lambda target, source, x, y: (x,y)
        fixture.xy_to_grid_cell=lambda message,x,y: (0,0) if y<0 else (0,1)
        fixture.pose_odom=SimpleNamespace(x=0.,y=-1.)
        fixture.publish_status=lambda *args,**kwargs: None
        fixture.cell_xy=lambda message,row,col: (0.,-1.)
        auth=DoorwayAuthentication(18,6,(0.,0.),(0.,-2.))
        auth.begin((-2.,-.1));auth.inside_xy=(0.,-1.);auth.state="returning"
        message=SimpleNamespace(header=SimpleNamespace(frame_id="map"),
                                info=SimpleNamespace(resolution=.1))
        route=fixture._portal_authentication_route_candidate(message,np.array([[0,-1]]),auth)
        self.assertEqual(auth.failure_reason,"no_reachable_source_side_return_goal")
        self.assertEqual(auth.state,"returning")
        self.assertEqual(route[2:4],auth.source_xy)
        self.assertFalse(auth.update_return(auth.source_xy))
        self.assertEqual(auth.state,"returning")


class AuthenticationMapCoverageOwnershipTest(unittest.TestCase):
    def test_known_free_waypoint_does_not_finish_authentication_motion(self):
        import numpy as np
        from global_frontier_route_state import GlobalFrontierRouteStateMixin
        node=GlobalFrontierRouteStateMixin()
        node.active_route_kind="frontier_endpoint";node.scan_range_max=8.
        node.dead_end_unknown_cells=1
        node.xy_to_grid_cell=lambda message,x,y:(2,2)
        message=SimpleNamespace(info=SimpleNamespace(resolution=.1))
        free=np.ones((5,5),dtype=bool);unknown=np.zeros_like(free)
        args=(message,free,unknown,2,3,(.2,.2),.75)
        self.assertTrue(node.active_endpoint_observed_from_map(*args))
        for phase in ("entering","turning","returning"):
            with self.subTest(phase=phase):
                node.active_portal_authentication_phase=phase
                self.assertFalse(node.active_endpoint_observed_from_map(*args))
        node.active_portal_authentication_phase=""
        self.assertTrue(node.active_endpoint_observed_from_map(*args))


class CompletedPlaceObservationOwnershipTest(unittest.TestCase):
    def test_completed_place_keeps_exit_and_never_restarts_authentication(self):
        node=GPBranchFrontierMixin();node.current_physical_place_id=2
        node.place_work_items=SimpleNamespace(place_is_closed=lambda place:place==2)
        node.junction_portal_authentication=None
        node._sync_portal_authentication(None,None,None,10.)
        self.assertIsNone(node.junction_portal_authentication)


class SourceSideReturnGoalSelectionTest(unittest.TestCase):
    def test_cleared_gate_sidestep_returns_toward_overshot_source(self):
        points=[(-2.2,0.),(-3.8,.6)]
        selected=select_reachable_return_sidestep_goal(points,[1.,.1],(-3.,0.),
            (0.,0.),(1.,0.),source_xy=(-1.,0.),gate_cleared=True)
        self.assertEqual(selected,0)
        self.assertIsNone(select_reachable_return_sidestep_goal(points[1:],[.1],(-3.,0.),
            (0.,0.),(1.,0.),source_xy=(-1.,0.),gate_cleared=True))

    def test_cleared_gate_endpoint_accounts_for_native_arrival_tolerance(self):
        points = [(-0.55, 0.0), (-1.05, 0.0)]
        selected = select_reachable_source_side_return_goal(
            points, [0.1, 5.0], (0., 0.), (1., 0.), (-1.2, 0.),
            resolution=0.1, gate_cleared=True, return_radius=0.75,
            goal_xy_tolerance=0.5)
        self.assertEqual(selected, 1)
        self.assertLessEqual(abs(points[selected][0] + 1.2) + 0.5 + 0.05, 0.75)
        self.assertIsNone(select_reachable_source_side_return_goal(
            points[:1], [0.1], (0., 0.), (1., 0.), (-1.2, 0.),
            resolution=0.1, gate_cleared=True, return_radius=0.75,
            goal_xy_tolerance=0.5))

    def test_return_radius_smaller_than_controller_tolerance_has_no_safe_endpoint(self):
        self.assertIsNone(select_reachable_source_side_return_goal(
            [(-1., 0.)], [1.], (0., 0.), (1., 0.), (-1., 0.),
            gate_cleared=True, return_radius=0.4, goal_xy_tolerance=0.5))

    def test_after_gate_clear_fallback_targets_reachable_junction_cell(self):
        import numpy as np
        points = np.asarray([(-0.55, 0.0), (-1.5, 0.0), (-0.2, 0.0)])
        # The robot has already crossed the Portal plane. Requiring more
        # source-axis progress would keep selecting farther-away cells and
        # may leave the authentication returning forever.
        selected = select_reachable_source_side_return_goal(
            points, [4.0, 0.1, 0.01], (0.0, 0.0), (1.0, 0.0),
            (-0.5, 0.0), resolution=0.1, current_xy=(-2.0, 0.0),
            gate_cleared=True, return_radius=0.75,
        )
        self.assertEqual(selected, 2)
        self.assertIsNone(select_reachable_source_side_return_goal(
            points, [4.0, 0.1, 0.01], (0.0, 0.0), (1.0, 0.0),
            (-0.5, 0.0), resolution=0.1, current_xy=(-2.0, 0.0),
            gate_cleared=True, return_radius=0.04,
        ))

    def test_return_fallback_selects_only_navfn_reachable_source_side_cells(self):
        import numpy as np
        points = np.asarray([
            (0.20, 0.0), (-0.20, 0.45), (-0.80, 0.10), (-1.0, 3.0),
        ])
        selected = select_reachable_source_side_return_goal(
            points, [0.1, 0.2, 1.5, 0.0],
            (0.0, 0.0), (1.0, 0.0), (-1.0, 0.0), resolution=0.05,
        )
        self.assertEqual(selected, 2)
        self.assertIsNone(select_reachable_source_side_return_goal(
            points[:1], [0.1], (0.0, 0.0), (1.0, 0.0),
            (-1.0, 0.0), resolution=0.05,
        ))

    def test_navfn_reachable_goal_can_cross_failed_straight_line_approach(self):
        import numpy as np
        points = np.asarray([(-0.8, 0.0), (-0.8, 1.5)])
        selected = select_reachable_source_side_return_goal(
            points, [1.0, 2.0], (0.0, 0.0), (1.0, 0.0),
            (-3.0, 0.0), resolution=0.1, current_xy=(0.2, 0.0),
            blocked_approach_points=[(-0.25, 0.0)],
            blocked_approach_radius=0.35,
        )
        # The straight segment intersects a historic failed approach point,
        # but Navfn's route to point 0 is reachable and may curve around it.
        self.assertEqual(selected, 0)

    def test_return_fallback_still_rejects_goal_at_failed_approach_sample(self):
        import numpy as np
        selected = select_reachable_source_side_return_goal(
            [(-0.8, 0.0), (-1.5, 0.0)], [1.0, 2.0],
            (0.0, 0.0), (1.0, 0.0), (-3.0, 0.0), resolution=0.1,
            current_xy=(0.2, 0.0), blocked_approach_points=[(-0.8, 0.0)],
            blocked_approach_radius=0.35,
        )
        self.assertEqual(selected, 1)

    def test_return_fallback_skips_previously_stalled_goal(self):
        import numpy as np
        points = np.asarray([(-0.8, 0.0), (-1.5, 0.0), (-2.2, 0.0)])
        selected = select_reachable_source_side_return_goal(
            points, [1.0, 1.5, 4.0], (0.0, 0.0), (1.0, 0.0),
            (-3.0, 0.0), resolution=0.1, current_xy=(0.2, 0.0),
            excluded_goals=[(-0.8, 0.0)],
        )
        self.assertEqual(selected, 1)
        self.assertIsNone(select_reachable_source_side_return_goal(
            points[:1], [1.0], (0.0, 0.0), (1.0, 0.0),
            (-3.0, 0.0), resolution=0.1, current_xy=(0.2, 0.0),
            excluded_goals=[(-0.8, 0.0)],
        ))

    def test_retry_fallback_skips_current_cell_and_advances_toward_source(self):
        import numpy as np
        from global_frontier_portal_authentication import DoorwayAuthentication
        fixture = GPBranchFrontierMixin()
        fixture.transform_xy = lambda target, source, x, y: (x, y)
        fixture.xy_to_grid_cell = lambda message, x, y: (0, 3)
        fixture.cell_xy = lambda message, row, col: (
            (-0.8, 0.0) if col == 0 else
            (-0.9, 0.0) if col == 1 else
            (-1.5, 0.0) if col == 2 else (-0.5, 0.0)
        )
        fixture.pose_odom = SimpleNamespace(x=-0.8, y=0.0)
        fixture.publish_status = lambda *args, **kwargs: None
        auth = DoorwayAuthentication(29, 8, (0.0, 0.0), (1.0, 0.0))
        auth.begin((-1.0, 0.0))
        auth.state = "returning"
        auth.failure_reason = "controller_stall"
        auth.return_recovery_blocked = True
        message = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            info=SimpleNamespace(resolution=0.1),
        )
        route = fixture._portal_authentication_route_candidate(
            message, np.asarray([[1, 5, 10, -1]], dtype=int), auth,
        )
        self.assertIsNotNone(route)
        self.assertEqual(list(route[0:4]), [0, 2, -1.5, 0.0])

    def test_retry_exhaustion_keeps_route_and_selects_source_side_fallback(self):
        import numpy as np
        from global_frontier_portal_authentication import DoorwayAuthentication
        fixture = GPBranchFrontierMixin()
        fixture.transform_xy = lambda target, source, x, y: (x, y)
        fixture.xy_to_grid_cell = lambda message, x, y: (0, 1)
        fixture.cell_xy = lambda message, row, col: (
            (-0.6, 0.0) if col == 0 else
            (0.2, 0.0) if col == 1 else (-1.5, 0.0)
        )
        events = []
        fixture.publish_status = lambda event, **fields: events.append((event, fields))
        auth = DoorwayAuthentication(28, 8, (0.0, 0.0), (1.0, 0.0))
        auth.begin((-1.0, 0.0))
        auth.state = "returning"
        auth.failure_reason = "authentication_physical_stall:turning"
        auth.return_recovery_blocked = True
        auth.return_retry_count = 3
        auth.failed_source_side_return_goals_odom = [(-0.6, 0.0)]
        message = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            info=SimpleNamespace(resolution=0.1),
        )
        route = fixture._portal_authentication_route_candidate(
            message, np.asarray([[0, 2, 3]], dtype=int), auth,
        )
        self.assertIsNotNone(route)
        self.assertEqual(list(route[0:4]), [0, 2, -1.5, 0.0])
        self.assertIn(
            "portal_authentication_source_side_return_goal_selected",
            [event for event, _ in events],
        )

    def test_gate_cleared_return_fallback_targets_junction_not_farther_axis(self):
        import numpy as np
        from global_frontier_portal_authentication import DoorwayAuthentication
        fixture = GPBranchFrontierMixin()
        fixture.transform_xy = lambda target, source, x, y: (x, y)
        fixture.xy_to_grid_cell = lambda message, x, y: (0, 1)
        points = {0: (-1.5, 0.0), 2: (-0.4, 0.0)}
        fixture.cell_xy = lambda message, row, col: points[col]
        fixture.pose_odom = SimpleNamespace(x=-2.0, y=0.0)
        fixture.publish_status = lambda *args, **kwargs: None
        auth = DoorwayAuthentication(26, 8, (0.0, 0.0), (1.0, 0.0))
        auth.begin((-0.5, 0.0))
        auth.state = "returning"
        auth.failure_reason = "authentication_physical_stall:turning"
        auth.return_gate_cleared = True
        message = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            info=SimpleNamespace(resolution=0.1),
        )
        route = fixture._portal_authentication_route_candidate(
            message, np.asarray([[4, -1, 0]], dtype=int), auth,
        )
        self.assertIsNotNone(route)
        self.assertEqual(list(route[0:4]), [0, 2, -0.4, 0.0])

    def test_finally_blocked_return_still_dispatches_source_side_recovery_route(self):
        import numpy as np
        from global_frontier_portal_authentication import DoorwayAuthentication
        fixture = GPBranchFrontierMixin()
        fixture.transform_xy = lambda target, source, x, y: (x, y)
        fixture.xy_to_grid_cell = lambda message, x, y: (0, 1)
        fixture.cell_xy = lambda message, row, col: (-0.6, 0.0) if col == 0 else (0.2, 0.0)
        fixture.pose_odom = SimpleNamespace(x=0.2, y=0.0)
        fixture.publish_status = lambda *args, **kwargs: None
        auth = DoorwayAuthentication(27, 8, (0.0, 0.0), (1.0, 0.0))
        auth.begin((-1.0, 0.0))
        auth.state = "returning"
        auth.failure_reason = "authentication_physical_stall:entering"
        auth.return_recovery_blocked = True
        auth.return_recovery_final_blocked = True
        message = SimpleNamespace(header=SimpleNamespace(frame_id="map"),
                                  info=SimpleNamespace(resolution=0.1))
        route = fixture._portal_authentication_route_candidate(
            message, np.asarray([[0, 2]], dtype=int), auth,
        )
        self.assertIsNotNone(route)
        self.assertEqual(list(route[0:4]), [0, 0, -0.6, 0.0])

    def test_return_fallback_must_advance_past_controller_tolerance(self):
        import numpy as np
        points = np.asarray([(-0.8, 0.0), (-1.5, 0.0), (-0.5, 0.0)])
        selected = select_reachable_source_side_return_goal(
            points, [0.1, 0.5, 0.01], (0.0, 0.0), (1.0, 0.0),
            (-1.0, 0.0), resolution=0.05, current_xy=(-0.8, 0.0),
        )
        self.assertEqual(selected, 1)
        # A reachable cell only 0.1 m farther source-side is inside the
        # controller's goal tolerance and must not become a progress goal.
        self.assertIsNone(select_reachable_source_side_return_goal(
            points[[0, 2]], [0.1, 0.01], (0.0, 0.0), (1.0, 0.0),
            (-1.0, 0.0), resolution=0.05, current_xy=(-0.8, 0.0),
        ))

    def test_return_fallback_does_not_reissue_a_sub_tolerance_source_step(self):
        import numpy as np
        points = np.asarray([(-0.55, 0.0)])
        self.assertIsNone(select_reachable_source_side_return_goal(
            points, [0.1], (0.0, 0.0), (1.0, 0.0),
            (-1.0, 0.0), resolution=0.1, minimum_depth=0.55,
            current_xy=(-0.1, 0.0),
        ))

    def test_return_fallback_requires_clearance_past_controller_goal_tolerance(self):
        import numpy as np
        points = np.asarray([(-0.2, 0.0), (-0.6, 0.0)])
        selected = select_reachable_source_side_return_goal(
            points, [1.0, 10.0], (0.0, 0.0), (1.0, 0.0),
            (-1.0, 0.0), resolution=0.1, minimum_depth=0.55,
            current_xy=(0.2, 0.0),
        )
        self.assertEqual(selected, 1)

    def test_return_fallback_can_advance_around_obstacle_even_if_farther_from_junction(self):
        import numpy as np
        points = np.asarray([(-0.6, 2.0), (-0.8, 0.0)])
        selected = select_reachable_source_side_return_goal(
            points, [1.0, 20.0], (0.0, 0.0), (1.0, 0.0),
            (-1.0, 0.0), resolution=0.05, current_xy=(0.2, 0.0),
        )
        self.assertEqual(selected, 0)

    def test_return_fallback_rejects_far_source_side_cell_for_local_sidestep(self):
        import numpy as np
        points = np.asarray([(-3.45, 0.75), (-0.8, 0.0)])
        selected = select_reachable_source_side_return_goal(
            points, [0.1, 0.5], (0.0, 0.0), (1.0, 0.0),
            (-1.0, 0.0), resolution=0.05, current_xy=(0.2, 0.0),
            maximum_source_distance=2.5,
        )
        self.assertEqual(selected, 1)
        self.assertIsNone(select_reachable_source_side_return_goal(
            points[:1], [0.1], (0.0, 0.0), (1.0, 0.0),
            (-1.0, 0.0), resolution=0.05, current_xy=(0.2, 0.0),
            maximum_source_distance=2.5,
        ))

    def test_route_candidate_uses_source_side_reachable_fallback(self):
        import numpy as np
        from global_frontier_portal_authentication import DoorwayAuthentication
        fixture = GPBranchFrontierMixin()
        fixture.transform_xy = lambda target, source, x, y: (x, y)
        fixture.xy_to_grid_cell = lambda message, x, y: (0, 1)
        fixture.cell_xy = lambda message, row, col: (
            (-0.6, 0.0) if col == 0 else (0.2, 0.0)
        )
        events = []
        fixture.publish_status = lambda event, **fields: events.append((event, fields))
        auth = DoorwayAuthentication(27, 8, (0.0, 0.0), (1.0, 0.0))
        auth.begin((-1.0, 0.0))
        auth.state = "returning"
        auth.failure_reason = "authentication_physical_stall:inside"
        message = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            info=SimpleNamespace(resolution=0.1),
        )
        route = fixture._portal_authentication_route_candidate(
            message, np.asarray([[0, -1, 2]], dtype=int), auth,
        )
        self.assertEqual(list(route[0:4]), [0, 0, -0.6, 0.0])
        self.assertIn(
            "portal_authentication_source_side_return_goal_selected",
            [event for event, _ in events],
        )


class AuthenticationPhysicalDeadlineHandoffTest(unittest.TestCase):
    def test_partial_entry_without_motion_returns_and_releases_exact_lease(self):
        from global_frontier_portal_authentication import DoorwayAuthentication
        auth = DoorwayAuthentication(14, 3, (0, 0), (1, 0))
        auth.begin((-1, 0))
        fixture = GPBranchFrontierMixin()
        fixture.pose_odom = SimpleNamespace(x=.1, y=0, theta=0)
        fixture.active_timeout = 20.0
        fixture.active_portal_authentication = lambda: auth
        events = []
        releases = []
        fixture.publish_status = lambda event, **fields: events.append((event, fields))
        fixture.release_active_frontier = lambda **fields: releases.append(fields)
        fixture._update_portal_authentication_from_map(None, None, None, [], 0, 0)
        self.assertEqual(auth.state, 'entering')
        fixture._update_portal_authentication_from_map(None, None, None, [], 0, 10)
        self.assertEqual(auth.state, 'returning')
        self.assertEqual(auth.failure_reason, 'authentication_physical_stall:entering')
        self.assertEqual(len(releases), 1)
        self.assertTrue(releases[0]['await_controller_release_ack'])
        self.assertFalse(releases[0]['queue_successor'])
        self.assertEqual(events[-1][0], 'portal_authentication_physical_progress_expired')

    def test_stalled_failure_return_remains_active_until_source_return_proven(self):
        from global_frontier_portal_authentication import (
            DoorwayAuthentication, JunctionDoorwayAuthentication,
        )
        first = DoorwayAuthentication(14, 3, (0, 0), (1, 0))
        sibling = DoorwayAuthentication(15, 3, (0, 0), (0, 1))
        task = JunctionDoorwayAuthentication(3, (first, sibling), settle_seconds=0)
        task.start_next((-1, 0), now=0)
        first.state = "returning"
        first.failure_reason = "authentication_physical_stall:inside"
        fixture = GPBranchFrontierMixin()
        fixture.pose_odom = SimpleNamespace(x=.2, y=0, theta=0)
        fixture.active_timeout = 20.0
        fixture.active_portal_authentication = lambda: first
        fixture.junction_portal_authentication = task
        fixture.publish_status = lambda *args, **kwargs: None
        releases = []
        fixture.release_active_frontier = lambda **fields: releases.append(fields)
        fixture._update_portal_authentication_from_map(None, None, None, [], 0, 0)
        fixture._update_portal_authentication_from_map(None, None, None, [], 0, 10)
        self.assertEqual(first.state, "returning")
        self.assertEqual(task.active_branch_id, first.branch_id)
        self.assertFalse(task.all_finished())
        self.assertIsNone(task.finish_active((-1, 0), now=11))
        self.assertEqual(task.active_branch_id, first.branch_id)
        self.assertEqual(len(releases), 1)
        self.assertTrue(any(e["event"] == "doorway_failure_return_retry" for e in first.events))
        # Only source-side return evidence may make this failed probe terminal.
        first.return_gate_cleared = True
        self.assertTrue(first.update_return((-0.5, 0)))
        self.assertEqual(first.state, "failed")
        self.assertEqual(task.finish_active((-1, 0), now=12).branch_id, sibling.branch_id)


class PendingPassabilityTest(unittest.TestCase):
    def test_blocked_pending_is_withdrawn_even_when_node_loses_its_fan(self):
        from global_frontier_portal_authentication import DoorwayAuthentication, JunctionDoorwayAuthentication
        fixture = GPBranchFrontierMixin()
        fixture.current_physical_place_id = 1
        fixture.publish_status = lambda *args, **kwargs: None
        auth = DoorwayAuthentication(8, 7, (0., 0.), (2., 0.))
        task = JunctionDoorwayAuthentication(7, [auth])
        fixture.portal_authentication_by_node = {7: task}
        branch = {"stable": True, "passability": "blocked", "state": "pending"}
        tree = SimpleNamespace(current_node_id=7,
            nodes={7: {"junction": True, "junction_confirmed": False, "branch_ids": [8]}},
            branches={8: branch})
        fixture._sync_portal_authentication(tree, None, None, now=2.)
        self.assertNotIn(8, task.branches)
        self.assertEqual(task.phase, "complete")
        self.assertEqual(branch["authentication_state"], "waiting_passable_section")


if __name__ == "__main__":
    unittest.main()
