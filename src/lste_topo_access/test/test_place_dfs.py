"""Behavioral checks for nested Place completion and certified SLAM boundaries."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from global_frontier_place_dfs import PlaceDfsTraversal
from global_frontier_place_partition import partition_place


class PlaceDfsTest(unittest.TestCase):
    def test_normal_return_has_physical_budget_independent_of_projection_refresh(self):
        dfs=PlaceDfsTraversal();dfs.enter(1,2,10)
        self.assertTrue(dfs.ready_to_return(2,projection_complete=True))
        self.assertIsNone(dfs.update_projection_recovery(2,True,0.,(4.,0.)))
        for second in range(1,60):
            self.assertTrue(dfs.ready_to_return(2,projection_complete=True))
            self.assertIsNone(dfs.update_projection_recovery(2,True,second,(4.,0.)))
        self.assertEqual(dfs.update_projection_recovery(2,True,60.,(4.,0.)),'return_blocked')
        self.assertFalse(dfs.return_to_parent(2,1,10))
        self.assertEqual(dfs.stack,[1,2])
        self.assertEqual(dfs.nodes[2]['state'],'recovery_blocked')
        self.assertFalse(dfs.ready_to_return(2,projection_complete=True))

    def test_normal_return_real_translation_refreshes_budget(self):
        dfs=PlaceDfsTraversal();dfs.enter(1,2,10)
        dfs.ready_to_return(2,projection_complete=True)
        dfs.update_projection_recovery(2,True,0.,(4.,0.))
        self.assertIsNone(dfs.update_projection_recovery(2,False,59.,(3.8,0.)))
        self.assertIsNone(dfs.update_projection_recovery(2,True,118.,(3.8,0.)))
        self.assertEqual(dfs.update_projection_recovery(2,True,119.,(3.8,0.)),'return_blocked')

    def test_child_must_finish_before_parent_and_returns_only_through_entry(self):
        dfs=PlaceDfsTraversal()
        self.assertTrue(dfs.enter(1,2,10));self.assertTrue(dfs.enter(2,3,11))
        self.assertFalse(dfs.ready_to_return(2,projection_complete=True))
        self.assertFalse(dfs.ready_to_return(3,unresolved_work=[7],projection_complete=True))
        self.assertFalse(dfs.ready_to_return(3,frontier_count=1,projection_complete=True))
        self.assertFalse(dfs.ready_to_return(3,pending_portals=[12],projection_complete=True))
        self.assertFalse(dfs.ready_to_return(3))
        self.assertTrue(dfs.ready_to_return(3,projection_complete=True))
        self.assertFalse(dfs.return_to_parent(3,1,11))
        self.assertFalse(dfs.return_to_parent(3,2,10))
        self.assertTrue(dfs.return_to_parent(3,2,11))
        self.assertTrue(dfs.ready_to_return(2,projection_complete=True))
        self.assertTrue(dfs.return_to_parent(2,1,10))
        self.assertFalse(dfs.enter(1,2,10));self.assertEqual(dfs.stack,[1])

    def test_invalid_projection_budget_survives_snapshots_and_preserves_failed_child(self):
        dfs=PlaceDfsTraversal();dfs.enter(1,2,10)
        self.assertIsNone(dfs.update_projection_recovery(2,False,0.,(4.,0.)))
        self.assertIsNone(dfs.update_projection_recovery(2,False,29.,(4.,0.)))
        self.assertEqual(dfs.update_projection_recovery(2,False,30.,(4.,0.)),'started')
        self.assertFalse(dfs.ready_to_return(2,projection_complete=True))
        self.assertFalse(dfs.return_to_parent(2,1,11))
        self.assertTrue(dfs.return_to_parent(2,1,10))
        self.assertEqual(dfs.nodes[2]['state'],'failed')
        self.assertEqual(dfs.stack,[1])

    def test_valid_projection_resets_wait_but_return_progress_does_not_reset_identity(self):
        dfs=PlaceDfsTraversal();dfs.enter(1,2,10)
        dfs.update_projection_recovery(2,False,0.,(4.,0.))
        dfs.update_projection_recovery(2,True,29.,(4.,0.))
        self.assertIsNone(dfs.update_projection_recovery(2,False,31.,(4.,0.)))
        self.assertEqual(dfs.update_projection_recovery(2,False,61.,(4.,0.)),'started')
        self.assertIsNone(dfs.update_projection_recovery(2,True,110.,(3.,0.)))
        self.assertEqual(dfs.nodes[2]['state'],'recovery_returning')
        self.assertIsNone(dfs.update_projection_recovery(2,False,169.,(3.,0.)))
        self.assertEqual(dfs.update_projection_recovery(2,False,170.,(3.,0.)),'blocked')
        self.assertFalse(dfs.return_to_parent(2,1,10))

    def test_back_edge_does_not_create_cycle_or_reparent_existing_child(self):
        dfs=PlaceDfsTraversal();dfs.enter(1,2,10)
        self.assertFalse(dfs.enter(2,1,11));self.assertEqual(dfs.stack,[1,2])

    def test_gate_cut_bounds_bent_room_by_connectivity_not_angular_sector(self):
        free=np.zeros((20,30),dtype=bool)
        free[8:12,1:28]=True;free[1:12,18:28]=True
        gate=dict(id=1,gate=(10.,10.),destination=(12.,10.),width=4.)
        mask,_,label,cuts=partition_place(free,1.,(0,0),(12.,10.),[gate],1)
        self.assertGreater(label,0);self.assertEqual(cuts,[1])
        self.assertFalse(mask[10,3]);self.assertTrue(mask[2,20])
        self.assertTrue(free[10,10])

    def test_unknown_or_wall_is_not_inferred_to_be_room_interior(self):
        free=np.zeros((8,8),dtype=bool);free[2:6,2:6]=True
        mask,_,_,_=partition_place(free,1.,(0,0),(3.,3.),[],1)
        self.assertEqual(np.count_nonzero(mask),16)
        self.assertFalse(mask[0,0])

class PortalPartitionEvidenceTest(unittest.TestCase):
    def test_known_room_bypass_and_visited_corridor_are_not_new_places(self):
        from global_frontier_place_partition import portal_partition_evidence
        gate=dict(id=0,gate=(10.,10.),destination=(8.,10.),width=4.)
        wide=np.ones((20,30),dtype=bool)
        result=portal_partition_evidence(wide,1.,(0,0),(12.,10.),(8.,10.),gate)
        self.assertTrue(result['reject']);self.assertEqual(result['reason'],'known_free_bypass_within_current_place')
        corridor=np.zeros_like(wide);corridor[8:12,1:28]=True
        result=portal_partition_evidence(corridor,1.,(0,0),(12.,10.),(8.,10.),gate,entry_xy=(5.,10.))
        self.assertTrue(result['reject']);self.assertEqual(result['reason'],'visited_entry_side_of_current_place')
        result=portal_partition_evidence(corridor,1.,(0,0),(12.,10.),(8.,10.),gate,entry_xy=(25.,10.))
        self.assertFalse(result['reject']);self.assertEqual(result['reason'],'distinct_destination_component')

class PlaceDfsPlannerTest(unittest.TestCase):
    def test_certified_child_preempts_parent_tsp_and_returns_via_exact_parent(self):
        from global_frontier_graph_route_planner import GraphRoutePlanner
        places=[dict(id=p,state='open',endpoint_observations=1) for p in (1,2,3,4)]
        portals=[dict(id=10,source_place_id=1,destination_place_id=2,state='crossed'),
                 dict(id=11,source_place_id=2,destination_place_id=3,state='crossed'),
                 dict(id=12,source_place_id=4,destination_place_id=3,state='crossed')]
        wi=dict(id=50,place_id=2,state='unresolved',active_attempt_id=None,anchor_xy=(4,0))
        planner=GraphRoutePlanner()
        plan=planner.plan(2,places=places,portals=portals,work_items=[wi],
            visible_work_item_ids=[50],room_work_item_only=True,
            dfs_parent_portal_id=10,dfs_ancestor_place_ids=[1])
        self.assertEqual(plan.action,'cross_portal');self.assertEqual(plan.first_portal_id,11)
        plan=planner.plan(3,places=places,portals=portals,room_work_item_only=True,
            dfs_parent_portal_id=11,dfs_ancestor_place_ids=[1,2])
        self.assertEqual(plan.first_portal_id,11)
        plan=planner.plan(2,places=places,portals=portals,work_items=[wi],
            visible_work_item_ids=[50],room_work_item_only=True,
            dfs_parent_portal_id=10,dfs_ancestor_place_ids=[1],dfs_completed_place_ids=[3])
        self.assertEqual(plan.obligation_id,50);self.assertEqual(plan.work_item_tour,(50,))

    def test_invalid_projection_retreat_uses_original_entry_before_work_or_children(self):
        from global_frontier_graph_route_planner import GraphRoutePlanner
        planner=GraphRoutePlanner()
        places=[dict(id=p,state='open',endpoint_observations=1) for p in (1,2,3)]
        portals=[dict(id=10,source_place_id=1,destination_place_id=2,state='crossed'),
                 dict(id=11,source_place_id=2,destination_place_id=3,state='crossed')]
        work=[dict(id=50,place_id=2,state='unresolved',active_attempt_id=7)]
        plan=planner.plan(2,places=places,portals=portals,work_items=work,
            dfs_parent_portal_id=10,dfs_recovery_return=True)
        self.assertEqual(plan.first_portal_id,10)
        self.assertEqual(plan.target_place_id,1)
        self.assertEqual(plan.reason,'dfs_invalid_projection_recovery_return')
        plan=planner.plan(2,places=places,portals=portals[1:],
            dfs_parent_portal_id=10,dfs_recovery_return=True)
        self.assertEqual(plan.action,'hold')
        plan=planner.plan(1,places=places,portals=portals,work_items=work,
                          dfs_execution_failed=True)
        self.assertEqual(plan.action,'hold')
        self.assertEqual(plan.reason,'dfs_execution_failed_return_obligation_preserved')

    def test_gp_mode_does_not_let_legacy_probe_steal_normal_work(self):
        from test_graph_route_adapter import AdapterFixture
        fixture=AdapterFixture(
            work_items=[dict(id=50,place_id=1,state='unresolved',active_attempt_id=None,anchor_xy=(4,0))],
            probes=[dict(id=91,source_place_id=1,state='pending',work_item_id=50)],
            portals=[dict(id=92,source_place_id=1,destination_place_id=None,state='certified')])
        fixture.gp_portal_assistance_enabled=True
        result=fixture._plan_durable_graph_route(map_epoch=1,visible_work_item_ids=[50],
                                                visible_probe_ids=[91])
        self.assertEqual(result.action,'observe_local_work')
        self.assertEqual(result.obligation_id,50)

    def test_fresh_frontier_or_incomplete_projection_prevents_egress(self):
        from global_frontier_graph_route_planner import GraphRoutePlanner
        kwargs=dict(places=[dict(id=p,state='open',endpoint_observations=1) for p in (1,2)],
                    portals=[dict(id=10,source_place_id=1,destination_place_id=2,state='crossed')],
                    room_work_item_only=True,dfs_parent_portal_id=10,dfs_ancestor_place_ids=[1])
        for extra in (dict(place_frontier_count=3),dict(place_projection_complete=False)):
            result=GraphRoutePlanner().plan(2,**kwargs,**extra)
            self.assertEqual(result.action,'hold')

    def test_advancing_frontier_is_new_work_despite_overlapping_unknown_support(self):
        from global_frontier_work_items import PlaceWorkItemLedger
        from global_frontier_work_item_components import ObservationSupport
        ledger=PlaceWorkItemLedger(merge_radius=.5)
        support=ObservationSupport(frozenset({(1,1)}),frozenset({(10,10),(11,10)}),
            anchor_xy=(1,0),normal_xy=(1,0),boundary_xy=(.5,0))
        _,details=ledger.reconcile(1,[support],0);first=next(iter(details))
        ledger.resolve(first,1,reason='viewed')
        newer=ObservationSupport(frozenset({(1,4)}),support.support_cells,
            anchor_xy=(4,0),normal_xy=(1,0),boundary_xy=(3.5,0))
        _,details=ledger.reconcile(1,[newer],2);second=next(iter(details))
        self.assertNotEqual(first,second);self.assertEqual(details[second]['state'],'unresolved')
        moved=ledger.transfer_observation_work_to_place(2,lambda xy:xy[0]>2,3)
        self.assertEqual([r['work_item_id'] for r in moved],[second])
        self.assertEqual(ledger.unresolved_ids(1),[]);self.assertEqual(ledger.unresolved_ids(2),[second])



class PlaceDfsFrozenWorkTest(unittest.TestCase):
    def test_child_keeps_own_resume_target_when_parent_wi_is_already_frozen(self):
        from types import SimpleNamespace
        from global_frontier_work_items import GlobalFrontierWorkItemLifecycleMixin
        dfs=PlaceDfsTraversal();dfs.enter(1,2,10)
        dfs.nodes[1]['suspended_work_item_id']=7
        item=dict(id=9,place_id=2,state='unresolved',active_attempt_id=None,kind='observation')
        fixture=GlobalFrontierWorkItemLifecycleMixin()
        fixture.place_dfs=dfs;fixture.current_physical_place_id=2
        fixture.active_work_item_id=None;fixture.suspended_work_item_id=7
        fixture.place_work_items=SimpleNamespace(snapshot=lambda:[item])
        fixture.publish_status=lambda *args,**kwargs:None
        self.assertEqual(fixture.suspend_active_work_item(1.0)['id'],9)
        self.assertEqual(dfs.nodes[1]['suspended_work_item_id'],7)
        self.assertEqual(dfs.nodes[2]['suspended_work_item_id'],9)
        self.assertIsNone(fixture.suspend_active_work_item(2.0))


class ObservationPortalAliasTest(unittest.TestCase):
    def test_existing_slam_cut_reuses_portal_but_same_side_candidate_is_new(self):
        from types import SimpleNamespace as S
        from global_frontier_place_portal_validation import duplicate_observation_portal
        free=np.zeros((10,20),dtype=bool);free[3:7,:]=True
        message=S(header=S(frame_id='map',stamp=S(to_sec=lambda:1.)),info=S(resolution=1.,origin=S(position=S(x=0.,y=0.))))
        context=S(known_free=free,unknown=np.zeros_like(free),components=S(epoch=1))
        portal=dict(id=10,source_place_id=1,gp_authentication_state='completed')
        projected=dict(id=10,gate=(7.5,5.),destination=(10.5,5.),width=4.)
        node=S(_place_partition_tf_key=('map',1.),_place_partition_projector=lambda x,y:(x,y),current_physical_place_id=1,portal_hypothesis_ledger=S(get=lambda i:portal),
               transform_xy=lambda t,s,x,y:(x,y),_update_place_partition=lambda *a:None,
               current_place_scope_projected_portals=[projected])
        tree=S(nodes={2:dict(physical_xy=(4.,5.))})
        branch=dict(node_id=2,physical_gate=(8.5,5.),heading=0.,map_geometry=dict(width=4.))
        self.assertEqual(duplicate_observation_portal(node,tree,branch,message,context)['id'],10)
        branch['physical_gate']=(4.5,5.)
        self.assertIsNone(duplicate_observation_portal(node,tree,branch,message,context))
        branch['physical_gate']=(8.5,5.);portal['gp_authentication_state']='pending'
        self.assertIsNone(duplicate_observation_portal(node,tree,branch,message,context))


class PortalWidthEvidenceTest(unittest.TestCase):
    def test_unmapped_gp_sector_keeps_finite_width_for_slam_confirmation(self):
        import math
        from global_frontier_place_partition import branch_opening_width
        branch=dict(map_geometry=dict(width=None),reach=3.2,angular_width=0.942477796)
        self.assertAlmostEqual(branch_opening_width(branch),2*3.2*math.sin(0.942477796/2))
        branch['map_geometry']['width']=2.2
        self.assertEqual(branch_opening_width(branch),2.2)
        self.assertIsNone(branch_opening_width({}))
        self.assertIsNone(branch_opening_width(dict(reach=3.,angular_width=float('nan'))))

class OffCenterPortalCutTest(unittest.TestCase):
    def test_measured_sides_close_sliver_left_by_an_off_center_gp_axis(self):
        free=np.zeros((12,22),dtype=bool);free[3:7,1:21]=True
        portal=dict(id=1,gate=(8.,6.5),destination=(12.,6.5),width=3.)
        leaking,_,_,_=partition_place(free,1.,(0,0),(4.,5.),[portal],1)
        self.assertTrue(leaking[5,16])
        from global_frontier_place_partition import leaking_portal_sides
        self.assertEqual(leaking_portal_sides(leaking,free,1.,(0,0),[portal]),[1])
        portal['bounds']=[(8.,3.),(8.,7.)]
        bounded,_,_,_=partition_place(free,1.,(0,0),(4.,5.),[portal],2)
        self.assertFalse(bounded[5,16]);self.assertTrue(bounded[5,4])
        self.assertEqual(leaking_portal_sides(bounded,free,1.,(0,0),[portal]),[])
        self.assertTrue(free[5,8])


class InsideFrontierLocalityTest(unittest.TestCase):
    def test_remote_frontier_in_forward_strip_does_not_authenticate_empty_inside(self):
        from types import SimpleNamespace as S
        from global_frontier_grid import frontier_mask
        from global_frontier_place_portal_validation import authentication_inside_frontiers
        free=np.zeros((18,24),dtype=bool);free[3:7,1:20]=True;free[10:14,10:15]=True
        unknown=np.zeros_like(free);unknown[10:14,15]=True
        raw=frontier_mask(free,unknown)
        self.assertGreater(np.count_nonzero(raw[:,9:16]),0)
        message=S(header=S(frame_id='map',stamp=S(to_sec=lambda:3.)),
                  info=S(resolution=1.,origin=S(position=S(x=0.,y=0.))))
        context=S(known_free=free,unknown=unknown,components=S(epoch=1))
        branch=dict(map_geometry=dict(width=4.))
        node=S(_place_partition_tf_key=('map',3.),_place_partition_projector=lambda x,y:(x,y),current_physical_place_id=1,_update_place_partition=lambda *args:None,transform_xy=lambda t,s,x,y:(x,y),current_place_scope_projected_portals=[])
        auth=S(branch_id=5,source_xy=(4.,5.),inside_xy=(12.,5.),gate_xy=(8.,5.),
               destination_xy=(12.,5.),required_depth=.75)
        local,evidence=authentication_inside_frontiers(node,S(branches={5:branch}),auth,message,context,raw)
        self.assertEqual(np.count_nonzero(local),0)
        self.assertEqual(evidence['map_stamp'],3.)
        free[3:7,16:20]=False;unknown[3:7,16]=True
        local,_=authentication_inside_frontiers(node,S(branches={5:branch}),auth,message,context,frontier_mask(free,unknown))
        self.assertGreater(np.count_nonzero(local),0)
        self.assertEqual(np.count_nonzero(local[10:]),0)
        # A long corridor belongs to the same Place even beyond 8m from its gate.
        free[3:7,1:23]=True;unknown[3:7,16]=False;unknown[3:7,23]=True
        local,evidence=authentication_inside_frontiers(node,S(branches={5:branch}),auth,message,context,frontier_mask(free,unknown))
        self.assertGreater(np.count_nonzero(local[:,22]),0)
        self.assertEqual(evidence['frontier_selection'],'entire_entered_component')


class CoherentMeasuredPortalTest(unittest.TestCase):
    def test_gate_is_frozen_from_same_section_as_width_and_sides(self):
        from global_frontier_place_partition import branch_gate_xy
        branch=dict(physical_gate=(5.,6.),map_geometry=dict(
            physical_position=(8.,6.),physical_bounds=[(8.,3.),(8.,7.)]))
        self.assertEqual(branch_gate_xy(branch),(8.,5.))
        branch['map_geometry']['physical_bounds']=[None,(8.,7.)]
        self.assertEqual(branch_gate_xy(branch),(8.,6.))
        branch['map_geometry'].pop('physical_position')
        self.assertEqual(branch_gate_xy(branch),(5.,6.))

    def test_partition_uses_measured_section_even_when_legacy_axis_is_older(self):
        free=np.zeros((12,22),dtype=bool);free[3:7,1:21]=True
        portal=dict(id=1,gate=(5.,5.),destination=(12.,5.),width=4.,bounds=[(8.,3.),(8.,7.)])
        mask,_,_,_=partition_place(free,1.,(0,0),(3.,5.),[portal],1)
        self.assertTrue(mask[5,6]);self.assertFalse(mask[5,12])



class SlamBoundaryRefinementTest(unittest.TestCase):
    def test_map_wall_correction_closes_only_measured_opening(self):
        from global_frontier_place_partition import refine_portal_cut, leaking_portal_sides
        free=np.zeros((20,30),dtype=bool);free[4:16,1:29]=True
        portal=dict(id=1,gate=(12.,10.),destination=(16.,10.),width=10.,
                    bounds=[(12.,4.),(12.,13.)])
        mask,_,_,_=partition_place(free,1.,(0,0),(20.,10.),[portal],1)
        self.assertEqual(leaking_portal_sides(mask,free,1.,(0,0),[portal]),[1])
        refined=refine_portal_cut(portal,free,1.,(0,0))
        mask,_,label,_=partition_place(free,1.,(0,0),(20.,10.),[refined],1)
        self.assertGreater(label,0);self.assertFalse(mask[10,5]);self.assertTrue(mask[10,20])
        self.assertEqual(leaking_portal_sides(mask,free,1.,(0,0),[refined]),[])
        self.assertEqual(portal['bounds'],[(12.,4.),(12.,13.)])

    def test_open_space_without_near_boundary_is_not_extended_to_invent_place(self):
        from global_frontier_place_partition import refine_portal_cut
        free=np.ones((30,30),dtype=bool)
        portal=dict(id=1,gate=(15.,15.),destination=(20.,15.),width=4.,
                    bounds=[(15.,13.),(15.,17.)])
        refined=refine_portal_cut(portal,free,1.,(0,0))
        self.assertEqual(refined['boundary_extension_m'],[0.,0.])
        mask,_,_,_=partition_place(free,1.,(0,0),(20.,15.),[refined],1)
        self.assertTrue(mask[15,5])

if __name__=='__main__':unittest.main()
