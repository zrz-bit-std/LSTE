"""An actual occupancy-grid gate must separate WI viewpoints in the auditor."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as S
import unittest
import numpy as np

path=Path(__file__).resolve().parents[3]/'scripts/tests/targeted_navigation/audit_place_dfs_geometry.py'
import sys
sys.path.insert(0,str(path.parent))
spec=importlib.util.spec_from_file_location('dfs_geometry_audit',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class GeometryAuditTest(unittest.TestCase):
    def test_physical_handoff_rejects_shifted_section_and_early_body_depth(self):
        portal=dict(physical_gate=[0.,0.],physical_destination=[2.,0.],
                    physical_bounds=[[0.,-1.],[0.,1.]],gp_authentication_required_depth=.75)
        self.assertFalse(module.physical_handoff_evidence(portal,[.4,0.])['verified'])
        self.assertTrue(module.physical_handoff_evidence(portal,[.8,0.])['verified'])
        self.assertTrue(module.physical_handoff_evidence(portal,[-.8,0.],reverse=True)['verified'])
        portal['physical_bounds']=[[1.,-1.],[1.,1.]]
        result=module.physical_handoff_evidence(portal,[.8,0.])
        self.assertFalse(result['verified'])
        self.assertIn('section disagree',result['reason'])

    def test_child_scope_seed_on_parent_side_is_rejected(self):
        grid=np.full((12,22),100,dtype=int);grid[3:7,1:21]=0
        message=S(info=S(height=12,width=22,resolution=1.,origin=S(position=S(x=0,y=0))),data=grid.ravel())
        portal=dict(id=7,gate=[8.,5.],destination=[12.,5.],width=4.,bounds=[[8.,3.],[8.,7.]])
        scope=dict(seed_map_xy=[4.,5.],entry_portal_id=7,projected_portals=[portal])
        labels,region,cell=module.component_from_map(message,scope)
        self.assertFalse(module.entry_destination_in_component(scope,labels,region,cell))
        scope['seed_map_xy']=[12.,5.]
        labels,region,cell=module.component_from_map(message,scope)
        self.assertTrue(module.entry_destination_in_component(scope,labels,region,cell))
        scope['entry_portal_id']=99
        self.assertFalse(module.entry_destination_in_component(scope,labels,region,cell))

    def test_completion_uses_recorded_clearance_but_keeps_far_unknown_boundary(self):
        grid=np.zeros((15,15),dtype=int)
        grid[1,1]=100
        grid[2,2]=grid[8,8]=-1
        message=S(info=S(height=15,width=15,resolution=1.0),data=grid.ravel())
        labels=np.ones(grid.shape,dtype=np.uint8)
        raw=module.completion_frontier_cells(message,{},labels,1)
        filtered=module.completion_frontier_cells(
            message,dict(frontier_clearance_m=2.0),labels,1)
        self.assertIn((1,2),raw)
        self.assertNotIn((1,2),filtered)
        self.assertTrue({(7,8),(9,8),(8,7),(8,9)}.issubset(filtered))
        self.assertLess(len(filtered),len(raw))

    def test_invalid_recorded_clearance_is_rejected(self):
        grid=np.zeros((5,5),dtype=int)
        message=S(info=S(height=5,width=5,resolution=1.0),data=grid.ravel())
        for value in (-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):
                module.completion_frontier_cells(
                    message,dict(frontier_clearance_m=value),np.ones(grid.shape),1)

    def test_portal_cuts_separate_rooms_and_unknown_never_becomes_interior(self):
        grid=np.zeros((9,15),dtype=int)
        grid[0,:]=grid[-1,:]=grid[:,0]=grid[:,-1]=100
        grid[1:8,7]=100;grid[3:6,7]=0;grid[2,3]=-1
        message=S(info=S(height=9,width=15,resolution=1.0,
                        origin=S(position=S(x=0,y=0))),data=grid.ravel())
        scope=dict(seed_map_xy=[3.5,4.5],projected_portals=[
            dict(id=1,gate=[7.5,4.5],destination=[10.5,4.5],width=3.0)])
        labels,region,cell=module.component_from_map(message,scope)
        self.assertGreater(region,0)
        self.assertEqual(labels[cell([4.5,4.5])],region)
        self.assertNotEqual(labels[cell([10.5,4.5])],region)
        self.assertEqual(labels[cell([3.5,2.5])],0)
        scope['projected_portals']=[]
        labels,region,cell=module.component_from_map(message,scope)
        self.assertEqual(labels[cell([10.5,4.5])],region)

    def test_invalid_scope_is_never_dispatch_or_completion_evidence(self):
        import tempfile,json
        from pathlib import Path
        from unittest.mock import patch
        from types import SimpleNamespace
        class EmptyBag:
            def __init__(self,*args,**kwargs):pass
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def read_messages(self,**kwargs):return iter(())
        scope=dict(valid=False,map_stamp=5.,place_id=1,projected_portals=[dict(id=999)])
        rows=[dict(frontier_event='portal_hypothesis_crossed',place_scope=scope),
              dict(frontier_event='work_item_dispatched',place_scope=scope),
              dict(frontier_event='place_dfs_completion_ready',place_scope=scope)]
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'metrics.log'
            path.write_text(''.join('event=global_frontier_event data='+json.dumps(row)+'\n' for row in rows))
            with patch.dict(sys.modules,rosbag=SimpleNamespace(Bag=EmptyBag)):
                result=module.audit(path,'unused.bag')
        self.assertFalse(result['passed'])
        self.assertEqual(result['checked'],[])
        self.assertEqual([e['reason'] for e in result['errors']],
            ['missing recorded map or viewpoint','missing valid completion map snapshot'])

    def test_async_scope_uses_only_preceding_tree_evidence(self):
        import tempfile, json
        from pathlib import Path
        rows=[dict(frontier_event='scope_before_tree'),
              dict(tree=dict(branches=[dict(id=4,authentication_state='completed')])),
              dict(frontier_event='scope_after_tree'),dict(tree={}),
              dict(frontier_event='scope_after_explicit_clear')]
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'metrics.log'
            path.write_text(''.join('event=global_frontier_event data='+json.dumps(row)+'\n' for row in rows))
            actual=list(module.events(path))
        self.assertNotIn('tree',actual[0][1])
        self.assertEqual(actual[2][1]['tree'],rows[1]['tree'])
        self.assertEqual(actual[4][1]['tree'],{})
        self.assertEqual([row[0] for row in actual],[1,2,3,4,5])

    def test_unbound_source_cut_needs_full_physical_certification_not_child_identity(self):
        import copy
        record=dict(id=8,source_place_id=2,destination_place_id=None,state='certified',
                    gp_authentication_state='completed',gp_authentication_branch_id=4,
                    opening_width=2.,physical_bounds=[[0.,-1.],[0.,1.]])
        evidence=dict(state='completed',source_side_proven=True,return_gate_cleared=True,
                      events=[dict(event=e) for e in ('doorway_entry_started','vehicle_fully_inside',
                        'inside_frontier_observed','room_facing_turn_complete','returned_to_junction_node')])
        event=dict(portal_hypotheses=dict(records=[record]),tree=dict(branches=[dict(id=4,
                   canonical_portal_id=8,authentication_state='completed',authentication_evidence=evidence)]))
        scope=dict(place_id=2,projected_portals=[dict(id=8,source_place_id=2,destination_place_id=None)])
        self.assertTrue(module.bound_portal_identities_valid(event,scope))
        for kind in ('no_evidence','no_return','wrong_source','already_bound'):
            ev=copy.deepcopy(event)
            if kind=='no_evidence':ev['tree']['branches'][0]['authentication_evidence']={}
            elif kind=='no_return':ev['tree']['branches'][0]['authentication_evidence']['events'].pop()
            elif kind=='wrong_source':ev['portal_hypotheses']['records'][0]['source_place_id']=1
            else:ev['portal_hypotheses']['records'][0]['destination_place_id']=3
            self.assertFalse(module.bound_portal_identities_valid(ev,scope),kind)

    def test_bound_portal_clip_requires_matching_physical_identity(self):
        portal=dict(id=7,source_place_id=2,destination_place_id=3)
        scope=dict(projected_portals=[portal])
        record=dict(portal,gp_authentication_state='completed',crossing_count=1)
        event=dict(portal_hypotheses=dict(records=[record]))
        self.assertTrue(module.bound_portal_identities_valid(event,scope))
        for change in (dict(crossing_count=0),dict(destination_place_id=4),
                       dict(gp_authentication_state='pending')):
            event['portal_hypotheses']['records']=[dict(record,**change)]
            self.assertFalse(module.bound_portal_identities_valid(event,scope))
        self.assertFalse(module.bound_portal_identities_valid({},scope))

    def test_bound_place_side_reconstructs_bypass_without_admitting_other_place(self):
        grid=np.full((12,22),100,dtype=int);grid[3:7,1:21]=0
        message=S(info=S(height=12,width=22,resolution=1.,origin=S(position=S(x=0,y=0))),data=grid.ravel())
        portal=dict(id=1,gate=[8.,6.5],destination=[12.,6.5],width=3.,
                    source_place_id=2,destination_place_id=3)
        scope=dict(place_id=2,seed_map_xy=[4.,5.],projected_portals=[portal])
        labels,region,cell=module.component_from_map(message,scope)
        self.assertEqual(labels[cell([4.,5.])],region)
        self.assertNotEqual(labels[cell([12.,6.5])],region)
        self.assertEqual(module.portal_side_separation(message,scope,labels,region,cell),([],1))
        scope.update(place_id=3,seed_map_xy=[12.,5.],entry_portal_id=1)
        labels,region,cell=module.component_from_map(message,scope)
        self.assertTrue(module.entry_destination_in_component(scope,labels,region,cell))
        self.assertNotEqual(labels[cell([4.,5.])],region)
        # Unbound or unrelated Place identities cannot authorize a side clip.
        for source,destination,place in ((2,None,2),(2,2,2),(2,3,4)):
            portal.update(source_place_id=source,destination_place_id=destination)
            scope.update(place_id=place)
            labels,region,cell=module.component_from_map(message,scope)
            self.assertEqual(module.portal_side_separation(message,scope,labels,region,cell)[0],[1])

    def test_side_clip_excludes_disconnected_same_side_fragment(self):
        grid=np.full((20,30),100,dtype=int);grid[2:18,1:28]=0
        grid[12,9:28]=100
        message=S(info=S(height=20,width=30,resolution=1.,origin=S(position=S(x=0,y=0))),data=grid.ravel())
        portal=dict(id=7,gate=[10.,10.],destination=[14.,10.],width=4.,
                    bounds=[[10.,8.],[10.,12.]],source_place_id=1,destination_place_id=2)
        scope=dict(place_id=2,seed_map_xy=[14.,10.],entry_portal_id=7,projected_portals=[portal])
        labels,region,cell=module.component_from_map(message,scope)
        self.assertGreater(region,0)
        self.assertEqual(labels[cell([14.,10.])],region)
        self.assertNotEqual(labels[cell([14.,15.])],region)

    def test_off_center_cut_is_reported_as_leaking_even_for_an_inside_goal(self):
        grid=np.full((12,22),100,dtype=int);grid[3:7,1:21]=0
        message=S(info=S(height=12,width=22,resolution=1.,origin=S(position=S(x=0,y=0))),data=grid.ravel())
        portal=dict(id=1,gate=[8.,6.5],destination=[12.,6.5],width=3.)
        scope=dict(seed_map_xy=[4.,5.],projected_portals=[portal])
        labels,region,cell=module.component_from_map(message,scope)
        self.assertEqual(module.portal_side_separation(message,scope,labels,region,cell)[0],[1])
        portal['bounds']=[[8.,3.],[8.,7.]]
        labels,region,cell=module.component_from_map(message,scope)
        self.assertEqual(module.portal_side_separation(message,scope,labels,region,cell),([],1))

if __name__=='__main__':unittest.main()
