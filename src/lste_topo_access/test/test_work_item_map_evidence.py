"""Map evidence can settle old WIs but may never erase a remaining local arc."""
import unittest
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from global_frontier_work_item_map_evidence import observed_boundary_evidence


class WorkItemMapEvidenceTest(unittest.TestCase):
    def test_known_old_arc_settles_while_farther_frontier_remains_new_work(self):
        free=np.ones((30,30),dtype=bool);frontier=np.zeros_like(free);mask=free.copy()
        frontier[15,25]=True
        proof=observed_boundary_evidence(free,frontier,mask,.1,(0.,0.),[(1.5,1.5),(1.6,1.5)],.5)
        self.assertIsNotNone(proof)
        frontier[15,18]=True
        self.assertIsNone(observed_boundary_evidence(free,frontier,mask,.1,(0.,0.),[(1.5,1.5)],.5))

    def test_unknown_or_outside_place_witness_is_not_completion(self):
        free=np.ones((30,30),dtype=bool);frontier=np.zeros_like(free);mask=free.copy()
        free[15,15]=False
        self.assertIsNone(observed_boundary_evidence(free,frontier,mask,.1,(0.,0.),[(1.5,1.5)],.5))
        free[15,15]=True;mask[15,15]=False
        self.assertIsNone(observed_boundary_evidence(free,frontier,mask,.1,(0.,0.),[(1.5,1.5)],.5))
        self.assertIsNone(observed_boundary_evidence(free,frontier,mask,.1,(0.,0.),[(.1,.1)],.5))

    def test_occupied_old_boundary_needs_owned_free_anchor_contact_and_no_frontier(self):
        free=np.ones((40,40),dtype=bool);mask=free.copy();frontier=np.zeros_like(free)
        occupied=np.zeros_like(free);occupied[15,16]=True;free[15,16]=False;mask[15,16]=False
        points=[(1.55,1.55),(1.65,1.55)]
        check=lambda:observed_boundary_evidence(free,frontier,mask,.1,(0.,0.),points,.5,occupied=occupied)
        proof=check();self.assertIsNotNone(proof)
        self.assertEqual(proof['occupied_boundary_witnesses'][0]['cell'],[15,16])
        frontier[15,18]=True;self.assertIsNone(check());frontier[15,18]=False
        occupied[15,16]=False;self.assertIsNone(check());occupied[15,16]=True
        free[15,15]=False;self.assertIsNone(check());free[15,15]=True
        for r,c in ((15,15),(14,16),(16,16),(15,17)):mask[r,c]=False
        self.assertIsNone(check())

    def test_runtime_evidence_never_steals_active_or_portal_owned_work(self):
        from types import SimpleNamespace as S
        from global_frontier_work_item_map_evidence import settle_observed_place_work
        grid=np.ones((30,30),dtype=bool);records=[]
        for i in range(1,5):
            records.append(dict(id=i,place_id=2 if i<4 else 1,state='unresolved',
                active_attempt_id=8 if i==2 else None,kind='portal_entry' if i==3 else 'frontier_observation',
                anchor_xy=(1.5,1.5),place_boundary_xy=(1.6,1.5)))
        resolved=[];events=[]
        ledger=S(merge_radius=.3,snapshot=lambda:records,
                 resolve=lambda i,t,r:resolved.append(i) or records[i-1])
        node=S(place_work_items=ledger,transform_xy=lambda *a:(_ for _ in ()).throw(AssertionError('latest TF forbidden')),
               _place_partition_tf_key=('map',5.),_place_partition_projector=lambda x,y:(x,y),
               publish_status=lambda *a,**k:events.append((a,k)))
        message=S(header=S(frame_id='map',stamp=S(to_sec=lambda:5.)),
                  info=S(resolution=.1,origin=S(position=S(x=0.,y=0.))))
        result=settle_observed_place_work(node,message,grid,np.zeros_like(grid),grid,2,10.)
        self.assertEqual(result,[1]);self.assertEqual(resolved,[1])
        self.assertEqual(events[0][0][0],'work_item_map_observed')
        self.assertEqual(events[0][1]['map_stamp'],5.)
        node._place_partition_tf_key=('map',4.)
        self.assertEqual(settle_observed_place_work(node,message,grid,np.zeros_like(grid),grid,2,10.),[])
        node._place_partition_tf_key=('map',5.)
        raw=np.zeros(grid.shape,dtype=int);raw[15,16]=100
        message.data=raw.ravel();grid[15,16]=False;resolved.clear();events.clear()
        result=settle_observed_place_work(node,message,grid,np.zeros_like(grid),grid,2,11.)
        self.assertEqual(result,[1]);self.assertEqual(resolved,[1])
        self.assertEqual(events[0][1]['evidence']['occupied_boundary_witnesses'][0]['cell'],[15,16])




class FrontierCompletionAccountingTest(unittest.TestCase):
    def test_resolved_descendant_is_accounted_but_new_or_unassigned_boundary_blocks(self):
        from global_frontier_work_item_map_evidence import frontier_work_evidence
        grid=np.zeros((5,5),dtype=bool);grid[2,2]=True
        mask=np.ones_like(grid)
        mapping={(2,2):7};details={7:dict(state='resolved')}
        proof=frontier_work_evidence(grid,mask,mapping,details,10.)
        self.assertEqual(proof['raw_count'],1);self.assertEqual(proof['pending_count'],0)
        self.assertEqual(proof['resolved_frontier_cells'],[[2,2,7]])
        grid[2,3]=True
        self.assertEqual(frontier_work_evidence(grid,mask,mapping,details,11.)['pending_count'],1)
        mapping[2,3]=8;details[8]=dict(state='unresolved')
        self.assertEqual(frontier_work_evidence(grid,mask,mapping,details,12.)['pending_count'],1)

    def test_advancing_frontier_uses_new_wi_and_cannot_hide_behind_completed_work(self):
        from global_frontier_work_item_map_evidence import frontier_work_evidence
        from global_frontier_work_items import PlaceWorkItemLedger
        from global_frontier_work_item_components import ObservationSupport
        ledger=PlaceWorkItemLedger(merge_radius=.5)
        old=ObservationSupport(frozenset([(2,2)]),frozenset([(20,20)]),anchor_xy=(1.,1.))
        mapping,details=ledger.reconcile(2,[old],1.);old_id=mapping[2,2]
        ledger.resolve(old_id,2.,'actual_observation_completed')
        moved=ObservationSupport(frozenset([(2,3)]),frozenset([(20,20)]),anchor_xy=(4.,1.))
        mapping,details=ledger.reconcile(2,[old,moved],3.)
        self.assertNotEqual(mapping[2,3],old_id)
        grid=np.zeros((5,5),dtype=bool);grid[2,2:4]=True
        proof=frontier_work_evidence(grid,np.ones_like(grid),mapping,details,3.)
        self.assertEqual(proof['pending_count'],1)


class EmptyPlaceScopeObservationTest(unittest.TestCase):
    def setup_case(self):
        from types import SimpleNamespace as S
        from global_frontier_work_item_map_evidence import observe_empty_place_scope
        region = dict(id=1,state='open',endpoint_observations=0)
        calls=[];events=[]
        def observe(*args,**kwargs):
            calls.append((args,kwargs));region['endpoint_observations']+=1;return region
        node=S(current_place_scope_valid=True,current_place_scope_id=1,
               pose_odom=S(x=2.5,y=2.5),region_memory=S(endpoint_observed=observe),
               transform_xy=lambda *a:(_ for _ in ()).throw(AssertionError('latest TF forbidden')),
               _place_partition_tf_key=('map',7.),_place_partition_projector=lambda x,y:(x,y),
               publish_status=lambda *args,**kwargs:events.append((args,kwargs)))
        msg=S(header=S(frame_id='map',stamp=S(to_sec=lambda:7.)),
              info=S(resolution=1.,origin=S(position=S(x=0.,y=0.))))
        free=np.ones((5,5),dtype=bool);unknown=np.zeros_like(free);mask=free.copy()
        return observe_empty_place_scope,node,msg,free,unknown,mask,region,calls,events

    def test_zero_frontier_observes_current_fragment_without_closing_or_resolving_wi(self):
        f,node,msg,free,unknown,mask,region,calls,events=self.setup_case()
        node.place_work_items=[dict(id=1,place_id=1,state='unresolved',anchor_xy=(20,2))]
        self.assertTrue(f(node,msg,free,unknown,mask,1,8.))
        self.assertEqual(region['endpoint_observations'],1)
        self.assertEqual(region['state'],'open')
        self.assertEqual(node.place_work_items[0]['state'],'unresolved')
        self.assertEqual(events[0][1]['map_stamp'],7.)

    def test_missing_scope_outside_pose_frontier_or_active_auth_never_counts_observed(self):
        from types import SimpleNamespace as S
        for condition in ('invalid','outside','frontier','active_auth','wrong_stamp','missing_projector'):
            f,node,msg,free,unknown,mask,region,calls,events=self.setup_case()
            if condition=='invalid':node.current_place_scope_valid=False
            elif condition=='outside':node.pose_odom.x=8.
            elif condition=='frontier':unknown[2,3]=True;free[2,3]=False
            elif condition=='wrong_stamp':node._place_partition_tf_key=('map',6.)
            elif condition=='missing_projector':node._place_partition_projector=None
            else:node.active_portal_authentication=lambda:S(state='entering')
            self.assertFalse(f(node,msg,free,unknown,mask,1,8.),condition)
            self.assertEqual(calls,[],condition)


if __name__=='__main__':
    unittest.main()
