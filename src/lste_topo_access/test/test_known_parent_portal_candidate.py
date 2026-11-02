"""A known parent direction is not a new authentication or child Place."""
import unittest,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from global_frontier_place_partition import portal_partition_evidence
class KnownParentCandidateTest(unittest.TestCase):
 def evidence(self,current=2,destination=(3.5,4.5),unknown=False,reverse=False):
  grid=np.zeros((10,20),dtype=bool);grid[3:6,:]=True
  if unknown:grid[4,3]=False
  p=dict(id=7,gate=(8.5,4.5),destination=(10.5,4.5),width=4.,bounds=[(8.5,2.5),(8.5,6.5)],source_place_id=1,destination_place_id=2)
  if reverse:p.update(source_place_id=2,destination_place_id=1)
  candidate=dict(id=0,gate=(12.5,4.5),destination=destination,width=4.,bounds=[(12.5,2.5),(12.5,6.5)])
  return portal_partition_evidence(grid,1.,(0,0),(15.5,4.5),destination,candidate,[p],current_place_id=current)
 def test_bound_parent_component_is_identified(self):
  r=self.evidence();self.assertEqual(r['reason'],'visited_entry_side_of_current_place');self.assertEqual(r['duplicate_portal_id'],7);self.assertEqual(r['parent_place_id'],1)
 def test_unknown_destination_cannot_prove_parent(self):
  self.assertNotEqual(self.evidence(unknown=True)['reason'],'visited_entry_side_of_current_place')
 def test_unbound_current_place_cannot_claim_return(self):
  self.assertNotEqual(self.evidence(current=9)['reason'],'visited_entry_side_of_current_place')
 def test_outgoing_child_edge_cannot_claim_parent(self):
  self.assertNotEqual(self.evidence(reverse=True)['reason'],'visited_entry_side_of_current_place')
class ParentCandidateIntegrationTest(unittest.TestCase):
 def run_candidate(self,crossed=True,gate=8.1,goal_depth=1.3):
  from types import SimpleNamespace as NS
  from global_frontier_place_portal_validation import duplicate_observation_portal
  import math
  grid=np.zeros((10,20),dtype=bool);grid[3:6,:]=True
  portal=dict(id=7,gate=(8.5,4.5),destination=(10.5,4.5),width=4.,bounds=[(8.5,2.5),(8.5,6.5)],source_place_id=1,destination_place_id=2,state='crossed' if crossed else 'certified',crossing_count=1 if crossed else 0,gp_authentication_state='completed')
  node=NS(_place_partition_tf_key=('map',1.),_place_partition_projector=lambda x,y:(x,y),current_physical_place_id=2,portal_hypothesis_ledger=NS(get=lambda key:portal),current_place_scope_projected_portals=[portal],transform_xy=lambda frame,source,x,y:(x,y),_update_place_partition=lambda *args:None)
  message=NS(header=NS(frame_id='map',stamp=NS(to_sec=lambda:1.)),info=NS(resolution=1.,origin=NS(position=NS(x=0.,y=0.))))
  context=NS(known_free=grid,unknown=~grid,components=NS(epoch=1))
  tree=NS(nodes={1:dict(physical_xy=(15.5,4.5))})
  branch=dict(node_id=1,heading=math.pi,map_geometry=dict(width=4.,physical_bounds=[(gate,2.5),(gate,6.5)]))
  result=duplicate_observation_portal(node,tree,branch,message,context,goal_depth=goal_depth)
  return result,branch
 def test_crossed_canonical_parent_marks_return_before_authentication(self):
  result,branch=self.run_candidate();self.assertEqual(result['id'],7);self.assertEqual(branch['connection'],'return')
 def test_uncrossed_certification_cannot_claim_visited_parent(self):
  result,branch=self.run_candidate(False);self.assertIsNone(result);self.assertNotIn('connection',branch)
 def test_candidate_uses_same_deep_goal_as_authentication(self):
  shallow,branch=self.run_candidate(gate=9.1,goal_depth=1.3)
  self.assertIsNone(shallow)
  deep,branch=self.run_candidate(gate=9.1,goal_depth=2.4)
  self.assertEqual(deep['id'],7);self.assertEqual(branch['connection'],'return')
if __name__=='__main__':unittest.main()
