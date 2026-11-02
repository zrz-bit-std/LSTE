"""Current-Place WI remain executable after region-memory reclassification."""
from pathlib import Path
from types import SimpleNamespace as S
from dataclasses import replace
import unittest,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from global_frontier_selection_planner import GlobalFrontierSelectionPlannerMixin
from global_frontier_frontier_decision import FrontierActionCandidate,FrontierDecisionEvidence
class Harness(GlobalFrontierSelectionPlannerMixin):
 def publish_status(self,*args,**kwargs):pass
class DurableMemoryTierTest(unittest.TestCase):
 def setUp(self):
  self.h=Harness();self.request=S(allowed_region_tiers=('new',),components=S(epoch=1))
  self.c=FrontierActionCandidate(candidate_id=('local',7),action_category='local',route=(1,2),region_tier='revisit',place_id=2,work_item_id=7,evidence=FrontierDecisionEvidence(4,1,0,1,2))
 def select(self,c=None,durable=False):return self.h._select_frontier_decision_candidate([c or self.c],('local',),self.request,durable_work_item=durable)
 def test_exact_current_work_survives_revisit_tier(self):self.assertEqual(self.select(durable=True),(1,2))
 def test_raw_frontier_still_obeys_memory_tier(self):self.assertIsNone(self.select())
 def test_unproven_path_stays_rejected(self):self.assertIsNone(self.select(replace(self.c,route_valid=False),True))
 def test_covered_view_stays_rejected(self):self.assertIsNone(self.select(replace(self.c,covered=True),True))
 def test_resolved_work_stays_rejected(self):self.assertIsNone(self.select(replace(self.c,work_item_unresolved=False),True))
 def test_graph_caller_requires_current_place_identity(self):
  from unittest.mock import patch
  self.h.frontier_action_policy='event_pareto';self.h.current_physical_place_id=2
  plan=S(obligation_kind='work_item',obligation_id=7,current_place_id=2,work_item_tour=(7,),action='observe_local_work')
  with patch('global_frontier_selection_planner.candidate_matches_tsp_work_item',return_value=True):
   self.assertEqual(self.h._select_graph_plan_candidate([self.c],plan,self.request),(1,2))
   plan.current_place_id=3
   self.assertIsNone(self.h._select_graph_plan_candidate([self.c],plan,self.request))
if __name__=='__main__':unittest.main()
