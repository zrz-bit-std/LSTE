"""Rejected policy candidates report reasons without changing admission."""
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import patch
import unittest,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from global_frontier_selection_planner import GlobalFrontierSelectionPlannerMixin
class Harness(GlobalFrontierSelectionPlannerMixin):
 def __init__(self):self.events=[]
 def publish_status(self,event,**data):self.events.append((event,data))
class RejectionReportTest(unittest.TestCase):
 def test_rejection_is_preserved_and_identical_report_is_deduplicated(self):
  h=Harness();candidate=S(work_item_id=7,place_id=2,action_category='local',region_tier='revisit')
  decision=S(selected=None,category=None,feasible=(),pareto_front=(),rejected=(S(candidate=candidate,reasons=('route_not_proven',)),),decision_epoch=1)
  request=S(allowed_region_tiers=None,components=S(epoch=1))
  with patch('global_frontier_selection_planner.select_frontier_action',return_value=decision):
   self.assertIsNone(h._select_frontier_decision_candidate([candidate],(),request))
   self.assertIsNone(h._select_frontier_decision_candidate([candidate],(),request))
  self.assertEqual(len(h.events),1);self.assertEqual(h.events[0][1]['rejected'][0]['reasons'],['route_not_proven'])
if __name__=='__main__':unittest.main()
