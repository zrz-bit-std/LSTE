"""GP authentication uses recovery topology with strict endpoint admission."""
from pathlib import Path
import sys,unittest
from types import SimpleNamespace as S
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from global_frontier_selection_planner import GlobalFrontierSelectionPlannerMixin
class AuthTopologyTest(unittest.TestCase):
    def fixture(self):
        return S(gp_portal_assistance_enabled=True,active_portal_authentication=lambda:S(state='returning'),
                 junction_portal_authentication=None,_reset_frontier_selection_metrics=lambda:None,
                 choose_gp_branch_frontier=lambda message,steps,*args:steps)
    def test_disconnected_strict_components_keep_reachable_strict_goal(self):
        route=np.array([[0,1,2,3,4]])
        strict=np.array([[0,-1,-1,-1,-1]])
        endpoints=np.array([[True,False,False,False,True]])
        selected=GlobalFrontierSelectionPlannerMixin.choose_frontier(self.fixture(),None,route,None,None,None,0.,preferred_steps=strict,preferred_mask=endpoints)
        self.assertEqual(selected[0,4],4)
        self.assertTrue(np.all(selected[~endpoints]<0))
    def test_no_endpoint_mask_cannot_promote_unchecked_recovery_cells(self):
        strict=np.array([[0,-1,-1]])
        selected=GlobalFrontierSelectionPlannerMixin.choose_frontier(self.fixture(),None,np.array([[0,1,2]]),None,None,None,0.,preferred_steps=strict)
        np.testing.assert_array_equal(selected,strict)
    def test_unreachable_strict_endpoint_stays_unreachable(self):
        selected=GlobalFrontierSelectionPlannerMixin.choose_frontier(self.fixture(),None,np.array([[0,-1]]),None,None,None,0.,preferred_steps=np.array([[0,-1]]),preferred_mask=np.array([[True,True]]))
        self.assertEqual(selected[0,1],-1)
if __name__=='__main__':unittest.main()
