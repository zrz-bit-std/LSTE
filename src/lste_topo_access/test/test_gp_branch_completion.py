"""GP stability/map waits must not cancel the persistent execution lease."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from global_frontier_planning_snapshot import GlobalFrontierPlanningSnapshotMixin
from gp_exploration_tree import ExplorationTree


class Probe(GlobalFrontierPlanningSnapshotMixin):
    def __init__(self):
        self.exploration_method='gp_branch_tree'
        self.gp_exploration_tree=ExplorationTree()
        self.frontier_exhausted=True
        self.events=[]
    def publish_status(self,event,**kwargs):self.events.append((event,kwargs))


class CompletionTest(unittest.TestCase):
    def test_no_sensor_observation_is_a_wait(self):
        p=Probe();p._report_no_frontier_selection()
        self.assertFalse(p.frontier_exhausted)
        self.assertEqual(p.events[0][0],'gp_branch_waiting')

    def test_unstable_or_unreachable_branch_is_not_exhaustion(self):
        p=Probe()
        p.gp_exploration_tree.observe((0,0),(0,0),[dict(heading=0,
            physical_gate=[1,0],geometry=dict(passability='unknown'))],0,1,1)
        p._report_no_frontier_selection()
        self.assertFalse(p.frontier_exhausted)
        self.assertEqual(p.events[0][1]['pending_branch_ids'],[1])
        self.assertNotIn('frontier_exhausted',[x[0] for x in p.events])

if __name__=='__main__':unittest.main()
