"""A prefetched branch must acquire its own graph and route identity."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from test_frontier_prefetch_method_boundary import GeometryBaselinePrefetchProbe
from gp_branch_frontier import GPBranchFrontierMixin
from gp_exploration_tree import ExplorationTree


class Probe(GPBranchFrontierMixin,GeometryBaselinePrefetchProbe):
    def __init__(self):
        GeometryBaselinePrefetchProbe.__init__(self)
        self.pose_odom=NS(x=0.,y=0.)
        self.active_route_lifecycle_transaction_id=123
        self.topology_component_epoch=7
        self.graph_route_action_transaction_sequence=40
        self.gp_exploration_tree=ExplorationTree()
        proposals=[dict(heading=h,physical_gate=g,geometry=dict(passability='open',width=2.))
                   for h,g in [(0.,[1.,0.]),(1.57,[0.,1.])]]
        self.gp_exploration_tree.observe((0,0),(0,0),proposals,0,1,1)
        self.commit_gp_branch_transaction(1,4)
        self.prefetched_branch_id=2
    def publish_status(self,*args,**kwargs):pass


class PrefetchTest(unittest.TestCase):
    def promote(self,preserve):
        p=Probe()
        result=p.install_prefetched_frontier(NS(info=NS(resolution=.1)),np.zeros((1,1)),
               (0,0),5.,0,0,0.,2.,preserve)
        return p,result

    def test_different_branch_has_new_graph_transaction(self):
        p,result=self.promote(False)
        self.assertTrue(result)
        self.assertEqual(p.active_route_id,5)
        self.assertEqual(p.gp_exploration_tree.active_branch_id,2)
        self.assertEqual(p.graph_route_action_transaction.graph_transaction_id,42)
        self.assertEqual(p.active_route_dispatch_contract['graph_transaction_id'],42)
        self.assertEqual(p.last_graph_route_plan.obligation_id,2)

    def test_shared_path_prefix_cannot_reuse_another_branch_lease(self):
        p,result=self.promote(True)
        self.assertIsNone(result)
        self.assertEqual(p.active_route_id,4)
        self.assertEqual(p.gp_exploration_tree.active_branch_id,1)
        self.assertEqual(p.graph_route_action_transaction.graph_transaction_id,41)

    def test_retired_branch_cannot_be_promoted_from_old_cache(self):
        p=Probe();p.gp_exploration_tree.branches[2]['state']='superseded'
        result=p.install_prefetched_frontier(NS(info=NS(resolution=.1)),np.zeros((1,1)),
               (0,0),5.,0,0,0.,2.,False)
        self.assertIsNone(result)
        self.assertEqual(p.active_route_id,4)
        self.assertEqual(p.gp_exploration_tree.active_branch_id,1)
        self.assertTrue(p.cleared)

if __name__=='__main__':unittest.main()
