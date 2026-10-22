import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from gp_exploration_tree import ExplorationTree


def proposal(heading,gate=(1,0)):
    return dict(heading=heading,physical_gate=list(gate),geometry=dict(
         position=list(gate),width=2.,passability='open'),variance=.1,reach=3.,angular_width=.5)


class TreeTest(unittest.TestCase):
    def test_duplicate_fused_direction_does_not_block_junction_confirmation(self):
        t=ExplorationTree()
        ps=[proposal(0),proposal(math.pi/2,(0,1)),
            proposal(math.pi,(-1,0)),proposal(math.pi+.1,(-1,.1))]
        t.observe((0,0),(0,0),ps,0,1,1)
        self.assertTrue(t.nodes[1].get('junction_confirmed'))
        self.assertEqual(len(t.branches),3)

    def test_authentication_freezes_gate_before_controller_dispatch(self):
        t=ExplorationTree()
        t.observe((0,0),(0,0),[proposal(0,(1,0))],0,1,1)
        t.branches[1]['authentication_state']='pending'
        t.observe((0,0),(0,0),[proposal(.2,(2,0))],1,2,2)
        self.assertEqual(t.branches[1]['physical_gate'],[1,0])
        self.assertEqual(t.branches[1]['heading'],0)
        t.dispatch(1,4,(0,0))
        self.assertTrue(t.arrived(1,(1.5,0)))

    def test_distinct_observations_required_and_semantics_choose(self):
        t=ExplorationTree()
        ps=[proposal(0),proposal(math.pi/2,(0,1))]
        t.observe((0,0),(0,0),ps,0,1,1)
        t.observe((0,0),(0,0),ps,5,1,2)
        self.assertIsNone(t.choose({1:1,2:5},[1,2]))
        t.observe((0,0),(0,0),ps,6,2,3,lambda a,b:True)
        self.assertEqual(t.choose({1:1,2:5},[1,2])['id'],2)
        t.dispatch(2,8,(0,0))
        self.assertEqual(t.branches[1]['state'],'pending')
        self.assertFalse(t.arrived(2,(0,.5)))
        self.assertTrue(t.arrived(2,(0,1.5)))

    def test_entered_branch_is_resumed_after_another_branch_is_dispatched(self):
        t=ExplorationTree()
        ps=[proposal(0,(1,0)),proposal(math.pi,(-1,0))]
        t.observe((0,0),(0,0),ps,0,1,1)
        t.observe((0,0),(0,0),ps,1,2,2,lambda a,b:True)
        t.dispatch(1,4,(0,0))
        self.assertTrue(t.arrived(1,(1.5,0)))
        t.dispatch(2,5,(0,0))
        self.assertEqual(t.active_branch_id,2)
        self.assertEqual(t.continuation_branch()['id'],1)

    def test_revisit_reuses_node_and_branch_identity(self):
        t=ExplorationTree(); ps=[proposal(0),proposal(math.pi/2,(0,1)),proposal(math.pi,(-1,0))]
        t.observe((0,0),(0,0),ps,0,1,1)
        t.observe((.1,.1),(.25,.15),ps,1,2,2,lambda a,b:True)
        self.assertEqual(len(t.nodes),1)
        self.assertEqual(len(t.branches),3)
        self.assertEqual(t.branches[1]['map_epoch'],2)

    def test_nearby_but_wall_separated_node_not_merged(self):
        t=ExplorationTree(node_radius=.5)
        ps=[proposal(0),proposal(math.pi/2),proposal(math.pi)]
        t.observe((0,0),(0,0),ps,0,1,1)
        t.observe((2,0),(2,0),ps,1,2,2,lambda a,b:False)
        self.assertEqual(len(t.nodes),2)

    def test_destination_is_attached_to_selected_branch(self):
        t=ExplorationTree(node_radius=.5)
        ps=[proposal(0),proposal(math.pi/2),proposal(math.pi)]
        t.observe((0,0),(0,0),ps,0,1,1)
        t.dispatch(1,1,(0,0))
        t.observe((3,0),(3,0),ps,1,2,2,lambda a,b:False)
        self.assertEqual(t.nodes[2]['parent_branch_id'],1)
        self.assertEqual(t.branches[1]['destination_node_id'],2)
        self.assertEqual(t.branches[1]['state'],'active')
        self.assertTrue(t.arrived(1,(3,0)))
        self.assertTrue(t.spaces)
        self.assertEqual(t.branches[2]['state'],'pending')

    def test_semantics_precede_geometric_tie_break(self):
        t=ExplorationTree();ps=[proposal(0),proposal(math.pi/2,(0,1))]
        t.observe((0,0),(0,0),ps,0,1,1)
        t.observe((0,0),(0,0),ps,1,2,2,lambda a,b:True)
        self.assertEqual(t.choose({1:.8,2:.2},[1,2],{1:-100,2:0})['id'],1)
        self.assertEqual(t.choose({1:.8,2:.8},[1,2],{1:-100,2:0})['id'],2)

    def test_passing_gate_plane_far_from_opening_is_not_entry(self):
        t=ExplorationTree();ps=[proposal(0)]
        t.observe((0,0),(0,0),ps,0,1,1)
        t.dispatch(1,1,(0,3))
        self.assertFalse(t.arrived(1,(2,3)))
        self.assertFalse(t.branches[1].get('destination_verified',False))
        self.assertEqual(len(t.spaces),1)

    def test_destination_side_start_does_not_invent_crossing(self):
        t=ExplorationTree();t.observe((0,0),(0,0),[proposal(0)],0,1,1)
        t.dispatch(1,1,(1.5,0))
        self.assertFalse(t.arrived(1,(2,0)))

    def test_confirmed_fork_retires_active_coarse_hypothesis(self):
        t=ExplorationTree(node_radius=2.)
        t.observe((0,0),(0,0),[proposal(0),proposal(math.pi)],0,1,1)
        t.dispatch(1,1,(0,0))
        early=[proposal(.1,(3.5,0)),proposal(.7,(3.5,.5)),proposal(math.pi,(2.5,0))]
        t.observe((3,0),(3,0),early,1,2,2,lambda a,b:True)
        coarse=t.nodes[2]['branch_ids'][0]
        t.dispatch(coarse,2,(3,0))
        refined=[proposal(0,(4,0)),proposal(math.pi/2,(3.5,1)),proposal(math.pi,(2.5,0))]
        t.observe((3.5,0),(3.5,0),refined,2,3,3,lambda a,b:True)
        self.assertEqual(t.branches[coarse]['state'],'superseded')
        live=[b for b in t.branches.values() if b['node_id']==2 and b['state']!='superseded']
        self.assertEqual(len(live),3)
        self.assertTrue(all(b.get('destination_space_id') is None or b.get('connection')=='return' for b in live))

    def test_candidate_carries_identity_through_activation_boundary(self):
        from global_frontier_models import SelectedFrontier
        first=[0,0,1.,2.,3.,0.,0.,0.,None,0,'frontier_endpoint']
        first.extend([None]*(21-len(first)));first.append(17)
        second=list(first);second[21]=23
        self.assertEqual(SelectedFrontier.from_candidate(tuple(first)).branch_id,17)
        self.assertEqual(SelectedFrontier.from_candidate(tuple(second)).branch_id,23)

if __name__=='__main__':unittest.main()
