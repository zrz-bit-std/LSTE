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

    def test_confirmed_junction_reuses_node_from_same_slam_component_without_line_of_sight(self):
        t=ExplorationTree(node_radius=2.0)
        first=[
            proposal(-2.34,(-1.0,-1.0)),
            proposal(.56,(0.0,0.0)),
            proposal(2.03,(-.7,.2)),
        ]
        second=[
            proposal(-2.34,(-1.3,-1.25)),
            proposal(.82,(1.0,1.2)),
            proposal(2.52,(-1.8,2.2)),
        ]
        t.observe((0.0,0.0),(0.0,0.0),first,0.0,1,1)
        t.observe((.9,.95),(.9,.95),second,1.0,2,2,
                  lambda a,b:False, same_component=lambda a,b:True)
        self.assertEqual(len(t.nodes),1)
        self.assertEqual(t.current_node_id,1)
        self.assertEqual(len(t.branches),3)
        self.assertTrue(all(t.branches[key]["stable"] for key in (1,2,3)))

    def test_confirming_refined_cross_reuses_matching_early_arm_identities(self):
        t=ExplorationTree(node_radius=2.0)
        early=[
            proposal(-1.07,(-1.79,-.46)),
            proposal(.20,(-1.55,.37)),
            proposal(-math.pi,(-2.68,.07)),
        ]
        confirmed=[
            proposal(.19,(1.99,.34)),
            proposal(1.38,(.14,.56)),
            proposal(3.12,(-2.47,.02)),
            proposal(-1.44,(.11,-.63)),
        ]
        t.observe((0.,0.),(0.,0.),early,0.,1,1)
        self.assertFalse(t.nodes[1].get('junction_confirmed',False))
        t.observe((0.,0.),(0.,0.),confirmed,1.,2,2,
                  lambda _a,_b:True,same_component=lambda _a,_b:True)
        self.assertTrue(t.nodes[1]['junction_confirmed'])
        self.assertEqual(t.current_node_id,1)
        self.assertEqual(len(t.branches),4)
        self.assertEqual(set(t.nodes[1]['branch_ids']),{1,2,3,4})
        self.assertEqual(t.branches[3]['state'],'pending')
        self.assertAlmostEqual(t.branches[3]['heading'],3.12)

    def test_cross_views_across_open_intersection_merge_into_one_node(self):
        t=ExplorationTree(node_radius=2.0)
        first=[
            proposal(-1.103,(-1.429,-.465)),
            proposal(.143,(-1.106,.157)),
            proposal(.968,(-1.360,.566)),
            proposal(-3.142,(-2.300,.071)),
        ]
        revisit=[
            proposal(-1.573,(-.056,-1.423)),
            proposal(-.082,(3.432,-.200)),
            proposal(1.539,(-.014,1.577)),
            proposal(3.132,(-2.957,.097)),
        ]
        t.observe((-1.700,.071),(-1.700,.071),first,0,1,1)
        t.nodes[1]['junction_confirmed']=True
        t.observe((1.291,.026),(1.291,.026),revisit,1,2,2,
                  lambda _a,_b:True,
                  same_component=lambda _a,_b:True)
        self.assertEqual(len(t.nodes),1)
        self.assertEqual(t.current_node_id,1)
        self.assertEqual(len(t.nodes[1]['branch_ids']),4)
        self.assertEqual(len(t.branches),4)

    def test_cross_views_with_opposite_side_heading_drift_merge(self):
        t=ExplorationTree(node_radius=2.0)
        first=[proposal(h,(-1.5,.1)) for h in (-.85,.14,.86,3.14)]
        revisit=[proposal(h,(x,y)) for h,(x,y) in zip(
            (-1.57,-.15,1.56,-3.09),
            ((-.02,-1.8),(1.77,.53),(-.01,1.50),(-3.11,.64)))]
        t.observe((-1.89,.06),(-1.89,.06),first,0,1,1)
        t.nodes[1]['junction_confirmed']=True
        for branch in t.branches.values():branch['state']='authenticated'
        t.observe((.04,-1.04),(.04,-1.04),revisit,1,2,2,
                  lambda _a,_b:False,
                  same_component=lambda _a,_b:True)
        self.assertEqual(len(t.nodes),1)
        self.assertEqual(len(t.branches),4)

    def test_cross_views_without_same_slam_component_stay_distinct(self):
        t=ExplorationTree(node_radius=2.0)
        first=[proposal(h,(-1.0,0.0)) for h in (-1.1,.14,.97,3.14)]
        revisit=[proposal(h,(1.0,0.0)) for h in (-1.57,-.08,1.54,3.13)]
        t.observe((-1.7,0),(-1.7,0),first,0,1,1)
        t.nodes[1]['junction_confirmed']=True
        t.observe((1.29,0),(1.29,0),revisit,1,2,2,
                  lambda _a,_b:True,
                  same_component=lambda _a,_b:False)
        self.assertEqual(len(t.nodes),2)

    def test_nearby_fork_in_same_component_stays_distinct_without_gate_overlap(self):
        t=ExplorationTree(node_radius=2.0)
        first=[
            proposal(-2.34,(-1.0,-1.0)),
            proposal(.56,(0.0,0.0)),
            proposal(2.03,(-.7,.2)),
        ]
        second=[
            proposal(-2.34,(4.0,4.0)),
            proposal(.82,(5.0,5.0)),
            proposal(2.52,(4.0,6.0)),
        ]
        t.observe((0.0,0.0),(0.0,0.0),first,0.0,1,1)
        t.observe((.9,.95),(.9,.95),second,1.0,2,2,
                  lambda a,b:False, same_component=lambda a,b:True)
        self.assertEqual(len(t.nodes),2)

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

    def test_unconfirmed_t_junction_estimate_merges_with_slammatched_center(self):
        t=ExplorationTree(node_radius=2.)
        # The first view sees a wide bidirectional corridor and cannot yet
        # confirm the T fan.
        early=[proposal(-.237,(-1.29,-.06)),
               proposal(3.124,(-3.01,.10))]
        t.observe((-1.87,.07),(-1.87,.07),early,0,1,1,lambda a,b:True)
        refined=[proposal(-2.493,(-1.06,1.01)),
                 proposal(1.343,(.12,2.37)),
                 proposal(-.451,(.52,1.53)),
                 proposal(3.126,(-3.13,.49))]

        t.observe((-.02,1.79),(-.02,1.79),refined,1,2,2,lambda a,b:True)

        self.assertEqual(len(t.nodes),1)
        live=[b for b in t.branches.values() if b['state']!='superseded']
        self.assertEqual(len(live),3)
        self.assertEqual(len(t.nodes),1)
        self.assertTrue(t.nodes[1].get('junction_confirmed'))
        self.assertEqual(t.nodes[1]['physical_xy'],[-.02,1.79])

    def test_confirmed_cross_revisit_merges_small_slamextent_offset(self):
        t = ExplorationTree(node_radius=2.0)
        first = [
            proposal(-0.957, (-1.490, -0.432)),
            proposal(0.146, (-1.242, 0.146)),
            proposal(0.977, (-1.499, 0.556)),
            proposal(3.140, (-2.435, 0.060)),
        ]
        revisit = [
            proposal(-1.577, (0.008, -1.375)),
            proposal(-0.231, (2.165, 0.621)),
            proposal(1.490, (0.080, 1.822)),
            proposal(-2.917, (-3.583, 0.301)),
        ]
        t.observe((-1.835, 0.059), (-1.835, 0.059), first, 0, 1, 1)
        # The run reached this viewpoint only after node 2 had passed its
        # multi-frame junction confirmation; model that established state.
        t.nodes[1]['junction_confirmed'] = True
        t.observe((0.034, 0.998), (0.034, 0.998), revisit, 1, 2, 2,
                  lambda _a, _b: True)

        self.assertEqual(len(t.nodes), 1)
        self.assertEqual(t.current_node_id, 1)
        self.assertEqual(len(t.nodes[1]["branch_ids"]), 4)

    def test_authenticated_arm_revisit_merges_only_with_slam_free_connection(self):
        t = ExplorationTree(node_radius=2.0)
        first = [
            proposal(-1.016, (-0.821, -0.541)),
            proposal(0.187, (-0.548, 0.080)),
            proposal(1.090, (-0.860, 0.501)),
            proposal(3.068, (-2.534, 0.072)),
        ]
        revisit = [
            proposal(-1.752, (0.535, -1.743)),
            proposal(0.001, (4.358, 0.036)),
            proposal(1.984, (-0.026, 2.043)),
            proposal(3.060, (-2.500, 0.070)),
        ]
        t.observe((0, 0), (0, 0), first, 0, 1, 1, lambda _a, _b: True)
        t.nodes[1]['junction_confirmed'] = True
        for branch in t.branches.values():
            branch['state'] = 'authenticated'

        t.observe((0, 0), (0, 0), revisit, 1, 2, 2,
                  lambda _a, _b: True)

        self.assertEqual(len(t.nodes[1]['branch_ids']), 4)
        self.assertEqual(len(t.branches), 4)

    def test_authenticated_arm_with_unknown_or_blocked_slam_path_stays_distinct(self):
        t = ExplorationTree(node_radius=2.0)
        first = [
            proposal(-1.016, (-0.821, -0.541)),
            proposal(0.187, (-0.548, 0.080)),
            proposal(1.090, (-0.860, 0.501)),
            proposal(3.068, (-2.534, 0.072)),
        ]
        revisit = [
            proposal(-1.752, (0.535, -1.743)),
            proposal(0.001, (4.358, 0.036)),
            proposal(1.984, (-0.026, 2.043)),
            proposal(3.060, (-2.500, 0.070)),
        ]
        t.observe((0, 0), (0, 0), first, 0, 1, 1, lambda _a, _b: True)
        t.nodes[1]['junction_confirmed'] = True
        for branch in t.branches.values():
            branch['state'] = 'authenticated'

        t.observe((0, 0), (0, 0), revisit, 1, 2, 2,
                  lambda a, b: tuple(a) == tuple(b))

        self.assertGreater(len(t.branches), 4)

    def test_nearby_fork_with_different_directions_stays_distinct(self):
        t = ExplorationTree(node_radius=2.0)
        first = [proposal(0), proposal(2.1), proposal(4.2)]
        second = [proposal(0.9), proposal(2.8), proposal(4.9)]
        t.observe((0, 0), (0, 0), first, 0, 1, 1)
        t.observe((2.1, 0), (2.1, 0), second, 1, 2, 2,
                  lambda _a, _b: True)

        self.assertEqual(len(t.nodes), 2)

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

    def test_canonical_parent_return_remains_one_live_direction(self):
        t=ExplorationTree(node_radius=1.)
        first=[proposal(math.pi,(-1,0)),proposal(0,(1,0)),proposal(math.pi/2,(0,1))]
        t.observe((0,0),(0,0),first,0.,1,1)
        branch=t.branches[1]
        branch.update(state='return',connection='return',canonical_portal_id=7,authentication_state='deduplicated')
        shifted=[proposal(math.pi,(-5,0)),first[1],first[2]]
        t.observe((0,0),(0,0),shifted,2.,2,2,lambda a,b:True)
        self.assertEqual(len(t.branches),3)
        self.assertTrue(branch['stable'])
        self.assertEqual(branch['state'],'return')
        self.assertIsNone(t.choose({1:1.0},[1]))

    def test_canonical_return_does_not_merge_a_blocked_distant_gate(self):
        t=ExplorationTree(node_radius=1.)
        first=[proposal(math.pi,(-1,0)),proposal(0,(1,0)),proposal(math.pi/2,(0,1))]
        t.observe((0,0),(0,0),first,0.,1,1)
        t.branches[1].update(state='return',connection='return',canonical_portal_id=7,authentication_state='deduplicated')
        shifted=[proposal(math.pi,(-5,0)),first[1],first[2]]
        t.observe((0,0),(0,0),shifted,2.,2,2,lambda a,b:math.dist(a,b)<.01)
        self.assertEqual(len(t.branches),4)

if __name__=='__main__':unittest.main()
