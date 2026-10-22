"""Task evidence follows calibrated optical bearings, not robot yaw guesses."""
import sys
import unittest
# Load NumPy before patch.dict restores sys.modules to avoid reloading its C API.
import numpy
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch, Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
with patch.dict(sys.modules,{'rospy':NS(Duration=lambda x:x)}):
    from gp_branch_frontier import GPBranchFrontierMixin
from gp_exploration_tree import ExplorationTree
from global_frontier_semantic_belief import SemanticTaskIntent


class SemanticTest(unittest.TestCase):
    def host(self):
        host=GPBranchFrontierMixin();host.gp_exploration_tree=ExplorationTree()
        tree=host.gp_exploration_tree
        proposals=[dict(heading=h,physical_gate=[1,i],geometry=dict(passability='open'))
                   for i,h in enumerate([0.,1.0])]
        tree.observe((0,0),(0,0),proposals,0,1,1)
        tree.observe((0,0),(0,0),proposals,1,2,2,lambda a,b:True)
        host.gp_branch_camera_info=NS(K=[200.,0.,320.,0.,200.,240.,0.,0.,1.],
                                    width=640,header=NS(frame_id='optical'))
        rotation=NS(x=-.5,y=.5,z=-.5,w=.5)
        host.tf_buffer=NS(lookup_transform=lambda *a:NS(transform=NS(rotation=rotation)))
        intent=SemanticTaskIntent('find_cup','v1',frozenset(['cup']),
                  frozenset(['desk']),frozenset(['bed']))
        host.semantic_place_belief=NS(intent=intent)
        return host

    def test_semantic_direction_and_repeated_frames(self):
        host=self.host()
        message=NS(header=NS(stamp=1),target_dets=[NS(label='cup',score=.9,cx=0.)],
                   env_dets=[NS(label='desk',score=.8,cx=.5)])
        host.observe_gp_branch_semantics(message)
        host.observe_gp_branch_semantics(message)
        tree=host.gp_exploration_tree
        self.assertAlmostEqual(tree.branches[1]['semantic_score'],.8)
        self.assertAlmostEqual(tree.branches[2]['semantic_score'],.9)
        chosen=tree.choose({i:b['semantic_score'] for i,b in tree.branches.items()},[1,2],{1:0,2:-100})
        self.assertEqual(chosen['id'],2)

    def test_authentication_route_keeps_priority_outside_source_node(self):
        host=self.host()
        host.gp_exploration_tree.current_node_id=99
        host.junction_portal_authentication=NS(node_id=1)
        auth=NS(node_id=1,state='returning')
        host.active_portal_authentication=lambda:auth
        host._portal_authentication_route_candidate=lambda message,steps,record:('return_to_source',record.node_id)
        self.assertEqual(host.choose_gp_branch_frontier(None,None,0,None,None,()),
                         ('return_to_source',1))

    def test_physical_return_releases_old_controller_lease_once(self):
        from global_frontier_portal_authentication import DoorwayAuthentication
        host=self.host()
        auth=DoorwayAuthentication(1,1,(1,0),(2,0))
        auth.source_xy=(0,0);auth.state='returning'
        host.pose_odom=NS(x=.5,y=0,theta=0)
        host.active_portal_authentication=lambda:auth
        host.finish_portal_authentication_branch=Mock()
        host.publish_status=Mock()
        host.release_active_frontier=Mock()
        host._update_portal_authentication_from_map(host.gp_exploration_tree,None,None,[],0,5)
        self.assertEqual(auth.state,'completed')
        host.release_active_frontier.assert_called_once_with(
            discard_prefetch=True,queue_successor=False,
            await_controller_release_ack=True,preserve_place_departure=False)
        host._update_portal_authentication_from_map(host.gp_exploration_tree,None,None,[],0,6)
        self.assertEqual(host.release_active_frontier.call_count,1)

    def test_missing_calibration_does_not_invent_bearing(self):
        host=self.host();host.gp_branch_camera_info=None
        host.observe_gp_branch_semantics(NS())
        self.assertIsNone(host.gp_exploration_tree.branches[1]['semantic_score'])

if __name__=='__main__':unittest.main()
