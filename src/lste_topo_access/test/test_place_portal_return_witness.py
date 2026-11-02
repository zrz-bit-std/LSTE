"""A frozen source on the SLAM gate cut needs a physically proven return witness."""
from pathlib import Path
from types import SimpleNamespace as S
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from global_frontier_portal_authentication import DoorwayAuthentication
from global_frontier_place_portal_validation import authenticated_place_portal_evidence


class ReturnedSourceWitnessTest(unittest.TestCase):
    def fixture(self):
        free=np.zeros((40,100),dtype=bool);free[10:30,1:99]=True
        message=S(header=S(frame_id='map',stamp=S(to_sec=lambda:1.)),info=S(resolution=.1,origin=S(position=S(x=0.,y=0.))))
        branch=dict(map_geometry=dict(width=2.,physical_bounds=[[5.,1.],[5.,3.]]))
        parent=free.copy();parent[:,50:]=False
        node=S(_place_partition_tf_key=('map',1.),_place_partition_projector=lambda x,y:(x,y),active_semantic_room=dict(portal_id=1),
               portal_authentication_map_context=S(known_free=free,unknown=~free,components=S(epoch=1)),
               portal_authentication_map_message=message,
               transform_xy=lambda target,source,x,y:(x,y),
               gp_exploration_tree=S(branches={7:branch}),current_physical_place_id=2,
               portal_hypothesis_ledger=S(get=lambda key:dict(physical_destination=[2.,2.])),
               current_place_scope_projected_portals=[],
               _update_place_partition=lambda *args:None,
               current_place_scope_mask=parent,current_place_scope_message=message,
               current_place_scope_valid=True,current_place_scope_id=2,
               pose_odom=S(x=4.5,y=2.))
        auth=DoorwayAuthentication(7,1,(5.,2.),(7.4,2.))
        auth.source_xy=(4.95,2.);auth.inside_xy=(5.8,2.)
        auth.state='completed';auth.return_gate_cleared=True
        return node,auth

    def test_actual_parent_side_return_resolves_source_on_cut(self):
        node,auth=self.fixture()
        evidence=authenticated_place_portal_evidence(node,auth)
        self.assertEqual(evidence['reason'],'distinct_destination_component')
        self.assertFalse(evidence['reject'])
        self.assertEqual(evidence['source_witness'],'physical_return_pose')
        self.assertNotEqual(evidence['source_label'],evidence['destination_label'])

    def test_return_witness_rejects_positive_same_place_bypass(self):
        node,auth=self.fixture()
        node.gp_exploration_tree.branches[7]['map_geometry'].update(
            width=.2,physical_bounds=[[5.,1.9],[5.,2.1]])
        evidence=authenticated_place_portal_evidence(node,auth)
        self.assertEqual(evidence['source_witness'],'physical_return_pose')
        self.assertTrue(evidence['reject'])
        self.assertEqual(evidence['reason'],'known_free_bypass_within_current_place')

    def test_unproven_return_cannot_replace_frozen_source(self):
        for condition in ('not_returned','destination_side','outside_parent','far_from_source'):
            with self.subTest(condition=condition):
                node,auth=self.fixture()
                if condition=='not_returned':auth.return_gate_cleared=False
                if condition=='destination_side':node.pose_odom.x=5.3
                if condition=='outside_parent':node.current_place_scope_mask[:]=False
                if condition=='far_from_source':node.pose_odom.x=4.0
                evidence=authenticated_place_portal_evidence(node,auth)
                self.assertEqual(evidence['reason'],'slam_evidence_incomplete')
                self.assertEqual(evidence['source_label'],0)

if __name__=='__main__':unittest.main()
