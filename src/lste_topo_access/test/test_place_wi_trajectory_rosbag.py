"""Independent raw-pose audit must catch ordinary cross-Place motion."""
import json
from pathlib import Path
import sys,tempfile,unittest
import genpy
import numpy as np
import rosbag
from geometry_msgs.msg import Pose2D,TransformStamped
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'scripts/tests/targeted_navigation'))
from audit_place_wi_trajectory import audit


class ActualTrajectoryAuditTest(unittest.TestCase):
    def run_case(self, outside=False, crossing=False, auth=False, hole=False, short_routes=False, missing_route=False, unbound=False, omit_tree=False):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);bag_path=root/'raw.bag';metrics=root/'metrics.log'
            grid=np.full((12,24),100,dtype=int);grid[3:7,1:23]=0
            if hole:grid[4:6,14:16]=100
            message=OccupancyGrid();message.header.stamp=genpy.Time.from_sec(1.)
            message.info.width=24;message.info.height=12;message.info.resolution=1.
            message.info.origin.orientation.w=1.;message.data=grid.ravel().tolist()
            portal=dict(id=7,gate=[8.5,5.],destination=[12.5,5.],width=4.,bounds=[[8.5,3.],[8.5,7.]],
                        source_place_id=1,destination_place_id=2)
            scope=dict(valid=True,place_id=2,map_stamp=1.,seed_map_xy=[12.5,5.5],entry_portal_id=7,projected_portals=[portal])
            data=dict(ros_time=1.05,active=True,route_id=9,frontier_event='work_item_dispatched',place_id=2,
                      goal_context=dict(goal_role='place_observation_work',owner_place_id=2),
                      place_scope=scope,portal_authentication=dict(active_branch_id=None),
                      portal_hypotheses=dict(records=[dict(portal,gp_authentication_state='completed',crossing_count=1)]))
            events=[data]
            if unbound:
                portal.update(destination_place_id=None)
                scope.update(place_id=1,seed_map_xy=[3.5,5.5],entry_portal_id=None)
                data.update(place_id=1,goal_context=dict(goal_role='place_observation_work',owner_place_id=1))
                data['portal_hypotheses']=dict(records=[dict(portal,state='certified',
                    gp_authentication_state='completed',crossing_count=0,gp_authentication_branch_id=4,
                    opening_width=4.,physical_bounds=[[8.5,3.],[8.5,7.]])])
                evidence=dict(state='completed',source_side_proven=True,return_gate_cleared=True,
                    events=[dict(event=e) for e in ('doorway_entry_started','vehicle_fully_inside',
                        'inside_frontier_observed','room_facing_turn_complete','returned_to_junction_node')])
                if not omit_tree:
                    events.insert(0,dict(ros_time=1.01,tree=dict(branches=[dict(id=4,canonical_portal_id=7,
                        authentication_state='completed',authentication_evidence=evidence)])))
            if crossing or auth:
                changed=dict(data,ros_time=1.4)
                if crossing:changed['goal_context']=dict(goal_role='certified_portal_crossing',owner_place_id=2,portal_crossing_certified=True)
                if auth:changed['portal_authentication']=dict(active_branch_id=3)
                events.append(changed)
            metrics.write_text(''.join('event=global_frontier_event data='+json.dumps(e)+'\n' for e in events))
            poses=[(1.1,12.5,5.5),(1.3,13.5,5.5),(1.5,4.5 if outside else 17.5 if hole else 13.8,5.5)]
            if unbound:poses=[(1.1,3.5,5.5),(1.3,4.5,5.5),(1.5,5.5,5.5)]
            if missing_route:
                events=[data,dict(data,ros_time=1.14,route_id=10),
                        dict(data,ros_time=1.15,frontier_event='frontier_execution_state')]
                metrics.write_text(''.join('event=global_frontier_event data='+json.dumps(e)+'\n' for e in events))
            if short_routes:
                events=[dict(data,ros_time=1.05,route_id=9),dict(data,ros_time=1.12,route_id=10),
                        dict(data,ros_time=1.15,route_id=8,active_execution_route_id=10,
                             frontier_event='controller_lease_release_acknowledged')]
                metrics.write_text(''.join('event=global_frontier_event data='+json.dumps(e)+'\n' for e in events))
                poses=[(1.1,12.5,5.5),(1.13,13.5,5.5),(1.16,13.8,5.5)]
            with rosbag.Bag(str(bag_path),'w') as bag:
                bag.write('/map',message,genpy.Time.from_sec(1.))
                for time in (1.,1.2,1.4,1.6):
                    tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom'
                    tf.header.stamp=genpy.Time.from_sec(time);tf.transform.rotation.w=1.
                    bag.write('/tf',TFMessage(transforms=[tf]),genpy.Time.from_sec(time))
                for time,x,y in poses:bag.write('/rbt_pose',Pose2D(x=x,y=y),genpy.Time.from_sec(time))
            return audit(metrics,bag_path)

    def test_measured_child_route_inside_its_place_passes(self):
        result=self.run_case();self.assertTrue(result['passed'],result);self.assertEqual(result['checked_pose_samples'],3)
        self.assertEqual(result['checked_dispatch_route_count'],1)

    def test_short_dispatched_route_is_sampled_and_old_ack_does_not_rename_it(self):
        result=self.run_case(short_routes=True)
        self.assertTrue(result['passed'],result)
        self.assertEqual(result['dispatch_route_count'],2)
        self.assertEqual(result['checked_dispatch_route_count'],2)
        self.assertEqual(result['unobserved_dispatch_routes'],[])

    def test_missing_route_samples_fail_even_when_another_route_in_place_is_covered(self):
        result=self.run_case(missing_route=True)
        self.assertFalse(result['passed'])
        self.assertEqual(result['unobserved_dispatch_routes'],[10])
        self.assertTrue(any('dispatched route' in e['reason'] for e in result['errors']))

    def test_unbound_scope_uses_preceding_full_certification_but_not_missing_proof(self):
        result=self.run_case(unbound=True)
        self.assertTrue(result['passed'],result)
        self.assertEqual(result['sample_period_s'],0.)
        result=self.run_case(unbound=True,omit_tree=True)
        self.assertFalse(result['passed'])

    def test_ordinary_motion_across_entry_is_rejected(self):
        result=self.run_case(outside=True);self.assertFalse(result['passed'])
        self.assertIn('outside current Place',result['errors'][0]['reason'])

    def test_authorized_crossing_and_authentication_are_distinct_transactions(self):
        for kwargs in (dict(crossing=True),dict(auth=True)):
            result=self.run_case(outside=True,**kwargs);self.assertTrue(result['passed'],result)
            self.assertEqual(result['checked_pose_samples'],2)

    def test_interpolation_catches_path_leak_with_both_sampled_endpoints_inside(self):
        result=self.run_case(hole=True);self.assertFalse(result['passed'])
        self.assertTrue(any('outside current Place' in e['reason'] for e in result['errors']))

if __name__=='__main__':unittest.main()
