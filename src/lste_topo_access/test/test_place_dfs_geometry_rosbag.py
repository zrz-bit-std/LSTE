"""Run separately with ROS installed: verify raw-bag inside frontier accounting."""
from pathlib import Path
import sys,tempfile,json,unittest
import numpy as np
import rosbag
import genpy
from nav_msgs.msg import OccupancyGrid
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'scripts/tests/targeted_navigation'))
from audit_place_dfs_geometry import audit


class InsideFrontierBagAuditTest(unittest.TestCase):
    def test_reconstructs_component_and_rejects_inflated_other_room_count(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);bag_path=root/'map.bag';metrics=root/'metrics.log'
            grid=np.full((12,24),100,dtype=int);grid[3:7,1:16]=0;grid[3:7,16]=-1
            message=OccupancyGrid();message.header.stamp=genpy.Time(1,0);message.header.frame_id='map'
            message.info.width=24;message.info.height=12;message.info.resolution=1.
            message.info.origin.orientation.w=1.;message.data=grid.ravel().tolist()
            with rosbag.Bag(str(bag_path),'w') as bag:bag.write('/map',message,genpy.Time(1,0))
            scope=dict(place_id=2,valid=True,map_stamp=1.,seed_map_xy=[12.5,5.5],
                       gate_map_xy=[8.5,5.],heading_map=0.,projected_portals=[
                           dict(id=1,gate=[8.5,5.],destination=[12.5,5.],width=4.,bounds=[[8.5,3.],[8.5,7.]])])
            inside=dict(frontier_event='portal_authentication_inside_evidence',branch_id=5,state='turning',
                        inside_frontier_scope=scope,left_frontier_cells=2,right_frontier_cells=2)
            dispatch=dict(frontier_event='work_item_dispatched',place_id=2,work_item_id=8,
                          place_scope=scope,place_membership=dict(viewpoint_map=[12.5,5.5]))
            def write():metrics.write_text(''.join('event=global_frontier_event data='+json.dumps(d)+'\n' for d in (inside,dispatch)))
            write();result=audit(str(metrics),str(bag_path));self.assertTrue(result['passed'],result)
            self.assertEqual(result['inside_frontier_checks'][0]['left'],2)
            inside['left_frontier_cells']=3;write();result=audit(str(metrics),str(bag_path))
            self.assertFalse(result['passed'])
            self.assertIn('inside frontier evidence differs',result['errors'][0]['reason'])
            inside['left_frontier_cells']=2
            scope.update(frontier_count=4,pending_frontier_count=0,frontier_work_evidence=dict(
                map_stamp=1.,raw_count=4,pending_count=0,resolved_count=4,
                resolved_frontier_cells=[[r,15,8] for r in range(3,7)]))
            complete=dict(frontier_event='place_dfs_completion_ready',place_id=2,place_scope=scope,
                          work_items=[dict(id=8,place_id=2,state='resolved')])
            def closure_write():metrics.write_text(''.join('event=global_frontier_event data='+json.dumps(d)+'\n' for d in (inside,dispatch,complete)))
            closure_write();result=audit(str(metrics),str(bag_path));self.assertTrue(result['passed'],result)
            scope['frontier_work_evidence']['resolved_frontier_cells'][0]=[3,14,8]
            closure_write();result=audit(str(metrics),str(bag_path));self.assertFalse(result['passed'])
            self.assertTrue(any('completion residual' in e['reason'] for e in result['errors']))

class PhysicalHandoffBagTest(unittest.TestCase):
    def test_handoff_uses_raw_pose_and_does_not_accept_runtime_crossed_label_alone(self):
        from geometry_msgs.msg import Pose2D
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);bag_path=root/'raw.bag';metrics=root/'metrics.log'
            portal=dict(id=7,physical_gate=[0.,0.],physical_destination=[2.,0.],
                        physical_bounds=[[0.,-1.],[0.,1.]],gp_authentication_required_depth=.75,
                        source_place_id=1,destination_place_id=2,state='crossed',crossing_count=1)
            event=dict(frontier_event='place_dfs_entered',ros_time=1.1,portal_id=7,place_id=2,
                       portal_hypotheses=dict(records=[portal]))
            metrics.write_text('event=global_frontier_event data='+json.dumps(event)+'\n')
            for x,expected in ((.4,False),(.8,True)):
                with rosbag.Bag(str(bag_path),'w') as bag:
                    bag.write('/rbt_pose',Pose2D(x=x,y=0.),genpy.Time.from_sec(1.05))
                result=audit(str(metrics),str(bag_path))
                self.assertEqual(result['physical_handoff_checks'][0]['verified'],expected)

if __name__=='__main__':unittest.main()
