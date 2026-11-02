"""Exercise the actual bag reader, ordinary dispatch coverage and mode timeline."""
import json,tempfile,unittest,sys
from pathlib import Path
import genpy,rosbag
from map_msgs.msg import OccupancyGridUpdate
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'scripts/tests/targeted_navigation'))
from audit_place_native_costmaps import audit,MASTERS,MASK
from test_native_costmap_replay import grid
class NativeBagTest(unittest.TestCase):
    def run_recording(self,tmp,violate=False,odom_frame=False,future_tf_only=False,inactive_tail=False,truncate_tail=False,late_physical_start=False,transient_clear=False):
        root=Path(tmp);metrics=root/'metrics.log';bagpath=root/'raw.bag'
        events=[dict(ros_time=0.,active=False),dict(ros_time=2.,active=True,
            frontier_event='work_item_dispatched',route_id=9,active_execution_route_id=9,action_generation=7,active_execution_route_kind='frontier_endpoint'),
            dict(ros_time=4.,active=False,pending=False,active_execution_route_id=9,action_generation=7,
                 released_controller_route=dict(route_id=9,transaction_id=9,graph_transaction_id=20,
                     lifecycle_transaction_id=30,action_generation=7,map_epoch=4,origin_epoch=4,projection_epoch=4,
                     controller_pending=False,terminal_received=False),
                 last_controller_lease_release_ack=dict(route_id=9,transaction_id=9,graph_transaction_id=20,
                     lifecycle_transaction_id=30,action_generation=7,map_epoch=4,origin_epoch=4,projection_epoch=4)),
            dict(ros_time=4.2,active=True,active_execution_route_kind='frontier_turn_connector',
                 portal_authentication=dict(active_branch_id=5)),dict(ros_time=6.,active=False)]
        if inactive_tail:
            events.append(dict(ros_time=2.5,active=False,pending=False,
                active_execution_route_id=9,action_generation=7,active_execution_route_kind='frontier_endpoint',
                awaiting_controller_lease_release_ack=True,
                portal_authentication=dict(active_branch_id=5),
                released_controller_route=dict(route_id=9,action_generation=7,controller_pending=True,terminal_received=False)))
            events.sort(key=lambda d:d['ros_time'])
        metrics.write_text(''.join('event=global_frontier_event data='+json.dumps(d)+'\n'for d in events))
        def master(data,stamp):
            m=grid(data,stamp=stamp,origin=(-2.,0.) if odom_frame else (0.,0.))
            if odom_frame:m.header.frame_id='odom'
            return m
        with rosbag.Bag(str(bagpath),'w')as bag:
            if odom_frame:
                from geometry_msgs.msg import TransformStamped
                from tf2_msgs.msg import TFMessage
                for stamp in ((10.,) if future_tf_only else (1.,6.)):
                    tr=TransformStamped();tr.header.frame_id='map';tr.child_frame_id='odom'
                    tr.header.stamp=genpy.Time.from_sec(stamp);tr.transform.rotation.w=1.
                    tr.transform.translation.x=2.
                    bag.write('/tf',TFMessage([tr]),genpy.Time.from_sec(stamp))
            from std_msgs.msg import String
            from geometry_msgs.msg import Pose2D
            from nav_msgs.msg import Odometry
            bag.write('/lste/global_frontier/status',String(json.dumps(events[0])),genpy.Time.from_sec(.5))
            if not late_physical_start:
                for topic,cls in (('/rbt_pose',Pose2D),('/pro3/wheel_odom',Odometry)):
                    pose=cls()
                    if hasattr(pose,'header'):pose.header.stamp=genpy.Time.from_sec(.5);pose.header.frame_id='odom'
                    bag.write(topic,pose,genpy.Time.from_sec(.5))
            bag.write(MASK,grid([[100,-1]],stamp=1.),genpy.Time.from_sec(1.))
            for topic in MASTERS:bag.write(topic,master([[100,0]],1.5),genpy.Time.from_sec(1.5))
            if transient_clear:
                bag.write(MASK,grid([[-1,-1]],stamp=2.5),genpy.Time.from_sec(2.5))
                bag.write(MASK,grid([[100,-1]],stamp=2.8),genpy.Time.from_sec(2.8))
            u=OccupancyGridUpdate();u.header.frame_id='odom' if odom_frame else 'map';u.header.stamp=genpy.Time.from_sec(3.)
            u.width=u.height=1;u.x=u.y=0;u.data=[0 if violate else 100]
            for topic in MASTERS:bag.write(topic+'_updates',u,genpy.Time.from_sec(3.))
            bag.write(MASK,grid([[-1,-1]],stamp=4.1),genpy.Time.from_sec(4.1))
            for topic in MASTERS:bag.write(topic,master([[0,100]],5.),genpy.Time.from_sec(5.))
            if not truncate_tail:
                from std_msgs.msg import String
                bag.write('/lste/global_frontier/status',String(json.dumps(events[-1])),genpy.Time.from_sec(6.))
                from geometry_msgs.msg import Pose2D
                from nav_msgs.msg import Odometry
                for topic,cls in (('/rbt_pose',Pose2D),('/pro3/wheel_odom',Odometry)):
                    pose=cls()
                    if hasattr(pose,'header'):pose.header.stamp=genpy.Time.from_sec(6.);pose.header.frame_id='odom'
                    bag.write(topic,pose,genpy.Time.from_sec(6.))
        return audit(metrics,bagpath)
    def test_full_update_dispatch_and_transparent_auth_are_replayed(self):
        with tempfile.TemporaryDirectory()as tmp:r=self.run_recording(tmp)
        self.assertTrue(r['passed'],r);self.assertEqual(r['dispatch_routes'],1)
        self.assertEqual(r['transparent_mask_checks'],2);self.assertFalse(r['uncovered_dispatches'])
    def test_dynamic_original_tf_is_used_for_rolling_odom_master(self):
        with tempfile.TemporaryDirectory()as tmp:r=self.run_recording(tmp,odom_frame=True)
        self.assertTrue(r['passed'],r)
    def test_future_only_tf_cannot_be_used_as_latest_substitute(self):
        with tempfile.TemporaryDirectory()as tmp:r=self.run_recording(tmp,odom_frame=True,future_tf_only=True)
        self.assertFalse(r['passed'])
        self.assertTrue(any('extrapolation' in e['reason'].lower() for e in r['errors']),r)
    def test_inactive_old_wi_lease_cannot_be_exempted_by_new_auth_task(self):
        with tempfile.TemporaryDirectory()as tmp:r=self.run_recording(tmp,violate=True,inactive_tail=True)
        self.assertFalse(r['passed'])
        self.assertTrue(any('nonlethal' in e['reason']for e in r['errors']),r)
    def test_bag_ending_before_owned_execution_end_cannot_pass(self):
        with tempfile.TemporaryDirectory()as tmp:r=self.run_recording(tmp,truncate_tail=True)
        self.assertFalse(r['passed'])
    def test_late_pose_odom_start_cannot_be_hidden_by_earlier_other_topics(self):
        with tempfile.TemporaryDirectory()as tmp:r=self.run_recording(tmp,late_physical_start=True)
        self.assertFalse(r['passed'])
    def test_transient_wrong_mask_between_master_publications_is_rejected(self):
        with tempfile.TemporaryDirectory()as tmp:r=self.run_recording(tmp,transient_clear=True)
        self.assertFalse(r['passed'])
        self.assertTrue(any('published mask mode' in e['reason']for e in r['errors']),r)
    def test_native_incremental_clear_cannot_be_hidden_by_valid_initial_full_grid(self):
        with tempfile.TemporaryDirectory()as tmp:r=self.run_recording(tmp,True)
        self.assertFalse(r['passed']);self.assertTrue(any('nonlethal' in e['reason']for e in r['errors']))
class PhysicalNativeBagTest(unittest.TestCase):
    def record(self,tmp,bad_mask=False,future_tf=False,missing_epoch=False):
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage
        from audit_place_native_costmaps import LOCAL_MASK
        NativeBagTest().run_recording(tmp)
        root=Path(tmp);records=[]
        with rosbag.Bag(str(root/'raw.bag')) as old:
            for topic,m,t in old.read_messages():
                if topic==MASK:
                    import copy
                    m.info.map_load_time=genpy.Time.from_sec(.5 if t.to_sec()<2 else 4.)
                    records.append((topic,m,t));local=copy.deepcopy(m);local.header.frame_id='odom';local.info.origin.position.x=-2.
                    if bad_mask and t.to_sec()<2:local.data=[0,-1]
                    if missing_epoch:local.info.map_load_time=genpy.Time()
                    records.append((LOCAL_MASK,local,t));continue
                if topic.startswith(MASTERS[1]):
                    m.header.frame_id='odom'
                    if hasattr(m,'info'):m.info.origin.position.x=-2.
                records.append((topic,m,t))
        for stamp in ((10.,) if future_tf else (.5,6.)):
            tr=TransformStamped();tr.header.frame_id='map';tr.child_frame_id='odom';tr.header.stamp=genpy.Time.from_sec(stamp)
            tr.transform.rotation.w=1.;tr.transform.translation.x=2.;records.append(('/tf',TFMessage([tr]),tr.header.stamp))
        path=root/'physical.bag'
        with rosbag.Bag(str(path),'w') as bag:
            for topic,m,t in sorted(records,key=lambda r:r[2].to_sec()):bag.write(topic,m,t)
        return audit(root/'metrics.log',path)
    def test_dual_native_frames_use_original_source_epoch(self):
        with tempfile.TemporaryDirectory() as tmp:
            result=self.record(tmp);self.assertTrue(result['passed'],result['errors']);self.assertIn('physical mask',result['mask_protocol'])
    def test_local_render_cannot_admit_source_exclusion(self):
        with tempfile.TemporaryDirectory() as tmp:
            result=self.record(tmp,bad_mask=True);self.assertFalse(result['passed']);self.assertTrue(any('physical mask admits' in e['reason'] for e in result['errors']))
    def test_missing_source_epoch_or_future_only_tf_cannot_pass(self):
        for field in ('missing_epoch','future_tf'):
            with self.subTest(field=field),tempfile.TemporaryDirectory() as tmp:
                result=self.record(tmp,**{field:True});self.assertFalse(result['passed'])

if __name__=='__main__':unittest.main()
