"""Actual costmap cells must enforce the Place before ordinary WI dispatch."""
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import patch
import sys,unittest
import numpy as np
import genpy
from nav_msgs.msg import OccupancyGrid
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from global_frontier_place_navigation import constraint_cells,costmap_contains_constraint,PlaceNavigationConstraints


def grid(data,frame='map',origin=(0.,0.),stamp=11.):
    data=np.asarray(data);m=OccupancyGrid();m.header.frame_id=frame;m.header.stamp=genpy.Time.from_sec(stamp)
    m.info.width=data.shape[1];m.info.height=data.shape[0];m.info.resolution=1.
    m.info.origin.position.x,m.info.origin.position.y=origin;m.info.origin.orientation.w=1.
    m.data=data.ravel().tolist();return m


class PlaceNavigationConstraintTest(unittest.TestCase):
    def fixture(self):
        allowed=np.zeros((4,6),dtype=bool);allowed[:,3:]=True
        message=grid(np.zeros((4,6),dtype=int))
        node=S(current_place_scope_message=message,current_place_scope_mask=allowed,
               current_place_scope_valid=True,current_place_scope_id=2,current_physical_place_id=2,
               map_msg=message,pose_odom=S(x=4.5,y=1.5),transform_xy=lambda t,s,x,y:(x,y),
               active_portal_authentication=lambda:None)
        controller=PlaceNavigationConstraints.__new__(PlaceNavigationConstraints)
        self.published=[];controller.publisher=S(publish=self.published.append)
        controller.message=None;controller.allowed=None;controller.key=None;controller.sequence=0
        controller.local_message=None;controller.local_allowed=None;controller.local_publisher=S(publish=lambda m:None)
        node._place_partition_tf_key=('map',11.);node._place_partition_projector=lambda x,y:(x,y)
        controller.last_report={};controller.bounded=False;controller.local=grid(np.zeros((4,6),dtype=int),'odom')
        node.costmap_msg=grid(np.zeros((4,6),dtype=int))
        return controller,node,allowed

    def test_ordinary_waits_for_both_real_costmaps_and_preserves_physical_obstacles(self):
        c,node,allowed=self.fixture()
        with patch('global_frontier_place_navigation.rospy.Time.now',return_value=genpy.Time(10,0)):
            self.assertFalse(c.prepare_candidate(node,S(route_kind='frontier_endpoint')))
            actual=np.where(allowed,0,100);actual[1,4]=100
            node.costmap_msg=grid(actual)
            self.assertFalse(c.ready(node))
            c.local=grid(actual,'odom')
            self.assertTrue(c.ready(node))
        published=np.asarray(self.published[-1].data).reshape(4,6)
        self.assertTrue(np.all(published[:,3:]==-1))
        self.assertTrue(np.all(published[:,:3]==100))
        self.assertEqual(node.costmap_msg.data[10],100)

    def test_handoff_scope_must_reproject_before_ordinary_dispatch(self):
        from global_frontier_portal_arrival_commit import GlobalFrontierPortalArrivalCommitMixin
        c,node,allowed=self.fixture()
        actual=np.where(allowed,0,100)
        node.costmap_msg=grid(actual);c.local=grid(actual,'odom')
        with patch('global_frontier_place_navigation.rospy.Time.now',return_value=genpy.Time(10,0)):
            self.assertTrue(c.prepare_candidate(node,S(route_kind='frontier_endpoint')))
            GlobalFrontierPortalArrivalCommitMixin._invalidate_place_scope_for_handoff(node,3)
            self.assertFalse(c.prepare_candidate(node,S(route_kind='frontier_endpoint')))
            self.assertEqual(c.last_report['reason'],'place_scope_pending')
            self.assertEqual(node.place_entry_rehydration_pending,3)
            # Even validity alone cannot authorize the predecessor's owner.
            node.current_physical_place_id=3;node.current_place_scope_valid=True
            self.assertFalse(c.prepare_candidate(node,S(route_kind='frontier_endpoint')))
            node.current_place_scope_id=3
            self.assertTrue(c.prepare_candidate(node,S(route_kind='frontier_endpoint')))

    def test_authentication_clears_only_constraint_and_does_not_modify_slam(self):
        c,node,allowed=self.fixture();before=list(node.map_msg.data)
        node.active_portal_authentication=lambda:S(state='returning')
        with patch('global_frontier_place_navigation.rospy.Time.now',return_value=genpy.Time(10,0)):
            self.assertTrue(c.set_mode(node,True))
        self.assertFalse(c.bounded)
        self.assertTrue(all(v==-1 for v in self.published[-1].data))
        self.assertEqual(node.map_msg.data,before)

    def test_transparent_mode_waits_for_owned_ordinary_release_ack(self):
        c,node,allowed=self.fixture()
        with patch('global_frontier_place_navigation.rospy.Time.now',return_value=genpy.Time(10,0)):
            self.assertTrue(c.set_mode(node,True))
            before=list(self.published[-1].data)
            node.active_portal_authentication=lambda:S(state='entering')
            node.awaiting_controller_lease_release_ack=True
            node.pending_controller_lease_releases={'old':object()}
            self.assertFalse(c.set_mode(node,False));self.assertTrue(c.bounded)
            self.assertEqual(self.published[-1].data,before)
            node.awaiting_controller_lease_release_ack=False
            self.assertFalse(c.set_mode(node,False))
            node.pending_controller_lease_releases={}
            self.assertTrue(c.set_mode(node,False));self.assertFalse(c.bounded)
            self.assertTrue(all(v==-1 for v in self.published[-1].data))

    def test_bounded_mode_waits_for_owned_auth_return_release_ack(self):
        c,node,allowed=self.fixture()
        node.active_portal_authentication=lambda:None
        node.awaiting_controller_lease_release_ack=True
        with patch('global_frontier_place_navigation.rospy.Time.now',return_value=genpy.Time(10,0)):
            self.assertFalse(c.set_mode(node,True));self.assertFalse(c.bounded)
            self.assertEqual(self.published,[])
            node.awaiting_controller_lease_release_ack=False
            self.assertTrue(c.set_mode(node,True));self.assertTrue(c.bounded)

    def test_graph_selected_portal_keeps_authorized_crossing_for_tuple_projection(self):
        c,node,allowed=self.fixture();node.last_graph_route_plan=S(action='cross_portal')
        with patch('global_frontier_place_navigation.rospy.Time.now',return_value=genpy.Time(10,0)):
            self.assertTrue(c.prepare_candidate(node,(1,2,4.5,1.5)))
        self.assertFalse(c.bounded)
        self.assertTrue(all(v==-1 for v in self.published[-1].data))

    def test_robot_outside_fixed_entrance_component_cannot_start_ordinary_work(self):
        c,node,allowed=self.fixture();node.pose_odom.x=1.5
        self.assertFalse(c.set_mode(node,True))
        self.assertFalse(c.last_report['robot_inside'])
        self.assertEqual(self.published,[])

    def test_new_map_stamp_with_identical_masks_refreshes_without_releasing_active_route(self):
        c,node,allowed=self.fixture();releases=[];suspended=[]
        with patch("global_frontier_place_navigation.rospy.Time.now",return_value=genpy.Time(10,0)):
            self.assertTrue(c.set_mode(node,True))
            old_global=list(c.message.data);old_local=list(c.local_message.data)
            node.active_route_id=7;node.active_frontier=(1,2,3.,4.)
            node.suspend_active_work_item=lambda *a:suspended.append(a)
            node.release_active_frontier=lambda **fields:releases.append(fields)
            node.current_place_scope_message=grid(np.zeros((4,6),dtype=int),stamp=12.)
            node._place_partition_tf_key=("map",12.)
            self.assertTrue(c.set_mode(node,True))
        self.assertEqual(releases,[]);self.assertEqual(suspended,[])
        self.assertEqual(c.sequence,2)
        self.assertEqual(c.message.data,old_global);self.assertEqual(c.local_message.data,old_local)
        self.assertEqual(c.message.info.map_load_time,genpy.Time.from_sec(12.))

    def test_scope_revision_retains_old_masks_until_exact_release_ack(self):
        c,node,allowed=self.fixture();releases=[];suspended=[]
        with patch('global_frontier_place_navigation.rospy.Time.now',return_value=genpy.Time(10,0)):
            self.assertTrue(c.set_mode(node,True));old=list(c.message.data);old_local=list(c.local_message.data)
            node.active_route_id=7;node.active_frontier=(1,2,3.,4.)
            node.suspend_active_work_item=lambda *a:suspended.append(a)
            def release(**fields):
                releases.append(fields);node.active_frontier=None;node.awaiting_controller_lease_release_ack=True
            node.release_active_frontier=release
            node.current_place_scope_message=grid(np.zeros((4,6),dtype=int),stamp=12.)
            node.current_place_scope_mask=allowed.copy();node.current_place_scope_mask[1,3]=False
            node._place_partition_tf_key=('map',12.)
            self.assertFalse(c.set_mode(node,True));self.assertEqual(len(releases),1);self.assertEqual(len(suspended),1)
            self.assertEqual(c.message.data,old);self.assertEqual(c.local_message.data,old_local)
            self.assertEqual(c.sequence,1);self.assertTrue(releases[0]['await_controller_release_ack'])
            c.update_scope(node);self.assertEqual(c.sequence,1)
            node.awaiting_controller_lease_release_ack=False;node.pending_controller_lease_releases={'old':object()}
            self.assertFalse(c.set_mode(node,True));self.assertEqual(c.sequence,1)
            node.pending_controller_lease_releases={};node.active_route_id=0
            self.assertTrue(c.set_mode(node,True));self.assertEqual(c.sequence,2)

    def test_constructor_exposes_two_frame_specific_latched_inputs(self):
        with patch('global_frontier_place_navigation.rospy.Publisher') as pub,patch('global_frontier_place_navigation.rospy.Subscriber'):
            PlaceNavigationConstraints()
        self.assertEqual([c.args[0] for c in pub.call_args_list],['/lste/place_navigation_mask','/lste/place_navigation_mask_odom'])
        self.assertTrue(all(c.kwargs['latch'] for c in pub.call_args_list))
    def test_transparent_bootstrap_initializes_without_any_tf_or_spatial_permission_claim(self):
        c,node,allowed=self.fixture();node._place_partition_tf_key=('map',1.)
        node.planar_xy_projector=lambda *a,**k:(_ for _ in ()).throw(AssertionError('transparent bootstrap must not wait for TF'))
        with patch('global_frontier_place_navigation.rospy.Time.now',return_value=genpy.Time(10,0)):
            c.bootstrap(node.map_msg,node)
        self.assertEqual(len(self.published),1);self.assertEqual(c.local_message.header.frame_id,'odom')
        self.assertEqual((c.local_message.info.width,c.local_message.info.height),(1,1))
        self.assertTrue(all(v==-1 for v in c.local_message.data))
    def test_authorized_transparency_clears_the_previous_whole_physical_canvas_without_tf(self):
        c,node,allowed=self.fixture()
        with patch('global_frontier_place_navigation.rospy.Time.now',return_value=genpy.Time(10,0)):
            self.assertTrue(c.set_mode(node,True));old=c.local_message
            node._place_partition_tf_key=('map',1.)
            node.planar_xy_projector=lambda *a,**k:(_ for _ in ()).throw(AssertionError('transparent mode performs no projection'))
            self.assertTrue(c.set_mode(node,False))
        self.assertEqual((c.local_message.info.width,c.local_message.info.height),(old.info.width,old.info.height))
        self.assertEqual(c.local_message.info.origin,old.info.origin)
        self.assertTrue(all(v==-1 for v in c.local_message.data))

    def test_rolling_odom_grid_is_checked_in_map_frame(self):
        c,node,allowed=self.fixture()
        values=np.zeros((6,4),dtype=int);values[:3,:]=100
        local=grid(values,'odom',(-4.,0.))
        project=lambda x,y:(y,-x)
        self.assertEqual(costmap_contains_constraint(local,node.map_msg,allowed,project),(True,12,0))
        local.data[0]=0
        self.assertEqual(costmap_contains_constraint(local,node.map_msg,allowed,project),(False,12,1))

if __name__=='__main__':unittest.main()
