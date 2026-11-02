"""Certification/dedup witnesses share the original raster's immutable TF."""
import math,sys,unittest
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
import genpy
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from global_frontier_portal_authentication import DoorwayAuthentication
from global_frontier_place_portal_validation import (
    _validation_snapshot_projector,authenticated_place_portal_evidence,
    authentication_inside_frontiers,duplicate_observation_portal)
from global_frontier_grid import frontier_mask
class ValidationSnapshotTest(unittest.TestCase):
    def fixture(self):
        free=np.zeros((10,30),dtype=bool);free[3:6,:]=True
        message=S(header=S(frame_id='map',stamp=genpy.Time.from_sec(1.)),
                  info=S(resolution=1.,origin=S(position=S(x=0.,y=0.))))
        context=S(known_free=free,unknown=~free,components=S(epoch=7))
        calls=[];updates=[]
        def exact(x,y):calls.append((x,y));return(x+2.,y)
        def latest(*args):raise AssertionError('latest TF cannot evaluate a stamped raster')
        node=S(_place_partition_tf_key=('map',1.),_place_partition_projector=exact,
               transform_xy=latest,current_physical_place_id=1,
               current_place_scope_projected_portals=[],
               _update_place_partition=lambda *args:updates.append(args),
               active_semantic_room=dict(portal_id=1),
               portal_authentication_map_context=context,portal_authentication_map_message=message,
               gp_exploration_tree=S(branches={7:dict(map_geometry=dict(width=4.,physical_bounds=[[8.5,2.5],[8.5,6.5]]))}),
               portal_hypothesis_ledger=S(get=lambda key:dict(physical_destination=[2.5,4.5])))
        auth=DoorwayAuthentication(7,1,(8.5,4.5),(10.9,4.5),gate_geometry=(4.,[(8.5,2.5),(8.5,6.5)]))
        auth.source_xy=(5.5,4.5);auth.inside_xy=(10.5,4.5)
        return node,auth,message,context,calls,updates
    def test_completed_validation_projects_all_witnesses_with_one_cached_map_tf(self):
        n,a,m,c,calls,updates=self.fixture()
        result=authenticated_place_portal_evidence(n,a)
        self.assertEqual(result['reason'],'distinct_destination_component')
        self.assertEqual(updates[0][4],(7.5,4.5))
        for point in (a.source_xy,a.inside_xy,a.gate_xy,a.destination_xy,(2.5,4.5),(8.5,2.5),(8.5,6.5)):
            self.assertIn(tuple(point),calls)
    def test_inside_frontier_component_uses_same_stamp_as_gate_and_seed(self):
        n,a,m,c,calls,updates=self.fixture()
        mask,evidence=authentication_inside_frontiers(n,n.gp_exploration_tree,a,m,c,frontier_mask(c.known_free,c.unknown))
        self.assertIsNotNone(mask);self.assertEqual(evidence['seed_map_xy'],[12.5,4.5])
        self.assertEqual(evidence['gate_map_xy'],[10.5,4.5]);self.assertEqual(evidence['map_stamp'],1.)
    def test_parent_alias_uses_cached_transform_even_with_newer_live_tf(self):
        n,a,m,c,calls,updates=self.fixture();n.current_physical_place_id=2
        portal=dict(id=9,source_place_id=1,destination_place_id=2,state='crossed',crossing_count=1,gp_authentication_state='completed')
        n.portal_hypothesis_ledger=S(get=lambda key:portal)
        n.current_place_scope_projected_portals=[dict(portal,gate=(10.5,4.5),destination=(12.5,4.5),width=4.,bounds=[(10.5,2.5),(10.5,6.5)])]
        tree=S(nodes={1:dict(physical_xy=(15.5,4.5))})
        branch=dict(node_id=1,heading=math.pi,physical_gate=(8.1,4.5),map_geometry=dict(width=4.,physical_bounds=[(8.1,2.5),(8.1,6.5)]))
        result=duplicate_observation_portal(n,tree,branch,m,c)
        self.assertEqual(result['id'],9);self.assertEqual(branch['connection'],'return')
    def test_same_map_cache_survives_tf_history_eviction_without_lookup(self):
        n,a,m,c,calls,updates=self.fixture()
        n.planar_xy_projector=lambda *args,**kwargs:(_ for _ in ()).throw(AssertionError('cached snapshot must not relookup'))
        self.assertIs(_validation_snapshot_projector(n,m,c,a.source_xy),n._place_partition_projector)
    def test_new_map_requests_exact_stamp_and_refuses_old_cache_on_tf_miss(self):
        n,a,m,c,calls,updates=self.fixture();n._place_partition_tf_key=('map',.5);requests=[]
        def acquire(target,source,stamp=None):requests.append((target,source,stamp.to_sec()));return None
        n.planar_xy_projector=acquire
        self.assertIsNone(_validation_snapshot_projector(n,m,c,a.source_xy))
        self.assertEqual(requests,[('map','odom',1.)]);self.assertEqual(updates,[])
        self.assertEqual(n._place_partition_tf_key,('map',.5))
    def test_new_exact_snapshot_must_be_committed_by_partition_before_use(self):
        n,a,m,c,calls,updates=self.fixture();n._place_partition_tf_key=('map',.5)
        exact=lambda x,y:(x+2.,y);n.planar_xy_projector=lambda target,source,stamp=None:exact
        self.assertIsNone(_validation_snapshot_projector(n,m,c,a.source_xy))
        def update(*args):n._place_partition_tf_key=('map',1.);n._place_partition_projector=exact
        n._update_place_partition=update
        self.assertIs(_validation_snapshot_projector(n,m,c,a.source_xy),exact)
class SnapshotInverseTest(unittest.TestCase):
    def test_inverse_uses_same_rotated_translated_snapshot(self):
        from global_frontier_geometry import inverse_planar_projector
        project=lambda x,y:(4.-y, -3.+x)
        inverse=inverse_planar_projector(project)
        self.assertEqual(inverse(*project(7.,11.)),(7.,11.))
        self.assertIsNone(inverse_planar_projector(lambda x,y:(x,0.)))

    def test_gp_discovery_uses_stamped_raster_with_acquisition_odom_pose(self):
        from gp_branch_frontier import GPBranchFrontierMixin
        from unittest.mock import patch
        node=GPBranchFrontierMixin();node.gp_portal_assistance_enabled=True
        node.gp_branch_scan=S(header=S(stamp=genpy.Time.from_sec(2.),frame_id='scan'),
            ranges=[],angle_min=0.,angle_increment=.1,range_min=.1,range_max=8.)
        captures=[];lookups=[]
        node.gp_exploration_tree=S(last_stamp=0.,current_node_id=None,nodes={},
            active_branch_id=None,observe=lambda *a,**k:captures.append(a),snapshot=lambda:{})
        node.gp_direction_model=S(propose=lambda *a,**k:[])
        node.portal_authentication_node_to_pin=lambda:None
        node.active_portal_authentication=lambda:None
        node.tree_observations_for_authentication=lambda tree,observations:observations
        node._sync_portal_authentication=lambda *a:None
        node._update_portal_authentication_from_map=lambda *a:None
        node.publish_status=lambda *a,**k:None
        node.clearance=.52
        node._place_partition_tf_key=('map',1.)
        node._place_partition_projector=lambda x,y:(4.-y,-3.+x)
        node.planar_xy_projector=lambda *a,**k:(_ for _ in ()).throw(AssertionError('cached map must survive TF eviction'))
        node.transform_xy=lambda *a:(_ for _ in ()).throw(AssertionError('latest TF'))
        transform=S(translation=S(x=2.,y=1.),rotation=S(x=0.,y=0.,z=0.,w=1.))
        def lookup(*args):lookups.append(args[:3]);return S(transform=transform)
        node.tf_buffer=S(lookup_transform=lookup)
        msg=S(header=S(frame_id='map',stamp=genpy.Time.from_sec(1.)),data=[0]*9,
            info=S(height=3,width=3,resolution=1.,origin=S(position=S(x=0.,y=0.))))
        node.update_gp_branch_tree(msg,S(components=S(epoch=4)),None,None,3.)
        self.assertEqual(captures[0][:2],((2.,1.),(3.,-1.)))
        self.assertEqual([(t,s,st.to_sec()) for t,s,st in lookups],[('odom','scan',2.)])

class WatchingSnapshotRefreshTest(unittest.TestCase):
    def fixture(self):
        from gp_branch_frontier import GPBranchFrontierMixin
        n=GPBranchFrontierMixin();n.current_physical_place_id=4;n.active_semantic_room={'node_id':2}
        n.current_place_scope_id=4;n.current_place_scope_mask=np.zeros((4,4),dtype=bool)
        n._place_partition_tf_key=('map',1.);n._place_partition_projector=lambda x,y:(x,y)
        n.transform_xy=lambda *a:(_ for _ in ()).throw(AssertionError('latest transform'))
        lookups=[];updates=[]
        def acquire(target,source,stamp):lookups.append(stamp.to_sec());return lambda x,y:(x,y)
        n.planar_xy_projector=acquire
        def update(message,*args):
            updates.append(message.header.stamp.to_sec());n._place_partition_tf_key=('map',3.)
            n.current_place_scope_mask=np.ones((4,4),dtype=bool)
        n._update_place_partition=update
        m=S(header=S(frame_id='map',stamp=genpy.Time.from_sec(3.)),info=S(resolution=1.,origin=S(position=S(x=0.,y=0.))))
        c=S(known_free=np.ones((4,4),dtype=bool),unknown=np.zeros((4,4),dtype=bool),components=S(epoch=4))
        tree=S(current_node_id=14,nodes={14:dict(id=14,physical_xy=(1.5,1.5),branch_ids=[],junction=True)},branches={})
        return n,m,c,tree,lookups,updates
    def test_watching_task_refreshes_new_map_before_ownership_guard(self):
        from unittest.mock import patch
        n,m,c,tree,lookups,updates=self.fixture()
        with patch('gp_branch_frontier.has_confirmed_three_way_fan',return_value=False) as fan:
            n._sync_portal_authentication(tree,m,c,7.)
        self.assertEqual(lookups,[3.]);self.assertEqual(updates,[3.]);fan.assert_called_once()
    def test_missing_new_exact_tf_refuses_old_scope_without_borrowing_latest(self):
        from unittest.mock import patch
        n,m,c,tree,lookups,updates=self.fixture();n.planar_xy_projector=lambda *a,**k:None
        with patch('gp_branch_frontier.has_confirmed_three_way_fan') as fan:
            n._sync_portal_authentication(tree,m,c,7.)
        self.assertEqual(updates,[]);fan.assert_not_called();self.assertEqual(n._place_partition_tf_key,('map',1.))

class WatchingAnchorGeometryTest(unittest.TestCase):
    def run_observation(self,pinned):
        from gp_branch_frontier import GPBranchFrontierMixin
        from unittest.mock import patch
        n=GPBranchFrontierMixin();n.gp_portal_assistance_enabled=True;n.clearance=.52
        n.gp_branch_scan=S(header=S(stamp=genpy.Time.from_sec(2.),frame_id='scan'),ranges=[8.]*12,angle_min=0.,angle_increment=.1,range_min=.1,range_max=8.)
        captures=[];node=dict(id=14,physical_xy=(1.5,1.5),junction_confirmed=True)
        n.gp_exploration_tree=S(current_node_id=14,nodes={14:node},last_stamp=0.,node_radius=1.,active_branch_id=None,
            observe=lambda *a,**k:captures.append(a),snapshot=lambda:{})
        proposals=[S(heading=h,reach=3.,angular_width=.3,variance=.01) for h in (0.,2.,4.)]
        n.gp_direction_model=S(angles=np.linspace(-math.pi,math.pi,12),propose=lambda *a,**k:proposals)
        n.portal_authentication_node_to_pin=lambda:14 if pinned else None;n.active_portal_authentication=lambda:None
        n.tree_observations_for_authentication=lambda tree,observations:observations
        n._sync_portal_authentication=lambda *a:None;n._update_portal_authentication_from_map=lambda *a:None
        n.publish_status=lambda *a,**k:None;n.xy_to_grid_cell=lambda *a:(0,0)
        n._place_partition_tf_key=('map',1.);n._place_partition_projector=lambda x,y:(x,y)
        n.tf_buffer=S(lookup_transform=lambda *a:S(transform=S(translation=S(x=7.,y=7.),rotation=S(x=0.,y=0.,z=0.,w=1.))))
        m=S(header=S(frame_id='map',stamp=genpy.Time.from_sec(1.)),data=[0]*1600,
            info=S(height=40,width=40,resolution=1.,origin=S(position=S(x=0.,y=0.))))
        with patch('gp_branch_frontier.map_ranges',return_value=[8.]*12) as rays,patch('gp_branch_frontier.junction_anchor',return_value=(5.,5.)) as anchor,patch('gp_branch_frontier.entrance_geometry',return_value=None):
            n.update_gp_branch_tree(m,S(components=S(epoch=1)),None,None,3.)
        return captures,rays,anchor
    def test_pending_watching_geometry_stays_at_its_pinned_source_when_robot_is_far(self):
        captures,rays,anchor=self.run_observation(True)
        self.assertEqual(captures[0][:2],((1.5,1.5),(1.5,1.5)));anchor.assert_not_called()
        self.assertEqual(rays.call_args.args[3],(1.5,1.5))
    def test_unowned_discovery_may_still_find_a_new_actual_junction_anchor(self):
        captures,rays,anchor=self.run_observation(False)
        anchor.assert_called_once();self.assertEqual(captures[0][:2],((5.,5.),(5.,5.)))

if __name__=='__main__':unittest.main()
