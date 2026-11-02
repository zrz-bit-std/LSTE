"""Independent master reconstruction must detect real lethal-mask violations."""
from pathlib import Path
from types import SimpleNamespace as S
import sys,unittest
import numpy as np
import genpy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'scripts/tests/targeted_navigation'))
from audit_place_native_costmaps import (apply_update,exclusion_check,execution_mode,
    authorized_dfs_return_local_egress,authorized_portal_authentication_route,
    controller_lease_intervals,controller_mode_at)

def grid(data,stamp=10.,origin=(0.,0.)):
    m=OccupancyGrid();a=np.asarray(data);m.header.frame_id='map';m.header.stamp=genpy.Time.from_sec(stamp)
    m.info.width=a.shape[1];m.info.height=a.shape[0];m.info.resolution=1.
    m.info.origin.position.x,m.info.origin.position.y=origin;m.info.origin.orientation.w=1.
    m.data=a.ravel().tolist();return m
class NativeReplayTest(unittest.TestCase):
    def test_incremental_erosion_of_exclusion_is_detected_without_mutating_old_grid(self):
        old=grid([[100,0],[100,0]]);mask=grid([[100,-1],[100,-1]],stamp=9.)
        self.assertEqual(exclusion_check(old,mask)['nonlethal_cells'],0)
        u=OccupancyGridUpdate();u.header.frame_id='map';u.header.stamp=genpy.Time.from_sec(11.)
        u.x=u.y=0;u.width=u.height=1;u.data=[0]
        new=apply_update(old,u)
        self.assertEqual(exclusion_check(new,mask)['nonlethal_cells'],1)
        self.assertEqual(old.data,[100,0,100,0])
    def test_orphan_and_bad_extent_frame_updates_are_not_silently_applied(self):
        u=OccupancyGridUpdate();u.width=u.height=1;u.data=[100];u.header.frame_id='map'
        with self.assertRaises(ValueError):apply_update(None,u)
        old=grid([[0]])
        u.x=1
        with self.assertRaises(ValueError):apply_update(old,u)
        u.x=0;u.header.frame_id='odom'
        with self.assertRaises(ValueError):apply_update(old,u)
    def test_physical_obstacles_inside_place_are_preserved_and_not_false_errors(self):
        result=exclusion_check(grid([[100,100],[100,99]]),grid([[100,-1],[100,-1]],stamp=9.))
        self.assertEqual(result,dict(excluded_cells=2,nonlethal_cells=0,
                                      ambiguous_cells=0))
    def test_recorded_frame_transform_and_no_overlap(self):
        native=grid([[100,0]],origin=(-2.,0.));mask=grid([[100,-1]],stamp=9.)
        self.assertEqual(exclusion_check(native,mask,(2.,0.))['nonlethal_cells'],0)
        with self.assertRaises(ValueError):exclusion_check(native,mask,(20.,0.))
    def test_old_master_cannot_prove_a_newer_mask(self):
        with self.assertRaises(ValueError):exclusion_check(grid([[100]],stamp=8.),grid([[100]],stamp=9.))
    def test_only_owned_auth_portal_or_canonical_dfs_return_is_transparent(self):
        for kind in ('frontier_endpoint','local_egress','frontier_turn_connector'):
            self.assertEqual(execution_mode(dict(active=True,active_execution_route_kind=kind)),'ordinary')
        returning=dict(active=True,active_execution_route_kind='local_egress',
            place_dfs=dict(stack=[1,2],nodes=[
                dict(place_id=1,parent_id=None,state='suspended',portal_id=None),
                dict(place_id=2,parent_id=1,state='returning',portal_id=7)]),
            place_scope=dict(valid=True,place_id=2,entry_portal_id=7,
                projected_portals=[dict(id=7,source_place_id=1,destination_place_id=2)]))
        self.assertTrue(authorized_dfs_return_local_egress(returning))
        self.assertEqual(execution_mode(returning),'transparent')
        for mutation in (
            dict(place_scope=dict(valid=False,place_id=2,entry_portal_id=7,
                projected_portals=[dict(id=7,source_place_id=1,destination_place_id=2)])),
            dict(place_scope=dict(valid=True,place_id=2,entry_portal_id=8,
                projected_portals=[dict(id=7,source_place_id=1,destination_place_id=2)])),
            dict(place_dfs=dict(stack=[1,2],nodes=[dict(place_id=2,parent_id=1,state='exploring',portal_id=7)])),
        ):
            bad=dict(returning);bad.update(mutation)
            self.assertFalse(authorized_dfs_return_local_egress(bad))
            self.assertEqual(execution_mode(bad),'ordinary')
        self.assertEqual(execution_mode(dict(active=True,active_execution_route_kind='portal_transition')),'transparent')
        auth_route=dict(active=True,active_execution_route_kind='frontier_endpoint',
            portal_authentication=dict(active_branch_id=5,branches=[dict(branch_id=5,state='entering')]),
            goal_context=dict(goal_role='place_observation',owner_place_id=1,work_item_id=None))
        self.assertTrue(authorized_portal_authentication_route(auth_route))
        self.assertEqual(execution_mode(auth_route),'transparent')
        ordinary_wi=dict(auth_route);ordinary_wi['goal_context']=dict(auth_route['goal_context'],work_item_id=22)
        self.assertFalse(authorized_portal_authentication_route(ordinary_wi))
        self.assertEqual(execution_mode(ordinary_wi),'ordinary')
        stale_auth=dict(auth_route);stale_auth['portal_authentication']=dict(active_branch_id=5,branches=[dict(branch_id=5,state='completed')])
        self.assertEqual(execution_mode(stale_auth),'ordinary')
        self.assertEqual(execution_mode(dict(active=False,active_execution_route_kind='portal_transition')),'idle')
    def test_raw_bridge_exact_ack_bounds_controller_ownership_not_graph_route_lifetime(self):
        identity=dict(route_id=17,action_generation=33,graph_transaction_id=42,
                      lifecycle_transaction_id=99,transaction_id=17,map_epoch=111,
                      origin_epoch=111,projection_epoch=111)
        rows=[(170.005,dict(identity,event='dispatch',active=True,
                            active_route_id=17,active_action_generation=33)),
              (170.019,dict(identity,event='lease_release_ack',active=False))]
        leases=controller_lease_intervals(rows,lambda route,stamp:'ordinary')
        self.assertEqual(len(leases),1)
        self.assertTrue(leases[0]['exact_ack'])
        self.assertEqual(controller_mode_at(170.010,leases),'ordinary')
        self.assertEqual(controller_mode_at(170.108,leases),'idle')

    def test_raw_bridge_mismatched_ack_does_not_retire_controller_ownership(self):
        identity=dict(route_id=17,action_generation=33,graph_transaction_id=42,
                      lifecycle_transaction_id=99,transaction_id=17,map_epoch=111,
                      origin_epoch=111,projection_epoch=111)
        rows=[(170.005,dict(identity,event='dispatch',active=True,
                            active_route_id=17,active_action_generation=33)),
              (170.019,dict(identity,event='lease_release_ack',action_generation=34,active=False))]
        leases=controller_lease_intervals(rows,lambda route,stamp:'ordinary')
        self.assertFalse(leases[0]['exact_ack'])
        self.assertEqual(controller_mode_at(170.108,leases),'ordinary')

    def test_inactive_owned_tail_keeps_original_authorization_until_exact_ack(self):
        identity=dict(route_id=9,transaction_id=9,graph_transaction_id=20,
                      lifecycle_transaction_id=30,action_generation=7,
                      map_epoch=4,origin_epoch=4,projection_epoch=4)
        row=dict(active=False,pending=False,active_execution_route_id=9,
                 active_execution_route_kind='frontier_endpoint',
                 awaiting_controller_lease_release_ack=True,
                 portal_authentication=dict(active_branch_id=5),
                 released_controller_route=dict(identity,controller_pending=True,terminal_received=False))
        self.assertEqual(execution_mode(row,'ordinary'),'ordinary')
        self.assertEqual(execution_mode(row,'transparent'),'transparent')
        row.update(awaiting_controller_lease_release_ack=False,
                   released_controller_route=dict(identity,controller_pending=False,terminal_received=True),
                   last_controller_lease_release_ack=dict(identity,action_generation=6))
        self.assertEqual(execution_mode(row,'ordinary'),'ordinary')
        row['last_controller_lease_release_ack']=dict(identity)
        row['released_controller_route']['terminal_received']=False
        self.assertEqual(execution_mode(row,'ordinary'),'idle')
if __name__=='__main__':unittest.main()
