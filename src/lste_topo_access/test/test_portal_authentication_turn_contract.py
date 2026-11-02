"""Physical locked sweeps survive ±pi, map changes and reject stale owners."""
import json,math,sys,threading,unittest
from pathlib import Path
from types import SimpleNamespace as S
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from portal_authentication_turn_contract import authentication_turn_contract, directed_turn_error
from teb_turn_supervisor_control import TebTurnSupervisorControlMixin
from teb_turn_supervisor_callbacks import TebTurnSupervisorCallbacksMixin
from global_frontier_route_command_publication import GlobalFrontierRouteCommandPublicationMixin


def contract():
    return dict(route_id=9,graph_transaction_id=19,lifecycle_transaction_id=3,
        node_id=2,branch_id=3,frame_id='odom',direction='clockwise',target_yaw=-4.208497479754039)


class AuthenticationTurnContractTest(unittest.TestCase):
    def fixture(self):
        c=contract();events=[]
        n=S(pose=S(theta=-1.0000109672546387),turn_target_yaw=c['target_yaw'],turn_velocity=0.,yaw_goal_tolerance=.35,
            turn_phase='portal_authentication_turn',turn_authentication_contract=c,
            latest_authentication_turn=c,active_action_route_kind='frontier_turn_connector',active_action_source='global_slam_frontier',pose_frame='odom',
            active_action_route_id=9,active_action_graph_transaction_id=19,_lifecycle_transaction_id_locked=lambda:3,
            _release_turn_locked=lambda reason,completed=False:events.append((reason,completed)),
            turn_abs_rotation=0.,turn_rotation_cap=5.,turn_initial_abs_error=3.55,
            turn_last_odom_progress_wall=0.,turn_no_progress_timeout=0.,max_vel_theta=.65,acc_lim_theta=.8,
            latest_planner_command=S(linear=S(x=0.),angular=S(z=0.)))
        return n,events
    def test_real_b3_counterexample_keeps_clockwise_despite_shortest_path_flip(self):
        n,events=self.fixture()
        self.assertGreater(math.atan2(math.sin(n.turn_target_yaw-n.pose.theta),math.cos(n.turn_target_yaw-n.pose.theta)),0.)
        command=TebTurnSupervisorControlMixin._turn_command_locked(n,.1)
        self.assertLess(command.angular.z,0.);self.assertEqual(events,[])
    def test_counterclockwise_and_wrap_crossing_keep_selected_sign(self):
        n,events=self.fixture();n.latest_authentication_turn['direction']='counterclockwise'
        n.turn_target_yaw=4.2;n.pose.theta=.5;n.latest_authentication_turn['target_yaw']=4.2
        command=TebTurnSupervisorControlMixin._turn_command_locked(n,.1)
        self.assertGreater(command.angular.z,0.)
        self.assertLess(directed_turn_error(-3.,3.,'clockwise'),0.)
    def test_stale_identity_refuses_even_when_yaw_is_already_within_tolerance(self):
        n,events=self.fixture();n.pose.theta=n.turn_target_yaw;n.active_action_route_id=10
        command=TebTurnSupervisorControlMixin._turn_command_locked(n,.1)
        self.assertEqual(command.angular.z,0.);self.assertEqual(events,[('authentication_turn_identity_changed',False)])
    def test_unrelated_action_source_cannot_consume_authentication_contract(self):
        n,events=self.fixture();n.active_action_source='target_tracker'
        TebTurnSupervisorControlMixin._turn_command_locked(n,.1)
        self.assertEqual(events,[('authentication_turn_identity_changed',False)])
    def test_target_tolerance_completes_without_extra_revolution(self):
        n,events=self.fixture();n.pose.theta=n.turn_target_yaw-.02
        TebTurnSupervisorControlMixin._turn_command_locked(n,.1)
        self.assertEqual(events,[('yaw_tolerance_reached',True)])
    def test_rotation_budget_still_enforced(self):
        n,events=self.fixture();n.turn_abs_rotation=5.;n.turn_capped_releases=0
        TebTurnSupervisorControlMixin._turn_command_locked(n,.1)
        self.assertEqual(events,[('odom_rotation_budget_exceeded',False)])
    def test_identity_frame_finite_and_direction_are_required(self):
        c=contract();self.assertIsNotNone(authentication_turn_contract(c,9,19,3))
        for key,value in [('route_id',10),('graph_transaction_id',20),('lifecycle_transaction_id',4),('frame_id','map'),('direction',''),('target_yaw',float('nan')),('branch_id',0)]:
            with self.subTest(key=key):
                bad=dict(c);bad[key]=value;self.assertIsNone(authentication_turn_contract(bad,9,19,3))
    def test_intent_callback_preserves_only_matching_contract_and_clears_on_next_route(self):
        n=S(lock=threading.RLock(),_activate_turn_locked=lambda:None,_release_turn_locked=lambda *a,**k:None)
        payload=dict(source='global_slam_frontier',route_kind='frontier_turn_connector',route_id=9,
            graph_transaction_id=19,lifecycle_transaction_id=3,authentication_turn=contract())
        TebTurnSupervisorCallbacksMixin.on_intent(n,S(data=json.dumps(payload)))
        self.assertEqual(n.latest_authentication_turn,contract())
        payload['route_id']=10
        TebTurnSupervisorCallbacksMixin.on_intent(n,S(data=json.dumps(payload)))
        self.assertIsNone(n.latest_authentication_turn)
    def test_route_producer_attaches_exact_identity_and_physical_turn(self):
        from global_frontier_reporting import GlobalFrontierReportingMixin as R
        from unittest.mock import patch
        class Reporter(R):
            def active_goal_context(self,route_kind=None,graph_transaction_id=None):
                return {'graph_transaction_id':graph_transaction_id}
        n=Reporter();messages=[]
        n.active_portal_authentication=lambda:S(state='turning',target_yaw=-4.2,turn_direction='clockwise',node_id=2,branch_id=3)
        n.graph_route_action_transaction=None;n.last_map_epoch=7;n.active_route_map_epoch=7
        n.active_route_id=9;n.active_action_generation=0;n.active_route_kind='frontier_turn_connector'
        n.active_transition_kind='';n.active_predecessor_route_id=0;n.active_transition_distance=None
        n.active_route_dispatch_contract={'route_id':9,'graph_transaction_id':19}
        n.route_lifecycle_transaction_id=lambda:3;n.command_publisher=S(publish=lambda m:messages.append(json.loads(m.data)))
        with patch('global_frontier_reporting.rospy.Time.now',return_value=__import__('rospy').Time.from_sec(1.)):
            n.publish_route_command('odom',1.,2.,-4.2,'frontier_turn_connector')
        raw=messages[0]['authentication_turn']
        self.assertEqual(raw['direction'],'clockwise');self.assertEqual(raw['target_yaw'],-4.2)
        self.assertIsNotNone(authentication_turn_contract(raw,9,19,3))

    def activation_fixture(self):
        from teb_turn_supervisor_contract import STATE_PASS_THROUGH
        n,events=self.fixture()
        n.mode=n.active_mode='teb';n.task_done=n.navigation_hold=False
        n.active_action_goal=S();n.active_action_source_goal=S();n.active_action_identity=('owned',)
        n._is_managed_action_locked=lambda:True;n._intent_matches_goal_locked=lambda *a:True
        n.endpoint_alignment_enabled=False;n._is_portal_transition_action_locked=lambda:False
        n._is_local_egress_action_locked=lambda:False;n._is_frontier_endpoint_action_locked=lambda:False
        n._turn_key_for_goal_locked=lambda *a:('turn',);n.completed_turn_key=None
        n._completed_turn_matches_locked=lambda *a:False;n.state=STATE_PASS_THROUGH
        n.turn_pending_identity=None;n.turn_start_confirm_duration=0.
        n._turn_clearance_reason_locked=lambda now:None;n._goal_xy=lambda goal:(1.,2.)
        n.turn_rotation_cap_factor=1.5;n.turn_count=0;n.latest_intent_source='global_slam_frontier'
        n.publish_status_locked=lambda *a,**k:None
        return n
    def test_activation_uses_frozen_physical_target_and_directed_initial_budget(self):
        from teb_turn_supervisor_turn_lifecycle import TebTurnSupervisorTurnLifecycleMixin as L
        n=self.activation_fixture()
        self.assertFalse(L._activate_turn_locked(n))
        self.assertTrue(L._activate_turn_locked(n))
        self.assertEqual(n.turn_phase,'portal_authentication_turn')
        self.assertAlmostEqual(n.turn_target_yaw,contract()['target_yaw'])
        self.assertGreater(n.turn_initial_abs_error,math.pi)
    def test_declared_auth_with_invalid_identity_does_not_fall_back_to_generic_turn(self):
        from teb_turn_supervisor_turn_lifecycle import TebTurnSupervisorTurnLifecycleMixin as L
        n=self.activation_fixture();n.latest_authentication_turn_required=True;n.active_action_route_id=10
        n._goal_yaw_in_pose_frame_locked=lambda goal:(_ for _ in ()).throw(AssertionError('generic turn fallback'))
        self.assertFalse(L._activate_turn_locked(n))

    def test_goal_publication_freezes_physical_pose_and_yaw_across_map_updates(self):
        from unittest.mock import patch
        auth=S(state='turning',inside_xy=(1.,2.),target_yaw=-4.2)
        goals=[];commands=[]
        n=S(active_route_kind='frontier_turn_connector',active_portal_authentication=lambda:auth,
            pose_odom=S(x=1.1,y=2.1),publish_route_command=lambda *a:commands.append(a),
            publisher=S(publish=lambda goal:goals.append(goal)))
        cmd=S(command_xy=(100.,200.),command_yaw=.9,route_kind='frontier_turn_connector',mission_route_kind='frontier_turn_connector')
        with patch('global_frontier_route_command_publication.rospy.Time.now',return_value=__import__('rospy').Time.from_sec(1.)):
            for _ in range(2):
                GlobalFrontierRouteCommandPublicationMixin.publish_active_route_pose(n,S(header=S(frame_id='map')),cmd)
                n.pose_odom=S(x=7.,y=8.);cmd.command_xy=(-300.,-400.);cmd.command_yaw=1.9
        self.assertEqual(commands[0],commands[1]);self.assertEqual(commands[0][:4],('odom',1.1,2.1,-4.2))

if __name__=='__main__':unittest.main()
