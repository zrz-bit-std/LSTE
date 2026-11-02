"""The native reset is bounded, action-owned and restores the original config."""
from pathlib import Path
from contextlib import nullcontext
from types import SimpleNamespace as S
import unittest,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from teb_native_reseed import NativeTebReseed
class ImmediateThread:
 def __init__(self,target,args,daemon):self.target=target;self.args=args
 def start(self):self.target(*self.args)
class Client:
 def __init__(self):self.value=1.;self.calls=[];self.fail_after_set=False
 def get_configuration(self,timeout):return {'force_reinit_new_goal_dist':self.value}
 def update_configuration(self,data):
  if not data:return {'force_reinit_new_goal_dist':self.value}
  self.value=data['force_reinit_new_goal_dist'];self.calls.append(self.value)
  if self.fail_after_set and self.value==0.:raise RuntimeError('response lost after apply')
  return {'force_reinit_new_goal_dist':self.value}
class NativeReseedTest(unittest.TestCase):
 def fixture(self,feedback=True,cancel=False):
  events=[];clock=[0.];identity=(7,11)
  owner=S(lock=nullcontext(),active_action=True,active_action_identity=identity,task_done=False,navigation_hold=False,latest_trajectory_command_wall=1.,trajectory_feedback_valid=False,trajectory_feedback_identity=None,publish_status_locked=lambda e,**kw:events.append((e,kw)))
  client=Client()
  def sleep(dt):
   clock[0]+=dt
   if cancel:owner.active_action=False
   if feedback and clock[0]>=.12:owner.latest_trajectory_command_wall=2.;owner.trajectory_feedback_valid=True;owner.trajectory_feedback_identity=identity
  helper=NativeTebReseed(owner,lambda:client,lambda:clock[0],sleep,ImmediateThread)
  return helper,owner,client,events,clock,identity
 def test_fresh_same_action_feedback_restores_threshold(self):
  h,o,c,e,t,i=self.fixture();self.assertTrue(h.request(i));self.assertEqual(c.calls,[0.,1.]);self.assertEqual(c.value,1.);self.assertLess(t[0],1.);self.assertTrue(any(x[0]=='native_teb_reseed_observed' and x[1]['feedback_acknowledged'] for x in e))
 def test_no_feedback_has_finite_window_and_restores(self):
  h,o,c,e,t,i=self.fixture(False);h.request(i);self.assertLessEqual(t[0],1.02);self.assertEqual(c.value,1.);self.assertFalse(next(d for n,d in e if n=='native_teb_reseed_observed')['feedback_acknowledged'])
 def test_release_restores_without_waiting_full_window(self):
  h,o,c,e,t,i=self.fixture(False,True);h.request(i);self.assertEqual(c.calls,[0.,1.]);self.assertLess(t[0],.1)
 def test_response_failure_after_apply_still_restores(self):
  h,o,c,e,t,i=self.fixture();c.fail_after_set=True;h.request(i);self.assertEqual(c.value,1.);self.assertTrue(any(n=='native_teb_reseed_failed' for n,d in e))
 def test_same_action_is_not_reseeded_repeatedly(self):
  h,o,c,e,t,i=self.fixture();h.request(i);self.assertFalse(h.request(i));self.assertEqual(c.calls,[0.,1.])
 def test_wrong_action_or_hold_has_no_configuration_side_effect(self):
  h,o,c,e,t,i=self.fixture();self.assertFalse(h.request((8,12)));o.navigation_hold=True;self.assertFalse(h.request(i));self.assertEqual(c.calls,[])
 def test_install_and_turn_boundary_are_present(self):
  root=Path(__file__).resolve().parents[1]
  self.assertIn('scripts/teb_native_reseed.py',(root/'CMakeLists.txt').read_text())
  self.assertIn('<exec_depend>dynamic_reconfigure</exec_depend>',(root/'package.xml').read_text())
  source=(root/'scripts/teb_turn_supervisor_turn_lifecycle.py').read_text()
  self.assertIn('completed and (previous_phase == "stalled_route_reorientation"',source)
  self.assertIn('reseed.request(previous_action_identity)',source)
 def test_other_action_feedback_cannot_acknowledge_reset(self):
  h,o,c,e,t,i=self.fixture();original_sleep=h.sleep
  def wrong_feedback(dt):original_sleep(dt);o.trajectory_feedback_identity=(8,12)
  h.sleep=wrong_feedback;h.request(i)
  self.assertFalse(next(d for n,d in e if n=='native_teb_reseed_observed')['feedback_acknowledged']);self.assertEqual(c.value,1.)
 def test_external_configuration_change_is_preserved(self):
  h,o,c,e,t,i=self.fixture();original_sleep=h.sleep
  def changed(dt):original_sleep(dt);c.value=.4
  h.sleep=changed;h.request(i);self.assertEqual(c.value,.4);self.assertTrue(any(n=='native_teb_reseed_restore_skipped' for n,d in e))
 def test_real_release_hook_only_after_successful_owned_recovery_turn(self):
  from teb_turn_supervisor_turn_lifecycle import TebTurnSupervisorTurnLifecycleMixin
  from teb_turn_supervisor_contract import STATE_TURNING
  for phase,completed,enabled,expected in [('stalled_route_reorientation',True,True,1),('stalled_route_reorientation',False,True,0),('initial_alignment',True,True,0),('stalled_route_reorientation',True,False,0)]:
   calls=[];o=S(state=STATE_TURNING,turn_key='turn',turn_phase=phase,turn_action_identity=(7,11),pose=S(theta=0.),turn_target_yaw=0.,turn_completed_count=0,turn_released_count=0,turn_settle_duration=0.,native_teb_reseed_enabled=enabled,native_teb_reseed=S(request=calls.append),publish_status_locked=lambda *a,**kw:None)
   TebTurnSupervisorTurnLifecycleMixin._release_turn_locked(o,'test',completed)
   self.assertEqual(len(calls),expected)
 def test_stall_detected_during_owned_initial_alignment_is_reseeded(self):
  from teb_turn_supervisor_turn_lifecycle import TebTurnSupervisorTurnLifecycleMixin
  from teb_turn_supervisor_contract import STATE_TURNING
  for phase,ready,completed,expected in [('pre_route_alignment',(7,11),True,1),('portal_entry_alignment',(7,11),True,1),('pre_route_alignment',(8,12),True,0),('pre_route_alignment',None,True,0),('pre_route_alignment',(7,11),False,0),('legacy_connector',(7,11),True,0)]:
   with self.subTest(phase=phase,ready=ready,completed=completed):
    calls=[];o=S(state=STATE_TURNING,turn_key='turn',turn_phase=phase,turn_action_identity=(7,11),stalled_route_ready_identity=ready,pose=S(theta=0.),turn_target_yaw=0.,turn_completed_count=0,turn_released_count=0,turn_settle_duration=0.,native_teb_reseed_enabled=True,native_teb_reseed=S(request=calls.append),publish_status_locked=lambda *a,**kw:None)
    TebTurnSupervisorTurnLifecycleMixin._release_turn_locked(o,'test',completed)
    self.assertEqual(len(calls),expected)
 def test_stale_subscriber_cache_cannot_leave_reseed_configuration_active(self):
  h,o,c,e,t,i=self.fixture()
  c.get_configuration=lambda timeout:{'force_reinit_new_goal_dist':1.}
  h.request(i);self.assertEqual(c.calls,[0.,1.]);self.assertEqual(c.value,1.)
if __name__=='__main__':unittest.main()
