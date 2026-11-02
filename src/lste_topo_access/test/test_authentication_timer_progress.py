"""Portal budget checks must precede a skipped map/planning lease pass."""
from pathlib import Path
from types import SimpleNamespace as S
import sys,unittest,runpy
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from gp_branch_frontier import GPBranchFrontierMixin
from global_frontier_planning_runtime import GlobalFrontierPlanningRuntimeMixin
from global_frontier_portal_authentication import DoorwayAuthentication
CHECK=Path(__file__).resolve().parents[3]/'scripts/tests/targeted_navigation/check_portal_authentication.py'
class Owner(GPBranchFrontierMixin,GlobalFrontierPlanningRuntimeMixin):
    def __init__(self):
        self.auth=DoorwayAuthentication(2,1,(1.,0.),(3.,0.));self.auth.begin((0.,0.))
        self.pose_odom=S(x=0.,y=0.,theta=0.);self.now=0.;self.time_provider=lambda:self.now
        self.active_timeout=20.;self.events=[];self.releases=[];self.preparations=0
        self.active_frontier=None;self.prefetched_frontier=None;self.map_msg=None
    def active_portal_authentication(self):return self.auth
    def publish_status(self,event,**fields):self.events.append((event,fields))
    def release_active_frontier(self,**fields):self.releases.append(fields)
    def _prepare_planning_cycle(self):self.preparations+=1;return None,True
    def _finish_planning_cycle(self,_token):pass
    def fresh_costmap(self):return None
class AuthenticationTimerProgressTest(unittest.TestCase):
    def tick(self,o,t):
        o.now=t
        with patch('rospy.loginfo_throttle'),patch('rospy.logerr'):o.on_timer(None)
    def test_slow_compute_yields_at_deadline_and_owner_settles_same_cycle(self):
        o=Owner();self.tick(o,0.)
        import copy
        before=copy.deepcopy(vars(o.auth))
        o.now=9.9;self.assertFalse(o.planning_should_preempt())
        o.now=10.05;self.assertTrue(o.planning_should_preempt())
        self.assertEqual(vars(o.auth),before)
        # Real movement at that deadline is not a stall or a reset by polling.
        o.pose_odom.x=.2;self.assertFalse(o.planning_should_preempt())
        o.pose_odom.x=0.
        o._finish_planning_cycle=lambda token:GlobalFrontierPlanningRuntimeMixin._finish_planning_cycle(o,token)
        o._prepare_planning_cycle=lambda:(None,False)
        def slow_compute(*args,**kwargs):
            o.now=10.1
            self.assertTrue(o.planning_should_preempt())
        o._on_timer=slow_compute
        self.tick(o,9.95)
        self.assertEqual(o.auth.state,'failed')
        expired=next(e for e in o.auth.events if e['event']=='doorway_physical_progress_expired')
        self.assertEqual(expired['elapsed'],10.1)
        self.assertEqual(len(o.releases),1)

    def test_entry_alignment_needs_improvement_not_arbitrary_rotation(self):
        import math
        o=Owner();o.pose_odom.theta=math.pi
        self.tick(o,0.)
        o.pose_odom.theta=math.pi-.2;self.tick(o,9.)
        self.assertEqual(o.auth.state,'entering')
        self.assertEqual(o.auth._progress_at,9.)
        proof=o.auth.events[-1]
        self.assertEqual(proof['event'],'doorway_physical_progress_observed')
        self.assertEqual(proof['kind'],'entry_alignment')
        self.assertEqual(proof['heading_yaw'],0.)
        # Moving away, or oscillating back to the last best heading, supplies
        # no new evidence and must still fail at the unchanged ten seconds.
        o.pose_odom.theta=math.pi-.1;self.tick(o,18.)
        self.assertEqual(o.auth._progress_at,9.)
        o.pose_odom.theta=math.pi-.2;self.tick(o,19.1)
        self.assertEqual(o.auth.state,'failed')
        self.assertEqual(o.auth.events[-1]['event'],'doorway_authentication_failed')

    def test_entering_at_source_settles_with_positive_proof_without_any_map(self):
        o=Owner();self.tick(o,0.);self.tick(o,10.1)
        self.assertEqual(o.auth.state,'failed');self.assertEqual(o.preparations,1)
        self.assertEqual(len(o.releases),1)
        check=runpy.run_path(str(CHECK))['bounded_stall_settled_at_source']
        record=o.auth.snapshot();self.assertTrue(check(record))
        self.tick(o,11.);self.assertEqual(len(o.releases),1)
        for kind in ('missing','far','wrong_side','late'):
            import copy
            r=copy.deepcopy(record)
            if kind=='missing':r['events']=[e for e in r['events'] if e['event']!='doorway_failure_settled_at_source']
            elif kind=='far':next(e for e in r['events'] if e['event']=='doorway_failure_settled_at_source')['pose_xy']=[2.,0.]
            elif kind=='wrong_side':next(e for e in r['events'] if e['event']=='doorway_failure_settled_at_source')['signed_depth']=.1
            else:next(e for e in r['events'] if e['event']=='doorway_physical_progress_expired')['elapsed']=100.
            self.assertFalse(check(r),kind)
    def test_away_from_source_preserves_return_and_stops_after_six_windows(self):
        o=Owner();o.auth.state='returning';o.pose_odom.x=4.
        self.tick(o,0.)
        for i in range(1,7):self.tick(o,i*10.1)
        self.assertEqual(o.auth.state,'returning')
        self.assertTrue(o.auth.return_recovery_final_blocked)
        self.tick(o,61.);self.tick(o,62.)
        self.assertEqual(sum(e=='portal_authentication_return_unrecoverable' for e,_ in o.events),1)
if __name__=='__main__':unittest.main()
