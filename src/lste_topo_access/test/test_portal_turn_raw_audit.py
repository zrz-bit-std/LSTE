"""No metadata-only acceptance of the opposite physical sweep."""
import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'scripts/tests/targeted_navigation'))
from audit_portal_turn_directions import audit_turns
class RawTurnTest(unittest.TestCase):
 def fixture(self,sign=-1):
  poses=[(i*.02,sign*min(i*.02,3.)) for i in range(151)]
  turn={(2,3):dict(start=.02,completed=3.,initial_yaw=0.,target_yaw=-3.,direction='clockwise')}
  return poses,turn
 def test_actual_chosen_sweep_passes(self):self.assertTrue(audit_turns(*self.fixture())['passed'])
 def test_opposite_shortest_sweep_never_passes_even_near_same_final_heading(self):
  poses,turn=self.fixture(1);turn[(2,3)]['target_yaw']=-math.pi+.08
  self.assertFalse(audit_turns(poses,turn)['passed'])
 def test_missing_initial_pose_does_not_borrow_later_yaw(self):
  poses,turn=self.fixture();self.assertFalse(audit_turns(poses[10:],turn)['passed'])
 def test_pose_gap_rejects(self):
  poses,turn=self.fixture();self.assertFalse(audit_turns(poses[:40]+poses[60:],turn)['passed'])
 def test_completion_without_recorded_start_rejects(self):
  poses,turn=self.fixture();del turn[(2,3)]['start'];self.assertFalse(audit_turns(poses,turn)['passed'])
 def test_nonfinite_target_is_not_positive_evidence(self):
  poses,turn=self.fixture();turn[(2,3)]['target_yaw']=float('nan');self.assertFalse(audit_turns(poses,turn)['passed'])
 def test_nonfinite_pose_or_time_is_rejected(self):
  poses,turn=self.fixture();poses[30]=(float('nan'),0.);self.assertFalse(audit_turns(poses,turn)['passed'])
 def test_equal_bag_stamps_keep_both_ordered_pose_records(self):
  poses,turn=self.fixture();baseline=audit_turns(poses,turn)['results'][0]['pose_count'];poses.insert(50,(poses[49][0],poses[49][1]-.005))
  result=audit_turns(poses,turn);self.assertTrue(result['passed']);self.assertEqual(result['results'][0]['pose_count'],baseline+1)
 def test_backward_bag_time_rejects(self):
  poses,turn=self.fixture();poses[50]=(poses[49][0]-.1,poses[50][1]);self.assertFalse(audit_turns(poses,turn)['passed'])
if __name__=='__main__':unittest.main()
