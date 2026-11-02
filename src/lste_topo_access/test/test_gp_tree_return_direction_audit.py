"""Return direction coverage requires canonical crossing and SLAM identities."""
import unittest,sys,math,copy
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'scripts/tests/targeted_navigation'))
from check_gp_tree import verified_return_direction
class ReturnDirectionAuditTest(unittest.TestCase):
 def setUp(self):
  self.portal=dict(id=7,state='crossed',crossing_count=1,gp_authentication_state='completed',source_place_id=1,destination_place_id=2,physical_gate=[0,0],physical_destination=[1,0])
  self.branch=dict(connection='return',authentication_state='deduplicated',canonical_portal_id=7,heading=math.pi,return_source_place_id=2,return_destination_place_id=1,return_portal_evidence=dict(reason='visited_entry_side_of_current_place',duplicate_portal_id=7,parent_place_id=1,source_label=2,destination_label=1))
 def verify(self):return verified_return_direction(self.branch,{7:self.portal})
 def test_verified_reverse_edge(self):self.assertTrue(self.verify())
 def test_failed_candidate_is_not_coverage(self):self.branch['authentication_state']='failed';self.assertFalse(self.verify())
 def test_certification_without_crossing_is_not_return(self):self.portal['crossing_count']=0;self.assertFalse(self.verify())
 def test_wrong_parent_identity_is_rejected(self):self.branch['return_destination_place_id']=3;self.assertFalse(self.verify())
 def test_unknown_or_same_component_is_rejected(self):
  for label in (0,2):self.branch['return_portal_evidence']['destination_label']=label;self.assertFalse(self.verify())
 def test_forward_heading_is_not_return(self):self.branch['heading']=0;self.assertFalse(self.verify())
if __name__=='__main__':unittest.main()
