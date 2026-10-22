import json
import tempfile
import unittest
from pathlib import Path
from check_portal_authentication import audit

class AuditTest(unittest.TestCase):
    def run_audit(self, repeat=False):
        branches=[dict(id=i,node_id=i,stable=True,passability='open',physical_gate=[i,0],state='authenticated') for i in (1,2)]
        tree=dict(nodes=[dict(id=1,junction=True,branch_ids=[1]),dict(id=2,junction=True,junction_confirmed=True,branch_ids=[2])],branches=branches)
        events=[dict(frontier_event='gp_branch_tree',tree=tree),dict(frontier_event='portal_authentication_branch_started',branch_id=2,node_id=2,state='entering')]
        if repeat:events.append(dict(events[-1]))
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'metrics.log'
            p.write_text(''.join('test event=global_frontier_event data='+json.dumps(e)+'\n' for e in events))
            return audit(p)
    def test_only_confirmed_junctions_require_observation(self):
        result=self.run_audit()
        self.assertEqual(result['candidate_branch_ids'],[2])
        self.assertTrue(result['checks']['every_stable_candidate_started'])
    def test_repeated_start_is_detected_before_unique_ids_are_formed(self):
        result=self.run_audit(repeat=True)
        self.assertEqual(result['duplicate_started_branch_ids'],[2])
        self.assertFalse(result['checks']['no_branch_started_twice'])

if __name__=='__main__':unittest.main()
