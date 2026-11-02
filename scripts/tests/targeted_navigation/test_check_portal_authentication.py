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
            p.write_text(''.join('test test event=global_frontier_event data='+json.dumps(e)+'\n' for e in events))
            return audit(p)
    def test_only_confirmed_junctions_require_observation(self):
        result=self.run_audit()
        self.assertEqual(result['candidate_branch_ids'],[2])
        self.assertTrue(result['checks']['every_stable_candidate_started'])
    def test_repeated_start_is_detected_before_unique_ids_are_formed(self):
        result=self.run_audit(repeat=True)
        self.assertEqual(result['duplicate_started_branch_ids'],[2])
        self.assertFalse(result['checks']['no_branch_started_twice'])

    def test_known_free_same_place_candidate_is_resolved_only_with_evidence_and_return(self):
        evidence = dict(branch_id=2, node_id=2, state='failed',
            failure_reason='known_free_bypass_within_current_place', source_xy=[0, 0],
            source_side_proven=True, return_gate_cleared=True,
            events=[dict(event='doorway_entry_started'), dict(event='vehicle_fully_inside'),
                dict(event='inside_frontier_observed', target_yaw=1.0, left=1, right=2),
                dict(event='room_facing_turn_complete'), dict(event='returned_to_junction_node'),
                dict(event='portal_candidate_rejected')])
        tree = dict(nodes=[dict(id=2, junction_confirmed=True, physical_xy=[0, 0], branch_ids=[2])],
            branches=[dict(id=2, node_id=2, stable=True, passability='open', physical_gate=[2, 0],
                state='authentication_failed', authentication_state='failed', authentication_evidence=evidence)],
            dedup_audit={'merged_candidate_count': 1})
        events = [
            dict(frontier_event='gp_branch_tree', tree=tree),
            dict(frontier_event='portal_authentication_branch_started', branch_id=2, node_id=2, state='entering'),
            dict(frontier_event='portal_authentication_route_selected', branch_id=2,
                node_id=2, phase='turning', target_yaw=1.0),
            dict(frontier_event='portal_authentication_route_selected', branch_id=2,
                node_id=2, phase='returning', target_yaw=1.0),
            dict(frontier_event='portal_candidate_rejected', branch_id=2,
                reason='known_free_bypass_within_current_place',
                spatial_evidence={'source_label': 4, 'destination_label': 4, 'entry_label': 4}),
            dict(frontier_event='portal_authentication_branch_finished', branch_id=2,
                node_id=2, state='failed', failure_reason='known_free_bypass_within_current_place'),
            dict(frontier_event='portal_authentication_observation_period', node_id=2,
                phase='complete', all_finished=True),
        ]
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'metrics.log'
            path.write_text(''.join('test event=global_frontier_event data='+json.dumps(e)+'\n' for e in events))
            accepted=audit(path)
            self.assertTrue(accepted['checks']['all_doorway_candidates_resolved'])
            self.assertTrue(accepted['branches'][0]['checks']['rejected_same_place_candidate_and_returned'])
            events[4]['spatial_evidence']['destination_label']=0
            path.write_text(''.join('test event=global_frontier_event data='+json.dumps(e)+'\n' for e in events))
            rejected=audit(path)
            self.assertFalse(rejected['checks']['all_doorway_candidates_resolved'])

if __name__=='__main__':unittest.main()
