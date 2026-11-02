#!/usr/bin/env python3
"""A bounded door failure must prove its deadline and return, not just abort."""
import copy
import tempfile
import json
import runpy
from pathlib import Path
import unittest

CHECK = Path(__file__).resolve().parents[3] / 'scripts/tests/targeted_navigation/check_portal_authentication.py'
bounded_stall_returned = runpy.run_path(str(CHECK))['bounded_stall_returned']


class BoundedAuthenticationFailureAuditTest(unittest.TestCase):
    def record(self):
        reason='authentication_physical_stall:entering'
        return dict(failure_reason=reason,source_xy=[0.,0.],source_side_proven=True,
                    events=[dict(event='doorway_physical_progress_expired',phase='entering',timeout=10.,elapsed=10.5),
                            dict(event='doorway_failure_return_started'),
                            dict(event='returned_to_junction_node_after_failure',pose_xy=[.3,.4],distance=.5,reason=reason)])

    def test_bounded_failed_door_with_physical_return_is_resolved(self):
        self.assertTrue(bounded_stall_returned(self.record()))

    def test_plain_abort_missing_return_or_late_stall_never_passes(self):
        for kind in ('plain_abort','no_return','late','not_at_source','wrong_reason'):
            record=self.record()
            if kind=='plain_abort':record['failure_reason']='navigation_route_failed:controller_stall'
            elif kind=='no_return':record['events'].pop()
            elif kind=='late':record['events'][0]['elapsed']=100.
            elif kind=='not_at_source':record['events'][-1].update(pose_xy=[2.,0.],distance=2.)
            else:record['events'][-1]['reason']='unrelated'
            self.assertFalse(bounded_stall_returned(record),kind)


class ExplicitRejectionEvidenceTest(unittest.TestCase):
    def record(self, planner):
        reason='no_reachable_interior_entry_goal' if planner else 'navigation_route_failed:active_timeout'
        proof=(dict(event='doorway_interior_plan_unavailable',feasible_interior_goal_count=0,
                    map_stamp=20.,map_epoch=4,navigation_clearance=.52,minimum_depth=1.5)
               if planner else dict(event='doorway_navigation_phase_budget_expired',phase='turning',
                    timeout=10.,elapsed=10.5,route_id=3,reason='active_timeout'))
        return dict(failure_reason=reason,source_xy=[0.,0.],source_side_proven=True,
                    required_depth=1.3,opening_width=1.2,physical_bounds=[[0.,-.6],[0.,.6]],
                    events=[proof,dict(event='doorway_failure_return_started'),
                            dict(event='returned_to_junction_node_after_failure',pose_xy=[.3,.4],distance=.5,reason=reason)])

    def test_explicit_planner_or_deadline_with_measured_return(self):
        check=runpy.run_path(str(CHECK))['evidenced_rejection_returned']
        self.assertTrue(check(self.record(True)))
        self.assertTrue(check(self.record(False)))

    def test_reason_string_does_not_replace_positive_evidence(self):
        check=runpy.run_path(str(CHECK))['evidenced_rejection_returned']
        for planner in [True,False]:
            r=self.record(planner);r['events'].pop(0);self.assertFalse(check(r))
            r=self.record(planner);r['events'][-1]['pose_xy']=[2.,0.];self.assertFalse(check(r))
        r=self.record(True);r['opening_width']=.9;self.assertFalse(check(r))
        r=self.record(True);r['source_xy']=[];r['events'][-1]['pose_xy']=[];self.assertFalse(check(r))
        r=self.record(True);r['events'][0]['map_stamp']=float('nan');self.assertFalse(check(r))
        r=self.record(True);r['events'][0]['minimum_depth']=float('inf');self.assertFalse(check(r))
        r=self.record(False);r['events'][0]['elapsed']=100.;self.assertFalse(check(r))


class FullAuthenticationScopeTest(unittest.TestCase):
    def test_full_scope_includes_started_unconfirmed_nested_nodes(self):
        audit=runpy.run_path(str(CHECK))['audit']
        tree=dict(nodes=[dict(id=1,junction_confirmed=True,branch_ids=[1]),
                         dict(id=2,junction_confirmed=False,branch_ids=[2])],
                  branches=[dict(id=1,node_id=1,stable=True,passability='open',physical_gate=[0,0])])
        rows=[dict(frontier_event='gp_branch_tree',tree=tree),
              dict(frontier_event='portal_authentication_branch_started',node_id=1,branch_id=1),
              dict(frontier_event='portal_authentication_branch_started',node_id=2,branch_id=2)]
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'metrics.log'
            path.write_text(''.join('event=ignored event=global_frontier_event data='+json.dumps(r)+'\n' for r in rows))
            self.assertEqual(audit(path)['started_branch_ids'],[1])
            self.assertEqual(audit(path,all_started_nodes=True)['started_branch_ids'],[1,2])

    def test_corrupt_metrics_never_silently_passes_authentication(self):
        load=runpy.run_path(str(CHECK))['load_events']
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'metrics.log'
            path.write_bytes(b'event=x event=global_frontier_event data={"bad":"\xff"}\n')
            with self.assertRaises(UnicodeDecodeError):load(path)
            path.write_text('event=x event=global_frontier_event data={broken}\n')
            with self.assertRaises(ValueError):load(path)

    def test_sample_corruption_and_partial_records_fail_full_log_gate(self):
        load=runpy.run_path(str(CHECK))['load_events']
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'metrics.log'
            for raw in ('event=x event=sample data={broken}\n',
                        'event=x event=sample data={"pose":[0,0,0]}',
                        'event=x event=sample data=[]\n',
                        'partial metrics record\n'):
                path.write_text(raw)
                with self.assertRaises(ValueError):load(path)
            path.write_text('event=x event=sample data={"pose":[0,0,0]}\n')
            self.assertEqual(load(path),[])

    def test_loader_retains_audit_fields_and_latest_tree_without_full_snapshots(self):
        load=runpy.run_path(str(CHECK))['load_events']
        rows=[dict(frontier_event='gp_branch_tree',tree={'nodes':[1]},dedup_audit={'hits':4}),
              dict(frontier_event='portal_authentication_route_selected',branch_id=3,phase='turning',
                   target_yaw=.2,work_items=['unused']*1000),
              dict(frontier_event='gp_branch_tree',tree={'nodes':[2]},dedup_audit={'hits':2}),
              dict(frontier_event='route_command',work_items=['unused']*1000)]
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'metrics.log'
            path.write_text(''.join('event=x event=global_frontier_event data='+json.dumps(r)+'\n' for r in rows))
            result=load(path)
            self.assertEqual(len(result),2)
            self.assertEqual(result[0][1]['tree'],{'nodes':[2]})
            self.assertEqual(result[0][1]['dedup_audit']['hits'],4)
            self.assertEqual(result[1][1],{'branch_id':3,'phase':'turning','target_yaw':.2})


if __name__=='__main__':unittest.main()
