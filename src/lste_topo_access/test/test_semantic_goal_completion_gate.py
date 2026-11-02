"""A cheap pending gate cannot certify success or skip full acceptance."""
from pathlib import Path
import sys,json,tempfile,unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'scripts/tests/targeted_navigation'))
import check_semantic_junction_goal as goal
class CompletionGateTest(unittest.TestCase):
    def write(self,path,rows):
        path.write_text(''.join('event=global_frontier_event data='+json.dumps(r)+'\n' for r in rows))
    def complete(self):return dict(graph_route_plan=dict(status='complete',current_place_id=1),place_dfs=dict(stack=[1]))
    def test_latest_pending_overrides_earlier_complete(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'m.log';self.write(p,[self.complete()]);self.assertTrue(goal.completion_candidate(p))
            self.write(p,[self.complete(),dict(graph_route_plan=None,place_dfs=dict(stack=[1,2]))]);self.assertFalse(goal.completion_candidate(p))
    def test_root_complete_cannot_finish_an_owned_route_or_release_wait(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'m.log'
            for fields in (dict(active_execution_route_id=9),dict(awaiting_controller_lease_release_ack=True)):
                self.write(p,[dict(self.complete(),**fields)])
                self.assertFalse(goal.completion_candidate(p))

    def test_retained_route_needs_inactive_terminal_and_exact_release_ack(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'m.log'
            identity=dict(route_id=166,transaction_id=166,graph_transaction_id=398,
                          lifecycle_transaction_id=123,action_generation=245,
                          map_epoch=1037,origin_epoch=1037,projection_epoch=1037)
            row=dict(self.complete(),active=False,pending=False,active_execution_route_id=166,
                     released_controller_route=dict(identity,terminal_received=True,controller_pending=False),
                     last_controller_lease_release_ack=dict(identity))
            self.write(p,[row]);self.assertTrue(goal.completion_candidate(p))
            self.write(p,[dict(row,released_controller_route=dict(identity,terminal_received=False,controller_pending=False))]);self.assertTrue(goal.completion_candidate(p))
            for changed in (dict(active=True),dict(pending=True),
                            dict(last_controller_lease_release_ack=dict(identity,action_generation=244)),
                            dict(pending_controller_lease_releases={'other':identity})):
                self.write(p,[dict(row,**changed)]);self.assertFalse(goal.completion_candidate(p))

    def test_missing_or_corrupt_status_falls_back_never_reuses_positive(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'m.log';self.write(p,[self.complete(),dict(tree={})]);self.assertIsNone(goal.completion_candidate(p))
            with p.open('a') as f:f.write('event=global_frontier_event data={bad}\n')
            self.assertIsNone(goal.completion_candidate(p))
    def test_pending_does_not_run_expensive_checks(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'m.log';out=Path(tmp)/'result.json';self.write(p,[dict(graph_route_plan=None,place_dfs=dict(stack=[1,2]))])
            with patch.object(sys,'argv',['check',str(p),'--headings','0','--output',str(out)]),patch.object(goal,'check') as expensive:
                with self.assertRaises(SystemExit) as exit:goal.main()
            self.assertEqual(exit.exception.code,1);expensive.assert_not_called()
            self.assertFalse(json.loads(out.read_text())['passed'])
    def test_complete_candidate_still_requires_all_started_authentication(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'m.log';out=Path(tmp)/'result.json';self.write(p,[self.complete()])
            with patch.object(sys,'argv',['check',str(p),'--headings','0','--output',str(out)]),patch.object(goal,'check',return_value=dict(passed=True)),patch.object(goal,'authentication_audit',side_effect=[dict(passed=True),dict(passed=False)]) as auth,patch.object(goal,'lifecycle_audit',return_value=dict(passed=True)),patch.object(goal,'tsp_audit',return_value=dict(passed=True)),patch.object(goal,'dfs_audit',return_value=dict(passed=True)):
                with self.assertRaises(SystemExit) as exit:goal.main()
            self.assertEqual(exit.exception.code,1)
            self.assertEqual(auth.call_args.kwargs,dict(all_started_nodes=True))
            self.assertFalse(json.loads(out.read_text())['passed'])
if __name__=='__main__':unittest.main()
