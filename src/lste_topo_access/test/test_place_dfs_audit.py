"""Verifier tests use small synthetic records; Gazebo remains the goal evidence."""
from pathlib import Path
import sys, tempfile, json, copy, unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'scripts/tests/targeted_navigation'))
from audit_place_dfs import audit


class PlaceDfsAuditTest(unittest.TestCase):
    def fixture(self):
        parent=dict(place_id=1,parent_id=None,portal_id=None,children=[2],state='suspended')
        child=dict(place_id=2,parent_id=1,portal_id=10,children=[],state='exploring')
        base=dict(portal_hypotheses=dict(records=[dict(id=10,source_place_id=1,
            destination_place_id=2,state='crossed',crossing_count=1,opening_width=2.,gp_authentication_state='completed')]),
            work_items=[dict(id=20,place_id=2,state='unresolved')],
            place_dfs=dict(stack=[1,2],nodes=[parent,child]),
            place_scope=dict(place_id=2,valid=True,frontier_count=0))
        entered=copy.deepcopy(base);entered.update(frontier_event='place_dfs_entered',place_id=2,parent_place_id=1,portal_id=10)
        dispatch=copy.deepcopy(base);dispatch.update(frontier_event='work_item_dispatched',work_item_id=20,
            goal_context=dict(owner_place_id=2),graph_route_plan=dict(work_item_tour=[20]),
            place_membership=dict(in_place=True,scope_place_id=2))
        ready=copy.deepcopy(base);ready['work_items'][0]['state']='resolved';ready['place_dfs']['nodes'][1]['state']='returning'
        ready.update(frontier_event='place_dfs_completion_ready',place_id=2)
        returned=copy.deepcopy(ready);returned['place_dfs']['stack']=[1]
        returned['place_dfs']['nodes'][1]['state']='completed';returned['place_dfs']['nodes'][0]['state']='exploring'
        returned.update(frontier_event='place_dfs_returned',place_id=2,parent_place_id=1,portal_id=10)
        return [entered,dispatch,ready,returned]

    def check(self,records,depth=2):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'metrics.log'
            p.write_text(''.join('event=global_frontier_event data='+json.dumps(r)+'\n' for r in records))
            return audit(p,depth)

    def test_observed_stack_and_actual_dispatch_pass_but_do_not_prove_deeper_dfs(self):
        self.assertTrue(self.check(self.fixture())['passed'])
        self.assertFalse(self.check(self.fixture(),3)['passed'])

    def test_failure_recovery_cannot_pass_even_after_an_apparent_complete_stack(self):
        for event in ('place_scope_recovery_started','place_scope_recovery_returned',
                      'place_scope_recovery_unrecoverable','portal_authentication_return_unrecoverable',
                      'place_dfs_return_unrecoverable'):
            records=self.fixture()
            records.insert(1,dict(records[0],frontier_event=event))
            result=self.check(records)
            self.assertFalse(result['passed'])
            self.assertFalse(result['checks']['no_unresolved_execution_failure'])

    def test_wrong_owner_or_outside_slam_viewpoint_fails(self):
        records=self.fixture();records[1]['goal_context']['owner_place_id']=1
        self.assertFalse(self.check(records)['passed'])
        records=self.fixture();records[1]['place_membership']['in_place']=False
        self.assertFalse(self.check(records)['passed'])

    def test_unfinished_child_and_wrong_return_portal_fail(self):
        records=self.fixture();records[2]['place_dfs']['nodes'][1]['children']=[3]
        self.assertFalse(self.check(records)['passed'])
        records=self.fixture();records[-1]['portal_id']=99
        self.assertFalse(self.check(records)['passed'])

    def test_root_frozen_work_must_not_be_abandoned_after_child_return(self):
        records=self.fixture();records[-1]['work_items'].append(dict(id=8,place_id=1,state='unresolved'))
        result=self.check(records)
        self.assertFalse(result['checks']['root_work_items_complete'])
        self.assertEqual(result['root_pending_work_items'],[8])

    def test_missing_width_cannot_pass_spatial_validation(self):
        records=self.fixture();records[0]['portal_hypotheses']['records'][0]['opening_width']=None
        result=self.check(records)
        self.assertFalse(result['checks']['finite_portal_width'])
        self.assertFalse(result['passed'])

    def test_missing_gp_proof_cannot_pass_vacuously(self):
        records=self.fixture()
        for r in records:r['portal_hypotheses']['records'][0].pop('gp_authentication_state')
        self.assertFalse(self.check(records)['passed'])

if __name__=='__main__':unittest.main()


class ResidualFrontierProofTest(unittest.TestCase):
    fixture = PlaceDfsAuditTest.fixture
    check = PlaceDfsAuditTest.check
    def test_residual_cell_requires_completed_owner_and_same_map_evidence(self):
        records=self.fixture();scope=records[2]['place_scope']
        scope.update(frontier_count=1,pending_frontier_count=0,map_stamp=10.,
            frontier_work_evidence=dict(map_stamp=10.,raw_count=1,pending_count=0,resolved_count=1,
                                        resolved_frontier_cells=[[3,4,20]]))
        self.assertTrue(self.check(records)['passed'])
        for field,value in (('resolved_frontier_cells',[]),('map_stamp',9.)):
            bad=copy.deepcopy(records);bad[2]['place_scope']['frontier_work_evidence'][field]=value
            self.assertFalse(self.check(bad)['passed'])
        bad=copy.deepcopy(records);bad[2]['place_scope']['frontier_work_evidence']['resolved_frontier_cells']=[[3,4,99]]
        self.assertFalse(self.check(bad)['passed'])
