"""Lifecycle audit projection keeps every decision input and unknown proof."""
from pathlib import Path
import json,runpy,tempfile,unittest,sys,copy,math
MODULE=Path(__file__).resolve().parents[3]/'scripts/tests/targeted_navigation/audit_semantic_room_lifecycle.py'
sys.path.insert(0,str(MODULE.parent))
ns=runpy.run_path(str(MODULE))
class LifecycleAuditProjectionTest(unittest.TestCase):
    def test_projection_reuses_histories_and_keeps_suppression_and_state(self):
        row=dict(frontier_event='x',work_items=[dict(id=2,place_id=3,state='unresolved',anchor_xy=[0,0],attempts=[dict(id=4,state='suspended',reason='portal_junction_observation_period',unused=list(range(100)))])],
                 tree=dict(nodes=[dict(id=1,junction_confirmed=True,physical_xy=[0,0])],branches=[dict(id=2,canonical_portal_id=4,state='authenticated',authentication_state='completed')]),
                 portal_authentication=dict(node_id=1,phase='complete',all_finished=True),
                 ignored=dict(reentry_suppressed=True,portal_id=4,branch_id=2),huge=list(range(1000)))
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'metrics.log';p.write_text(('x event=global_frontier_event data='+json.dumps(row)+'\n')*2)
            rows=list(ns['records'](p))
            self.assertEqual(len(rows),2);self.assertNotIn('huge',rows[0])
            self.assertIs(rows[0]['work_items'],rows[1]['work_items'])
            self.assertEqual(rows[0]['work_items'][0]['attempts'][0]['reason'],'portal_junction_observation_period')
            self.assertEqual(rows[0]['suppression_evidence'][0]['portal_id'],4)
    def test_parent_direction_needs_bound_reverse_identity_and_positive_free_evidence(self):
        portal=dict(id=7,state='crossed',source_place_id=2,destination_place_id=3,
                    crossing_count=2,gp_authentication_state='completed',physical_gate=[0,0],physical_destination=[2,0])
        proof=dict(reason='visited_entry_side_of_current_place',duplicate_portal_id=7,parent_place_id=2,
                   source_label=2,destination_label=1)
        branch=dict(id=8,node_id=4,connection='return',authentication_state='deduplicated',
                    canonical_portal_id=7,return_source_place_id=3,return_destination_place_id=2,
                    heading=math.pi,return_portal_evidence=proof)
        row=dict(tree=dict(branches=[branch,dict(branch,id=9)]),portal_hypotheses=dict(records=[portal]))
        def check(value,rooms=[]):
            with tempfile.TemporaryDirectory() as tmp:
                p=Path(tmp)/'metrics.log';p.write_text('event=global_frontier_event data='+json.dumps(value)+'\n')
                snapshots=list(ns['records'](p))
            return ns['verified_parent_directions'](snapshots,4,rooms)
        original=copy.deepcopy(row)
        self.assertEqual(len(check(row)),1)
        self.assertEqual(row,original)
        self.assertEqual(check(row,[dict(branch_id=8)]),[])
        for kind in ('same_component','wrong_parent','not_crossed','wrong_heading','missing_portal','other_node'):
            r=copy.deepcopy(row)
            for b in r['tree']['branches']:
                if kind=='same_component':b['return_portal_evidence']['destination_label']=2
                elif kind=='wrong_parent':b['return_destination_place_id']=99
                elif kind=='wrong_heading':b['heading']=0.
                elif kind=='other_node':b['node_id']=5
            if kind=='not_crossed':r['portal_hypotheses']['records'][0]['crossing_count']=0
            if kind=='missing_portal':r['portal_hypotheses']['records']=[]
            self.assertEqual(check(r),[],kind)

    def test_parent_resume_binds_to_same_junction_suspension_event(self):
        def row(event, wi, *, node=10, state='unresolved', attempts=None):
            return dict(frontier_event=event, work_item_id=wi,
                        portal_authentication=dict(node_id=node, phase='watching', all_finished=False),
                        tree=dict(nodes=[dict(id=10, junction_confirmed=True, physical_xy=[0,0])]),
                        work_items=[dict(id=wi, place_id=1, state=state,
                                         attempts=attempts or [])])
        suspended = row('work_item_suspended', 3, attempts=[
            dict(id=1, state='suspended', reason='place_scope_revision')])
        resumed = row('work_item_settled', 3, state='resolved', attempts=[
            dict(id=1, state='suspended', reason='place_scope_revision'),
            dict(id=469, state='succeeded', reason='endpoint_coverage_recorded')])
        other_node = row('work_item_suspended', 9, node=11, attempts=[
            dict(id=1, state='suspended', reason='place_scope_revision')])
        def run(rows):
            with tempfile.TemporaryDirectory() as directory:
                path=Path(directory)/'metrics.log'
                path.write_text(''.join('event=global_frontier_event data='+json.dumps(x)+chr(10) for x in rows))
                return ns['audit'](path,[0,0],2.0,0)
        self.assertTrue(run([suspended,resumed])['checks']['suspended_parent_work_item_resumed'])
        self.assertFalse(run([other_node,resumed])['checks']['suspended_parent_work_item_resumed'])
        self.assertFalse(run([suspended,row('work_item_settled',3,state='unresolved',attempts=[
            dict(id=1,state='suspended',reason='place_scope_revision')])])['checks']['suspended_parent_work_item_resumed'])

    def test_unknown_requires_explicit_semantic_and_physical_certification(self):
        room=dict(branch_id=2,state='observed',invalidated=False,exploration_suppressed=False,
                  semantic_evidence='unavailable',labels=[],stable_frame_count=0,semantic_score=-1.)
        check=ns['unknown_semantics_certified']
        self.assertTrue(check(room,{2}));self.assertFalse(check(room,set()))
        for k,v in [('semantic_evidence',None),('invalidated',True),('state','pruned'),('exploration_suppressed',True)]:
            r=dict(room);r[k]=v;self.assertFalse(check(r,{2}))
if __name__=='__main__':unittest.main()
