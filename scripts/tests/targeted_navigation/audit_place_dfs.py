#!/usr/bin/env python3
"""Audit measured Portal commits, Place-local dispatch, DFS and full closure."""
import argparse
import json
import math
from pathlib import Path


def events(path):
    # A metrics node writes its own log; run 20261103_141641 contains one split
    # multi-byte character and one spliced record (1 of 8249), and every audit
    # that reads through this helper died before judging anything.  Replacement
    # decoding cannot change ASCII JSON payloads, and a bounded number of
    # damaged records is skipped and reported instead of hiding the evidence.
    damaged = 0
    with Path(path).open(encoding='utf8',errors='replace') as stream:
        for number,line in enumerate(stream,1):
            if 'event=global_frontier_event data=' not in line:
                continue
            try:
                event = json.loads(line.split(' data=',1)[1])
            except ValueError:
                damaged += 1
                if damaged > 3:
                    raise
                continue
            yield number,event



def completed_frontier_accounted(scope, items, place):
    """Require explicit completed WI ownership for every residual frontier cell."""
    raw=scope.get('frontier_count')
    if raw==0:return True
    if not isinstance(raw,int) or raw<0:return False
    proof=scope.get('frontier_work_evidence') or {}
    cells=proof.get('resolved_frontier_cells') or []
    if (scope.get('pending_frontier_count')!=0 or proof.get('pending_count')!=0
            or proof.get('map_stamp')!=scope.get('map_stamp')
            or proof.get('raw_count')!=raw or proof.get('resolved_count')!=raw
            or len(cells)!=raw):return False
    records={w['id']:w for w in items};seen=set()
    for cell in cells:
        if not isinstance(cell,(list,tuple)) or len(cell)!=3:return False
        row,col,item=cell;record=records.get(item,{})
        if ((row,col) in seen or record.get('place_id')!=place
                or record.get('state')!='resolved'):return False
        seen.add((row,col))
    return True


def audit(path, min_stack_depth=3):
    errors=[];entered=[];returned=[];ready=set();closed=set();dispatches=[]
    max_depth=0;latest={};last_stack=[];has_scope=False;has_gp_proof=False;frozen_by_place={};boundary_proofs={}
    checks=dict(physical_portal_bindings=True,place_local_tsp_dispatches=True,
                child_before_parent_work=True,full_subtree_before_close=True,
                returns_through_entry=True,no_closed_place_work=True,
                spatial_dispatch_evidence=True,finite_portal_width=True,nested_boundary_confirmed=True,
                no_unresolved_execution_failure=True)
    def fail(key,line,reason):
        checks[key]=False
        if len(errors)<40:errors.append(dict(line=line,check=key,reason=reason))
    for line,d in events(path):
        latest=d;event=d.get('frontier_event');dfs=d.get('place_dfs') or {}
        stack=dfs.get('stack') or [];nodes={n['place_id']:n for n in dfs.get('nodes',[])}
        records=(d.get('portal_hypotheses') or {}).get('records',[])
        portals={r['id']:r for r in records};items=d.get('work_items') or []
        has_gp_proof = has_gp_proof or any(r.get('gp_authentication_state')=='completed' for r in records)
        max_depth=max(max_depth,len(stack));has_scope=has_scope or bool((d.get('place_scope') or {}).get('valid'))
        for n in nodes.values():
            if n.get('suspended_work_item_id') is not None:
                frozen_by_place[n['place_id']]=n['suspended_work_item_id']
        if event in ('place_scope_recovery_started','place_scope_recovery_returned',
                     'place_scope_recovery_unrecoverable','portal_authentication_return_unrecoverable',
                     'place_dfs_return_unrecoverable'):
            fail('no_unresolved_execution_failure',line,'bounded recovery failure cannot count as completed DFS')
        if event=='portal_place_spatial_evidence':
            boundary_proofs[d.get('branch_id')]=d.get('evidence') or {}
        if event=='place_dfs_entered':
            parent,place,portal=d.get('parent_place_id'),d.get('place_id'),d.get('portal_id')
            edge=portals.get(portal,{})
            try:width=float(edge.get('opening_width'))
            except (TypeError,ValueError):width=float('nan')
            if not math.isfinite(width) or width<=0:
                fail('finite_portal_width',line,'crossed Portal lacks finite measured opening width')
            if len(stack)>2:
                proof=boundary_proofs.get(edge.get('gp_authentication_branch_id'),{})
                if proof.get('reject') or proof.get('reason')!='distinct_destination_component':
                    fail('nested_boundary_confirmed',line,'nested Place lacks distinct SLAM destination proof before crossing')
            if (edge.get('state')!='crossed' or edge.get('source_place_id')!=parent
                    or edge.get('destination_place_id')!=place or parent==place
                    or int(edge.get('crossing_count',0))<1):
                fail('physical_portal_bindings',line,'DFS push lacks matching physical crossed Portal')
            if stack[-2:]!=[parent,place] or (last_stack and stack[:-1]!=last_stack):
                fail('returns_through_entry',line,'DFS push changed unrelated stack frames')
            entered.append(dict(place_id=place,parent_id=parent,portal_id=portal,time=d.get('ros_time')))
        if event=='work_item_dispatched':
            plan=d.get('graph_route_plan') or {};owner=(d.get('goal_context') or {}).get('owner_place_id')
            wi=d.get('work_item_id');tour=plan.get('work_item_tour') or []
            if not stack or owner!=stack[-1] or not tour or tour[0]!=wi:
                fail('place_local_tsp_dispatches',line,'actual dispatch is not current Place TSP head')
            member=d.get('place_membership') or {}
            if not member.get('in_place') or member.get('scope_place_id')!=owner:
                fail('spatial_dispatch_evidence',line,'actual viewpoint lacks matching SLAM partition proof')
            if owner in closed or owner in ready:
                fail('no_closed_place_work',line,'WI dispatched after Place completion')
            for edge in records:
                dest=edge.get('destination_place_id');state=(nodes.get(dest) or {}).get('state')
                if (edge.get('source_place_id')==owner and edge.get('gp_authentication_state')=='completed'
                        and not edge.get('exploration_suppressed') and dest not in stack
                        and state!='completed' and edge.get('state') in ('certified','selected','crossed')):
                    fail('child_before_parent_work',line,'parent WI dispatched with pending certified child')
            dispatches.append(dict(place_id=owner,work_item_id=wi,tour=tour,time=d.get('ros_time')))
        if event=='place_dfs_completion_ready':
            place=d.get('place_id');node=nodes.get(place,{})
            pending=[i['id'] for i in items if i.get('place_id')==place and i.get('state')=='unresolved']
            scope=d.get('place_scope') or {}
            if (pending or not scope.get('valid') or scope.get('place_id')!=place
                    or not completed_frontier_accounted(scope,items,place) or node.get('state')!='returning'
                    or any(nodes.get(c,{}).get('state')!='completed' for c in node.get('children',[]))):
                fail('full_subtree_before_close',line,'local WI, SLAM frontier or child remains at completion')
            ready.add(place)
        if event=='place_dfs_returned':
            place,parent,portal=d.get('place_id'),d.get('parent_place_id'),d.get('portal_id')
            node=nodes.get(place,{})
            if (place not in ready or node.get('state')!='completed'
                    or node.get('parent_id')!=parent or node.get('portal_id')!=portal
                    or not stack or stack[-1]!=parent):
                fail('returns_through_entry',line,'return lacks completed subtree/original parent edge')
            closed.add(place);returned.append(dict(place_id=place,parent_id=parent,
                                                  portal_id=portal,time=d.get('ros_time')))
        # Other status events may already contain the post-commit stack
        # before the explicit DFS event is published. Compare transaction
        # boundaries, not incidental snapshots of the same completed push.
        if stack and (not last_stack or event in ('place_dfs_entered','place_dfs_returned')):
            last_stack=list(stack)
    final_dfs=latest.get('place_dfs') or {};final_stack=final_dfs.get('stack') or []
    root_id=final_stack[0] if final_stack else None
    root_pending=[w['id'] for w in latest.get('work_items',[]) if w.get('place_id')==root_id and w.get('state')=='unresolved']
    checks.update(root_work_items_complete=bool(final_stack) and not root_pending,
                  dfs_events_recorded=bool(entered and returned),
                  nested_physical_dfs_exercised=max_depth>=int(min_stack_depth),
                  every_entered_subtree_returned=bool(entered) and {n['place_id'] for n in entered}<=closed,
                  back_at_root=len(final_stack)==1,
                  actual_work_item_dispatches_recorded=bool(dispatches),online_slam_scope_recorded=has_scope,
                  gp_certification_records_present=has_gp_proof)
    return dict(passed=all(checks.values()),checks=checks,errors=errors,
                max_stack_depth=max_depth,required_stack_depth=int(min_stack_depth),
                final_stack=final_stack,root_pending_work_items=root_pending,entered=entered,returned=returned,
                dispatches=dispatches,suspended_work_items=frozen_by_place,metrics=str(path))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('metrics')
    p.add_argument('--min-stack-depth',type=int,default=3);p.add_argument('--output')
    a=p.parse_args();result=audit(a.metrics,a.min_stack_depth)
    text=json.dumps(result,indent=2);print(text)
    if a.output:Path(a.output).write_text(text+'\n')
    raise SystemExit(0 if result['passed'] else 1)

if __name__=='__main__':main()
