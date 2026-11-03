#!/usr/bin/env python3
"""Audit actual ROS metrics, never synthetic expectations in the detector."""
import argparse
import json
import math
from pathlib import Path



def verified_return_direction(branch, portals):
    """Count only a physically bound reverse edge with positive SLAM evidence."""
    if (branch.get('connection') != 'return'
            or branch.get('authentication_state') != 'deduplicated'):
        return False
    portal = portals.get(branch.get('canonical_portal_id'))
    proof = branch.get('return_portal_evidence') or {}
    if portal is None:
        return False
    try:
        if (portal.get('state') != 'crossed'
                or int(portal.get('crossing_count', 0)) < 1
                or portal.get('gp_authentication_state') != 'completed'
                or portal['source_place_id'] == portal['destination_place_id']
                or branch.get('return_source_place_id') != portal['destination_place_id']
                or branch.get('return_destination_place_id') != portal['source_place_id']
                or proof.get('reason') != 'visited_entry_side_of_current_place'
                or proof.get('duplicate_portal_id') != portal['id']
                or proof.get('parent_place_id') != portal['source_place_id']
                or int(proof.get('source_label', 0)) <= 0
                or int(proof.get('destination_label', 0)) <= 0
                or proof['source_label'] == proof['destination_label']):
            return False
        gate, destination = portal['physical_gate'], portal['physical_destination']
        dx, dy = destination[0]-gate[0], destination[1]-gate[1]
        length = math.hypot(dx, dy)
        heading = float(branch['heading'])
        return length > 1e-6 and (math.cos(heading)*dx+math.sin(heading)*dy)/length < -.5
    except (KeyError, TypeError, ValueError):
        return False


def check(path,expected,center,radius,heading_tolerance=0.45):
    trees=[];events=[];latest_tree=None;latest_portal_records={};same_place_rejections=set()
    damaged=0
    with Path(path).open(encoding="utf-8", errors="replace") as stream:
      for line in stream:
          if ' data=' not in line:continue
          try:
              data=json.loads(line.split(' data=',1)[1])
          except ValueError:
              damaged+=1
              if damaged>3:raise
              continue
          if data.get('tree'):
              latest_tree=data['tree']
              # Historical checks use only confirmed node identities and
              # pending states. Keep the full geometry only for latest_tree.
              nodes=[n for n in latest_tree.get('nodes',[]) if n.get('junction_confirmed')]
              ids={n['id'] for n in nodes}
              trees.append(dict(nodes=[{k:n[k] for k in ('id','junction_confirmed','physical_xy') if k in n} for n in nodes],
                                branches=[{k:b[k] for k in ('id','node_id','state') if k in b}
                                          for b in latest_tree.get('branches',[]) if b.get('node_id') in ids]))
          portal_snapshot=data.get('portal_hypotheses') or {}
          if portal_snapshot.get('records') is not None:
              latest_portal_records={
                  int(record.get('id')):record
                  for record in portal_snapshot.get('records',[])
                  if record.get('id') is not None
              }
          if data.get('frontier_event')=='gp_branch_selected':events.append(data.get('branch_id'))
          if (data.get('frontier_event') == 'portal_candidate_rejected'
                  and data.get('reason') == 'known_free_bypass_within_current_place'):
              spatial=data.get('spatial_evidence') or {}
              auth=data.get('portal_authentication') or {}
              try:
                  source=int(spatial.get('source_label',0));destination=int(spatial.get('destination_label',0))
                  entry=int(spatial.get('entry_label',0))
              except (TypeError,ValueError):
                  continue
              if source>0 and source==destination==entry:
                  for branch_record in auth.get('branches',[]):
                      names=[item.get('event') for item in branch_record.get('events',[])]
                      branch_id=branch_record.get('branch_id')
                      if (branch_id is not None
                              and branch_record.get('failure_reason')=='known_free_bypass_within_current_place'
                              and branch_record.get('source_side_proven') is True
                              and branch_record.get('return_gate_cleared') is True
                              and 'portal_candidate_rejected' in names):
                          same_place_rejections.add(int(branch_id))
    checks={}; details={}
    checks['live_tree_evidence']=bool(trees)
    confirmed=[]
    for tree in trees:
        for node in tree['nodes']:
            if node.get('junction_confirmed') and math.dist(node['physical_xy'],center)<=radius:
                confirmed.append((tree,node))
    latest=latest_tree or {}
    nodes=[n for n in latest.get('nodes',[]) if n.get('junction_confirmed')
           and math.dist(n['physical_xy'],center)<=radius]
    branches=[]
    if nodes:
        node=min(nodes,key=lambda n:math.dist(n['physical_xy'],center))
        branches=[b for b in latest['branches'] if b['node_id']==node['id']
                  and (b['state'] not in ('superseded','authentication_failed')
                       or int(b.get('id',-1)) in same_place_rejections
                       or verified_return_direction(b, latest_portal_records))
                  and b['stable']]
    def near(branch,heading):
        return abs(math.atan2(math.sin(branch['heading']-heading),
                             math.cos(branch['heading']-heading)))<heading_tolerance
    covered=bool(branches) and all(any(near(b,h) for b in branches) for h in expected)
    checks['expected_directions_present']=covered
    if covered:
        checks['no_duplicate_live_directions']=len(branches)==len(expected)
        checks['geometry_measured']=all(float(b.get('map_geometry',{}).get('width') or 0)>0
                                      and all(x is not None for x in b['map_geometry'].get('bounds',[None,None]))
                                      for b in branches)
        checks['pending_branch_preserved']=any(any(b['node_id']==n['id'] and b['state']=='pending'
                                                 for b in t['branches']) for t,n in confirmed)
        spaces={space['id']:space for space in latest.get('spaces',[])}
        local_continuations=[b for b in branches if int(b.get('id',-1)) in same_place_rejections]
        checks['same_place_continuations_have_no_child_place']=all(
            b.get('canonical_portal_id') is None and b.get('room_place_id') is None
            and bool(b.get('room_invalidated')) for b in local_continuations)
        outward=[b for b in branches if b.get('connection')!='return'
                 and int(b.get('id',-1)) not in same_place_rejections]
        def crossed_place_for(branch):
            portal_id=branch.get('canonical_portal_id')
            portal=latest_portal_records.get(int(portal_id)) if portal_id is not None else None
            place_id=branch.get('room_place_id')
            if portal is None or place_id is None:
                return None
            try:
                if (str(portal.get('state')) != 'crossed'
                    or int(portal.get('destination_place_id')) != int(place_id)
                    or int(portal.get('source_place_id')) == int(place_id)
                    or int(portal.get('crossing_count',0)) < 1):
                    return None
            except (TypeError,ValueError):
                return None
            return portal
        entered=[(branch,crossed_place_for(branch)) for branch in outward]
        checks['outward_branches_physically_entered']=bool(outward) and all(
              portal is not None for _branch,portal in entered)
        checks['child_space_recorded']=bool(outward) and all(
              portal is not None
              and int(portal['source_place_id']) != int(portal['destination_place_id'])
              for _branch,portal in entered)
        ids={n['id'] for t,n in confirmed}
        checks['junction_identity_stable']=len(ids)==1
        details=dict(node_id=node['id'],branches=[dict(id=b['id'],heading=b['heading'],
                  state=b['state'],width=b['map_geometry']['width'],gate=b.get('physical_gate'),
                  canonical_portal_id=b.get('canonical_portal_id'),
                  destination_place_id=b.get('room_place_id'),
                  destination_verified=crossed_place_for(b) is not None) for b in branches],
                  junction_ids=sorted(ids),
                  resolved_same_place_branches=sorted(same_place_rejections))
    return dict(passed=bool(checks) and all(checks.values()),checks=checks,details=details,
                source=str(path),selection_events=len(events))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('metrics');p.add_argument('--headings',nargs='+',type=float,required=True)
    p.add_argument('--center',nargs=2,type=float,default=[2,0]);p.add_argument('--radius',type=float,default=2);p.add_argument('--heading-tolerance',type=float,default=0.45)
    p.add_argument('--output');a=p.parse_args();r=check(a.metrics,a.headings,a.center,a.radius,a.heading_tolerance)
    text=json.dumps(r,indent=2);print(text)
    if a.output:Path(a.output).write_text(text+'\n')
    raise SystemExit(0 if r['passed'] else 1)
