#!/usr/bin/env python3
"""Audit actual ROS metrics, never synthetic expectations in the detector."""
import argparse
import json
import math
from pathlib import Path


def check(path,expected,center,radius):
    trees=[];events=[]
    for line in Path(path).open():
        if ' data=' not in line:continue
        data=json.loads(line.split(' data=',1)[1])
        if 'tree' in data:trees.append(data['tree'])
        if data.get('frontier_event')=='gp_branch_selected':events.append(data)
    checks={}; details={}
    checks['live_tree_evidence']=bool(trees)
    confirmed=[]
    for tree in trees:
        for node in tree['nodes']:
            if node.get('junction_confirmed') and math.dist(node['physical_xy'],center)<=radius:
                confirmed.append((tree,node))
    latest=trees[-1] if trees else {}
    nodes=[n for n in latest.get('nodes',[]) if n.get('junction_confirmed')
           and math.dist(n['physical_xy'],center)<=radius]
    branches=[]
    if nodes:
        node=min(nodes,key=lambda n:math.dist(n['physical_xy'],center))
        branches=[b for b in latest['branches'] if b['node_id']==node['id']
                  and b['state']!='superseded' and b['stable']]
    def near(branch,heading):
        return abs(math.atan2(math.sin(branch['heading']-heading),
                             math.cos(branch['heading']-heading)))<0.45
    covered=bool(branches) and all(any(near(b,h) for b in branches) for h in expected)
    checks['expected_directions_present']=covered
    if covered:
        checks['no_duplicate_live_directions']=len(branches)==len(expected)
        checks['geometry_measured']=all(b.get('map_geometry',{}).get('width',0)>0
                                      and all(x is not None for x in b['map_geometry'].get('bounds',[None,None]))
                                      for b in branches)
        checks['pending_branch_preserved']=any(any(b['node_id']==n['id'] and b['state']=='pending'
                                                 for b in t['branches']) for t,n in confirmed)
        spaces={space['id']:space for space in latest.get('spaces',[])}
        outward=[b for b in branches if b.get('connection')!='return']
        checks['outward_branches_physically_entered']=bool(outward) and all(
              b.get('destination_verified') and b.get('gate_crossed') and
              b.get('destination_space_id') in spaces for b in outward)
        checks['child_space_recorded']=bool(outward) and all(
              spaces.get(b.get('destination_space_id'),{}).get('parent_branch_id')==b['id']
              or b.get('destination_node_id') is not None for b in outward)
        ids={n['id'] for t,n in confirmed}
        checks['junction_identity_stable']=len(ids)==1
        details=dict(node_id=node['id'],branches=[dict(id=b['id'],heading=b['heading'],
                  state=b['state'],width=b['map_geometry']['width'],gate=b.get('physical_gate'),
                  destination_space_id=b.get('destination_space_id'),
                  destination_verified=b.get('destination_verified',False)) for b in branches],
                  junction_ids=sorted(ids))
    return dict(passed=bool(checks) and all(checks.values()),checks=checks,details=details,
                source=str(path),selection_events=len(events))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('metrics');p.add_argument('--headings',nargs='+',type=float,required=True)
    p.add_argument('--center',nargs=2,type=float,default=[2,0]);p.add_argument('--radius',type=float,default=2)
    p.add_argument('--output');a=p.parse_args();r=check(a.metrics,a.headings,a.center,a.radius)
    text=json.dumps(r,indent=2);print(text)
    if a.output:Path(a.output).write_text(text+'\n')
    raise SystemExit(0 if r['passed'] else 1)
