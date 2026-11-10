#!/usr/bin/env python3
"""Plot measured motion, physical Place tree, and actual WI/DFS transactions."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon
import numpy as np
from plot_portal_authentication import read_metrics, ROOT
from plot_world_run import read_scene
sys.path.insert(0,str(ROOT/'scripts/tests/targeted_navigation'))
from audit_place_dfs import audit

COLORS=['#0072B2','#D55E00','#009E73','#8B5597','#9B7100','#333333']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('metrics',type=Path);parser.add_argument('--world',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--min-stack-depth',type=int,default=3)
    args=parser.parse_args()
    events,samples,_,_=read_metrics(args.metrics)
    if not events or not samples:raise SystemExit('Measured events and pose samples are required')
    result=audit(args.metrics,args.min_stack_depth)
    latest=events[-1][1];dfs=latest.get('place_dfs') or {}
    nodes={n['place_id']:n for n in dfs.get('nodes',[])}
    portals={p['id']:p for p in (latest.get('portal_hypotheses') or {}).get('records',[])}
    # Destinations that were physically certified but never entered.  Newer
    # logs carry the durable reason; older logs are reconstructed from the
    # Portal ledger so the same figure still works on sealed runs.
    observed_rows=list(dfs.get('observed') or [])
    if not observed_rows:
        for portal in portals.values():
            destination=portal.get('destination_place_id')
            try:destination=int(destination)
            except (TypeError,ValueError):continue
            if destination in nodes:continue
            xy=portal.get('physical_destination') or portal.get('map_destination')
            if xy is None:continue
            observed_rows.append(dict(portal_id=portal.get('id'),
                source_place_id=portal.get('source_place_id'),
                destination_place_id=destination,reason='',
                semantic_score=portal.get('semantic_score'),
                heading_delta_deg=portal.get('defer_heading_delta_deg'),
                gate=portal.get('physical_gate'),destination=xy))
    colors={p:COLORS[i%len(COLORS)] for i,p in enumerate(sorted(nodes))}
    dispatches=[];transitions=[]
    for event,d in events:
        if event=='work_item_dispatched':
            item=next((w for w in d.get('work_items',[]) if w.get('id')==d.get('work_item_id')), {})
            attempt=next((a for a in item.get('attempts',[]) if a.get('id')==d.get('attempt_id')), {})
            dispatches.append(dict(index=len(dispatches)+1,time=d.get('ros_time'),place=d.get('place_id'),
                                   wi=d.get('work_item_id'),xy=attempt.get('viewpoint_xy')))
        if event in ('place_dfs_entered','place_dfs_returned','place_dfs_completion_ready'):
            transitions.append(dict(event=event,time=d.get('ros_time'),place=d.get('place_id'),
                                    parent=d.get('parent_place_id'),portal=d.get('portal_id'),
                                    stack=(d.get('place_dfs') or {}).get('stack',[])))
    fig=plt.figure(figsize=(15,10),constrained_layout=True,facecolor='white')
    grid=fig.add_gridspec(2,2,height_ratios=[3,1.4],width_ratios=[1.5,1])
    route,tree,timeline=fig.add_subplot(grid[0,0]),fig.add_subplot(grid[0,1]),fig.add_subplot(grid[1,:])
    shapes,_,missing=read_scene(args.world,[ROOT/'worlds/targeted_navigation/models',Path.home()/'.gazebo/models',ROOT/'worlds'])
    for shape in shapes:
        if shape.get('kind')=='wall':
            route.add_patch(Polygon(shape['xy'],closed=True,facecolor='#39434B',lw=0.3,zorder=1))
    poses=np.asarray([s['pose'][:2] for s in samples]);times=np.asarray([s['ros_time'] for s in samples])
    segments=np.stack([poses[:-1],poses[1:]],axis=1)
    valid=np.isfinite(times[:-1])&np.isfinite(times[1:])&((times[1:]-times[:-1])<=2.0)&(times[1:]>=times[:-1])
    route.add_collection(LineCollection(segments[valid],colors='#75889A',linewidths=1.1,zorder=2))
    route.scatter(*poses[0],c='black',marker='o',s=45,zorder=4,label='Start')
    route.scatter(*poses[-1],c='black',marker='x',s=50,zorder=4,label='Last measured pose')
    for p in portals.values():
        xy=p.get('physical_gate')
        if xy:
            route.scatter(*xy,marker='D',facecolors='white',edgecolors='black',s=40,zorder=5)
            route.annotate('G%s'%p['id'],xy,xytext=(4,-10),textcoords='offset points',fontsize=8)
    for d in dispatches:
        if d['xy'] is None:continue
        route.scatter(*d['xy'],marker='s',s=28,c=colors.get(d['place'],'#333333'),zorder=5)
        route.annotate(str(d['index']),d['xy'],xytext=(3,3),textcoords='offset points',fontsize=7)
    # The Place tree alone loses where each committed Place is on the route.
    # Label it at the durable Portal destination used for the crossing (the
    # root Place uses the first measured pose).  Only physically entered
    # Places in the DFS snapshot are labeled; pre-created but unvisited
    # destinations stay off the route panel.
    roots={p for p,n in nodes.items() if n.get('parent_id') is None}
    place_xy={}
    for p in nodes:
        if p in roots and len(poses):
            place_xy[p]=(float(poses[0][0]),float(poses[0][1]))
            continue
        edge=portals.get(nodes[p].get('portal_id'))
        if not isinstance(edge,dict):continue
        xy=edge.get('physical_destination') or edge.get('map_destination')
        if xy is None:continue
        place_xy[p]=(float(xy[0]),float(xy[1]))
    # Semantic destinations created by GP doorway observation but not yet
    # committed by a DFS crossing.  They exist in Place memory and own a
    # destination witness, so omitting them makes an observed room look
    # unexplored.  Draw them dashed and neutral, never as committed Places.
    observed_xy={}
    for row in observed_rows:
        try:destination=int(row.get('destination_place_id'))
        except (TypeError,ValueError):continue
        if destination in nodes or destination in roots:continue
        xy=row.get('destination')
        if xy is None:continue
        observed_xy[destination]=(float(xy[0]),float(xy[1]))
    for index,p in enumerate(sorted(observed_xy)):
        xy=observed_xy[p]
        near=min((float(np.hypot(xy[0]-cx,xy[1]-cy)) for cx,cy in place_xy.values()),
                 default=99.0)
        route.scatter(*xy,s=280,facecolors='none',edgecolors='#7c8798',
                      linewidths=1.3,linestyles='--',zorder=6,
                      label='Observed destination (not committed yet)' if index==0 else None)
        route.annotate('P%d obs'%p,xy,xytext=(-44,-16) if near<0.8 else (8,-13),
                       textcoords='offset points',fontsize=8.5,style='italic',
                       color='#5b6675',zorder=7,
                       bbox=dict(boxstyle='round,pad=.16',facecolor='white',
                                 edgecolor='none',alpha=.85))
    for index,p in enumerate(sorted(place_xy)):
        xy=place_xy[p]
        route.scatter(*xy,s=300,facecolors='none',edgecolors=colors.get(p,'#333333'),
                      linewidths=2.0,zorder=8,label='Committed Place' if index==0 else None)
        route.annotate('P%d'%p,xy,xytext=(8,7),textcoords='offset points',fontsize=10,
                       weight='bold',color=colors.get(p,'#333333'),zorder=9,
                       bbox=dict(boxstyle='round,pad=.2',facecolor='white',
                                 edgecolor='none',alpha=.88))
    route.set_aspect('equal',adjustable='box')
    route.set(xlim=(poses[:,0].min()-3,poses[:,0].max()+3),ylim=(poses[:,1].min()-3,poses[:,1].max()+3),
              xlabel='Odometry x (m)',ylabel='Odometry y (m)',
              title='Measured route, committed Places and WI dispatch indices')
    route.grid(alpha=.15);route.legend(fontsize=8,loc="upper right")
    # Tree edges represent physical Portal commits, never GP candidates alone.
    positions={};leaf=[0]
    def position(place,depth=0):
        children=nodes[place].get('children',[])
        if children:
            x=float(np.mean([position(c,depth+1) for c in children]))
        else:x=float(leaf[0]);leaf[0]+=1
        positions[place]=(x,-depth);return x
    roots=[p for p,n in nodes.items() if n.get('parent_id') is None]
    observed_by_parent={}
    for row in observed_rows:
        try:destination=int(row.get('destination_place_id'))
        except (TypeError,ValueError):continue
        try:source=int(row.get('source_place_id'))
        except (TypeError,ValueError):continue
        if destination in nodes:continue
        observed_by_parent.setdefault(source,[]).append((destination,row))
    observed_positions={}
    def position_observed(destination,depth):
        # An observed destination is a leaf: it was never entered, so it owns
        # no subtree.  Giving it the same depth as a committed child exposes
        # the missed room instead of leaving it implicit.
        if destination in observed_positions:return observed_positions[destination][0]
        x=float(leaf[0]);leaf[0]+=1
        observed_positions[destination]=(x,-depth)
        return x
    def position(place,depth=0):
        children=list(nodes[place].get('children',[]))
        extra=[destination for destination,_ in sorted(observed_by_parent.get(place,[]))]
        if children or extra:
            xs=[position(child,depth+1) for child in children]
            xs+=[position_observed(destination,depth+1) for destination in extra]
            x=float(np.mean(xs))
        else:x=float(leaf[0]);leaf[0]+=1
        positions[place]=(x,-depth);return x
    for p in roots:position(p)
    for p,n in nodes.items():
        x,y=positions[p];parent=n.get('parent_id')
        if parent in positions:
            a,b=positions[parent]
            tree.annotate('',xy=(x,y+.12),xytext=(a,b-.12),arrowprops=dict(arrowstyle='->',color='#555555'))
            edge=portals.get(n.get('portal_id'),{})
            text='G%s'%n.get('portal_id')
            if edge.get('semantic_score') is not None:text+=' / score %.2f'%edge['semantic_score']
            tree.text((a+x)/2+.03,(b+y)/2,text,fontsize=8)
        tree.scatter(x,y,s=850,facecolors=colors[p],edgecolors='black',marker='s' if n['state']=='completed' else 'o',zorder=3)
        tree.text(x,y,'P%s'%p,color='black',ha='center',va='center',fontsize=10,weight='bold')
        tree.text(x,y-.22,n['state'],ha='center',va='top',fontsize=8)
    for source,rows in sorted(observed_by_parent.items()):
        if source not in positions:continue
        a,b=positions[source]
        for destination,row in sorted(rows):
            if destination not in observed_positions:continue
            x,y=observed_positions[destination]
            tree.annotate('',xy=(x,y+.12),xytext=(a,b-.12),
                          arrowprops=dict(arrowstyle='->',color='#9AA0A6',
                                          linestyle=(0,(2,2))))
            label='G%s'%row.get('portal_id')
            if row.get('semantic_score') is not None:label+=' / %.2f'%float(row['semantic_score'])
            if row.get('reason'):label+='\n%s'%row['reason']
            if row.get('heading_delta_deg') is not None:label+=' %.0f\u00b0'%float(row['heading_delta_deg'])
            tree.text(x+.12,y+.22,label,fontsize=7,color='#6B7280',style='italic',
                      ha='left',va='bottom')
            tree.scatter(x,y,s=760,facecolors='white',edgecolors='#9AA0A6',marker='s',
                         linewidths=1.6,zorder=3)
            tree.text(x,y,'P%s\nobs'%destination,color='#6B7280',ha='center',
                      va='center',fontsize=8,weight='bold')
    tree.set_xlim(-.6,max(leaf[0]-.4,.6));tree.set_ylim(min((v[1] for v in positions.values()),default=0)-.7,.65)
    tree.legend(handles=[plt.Line2D([],[],marker='s',linestyle='',markerfacecolor=COLORS[0],
                                    markeredgecolor='black',markersize=8,label='Committed Place'),
                         plt.Line2D([],[],marker='s',linestyle='',markerfacecolor='white',
                                    markeredgecolor='#9AA0A6',markersize=8,
                                    label='Observed, not committed')],
                loc='lower left',fontsize=7,frameon=False)
    tree.set_title('Physically committed Place tree\n'
                   'Square = completed; circle = unfinished; grey = observed, not committed',
                   fontsize=10);tree.axis('off')
    for p in sorted(nodes):
        rows=[d for d in dispatches if d['place']==p]
        timeline.scatter([d['time'] for d in rows],[p]*len(rows),marker='s',s=30,color=colors[p],label='P%s WI'%p)
        for d in rows:timeline.annotate('W%s'%d['wi'],(d['time'],p),xytext=(2,5),textcoords='offset points',fontsize=7,rotation=35)
    symbols={'place_dfs_entered':('v','enter'),'place_dfs_returned':('^','return'),'place_dfs_completion_ready':('x','ready')}
    for t in transitions:
        marker,label=symbols[t['event']]
        timeline.scatter(t['time'],t['place'],marker=marker,s=80,c='black',zorder=5)
        timeline.annotate(label,(t['time'],t['place']),xytext=(0,-15),textcoords='offset points',ha='center',fontsize=8)
    timeline.set(xlabel='ROS simulation time (s)',ylabel='Place ID',yticks=sorted(nodes),title='Actual WI dispatch and DFS transactions (retries retained)')
    timeline.set_ylim(min(nodes,default=1)-.5,max(nodes,default=1)+.6)
    timeline.grid(alpha=.15);timeline.legend(loc='upper left',fontsize=8,ncol=4)
    status='PASS' if result['passed'] else 'INCOMPLETE / checks unmet'
    fig.suptitle('Place-local TSP / DFS — '+status,fontsize=15)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,dpi=160,facecolor='white');plt.close(fig)
    payload=dict(metrics=str(args.metrics),metrics_sha256=hashlib.sha256(args.metrics.read_bytes()).hexdigest(),
                 world=str(args.world),audit=result,dispatches=dispatches,transitions=transitions,
                 observed=observed_rows,
                 transformations=['Pose samples joined only across gaps <=2s','No synthetic route between WI goals',
                                  'Tree nodes are physically entered Places only; grey nodes are observed destinations that were never committed',
                                  'Uncommitted reasons come from the Portal ledger, not from post-hoc inference',
                                  'Odometry plotted over world geometry'],
                 missing_models=missing,matplotlib=matplotlib.__version__)
    args.output.with_suffix('.json').write_text(json.dumps(payload,indent=2)+'\n')
    print(args.output)

if __name__=='__main__':main()
