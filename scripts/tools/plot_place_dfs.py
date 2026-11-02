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
    route.set_aspect('equal',adjustable='box')
    route.set(xlim=(poses[:,0].min()-2,poses[:,0].max()+2),ylim=(poses[:,1].min()-2,poses[:,1].max()+2),
              xlabel='Odometry x (m)',ylabel='Odometry y (m)',title='Measured route and WI dispatch indices')
    route.grid(alpha=.15);route.legend(fontsize=8)
    # Tree edges represent physical Portal commits, never GP candidates alone.
    positions={};leaf=[0]
    def position(place,depth=0):
        children=nodes[place].get('children',[])
        if children:
            x=float(np.mean([position(c,depth+1) for c in children]))
        else:x=float(leaf[0]);leaf[0]+=1
        positions[place]=(x,-depth);return x
    roots=[p for p,n in nodes.items() if n.get('parent_id') is None]
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
    tree.set_xlim(-.6,max(leaf[0]-.4,.6));tree.set_ylim(min((v[1] for v in positions.values()),default=0)-.7,.65)
    tree.set_title('Physically committed Place tree\nSquare = completed; circle = unfinished',fontsize=11);tree.axis('off')
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
                 transformations=['Pose samples joined only across gaps <=2s','No synthetic route between WI goals',
                                  'Tree includes physically entered Places only','Odometry plotted over world geometry'],
                 missing_models=missing,matplotlib=matplotlib.__version__)
    args.output.with_suffix('.json').write_text(json.dumps(payload,indent=2)+'\n')
    print(args.output)

if __name__=='__main__':main()
