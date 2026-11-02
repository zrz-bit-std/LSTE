#!/usr/bin/env python3
"""Additional physical nested-fork regression; never overwrites the four main worlds."""
from copy import deepcopy
from collections import deque
import json
import xml.etree.ElementTree as E
import numpy as np
from matplotlib.path import Path as PolygonPath
from matplotlib.patches import Polygon
from generate_semantic_room_worlds import ROOT,OUT,THEMES,child,wall,boundaries
import sys
sys.path.insert(0,str(ROOT/'scripts/tools'))
from plot_world_run import read_scene,draw_scene
import matplotlib.pyplot as plt


def main():
    name='semantic_nested_place_dfs'
    base=E.parse(OUT/'semantic_t_junction.world').getroot();world=base.find('world');world.set('name',name)
    for model in list(world.findall('model')):
        if model.get('name')!='ground_plane':world.remove(model)
    corridors=[(-12.,16.,-1.5,1.5),(-1.5,1.5,0.,10.),(8.5,11.5,0.,10.)]
    rooms=[dict(name='west_lounge',theme='lounge',bounds=[-20.,-12.,-4.,4.],door=[-12.,0.],target=[-13.,0.]),
           dict(name='north_meeting',theme='meeting',bounds=[-4.,4.,10.,18.],door=[0.,10.],target=[0.,11.]),
           dict(name='east_office',theme='office',bounds=[16.,24.,-4.,4.],door=[16.,0.],target=[17.,0.]),
           dict(name='nested_breakroom',theme='breakroom',bounds=[6.,14.,10.,18.],door=[10.,10.],target=[10.,11.])]
    rects=corridors+[r['bounds'] for r in rooms];segments=boundaries(rects)
    for room in rooms:
        x,y=room['door']
        if y==0:segments.extend([(x,-1.5,x,-1.1),(x,1.1,x,1.5)])
        else:segments.extend([(x-1.5,y,x-1.1,y),(x+1.1,y,x+1.5,y)])
    structure=child(world,'model',name='nested_place_structure');child(structure,'static','true');link=child(structure,'link',name='walls')
    for i,segment in enumerate(segments):wall(link,'wall_%s'%i,segment)
    objects=[]
    for room in rooms:
        a,b,c,d=room['bounds'];cx,cy=(a+b)/2,(c+d)/2
        for index,(model,dx,dy,z,yaw) in enumerate(THEMES[room['theme']]):
            source=ROOT/'worlds/targeted_navigation/models'/model/'model.sdf'
            if not source.is_file():source=__import__('pathlib').Path.home()/'.gazebo/models'/model/'model.sdf'
            model_node=deepcopy(E.parse(source).getroot().find('model'))
            instance='%s_%s_%s'%(room['name'],model,index);model_node.set('name',instance)
            for old in list(model_node.findall('pose'))+list(model_node.findall('static')):model_node.remove(old)
            child(model_node,'pose','%s %s %s 0 0 %s'%(cx+dx,cy+dy,z,yaw));child(model_node,'static','true');world.append(model_node)
            objects.append(dict(name=instance,model=model,room=room['name'],xy=[cx+dx,cy+dy]))
    path=OUT/(name+'.world');E.ElementTree(base).write(path,encoding='utf-8',xml_declaration=True)
    from pathlib import Path
    shapes,_,missing=read_scene(path,[OUT/'models',Path.home()/'.gazebo/models',ROOT/'worlds'])
    resolution=.2;xs=np.arange(-22,26,resolution);ys=np.arange(-6,20,resolution)
    xx,yy=np.meshgrid(xs,ys);points=np.column_stack((xx.ravel(),yy.ravel()));free=np.zeros(xx.shape,dtype=bool)
    for a,b,c,d in rects:free|=(xx>a)&(xx<b)&(yy>c)&(yy<d)
    for shape in shapes:free&=~PolygonPath(shape['xy']).contains_points(points).reshape(free.shape)
    safe=free.copy();padded=np.pad(free,2,constant_values=False)
    for dy in range(-2,3):
        for dx in range(-2,3):
            if np.hypot(dx,dy)*resolution<=.4+1e-8:safe&=padded[2+dy:2+dy+len(ys),2+dx:2+dx+len(xs)]
    def cell(point):return int(round((point[1]-ys[0])/resolution)),int(round((point[0]-xs[0])/resolution))
    start=cell((-3.5,0));seen={start};queue=deque([start])
    while queue:
        row,col=queue.popleft()
        for r,c in ((row-1,col),(row+1,col),(row,col-1),(row,col+1)):
            if 0<=r<safe.shape[0] and 0<=c<safe.shape[1] and safe[r,c] and (r,c) not in seen:
                seen.add((r,c));queue.append((r,c))
    checks=dict(models_resolved=not missing,spawn_has_040m_clearance=bool(safe[start]),
                all_four_room_entrances_reachable=all(cell(r['target']) in seen for r in rooms))
    manifest=dict(world=name,spawn=[-3.5,0,0],junctions=[[0,0],[10,0]],rooms=rooms,
                  floor_rectangles=rects,objects=objects,expected_minimum_physical_dfs_depth=3,
                  geometry_checks=checks,passed=all(checks.values()),missing_models=missing)
    (OUT/(name+'.json')).write_text(json.dumps(manifest,indent=2)+'\n')
    fig,ax=plt.subplots(figsize=(14,9));draw_scene(ax,shapes)
    for shape in shapes:
        if shape.get('kind')=='wall':ax.add_patch(Polygon(shape['xy'],closed=True,facecolor='#39434B',edgecolor='#39434B',lw=.3,zorder=3))
    for room in rooms:
        a,b,c,d=room['bounds'];ax.text((a+b)/2,d-.5,room['name'],ha='center',fontsize=9)
    for index,(x,y) in enumerate(manifest['junctions']):ax.scatter(x,y,marker='x',s=90,c='black');ax.annotate('J%s'%index,(x,y),xytext=(4,7),textcoords='offset points')
    ax.scatter(-3.5,0,marker='>',s=90,c='#0072B2');ax.set_title('Additional physical nested DFS world — layout validation only')
    ax.set_aspect('equal');fig.tight_layout();fig.savefig(OUT/(name+'_layout.png'),dpi=160);plt.close(fig)
    print(json.dumps(checks));assert manifest['passed'],manifest

if __name__=='__main__':main()
