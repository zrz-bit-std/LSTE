#!/usr/bin/env python3
"""Generate long T/L/cross corridors, real doorway-connected rooms and scene plans."""
from pathlib import Path
import json
from copy import deepcopy
import math
import xml.etree.ElementTree as E
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / 'worlds/targeted_navigation'
SPAWN = [-3.5, 0.0, 0.0]
EXTRA_CORRIDORS = []
ARMS = {'west': (-22,-14,-4,4), 'east': (12,20,-4,4),
        'north': (-4,4,12,20), 'south': (-4,4,-20,-12)}
CORRIDORS = {'west': (-14,0,-1.5,1.5), 'east': (0,12,-1.5,1.5),
             'north': (-1.5,1.5,0,12), 'south': (-1.5,1.5,-12,0)}
THEMES = {
 'office': [('desk_brown',-2,2,0,0),('desk_brown',2,2,0,0),
 ('office_chair',-2,-0.8,0,0),('office_chair',2,-0.8,0,0),
 ('monitor_3',-2,2.2,0.8,0),('monitor_2',2,2.2,0.8,0),
 ('keyboard',-2,1.8,0.8,0),('keyboard',2,1.8,0.8,0),
 ('mouse',-1.5,1.8,0.8,0),('computer_case',2.8,2,0,0),
 ('cup_yellow',-2.5,2,0.8,0),('cup_blue',2.5,2,0.8,0),
 ('bookshelf',-2.8,-2.6,0,0),('file_cabinet_large',2.8,-2.6,0,0),
 ('file_box_1',-1.8,-2.8,0,0),('plant_1_single',2.8,2.8,0,0)],
 'meeting': [('table_conference_2',0,0,0,0),
 ('office_chair',-1.6,-1,0,math.pi/2),('office_chair',-1.6,1,0,math.pi/2),
 ('office_chair',1.6,-1,0,-math.pi/2),('office_chair',1.6,1,0,-math.pi/2),
 ('office_chair',0,2.2,0,math.pi),('office_chair',-2.8,-2.2,0,0),
 ('cup_yellow',-0.5,0.5,0.85,0),('cup_blue',0.5,-0.5,0.85,0),
 ('file_1',0,0,0.85,0),('file_box_1',2.8,2.8,0,0),
 ('bookshelf',-2.8,2.8,0,0),('plant_1_single',2.8,-2.8,0,0)],
 'lounge': [('sofa_set_1',0,2.2,0,0),('coffee_table_1',0,0.5,0,0),
 ('cup_green',0,0.5,0.5,0),('cup_yellow',0.4,0.5,0.5,0),
 ('book_blue',-0.4,0.5,0.5,0),('bookshelf',-2.8,-2.8,0,0),
 ('office_chair',2.5,-1,0,0),('plant_1_single',2.8,2.8,0,0),
 ('plant_1_single',-2.8,2.8,0,0),('file_box_2',2.8,-2.8,0,0)],
 'breakroom': [('cafe_table',-1.7,1.8,0,0),('cafe_table',1.7,1.8,0,0),
 ('kitchen_chair',-2.5,1.8,0,math.pi/2),('kitchen_chair',-0.8,1.8,0,-math.pi/2),
 ('kitchen_chair',0.8,1.8,0,math.pi/2),('kitchen_chair',2.5,1.8,0,-math.pi/2),
 ('cup_green',-1.7,1.8,0.8,0),('cup_blue',1.7,1.8,0.8,0),
 ('file_cabinet_large',-2.8,-2.8,0,0),('file_box_2',2.8,-2.8,0,0),
 ('plant_1_single',2.8,2.8,0,0)]}
CASES = {'semantic_t_junction': [('west','lounge'),('east','office'),('north','meeting')],
         'semantic_l_junction': [('west','office'),('north','breakroom')],
         'semantic_cross_junction': [('west','lounge'),('east','office'),('north','meeting'),('south','breakroom')]}

def child(parent,tag,text=None,**attrs):
 n=E.SubElement(parent,tag,attrs)
 if text is not None:n.text=str(text)
 return n

def wall(link,name,segment):
 x1,y1,x2,y2=segment
 dx,dy=x2-x1,y2-y1
 length=math.hypot(dx,dy)
 for kind in ('collision','visual'):
  n=child(link,kind,name=name+'_'+kind)
  child(n,'pose',f'{(x1+x2)/2} {(y1+y2)/2} 1.5 0 0 {math.atan2(dy,dx)}')
  child(child(child(n,'geometry'),'box'),'size',f'{length+0.12} 0.12 3')
  if kind=='visual':
   m=child(n,'material');child(m,'ambient','0.82 0.85 0.89 1');child(m,'diffuse','0.82 0.85 0.89 1')

def boundaries(rects):
 xs=sorted({r[i] for r in rects for i in (0,1)})
 ys=sorted({r[i] for r in rects for i in (2,3)})
 cells={(i,j) for i in range(len(xs)-1) for j in range(len(ys)-1)
        if any(a<(xs[i]+xs[i+1])/2<b and c<(ys[j]+ys[j+1])/2<d for a,b,c,d in rects)}
 groups={}
 for i,j in cells:
  for neighbor,axis,fixed,lo,hi in [((i-1,j),'v',xs[i],ys[j],ys[j+1]),((i+1,j),'v',xs[i+1],ys[j],ys[j+1]),((i,j-1),'h',ys[j],xs[i],xs[i+1]),((i,j+1),'h',ys[j+1],xs[i],xs[i+1])]:
   if neighbor not in cells:groups.setdefault((axis,fixed),[]).append((lo,hi))
 result=[]
 for (axis,fixed),spans in sorted(groups.items()):
  merged=[]
  for lo,hi in sorted(spans):
   if merged and lo<=merged[-1][1]:merged[-1][1]=max(hi,merged[-1][1])
   else:merged.append([lo,hi])
  for lo,hi in merged:
   result.append((fixed,lo,fixed,hi) if axis=='v' else (lo,fixed,hi,fixed))
 return result

def generate(name,arms):
 base=E.parse(OUT/'t_junction_small.world').getroot()
 world=base.find('world');world.set('name',name)
 for n in list(world.findall('model')):
  if n.get('name')!='ground_plane':world.remove(n)
 # Large enough for all 8x8 rooms and long arms.
 for size in world.findall("./model[@name='ground_plane']//plane/size"):size.text='60 60'
 struct=child(world,'model',name='semantic_room_structure');child(struct,'static','true');link=child(struct,'link',name='walls')
 rects=[(-1.5,1.5,-1.5,1.5)]+[CORRIDORS[a] for a,t in arms]+[CORRIDORS[a] for a in EXTRA_CORRIDORS]+[ARMS[a] for a,t in arms]
 segments=boundaries(rects)
 for a,t in arms:
  if a in ('west','east'):
   x=ARMS[a][1] if a=='west' else ARMS[a][0]
   segments += [(x,-1.5,x,-1.1),(x,1.1,x,1.5)]
  else:
   y=ARMS[a][2] if a=='north' else ARMS[a][3]
   segments += [(-1.5,y,-1.1,y),(1.1,y,1.5,y)]
 for i,seg in enumerate(segments):wall(link,f'wall_{i}',seg)
 objects=[];rooms=[]
 for arm,theme in arms:
  x0,x1,y0,y1=ARMS[arm];cx=(x0+x1)/2;cy=(y0+y1)/2
  rooms.append(dict(arm=arm,theme=theme,bounds=[x0,x1,y0,y1],door_width=2.2))
  for i,(model,dx,dy,z,yaw) in enumerate(THEMES[theme]):
   assert (Path.home()/'.gazebo/models'/model/'model.sdf').is_file(),model
   instance=f'{arm}_{theme}_{model}_{i}'
   n=deepcopy(E.parse(Path.home()/'.gazebo/models'/model/'model.sdf').getroot().find('model'))
   n.set('name',instance)
   for old in list(n.findall('pose'))+list(n.findall('static')):n.remove(old)
   child(n,'pose',f'{cx+dx} {cy+dy} {z} 0 0 {yaw}');child(n,'static','true');world.append(n)
   objects.append(dict(name=instance,model=model,room=arm,xy=[cx+dx,cy+dy],z=z))
 E.ElementTree(base).write(OUT/(name+'.world'),encoding='utf-8',xml_declaration=True)
 corridor_lengths={}
 for arm,_theme in arms:
  x0,x1,y0,y1=CORRIDORS[arm]
  corridor_lengths[arm]=abs(x1-x0) if arm in ('west','east') else abs(y1-y0)
 manifest=dict(scene_version=2,world=name,central_junction=[0,0],spawn=list(SPAWN),corridor_only_arms=list(EXTRA_CORRIDORS),corridor_width=3,arm_corridor_lengths=corridor_lengths,rooms=rooms,objects=objects,physical_furniture_collisions=True,floor_rectangles=rects)
 (OUT/(name+'.json')).write_text(json.dumps(manifest,indent=2)+'\n')
 fig,ax=plt.subplots(figsize=(12,12))
 for a,b,c,d in rects:ax.add_patch(Rectangle((a,c),b-a,d-c,fc='#edf2f5',ec='none'))
 for x1,y1,x2,y2 in segments:ax.plot([x1,x2],[y1,y2],c='#253747',lw=2)
 for room in rooms:
  a,b,c,d=room['bounds'];ax.text((a+b)/2,d-0.5,room['arm']+' / '+room['theme'],ha='center',fontsize=10)
 for ob in objects:
  x,y=ob['xy'];ax.scatter(x,y,s=22);ax.annotate(ob['model'],(x,y),xytext=(3,3),textcoords='offset points',fontsize=5)
 ax.scatter(-3.5,0,marker='>',s=70,c='red',label='robot spawn')
 ax.scatter(0,0,marker='x',s=65,c='black',label='junction')
 ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('x [m]');ax.set_ylabel('y [m]')
 ax.set_title(f'{name}: long corridors + 8 x 8 m rooms\n{len(rooms)} rooms / {len(objects)} physical semantic objects; 2.2 m doorways')
 ax.legend();fig.tight_layout();fig.savefig(OUT/(name+'_layout.png'),dpi=180);plt.close(fig)
 validate_scene(name,manifest)
 print(name,'rooms',len(rooms),'objects',len(objects),'walls',len(segments))
 assert all(len([o for o in objects if o['room']==r['arm']])>=10 for r in rooms)


def validate_scene(name,manifest):
 import sys
 from collections import deque
 import numpy as np
 from matplotlib.path import Path as PolygonPath
 sys.path.insert(0,str(ROOT/'scripts/tools'))
 from plot_world_run import read_scene,draw_scene
 shapes,_,missing=read_scene(OUT/(name+'.world'),[Path.home()/'.gazebo/models',ROOT/'worlds'])
 objects={o['name']:o for o in manifest['objects']}
 rooms={r['arm']:r['bounds'] for r in manifest['rooms']}
 outside=[]
 for shape in shapes:
  if shape['model'] in objects:
   a,b,c,d=rooms[objects[shape['model']]['room']]
   if any(x<a or x>b or y<c or y>d for x,y in shape['xy']):outside.append(shape['model'])
 res=.2
 xs=np.arange(-24,22,res);ys=np.arange(-22,22,res)
 xx,yy=np.meshgrid(xs,ys);points=np.column_stack((xx.ravel(),yy.ravel()))
 free=np.zeros(xx.shape,dtype=bool)
 for a,b,c,d in manifest['floor_rectangles']:free|=(xx>=a)&(xx<=b)&(yy>=c)&(yy<=d)
 for shape in shapes:free &= ~PolygonPath(shape['xy']).contains_points(points).reshape(xx.shape)
 safe=free.copy()
 for di in range(-2,3):
  for dj in range(-2,3):
   if di*di+dj*dj<=4:safe &= np.roll(free,(di,dj),axis=(0,1))
 free=safe
 def cell(x,y):return int(round((y-ys[0])/res)),int(round((x-xs[0])/res))
 spawn=manifest.get("spawn",[-3.5,0,0])
 start=cell(spawn[0],spawn[1]);seen={start};queue=deque([start])
 while queue:
  i,j=queue.popleft()
  for nxt in ((i-1,j),(i+1,j),(i,j-1),(i,j+1)):
   a,b=nxt
   if 0<=a<free.shape[0] and 0<=b<free.shape[1] and free[a,b] and nxt not in seen:seen.add(nxt);queue.append(nxt)
 targets={}
 reachable={}
 for arm,(x0,x1,y0,y1) in rooms.items():
  if arm=='west':
   targets[arm]=[(x1-depth,y) for depth in np.arange(.6,2.6,.2) for y in np.arange(-.6,.61,.2)]
  elif arm=='east':
   targets[arm]=[(x0+depth,y) for depth in np.arange(.6,2.6,.2) for y in np.arange(-.6,.61,.2)]
  elif arm=='north':
   targets[arm]=[(x,y0+depth) for depth in np.arange(.6,2.6,.2) for x in np.arange(-.6,.61,.2)]
  else:
   targets[arm]=[(x,y1-depth) for depth in np.arange(.6,2.6,.2) for x in np.arange(-.6,.61,.2)]
  reachable[arm]=any(cell(*target) in seen for target in targets[arm])
 checks=dict(models_resolved=not missing,all_objects_inside_rooms=not outside,
             every_room_doorway_reachable_with_040m_clearance=all(reachable.values()),
             at_least_10_objects_per_room=all(sum(o['room']==arm for o in objects.values())>=10 for arm in rooms))
 result=dict(passed=all(checks.values()),checks=checks,reachable_room_doors=reachable,
             missing_models=missing,outside_room_objects=sorted(set(outside)))
 (OUT/(name+'_geometry_result.json')).write_text(json.dumps(result,indent=2)+'\n')
 assert result['passed'],result
 fig,ax=plt.subplots(figsize=(13,11));draw_scene(ax,shapes)
 for arm,(a,b,c,d) in rooms.items():ax.text((a+b)/2,d-.4,arm+' / '+next(r['theme'] for r in manifest['rooms'] if r['arm']==arm),ha='center',fontsize=9)
 ax.scatter(manifest.get("spawn",[-3.5,0])[0],manifest.get("spawn",[-3.5,0])[1],c='#d74c3c',marker='>',s=80,zorder=10,label='spawn')
 ax.scatter(0,0,c='#24759c',marker='x',s=80,zorder=10,label='junction')
 ax.set_title(name+': physical furniture / 8 x 8 m rooms / 12-14 m corridors')
 ax.legend();fig.tight_layout();fig.savefig(OUT/(name+'_scene.png'),dpi=180);plt.close(fig)

if __name__=='__main__':
 for name,arms in CASES.items():generate(name,arms)
