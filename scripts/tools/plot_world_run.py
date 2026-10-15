#!/usr/bin/env python3
"""Render a saved SDF scene and measured run without requiring ROS or Gazebo."""
import argparse
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
import numpy as np


def pose_matrix(text):
    v = [float(x) for x in (text or '0 0 0 0 0 0').split()]
    x, y, z, r, p, a = v
    cr, sr, cp, sp, ca, sa = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(a), math.sin(a)
    t = np.eye(4)
    t[:3, :3] = [[ca*cp, ca*sp*sr-sa*cr, ca*sp*cr+sa*sr],
                   [sa*cp, sa*sp*sr+ca*cr, sa*sp*cr-ca*sr], [-sp, cp*sr, cp*cr]]
    t[:3, 3] = [x, y, z]
    return t


def transform(points, matrix):
    return (matrix @ np.column_stack([points, np.ones(len(points))]).T).T[:, :3]


def hull(points):
    points = sorted(set(map(tuple, np.round(points[:, :2], 6))))
    if len(points) < 3:
        return None
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    halves = []
    for seq in (points, points[::-1]):
        half = []
        for point in seq:
            while len(half) >= 2 and cross(half[-2], half[-1], point) <= 0:
                half.pop()
            half.append(point)
        halves.extend(half[:-1])
    return np.asarray(halves)


def dae_vertices(path):
    """Read position sources with COLLADA scene transforms, units and up axis."""
    root = ET.parse(path).getroot()
    ns = {'c': root.tag.split('}')[0].strip('{')}
    geometries = {}
    for g in root.findall('.//c:library_geometries/c:geometry', ns):
        sources = {s.get('id'): s for s in g.findall('c:mesh/c:source', ns)}
        vertices = []
        for i in g.findall('c:mesh/c:vertices/c:input', ns):
            if i.get('semantic') != 'POSITION':
                continue
            s = sources[i.get('source').lstrip('#')]
            accessor = s.find('c:technique_common/c:accessor', ns)
            stride = int(accessor.get('stride', '3'))
            vals = np.fromstring(s.findtext('c:float_array', namespaces=ns), sep=' ')
            vertices.append(vals.reshape(-1, stride)[:, :3])
        if vertices:
            geometries[g.get('id')] = np.vstack(vertices)
    result = []
    def visit(node, parent):
        local = np.eye(4)
        for child in node:
            tag = child.tag.split('}')[-1]
            vals = np.fromstring(child.text or '', sep=' ')
            t = np.eye(4)
            if tag == 'matrix':
                t = vals.reshape(4, 4)
            elif tag == 'translate':
                t[:3, 3] = vals
            elif tag == 'scale':
                t[:3, :3] = np.diag(vals)
            elif tag == 'rotate':
                axis = vals[:3]/np.linalg.norm(vals[:3])
                a = math.radians(vals[3]); c, s = math.cos(a), math.sin(a)
                x, y, z = axis
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                t[:3, :3] = c*np.eye(3)+(1-c)*np.outer(axis, axis)+s*skew
            else:
                continue
            local = local @ t
        matrix = parent @ local
        for inst in node.findall('c:instance_geometry', ns):
            key = inst.get('url', '').lstrip('#')
            if key in geometries:
                result.append(transform(geometries[key], matrix))
        for child in node.findall('c:node', ns):
            visit(child, matrix)
    ref = root.find('c:scene/c:instance_visual_scene', ns)
    scenes = root.findall('c:library_visual_scenes/c:visual_scene', ns)
    for scene in scenes:
        if ref is not None and scene.get('id') != ref.get('url', '').lstrip('#'):
            continue
        for node in scene.findall('c:node', ns):
            visit(node, np.eye(4))
    if not result:
        result = list(geometries.values())
    unit = root.find('c:asset/c:unit', ns)
    factor = float(unit.get('meter', '1')) if unit is not None else 1.
    up = root.findtext('c:asset/c:up_axis', 'Z_UP', ns)
    conversion = np.eye(4)
    if up == 'Y_UP':
        conversion = pose_matrix('0 0 0 1.5707963267948966 0 0')
    return [transform(v*factor, conversion) for v in result]


def read_scene(world_path, model_roots):
    world = ET.parse(world_path).getroot().find('world')
    state = world.find('state')
    saved = {m.get('name'): m for m in state.findall('model')} if state is not None else {}
    shapes, missing, cache = [], [], {}
    target = None
    for model in world.findall('model'):
        name = model.get('name')
        if name == 'ground_plane':
            continue
        sm = saved.get(name)
        model_pose = pose_matrix(sm.findtext('pose') if sm is not None else model.findtext('pose'))
        if name == 'cup_yellow':
            target = model_pose[:2, 3].tolist()
        saved_links = {l.get('name'): l for l in sm.findall('link')} if sm is not None else {}
        for link in model.findall('link'):
            sl = saved_links.get(link.get('name'))
            lp = pose_matrix(sl.findtext('pose')) if sl is not None else model_pose @ pose_matrix(link.findtext('pose'))
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                matrix = lp @ pose_matrix(collision.findtext('pose'))
                vertices = []
                box = geometry.find('box')
                mesh = geometry.find('mesh')
                if box is not None:
                    size = np.fromstring(box.findtext('size'), sep=' ')/2
                    vertices = [np.array([[x,y,z] for x in [-size[0],size[0]] for y in [-size[1],size[1]] for z in [-size[2],size[2]]])]
                elif mesh is not None:
                    uri = mesh.findtext('uri')
                    relative = uri.removeprefix('model://') if hasattr(uri, 'removeprefix') else uri[len('model://'):]
                    path = next((r/relative for r in model_roots if (r/relative).is_file()), None)
                    if path is None:
                        missing.append({'model': name, 'uri': uri})
                        continue
                    if path not in cache:
                        cache[path] = dae_vertices(path)
                    scale = np.fromstring(mesh.findtext('scale', '1 1 1'), sep=' ')
                    vertices = [v*scale for v in cache[path]]
                for v in vertices:
                    polygon = hull(transform(v, matrix))
                    if polygon is not None:
                        shapes.append({'model': name, 'link': link.get('name'), 'kind': 'wall' if name == 'env04' else 'object', 'xy': polygon.tolist()})
    return shapes, target, missing


def read_records(log):
    records = []
    for line in log.open(encoding='utf-8'):
        if ' event=' not in line or ' data=' not in line:
            continue
        event = line.split(' event=', 1)[1].split(' data=', 1)[0]
        records.append((event, json.loads(line.split(' data=', 1)[1])))
    return records


def draw_scene(ax, shapes):
    for shape in shapes:
        wall = shape['kind'] == 'wall'
        ax.add_patch(Polygon(shape['xy'], closed=True,
                             facecolor='#334155' if wall else '#e5e9ec',
                             edgecolor='#334155' if wall else '#b7c1ca',
                             linewidth=.4, zorder=2 if wall else 1))
    ax.set_aspect('equal')
    ax.set_axisbelow(True)
    ax.grid(color='#e9edf1', linewidth=.45)
    ax.tick_params(length=0, pad=6, colors='#64748b')
    for spine in ax.spines.values():
        spine.set_color('#d5dde5')
    ax.set_xlabel('World x (m)'); ax.set_ylabel('World y (m)')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--model-root', type=Path, action='append')
    args = parser.parse_args()
    roots = args.model_root or [Path.home()/'.gazebo/models']
    shapes, target, missing = read_scene(args.world, roots)
    records = read_records(args.log)
    def event(name):
        return next(d for e,d in records if e == name)
    start = event('run_start'); done = event('task_done'); stop = event('run_stop')
    t0 = start['ros_time']; td = done['ros_time']-t0
    completion = event('task_completed')['completion_snapshot']
    samples = [d for e,d in records if e == 'sample' and d.get('pose') and d['ros_time'] <= done['ros_time']]
    cp = dict(completion['metrics']); cp['ros_time'] = completion['boundary_ros_time']
    samples.append(cp)
    xy = np.array([d['pose'][:2] for d in samples])
    times = np.array([d['ros_time']-t0 for d in samples])
    first = event('target_acquired')
    decisions = [d for e,d in records if e == 'perception_decision' and d.get('metrics_pose')]
    first_decision = min(decisions, key=lambda d: abs(d['ros_time']-first['ros_time']))
    first_pose = first_decision['metrics_pose'][:2]
    final_exposure = [d for e,d in records if e == 'target_exposure_ended'][-1]
    matched = final_exposure['matched_detector_frames']; exposed = final_exposure['exposed_detector_frames']
    run = start['run_timestamp']
    artifact = next(args.log.parent.glob('*_failure_0001.json'), None)
    failure = json.loads(artifact.read_text()) if artifact else None
    stall = failure.get('trigger_sample', {}).get('pose', []) if failure else []
    if not stall and failure:
        stall = stop['summary']['failure_evidence']['last_failure']['trigger_pose']

    plt.rcParams.update({'font.family':'DejaVu Sans', 'font.size':9, 'axes.labelsize':9,
                         'axes.titlesize':11, 'text.color':'#172b40', 'axes.labelcolor':'#475569',
                         'pdf.fonttype':42, 'ps.fonttype':42, 'svg.fonttype':'none'})
    fig = plt.figure(figsize=(15.5, 10.5), facecolor='white')
    gs = fig.add_gridspec(3, 2, width_ratios=[1.65, 1], height_ratios=[1, .65, .38],
                         left=.06, right=.965, top=.865, bottom=.215, wspace=.21, hspace=.55)
    ax = fig.add_subplot(gs[:2,0]); zoom = fig.add_subplot(gs[0,1]); info = fig.add_subplot(gs[1,1]); timeline = fig.add_subplot(gs[2,:])
    fig.text(.06,.952,'ENV04  /  VISUAL OBJECT SEARCH',fontsize=11,weight='bold',color='#147d92')
    fig.text(.06,.913,'Successful yellow-cup search',fontsize=26,weight='bold')
    fig.text(.965,.935,'RUN '+run+'\nTEB navigation · online SLAM',ha='right',va='center',fontsize=10,color='#64748b')
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list('route',['#263e78','#247fa0','#31a997','#c5b948'])
    norm = matplotlib.colors.Normalize(0,td)
    for a in [ax,zoom]:
        draw_scene(a, shapes)
        segments = np.stack([xy[:-1],xy[1:]], axis=1)
        a.add_collection(LineCollection(segments,colors='white',linewidths=4.5,zorder=4))
        lc = LineCollection(segments,cmap=cmap,norm=norm,linewidths=2.4,zorder=5)
        lc.set_array(times[:-1]); a.add_collection(lc)
        a.scatter(*xy[0],s=80,facecolor='#263e78',edgecolor='white',linewidth=1.4,zorder=8)
        a.scatter(*xy[-1],s=90,marker='s',facecolor='#209383',edgecolor='white',linewidth=1.4,zorder=8)
        a.scatter(*first_pose,s=65,marker='D',facecolor='#4d8cbc',edgecolor='white',linewidth=1,zorder=8)
        a.scatter(*target,s=170,marker='*',facecolor='#e2ad25',edgecolor='#8c6517',linewidth=.7,zorder=9)
        if stall:
            a.scatter(*stall[:2],s=65,marker='X',facecolor='#c67450',edgecolor='white',linewidth=1,zorder=8)
    bounds = np.vstack([np.asarray(s['xy']) for s in shapes if s['kind']=='wall'])
    lo,hi = bounds.min(axis=0),bounds.max(axis=0)
    ax.set_xlim(lo[0]-1.1,hi[0]+1.1);ax.set_ylim(lo[1]-1.2,hi[1]+1.2)
    ax.set_title('a   Scene geometry and measured trajectory',loc='left',pad=14,weight='bold')
    zoom.set_xlim(14.5,25.5); zoom.set_ylim(-13.6,-1.8)
    zoom.set_title('b   Target-area detail',loc='left',pad=14,weight='bold')
    def label(a, point, text, offset):
        a.annotate(text,xy=point,xytext=offset,textcoords='offset points',fontsize=8,
                   color='#334155',bbox=dict(boxstyle='round,pad=.3',fc='white',ec='none',alpha=.94),
                   arrowprops=dict(arrowstyle='-',color='#94a3b8',lw=.7),zorder=10)
    label(ax,xy[0],'Start',(10,12))
    label(zoom,target,'Yellow cup\n(saved world state)',(-77,13))
    label(zoom,xy[-1],'Task complete',(-89,12))
    label(zoom,first_pose,'First target detection',(-109,15))
    if stall: label(ax,stall[:2],'Transient planning stall',(15,-20))
    for i in range(8,len(xy)-1,23):
        delta = xy[i+1]-xy[i]
        if np.linalg.norm(delta) > .07:
            ax.annotate('',xy=xy[i+1],xytext=xy[i],arrowprops=dict(arrowstyle='->',color=cmap(norm(times[i])),lw=1.2),zorder=6)
    ax.plot([lo[0]+1,lo[0]+6],[lo[1]+.65]*2,color='#334155',lw=2)
    ax.text(lo[0]+3.5,lo[1]+1.05,'5 m',ha='center',fontsize=8)
    handles = [Line2D([],[],marker=m,color='none',markerfacecolor=c,markeredgecolor='white',markersize=8,label=l)
               for m,c,l in [('o','#263e78','Start'),('D','#4d8cbc','First detection'),('s','#209383','Task complete'),('*','#e2ad25','Target truth'),('X','#c67450','Transient stall')]]
    fig.legend(handles=handles,loc='lower left',bbox_to_anchor=(.055,.072),ncol=5,frameon=False,columnspacing=2)
    cax = fig.add_axes([.06,.133,.50,.011])
    cb = fig.colorbar(lc,cax=cax,orientation='horizontal');cb.outline.set_visible(False)
    cb.ax.tick_params(length=0,labelsize=8);cb.set_label('Elapsed simulation time (s)',fontsize=8,labelpad=2)

    info.axis('off');info.set_title('c   Completion evidence',loc='left',weight='bold',pad=10)
    wall = completion['boundary_wall_elapsed_seconds']
    total_path = completion['metrics']['path_length']
    info.text(0,.84,'SUCCESS',fontsize=21,weight='bold',color='#209383')
    info.text(0,.67,'/lste/task_done = true',fontsize=11,family='DejaVu Sans Mono')
    rows = [('Path length',f'{total_path:.2f} m'),('Time to completion',f'{td:.2f} s simulation / {wall:.1f} s wall'),
            ('First target detection',f'{first["ros_time"]-t0:.2f} s simulation'),
            ('Target approach terminals',str(completion['metrics']['target_approach_terminals'])),
            ('Final exposure matched',f'{matched} / {exposed} frames ({100*matched/exposed:.1f}%)'),
            ('Completion evidence','terminal_track_continuity')]
    for i,(key,value) in enumerate(rows):
        y=.51-i*.105
        info.text(0,y,key,color='#64748b',fontsize=8.5)
        info.text(1,y,value,ha='right',fontsize=8.5)
    timeline.set_title('d   Search progression',loc='left',pad=12,weight='bold')
    tf = first['ros_time']-t0;tc = event('target_follow_confirmed')['ros_time']-t0
    timeline.axvspan(0,tf,color='#edf2f8');timeline.axvspan(tf,td,color='#e5f3ef')
    timeline.plot(times,[d['path_length'] for d in samples],color='#247fa0',lw=1.8)
    for t,c,text,y in [(tf,'#4d8cbc','First detection',.93),(tc,'#147d92','Track confirmed',.56),(td,'#209383','Complete',.93)]:
        timeline.axvline(t,color=c,lw=.9,ls='--')
        timeline.text(t-1,y,text,ha='right',va='top',transform=timeline.get_xaxis_transform(),fontsize=8,color=c,
                      bbox=dict(facecolor='white',edgecolor='none',alpha=.85,pad=2))
    if failure:
        trigger_time = failure.get('trigger_sample',{}).get('ros_time')
        end_time = failure.get('end_sample',{}).get('ros_time')
        if trigger_time is not None and end_time is not None:
            timeline.axvspan(trigger_time-t0,end_time-t0,color='#c67450',alpha=.13)
    timeline.set_xlim(0,td+3);timeline.set_ylim(0,total_path*1.15)
    timeline.set_xlabel('Elapsed simulation time (s)');timeline.set_ylabel('Distance (m)')
    timeline.spines['top'].set_visible(False);timeline.spines['right'].set_visible(False)
    timeline.grid(axis='y',color='#e9edf1',lw=.5)
    fig.text(.06,.031,'Scene: SDF collision footprints + saved model/link poses. Trajectory: logged odometry; world overlay assumes aligned odom/world axes.\n'
             'Straight segments connect measured samples; no trajectory smoothing. No saved camera image or depth evidence is synthesized.',fontsize=7.5,color='#64748b')
    output = args.output_dir;output.mkdir(parents=True,exist_ok=True)
    stem = output/(run+'_env04_success_trajectory')
    for suffix in ['png','pdf','svg']:
        fig.savefig(str(stem)+'.'+suffix,dpi=350,facecolor='white')
    plt.close(fig)
    with (output/'trajectory_samples.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['elapsed_sim_s','ros_time_s','odom_x_m','odom_y_m','yaw_rad','cumulative_path_m'])
        for d in samples:writer.writerow([d['ros_time']-t0,d['ros_time'],*d['pose'],d['path_length']])
    manifest={'world':str(args.world.resolve()),'source_log':str(args.log.resolve()),'target_xy':target,
              'geometry_source':'SDF collision footprints; saved state poses override definition poses',
              'frame_assumption':'logged odom and world nominally aligned; no recorded full TF available',
              'footprint_method':'per-mesh projected convex hull; furniture footprint is not a SLAM occupancy map',
              'missing_meshes':missing,'shapes':shapes}
    (output/'env04_scene_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    report={'run':run,'task_done':done['value'],'completion_evidence':done['completion_evidence'],
            'simulation_seconds_to_done':td,'wall_seconds_to_done':wall,'path_length_m':total_path,
            'trajectory_rows':len(samples),'target_truth_xy':target,'first_detection_xy':first_pose,
            'completion_pose':cp['pose'],'run_stop_pose':stop['summary']['pose'],
            'final_exposure_frames':{'matched':matched,'exposed':exposed},
            'missing_meshes':missing,'figures':[str(stem)+'.'+s for s in ['png','pdf','svg']]}
    (output/'visualization_summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
