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
                        shapes.append({'model': name, 'link': link.get('name'), 'kind': 'wall' if name in ('env04', 'semantic_room_structure') else 'object', 'xy': polygon.tolist()})
    return shapes, target, missing


def read_records(log):
    records = []
    if not log.exists():
        return records
    for line in log.open(encoding='utf-8'):
        if ' event=' not in line or ' data=' not in line:
            continue
        event = line.split(' event=', 1)[1].split(' data=', 1)[0]
        records.append((event, json.loads(line.split(' data=', 1)[1])))
    return records


def extract_run(records, attempt=None):
    """Keep absent success/detection evidence absent when plotting a failure."""
    attempt = attempt or {}
    def event(name, default=None):
        return next((d for e, d in records if e == name), default)
    start = event('run_start', {})
    observed_done = next((d for e, d in records if e == 'task_done' and d.get('value') is True), None)
    done = None if attempt.get('status') in {'failed','infrastructure_failure'} else observed_done
    stop = event('run_stop', {})
    completion_event = event('task_completed', {})
    completion = completion_event.get('completion_snapshot', {})
    periodic = [d for e, d in records if e == 'sample' and d.get('pose')]
    final = completion.get('metrics', {}) if done else stop.get('summary', {})
    if not final:
        final = periodic[-1] if periodic else {}
    t0 = start.get('ros_time', periodic[0]['ros_time'] if periodic else 0.0)
    time_origin = 'run_start'
    if t0 == 0 and periodic and periodic[0]['ros_time'] > 5:
        # Gazebo loads a saved simulation clock after metrics has registered.
        # The unready ROS clock at run_start is not elapsed exploration time.
        t0 = periodic[0]['ros_time']
        time_origin = 'first_pose_sample_run_start_clock_unready'
    end = done['ros_time'] if done else final.get('ros_time', t0)
    samples = [d for d in periodic if d['ros_time'] <= end]
    if final.get('pose'):
        final = dict(final)
        final['ros_time'] = completion.get('boundary_ros_time', end) if done else end
        samples.append(final)
    first = event('target_acquired')
    decisions = [d for e, d in records if e == 'perception_decision' and d.get('metrics_pose')]
    first_pose = None
    if first and decisions:
        first_pose = min(decisions, key=lambda d: abs(d['ros_time']-first['ros_time']))['metrics_pose'][:2]
    exposures = [d for e, d in records if e == 'target_exposure_ended']
    exposure = exposures[-1] if exposures else {}
    return dict(start=start, done=done, stop=stop, completion=completion,
                samples=samples, t0=t0, duration=max(0.0, end-t0), first=first,
                time_origin=time_origin,
                first_pose=first_pose, confirmed=event('target_follow_confirmed'),
                matched=exposure.get('matched_detector_frames'),
                exposed=exposure.get('exposed_detector_frames'),
                final=final, wall=completion.get('boundary_wall_elapsed_seconds') if done
                else stop.get('wall_duration_seconds'),
                evidence=done.get('completion_evidence') if done else None,
                recorded_task_done_true=observed_done is not None,
                outcome='success' if done else attempt.get('status', 'not_completed'))


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


def failure_caption(attempt, final, stop, failure, simulation_seconds, wall_seconds):
    """Summarize only recorded failure cause and terminal-state evidence."""
    reason = attempt.get('reason') or 'task_done was not observed'
    if reason.startswith('stationary_over_'):
        limit = reason[len('stationary_over_'):].replace('_wall_seconds', '')
        cause = f'No translation >{limit} s (radius {attempt.get("stationary_radius_m", "?")} m)'
    elif reason.startswith('exploration_over_'):
        limit = reason[len('exploration_over_'):].replace('_wall_seconds', '')
        cause = f'Exploration exceeded {limit} s wall-time limit'
    elif attempt.get('status') == 'infrastructure_failure':
        cause = 'Experiment infrastructure failed'
    else:
        cause = reason.replace('_', ' ')

    position = attempt.get('stall_pose') or stop.get('summary', {}).get('pose') or final.get('pose')
    details = []
    if position and len(position) >= 2:
        details.append(f'stopped near ({position[0]:.2f}, {position[1]:.2f}) m')
    if wall_seconds is not None:
        details.append(f'{wall_seconds:.1f} s wall')
    if simulation_seconds is not None:
        if simulation_seconds > 0 or final:
            details.append(f'{simulation_seconds:.1f} s simulation')
    path = final.get('path_length')
    if path is not None:
        details.append(f'{path:.1f} m traveled')
    elif attempt.get('status') == 'infrastructure_failure':
        details.append('no odometry recorded')

    diagnosis = failure.get('diagnosis', {}) if failure else {}
    primary = diagnosis.get('primary_cause')
    if primary:
        details.append('log diagnosis: ' + primary.replace('_', ' '))
    if attempt.get('startup_diagnostic'):
        details.append(attempt['startup_diagnostic'])
    return 'FAILURE  |  ' + cause + ('  ·  ' + '  ·  '.join(details) if details else '')


def startup_failure_diagnostic(run_directory):
    """Translate explicit startup log errors into a short, evidence-based label."""
    logs = list(run_directory.glob('*_world.log')) + list(run_directory.glob('*_pro3.log'))
    contents = '\n'.join(path.read_text(errors='replace') for path in logs)
    details = []
    if 'malloc(): smallbin double linked list corrupted' in contents:
        details.append('Gazebo aborted after heap-corruption error')
    elif 'process has died' in contents and 'gzserver' in contents:
        details.append('Gazebo server exited unexpectedly')
    if 'Spawn service failed' in contents:
        details.append('Pro3 spawn service unavailable')
    elif 'did not publish /pro3/wheel_odom' in contents:
        details.append('Pro3 odometry did not start')
    return '; '.join(details) or None


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
    result_path = args.log.parent/'attempt_result.json'
    attempt = json.loads(result_path.read_text()) if result_path.exists() else {}
    data = extract_run(records, attempt)
    start, done, stop = data['start'], data['done'], data['stop']
    t0, td, samples = data['t0'], data['duration'], data['samples']
    xy = np.asarray([d['pose'][:2] for d in samples]).reshape(-1, 2)
    times = np.array([d['ros_time']-t0 for d in samples])
    first, first_pose = data['first'], data['first_pose']
    matched, exposed = data['matched'], data['exposed']
    run = start.get('run_timestamp', args.log.parent.name)
    artifact = next(args.log.parent.glob('*_failure_0001.json'), None)
    failure = json.loads(artifact.read_text()) if artifact else None
    stall = attempt.get('stall_pose') or (failure.get('trigger_sample', {}).get('pose', []) if failure else [])
    if not stall and failure:
        stall = stop.get('summary', {}).get('failure_evidence', {}).get('last_failure', {}).get('trigger_pose', [])
    successful = done is not None
    end_color = '#209383' if successful else '#aa4339'

    plt.rcParams.update({'font.family':'DejaVu Sans', 'font.size':9, 'axes.labelsize':9,
                         'axes.titlesize':11, 'text.color':'#172b40', 'axes.labelcolor':'#475569',
                         'pdf.fonttype':42, 'ps.fonttype':42, 'svg.fonttype':'none'})
    fig = plt.figure(figsize=(15.5, 10.5), facecolor='white')
    gs = fig.add_gridspec(3, 2, width_ratios=[1.65, 1], height_ratios=[1, .65, .38],
                         left=.06, right=.965, top=.865, bottom=.28, wspace=.21, hspace=.55)
    ax = fig.add_subplot(gs[:2,0]); zoom = fig.add_subplot(gs[0,1]); info = fig.add_subplot(gs[1,1]); timeline = fig.add_subplot(gs[2,:])
    fig.text(.06,.952,'ENV04  /  VISUAL OBJECT SEARCH',fontsize=11,weight='bold',color='#147d92')
    fig.text(.06,.913,('Successful' if successful else 'Unsuccessful')+' yellow-cup search',fontsize=26,weight='bold')
    fig.text(.965,.935,'RUN '+run+'\nTEB navigation · online SLAM',ha='right',va='center',fontsize=10,color='#64748b')
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list('route',['#263e78','#247fa0','#31a997','#c5b948'])
    norm = matplotlib.colors.Normalize(0,max(td, 1))
    gaps = np.diff(times)
    gap_limit = max(2.0, 3*float(np.median(gaps[gaps > 0]))) if np.any(gaps > 0) else 2.0
    valid = (gaps >= 0) & (gaps <= gap_limit)
    for a in [ax,zoom]:
        draw_scene(a, shapes)
        segments = np.stack([xy[:-1],xy[1:]], axis=1)[valid]
        a.add_collection(LineCollection(segments,colors='white',linewidths=4.5,zorder=4))
        lc = LineCollection(segments,cmap=cmap,norm=norm,linewidths=2.4,zorder=5)
        lc.set_array(times[:-1][valid]); a.add_collection(lc)
        if len(xy):
            a.scatter(*xy[0],s=80,facecolor='#263e78',edgecolor='white',linewidth=1.4,zorder=8)
            a.scatter(*xy[-1],s=90,marker='s',facecolor=end_color,edgecolor='white',linewidth=1.4,zorder=8)
        if first_pose:
            a.scatter(*first_pose,s=65,marker='D',facecolor='#4d8cbc',edgecolor='white',linewidth=1,zorder=8)
        if target:
            a.scatter(*target,s=170,marker='*',facecolor='#e2ad25',edgecolor='#8c6517',linewidth=.7,zorder=9)
        if stall:
            a.scatter(*stall[:2],s=65,marker='X',facecolor='#c67450',edgecolor='white',linewidth=1,zorder=8)
    bounds = np.vstack([np.asarray(s['xy']) for s in shapes if s['kind']=='wall'])
    lo,hi = bounds.min(axis=0),bounds.max(axis=0)
    ax.set_xlim(lo[0]-1.1,hi[0]+1.1);ax.set_ylim(lo[1]-1.2,hi[1]+1.2)
    ax.set_title('a   Scene geometry and measured trajectory',loc='left',pad=14,weight='bold')
    if not successful and len(xy):
        center = attempt.get('stall_pose') or xy[-1]
        zoom.set_xlim(center[0]-4,center[0]+4); zoom.set_ylim(center[1]-4,center[1]+4)
        detail_title = 'Failure-area detail'
    elif not successful:
        zoom.set_xlim(14.5,25.5); zoom.set_ylim(-13.6,-1.8)
        detail_title = 'Target-area context · no pose recorded'
    else:
        zoom.set_xlim(14.5,25.5); zoom.set_ylim(-13.6,-1.8)
        detail_title = 'Target-area detail'
    zoom.set_title('b   '+detail_title,loc='left',pad=14,weight='bold')
    def label(a, point, text, offset):
        a.annotate(text,xy=point,xytext=offset,textcoords='offset points',fontsize=8,
                   color='#334155',bbox=dict(boxstyle='round,pad=.3',fc='white',ec='none',alpha=.94),
                   arrowprops=dict(arrowstyle='-',color='#94a3b8',lw=.7),zorder=10)
    if len(xy):
        label(ax,xy[0],'First sample' if data['time_origin'] != 'run_start' else 'Start',(10,12))
        label(zoom,xy[-1],'Task complete' if successful else 'Run stopped',(-89,12))
    else:
        ax.text(.5,.5,'No recorded odometry',transform=ax.transAxes,ha='center')
    if target: label(zoom,target,'Yellow cup\n(saved world state)',(-65,-38))
    if first_pose: label(zoom,first_pose,'First target detection',(-109,15))
    if stall: label(ax,stall[:2],'Stationary failure' if attempt.get('stall_pose') else 'Recorded planning episode',(15,-20))
    for i in range(8,len(xy)-1,23):
        delta = xy[i+1]-xy[i]
        if np.linalg.norm(delta) > .07:
            ax.annotate('',xy=xy[i+1],xytext=xy[i],arrowprops=dict(arrowstyle='->',color=cmap(norm(times[i])),lw=1.2),zorder=6)
    ax.plot([lo[0]+1,lo[0]+6],[lo[1]+.65]*2,color='#334155',lw=2)
    ax.text(lo[0]+3.5,lo[1]+1.05,'5 m',ha='center',fontsize=8)
    handles = [Line2D([],[],marker=m,color='none',markerfacecolor=c,markeredgecolor='white',markersize=8,label=l)
               for m,c,l in [('o','#263e78','Start'),('D','#4d8cbc','First detection'),('s',end_color,'Task complete' if successful else 'Run stop'),('*','#e2ad25','Saved target'),('X','#c67450','Recorded stall')]]
    fig.legend(handles=handles,loc='lower left',bbox_to_anchor=(.055,.072),ncol=5,frameon=False,columnspacing=2)
    cax = fig.add_axes([.06,.133,.50,.011])
    cb = fig.colorbar(lc,cax=cax,orientation='horizontal');cb.outline.set_visible(False)
    cb.ax.tick_params(length=0,labelsize=8);cb.set_label('Elapsed simulation time (s)',fontsize=8,labelpad=2)

    info.axis('off');info.set_title('c   Outcome evidence',loc='left',weight='bold',pad=10)
    wall = data['wall']
    total_path = data['final'].get('path_length')
    info.text(0,.84,'SUCCESS' if successful else 'NOT COMPLETED',fontsize=21,weight='bold',color=end_color)
    signal = 'true' if successful else ('true, outside trial limit' if data['recorded_task_done_true'] else 'not observed true')
    info.text(0,.67,'/lste/task_done = '+signal,fontsize=10,family='DejaVu Sans Mono')
    rows = [('Path length',f'{total_path:.2f} m' if total_path is not None else 'Not recorded'),
            ('Observed duration',f'{td:.2f} s simulation / '+(f'{wall:.1f} s wall' if wall is not None else 'wall unavailable')),
            ('First target detection',f'{first["ros_time"]-t0:.2f} s simulation' if first else 'Not recorded'),
            ('Target approach terminals',str(data['final'].get('target_approach_terminals','Not recorded'))),
            ('Final exposure matched',f'{matched} / {exposed} frames ({100*matched/exposed:.1f}%)' if exposed else 'Not recorded'),
            ('Evidence / stop reason',data['evidence'] or attempt.get('reason','No success evidence'))]
    for i,(key,value) in enumerate(rows):
        y=.51-i*.105
        info.text(0,y,key,color='#64748b',fontsize=8.5)
        if value.startswith('stationary_over_'):
            value = 'Stationary >'+value.split('_')[2]+' s wall'
        if value.startswith('exploration_over_'):
            value = 'Exploration >'+value.split('_')[2]+' s wall'
        info.text(1,y,value,ha='right',fontsize=8)
    timeline.set_title('d   Search progression',loc='left',pad=12,weight='bold')
    tf = first['ros_time']-t0 if first else td
    timeline.axvspan(0,tf,color='#edf2f8');timeline.axvspan(tf,td,color='#e5f3ef' if successful else '#faedeb')
    distances = np.array([d.get('path_length',np.nan) for d in samples])
    timeline_x, timeline_y = [], []
    for i in range(len(times)):
        if i and not valid[i-1]:
            timeline_x.append(np.nan); timeline_y.append(np.nan)
        timeline_x.append(times[i]); timeline_y.append(distances[i])
    timeline.plot(timeline_x,timeline_y,color='#247fa0',lw=1.8)
    milestones = [(td,end_color,'Complete' if successful else 'Stopped',.93)]
    if first: milestones.append((tf,'#4d8cbc','First detection',.93))
    if data['confirmed']: milestones.append((data['confirmed']['ros_time']-t0,'#147d92','Track confirmed',.56))
    for t,c,text,y in milestones:
        timeline.axvline(t,color=c,lw=.9,ls='--')
        timeline.text(t-1,y,text,ha='right',va='top',transform=timeline.get_xaxis_transform(),fontsize=8,color=c,
                      bbox=dict(facecolor='white',edgecolor='none',alpha=.85,pad=2))
    if failure:
        trigger_time = failure.get('trigger_sample',{}).get('ros_time')
        end_time = failure.get('end_sample',{}).get('ros_time')
        if trigger_time is not None and end_time is not None:
            timeline.axvspan(trigger_time-t0,end_time-t0,color='#c67450',alpha=.13)
    timeline.set_xlim(0,td+3);timeline.set_ylim(0,max(1,(total_path or 0)*1.15))
    timeline.set_xlabel('Elapsed simulation time (s)');timeline.set_ylabel('Distance (m)')
    timeline.spines['top'].set_visible(False);timeline.spines['right'].set_visible(False)
    timeline.grid(axis='y',color='#e9edf1',lw=.5)
    if not successful:
        attempt['startup_diagnostic'] = startup_failure_diagnostic(args.log.parent)
        caption = failure_caption(attempt, data['final'], stop, failure, td, wall)
        fig.text(.06,.217,caption,fontsize=8.4,color='#8b3029',weight='medium',
                 bbox=dict(boxstyle='round,pad=.55',facecolor='#fbf1ef',edgecolor='#e5c4bd',linewidth=.7))
        if not len(xy):
            timeline.text(.5,.52,'No odometry samples recorded; search did not start',
                          transform=timeline.transAxes,ha='center',va='center',fontsize=9,color='#64748b')
    fig.text(.06,.031,'Scene: SDF collision footprints + saved model/link poses. Trajectory: logged odometry; world overlay assumes aligned odom/world axes.\n'
             'Segments join measured samples; large logging gaps are left open. No trajectory smoothing or synthesized camera/depth evidence.',fontsize=7.5,color='#64748b')
    if data['time_origin'] != 'run_start':
        fig.text(.06,.012,'Time origin: first recorded pose sample; run_start ROS clock was not ready (0 before saved-world clock loaded).',fontsize=7,color='#64748b')
    output = args.output_dir;output.mkdir(parents=True,exist_ok=True)
    stem = output/(run+'_env04_'+('success' if successful else 'failure')+'_trajectory')
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
    report={'run':run,'outcome':data['outcome'],'task_done':data['recorded_task_done_true'],
            'within_trial_success':successful,
            'recorded_task_done_true':data['recorded_task_done_true'],'completion_evidence':data['evidence'],
            'simulation_seconds_to_done':td if successful else None,'wall_seconds_to_done':wall if successful else None,
            'observed_simulation_seconds':td,'observed_wall_seconds':wall,'path_length_m':total_path,
            'simulation_time_origin':data['time_origin'],'time_origin_ros_s':t0,
            'run_start_ros_s':start.get('ros_time'),
            'trajectory_rows':len(samples),'target_truth_xy':target,'first_detection_xy':first_pose,
            'completion_pose':data['final'].get('pose') if successful else None,
            'run_stop_pose':stop.get('summary',{}).get('pose'),
            'stop_reason':attempt.get('reason'),'stall_pose':attempt.get('stall_pose'),
            'omitted_gap_segments':int(np.count_nonzero(~valid)),'gap_limit_sim_s':gap_limit,
            'final_exposure_frames':{'matched':matched,'exposed':exposed},
            'missing_meshes':missing,'figures':[str(stem)+'.'+s for s in ['png','pdf','svg']]}
    (output/'visualization_summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
