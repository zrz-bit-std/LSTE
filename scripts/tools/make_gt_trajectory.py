#!/usr/bin/env python3
"""Derive a reproducible ground-truth reference trajectory for one world.

The reference is the measured odometry of a sealed, audited successful run.
Recorded samples are already expressed in the world frame: the robot spawns at
the run's configured start pose and the recorded odometry carries no SLAM
correction, which the run figure footer states explicitly.  The script keeps
the samples as recorded (no smoothing, no gap closing) and additionally emits a
uniform-time resample, a milestone waypoint skeleton and a metadata block with
provenance hashes so the artefact can be regenerated and audited.
"""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plot_world_run import extract_run, read_records, read_scene


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def build(run_directory, world, dt):
    metrics = next(iter(sorted(run_directory.glob('*_navigation_metrics.log'))), None)
    if metrics is None:
        raise SystemExit('No *_navigation_metrics.log under %s' % run_directory)
    attempt_path = run_directory / 'attempt_result.json'
    attempt = json.loads(attempt_path.read_text()) if attempt_path.exists() else {}
    records = read_records(metrics)
    data = extract_run(records, attempt)
    if data['done'] is None:
        raise SystemExit('Run has no task_done=true; refusing to freeze it as ground truth')
    samples = data['samples']
    if len(samples) < 2:
        raise SystemExit('Fewer than two pose samples recorded')

    t0 = data['t0']
    elapsed = np.array([s['ros_time'] - t0 for s in samples], dtype=float)
    xy = np.array([s['pose'][:2] for s in samples], dtype=float)
    yaw = np.unwrap(np.array([s['pose'][2] if len(s['pose']) > 2 else 0.0 for s in samples], dtype=float))
    length = np.array([s.get('path_length', np.nan) for s in samples], dtype=float)
    if np.all(~np.isfinite(length)):
        length = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])

    gaps = np.diff(elapsed)
    step = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    speed = np.divide(step, gaps, out=np.zeros_like(step), where=gaps > 0)
    speed = np.concatenate([[0.0], speed])
    gap_limit = max(2.0, 3 * float(np.median(gaps[gaps > 0]))) if np.any(gaps > 0) else 2.0
    segment = np.concatenate([[0], np.cumsum(gaps > gap_limit)]).astype(int)

    roots = [Path.home() / '.gazebo/models',
             Path(__file__).resolve().parents[2] / 'worlds/targeted_navigation/models',
             Path(__file__).resolve().parents[2] / 'worlds']
    shapes, target, missing = read_scene(world, roots)
    if target is None:
        target = attempt.get('audit', {}).get('completion', {}).get('target_xy')
    target = np.array(target, dtype=float) if target is not None else None
    distance = (np.linalg.norm(xy - target, axis=1) if target is not None
                else np.full(len(xy), np.nan))

    first = data['first']
    confirmed = data['confirmed']
    tf = first['ros_time'] - t0 if first else elapsed[-1]
    tc = confirmed['ros_time'] - t0 if confirmed else elapsed[-1]
    td = float(elapsed[-1])
    bounds = [(0.0, tf, 'explore'), (tf, tc, 'approach'), (tc, td + 1e-9, 'lock_confirm')]
    phase = np.array([next((name for lo, hi, name in bounds if lo <= t < hi), 'lock_confirm')
                      for t in elapsed])

    # Milestone skeleton: start, every physically entered Place, target events,
    # completion.  Poses are read from the recorded samples, never invented.
    spawn = [3.4, 0.0]
    config = run_directory / 'pipeline_config.yaml'
    if config.exists():
        for line in config.read_text(errors='replace').splitlines():
            key, _, value = line.partition(':')
            if key.strip() == 'PRO3_SPAWN_X':
                spawn[0] = float(value.strip())
            elif key.strip() == 'PRO3_SPAWN_Y':
                spawn[1] = float(value.strip())

    def pose_at(ros_time):
        index = int(np.argmin(np.abs(np.array([s['ros_time'] for s in samples]) - ros_time)))
        return [float(xy[index][0]), float(xy[index][1]), float(wrap(yaw[index])), float(elapsed[index])]

    waypoints = [dict(kind='start', label='start',
                      ros_time=float(samples[0]['ros_time']), pose=pose_at(samples[0]['ros_time'])[:3])]
    for event, d in records:
        # Place entries are carried as frontier_event names inside the
        # global_frontier_event record, not as top-level event names.
        name = d.get('frontier_event') if event == 'global_frontier_event' else event
        if name != 'place_dfs_entered':
            continue
        pose = pose_at(d['ros_time'])
        waypoints.append(dict(kind='place_entered', label='P%s' % d.get('place_id'),
                              place_id=d.get('place_id'), parent_place_id=d.get('parent_place_id'),
                              portal_id=d.get('portal_id'), ros_time=float(d['ros_time']),
                              pose=pose[:3], elapsed_sim_s=pose[3]))
    for kind, event in (('first_detection', 'target_acquired'),
                        ('track_confirmed', 'target_follow_confirmed'),
                        ('task_complete', 'target_close_confirmed')):
        row = next((d for e, d in records if e == event), None)
        if row is None:
            continue
        pose = pose_at(row['ros_time'])
        waypoints.append(dict(kind=kind, label=kind.replace('_', ' '),
                              ros_time=float(row['ros_time']), pose=pose[:3],
                              elapsed_sim_s=pose[3]))
    waypoints.sort(key=lambda w: w['ros_time'])

    # Uniform-time resample.  Yaw is unwrapped before interpolation so a turn
    # through +/-pi does not average to zero.  The grid starts at the first
    # recorded pose: the metrics recorder's first sample lags run_start by a
    # couple of seconds and that interval is not part of the measured path.
    grid = np.arange(float(elapsed[0]), td + 1e-9, dt)
    if grid.size and grid[-1] < td - 1e-9:
        grid = np.append(grid, td)
    resampled = dict(
        elapsed_sim_s=grid.tolist(),
        x_m=np.interp(grid, elapsed, xy[:, 0]).tolist(),
        y_m=np.interp(grid, elapsed, xy[:, 1]).tolist(),
        yaw_rad=np.array([wrap(v) for v in np.interp(grid, elapsed, yaw)]).tolist(),
    )
    resampled['cumulative_path_m'] = np.concatenate(
        [[0.0], np.cumsum(np.linalg.norm(np.diff(np.column_stack(
            [resampled['x_m'], resampled['y_m']]), axis=0), axis=1))]).tolist()
    # Continue the recorded odometry length instead of restarting at zero, so
    # both files report the same distance for the same pose.
    resampled['cumulative_path_m'] = [float(length[0]) + value
                                      for value in resampled['cumulative_path_m']]
    if target is not None:
        resampled['distance_to_target_m'] = np.linalg.norm(
            np.column_stack([resampled['x_m'], resampled['y_m']]) - target, axis=1).tolist()

    # Frame alignment is evidenced by the raw pose recorder, whose very first
    # row is the configured spawn pose; the metrics sample stream starts a
    # couple of seconds later and therefore already includes some motion.
    alignment = None
    for recorder in sorted(run_directory.glob('*_pose.jsonl')):
        with recorder.open(encoding='utf-8', errors='replace') as stream:
            for line in stream:
                try:
                    row = json.loads(line)
                except ValueError:
                    continue
                if row.get('kind') == 'pose':
                    alignment = float(math.hypot(row['x'] - spawn[0], row['y'] - spawn[1]))
                    break
        break

    checks = {
        'time_monotonic': bool(np.all(gaps >= 0)),
        'no_nan_pose': bool(np.all(np.isfinite(xy))),
        'gap_limit_sim_s': gap_limit,
        'gap_segments': int(segment[-1] + 1),
        'max_sample_step_m': float(step.max()) if len(step) else 0.0,
        'chord_sum_vs_recorded_path_m': abs(float(length[-1]) - float(step.sum())) if len(step) else 0.0,
        'odom_world_alignment_m': alignment,
        'first_sample_offset_from_spawn_m': float(np.linalg.norm(xy[0] - np.array(spawn))),
    }
    meta = dict(
        environment=world.stem,
        world=str(world.resolve()),
        source_run=run_directory.name,
        source_log=str(metrics.resolve()),
        source_log_sha256=sha256(metrics),
        source_run_audit={
            'verdict': attempt.get('audit', {}).get('verdict'),
            'task_success': attempt.get('audit', {}).get('task_success'),
            'completion_evidence': data['evidence'],
            'reason': attempt.get('reason'),
        },
        frame='world (Gazebo world axes; recorded odometry is uncorrected and starts at the configured spawn pose)',
        sampling='as recorded by lste_navigation_metrics event=sample (0.5 s nominal)',
        target_truth_xy=target.tolist() if target is not None else None,
        phases=[{'name': name, 'start_s': lo, 'end_s': hi} for lo, hi, name in bounds],
        milestones={
            'first_detection_sim_s': float(tf),
            'track_confirmed_sim_s': float(tc),
            'task_done_sim_s': float(td),
        },
        stats={
            'samples': len(samples),
            'duration_sim_s': float(td),
            'duration_wall_s': data['wall'],
            'first_sample_sim_s': float(elapsed[0]),
            'recorded_span_sim_s': float(elapsed[-1] - elapsed[0]),
            'path_length_m': float(length[-1]),
            'visited_places': len([w for w in waypoints if w['kind'] == 'place_entered']),
            'final_distance_to_target_m': float(distance[-1]) if np.isfinite(distance[-1]) else None,
            'mean_speed_mps': float(step.sum() / td) if td > 0 else None,
            'stationary_fraction': float(np.mean(speed < 0.02)),
        },
        checks=checks,
        missing_meshes=missing,
        columns={
            'elapsed_sim_s': 'seconds since metrics run_start',
            'x_m': 'world x of the recorded base pose',
            'y_m': 'world y of the recorded base pose',
            'yaw_rad': 'recorded heading, wrapped to [-pi, pi]',
            'cumulative_path_m': 'odometry path length at that sample',
            'speed_mps': 'finite difference of cumulative path length',
            'distance_to_target_m': 'euclidean distance to the saved target pose',
            'phase': 'explore | approach | lock_confirm',
            'segment_id': 'increments where the logging gap exceeds gap_limit_sim_s',
        },
        transformations=[
            'No smoothing, no gap closing, no synthetic route',
            'Reference path is a demonstration, not an optimal or shortest path',
            'Uniform-time file interpolates linearly between recorded samples only',
        ],
    )
    return dict(data=data, elapsed=elapsed, xy=xy, yaw=yaw, length=length, speed=speed,
                segment=segment, phase=phase, target=target, distance=distance,
                shapes=shapes, waypoints=waypoints, resampled=resampled, meta=meta,
                gap_limit=gap_limit)


def write_csv(path, rows, columns):
    lines = [','.join(columns)]
    for row in rows:
        lines.append(','.join(str(row[c]) for c in columns))
    path.write_text('\n'.join(lines) + '\n')


def draw(path, result, world_stem):
    xy, elapsed, target = result['xy'], result['elapsed'], result['target']
    fig = plt.figure(figsize=(15, 8.6), facecolor='white')
    grid = fig.add_gridspec(2, 3, width_ratios=[1.9, .55, .55], height_ratios=[1, 1],
                            left=.055, right=.97, top=.88, bottom=.1, wspace=.28, hspace=.3)
    route = fig.add_subplot(grid[:, 0])
    dist = fig.add_subplot(grid[0, 1:])
    speed = fig.add_subplot(grid[1, 1:])

    for shape in result['shapes']:
        if shape.get('kind') == 'wall':
            route.add_patch(Polygon(shape['xy'], closed=True, facecolor='#39434B', lw=.3, zorder=1))
    segments = np.stack([xy[:-1], xy[1:]], axis=1)
    valid = (np.diff(elapsed) <= result['gap_limit']) & (np.diff(elapsed) >= 0)
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list(
        'gt', ['#263e78', '#247fa0', '#31a997', '#c5b948'])
    norm = matplotlib.colors.Normalize(0, max(elapsed[-1], 1))
    route.add_collection(LineCollection(segments[valid], colors='white', linewidths=4.2, zorder=3))
    collection = LineCollection(segments[valid], cmap=cmap, norm=norm, linewidths=2.3, zorder=4)
    collection.set_array(elapsed[:-1][valid])
    route.add_collection(collection)
    route.scatter(*xy[0], s=80, facecolor='#263e78', edgecolor='white', linewidth=1.4, zorder=8)
    route.scatter(*xy[-1], s=90, marker='s', facecolor='#209383', edgecolor='white', linewidth=1.4, zorder=8)
    if target is not None:
        route.scatter(*target, s=180, marker='*', facecolor='#e2ad25', edgecolor='#8c6517', zorder=9,
                      label='Target (saved world pose)')
        route.annotate('Target\n(%.2f, %.2f)' % (target[0], target[1]), xy=target,
                       xytext=(-62, -34), textcoords='offset points', fontsize=8.5,
                       bbox=dict(boxstyle='round,pad=.3', fc='white', ec='none', alpha=.9),
                       arrowprops=dict(arrowstyle='-', color='#94a3b8', lw=.7))
    for waypoint in result['waypoints']:
        if waypoint['kind'] not in ('place_entered', 'first_detection', 'track_confirmed'):
            continue
        marker, color = (('D', '#4d8cbc') if waypoint['kind'] != 'place_entered'
                         else ('o', '#7b8794'))
        p = waypoint['pose']
        route.scatter(p[0], p[1], s=46, marker=marker, facecolor='none', edgecolor=color, zorder=7)
        offset = (6, -12) if waypoint['kind'] == 'first_detection' else (6, 6)
        route.annotate(waypoint['label'], (p[0], p[1]), xytext=offset, textcoords='offset points',
                       fontsize=7.5, color=color)
    walls = np.vstack([np.asarray(s['xy']) for s in result['shapes'] if s.get('kind') == 'wall'])
    if len(walls):
        lo, hi = walls.min(axis=0), walls.max(axis=0)
        route.set_xlim(lo[0] - 1, hi[0] + 1)
        route.set_ylim(lo[1] - 1.5, hi[1] + 1.5)
        route.plot([lo[0] + 1, lo[0] + 6], [lo[1] + .65] * 2, color='#334155', lw=2)
        route.text(lo[0] + 3.5, lo[1] + 1.05, '5 m', ha='center', fontsize=8)
    route.set_aspect('equal')
    route.set_xlabel('World x (m)')
    route.set_ylabel('World y (m)')
    route.grid(color='#e9edf1', lw=.45)
    route.set_title('Ground-truth reference trajectory · %s' % world_stem.replace('_', ' '),
                    loc='left', weight='bold', pad=12)
    handles = [plt.Line2D([], [], marker=m, color='none', markerfacecolor=c, markeredgecolor='white',
                          markersize=8, label=l)
               for m, c, l in (('o', '#263e78', 'Start (first sample)'),
                               ('s', '#209383', 'Task complete'),
                               ('D', '#4d8cbc', 'Target events'),
                               ('o', '#7b8794', 'Place entered'))]
    if target is not None:
        handles.append(plt.Line2D([], [], marker='*', color='none', markerfacecolor='#e2ad25',
                                  markeredgecolor='#8c6517', markersize=11,
                                  label='Target (saved world pose)'))
    route.legend(handles=handles, loc='upper right', fontsize=7.5, frameon=False)

    if np.isfinite(result['distance']).any():
        dist.plot(elapsed, result['distance'], color='#247fa0', lw=1.8)
        dist.axhline(1.0, color='#94a3b8', lw=.8, ls='--')
        dist.text(elapsed[-1], 1.0, '1 m', ha='right', va='bottom', fontsize=7, color='#94a3b8')
        dist.set_ylabel('Distance to target (m)')
    for lo, hi, name in [(p['start_s'], p['end_s'], p['name']) for p in result['meta']['phases']]:
        dist.axvspan(lo, hi, color='#eef3f8' if name == 'explore' else '#f3f8f5', zorder=0)
        speed.axvspan(lo, hi, color='#eef3f8' if name == 'explore' else '#f3f8f5', zorder=0)
    milestone_handles = []
    for label, value, color in ((('First detection'), result['meta']['milestones']['first_detection_sim_s'], '#4d8cbc'),
                                (('Track confirmed'), result['meta']['milestones']['track_confirmed_sim_s'], '#147d92'),
                                (('Task done'), result['meta']['milestones']['task_done_sim_s'], '#209383')):
        for ax in (dist, speed):
            ax.axvline(value, color=color, lw=.9, ls='--')
        milestone_handles.append(plt.Line2D([], [], color=color, lw=.9, ls='--', label=label))
    dist.legend(handles=milestone_handles, loc='upper right', fontsize=7, frameon=False)

    window = max(3, int(round(5.0 / max(np.median(np.diff(elapsed)), 1e-3))))
    speed.fill_between(elapsed, 0, result['speed'], color='#c5b948', alpha=.3, linewidth=0)
    speed.plot(elapsed, result['speed'], color='#c5b948', lw=.7, alpha=.9)
    if window < len(elapsed):
        trend = np.convolve(result['speed'], np.ones(window) / window, mode='valid')
        speed.plot(elapsed[window - 1:], trend, color='#6b5c14', lw=1.7,
                   label='%d-sample mean (%.2f m/s)' % (window, float(trend.mean())))
    speed.set_xlabel('Elapsed simulation time (s)')
    speed.set_ylabel('Speed (m/s)')
    speed.set_ylim(bottom=0)
    speed.legend(loc='upper right', fontsize=7, frameon=False)
    for ax in (dist, speed):
        ax.grid(color='#e9edf1', lw=.5)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
    fig.colorbar(collection, ax=[dist, speed], location='right', shrink=.55, pad=.02,
                 label='Elapsed simulation time (s)')

    stats = result['meta']['stats']
    fig.suptitle('env04 ground truth · %d samples · %.1f m · %.1f s simulation · source run %s'
                 % (stats['samples'], stats['path_length_m'], stats['duration_sim_s'],
                    result['meta']['source_run']),
                 x=.055, ha='left', fontsize=13, weight='bold', color='#147d92')
    fig.text(.055, .018, 'Recorded odometry of one audited successful run, kept unsmoothed. '
             'World frame assumed aligned with odometry at the spawn pose; no camera/depth evidence.',
             fontsize=7.5, color='#64748b')
    fig.savefig(path, dpi=200, facecolor='white')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path, help='Run directory under runtime/navigation/logs')
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--out', type=Path,
                        default=Path(__file__).resolve().parents[2] / 'runtime/navigation/gt')
    parser.add_argument('--dt', type=float, default=1.0, help='Uniform resample step in simulation seconds')
    args = parser.parse_args()

    result = build(args.run, args.world, args.dt)
    directory = args.out / args.world.stem.replace('_no_pro3', '')
    directory.mkdir(parents=True, exist_ok=True)
    stem = directory / (directory.name + '_gt_trajectory')

    elapsed = result['elapsed']
    columns = ['elapsed_sim_s', 'ros_time_s', 'x_m', 'y_m', 'yaw_rad', 'heading_deg',
               'cumulative_path_m', 'speed_mps', 'distance_to_target_m', 'phase', 'segment_id']
    rows = []
    for i, sample in enumerate(result['data']['samples']):
        rows.append({
            'elapsed_sim_s': round(float(elapsed[i]), 4),
            'ros_time_s': round(float(sample['ros_time']), 4),
            'x_m': round(float(result['xy'][i][0]), 4),
            'y_m': round(float(result['xy'][i][1]), 4),
            'yaw_rad': round(float(wrap(result['yaw'][i])), 4),
            'heading_deg': round(math.degrees(wrap(result['yaw'][i])), 2),
            'cumulative_path_m': round(float(result['length'][i]), 4),
            'speed_mps': round(float(result['speed'][i]), 4),
            'distance_to_target_m': (round(float(result['distance'][i]), 4)
                                     if np.isfinite(result['distance'][i]) else ''),
            'phase': result['phase'][i],
            'segment_id': int(result['segment'][i]),
        })
    write_csv(Path(str(stem) + '.csv'), rows, columns)

    resampled = result['resampled']
    uniform = []
    for i, t in enumerate(resampled['elapsed_sim_s']):
        uniform.append({
            'elapsed_sim_s': round(float(t), 3),
            'x_m': round(float(resampled['x_m'][i]), 4),
            'y_m': round(float(resampled['y_m'][i]), 4),
            'yaw_rad': round(float(resampled['yaw_rad'][i]), 4),
            'cumulative_path_m': round(float(resampled['cumulative_path_m'][i]), 4),
            'distance_to_target_m': (round(float(resampled['distance_to_target_m'][i]), 4)
                                     if 'distance_to_target_m' in resampled else ''),
        })
    write_csv(Path(str(stem) + '_%gs.csv' % args.dt), uniform,
              ['elapsed_sim_s', 'x_m', 'y_m', 'yaw_rad', 'cumulative_path_m', 'distance_to_target_m'])

    (Path(str(stem) + '_waypoints.json')).write_text(
        json.dumps({'environment': result['meta']['environment'], 'frame': result['meta']['frame'],
                    'source_run': result['meta']['source_run'], 'waypoints': result['waypoints']},
                   indent=2) + '\n')
    (Path(str(stem) + '_meta.json')).write_text(json.dumps(result['meta'], indent=2) + '\n')
    draw(Path(str(stem) + '.png'), result, directory.name)
    print(json.dumps({'files': [str(p) for p in sorted(directory.glob('*'))],
                      'stats': result['meta']['stats'], 'checks': result['meta']['checks']},
                     indent=2))


if __name__ == '__main__':
    main()
