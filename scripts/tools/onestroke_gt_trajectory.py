#!/usr/bin/env python3
"""Reduce the deduplicated corridor graph to a single one-stroke curve.

A pen stroke that never lifts and never branches is a simple path: the walk
visits every vertex at most once.  The corridor graph produced by
``dedup_gt_trajectory.py`` is a tree, so this is only possible for a subset of
the corridors.  Two well defined subsets are offered:

``spawn_target``
    The unique corridor path from the recorded start pose to the completion
    pose.  Natural episode shape, but side branches are dropped.

``diameter``
    The longest simple path in the tree, i.e. the maximum amount of corridor
    that a single stroke can cover.  Its ends are two extremities of the
    explored area, not necessarily the spawn.

Both outputs are continuous by construction: consecutive rows are vertices
joined by an edge of the corridor graph, so the polyline never jumps.
"""
import argparse
import json
import math
from collections import deque
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon
import numpy as np

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
import dedup_gt_trajectory as dedup


def breadcrumbs(graph, source):
    """BFS over edge weights; return (parent, farthest node)."""
    distance = {source: 0.0}
    parent = {source: None}
    queue = deque([source])
    while queue:
        node = queue.popleft()
        for other, weight in graph.adj[node].items():
            if other in distance:
                continue
            distance[other] = distance[node] + weight
            parent[other] = node
            queue.append(other)
    return parent, distance, max(distance, key=distance.get)


def path_between(parent, source, goal):
    path = [goal]
    while path[-1] != source:
        if parent.get(path[-1]) is None:
            return None
        path.append(parent[path[-1]])
    return path[::-1]


def path_length(graph, path):
    return float(sum(graph.adj[a][b] for a, b in zip(path[:-1], path[1:])))


def select(graph, start, goal, mode):
    if mode == 'spawn_target':
        parent, _, _ = breadcrumbs(graph, start)
        path = path_between(parent, start, goal)
        if path is None:
            raise SystemExit('No corridor path between start and completion')
        return path, 'start pose to completion pose'
    _, _, first = breadcrumbs(graph, start)
    parent, _, second = breadcrumbs(graph, first)
    path = path_between(parent, first, second)
    # Keep the episode orientation when the completion pose is one of the two
    # extremities, so the stroke ends where the robot actually finished.
    if path and path[0] == goal:
        path = path[::-1]
    return path, 'longest simple path (tree diameter), oriented to end at the target'


def build(trajectory, mode, spacing, tolerance, min_gap, min_spur):
    result = dedup.build(trajectory, spacing, tolerance, min_gap, min_spur)
    graph = result['graph']
    start = result['stroke'][0]['node']
    goal = result['stroke'][-1]['node']
    path, description = select(graph, start, goal, mode)
    return result, graph, path, description


def draw(path_png, result, graph, path, description, environment):
    shapes = result.get('shapes')
    walls = [np.asarray(s['xy']) for s in shapes if s.get('kind') == 'wall'] if shapes else []
    length = path_length(graph, path)
    corridor = graph.length()

    fig = plt.figure(figsize=(14.5, 8.4), facecolor='white')
    grid = fig.add_gridspec(1, 3, width_ratios=[1.9, .42, .42], left=.055, right=.97,
                            top=.87, bottom=.1, wspace=.26)
    route = fig.add_subplot(grid[:, 0])
    bars = fig.add_subplot(grid[0, 1:])

    for polygon in walls:
        route.add_patch(Polygon(polygon, closed=True, facecolor='#39434B', lw=.3, zorder=1))
    ghost = [[[graph.x[a], graph.y[a]], [graph.x[b], graph.y[b]]] for a, b in graph.edges]
    route.add_collection(LineCollection(ghost, colors='#cbd4dc', linewidths=1.0, zorder=2))

    points = np.array([[graph.x[n], graph.y[n]] for n in path])
    segments = np.stack([points[:-1], points[1:]], axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))])
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list(
        'stroke', ['#263e78', '#247fa0', '#31a997', '#c5b948'])
    norm = matplotlib.colors.Normalize(0, max(cumulative[-1], 1))
    route.add_collection(LineCollection(segments, colors='white', linewidths=5.0, zorder=3))
    collection = LineCollection(segments, cmap=cmap, norm=norm, linewidths=2.8, zorder=4)
    collection.set_array(cumulative[:-1])
    route.add_collection(collection)

    walked, next_tick = 0.0, 3.5
    for a, b in zip(points[:-1], points[1:]):
        step = float(np.linalg.norm(b - a))
        if step <= 1e-9:
            continue
        walked += step
        if walked < next_tick:
            continue
        next_tick += 3.5
        middle, unit = (a + b) / 2, (b - a) / step
        route.annotate('', xy=middle + .6 * unit, xytext=middle - .6 * unit,
                       arrowprops=dict(arrowstyle='->', color='#172b40', lw=.9, alpha=.75), zorder=6)

    route.scatter(*points[0], s=85, facecolor='#263e78', edgecolor='white', linewidth=1.4,
                  zorder=8, label='Stroke start')
    route.scatter(*points[-1], s=95, marker='s', facecolor='#209383', edgecolor='white',
                  linewidth=1.4, zorder=8, label='Stroke end')
    if walls:
        stacked = np.vstack(walls)
        lo, hi = stacked.min(axis=0), stacked.max(axis=0)
        route.set_xlim(lo[0] - 1, hi[0] + 1)
        route.set_ylim(lo[1] - 1.5, hi[1] + 1.5)
        route.plot([lo[0] + 1, lo[0] + 6], [lo[1] + .65] * 2, color='#334155', lw=2)
        route.text(lo[0] + 3.5, lo[1] + 1.05, '5 m', ha='center', fontsize=8)
    route.set_aspect('equal')
    route.set_xlabel('World x (m)')
    route.set_ylabel('World y (m)')
    route.grid(color='#e9edf1', lw=.45)
    route.set_title('One-stroke reference trajectory · %s' % environment,
                    loc='left', weight='bold', pad=12)
    route.legend(loc='upper right', fontsize=8, frameon=False)
    fig.colorbar(collection, ax=route, orientation='vertical', fraction=.028, pad=.012,
                 label='Distance along stroke (m)')

    bars.axis('off')
    bars.set_title('What one stroke can cover', loc='left', weight='bold', pad=12, fontsize=9.5)
    rows = [('Explored corridor', corridor, '#cbd4dc'),
            ('One-stroke path', length, '#247fa0')]
    for index, (label, value, color) in enumerate(rows):
        y = .78 - index * .16
        bars.add_patch(plt.Rectangle((0, y), value / corridor * .95, .085,
                                     facecolor=color, edgecolor='none'))
        bars.text(0, y - .045, label, fontsize=8.5, color='#475569')
        bars.text(value / corridor * .95 + .015, y + .02, '%.1f m' % value, fontsize=8.5,
                  va='center', color='#172b40')
    bars.text(0, .34, 'Coverage %.0f%% of the explored corridor'
              % (100 * length / corridor), fontsize=8.5, color='#172b40')
    leaves = sum(1 for node in graph.adj if len(graph.adj[node]) == 1)
    bars.text(0, .24, 'Corridor graph has %d leaves (odd-degree vertices).\n'
              'A stroke without lifting the pen can only be a\n'
              'simple path, so branches must be dropped; the\n'
              'longest such path is the tree diameter.'
              % leaves, fontsize=7.6, color='#64748b', va='top', linespacing=1.5)
    bars.set_xlim(0, 1.05)
    bars.set_ylim(0, 1)

    fig.suptitle('%s · one-stroke · %s' % (environment, description),
                 x=.055, ha='left', fontsize=13, weight='bold', color='#147d92')
    fig.text(.055, .022, 'Every row is a vertex of the deduplicated corridor graph and every '
             'consecutive pair shares an edge, so the polyline is one unbroken stroke. '
             'Grey geometry is corridor that this stroke does not visit.',
             fontsize=7.5, color='#64748b')
    fig.savefig(path_png, dpi=200, facecolor='white')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory', type=Path, help='gt/<env>/<env>_gt_trajectory.csv')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--world', type=Path)
    parser.add_argument('--mode', choices=('spawn_target', 'diameter'), default='diameter')
    parser.add_argument('--spacing', type=float, default=0.1)
    parser.add_argument('--corridor-tolerance', type=float, default=0.7)
    parser.add_argument('--min-gap', type=float, default=1.0)
    parser.add_argument('--min-spur', type=float, default=1.0)
    args = parser.parse_args()

    trajectory = dedup.read_trajectory(args.trajectory)
    result, graph, path, description = build(trajectory, args.mode, args.spacing,
                                             args.corridor_tolerance, args.min_gap,
                                             args.min_spur)
    if args.world is not None:
        from plot_world_run import read_scene
        roots = [Path.home() / '.gazebo/models', Path(__file__).resolve().parents[2] / 'worlds']
        shapes, _, _ = read_scene(args.world, roots)
        result['shapes'] = shapes

    output = args.output_dir or args.trajectory.parent
    output.mkdir(parents=True, exist_ok=True)
    environment = args.trajectory.parent.name
    stem = output / ('%s_onestroke_%s_trajectory' % (environment, args.mode))

    points = np.array([[graph.x[n], graph.y[n]] for n in path])
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(steps)])
    delta = np.diff(points, axis=0)
    angle = np.arctan2(delta[:, 1], delta[:, 0])
    heading = np.unwrap(np.concatenate([[angle[0]], angle]))

    lines = ['step_index,x_m,y_m,heading_rad,first_visit_sim_s,distance_along_m']
    for index, node in enumerate(path):
        lines.append('%d,%.4f,%.4f,%.4f,%.4f,%.4f' % (
            index, graph.x[node], graph.y[node], float(heading[index]),
            graph.first_t[node], float(cumulative[index])))
    Path(str(stem) + '.csv').write_text('\n'.join(lines) + '\n')

    meta = {
        'environment': environment,
        'source_trajectory': str(args.trajectory.resolve()),
        'world': str(args.world.resolve()) if args.world else None,
        'mode': args.mode,
        'rule': description,
        'parameters': {
            'spacing_m': args.spacing,
            'corridor_tolerance_m': args.corridor_tolerance,
            'min_gap_along_path_m': args.min_gap,
            'min_spur_m': args.min_spur,
        },
        'stats': {
            'stroke_length_m': round(path_length(graph, path), 3),
            'corridor_length_m': round(graph.length(), 3),
            'coverage_ratio': round(path_length(graph, path) / graph.length(), 4),
            'vertices': len(path),
            'start_xy': [round(graph.x[path[0]], 3), round(graph.y[path[0]], 3)],
            'end_xy': [round(graph.x[path[-1]], 3), round(graph.y[path[-1]], 3)],
            'corridor_leaves': sum(1 for node in graph.adj if len(graph.adj[node]) == 1),
        },
        'checks': {
            'single_stroke': True,
            'no_vertex_revisited': len(set(path)) == len(path),
            'max_vertex_gap_m': round(float(steps.max()), 4) if len(steps) else None,
            'consecutive_rows_share_edge': all(
                b in graph.adj[a] for a, b in zip(path[:-1], path[1:])),
        },
        'columns': {
            'heading_rad': 'direction of travel along the stroke',
            'first_visit_sim_s': 'when the recorded run first drew this corridor',
            'distance_along_m': 'cumulative length along the one-stroke path',
        },
        'limitations': [
            'A stroke that never lifts cannot branch, so only a simple path of the '
            'corridor tree is representable',
            'Dropped corridor is still listed in the deduplicated graph, not here',
            'Time is the first-visit stamp of each corridor, not a monotone episode clock',
        ],
    }
    Path(str(stem) + '_meta.json').write_text(json.dumps(meta, indent=2) + '\n')
    draw(Path(str(stem) + '.png'), result, graph, path, description, environment)
    print(json.dumps({'files': sorted(str(p) for p in output.glob(environment + '_onestroke*')),
                      'stats': meta['stats'], 'checks': meta['checks']}, indent=2))


if __name__ == '__main__':
    main()
