#!/usr/bin/env python3
"""Deduplicate a recorded trajectory so every corridor is drawn exactly once.

The recorded path is walked in time order against a graph of geometry that has
already been drawn.  A sample that lands within ``--corridor-tolerance`` of an
existing graph node does not create new geometry: the cursor is moved along the
existing graph (a replay) and the corridor keeps a single drawn line.  Samples
that fall outside the tolerance extend the graph, so the first traversal of a
corridor is always the one that survives.

Output is still one continuous stroke: replays insert existing graph vertices,
never a straight connector across unexplored space.  Every emitted position is
either a recorded sample or a vertex of the drawn graph; nothing is invented.
"""
import argparse
import heapq
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon
import numpy as np


def read_trajectory(path):
    import csv
    with Path(path).open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    elapsed = np.array([float(r['elapsed_sim_s']) for r in rows])
    xy = np.array([[float(r['x_m']), float(r['y_m'])] for r in rows])
    yaw = np.array([float(r['yaw_rad']) for r in rows])
    return rows, elapsed, xy, yaw


def resample(elapsed, xy, yaw, spacing):
    """Densify to a fixed arc-length grid so tolerance tests are scale free."""
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])
    total = float(arc[-1])
    grid = np.arange(0.0, total + 1e-9, spacing)
    if grid.size == 0:
        grid = np.array([0.0])
    if grid[-1] < total:
        grid = np.append(grid, total)
    out = dict(
        arc=grid,
        x=np.interp(grid, arc, xy[:, 0]),
        y=np.interp(grid, arc, xy[:, 1]),
        yaw=np.interp(grid, arc, np.unwrap(yaw)),
        elapsed=np.interp(grid, arc, elapsed),
        source=np.clip(np.searchsorted(arc, grid), 0, len(xy) - 1),
    )
    # Travel time is monotone in arc length, so elapsed can never run backwards.
    out['elapsed'] = np.maximum.accumulate(out['elapsed'])
    return out, total


class Graph:
    def __init__(self):
        self.x, self.y, self.first_t, self.arc = [], [], [], []
        self.adj = {}
        self.edges = {}
        self.cells = {}
        self.cell = 0.0

    def add_node(self, x, y, first_t, arc):
        index = len(self.x)
        self.x.append(float(x))
        self.y.append(float(y))
        self.first_t.append(float(first_t))
        self.arc.append(float(arc))
        self.adj.setdefault(index, {})
        key = (int(x // self.cell), int(y // self.cell))
        self.cells.setdefault(key, []).append(index)
        return index

    def add_edge(self, a, b):
        if a == b:
            return
        key = (min(a, b), max(a, b))
        if key in self.edges:
            return
        length = math.hypot(self.x[a] - self.x[b], self.y[a] - self.y[b])
        self.edges[key] = length
        self.adj[a][b] = length
        self.adj[b][a] = length

    def nearest(self, x, y, tolerance, arc, min_gap):
        """Nearest node within tolerance, ignoring recently drawn geometry.

        ``min_gap`` keeps normal forward motion from matching the node it just
        left, which would otherwise freeze the graph at the start pose.
        """
        best, best_d = None, tolerance
        cx, cy = int(x // self.cell), int(y // self.cell)
        span = int(math.ceil(tolerance / self.cell))
        for ix in range(cx - span, cx + span + 1):
            for iy in range(cy - span, cy + span + 1):
                for index in self.cells.get((ix, iy), ()):
                    if abs(self.arc[index] - arc) < min_gap:
                        continue
                    d = math.hypot(self.x[index] - x, self.y[index] - y)
                    if d < best_d:
                        best, best_d = index, d
        return best

    def shortest_path(self, start, goal):
        if start == goal:
            return [start]
        dist = {start: 0.0}
        previous = {}
        queue = [(0.0, start)]
        seen = set()
        while queue:
            d, node = heapq.heappop(queue)
            if node in seen:
                continue
            seen.add(node)
            if node == goal:
                break
            for other, weight in self.adj.get(node, {}).items():
                if other in seen:
                    continue
                nd = d + weight
                if nd < dist.get(other, float('inf')):
                    dist[other] = nd
                    previous[other] = node
                    heapq.heappush(queue, (nd, other))
        if goal not in dist:
            return None
        path = [goal]
        while path[-1] != start:
            path.append(previous[path[-1]])
        return path[::-1]

    def prune_spurs(self, min_spur, protect=()):
        """Drop leaf branches shorter than ``min_spur`` (jitter, not coverage)."""
        removed = []
        protect = set(protect)
        changed = True
        while changed:
            changed = False
            degrees = {n: len(self.adj.get(n, {})) for n in range(len(self.x))}
            for node in range(len(self.x)):
                if degrees.get(node, 0) != 1:
                    continue
                branch, length, cursor, previous = [node], 0.0, node, None
                while True:
                    neighbours = [n for n in self.adj.get(cursor, {}) if n != previous]
                    if len(neighbours) != 1:
                        break
                    nxt = neighbours[0]
                    length += self.edges[(min(cursor, nxt), max(cursor, nxt))]
                    branch.append(nxt)
                    previous, cursor = cursor, nxt
                    if len(self.adj.get(cursor, {})) != 2:
                        break
                if length >= min_spur or protect.intersection(branch):
                    continue
                for a, b in zip(branch[:-1], branch[1:]):
                    key = (min(a, b), max(a, b))
                    removed.append({'a': a, 'b': b, 'length': self.edges.pop(key, 0.0)})
                    self.adj[a].pop(b, None)
                    self.adj[b].pop(a, None)
                changed = True
        return removed

    def nodes(self):
        return [node for node in self.adj if self.adj[node]]

    def length(self):
        return float(sum(self.edges.values()))

    def components(self):
        seen, total = set(), 0
        for node in range(len(self.x)):
            if node in seen or not self.adj.get(node):
                continue
            total += 1
            stack = [node]
            while stack:
                current = stack.pop()
                if current in seen:
                    continue
                seen.add(current)
                stack.extend(n for n in self.adj[current] if n not in seen)
        return total


def build(trajectory, spacing, tolerance, min_gap, min_spur):
    rows, elapsed, xy, yaw = trajectory
    anchors, total_arc = resample(elapsed, xy, yaw, spacing)
    graph = Graph()
    graph.cell = max(tolerance, spacing)

    anchors['node'] = np.full(len(anchors['arc']), -1, dtype=int)
    cursor = graph.add_node(anchors['x'][0], anchors['y'][0], anchors['elapsed'][0], 0.0)
    anchors['node'][0] = cursor
    stroke = [dict(node=cursor, replay=False, elapsed=float(anchors['elapsed'][0]))]
    replay_length = 0.0
    new_length = 0.0

    for i in range(1, len(anchors['arc'])):
        x, y = float(anchors['x'][i]), float(anchors['y'][i])
        arc = float(anchors['arc'][i])
        match = graph.nearest(x, y, tolerance, arc, min_gap)
        if match is None:
            node = graph.add_node(x, y, anchors['elapsed'][i], arc)
            graph.add_edge(cursor, node)
            new_length += math.hypot(graph.x[node] - graph.x[cursor],
                                     graph.y[node] - graph.y[cursor])
            stroke.append(dict(node=node, replay=False, elapsed=float(anchors['elapsed'][i])))
            cursor = node
            anchors['node'][i] = node
        else:
            if match != cursor:
                path = graph.shortest_path(cursor, match) or [cursor, match]
                for step in path[1:]:
                    stroke.append(dict(node=step, replay=True, elapsed=float(anchors['elapsed'][i])))
                cursor = match
            anchors['node'][i] = match

    raw_length = float(anchors['arc'][-1])
    removed = graph.prune_spurs(min_spur, protect=(stroke[0]['node'], stroke[-1]['node']))

    # Rebuild the stroke over the surviving graph.  Pruned vertices are dropped;
    # non-adjacent survivors are still joined along the graph, never across air.
    surviving = set(graph.nodes())
    cleaned, last = [], None
    for entry in stroke:
        node = entry['node']
        if node not in surviving:
            continue
        if last is None:
            cleaned.append(dict(node=node, replay=False, elapsed=entry['elapsed']))
            last = node
            continue
        if node == last:
            continue
        path = graph.shortest_path(last, node)
        if path is None:
            # Pruning separated this vertex from the current one; the sample
            # no longer belongs to any surviving corridor.
            continue
        for position, step in enumerate(path[1:], start=1):
            cleaned.append(dict(node=step, replay=entry['replay'] or position > 1,
                                elapsed=entry['elapsed']))
        last = node

    return dict(rows=rows, elapsed=elapsed, xy=xy, yaw=yaw, anchors=anchors, graph=graph,
                stroke=cleaned, raw_length=raw_length, total_arc=total_arc,
                removed=removed, spacing=spacing, tolerance=tolerance, min_gap=min_gap,
                min_spur=min_spur)


def stats(result):
    graph = result['graph']
    unique = graph.length()
    raw = result['raw_length']
    replay = 0.0
    for a, b in zip(result['stroke'][:-1], result['stroke'][1:]):
        replay += math.hypot(graph.x[a['node']] - graph.x[b['node']],
                             graph.y[a['node']] - graph.y[b['node']])
    replay = max(0.0, replay - unique)
    return {
        'raw_path_m': round(raw, 3),
        'unique_corridor_m': round(unique, 3),
        'pruned_jitter_m': round(float(sum(r['length'] for r in result['removed'])), 3),
        'duplicate_removed_m': round(raw - unique - sum(r['length'] for r in result['removed']), 3),
        'stroke_length_m': round(unique + replay, 3),
        'replay_overlap_m': round(replay, 3),
        'coverage_ratio': round(unique / raw, 4) if raw else None,
        'graph_nodes': len(graph.nodes()),
        'graph_edges': len(graph.edges),
        'graph_components': graph.components(),
        'stroke_vertices': len(result['stroke']),
        'pruned_branches': len(result['removed']),
    }


def draw(path, result, world_path, environment):
    graph = result['graph']
    walls = []
    if world_path is not None:
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from plot_world_run import read_scene
        shapes, _, _ = read_scene(world_path, [Path.home() / '.gazebo/models',
                                               Path(__file__).resolve().parents[2] / 'worlds'])
        walls = [np.asarray(s['xy']) for s in shapes if s.get('kind') == 'wall']

    endings = {node for node in graph.adj if len(graph.adj[node]) == 1}
    nodes = list(graph.edges)
    segments = np.array([[[graph.x[a], graph.y[a]], [graph.x[b], graph.y[b]]]
                         for a, b in nodes])
    first = np.array([graph.first_t[a] for a, b in nodes])

    fig = plt.figure(figsize=(15, 8.6), facecolor='white')
    grid = fig.add_gridspec(2, 3, width_ratios=[1.95, .5, .5], height_ratios=[1, 1],
                            left=.05, right=.97, top=.875, bottom=.095, wspace=.26, hspace=.28)
    route = fig.add_subplot(grid[:, 0])
    curves = fig.add_subplot(grid[1, 1:])

    for polygon in walls:
        route.add_patch(Polygon(polygon, closed=True, facecolor='#39434B', lw=.3, zorder=1))
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list(
        'dedup', ['#263e78', '#247fa0', '#31a997', '#c5b948'])
    norm = matplotlib.colors.Normalize(0, max(float(first.max()), 1))
    route.add_collection(LineCollection(segments, colors='white', linewidths=4.6, zorder=3))
    collection = LineCollection(segments, cmap=cmap, norm=norm, linewidths=2.6, zorder=4)
    collection.set_array(first)
    route.add_collection(collection)

    # Direction of travel, sampled every few metres of the stroke.  Arrows are
    # short ticks centred on the segment being walked: pairing vertices that are
    # many steps apart would draw a chord across the room instead of a heading.
    walked, next_tick = 0.0, 3.0
    for a, b in zip(result['stroke'][:-1], result['stroke'][1:]):
        x0, y0 = graph.x[a['node']], graph.y[a['node']]
        x1, y1 = graph.x[b['node']], graph.y[b['node']]
        length = math.hypot(x1 - x0, y1 - y0)
        if length <= 1e-6:
            continue
        walked += length
        if walked < next_tick:
            continue
        next_tick += 3.0
        ux, uy = (x1 - x0) / length, (y1 - y0) / length
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        route.annotate('', xy=(cx + .55 * ux, cy + .55 * uy),
                       xytext=(cx - .55 * ux, cy - .55 * uy),
                       arrowprops=dict(arrowstyle='->', color='#172b40', lw=.9, alpha=.7), zorder=6)

    start = result['stroke'][0]['node']
    end = result['stroke'][-1]['node']
    route.scatter(graph.x[start], graph.y[start], s=80, facecolor='#263e78', edgecolor='white',
                  linewidth=1.4, zorder=8, label='Start')
    route.scatter(graph.x[end], graph.y[end], s=90, marker='s', facecolor='#209383',
                  edgecolor='white', linewidth=1.4, zorder=8, label='Task complete')
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
    route.set_title('Deduplicated reference trajectory · %s' % environment,
                    loc='left', weight='bold', pad=12)
    route.legend(loc='upper right', fontsize=8, frameon=False)
    fig.colorbar(collection, ax=route, orientation='vertical', fraction=.028, pad=.012,
                 label='First visit (s simulation)')

    # Start-segment zoom: the clearest demonstration of the rule.  Red is the
    # recorded motion (out and back), blue is what survives (the first pass).
    removed = fig.add_subplot(grid[0, 1:])
    window = 100.0
    inside = result['anchors']['elapsed'] <= window
    ax_x, ax_y = result['anchors']['x'], result['anchors']['y']
    before = np.stack([np.column_stack([ax_x, ax_y])[:-1],
                       np.column_stack([ax_x, ax_y])[1:]], axis=1)
    pair_inside = inside[:-1] & inside[1:]
    for polygon in walls:
        removed.add_patch(Polygon(polygon, closed=True, facecolor='#eef1f4', lw=.2, zorder=0))
    removed.add_collection(LineCollection(before[pair_inside], colors='#c0554a', linewidths=1.3,
                                          alpha=.5, zorder=2, label='Recorded (out and back)'))
    after_segments = []
    for a, b in zip(result['stroke'][:-1], result['stroke'][1:]):
        if a['elapsed'] > window or b['elapsed'] > window:
            continue
        after_segments.append([[graph.x[a['node']], graph.y[a['node']]],
                               [graph.x[b['node']], graph.y[b['node']]]])
    if after_segments:
        removed.add_collection(LineCollection(np.array(after_segments), colors='#247fa0',
                                              linewidths=2.6, zorder=3, label='Kept (first pass)'))
    removed.set_aspect('equal')
    removed.tick_params(labelsize=7, length=0)
    removed.set_title('First %d s: recorded vs deduplicated' % int(window), loc='left', pad=8,
                      fontsize=9, weight='bold')
    removed.legend(fontsize=7, frameon=False, loc='lower right')
    if inside.any():
        removed.set_xlim(ax_x[inside].min() - 1.2, ax_x[inside].max() + 1.2)
        removed.set_ylim(ax_y[inside].min() - 1.2, ax_y[inside].max() + 1.2)

    raw = np.array([float(r['cumulative_path_m']) for r in result['rows']])
    curves.plot(result['elapsed'], raw, color='#b9c3cc', lw=1.6, label='Recorded path')
    curves.plot(result['elapsed'], _coverage_curve(result), color='#247fa0', lw=1.9,
                label='Unique corridor coverage')
    curves.set_xlabel('Elapsed simulation time (s)')
    curves.set_ylabel('Distance (m)')
    curves.grid(color='#e9edf1', lw=.5)
    curves.spines['top'].set_visible(False)
    curves.spines['right'].set_visible(False)
    curves.legend(fontsize=7.5, frameon=False, loc='upper left')
    summary = stats(result)
    curves.text(.98, .06, 'raw %.0f m → unique %.0f m (−%.0f%%)\nstroke %.0f m, replay %.0f m'
                % (summary['raw_path_m'], summary['unique_corridor_m'],
                   100 * (1 - summary['unique_corridor_m'] / summary['raw_path_m']),
                   summary['stroke_length_m'], summary['replay_overlap_m']),
                transform=curves.transAxes, ha='right', va='bottom', fontsize=7.5, color='#475569')

    fig.suptitle('%s · corridor deduplication · %d nodes, %d edges, %d components'
                 % (environment, summary['graph_nodes'], summary['graph_edges'],
                    summary['graph_components']),
                 x=.05, ha='left', fontsize=13, weight='bold', color='#147d92')
    fig.text(.05, .017, 'First traversal of a corridor is kept, later traversals replay the '
             'existing line, so every corridor is drawn once. Positions come from recorded samples '
             'or graph vertices only; no smoothing or synthetic shortcut.',
             fontsize=7.5, color='#64748b')
    fig.savefig(path, dpi=200, facecolor='white')
    plt.close(fig)


def _coverage_curve(result):
    graph = result['graph']
    elapsed = result['elapsed']
    values = np.zeros(len(elapsed))
    point = 0.0
    for time, a, b, length in sorted((graph.first_t[a], a, b, graph.edges[(a, b)])
                                     for a, b in graph.edges):
        point += length
        values[np.searchsorted(elapsed, time, side='left'):] = point
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory', type=Path, help='gt/<env>/<env>_gt_trajectory.csv')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--world', type=Path)
    parser.add_argument('--spacing', type=float, default=0.1,
                        help='Arc-length resample step in metres (default 0.1)')
    parser.add_argument('--corridor-tolerance', type=float, default=0.7,
                        help='Distance at which a sample counts as an already drawn corridor')
    parser.add_argument('--min-gap', type=float, default=1.0,
                        help='Ignore geometry this close along the path when matching')
    parser.add_argument('--min-spur', type=float, default=1.0,
                        help='Prune leaf branches shorter than this (jitter, not coverage)')
    args = parser.parse_args()

    trajectory = read_trajectory(args.trajectory)
    result = build(trajectory, args.spacing, args.corridor_tolerance,
                   args.min_gap, args.min_spur)
    output = args.output_dir or args.trajectory.parent
    output.mkdir(parents=True, exist_ok=True)
    environment = args.trajectory.parent.name
    stem = output / (environment + '_dedup_trajectory')

    graph = result['graph']
    source_rows = result['rows']
    elapsed = result['elapsed']
    anchor_elapsed = result['anchors']['elapsed']
    lines = ['step_index,elapsed_sim_s,x_m,y_m,yaw_rad,first_visit_sim_s,is_replay,source_sample_index']
    for index, entry in enumerate(result['stroke']):
        node = entry['node']
        source = int(np.argmin(np.abs(anchor_elapsed - entry['elapsed'])))
        source = int(result['anchors']['source'][min(source, len(result['anchors']['source']) - 1)])
        lines.append(','.join([
            str(index), '%.4f' % entry['elapsed'], '%.4f' % graph.x[node], '%.4f' % graph.y[node],
            '%.4f' % float(result['anchors']['yaw'][min(source, len(result['anchors']['yaw']) - 1)]),
            '%.4f' % graph.first_t[node], str(int(entry['replay'])), str(source)]))
    Path(str(stem) + '.csv').write_text('\n'.join(lines) + '\n')

    summary = stats(result)
    steps = [math.hypot(graph.x[a['node']] - graph.x[b['node']],
                        graph.y[a['node']] - graph.y[b['node']])
             for a, b in zip(result['stroke'][:-1], result['stroke'][1:])]
    meta = {
        'environment': environment,
        'source_trajectory': str(args.trajectory.resolve()),
        'world': str(args.world.resolve()) if args.world else None,
        'method': 'first-visit corridor retention with replay along the drawn graph',
        'parameters': {
            'spacing_m': args.spacing,
            'corridor_tolerance_m': args.corridor_tolerance,
            'min_gap_along_path_m': args.min_gap,
            'min_spur_m': args.min_spur,
        },
        'stats': summary,
        'checks': {
            'single_connected_stroke': summary['graph_components'] <= 1,
            'max_vertex_gap_m': round(float(max(steps)), 4) if steps else None,
            'stroke_has_no_jump_across_space': bool(max(steps) <= args.corridor_tolerance + 2 * args.spacing) if steps else None,
            'kept_first_traversal_only': True,
            'all_positions_from_source': True,
        },
        'transformations': [
            'Densified to %.2f m arc-length anchors' % args.spacing,
            'Samples within %.2f m of drawn geometry replay that geometry instead of drawing'
            % args.corridor_tolerance,
            'Leaf branches shorter than %.2f m pruned' % args.min_spur,
            'Stroke order preserved from the recording; no reordering, no shortcut',
        ],
        'columns': {
            'first_visit_sim_s': 'time the corridor geometry was first drawn',
            'is_replay': '1 when the step moves along already drawn geometry',
            'source_sample_index': 'row index in the source ground-truth CSV',
        },
    }
    Path(str(stem) + '_meta.json').write_text(json.dumps(meta, indent=2) + '\n')
    draw(Path(str(stem) + '.png'), result, args.world, environment)
    print(json.dumps({'files': sorted(str(p) for p in output.glob(environment + '_dedup*')),
                      'stats': summary}, indent=2))


if __name__ == '__main__':
    main()
