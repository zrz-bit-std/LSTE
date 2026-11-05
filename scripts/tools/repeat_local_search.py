#!/usr/bin/env python3
"""Run independent local searches until three newly recorded successes exist."""
import argparse
import csv
import datetime as dt
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import shlex
import subprocess
import sys
import time


WS = Path(__file__).resolve().parents[2]


def save(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')
    temporary.replace(path)


def new_records(path, offset):
    if not path.exists():
        return [], offset
    records = []
    with path.open() as stream:
        stream.seek(offset)
        while True:
            start = stream.tell()
            line = stream.readline()
            if not line or not line.endswith('\n'):
                return records, start
            if ' event=' in line and ' data=' in line:
                event, payload = line.split(' event=', 1)[1].split(' data=', 1)
                data = json.loads(payload)
                data['_logged_wall_epoch'] = dt.datetime.fromisoformat(line.split(' level=', 1)[0]).timestamp()
                records.append((event, data))


class StationaryWatch:
    """Translation outside the recorded radius starts a fresh timeout window."""
    def __init__(self, seconds=50.0, radius=0.35):
        self.seconds, self.radius = seconds, radius
        self.anchor = None
        self.since = None

    def observe(self, pose, now):
        if self.anchor is None or math.hypot(
            pose[0] - self.anchor[0], pose[1] - self.anchor[1]
        ) > self.radius:
            self.anchor, self.since = list(pose), now

    def age(self, now):
        return 0.0 if self.since is None else now - self.since

    def stalled(self, now):
        return self.since is not None and self.age(now) > self.seconds


def lifecycle(script, environment, output, wait=True):
    command = ['bash', '-lc',
               'source scripts/config/pipeline_env.sh; exec ' + script]
    process = subprocess.Popen(command, cwd=str(WS), env=environment,
                               stdout=output, stderr=subprocess.STDOUT)
    if wait:
        return process.wait(timeout=120)
    return process


def capture_windows(directory):
    for window in ['goal', 'global_frontier', 'teb_nav', 'cmd_vel_mux', 'metrics']:
        with (directory / (directory.name + '_' + window + '_tail.log')).open('w') as stream:
            subprocess.run(['tmux', 'capture-pane', '-pJ', '-t', 'lste:' + window,
                            '-S', '-300'], stdout=stream, stderr=subprocess.STDOUT,
                           timeout=10, check=False)


def capture_streams(directory, attached):
    """Retain each new tmux pane's output until the experiment is stopped."""
    result = subprocess.run(['tmux', 'list-windows', '-t', 'lste', '-F', '#{window_name}'],
                            capture_output=True, text=True, timeout=5)
    for window in result.stdout.splitlines():
        if window in attached:
            continue
        target = 'lste:' + window
        existing = subprocess.run(['tmux', 'display-message', '-p', '-t', target, '#{pane_pipe}'],
                                  capture_output=True, text=True, timeout=5)
        if existing.stdout.strip() == '1':
            attached.add(window)
            continue
        path = directory / (directory.name + '_' + window + '.log')
        with path.open('a') as stream:
            subprocess.run(['tmux', 'capture-pane', '-pJ', '-t', target, '-S', '-1000'],
                           stdout=stream, stderr=subprocess.STDOUT, timeout=5)
        command = 'exec tee -a ' + shlex.quote(str(path)) + ' >/dev/null'
        pipe = subprocess.run(['tmux', 'pipe-pane', '-o', '-t', target, command],
                              capture_output=True, timeout=5)
        if pipe.returncode == 0:
            attached.add(window)


def write_summary(campaign, state):
    rows = []
    lines = ['# 本地黄杯搜索重复实验', '',
             '本批次新增成功：%d / %d。历史成功实验不计入。' %
             (state['success_count'], state['required_successes']), '',
             '判失败：距本轮最近一个位置锚点 %.2f m 内持续无平移进展超过 %.0f 秒墙钟时间；'
             '转动与小幅里程计噪声不重置计时。每轮完整重启 Gazebo 与业务节点。' %
             (state['stationary_radius_m'], state['stall_seconds']), '',
             '累计探索超过 %.0f 秒墙钟时间也判失败；探索时间从指标 run_start 记录开始，单独记录启动耗时。' %
             state.get('max_exploration_seconds',600), '',
             '| 轮次 | 运行编号 | 结果 | 行程 m | 仿真时间 s | 墙钟时间 s | 图 |',
             '| --- | --- | --- | --- | --- | --- | --- |']
    for attempt in state['attempts']:
        directory = Path(attempt['source_log']).parent
        summary_path = directory / 'visualization/visualization_summary.json'
        summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
        png = next((p for p in summary.get('figures', []) if p.endswith('.png')), None)
        figure = '[PNG](%s)' % os.path.relpath(png, campaign) if png else '绘图未完成'
        values = [attempt['attempt'], attempt['run'], attempt['status'],
                  summary.get('path_length_m'), summary.get('observed_simulation_seconds'),
                  summary.get('observed_wall_seconds')]
        rows.append(values + [attempt.get('reason'), png])
        cells = ['—' if v is None else ('%.2f' % v if isinstance(v, float) else str(v)) for v in values]
        lines.append('| ' + ' | '.join(cells + [figure]) + ' |')
    lines += ['', '各图包含场景与实测轨迹、目标区域、结果证据和行程时间线。'
              '缺失的检测、相机或深度证据不会补造。',
              '图中底图来自 world 碰撞几何，不是 SLAM 栅格；轨迹叠加假设 odom/world 名义对齐。', '']
    (campaign / 'README.md').write_text('\n'.join(lines))
    with (campaign / 'results.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['attempt','run','status','path_length_m','simulation_seconds',
                         'wall_seconds','stop_reason','figure_png'])
        writer.writerows(rows)


def render_attempt(result, world):
    directory = Path(result['source_log']).parent
    history = result.setdefault('visualization_attempts', [])
    if not history and result.get('visualization_exit_code') is not None:
        history.append(result['visualization_exit_code'])
    with (directory / (result['run'] + '_visualization.log')).open('a') as output:
        command = [sys.executable, '-X', 'faulthandler', str(WS/'scripts/tools/plot_world_run.py'),
                   result['source_log'], '--world', str(world),
                   '--output-dir', str(directory/'visualization')]
        for _ in range(2):
            code = subprocess.run(command, stdout=output, stderr=subprocess.STDOUT, timeout=180).returncode
            history.append(code)
            result['visualization_exit_code'] = code
            if code >= 0:
                break
            output.write('\nNative plotting process exited %d; one bounded retry follows.\n' % code)
            output.flush()
            time.sleep(2)
        subprocess.run([sys.executable, '-X', 'faulthandler',
                        str(WS/'scripts/tools/plot_place_dfs.py'),
                        result['source_log'], '--world', str(world),
                        '--output', str(directory/'visualization'
                                        /(result['run']+'_place_dfs.png'))],
                       stdout=output, stderr=subprocess.STDOUT, timeout=180)
    save(directory/'attempt_result.json', result)


def run_attempt(campaign, number, args, resume=None):
    run = resume['run'] if resume else dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    directory = WS / 'runtime/navigation/logs' / run
    directory.mkdir(parents=True, exist_ok=bool(resume))
    if not (directory/'pipeline_config.yaml').exists():
        shutil.copy2(campaign/'pipeline_config.yaml',directory/'pipeline_config.yaml')
    metrics = directory / (run + '_navigation_metrics.log')
    environment = dict(os.environ, LSTE_WS=str(WS), LSTE_NO_ATTACH='1',
                       NAVIGATION_RUN_DIRECTORY=str(directory),
                       PIPELINE_CONFIG=str(campaign / 'pipeline_config.yaml'))
    result = dict(resume) if resume else {'run': run, 'attempt': number, 'status': 'running',
              'started_at': dt.datetime.now().isoformat(),
              'source_log': str(metrics), 'world': str(args.world),
              'stall_seconds': args.stall_seconds, 'stationary_radius_m': args.radius}
    result['stall_seconds'] = args.stall_seconds
    result['max_exploration_seconds'] = args.max_exploration_seconds
    save(directory / 'attempt_result.json', result)
    save(campaign / 'current_attempt.json', result)
    watch = StationaryWatch(args.stall_seconds, args.radius)
    offset, last_report = 0, 0
    latest = {}
    exploration_started_wall = None
    attached = set()
    started = time.monotonic() - (max(0, time.time()-dt.datetime.fromisoformat(result['started_at']).timestamp()) if resume else 0)
    with (directory / (run + '_lifecycle.log')).open('a' if resume else 'w', buffering=1) as output:
        launcher = None
        if not resume:
            lifecycle('./scripts/lifecycle/stop_all_tmux.sh', environment, output)
            launcher = lifecycle('./scripts/lifecycle/run_all_tmux.sh', environment, output, False)
        else:
            output.write('[resume] Continued existing experiment with stationary timeout %.0f s\n' % args.stall_seconds)
        try:
            while True:
                now = time.monotonic()
                capture_streams(directory, attached)
                policy = json.loads((campaign/'policy.json').read_text())
                if watch.seconds != float(policy['stall_seconds']):
                    watch.seconds = args.stall_seconds = float(policy['stall_seconds'])
                    result['stall_seconds'] = watch.seconds
                    save(directory/'attempt_result.json', result)
                    save(campaign/'current_attempt.json', result)
                args.max_exploration_seconds = float(policy.get('max_exploration_seconds',600))
                records, offset = new_records(metrics, offset)
                for event, data in records:
                    if event == 'run_start':
                        exploration_started_wall = data['_logged_wall_epoch']
                    if event == 'sample' and data.get('pose'):
                        if exploration_started_wall is None:
                            exploration_started_wall = data['_logged_wall_epoch']
                        latest = data
                        observed = now - max(0, time.time()-data['_logged_wall_epoch'])
                        watch.observe(data['pose'], observed)
                    if event == 'task_done' and data.get('value') is True:
                        elapsed = data['_logged_wall_epoch'] - exploration_started_wall if exploration_started_wall is not None else None
                        if elapsed is not None and elapsed <= args.max_exploration_seconds:
                            result.update(status='success', completion_event=data,
                                          exploration_wall_seconds=elapsed)
                elapsed = time.time()-exploration_started_wall if exploration_started_wall is not None else None
                if result['status'] == 'success':
                    break
                if elapsed is not None and elapsed > args.max_exploration_seconds:
                    result.update(status='failed', reason='exploration_over_%g_wall_seconds' % args.max_exploration_seconds,
                                  exploration_wall_seconds=elapsed,
                                  stop_pose=latest.get('pose'), last_sample=latest)
                    break
                if watch.stalled(now):
                    result.update(status='failed', reason='stationary_over_%g_wall_seconds' % args.stall_seconds,
                                  stall_pose=latest.get('pose'),
                                  stall_anchor=watch.anchor,
                                  stationary_wall_seconds=watch.age(now),
                                  last_sample=latest)
                    break
                code = launcher.poll() if launcher is not None else 0
                if code is not None and code != 0:
                    result.update(status='infrastructure_failure',
                                  reason='launcher_failed', launcher_exit_code=code)
                    break
                if not latest and now - started > 60:
                    result.update(status='infrastructure_failure', reason='no_metrics_pose_after_60s')
                    break
                if now - last_report >= 30:
                    print(json.dumps({'run': run, 'attempt': number,
                                      'pose': latest.get('pose'),
                                      'path_length_m': latest.get('path_length'),
                                      'exploration_wall_seconds': None if elapsed is None else round(elapsed,1),
                                      'stationary_wall_seconds': round(watch.age(now), 1)},
                                     ensure_ascii=False), flush=True)
                    save(campaign / 'live_status.json', dict(result, pose=latest.get('pose'),
                         stationary_wall_seconds=watch.age(now), last_sample=latest,
                         exploration_wall_seconds=elapsed,
                         runner_pid=os.getpid()))
                    last_report = now
                time.sleep(2)
        finally:
            capture_windows(directory)
            if launcher is not None and launcher.poll() is None:
                launcher.terminate()
                launcher.wait(timeout=15)
            lifecycle('./scripts/lifecycle/stop_all_tmux.sh', environment, output)
    result['ended_at'] = dt.datetime.now().isoformat()
    result['wall_seconds_including_startup'] = round(time.monotonic() - started, 3)
    save(directory / 'attempt_result.json', result)
    save(campaign / 'current_attempt.json', result)
    render_attempt(result,args.world)
    save(directory / 'attempt_result.json', result)
    save(campaign / 'current_attempt.json', result)
    print(json.dumps({k: result[k] for k in ['run', 'attempt', 'status', 'visualization_exit_code'] }), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--campaign-dir', type=Path, required=True)
    parser.add_argument('--successes', type=int, default=3)
    parser.add_argument('--stall-seconds', type=float, default=50.0)
    parser.add_argument('--radius', type=float, default=0.35)
    parser.add_argument('--max-exploration-seconds', type=float, default=600.0)
    parser.add_argument('--resume-active', action='store_true', help='Attach to the recorded live trial without restarting Gazebo.')
    args = parser.parse_args()
    campaign = args.campaign_dir.resolve()
    campaign.mkdir(parents=True, exist_ok=True)
    with (campaign / 'runner.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        import yaml
        config = campaign / 'pipeline_config.yaml'
        if not config.exists():
            shutil.copy2(WS / 'scripts/config/pipeline_defaults.yaml', config)
        values = yaml.safe_load(config.read_text())
        args.world = WS / values['WORLD']
        shutil.copy2(args.world, campaign / args.world.name)
        state_path = campaign / 'campaign.json'
        state = json.loads(state_path.read_text()) if state_path.exists() else {
            'created_at': dt.datetime.now().isoformat(), 'required_successes': args.successes,
            'world': str(args.world), 'task_id': values['TASK_ID'],
            'git_revision': subprocess.check_output(['git','rev-parse','HEAD'],cwd=str(WS),text=True).strip(),
            'stall_seconds': args.stall_seconds, 'stationary_radius_m': args.radius,
            'max_exploration_seconds': args.max_exploration_seconds,
            'attempts': [], 'success_count': 0, 'status': 'running'}
        if state['stall_seconds'] != args.stall_seconds:
            state.setdefault('policy_changes', []).append({
                'at': dt.datetime.now().isoformat(), 'previous_seconds': state['stall_seconds'],
                'new_seconds': args.stall_seconds, 'reason': 'user_requested_timeout_change'})
            state['stall_seconds'] = args.stall_seconds
        save(state_path, state)
        save(campaign/'policy.json', {'stall_seconds': args.stall_seconds,
                                     'max_exploration_seconds': args.max_exploration_seconds,
                                     'stationary_radius_m': args.radius})
        write_summary(campaign, state)
        for attempt in state['attempts']:
            if attempt.get('visualization_exit_code') != 0:
                render_attempt(attempt,args.world)
                save(state_path,state)
                write_summary(campaign,state)
                if attempt['visualization_exit_code'] != 0:
                    raise SystemExit('Visualization repair failed; inspect preserved logs.')
        while state['success_count'] < args.successes:
            args.stall_seconds = float(json.loads((campaign/'policy.json').read_text())['stall_seconds'])
            state['stall_seconds'] = args.stall_seconds
            state['max_exploration_seconds'] = float(json.loads((campaign/'policy.json').read_text()).get('max_exploration_seconds',600))
            resume = None
            if args.resume_active:
                resume = json.loads((campaign/'current_attempt.json').read_text())
                if resume['status'] != 'running' or any(a['run'] == resume['run'] for a in state['attempts']):
                    raise SystemExit('No unrecorded active trial to resume.')
                args.resume_active = False
            result = run_attempt(campaign, len(state['attempts']) + 1, args, resume)
            state['attempts'].append(result)
            state['success_count'] = sum(a['status'] == 'success' for a in state['attempts'])
            save(state_path, state)
            write_summary(campaign, state)
            if result['visualization_exit_code'] != 0 or result['status'] == 'infrastructure_failure':
                raise SystemExit('Attempt saved; inspect infrastructure/visualization before resuming.')
        state['status'] = 'three_successes_recorded_pending_audit'
        save(state_path, state)


if __name__ == '__main__':
    main()
