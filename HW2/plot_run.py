"""Report figures and tables from one hw2_nav run (runs on the Mac, no ROS).

python HW2/plot_run.py [HW2/logs/run_<time>] [--truth truth.yaml]

Writes into the run directory:
  trajectory.png  estimated pose every 0.5 s, tag updates, waypoints, tags
  summary.md      total distance, smoothness numbers, waypoint estimates / errors

truth.yaml (measured with a tape at each waypoint during the video):
  waypoints:
    - {wp: 1, x: 1.01, y: -0.02, theta_deg: 1.5}
"""
import argparse
import csv
import glob
import math
import os

import matplotlib
import yaml

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def read_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def latest_run(log_dir):
    runs = sorted(glob.glob(os.path.join(log_dir, 'run_*')))
    if not runs:
        raise SystemExit(f'no runs in {log_dir}')
    return runs[-1]


def load_run(run_dir):
    def path(name):
        return os.path.join(run_dir, name)
    with open(path('config.yaml')) as f:
        config = yaml.safe_load(f)
    return {name: read_csv(path(f'{name}.csv'))
            for name in ('trajectory', 'steps', 'observations', 'waypoints')} | {'config': config}


def load_truth(path):
    with open(path) as f:
        return {int(w['wp']): w for w in yaml.safe_load(f)['waypoints']}


# ---------- numbers ----------

def total_motion(steps):
    """(metres driven, degrees turned) over all primitives."""
    drive = sum(float(s['amount']) for s in steps if s['kind'] == 'drive')
    spin = sum(float(s['amount']) for s in steps if s['kind'] == 'spin')
    return drive, spin


def smoothness(steps, trajectory):
    """Counts and times that describe how smoothly the robot moved."""
    t_end = max(float(r['t']) for r in trajectory) if trajectory else 0.0
    moving = sum(float(s['t_stop']) - float(s['t_start']) for s in steps)
    cmds = sorted({float(s['cmd']) for s in steps})
    return {
        'primitives': len(steps),
        'drives': sum(s['kind'] == 'drive' for s in steps),
        'spins': sum(s['kind'] == 'spin' for s in steps),
        'per_level': {c: sum(float(s['cmd']) == c for s in steps) for c in cmds},
        'run_time': t_end,
        'moving_time': moving,
    }


def pose_of(row, prefix=''):
    return (float(row[f'{prefix}x']), float(row[f'{prefix}y']),
            float(row[f'{prefix}theta_deg']))


def pose_error(a, b):
    """a - b: (translation m, rotation deg)."""
    return math.hypot(a[0] - b[0], a[1] - b[1]), wrap_deg(a[2] - b[2])


def waypoint_rows(waypoints, truth=None):
    """Per waypoint: goal, estimate, and errors when the true pose is known."""
    rows = []
    for w in waypoints:
        goal = (float(w['goal_x']), float(w['goal_y']), float(w['goal_theta_deg']))
        est = pose_of(w)
        row = {'wp': int(w['wp']), 'goal': goal, 'est': est, 'reached': w['reached'] == '1',
               'tag_based': w['tag_based'] == '1', 'tags': w['tags'], 'note': w['note'],
               'est_vs_goal': pose_error(est, goal)}
        t = (truth or {}).get(row['wp'])
        if t is not None:
            true = (float(t['x']), float(t['y']), float(t['theta_deg']))
            row['true'] = true
            row['localization'] = pose_error(est, true)     # how wrong the estimate was
            row['arrival'] = pose_error(true, goal)          # how far from the goal it stopped
        rows.append(row)
    return rows


# ---------- outputs ----------

def fmt_pose(p):
    return f'({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.1f}°)'


def fmt_err(e):
    return f'{e[0] * 100:.1f} cm, {e[1]:+.1f}°'


def summary_markdown(run_dir, run, truth):
    drive, spin = total_motion(run['steps'])
    sm = smoothness(run['steps'], run['trajectory'])
    rows = waypoint_rows(run['waypoints'], truth)
    lines = [f'# HW2 run `{os.path.basename(run_dir)}`', '',
             '## Total motion', '',
             f'- driven: **{drive:.3f} m**',
             f'- turned: **{spin:.1f}°** ({math.radians(spin):.2f} rad)',
             '- planned minimum: 5.236 m and 360°', '',
             '## Smoothness', '',
             f'- primitives: {sm["primitives"]} ({sm["drives"]} drives, {sm["spins"]} spins); '
             f'each one starts and ends at rest',
             '- per command level: '
             + ', '.join(f'{c:g}: {n}' for c, n in sm['per_level'].items()),
             f'- run time {sm["run_time"]:.1f} s, moving {sm["moving_time"]:.1f} s '
             f'({100 * sm["moving_time"] / max(sm["run_time"], 1e-9):.0f} %)', '',
             '## Waypoints', '']
    if truth:
        lines += ['| wp | goal | estimate | true (tape) | localization error | arrival error |',
                  '|---|---|---|---|---|---|']
        for r in rows:
            true = fmt_pose(r['true']) if 'true' in r else '—'
            loc = fmt_err(r['localization']) if 'true' in r else '—'
            arr = fmt_err(r['arrival']) if 'true' in r else '—'
            lines.append(f'| {r["wp"]} | {fmt_pose(r["goal"])} | {fmt_pose(r["est"])} | '
                         f'{true} | {loc} | {arr} |')
    else:
        lines += ['| wp | goal | estimate | estimate − goal | tags | note |',
                  '|---|---|---|---|---|---|']
        for r in rows:
            lines.append(f'| {r["wp"]} | {fmt_pose(r["goal"])} | {fmt_pose(r["est"])} | '
                         f'{fmt_err(r["est_vs_goal"])} | {r["tags"]} | {r["note"]} |')
        lines += ['', 'No truth.yaml: add the tape-measured pose of each waypoint to get '
                  'localization and arrival errors.']
    return '\n'.join(lines) + '\n'


def draw_arrow(ax, x, y, theta_deg, length, **kw):
    t = math.radians(theta_deg)
    ax.annotate('', xy=(x + length * math.cos(t), y + length * math.sin(t)), xytext=(x, y),
                arrowprops=dict(arrowstyle='->', **kw))


def plot_trajectory(run, truth, out_path):
    traj = run['trajectory']
    grid = [r for r in traj if r['source'] in ('start', 'hold', 'predict')]
    updates = [r for r in traj if r['source'] == 'update']
    fig, ax = plt.subplots(figsize=(6, 7))
    ax.plot([float(r['x']) for r in grid], [float(r['y']) for r in grid], '-', color='0.8',
            zorder=1)
    ax.scatter([float(r['x']) for r in grid], [float(r['y']) for r in grid], s=10,
               color='tab:blue', label='estimate every 0.5 s', zorder=2)
    ax.scatter([float(r['x']) for r in updates], [float(r['y']) for r in updates], s=30,
               marker='x', color='tab:orange', label='tag update', zorder=3)
    for w in run['waypoints']:
        gx, gy, gth = float(w['goal_x']), float(w['goal_y']), float(w['goal_theta_deg'])
        ax.scatter([gx], [gy], s=80, facecolors='none', edgecolors='k', zorder=4)
        draw_arrow(ax, gx, gy, gth, 0.15, color='k')
        ax.annotate(f'WP{w["wp"]}', (gx, gy), textcoords='offset points', xytext=(8, -12))
    for lm in run['config']['map']['landmarks']:
        ax.scatter([lm['x']], [lm['y']], s=60, marker='s', color='tab:green', zorder=4)
        draw_arrow(ax, lm['x'], lm['y'], lm['yaw_deg'], 0.15, color='tab:green')
        ax.annotate(f'tag {lm["id"]}', (lm['x'], lm['y']), textcoords='offset points',
                    xytext=(6, 6), color='tab:green')
    if truth:
        tx = [float(t['x']) for t in truth.values()]
        ty = [float(t['y']) for t in truth.values()]
        ax.scatter(tx, ty, s=40, marker='+', color='tab:red', label='true (tape)', zorder=5)
    ax.set_aspect('equal')
    ax.margins(0.12)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Estimated trajectory (world frame)')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.08), ncol=3, fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('run_dir', nargs='?', help='default: newest run in HW2/logs')
    ap.add_argument('--truth', help='truth.yaml (default: <run_dir>/truth.yaml if present)')
    a = ap.parse_args()
    run_dir = a.run_dir or latest_run(os.path.join(HERE, 'logs'))
    truth_path = a.truth or os.path.join(run_dir, 'truth.yaml')
    truth = load_truth(truth_path) if os.path.exists(truth_path) else None
    run = load_run(run_dir)
    plot_trajectory(run, truth, os.path.join(run_dir, 'trajectory.png'))
    md = summary_markdown(run_dir, run, truth)
    with open(os.path.join(run_dir, 'summary.md'), 'w') as f:
        f.write(md)
    print(md)
    print(f'written: {run_dir}/trajectory.png, summary.md')


if __name__ == '__main__':
    main()
