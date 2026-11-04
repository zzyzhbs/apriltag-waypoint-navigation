#!/usr/bin/env python3
"""README figures and result tables, built only from the run logs and config files.

python scripts/make_figures.py [--logs DIR] [--config DIR] [--out DIR]

Defaults: HW2/logs, robot_control/config, docs/figures.

Writes four PNG figures and numbers.md, which lists every number the figures show.
Needs matplotlib and PyYAML. Imports nothing from robot_control and needs no ROS.
"""
import argparse
import collections
import csv
import datetime
import glob
import math
import os
import re
import statistics

import matplotlib
import yaml

matplotlib.use('Agg')
import matplotlib.lines as mlines  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK2 = '#52514e'
MUTED = '#898781'
GRID = '#e1e0d9'
AXIS = '#c3c2b7'
BLUE = '#2a78d6'
ORANGE = '#eb6834'
AQUA = '#1baf7a'

BEST_RUN = 'run_20261024_055456'
FALSE_COMPLETION_RUN = 'run_20261023_071302'
FINE_CMD = 0.25
COARSE_CMD = 0.35
DEFAULT_MIN_T = 0.2     # ol_model.MIN_T
EXTRAP = 0.2            # ol_model.EXTRAP

PHASES = {
    'A': 'coarse level only (`study_wood2`)',
    'B': 'adds CW time_scale 0.77 (`study_wood2_fixing`)',
    'C': 'adds fine level cmd 0.25, no gate',
    'D': 'adds `fine_max_deg` gate',
}


# ---------- loading ----------

def read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def read_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def load_runs(log_dir):
    runs = []
    for path in sorted(glob.glob(os.path.join(log_dir, 'run_*'))):
        name = os.path.basename(path)
        run = {'name': name, 'stamp': datetime.datetime.strptime(name, 'run_%Y%m%d_%H%M%S'),
               'config': read_yaml(os.path.join(path, 'config.yaml'))}
        for table in ('trajectory', 'steps', 'observations', 'waypoints'):
            run[table] = read_csv(os.path.join(path, f'{table}.csv'))
        runs.append(run)
    return runs


# ---------- numbers ----------

def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def is_complete(run):
    return len(run['waypoints']) == 3 and all(w['reached'] == '1' for w in run['waypoints'])


def fine_trials_end(calib):
    times = [t['time'] for d in ('ccw', 'cw') for t in calib['spin'][d]['trials']
             if t['cmd'] == FINE_CMD]
    return datetime.datetime.fromisoformat(max(times))


def phase_of(run, fine_end):
    if 'fine_max_deg' in run['config']['nav']['control']:
        return 'D'
    if run['config']['floor'] == 'study_wood2':
        return 'A'
    return 'B' if run['stamp'] <= fine_end else 'C'


def waypoint_rows(run):
    """Per waypoint: estimate minus goal, filter sigma, and tag update after the last primitive."""
    rows = []
    for w in run['waypoints']:
        steps = [s for s in run['steps'] if s['wp'] == w['wp']]
        last_stop = max((float(s['t_stop']) for s in steps), default=-math.inf)
        updated = any(o['wp'] == w['wp'] and o['accepted'] == '1' and float(o['t']) >= last_stop
                      for o in run['observations'])
        rows.append({
            'run': run['name'], 'wp': int(w['wp']),
            'pos_cm': 100 * math.hypot(float(w['x']) - float(w['goal_x']),
                                       float(w['y']) - float(w['goal_y'])),
            'head_deg': wrap_deg(float(w['theta_deg']) - float(w['goal_theta_deg'])),
            'sigma_pos_cm': 100 * math.hypot(float(w['sx']), float(w['sy'])),
            'sigma_head_deg': float(w['stheta_deg']),
            'updated': updated, 'note': w['note']})
    return rows


def observation_rows(runs):
    """All observation rows with a category, and geometry when the log has it."""
    rows = []
    for run in runs:
        for o in run['observations']:
            row = {'run': run['name'], 't': float(o['t']), 'id': o['id'], 'd2': o['d2'],
                   'dist': float(o['dist']) if o['dist'] else None,
                   'oblique': float(o['oblique_deg']) if o['oblique_deg'] else None}
            reason = o['reason']
            if o['accepted'] == '1':
                row['category'] = 'accepted'
            elif reason == 'gate':
                row['category'] = 'gate'
            elif 'only' in reason:
                row['category'] = 'too few frames'
            elif 'unstable' in reason:
                row['category'] = 'unstable'
            elif 'out of range' in reason:
                row['category'] = 'out of range'
                m = re.search(r'd ([\d.]+) m, oblique ([\d.]+) deg', reason)
                row['dist'], row['oblique'] = float(m.group(1)), float(m.group(2))
            else:
                raise ValueError(f'unknown reason: {reason!r}')
            rows.append(row)
    return rows


def first_looks(run):
    """Observation rows with geometry from the look before the first primitive."""
    if not run['steps']:
        return []
    start = float(run['steps'][0]['t_start'])
    return [o for o in run['observations'] if o['dist'] and float(o['t']) <= start + 0.01]


def first_leg_progress(run):
    """Fall in tag 0's measured x over the first drive, or None if a look is missing."""
    steps = run['steps']
    if not steps or steps[0]['kind'] != 'drive':
        return None
    start, stop = float(steps[0]['t_start']), float(steps[0]['t_stop'])
    next_start = float(steps[1]['t_start']) if len(steps) > 1 else math.inf
    tag0 = [o for o in run['observations'] if o['id'] == '0']
    before = [o for o in tag0 if float(o['t']) <= start + 0.01]
    after = [o for o in tag0 if stop <= float(o['t']) <= next_start + 0.01]
    if not before or not after:
        return None
    return {'cmd': float(steps[0]['amount']),
            'drop': float(before[0]['x']) - float(after[0]['x'])}


def consistent_with_halt(run):
    """True when the log ends the way NavigationHalt would leave it (inference).

    Navigator.check_consistency raises on the second look in a row with every tag rejected
    by the gate, before that look is logged. So the newest logged look with geometry is all
    gate rejections, a primitive ran after it, and the log stops one settle and look later.
    """
    looks = collections.OrderedDict()
    for o in run['observations']:
        if o['dist']:
            looks.setdefault(o['t'], []).append(o)
    if not looks or not run['steps']:
        return False
    t_look, last = list(looks.items())[-1]
    last_stop = float(run['steps'][-1]['t_stop'])
    return (all(o['reason'] == 'gate' for o in last) and last_stop > float(t_look)
            and float(run['trajectory'][-1]['t']) - last_stop <= 2.5)


def motion_totals(run):
    drive = sum(float(s['amount']) for s in run['steps'] if s['kind'] == 'drive')
    spin = sum(float(s['amount']) for s in run['steps'] if s['kind'] == 'spin')
    points = [(float(r['x']), float(r['y'])) for r in run['trajectory']]
    path = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    moving = sum(float(s['t_stop']) - float(s['t_start']) for s in run['steps'])
    return {'drive': drive, 'spin': spin, 'path': path, 'moving': moving,
            'primitives': len(run['steps']), 'time': float(run['trajectory'][-1]['t'])}


def fit_at(fit, T):
    return fit['rate'] * T + fit['offset']


def min_amount(calib, direction, cmd):
    """Same value as step_timing.min_amount: run for min_T, or min_T / time_scale if longer."""
    section = calib['spin'][direction]
    min_T = (section.get('min_T') or {}).get(cmd, DEFAULT_MIN_T)
    T = max(min_T, min_T / float(section.get('time_scale', 1.0)))
    return T, fit_at(calib['fit']['spin'][direction][cmd], T)


def drive_min_amount(calib, cmd):
    return fit_at(calib['fit']['drive'][cmd], DEFAULT_MIN_T)


def long_fine_spins(runs, calib):
    """Logged spins at the fine level that ran longer than its calibrated range allows."""
    out = []
    for run in runs:
        for s in run['steps']:
            if s['kind'] != 'spin' or float(s['cmd']) != FINE_CMD:
                continue
            t_max = calib['fit']['spin'][s['dir']][FINE_CMD]['t_max']
            if float(s['T']) > t_max * (1 + EXTRAP):
                out.append({'run': run['name'], 'dir': s['dir'], 'amount': float(s['amount']),
                            'T': float(s['T']), 'wp': s['wp']})
    return out


# ---------- figure helpers ----------

def set_style():
    plt.rcParams.update({
        'font.family': 'DejaVu Sans', 'font.size': 9.5,
        'axes.edgecolor': AXIS, 'axes.linewidth': 0.8, 'axes.labelcolor': INK2,
        'axes.spines.top': False, 'axes.spines.right': False,
        'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.7, 'axes.axisbelow': True,
        'axes.facecolor': SURFACE, 'figure.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
        'xtick.color': MUTED, 'ytick.color': MUTED,
        'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2,
        'legend.frameon': False, 'legend.fontsize': 8.8, 'text.color': INK2,
    })


def headline(fig, title, subtitle, x=0.075, y=0.975):
    fig.text(x, y, title, fontsize=12.5, fontweight='bold', color=INK, va='top')
    fig.text(x, y - 0.045 * 5.0 / fig.get_figheight(), subtitle, fontsize=9.3, color=INK2,
             va='top', linespacing=1.35)


def save(fig, out_dir, name):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=200, metadata={'Software': None})
    plt.close(fig)
    return path


def arrow(ax, x, y, theta_deg, length, color):
    t = math.radians(theta_deg)
    ax.annotate('', xy=(x + length * math.cos(t), y + length * math.sin(t)), xytext=(x, y),
                arrowprops=dict(arrowstyle='-|>', color=color, lw=1.2, shrinkA=0, shrinkB=0),
                zorder=6)


def dot_handle(color, label, hollow=False, marker='o', size=6.5):
    return mlines.Line2D([], [], linestyle='none', marker=marker, markersize=size,
                         markerfacecolor=SURFACE if hollow else color, markeredgecolor=color,
                         markeredgewidth=1.3 if hollow else 0.0, label=label)


# ---------- figures ----------

def figure_trajectories(runs, out_dir, wide=False):
    """Portrait version, or with wide=True a landscape one with the legend beside the map."""
    complete = [r for r in runs if is_complete(r)]
    others = [r for r in complete if r['name'] not in (BEST_RUN, FALSE_COMPLETION_RUN)]
    fig, ax = plt.subplots(figsize=(7.8, 6.1) if wide else (6.5, 8.5))
    if wide:
        fig.subplots_adjust(left=0.095, right=0.575, top=0.855, bottom=0.085)
        ax.set_anchor('W')
    else:
        fig.subplots_adjust(left=0.1, right=0.97, top=0.895, bottom=0.07)

    def line(run, **kw):
        xs = [float(r['x']) for r in run['trajectory']]
        ys = [float(r['y']) for r in run['trajectory']]
        ax.plot(xs, ys, solid_joinstyle='round', solid_capstyle='round', **kw)

    for run in others:
        line(run, color=MUTED, lw=1.0, alpha=0.75, zorder=2)
    false_run = next(r for r in complete if r['name'] == FALSE_COMPLETION_RUN)
    best_run = next(r for r in complete if r['name'] == BEST_RUN)
    line(best_run, color=BLUE, lw=2.0, zorder=3)
    line(false_run, color=ORANGE, lw=1.3, zorder=4)
    updates = [r for r in best_run['trajectory'] if r['source'] == 'update']
    ax.scatter([float(r['x']) for r in updates], [float(r['y']) for r in updates], s=34,
               color=BLUE, edgecolor=SURFACE, linewidth=0.9, zorder=5)

    goals = {int(w['wp']): w for w in best_run['waypoints']}
    labels = {1: ('WP1', (10, -14)), 2: ('WP2', (9, 7)), 3: ('WP3 and start', (9, -26))}
    for wp, w in goals.items():
        gx, gy, gth = float(w['goal_x']), float(w['goal_y']), float(w['goal_theta_deg'])
        ax.scatter([gx], [gy], s=95, facecolor='none', edgecolor=INK, linewidth=1.2, zorder=6)
        arrow(ax, gx, gy, gth, 0.2, INK)
        ax.annotate(labels[wp][0], (gx, gy), textcoords='offset points', xytext=labels[wp][1],
                    color=INK, fontweight='bold')
    for lm in best_run['config']['map']['landmarks']:
        ax.scatter([lm['x']], [lm['y']], s=70, marker='s', color=INK2, zorder=6)
        arrow(ax, lm['x'], lm['y'], lm['yaw_deg'], 0.2, INK2)
        offset = (9, 7) if lm['id'] != 2 else (11, -13)
        ax.annotate(f'tag {lm["id"]}', (lm['x'], lm['y']), textcoords='offset points',
                    xytext=offset, color=INK2)
    ax.set_aspect('equal')
    ax.set_xlim(-0.55, 1.8)
    ax.set_ylim(-0.75, 2.85)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    handles = [
        mlines.Line2D([], [], color=BLUE, lw=2.0, label=f'{BEST_RUN}\nestimated residuals '
                      '3.1, 3.3, 5.1 cm'),
        dot_handle(BLUE, 'accepted tag updates in that run'),
        mlines.Line2D([], [], color=MUTED, lw=1.0, label=f'{len(others)} other runs'),
        mlines.Line2D([], [], color=ORANGE, lw=1.5, label=f'{FALSE_COMPLETION_RUN}\nWP2 flagged '
                      'reached 198.5 cm\nfrom the goal (logic fixed after this run)'),
    ]
    anchor = (1.05, 1.0) if wide else (0.0, 1.0)
    ax.legend(handles=handles, loc='upper left', bbox_to_anchor=anchor, labelcolor=INK2,
              handlelength=1.8, labelspacing=0.9, borderaxespad=0.2)
    headline(fig, f'Estimated paths of the {len(complete)} runs flagged complete',
             'All three waypoints have reached = 1 in waypoints.csv. EKF estimates, world frame;\n'
             'jumps are tag updates. No external ground truth was recorded.',
             x=0.04, y=0.985)
    return save(fig, out_dir, 'trajectories_wide.png' if wide else 'trajectories.png')


def figure_residuals(runs, out_dir):
    complete = [r for r in runs if is_complete(r)]
    fig, axes = plt.subplots(1, 2, figsize=(9.4, 5.2))
    fig.subplots_adjust(left=0.065, right=0.985, top=0.82, bottom=0.235, wspace=0.17)
    pos_ax, head_ax = axes
    pos_top, head_low, head_top = 14.0, -40.0, 25.0
    n_updated = n_total = n_zero = 0

    def color_of(name):
        return {BEST_RUN: BLUE, FALSE_COMPLETION_RUN: ORANGE}.get(name, MUTED)

    def mark(ax, x, y, color, updated):
        ax.scatter([x], [y], s=46, zorder=4, clip_on=False,
                   facecolor=color if updated else SURFACE,
                   edgecolor=SURFACE if updated else color, linewidth=0.9 if updated else 1.4)

    for i, run in enumerate(complete):
        offset = (i - (len(complete) - 1) / 2) * 0.095
        color = color_of(run['name'])
        for row in waypoint_rows(run):
            x = row['wp'] - 1 + offset
            n_total += 1
            n_updated += row['updated']
            n_zero += round(row['pos_cm'], 1) == 0.0
            if row['pos_cm'] <= pos_top:
                pos_ax.vlines(x, max(0.0, row['pos_cm'] - row['sigma_pos_cm']),
                              row['pos_cm'] + row['sigma_pos_cm'], color=color, lw=0.9,
                              alpha=0.65, zorder=2)
                mark(pos_ax, x, row['pos_cm'], color, row['updated'])
            else:
                pos_ax.scatter([x], [pos_top - 0.35], marker='^', s=46, color=color, zorder=4)
                pos_ax.annotate(f'{row["pos_cm"]:.1f} cm, off scale', (x, pos_top - 0.35),
                                textcoords='offset points', xytext=(8, -3), color=INK2)
            if head_low <= row['head_deg'] <= head_top:
                head_ax.vlines(x, row['head_deg'] - row['sigma_head_deg'],
                               row['head_deg'] + row['sigma_head_deg'], color=color, lw=0.9,
                               alpha=0.65, zorder=2)
                mark(head_ax, x, row['head_deg'], color, row['updated'])
            else:
                head_ax.scatter([x], [head_low + 1.2], marker='v', s=46, color=color, zorder=4)
                head_ax.annotate(f'−{abs(row["head_deg"]):.1f}°, off scale',
                                 (x, head_low + 1.2),
                                 textcoords='offset points', xytext=(-8, 8), ha='center',
                                 color=INK2)

    pos_ax.axhspan(0, 3, color=GRID, alpha=0.6, zorder=1, lw=0)
    pos_ax.text(2.87, 3.12, 'target 3 cm', ha='right', va='bottom', color=INK2, fontsize=8.6)
    head_ax.axhspan(-3, 3, color=GRID, alpha=0.6, zorder=1, lw=0)
    head_ax.text(2.87, 3.5, 'target ±3°', ha='right', va='bottom', color=INK2, fontsize=8.6)
    pos_ax.set_ylim(0, pos_top)
    head_ax.set_ylim(head_low, head_top)
    pos_ax.set_ylabel('distance from estimate to goal (cm)')
    head_ax.set_ylabel('heading, estimate minus goal (°)')
    for ax in axes:
        ax.set_xlim(-0.5, 2.9)
        ax.set_xticks([0, 1, 2])
        ax.set_xticklabels([f'WP{i}\nn = {len(complete)}' for i in (1, 2, 3)])
        ax.grid(axis='x', visible=False)

    handles = [
        dot_handle(INK2, 'accepted tag update after the last primitive '
                   f'({n_updated} of {n_total})'),
        dot_handle(INK2, f'no such update ({n_total - n_updated} of {n_total}; '
                   f'{n_zero} are exactly 0.0 cm: the predicted endpoint)', hollow=True),
        mlines.Line2D([], [], color=INK2, lw=0.9, label='filter ±1σ'),
        dot_handle(BLUE, BEST_RUN),
        dot_handle(ORANGE, f'{FALSE_COMPLETION_RUN} (WP2 flagged reached in error)'),
        dot_handle(MUTED, f'{len(complete) - 2} other runs'),
    ]
    fig.legend(handles=handles, loc='lower left', bbox_to_anchor=(0.05, 0.005), ncol=2,
               labelcolor=INK2, columnspacing=2.0, handletextpad=0.5, labelspacing=0.7)
    headline(fig, 'Waypoint residuals of the filter estimate',
             'Runs with all three waypoints flagged reached. These are estimates against the '
             'goal, not measured errors.', x=0.015, y=0.975)
    return save(fig, out_dir, 'waypoint_residuals.png')


def figure_observations(runs, out_dir):
    rows = observation_rows(runs)
    with_geometry = [r for r in rows if r['dist'] is not None]
    fig, ax = plt.subplots(figsize=(8.6, 5.4))
    fig.subplots_adjust(left=0.075, right=0.975, top=0.84, bottom=0.1)
    nav = runs[-1]['config']['nav']
    look, obs = nav['look'], nav['ekf']['obs']
    for x in (look['min_dist'], look['max_dist']):
        ax.axvline(x, color=AXIS, lw=1.0, zorder=1)
    for y in (obs['oblique_deg'], look['max_oblique_deg']):
        ax.axhline(y, color=AXIS, lw=1.0, zorder=1)
    ax.text(look['min_dist'] + 0.03, 58.2, f'{look["min_dist"]} m limit', fontsize=8.4, va='top')
    ax.text(look['max_dist'] + 0.03, 58.2, f'{look["max_dist"]:.0f} m limit', fontsize=8.4,
            va='top')
    ax.text(1.0, obs['oblique_deg'] + 0.6,
            f'above {obs["oblique_deg"]:.0f}°: heading noise × {obs["oblique_factor"]:.0f}',
            fontsize=8.4, va='bottom')
    ax.text(1.0, look['max_oblique_deg'] + 0.6, f'{look["max_oblique_deg"]:.0f}° limit',
            fontsize=8.4, va='bottom')

    def points(category):
        return [r for r in with_geometry if r['category'] == category]

    styles = [
        ('accepted', dict(marker='o', s=30, color=BLUE, alpha=0.6, edgecolor=SURFACE,
                          linewidth=0.5, zorder=3)),
        ('gate', dict(marker='X', s=52, color=ORANGE, edgecolor=SURFACE, linewidth=0.5, zorder=4)),
        ('out of range', dict(marker='^', s=56, color=AQUA, edgecolor=SURFACE, linewidth=0.5,
                              zorder=4)),
    ]
    for category, style in styles:
        p = points(category)
        ax.scatter([r['dist'] for r in p], [r['oblique'] for r in p], **style)
    for (dist, oblique), n in collections.Counter(
            (r['dist'], r['oblique']) for r in points('out of range')).items():
        ax.annotate(f'out of range\nn = {n}', (dist, oblique), textcoords='offset points',
                    xytext=(7, 0), va='center', fontsize=8.2, linespacing=1.15)

    gate_first = []
    for run in runs:
        gate_first += [float(o['dist']) for o in first_looks(run) if o['reason'] == 'gate']
    ax.annotate(f'{len(gate_first)} gate rejections at the first look from the start pose:\n'
                f'tag 0 measured {min(gate_first):.2f} to {max(gate_first):.2f} m, '
                '1.50 m in the map',
                xy=(statistics.mean(gate_first), 2.0), xytext=(1.25, 17.5), color=INK2,
                fontsize=8.8,
                arrowprops=dict(arrowstyle='-', color=INK2, lw=0.7, shrinkB=5))
    ax.set_xlim(0, 3.4)
    ax.set_ylim(0, 62)
    ax.set_xlabel('distance from robot centre to tag (m)')
    ax.set_ylabel('viewing obliquity of the tag (°)')
    gate = nav['ekf']['gate']
    counts = collections.Counter(r['category'] for r in rows)
    handles = [
        dot_handle(BLUE, f'accepted ({counts["accepted"]})'),
        dot_handle(ORANGE, f'rejected by chi-square gate, d² > {gate} ({counts["gate"]})',
                   marker='X', size=7.5),
        dot_handle(AQUA, f'rejected, out of range ({counts["out of range"]})', marker='^',
                   size=7.5),
    ]
    ax.legend(handles=handles, loc='upper right', bbox_to_anchor=(0.875, 0.93), labelcolor=INK2,
              facecolor=SURFACE, frameon=True, edgecolor='none', framealpha=1.0)
    headline(fig, f'Tag observations across all {len(runs)} runs, by outcome',
             f'{len(with_geometry)} of {len(rows)} logged rows have geometry. The other '
             f'{len(rows) - len(with_geometry)} are rejections with none logged: '
             f'{counts["too few frames"]} too few frames, {counts["unstable"]} unstable.',
             x=0.015, y=0.975)
    return save(fig, out_dir, 'observations.png')


def figure_calibration(runs, calib, nav_cfg, out_dir):
    fig, axes = plt.subplots(2, 2, figsize=(9.4, 7.2))
    fig.subplots_adjust(left=0.07, right=0.985, top=0.84, bottom=0.075, hspace=0.3, wspace=0.14)
    fine_max = nav_cfg['control']['fine_max_deg']
    requests = long_fine_spins(runs, calib)
    for col, direction in enumerate(('ccw', 'cw')):
        coarse_min = min_amount(calib, direction, COARSE_CMD)
        for row, cmd in enumerate((COARSE_CMD, FINE_CMD)):
            ax = axes[row][col]
            fit = calib['fit']['spin'][direction][cmd]
            x_max, y_max = (1.8, 235.0) if cmd == COARSE_CMD else (2.2, 45.0)
            ax.plot([0, x_max], [fit_at(fit, 0), fit_at(fit, x_max)], color=BLUE, lw=1.0,
                    ls=(0, (4, 3)), alpha=0.7, zorder=2)
            ax.plot([fit['t_min'], fit['t_max']],
                    [fit_at(fit, fit['t_min']), fit_at(fit, fit['t_max'])],
                    color=BLUE, lw=2.0, solid_capstyle='round', zorder=3)
            trials = [t for t in calib['spin'][direction]['trials'] if t['cmd'] == cmd]
            ax.scatter([t['T'] for t in trials], [t['angle'] for t in trials], s=34, color=BLUE,
                       edgecolor=SURFACE, linewidth=0.9, alpha=0.85, zorder=4)
            T_min, smallest = min_amount(calib, direction, cmd)
            ax.scatter([T_min], [smallest], s=60, marker='D', facecolor=SURFACE, edgecolor=INK,
                       linewidth=1.3, zorder=6)
            if cmd == COARSE_CMD:
                ax.annotate(f'smallest step {smallest:.2f}°', (T_min, smallest),
                            textcoords='offset points', xytext=(10, -4), color=INK, fontsize=8.8)
            else:
                ax.annotate(f'smallest step {smallest:.2f}°', (T_min, smallest),
                            xytext=(0.42, 1.6), va='center', color=INK, fontsize=8.8,
                            arrowprops=dict(arrowstyle='-', color=INK2, lw=0.7, shrinkB=5))
            if cmd == FINE_CMD:
                ax.axhspan(fine_max, coarse_min[1], color=ORANGE, alpha=0.12, lw=0, zorder=1)
                ax.axhline(fine_max, color=INK2, lw=0.8, zorder=2)
                ax.text(x_max - 0.03, fine_max - 0.7, f'fine_max {fine_max:.0f}°', ha='right',
                        va='top', fontsize=8.4)
                ax.text(x_max - 0.03, (fine_max + coarse_min[1]) / 2,
                        f'no executable level, {fine_max:.0f}° to {coarse_min[1]:.1f}°',
                        ha='right', va='center', fontsize=8.4)
                mine = [r for r in requests if r['dir'] == direction]
                ax.scatter([r['T'] for r in mine], [r['amount'] for r in mine], s=52, marker='s',
                           color=ORANGE, edgecolor=SURFACE, linewidth=0.8, zorder=5)
                if direction == 'ccw':
                    top = max(mine, key=lambda r: r['amount'])
                    ax.annotate(f'{top["amount"]:.1f}° requested, T = {top["T"]:.2f} s,\n'
                                'beyond the calibrated range', (top['T'], top['amount']),
                                textcoords='offset points', xytext=(-10, -2), ha='right',
                                va='center', fontsize=8.6, color=INK2)
            ax.set_xlim(0, x_max)
            ax.set_ylim(0, y_max)
            ax.set_title(f'{direction.upper()}, cmd {cmd}: {fit["rate"]:.1f} °/s, '
                         f'rms {fit["rms"]:.1f}°, n = {fit["n"]}', loc='left', fontsize=9.6,
                         color=INK, pad=6)
            if col == 0:
                ax.set_ylabel('turn angle (°)')
            if row == 1:
                ax.set_xlabel('command duration T (s)')
    handles = [
        dot_handle(BLUE, 'hand-measured trial'),
        mlines.Line2D([], [], color=BLUE, lw=2.0, label='linear fit, calibrated range'),
        mlines.Line2D([], [], color=BLUE, lw=1.0, ls=(0, (4, 3)), label='fit extrapolated'),
        dot_handle(INK, 'smallest executable step', hollow=True, marker='D', size=6),
        dot_handle(ORANGE, f'logged spin past the fine level\'s calibrated range '
                   f'(n = {len(requests)}, all before the gate)', marker='s'),
    ]
    fig.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.008, 0.945), ncol=3,
               labelcolor=INK2, columnspacing=1.5, handletextpad=0.5, handlelength=1.6,
               fontsize=8.6, labelspacing=0.45)
    headline(fig, 'Spin calibration: coarse and fine command levels',
             'Trials and fits from study_wood2_hw2_ext.yaml. Logged requests include time_scale, '
             'so they sit above the raw fit.', x=0.015, y=0.985)
    return save(fig, out_dir, 'spin_calibration.png')


# ---------- numbers.md ----------

def md_table(header, rows):
    lines = ['| ' + ' | '.join(header) + ' |', '|' + '---|' * len(header)]
    lines += ['| ' + ' | '.join(str(c) for c in row) + ' |' for row in rows]
    return lines + ['']


def write_numbers(runs, calib, before, out_dir):
    fine_end = fine_trials_end(calib)
    lines = ['# Numbers behind the README', '',
             'Generated by `scripts/make_figures.py` from `HW2/logs/` and '
             '`robot_control/config/`. Do not edit by hand.',
             'Run names use the robot clock. No run has a `truth.yaml`, so every waypoint value '
             'is the filter estimate minus the goal.', '']

    complete = [r for r in runs if is_complete(r)]
    lines += ['## Calibration phases', '']
    table = []
    for phase, text in PHASES.items():
        members = [r for r in runs if phase_of(r, fine_end) == phase]
        table.append((phase, text, len(members), sum(is_complete(r) for r in members)))
    table.append(('all', '', len(runs), len(complete)))
    lines += md_table(['phase', 'calibration', 'runs', 'all three waypoints flagged reached'],
                      table)
    lines += ['Phase B and C share `config.yaml`; the boundary is the last fine-level trial, '
              f'{fine_end.isoformat()} in `study_wood2_hw2_ext.yaml` (inference).', '']

    lines += ['## All runs', '']
    table = []
    for run in runs:
        reached = ''.join(w['reached'] for w in run['waypoints']) or '-'
        table.append((run['name'], phase_of(run, fine_end), reached, len(run['steps']),
                      run['trajectory'][-1]['t'], len(run['observations']),
                      '; '.join(w['note'] for w in run['waypoints'] if w['note'])))
    lines += md_table(['run', 'phase', 'reached per waypoint', 'primitives', 'end (s)',
                       'observations', 'notes'], table)
    no_level = [r['name'] for r in runs
                if any('no calibrated level' in w['note'] for w in r['waypoints'])]
    silent = [r for r in runs if not is_complete(r) and r['name'] not in no_level]
    halts = [r['name'] for r in silent if consistent_with_halt(r)]
    lines += [f'Incomplete runs: {len(runs) - len(complete)}. {len(no_level)} logged '
              f'`no calibrated level`: {", ".join(no_level)}. The other {len(silent)} have no '
              'terminal record, because console output was not saved.',
              f'Of those, {len(halts)} end the way `NavigationHalt` would leave the log '
              f'(inference): {", ".join(halts)}. The reason for the rest is unknown.', '']

    lines += ['## Complete runs, per waypoint', '']
    table, zero, updated, total = [], 0, 0, 0
    per_wp = collections.Counter()
    for run in complete:
        for r in waypoint_rows(run):
            total += 1
            zero += round(r['pos_cm'], 1) == 0.0
            updated += r['updated']
            per_wp[r['wp']] += r['updated']
            table.append((run['name'], r['wp'], f'{r["pos_cm"]:.1f}', f'{r["head_deg"]:+.1f}',
                          f'{r["sigma_pos_cm"]:.1f}', f'{r["sigma_head_deg"]:.1f}',
                          'yes' if r['updated'] else 'no', r['note']))
    lines += md_table(['run', 'wp', 'position (cm)', 'heading (°)', 'σ position (cm)',
                       'σ heading (°)', 'tag update after last primitive', 'note'], table)
    lines += [f'{total} rows, {updated} with a tag update after the last primitive '
              f'(WP1 {per_wp[1]}, WP2 {per_wp[2]}, WP3 {per_wp[3]}), {zero} exactly 0.0 cm.',
              'σ position is the root sum of squares of the logged x and y sigmas.', '']

    lines += ['## Motion totals, complete runs', '',
              '`commanded drive` and `commanded spin` sum `steps.csv` amounts. `estimated path` '
              'sums `trajectory.csv` segments, including jumps from EKF updates. The geometric '
              'path between the waypoints is 5.236 m.', '']
    table = []
    for run in complete:
        m = motion_totals(run)
        table.append((run['name'], f'{m["drive"]:.3f}', f'{m["spin"]:.1f}', f'{m["path"]:.3f}',
                      m['primitives'], f'{m["time"]:.1f}', f'{m["moving"]:.1f}'))
    lines += md_table(['run', 'commanded drive (m)', 'commanded spin (°)', 'estimated path (m)',
                       'primitives', 'run time (s)', 'moving (s)'], table)

    rows = observation_rows(runs)
    counts = collections.Counter(r['category'] for r in rows)
    gate_ids = collections.Counter(r['id'] for r in rows if r['category'] == 'gate')
    with_geometry = [r for r in rows if r['dist'] is not None]
    lines += ['## Observations, all runs', '']
    outcomes = ('accepted', 'gate', 'too few frames', 'out of range', 'unstable')
    lines += md_table(['outcome', 'rows'],
                      [(k, counts[k]) for k in outcomes] + [('total', len(rows))])
    lines += [f'Gate rejections by tag: {dict(sorted(gate_ids.items()))}. '
              f'Largest logged obliquity {max(r["oblique"] for r in with_geometry):.1f}°. '
              'Distance range '
              f'{min(r["dist"] for r in with_geometry):.3f} to '
              f'{max(r["dist"] for r in with_geometry):.3f} m.', '']

    lines += ['## First look from the start pose', '',
              'Rows with geometry before the first primitive. Tag 0 is at 1.500 m in the map.', '']
    by_day = collections.defaultdict(list)
    for run in runs:
        by_day[run['stamp'].date().isoformat()] += first_looks(run)
    table = []
    for day, obs in sorted(by_day.items()):
        xs = [float(o['x']) for o in obs]
        gate = sum(o['reason'] == 'gate' for o in obs)
        table.append((day, len(obs), f'{statistics.mean(xs):.3f}', f'{min(xs):.3f}',
                      f'{max(xs):.3f}', gate))
    lines += md_table(['run date', 'looks', 'mean tag x (m)', 'min (m)', 'max (m)',
                       'rejected by gate'], table)

    progress = [p for p in map(first_leg_progress, runs) if p]
    ratios = [p['drop'] / p['cmd'] for p in progress]
    lines += ['## Tag 0 progress over the first drive', '',
              'Fall in the measured tag 0 distance between the looks before and after the first '
              'drive, against the commanded length of that drive.', '']
    lines += md_table(['runs', 'mean commanded (m)', 'mean measured fall (m)', 'mean ratio',
                       'min ratio', 'max ratio'],
                      [(len(progress), f'{statistics.mean(p["cmd"] for p in progress):.3f}',
                        f'{statistics.mean(p["drop"] for p in progress):.3f}',
                        f'{statistics.mean(ratios):.3f}', f'{min(ratios):.3f}',
                        f'{max(ratios):.3f}')])

    level_counts = collections.Counter((s['kind'], s['cmd']) for r in runs for s in r['steps'])
    lines += ['## Command levels used, all runs', '']
    lines += md_table(['kind', 'cmd', 'primitives'],
                      [(k, c, n) for (k, c), n in sorted(level_counts.items())])

    lines += ['## Smallest executable steps', '']
    table = []
    for direction in ('ccw', 'cw'):
        table.append((f'spin {direction}', f'{min_amount(before, direction, COARSE_CMD)[1]:.2f}°',
                      f'{min_amount(calib, direction, COARSE_CMD)[1]:.2f}°',
                      f'{min_amount(calib, direction, FINE_CMD)[1]:.2f}°'))
    table.append(('drive', f'{drive_min_amount(before, 0.3):.3f} m',
                  f'{drive_min_amount(calib, 0.3):.3f} m', 'not calibrated'))
    lines += md_table(['primitive', 'before fine calibration, coarse level (`study_wood2`)',
                       'now, coarse level (`study_wood2_hw2_ext`)', 'now, fine level'], table)

    lines += ['## Logged fine-level spins past the calibrated range', '']
    lines += md_table(['run', 'wp', 'dir', 'requested (°)', 'T (s)'],
                      [(r['run'], r['wp'], r['dir'], f'{r["amount"]:.2f}', f'{r["T"]:.3f}')
                       for r in long_fine_spins(runs, calib)])

    lines += ['## Hand-measured check of open-loop primitives', '',
              'From the `verify` section of `study_wood2_hw2_ext.yaml`.', '']
    table = []
    for v in calib['verify']:
        measured = v['x'] if v['kind'] == 'drive' else v['angle']
        unit = 'm' if v['kind'] == 'drive' else '°'
        table.append((v['kind'], v['dir'], f'{v["target"]} {unit}', f'{measured} {unit}'))
    lines += md_table(['kind', 'dir', 'target', 'measured'], table)

    path = os.path.join(out_dir, 'numbers.md')
    with open(path, 'w') as f:
        f.write('\n'.join(lines).rstrip() + '\n')
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--logs', default=os.path.join(ROOT, 'HW2', 'logs'))
    ap.add_argument('--config', default=os.path.join(ROOT, 'robot_control', 'config'))
    ap.add_argument('--out', default=os.path.join(ROOT, 'docs', 'figures'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    runs = load_runs(a.logs)
    calib = read_yaml(os.path.join(a.config, 'open_loop', 'study_wood2_hw2_ext.yaml'))
    before = read_yaml(os.path.join(a.config, 'open_loop', 'study_wood2.yaml'))
    nav_cfg = read_yaml(os.path.join(a.config, 'hw2', 'nav.yaml'))
    set_style()
    written = [
        figure_trajectories(runs, a.out),
        figure_trajectories(runs, a.out, wide=True),
        figure_residuals(runs, a.out),
        figure_observations(runs, a.out),
        figure_calibration(runs, calib, nav_cfg, a.out),
        write_numbers(runs, calib, before, a.out),
    ]
    for path in written:
        print(os.path.relpath(path, ROOT))


if __name__ == '__main__':
    main()
