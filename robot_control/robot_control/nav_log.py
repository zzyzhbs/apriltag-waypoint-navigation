"""CSV logs of one HW2 run (no ROS).

run_<time>/trajectory.csv    estimated pose every dt s (report: 2D trajectory)
run_<time>/steps.csv         every primitive (smoothness, total distance)
run_<time>/observations.csv  every tag in every look (noise handling)
run_<time>/waypoints.csv     estimate at each waypoint (waypoint errors)
run_<time>/config.yaml       parameters and map used
"""
import csv
import math
import os
import time

import yaml

from robot_control.ekf import sigmas
from robot_control.ol_primitive import wheel_cmds
from robot_control.se2 import Pose2, wrap_angle
from robot_control.stop_look import NavLog

TRAJ_DT = 0.5


def interpolate(a, b, f):
    """Pose between a and b; f in [0, 1]."""
    return Pose2(a.x + f * (b.x - a.x), a.y + f * (b.y - a.y),
                 wrap_angle(a.theta + f * wrap_angle(b.theta - a.theta)))


class TrajectorySampler:
    """Emits the pose on a fixed time grid: held while still, interpolated while moving."""

    def __init__(self, write_row, dt=TRAJ_DT):
        self.write_row = write_row      # write_row(t, pose, sig, source)
        self.dt = dt
        self.next_t = 0.0
        self.pose = None
        self.sig = None

    def hold(self, t_end):
        while self.pose is not None and self.next_t <= t_end:
            self.write_row(self.next_t, self.pose, self.sig, 'hold')
            self.next_t += self.dt

    def set(self, t, pose, sig, source):
        """New estimate at time t (start, or after a tag update)."""
        self.hold(t)
        self.pose, self.sig = pose, sig
        self.write_row(t, pose, sig, source)

    def move(self, t0, t1, pose_after, sig_after):
        """Motion from the current pose to pose_after during [t0, t1]."""
        self.hold(t0)
        start = self.pose
        while self.next_t <= t1:
            f = (self.next_t - t0) / (t1 - t0) if t1 > t0 else 1.0
            self.write_row(self.next_t, interpolate(start, pose_after, f), sig_after, 'predict')
            self.next_t += self.dt
        self.pose, self.sig = pose_after, sig_after


def pose_cells(p):
    return [f'{p.x:.4f}', f'{p.y:.4f}', f'{math.degrees(p.theta):.2f}']


def sigma_cells(sig):
    return [f'{sig[0]:.4f}', f'{sig[1]:.4f}', f'{math.degrees(sig[2]):.2f}']


class CsvNavLog(NavLog):
    def __init__(self, run_dir, start_pose, start_sig, clock=time.monotonic):
        os.makedirs(run_dir, exist_ok=True)
        self.dir = run_dir
        self.clock = clock
        self.t_ref = clock()
        self.files = []
        self.traj = self.open('trajectory.csv', ['t', 'x', 'y', 'theta_deg', 'sx', 'sy',
                                                 'stheta_deg', 'source'])
        self.steps = self.open('steps.csv', ['wp', 'kind', 'dir', 'amount', 'cmd', 'L', 'R',
                                             'T', 't_start', 't_stop', 'x0', 'y0', 'theta0_deg',
                                             'x1', 'y1', 'theta1_deg'])
        self.obs = self.open('observations.csv', ['t', 'wp', 'id', 'x', 'y', 'psi_deg', 'dist',
                                                  'oblique_deg', 'count', 'accepted', 'd2',
                                                  'reason'])
        self.wps = self.open('waypoints.csv', ['t', 'wp', 'goal_x', 'goal_y', 'goal_theta_deg',
                                               'reached', 'tag_based', 'x', 'y', 'theta_deg',
                                               'sx', 'sy', 'stheta_deg', 'iters', 'tags',
                                               'note'])
        self.sampler = TrajectorySampler(self.write_traj)
        self.sampler.set(0.0, start_pose, start_sig, 'start')

    def open(self, name, header):
        f = open(os.path.join(self.dir, name), 'w', newline='')
        self.files.append(f)
        w = csv.writer(f)
        w.writerow(header)
        return w

    def now(self):
        return self.clock() - self.t_ref

    def flush(self):
        for f in self.files:
            f.flush()

    def write_traj(self, t, pose, sig, source):
        self.traj.writerow([f'{t:.2f}', *pose_cells(pose), *sigma_cells(sig), source])

    def write_config(self, data):
        with open(os.path.join(self.dir, 'config.yaml'), 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)

    # ---------- NavLog hooks ----------

    def on_look(self, goal_index, records, pose, P):
        t = self.now()
        for r in records:
            o = r.obs
            cells = (['', '', '', '', '', '', ''] if o is None else
                     [o.id, f'{o.x:.4f}', f'{o.y:.4f}', f'{math.degrees(o.psi):.2f}',
                      f'{o.dist:.3f}', f'{math.degrees(o.oblique):.1f}', o.count])
            self.obs.writerow([f'{t:.2f}', goal_index, *cells, int(r.accepted),
                               f'{r.d2:.2f}', r.reason])
        if any(r.accepted for r in records):
            self.sampler.set(t, pose, sigmas(P), 'update')
        self.flush()

    def on_step(self, goal_index, step, pose_before, pose_after, P, timing=None):
        L, R = wheel_cmds(step.kind, step.dir, step.cmd)
        t1 = self.now()
        t0, t1 = (timing[0] - self.t_ref, timing[1] - self.t_ref) if timing else (t1 - step.T, t1)
        self.steps.writerow([goal_index, step.kind, step.dir, f'{step.amount:.4f}', step.cmd,
                             f'{L:.3f}', f'{R:.3f}', f'{step.T:.3f}', f'{t0:.2f}', f'{t1:.2f}',
                             *pose_cells(pose_before), *pose_cells(pose_after)])
        self.sampler.move(t0, t1, pose_after, sigmas(P))
        self.flush()

    def on_waypoint(self, result):
        g = result.goal
        self.wps.writerow([f'{self.now():.2f}', result.index, *pose_cells(g),
                           int(result.reached), int(result.tag_based), *pose_cells(result.pose),
                           *sigma_cells(result.sigmas), result.iters,
                           ' '.join(str(i) for i in result.tags_used), result.note])
        self.flush()

    def close(self):
        self.sampler.hold(self.now())
        for f in self.files:
            f.close()


class TeeLog(NavLog):
    """Forwards every hook to several logs."""

    def __init__(self, *logs):
        self.logs = logs

    def on_look(self, *args):
        for log in self.logs:
            log.on_look(*args)

    def on_step(self, *args):
        for log in self.logs:
            log.on_step(*args)

    def on_waypoint(self, *args):
        for log in self.logs:
            log.on_waypoint(*args)
