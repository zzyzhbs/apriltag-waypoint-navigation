"""Waypoint navigator: turn -> drive -> turn state machine (no ROS dependency)."""
from __future__ import annotations

import math
from dataclasses import dataclass

from robot_control.kinematics import (
    FloorProfile, from_command, rotate_command, to_command, wrap_angle,
)

STOP = (0.0, 0.0)


@dataclass
class NavParams:
    v_max: float = 0.25          # m/s
    v_min: float = 0.05
    accel: float = 0.3           # m/s^2
    k_heading: float = 2.0
    w_max: float = 0.6           # rad/s, heading correction while driving
    pos_tol: float = 0.05        # m
    ang_tol_deg: float = 4.0
    drive_coast: float = 0.02    # m, stop this early
    settle_s: float = 1.0
    start_still_s: float = 10.0  # long still -> better initial gyro bias
    pulse_min_s: float = 0.08
    pulse_max_s: float = 0.4
    max_corrections: int = 3
    default_rot_rate: float = 1.5  # rad/s, used while rot_rate_* is uncalibrated
    lookahead_min: float = 0.15  # m
    # heading while driving: 'gyro', or 'model' (commands + trim; gyro random walk
    # is ~+-10 deg over a 5 s drive, so the gyro is only trusted for in-place turns)
    drive_heading: str = 'model'


def load_waypoints(path: str) -> list[tuple[float, float, float]]:
    wps = []
    with open(path) as f:
        for line in f:
            line = line.split('#')[0].strip()
            if line:
                x, y, th = (float(v) for v in line.split(','))
                wps.append((x, y, th))
    return wps


class Navigator:
    """Call step(t, theta) at a fixed rate; it returns the [L, R] command to send.

    theta comes from GyroHeading; x, y are dead-reckoned from the commands sent.
    """

    def __init__(self, waypoints, profile: FloorProfile, params: NavParams | None = None):
        self.wps = list(waypoints)
        self.p = profile
        self.np = params or NavParams()
        self.x, self.y, self.theta = self.wps[0]
        self.wp_index = 0
        self.phase = 'start'
        self.done = False
        self.arrivals = []         # (index, x, y, theta) estimated at each waypoint
        self.events = []           # human-readable log lines
        self._t = None
        self._t0 = None
        self._offset = 0.0         # self.theta - gyro theta
        self._cmd = STOP
        self._stop_t = -math.inf
        self._wait_until = 0.0
        # learned online from each turn (start from profile / defaults)
        self._coast = {1: profile.rot_coast_ccw, -1: profile.rot_coast_cw}
        self._pulse_rate = {1: profile.rot_rate_ccw or self.np.default_rot_rate,
                            -1: profile.rot_rate_cw or self.np.default_rot_rate}

    # ---------- public ----------

    def stationary(self, t: float) -> bool:
        return self._cmd == STOP and t - self._stop_t >= self.np.settle_s

    def step(self, t: float, gyro_theta: float) -> tuple[float, float]:
        """gyro_theta: continuous heading from GyroHeading."""
        if self._t is None:
            self._t0, dt = t, 0.0
        else:
            dt = t - self._t
        self._t = t
        v, w = from_command(*self._cmd, self.p)
        if self.phase == 'drive' and self.np.drive_heading == 'model':
            theta = self.theta + w * dt
            self._offset = theta - gyro_theta
        else:
            theta = gyro_theta + self._offset
        th_mid = (self.theta + theta) / 2.0
        self.x += v * math.cos(th_mid) * dt
        self.y += v * math.sin(th_mid) * dt
        self.theta = theta

        cmd = getattr(self, f'_{self.phase}')(t)
        if cmd == STOP and self._cmd != STOP:
            self._stop_t = t
        self._cmd = cmd
        return cmd

    # ---------- phases ----------

    def _start(self, t):
        if t - self._t0 >= self.np.start_still_s:
            self._arrive(t)
        return STOP

    def _done(self, t):
        return STOP

    def _next_waypoint(self, t):
        self.wp_index += 1
        if self.wp_index >= len(self.wps):
            self.phase, self.done = 'done', True
            self._log(t, 'all waypoints done')
            return
        gx, gy, gth = self.wps[self.wp_index]
        self._log(t, f'-> waypoint {self.wp_index} ({gx}, {gy}, {gth})')
        if math.hypot(gx - self.x, gy - self.y) > self.np.pos_tol:
            self._begin_turn(t, math.atan2(gy - self.y, gx - self.x),
                             lambda t: self._begin_drive(t, gx, gy))
        else:
            self._begin_turn(t, gth, self._arrive)

    def _arrive(self, t):
        self.arrivals.append((self.wp_index, self.x, self.y, self.theta))
        self._log(t, f'at waypoint {self.wp_index}: est ({self.x:.3f}, {self.y:.3f}, '
                     f'{math.degrees(wrap_angle(self.theta)):.1f} deg)')
        self._next_waypoint(t)

    # ---------- in-place turn ----------

    def _begin_turn(self, t, target, then):
        self.phase = 'turn'
        self._target, self._then = target, then
        self._sub, self._spun, self._corrections = 'check', False, 0

    def _turn(self, t):
        err = wrap_angle(self._target - self.theta)
        d = 1 if err > 0 else -1
        if self._sub == 'check':
            self._learn()
            if abs(err) <= math.radians(self.np.ang_tol_deg):
                return self._finish_turn(t)
            self._move_th0 = self.theta
            if not self._spun:
                self._spun, self._sub, self._dir = True, 'spin', d
            elif self._corrections < self.np.max_corrections:
                self._corrections += 1
                dur = abs(err) / self._pulse_rate[d]
                self._pulse_dur = min(max(dur, self.np.pulse_min_s), self.np.pulse_max_s)
                self._sub, self._dir, self._pulse_end = 'pulse', d, t + self._pulse_dur
            else:
                self._log(t, f'turn gave up, error {math.degrees(err):.1f} deg')
                return self._finish_turn(t)
        remaining = err * self._dir
        if self._sub == 'spin' and remaining <= self._coast[self._dir]:
            self._stop_th = self.theta
            return self._settle(t)
        if self._sub == 'pulse' and (t >= self._pulse_end or remaining <= 0):
            return self._settle(t)
        if self._sub == 'settle':
            if t >= self._wait_until:
                self._sub = 'check'
            return STOP
        return rotate_command(self._dir, self.p)

    def _finish_turn(self, t):
        self._then(t)
        return self._drive(t) if self.phase == 'drive' else STOP

    def _settle(self, t):
        self._last_move, self._sub = self._sub, 'settle'
        self._wait_until = t + self.np.settle_s + 0.1
        return STOP

    def _learn(self):
        """Update coast / pulse rate from the move that just settled."""
        move, self._last_move = getattr(self, '_last_move', None), None
        d = getattr(self, '_dir', 1)
        if move == 'spin' and self._stop_th != self._move_th0:
            coast = (self.theta - self._stop_th) * d
            self._coast[d] = max(0.0, (self._coast[d] + coast) / 2.0)
        elif move == 'pulse':
            rate = (self.theta - self._move_th0) * d / self._pulse_dur
            self._pulse_rate[d] = min(max((self._pulse_rate[d] + rate) / 2.0, 0.3), 5.0)

    # ---------- straight drive ----------

    def _begin_drive(self, t, gx, gy):
        self.phase = 'drive'
        self._goal = (gx, gy)
        dist = math.hypot(gx - self.x, gy - self.y)
        self._u = ((gx - self.x) / dist, (gy - self.y) / dist)
        self._drive_t0 = t

    def _drive(self, t):
        n = self.np
        dx, dy = self._goal[0] - self.x, self._goal[1] - self.y
        along = dx * self._u[0] + dy * self._u[1]
        if along <= n.drive_coast:
            self.phase, self._wait_until = 'drive_settle', t + n.settle_s + 0.1
            return STOP
        if math.hypot(dx, dy) > n.lookahead_min:
            psi = math.atan2(dy, dx)
        else:
            psi = math.atan2(self._u[1], self._u[0])
        w = max(-n.w_max, min(n.w_max, n.k_heading * wrap_angle(psi - self.theta)))
        v = min(n.v_max, n.v_min + n.accel * (t - self._drive_t0),
                max(n.v_min, math.sqrt(2.0 * n.accel * along)))
        return to_command(v, w, self.p)

    def _drive_settle(self, t):
        if t >= self._wait_until:
            self._begin_turn(t, self.wps[self.wp_index][2], self._arrive)
        return STOP

    def _log(self, t, msg):
        self.events.append(f'[{t - self._t0:7.2f}s] {msg}')
