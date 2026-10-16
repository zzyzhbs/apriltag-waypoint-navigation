"""Waypoints -> open-loop primitive list (no ROS).

python -m robot_control.ol_planner <waypoints.txt> <calib.yaml> [--drive-cmd 0.3] [--spin-cmd 0.35]
"""
import argparse
import math
from dataclasses import dataclass, field

from robot_control.ol_model import Calibration, ModelError, wrap_deg

TIE_DEG = 1.0   # |turn| within this of 180 -> ccw


@dataclass
class Step:
    kind: str               # 'drive' | 'spin'
    dir: str                # 'fwd' | 'ccw' | 'cw'
    amount: float           # m or deg (positive)
    cmd: float = 0.0
    T: float | None = None
    warn: str | None = None

    def describe(self):
        unit = 'm' if self.kind == 'drive' else 'deg'
        T = f'{self.T:.2f} s' if self.T is not None else '   ?  '
        s = f'{self.kind:5s} {self.dir:3s} {self.amount:7.3f} {unit:3s} cmd {self.cmd:.2f}  T {T}'
        return s + (f'  !! {self.warn}' if self.warn else '')


@dataclass
class Leg:
    index: int                      # waypoint index (1 = first after start)
    goal: tuple                     # (x, y, theta rad)
    steps: list = field(default_factory=list)
    end: tuple = (0.0, 0.0, 0.0)    # planned pose after the leg


def load_waypoints(path):
    wps = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                wps.append(tuple(float(v) for v in line.split(',')))
    if len(wps) < 2:
        raise ValueError(f'{path}: need a start pose and at least one waypoint')
    return wps


def plan_motions(wps, pos_tol=0.02, min_turn_deg=2.0):
    """Turn-drive-turn per waypoint, from the planned (assumed exact) pose."""
    x, y, th = wps[0]
    th = math.degrees(th)
    legs = []

    def turn(steps, target_deg):
        nonlocal th
        d = wrap_deg(target_deg - th)
        if abs(d) < min_turn_deg:
            return
        if abs(abs(d) - 180.0) < TIE_DEG and d < 0:
            d += 360.0
        steps.append(Step('spin', 'ccw' if d > 0 else 'cw', abs(d)))
        th = wrap_deg(th + d)

    for i, (gx, gy, gth) in enumerate(wps[1:], 1):
        leg = Leg(i, (gx, gy, gth))
        D = math.hypot(gx - x, gy - y)
        if D > pos_tol:
            turn(leg.steps, math.degrees(math.atan2(gy - y, gx - x)))
            leg.steps.append(Step('drive', 'fwd', D))
            x += D * math.cos(math.radians(th))
            y += D * math.sin(math.radians(th))
        turn(leg.steps, math.degrees(gth))
        leg.end = (x, y, math.radians(th))
        legs.append(leg)
    return legs


def attach_timing(legs, calib, drive_cmd, spin_cmd):
    """Fill cmd / T for each step. Model errors are kept in step.warn with T = None."""
    for leg in legs:
        for s in leg.steps:
            s.cmd = drive_cmd if s.kind == 'drive' else spin_cmd
            try:
                if s.kind == 'drive':
                    s.T, s.warn = calib.drive_time(s.amount, s.cmd)
                else:
                    # This study_wood2 correction is only for wp3's CW 90 deg
                    # turn. Later CW turns use their unadjusted calibration.
                    wp3_cw_90 = (calib.data.get('floor') == 'study_wood2'
                                 and leg.index == 3 and s.dir == 'cw'
                                 and abs(s.amount - 90.0) < 0.5)
                    s.T, s.warn = calib.spin_time(
                        s.dir, s.amount, s.cmd, apply_time_scale=wp3_cw_90)
            except ModelError as e:
                s.T, s.warn = None, str(e)
    return legs


def plan(wps, calib, drive_cmd=0.3, spin_cmd=0.35, pos_tol=0.02, min_turn_deg=2.0):
    return attach_timing(plan_motions(wps, pos_tol, min_turn_deg), calib, drive_cmd, spin_cmd)


def ready(legs):
    return all(s.T is not None for leg in legs for s in leg.steps)


def format_plan(legs, trim=0.0):
    lines = [f'drive trim {trim:+.4f}']
    total = 0.0
    for leg in legs:
        gx, gy, gth = leg.goal
        ex, ey, eth = leg.end
        lines.append(f'wp{leg.index} goal ({gx:g}, {gy:g}, {math.degrees(gth):.1f} deg)  '
                     f'planned end ({ex:.3f}, {ey:.3f}, {math.degrees(eth):.1f} deg)')
        for s in leg.steps:
            lines.append('    ' + s.describe())
            total += s.T or 0.0
    lines.append(f'total motion time {total:.1f} s')
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('waypoints')
    ap.add_argument('calib')
    ap.add_argument('--drive-cmd', type=float, default=0.3)
    ap.add_argument('--spin-cmd', type=float, default=0.35)
    a = ap.parse_args()
    calib = Calibration.load(a.calib)
    legs = plan(load_waypoints(a.waypoints), calib, a.drive_cmd, a.spin_cmd)
    print(format_plan(legs, calib.trim))
    if not ready(legs):
        print('NOT READY: some steps have no calibrated T')


if __name__ == '__main__':
    main()
