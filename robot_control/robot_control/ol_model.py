"""Open-loop calibration model (no ROS).

Per command level c, a primitive run for T seconds moves
    drive: d(T) = v * T + d0      (m)
    spin:  a(T) = w * T + a0      (deg, per direction)
Fits are always recomputed from the raw trials in config/open_loop/<floor>.yaml.
"""
import math
import os
from dataclasses import asdict, dataclass
from datetime import date, datetime

import yaml

DIRS = ('ccw', 'cw')
MIN_T = 0.2      # s, shorter runs are unreliable
EXTRAP = 0.2     # allowed extrapolation beyond calibrated T range (fraction)
DEFAULT_CALIB_DIR = '~/ros2_ws/src/CSE276A/robot_control/config/open_loop'


class ModelError(ValueError):
    pass


@dataclass
class LineFit:
    rate: float         # m/s or deg/s
    offset: float       # m or deg
    rms: float          # residual rms
    n: int              # trials used
    n_durations: int    # distinct T values (1 -> offset forced to 0)
    t_min: float
    t_max: float

    def at(self, T):
        return self.rate * T + self.offset


def fit_line(points):
    """Least-squares value = rate * T + offset over [(T, value)]."""
    if not points:
        raise ModelError('no trials')
    n = len(points)
    Ts = [p[0] for p in points]
    vs = [p[1] for p in points]
    distinct = len(set(Ts))
    if distinct == 1:
        rate, offset = sum(vs) / n / Ts[0], 0.0
    else:
        mt, mv = sum(Ts) / n, sum(vs) / n
        stt = sum((t - mt) ** 2 for t in Ts)
        rate = sum((t - mt) * (v - mv) for t, v in points) / stt
        offset = mv - rate * mt
    rms = math.sqrt(sum((v - (rate * t + offset)) ** 2 for t, v in points) / n)
    return LineFit(rate, offset, rms, n, distinct, min(Ts), max(Ts))


def fit_at(fits, cmd):
    """Fit for cmd; linear interpolation between neighbouring levels, no extrapolation."""
    if not fits:
        raise ModelError('not calibrated')
    keys = sorted(fits)
    for k in keys:
        if abs(k - cmd) < 1e-9:
            return fits[k]
    if not keys[0] < cmd < keys[-1]:
        raise ModelError(f'cmd {cmd} outside calibrated levels {keys}')
    hi = next(k for k in keys if k > cmd)
    lo = keys[keys.index(hi) - 1]
    a, b, w = fits[lo], fits[hi], (cmd - lo) / (hi - lo)
    return LineFit(
        rate=a.rate + w * (b.rate - a.rate),
        offset=a.offset + w * (b.offset - a.offset),
        rms=max(a.rms, b.rms), n=0, n_durations=min(a.n_durations, b.n_durations),
        t_min=min(a.t_min, b.t_min), t_max=max(a.t_max, b.t_max))


def invert(fit, target):
    """Run time T reaching target. Returns (T, warning or None)."""
    if fit.rate <= 0:
        raise ModelError(f'non-positive rate {fit.rate:.3f}')
    T = (target - fit.offset) / fit.rate
    if T < MIN_T:
        raise ModelError(f'T = {T:.2f} s < {MIN_T} s for target {target:g}')
    warn = None
    if T < fit.t_min * (1 - EXTRAP) or T > fit.t_max * (1 + EXTRAP):
        warn = f'T = {T:.2f} s outside calibrated {fit.t_min:g}-{fit.t_max:g} s'
    elif fit.n_durations < 2:
        warn = 'only one calibrated duration (offset assumed 0)'
    return T, warn


def curvature(x, y):
    """Signed curvature of the circle through (0,0) tangent to +x and through (x, y)."""
    return 2.0 * y / (x * x + y * y)


def solve_trim(points):
    """Secant / linear fit of curvature vs trim over [(trim, kappa)]; trim where kappa = 0."""
    if len({p[0] for p in points}) < 2:
        return None
    fit = fit_line(points)
    if fit.rate == 0:
        return None
    return -fit.offset / fit.rate


def wrap_deg(a):
    """Wrap to (-180, 180]."""
    a = math.fmod(a, 360.0)
    if a <= -180.0:
        a += 360.0
    elif a > 180.0:
        a -= 360.0
    return a


def now_str():
    return datetime.now().isoformat(timespec='seconds')


class Calibration:
    def __init__(self, data, path=None):
        self.data = data
        self.path = path
        data.setdefault('drive', {}).setdefault('trim', 0.0)
        data['drive'].setdefault('trials', [])
        for d in DIRS:
            data.setdefault('spin', {}).setdefault(d, {}).setdefault('trials', [])
        data.setdefault('verify', [])

    @classmethod
    def load(cls, path, floor=None, create=False):
        path = os.path.expanduser(path)
        if os.path.exists(path):
            with open(path) as f:
                data = yaml.safe_load(f) or {}
        elif create:
            data = {'floor': floor}
        else:
            raise ModelError(f'calibration file not found: {path}')
        return cls(data, path)

    @classmethod
    def for_floor(cls, calib_dir, floor, create=False):
        if not floor:
            raise ModelError('Set the floor: -p floor:=<name> (e.g. wood).')
        return cls.load(os.path.join(os.path.expanduser(calib_dir), f'{floor}.yaml'),
                        floor, create)

    def save(self):
        self.data['updated'] = date.today().isoformat()
        try:
            self.data['fit'] = self.summary()
        except ModelError:
            pass
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            yaml.safe_dump(self.data, f, sort_keys=False, default_flow_style=None)
        os.replace(tmp, self.path)

    # ---------- data ----------

    @property
    def trim(self):
        return float(self.data['drive']['trim'])

    @trim.setter
    def trim(self, value):
        self.data['drive']['trim'] = round(float(value), 4) + 0.0  # no -0.0

    def drive_trials(self):
        return self.data['drive']['trials']

    def spin_trials(self, d):
        return self.data['spin'][d]['trials']

    def add(self, trials, trial):
        """Append a trial (stamped) and save immediately."""
        trial['time'] = now_str()
        trials.append(trial)
        self.save()

    # ---------- fits ----------

    def drive_fits(self):
        """{cmd: LineFit} of distance travelled (chord length) vs T."""
        groups = {}
        for t in self.drive_trials():
            d = math.hypot(t['x'], t.get('y', 0.0))
            groups.setdefault(float(t['cmd']), []).append((float(t['T']), d))
        return {c: fit_line(p) for c, p in groups.items()}

    def spin_fits(self, d):
        groups = {}
        for t in self.spin_trials(d):
            groups.setdefault(float(t['cmd']), []).append((float(t['T']), abs(t['angle'])))
        return {c: fit_line(p) for c, p in groups.items()}

    def drive_time(self, dist, cmd):
        return invert(fit_at(self.drive_fits(), cmd), dist)

    def spin_time(self, d, deg, cmd):
        fit = fit_at(self.spin_fits(d), cmd)
        T, warn = invert(fit, deg)
        # Optional direction-specific correction to command duration. Keep raw
        # trial angles intact so the measured fit remains auditable.
        scale = float(self.data['spin'][d].get('time_scale', 1.0))
        if not math.isfinite(scale) or scale <= 0:
            raise ModelError(f'spin {d} time_scale must be positive and finite')
        T *= scale
        if T < MIN_T:
            raise ModelError(f'adjusted T = {T:.2f} s < {MIN_T} s for target {deg:g}')
        if T < fit.t_min * (1 - EXTRAP) or T > fit.t_max * (1 + EXTRAP):
            warn = f'adjusted T = {T:.2f} s outside calibrated {fit.t_min:g}-{fit.t_max:g} s'
        return T, warn

    def trim_points(self, cmd=None):
        return [(float(t.get('trim', 0.0)), curvature(t['x'], t.get('y', 0.0)))
                for t in self.drive_trials()
                if cmd is None or abs(t['cmd'] - cmd) < 1e-9]

    def suggest_trim(self, cmd=None):
        return solve_trim(self.trim_points(cmd))

    def summary(self):
        def dump(fits):
            return {c: {k: round(v, 4) if isinstance(v, float) else v
                        for k, v in asdict(f).items()} for c, f in sorted(fits.items())}
        return {'drive': dump(self.drive_fits()),
                'spin': {d: dump(self.spin_fits(d)) for d in DIRS}}


def format_fit(name, cmd, f, unit):
    note = '' if f.n_durations >= 2 else '  (1 duration, offset=0)'
    return (f'{name} cmd {cmd:g}: rate {f.rate:.4f} {unit}/s, offset {f.offset:+.4f} {unit}, '
            f'rms {f.rms:.4f} {unit}, n={f.n}, T {f.t_min:g}-{f.t_max:g} s{note}')
