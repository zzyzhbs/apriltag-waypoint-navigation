"""Differential-drive kinematics and per-floor calibration profiles (no ROS dependency)."""
from __future__ import annotations

import math
import os
from dataclasses import asdict, dataclass, fields
from datetime import date

import yaml

# Robot constants (Waveshare UGV Rover firmware, ugv_config.h)
WHEEL_D = 0.08
TRACK_NOMINAL = 0.172
CMD_MAX = 0.5  # motor_control.py clips each side to +-0.5
ZERO_EPS = 1e-4

NOMINAL = 'nominal'
DEFAULT_PROFILE_DIR = '~/ros2_ws/src/CSE276A/robot_control/config/floors'


@dataclass
class FloorProfile:
    floor: str
    # driving (straight / small corrections)
    linear_scale: float = 1.0      # actual ground speed / commanded speed
    trim: float = 0.0              # L *= (1 - trim), R *= (1 + trim)
    track_eff: float = TRACK_NOMINAL  # effective track width while driving, m
    min_cmd: float = 0.0           # rolling deadband
    # in-place rotation primitive
    rot_cmd: float = 0.3           # wheel command magnitude for spinning
    rot_rate_ccw: float = 0.0      # steady rate at rot_cmd, rad/s (0 = unknown)
    rot_rate_cw: float = 0.0
    rot_coast_ccw: float = 0.0     # extra rotation after the stop command, rad
    rot_coast_cw: float = 0.0
    rot_deadband: float = 0.0      # spin start command (logged only)
    # IMU
    gyro_scale: float = 1.0        # deg/s per gz unit (sign included)


PARAM_NAMES = [f.name for f in fields(FloorProfile) if f.name != 'floor']


# ---------- angles ----------

def wrap_angle(a: float) -> float:
    """Normalize angle to (-pi, pi]."""
    a = math.atan2(math.sin(a), math.cos(a))
    return math.pi if a == -math.pi else a


def quat_to_yaw(w: float, x: float, y: float, z: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ---------- kinematics ----------

def inverse(v: float, w: float, p: FloorProfile) -> tuple[float, float]:
    """Body (v, w) -> wheel ground speeds (vL, vR)."""
    half = p.track_eff / 2.0
    return v - w * half, v + w * half


def forward(vL: float, vR: float, p: FloorProfile) -> tuple[float, float]:
    """Wheel ground speeds (vL, vR) -> body (v, w)."""
    return (vR + vL) / 2.0, (vR - vL) / p.track_eff


def to_command(v: float, w: float, p: FloorProfile,
               cmd_max: float = CMD_MAX) -> tuple[float, float]:
    """Body (v, w) -> motor command [L, R] (spec section 4, steps 1-5)."""
    vL, vR = inverse(v, w, p)
    L, R = vL / p.linear_scale, vR / p.linear_scale
    L, R = L * (1.0 - p.trim), R * (1.0 + p.trim)

    # Proportional saturation keeps L:R ratio, i.e. the turning radius
    s = max(1.0, abs(L) / cmd_max, abs(R) / cmd_max)
    L, R = L / s, R / s

    L, R = _deadband(L, p.min_cmd), _deadband(R, p.min_cmd)
    return _clip(L, cmd_max), _clip(R, cmd_max)


def from_command(L: float, R: float, p: FloorProfile) -> tuple[float, float]:
    """Actually-sent command [L, R] -> estimated body (v, w)."""
    L, R = L / (1.0 - p.trim), R / (1.0 + p.trim)
    return forward(L * p.linear_scale, R * p.linear_scale, p)


def rotate_command(direction: int, p: FloorProfile) -> tuple[float, float]:
    """In-place spin at rot_cmd; direction +1 = CCW, -1 = CW."""
    c = p.rot_cmd if direction > 0 else -p.rot_cmd
    return -c, c


def rotate_coast(direction: int, p: FloorProfile) -> float:
    """Angle (rad, >= 0) the robot keeps turning after the stop command."""
    return p.rot_coast_ccw if direction > 0 else p.rot_coast_cw


def integrate_pose(x: float, y: float, th: float,
                   v: float, w: float, dt: float) -> tuple[float, float, float]:
    """One dead-reckoning step; midpoint heading reduces error on arcs."""
    th_mid = th + w * dt / 2.0
    return (x + v * math.cos(th_mid) * dt,
            y + v * math.sin(th_mid) * dt,
            wrap_angle(th + w * dt))


def _deadband(x: float, min_cmd: float) -> float:
    if abs(x) < ZERO_EPS:
        return 0.0
    if abs(x) < min_cmd:
        return math.copysign(min_cmd, x)
    return x


def _clip(x: float, lim: float) -> float:
    return max(-lim, min(lim, x))


# ---------- profile I/O ----------

def profile_path(profile_dir: str, floor: str) -> str:
    return os.path.join(os.path.expanduser(profile_dir), f'{floor}.yaml')


def _read_yaml(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def load_profile(profile_dir: str, floor: str, allow_missing: bool = False) -> FloorProfile:
    """Load <profile_dir>/<floor>.yaml; 'nominal' returns defaults.

    allow_missing=True returns defaults for a new floor (used by calibration).
    """
    if floor == NOMINAL:
        return FloorProfile(floor=NOMINAL)
    path = profile_path(profile_dir, floor)
    if not os.path.exists(path):
        if allow_missing:
            return FloorProfile(floor=floor)
        raise FileNotFoundError(f'No profile for floor "{floor}": {path}')
    params = _read_yaml(path).get('params') or {}
    known = {k: v for k, v in params.items() if k in PARAM_NAMES}
    return FloorProfile(floor=floor, **known)


def load_trials(profile_dir: str, floor: str, test: str | None = None) -> list[dict]:
    trials = _read_yaml(profile_path(profile_dir, floor)).get('trials') or []
    return [t for t in trials if test is None or t.get('test') == test]


def save_profile(profile_dir: str, p: FloorProfile, trial: dict | None = None) -> str:
    """Write params; append trial to the existing trial log. Returns file path."""
    if p.floor == NOMINAL:
        raise ValueError('The nominal profile is built in and cannot be saved')
    path = profile_path(profile_dir, p.floor)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    trials = _read_yaml(path).get('trials') or []
    if trial is not None:
        trials.append(trial)
    params = asdict(p)
    params.pop('floor')
    data = {
        'floor': p.floor,
        'updated': date.today().isoformat(),
        'params': params,
        'trials': trials,
    }
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)
    return path
