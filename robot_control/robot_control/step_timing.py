"""Pick the command level and run time for one primitive (no ROS).

Levels are listed fast -> slow: coarse levels for big moves, fine levels for
small corrections. Amounts use HW1 units: drive in m, spin in deg (positive).
"""
from robot_control.ol_model import EXTRAP, ModelError, fit_for
from robot_control.ol_planner import Step

DRIVE_LEVELS = (0.3, 0.2)
SPIN_LEVELS = (0.35, 0.25)


def level_section(calib, kind, direction):
    """Calibration YAML section holding this primitive's trials and options."""
    return calib.data['drive'] if kind == 'drive' else calib.data['spin'][direction]


def level_fit(calib, kind, direction, cmd):
    fits = calib.drive_fits() if kind == 'drive' else calib.spin_fits(direction)
    return fit_for(fits, cmd)


def run_time(calib, kind, direction, amount, cmd):
    """(T, warning) from the calibration; raises ModelError."""
    if kind == 'drive':
        return calib.drive_time(amount, cmd)
    return calib.spin_time(direction, amount, cmd)


def range_check(fit, T):
    """'short' / 'ok' / 'long' against the calibrated durations (±EXTRAP)."""
    if T < fit.t_min * (1 - EXTRAP):
        return 'short'
    if T > fit.t_max * (1 + EXTRAP):
        return 'long'
    return 'ok'


def pick_level(calib, kind, direction, amount, levels):
    """Step with cmd and T. T is None (warn says why) when no level can do it.

    1. the fastest level whose T is inside its calibrated range;
    2. else, for long moves, the fastest level (a long run is steady speed);
    3. else the slowest level (closest to the small amount asked for).
    """
    found, errors = [], []
    for cmd in levels:
        try:
            T, warn = run_time(calib, kind, direction, amount, cmd)
            where = range_check(level_fit(calib, kind, direction, cmd), T)
        except ModelError as e:
            errors.append(f'cmd {cmd:g}: {e}')
            continue
        if where == 'ok':
            return Step(kind, direction, amount, cmd, T, warn)
        found.append((where, cmd, T, warn))
    if not found:
        cmd = levels[-1] if levels else 0.0
        return Step(kind, direction, amount, cmd, None, '; '.join(errors) or 'no levels')
    longs = [f for f in found if f[0] == 'long']
    _, cmd, T, warn = longs[0] if longs else found[-1]
    return Step(kind, direction, amount, cmd, T, warn)


def min_amount(calib, kind, direction, cmd):
    """Smallest move one level can make (run for its min_T); raises ModelError."""
    fit = level_fit(calib, kind, direction, cmd)
    section = level_section(calib, kind, direction)
    min_T = calib.min_T(section, cmd)
    scale = float(section.get('time_scale', 1.0)) if kind == 'spin' else 1.0
    # spin_time checks min_T both before and after applying time_scale
    return fit.at(max(min_T, min_T / scale))


def min_step(calib, kind, direction, levels):
    """Smallest move over all calibrated levels, or None if none is calibrated."""
    amounts = []
    for cmd in levels:
        try:
            amounts.append(min_amount(calib, kind, direction, cmd))
        except ModelError:
            continue
    return min(amounts) if amounts else None
