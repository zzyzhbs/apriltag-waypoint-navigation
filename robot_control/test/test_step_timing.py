import pytest

from robot_control.ol_model import Calibration
from robot_control.step_timing import min_amount, min_step, pick_level

LEVELS = (0.3, 0.2)
SPIN = (0.35, 0.25)


def make_calib(tmp_path, fine=True):
    """coarse drive 0.4 m/s (T 2-4 s), fine 0.1 m/s (T 0.2-0.6 s); same idea for spin."""
    c = Calibration.load(tmp_path / 'f.yaml', 'f', create=True)
    for T in (2.0, 3.0, 4.0):
        c.drive_trials().append({'cmd': 0.3, 'T': T, 'x': 0.4 * T - 0.02, 'y': 0.0})
    for T in (0.4, 0.8, 1.6):
        c.spin_trials('ccw').append({'cmd': 0.35, 'T': T, 'angle': 125.0 * T - 6.0})
    if fine:
        for T in (0.2, 0.4, 0.6):
            c.drive_trials().append({'cmd': 0.2, 'T': T, 'x': 0.1 * T - 0.005, 'y': 0.0})
            c.spin_trials('ccw').append({'cmd': 0.25, 'T': T, 'angle': 20.0 * T - 1.0})
    return c


def test_big_move_uses_coarse(tmp_path):
    s = pick_level(make_calib(tmp_path), 'drive', 'fwd', 1.0, LEVELS)
    assert s.cmd == 0.3
    assert s.T == pytest.approx(1.02 / 0.4)
    assert s.warn is None


def test_small_move_uses_fine(tmp_path):
    s = pick_level(make_calib(tmp_path), 'drive', 'fwd', 0.03, LEVELS)
    assert s.cmd == 0.2
    assert s.T == pytest.approx(0.035 / 0.1)


def test_long_move_extrapolates_fastest(tmp_path):
    s = pick_level(make_calib(tmp_path), 'drive', 'fwd', 2.236, LEVELS)
    assert s.cmd == 0.3
    assert 'outside' in s.warn


def test_no_fine_level_falls_back_to_coarse(tmp_path):
    # 40 deg at 0.35 is T 0.37 s: inside 0.4 s - 20 %
    s = pick_level(make_calib(tmp_path, fine=False), 'spin', 'ccw', 40.0, SPIN)
    assert s.cmd == 0.35 and s.T is not None


def test_too_small_has_no_time(tmp_path):
    s = pick_level(make_calib(tmp_path, fine=False), 'drive', 'fwd', 0.02, LEVELS)
    assert s.T is None
    assert 'cmd 0.3' in s.warn and 'cmd 0.2' in s.warn


def test_min_amount_and_min_T_override(tmp_path):
    c = make_calib(tmp_path)
    assert min_amount(c, 'spin', 'ccw', 0.25) == pytest.approx(20.0 * 0.2 - 1.0)
    c.data['spin']['ccw']['min_T'] = {0.25: 0.1}
    assert min_amount(c, 'spin', 'ccw', 0.25) == pytest.approx(1.0)
    s = pick_level(c, 'spin', 'ccw', 1.5, SPIN)
    assert s.cmd == 0.25 and s.T == pytest.approx(0.125)


def test_min_amount_respects_time_scale(tmp_path):
    c = make_calib(tmp_path)
    c.data['spin']['ccw']['time_scale'] = 0.5     # commanded T = half the fitted T
    # commanded T >= 0.2 needs fitted T >= 0.4
    assert min_amount(c, 'spin', 'ccw', 0.25) == pytest.approx(20.0 * 0.4 - 1.0)


def test_min_step(tmp_path):
    assert min_step(make_calib(tmp_path), 'drive', 'fwd', LEVELS) == pytest.approx(0.015)
    assert min_step(make_calib(tmp_path, fine=False), 'spin', 'cw', SPIN) is None
