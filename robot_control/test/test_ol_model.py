import math

import pytest

from robot_control.ol_model import (
    Calibration, ModelError, curvature, fit_at, fit_line, invert, solve_trim, wrap_deg,
)


def make_calib(tmp_path):
    c = Calibration.load(tmp_path / 'wood.yaml', 'wood', create=True)
    for T in (2.0, 3.0, 4.0):
        for cmd, v, d0 in ((0.2, 0.25, -0.03), (0.4, 0.5, 0.05)):
            c.drive_trials().append({'cmd': cmd, 'T': T, 'trim': 0.0, 'x': v * T + d0, 'y': 0.0})
    for T in (0.5, 1.5):
        c.spin_trials('ccw').append({'cmd': 0.35, 'T': T, 'angle': 120.0 * T + 6.0})
    return c


def test_fit_known_line():
    f = fit_line([(T, 0.375 * T - 0.02) for T in (2.0, 3.0, 4.0, 5.0)])
    assert f.rate == pytest.approx(0.375)
    assert f.offset == pytest.approx(-0.02)
    assert f.rms == pytest.approx(0.0, abs=1e-12)
    assert (f.n, f.n_durations, f.t_min, f.t_max) == (4, 4, 2.0, 5.0)


def test_fit_single_duration_zero_offset():
    f = fit_line([(4.0, 1.5), (4.0, 1.46)])
    assert f.offset == 0.0
    assert f.rate == pytest.approx(1.48 / 4.0)
    assert f.rms == pytest.approx(0.02)


def test_interpolation_and_range(tmp_path):
    fits = make_calib(tmp_path).drive_fits()
    f = fit_at(fits, 0.3)
    assert f.rate == pytest.approx(0.375)
    assert f.offset == pytest.approx(0.01)
    assert fit_at(fits, 0.4).rate == pytest.approx(0.5)
    with pytest.raises(ModelError):
        fit_at(fits, 0.5)
    with pytest.raises(ModelError):
        fit_at({}, 0.3)


def test_invert(tmp_path):
    c = make_calib(tmp_path)
    T, warn = c.drive_time(1.0, 0.3)
    assert T == pytest.approx((1.0 - 0.01) / 0.375)
    assert warn is None
    T, warn = c.spin_time('ccw', 90.0, 0.35)
    assert T == pytest.approx(84.0 / 120.0)
    assert warn is None


def test_invert_warnings_and_errors(tmp_path):
    c = make_calib(tmp_path)
    _, warn = c.drive_time(3.0, 0.3)    # T ~ 8 s, calibrated up to 4 s
    assert 'outside' in warn
    _, warn = c.drive_time(1.75, 0.3)   # T ~ 4.64 s, within +20%
    assert warn is None
    with pytest.raises(ModelError):
        c.drive_time(0.05, 0.3)         # T < 0.2 s
    with pytest.raises(ModelError):
        c.spin_time('cw', 90.0, 0.35)   # not calibrated
    single = fit_line([(4.0, 1.5)])
    assert 'one calibrated duration' in invert(single, 1.4)[1]


def test_curvature():
    r = 2.0
    a = 0.5
    assert curvature(r * math.sin(a), r * (1 - math.cos(a))) == pytest.approx(1 / r)
    assert curvature(1.0, 0.0) == 0.0


def test_trim_secant(tmp_path):
    c = make_calib(tmp_path)
    c.drive_trials().clear()
    # kappa = 4 * (trim - 0.015): two runs, then solve
    for trim in (0.0, 0.02):
        k = 4 * (trim - 0.015)
        y = (1 - math.cos(1.5 * k)) / k
        x = math.sin(1.5 * k) / k
        c.drive_trials().append({'cmd': 0.3, 'T': 4.0, 'trim': trim, 'x': x, 'y': y})
    assert c.suggest_trim(0.3) == pytest.approx(0.015, abs=1e-9)
    assert c.suggest_trim(0.2) is None
    assert solve_trim([(0.0, 0.1), (0.0, 0.2)]) is None


def test_yaml_roundtrip(tmp_path):
    c = make_calib(tmp_path)
    c.trim = 0.012345
    c.add(c.spin_trials('cw'), {'cmd': 0.35, 'T': 1.0, 'angle': 118.0})
    d = Calibration.load(tmp_path / 'wood.yaml')
    assert d.trim == 0.0123
    assert d.data['floor'] == 'wood'
    assert len(d.drive_trials()) == 6
    assert d.spin_trials('cw')[0]['angle'] == 118.0
    assert 'time' in d.spin_trials('cw')[0]
    assert d.drive_fits()[0.4].rate == pytest.approx(0.5)
    assert 'fit' in d.data
    with pytest.raises(ModelError):
        Calibration.load(tmp_path / 'missing.yaml')
    with pytest.raises(ModelError):
        Calibration.for_floor(tmp_path, '')


def test_wrap_deg():
    assert wrap_deg(190) == -170
    assert wrap_deg(-180) == 180
    assert wrap_deg(540) == 180
