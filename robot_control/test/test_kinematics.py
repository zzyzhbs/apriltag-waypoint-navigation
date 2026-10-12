import math

import pytest

from robot_control.kinematics import (
    CMD_MAX, FloorProfile, forward, from_command, integrate_pose, inverse,
    load_profile, load_trials, quat_to_yaw, save_profile, to_command, wrap_angle,
)

NOM = FloorProfile(floor='nominal')
CAL = FloorProfile(floor='test', linear_scale=0.9, track_eff=0.25, trim=0.03, min_cmd=0.05)


@pytest.mark.parametrize('p', [NOM, CAL])
@pytest.mark.parametrize('v,w', [(0.3, 0.0), (0.0, 1.0), (0.2, -0.5), (-0.1, 0.3)])
def test_forward_inverse_roundtrip(p, v, w):
    assert forward(*inverse(v, w, p), p) == pytest.approx((v, w))


def test_nominal_straight_command():
    assert to_command(0.3, 0.0, NOM) == pytest.approx((0.3, 0.3))


def test_nominal_rotation_command():
    L, R = to_command(0.0, 1.0, NOM)
    assert L == pytest.approx(-0.086) and R == pytest.approx(0.086)


def test_saturation_keeps_ratio():
    # wants [0.8, 0.4] -> [0.5, 0.25], same turning radius
    v, w = forward(0.8, 0.4, NOM)
    L, R = to_command(v, w, NOM)
    assert max(abs(L), abs(R)) == pytest.approx(CMD_MAX)
    assert L / R == pytest.approx(2.0)


@pytest.mark.parametrize('v,w', [(0.2, 0.3), (0.1, -0.3), (0.05, 0.0)])
def test_from_command_inverts_to_command(v, w):
    # no saturation / deadband in these cases
    assert from_command(*to_command(v, w, CAL), CAL) == pytest.approx((v, w))


def test_deadband_boosts_small_and_keeps_zero():
    p = FloorProfile(floor='t', min_cmd=0.05)
    L, R = to_command(0.0, 0.2, p)  # +-0.0172 before deadband
    assert (L, R) == pytest.approx((-0.05, 0.05))
    assert to_command(0.0, 0.0, p) == (0.0, 0.0)


@pytest.mark.parametrize('a,expected', [
    (0.0, 0.0), (math.pi, math.pi), (-math.pi, math.pi),
    (3 * math.pi / 2, -math.pi / 2), (math.radians(-358), math.radians(2)),
])
def test_wrap_angle(a, expected):
    assert wrap_angle(a) == pytest.approx(expected)


@pytest.mark.parametrize('yaw_deg', [0, 90, -90, 179, -45])
def test_quat_to_yaw(yaw_deg):
    h = math.radians(yaw_deg) / 2
    assert quat_to_yaw(math.cos(h), 0.0, 0.0, math.sin(h)) == pytest.approx(math.radians(yaw_deg))


def test_integrate_pose_straight_and_arc():
    x, y, th = integrate_pose(0, 0, 0, 0.3, 0.0, 1.0)
    assert (x, y, th) == pytest.approx((0.3, 0.0, 0.0))

    # quarter circle of radius 1 in many small steps ends near (1, 1, pi/2)
    x = y = th = 0.0
    n = 1000
    for _ in range(n):
        x, y, th = integrate_pose(x, y, th, 1.0, 1.0, (math.pi / 2) / n)
    assert (x, y, th) == pytest.approx((1.0, 1.0, math.pi / 2), abs=1e-4)


def test_profile_roundtrip(tmp_path):
    d = str(tmp_path)
    assert load_profile(d, 'nominal') == NOM
    with pytest.raises(FileNotFoundError):
        load_profile(d, 'wood')
    assert load_profile(d, 'wood', allow_missing=True) == FloorProfile(floor='wood')

    p = FloorProfile(floor='wood', linear_scale=0.95, track_eff=0.21)
    save_profile(d, p, {'test': 'straight', 'measured_m': 1.14})
    save_profile(d, p, {'test': 'rotate', 'omega': 1.9})
    assert load_profile(d, 'wood') == p
    assert len(load_trials(d, 'wood')) == 2
    assert load_trials(d, 'wood', 'straight')[0]['measured_m'] == 1.14
