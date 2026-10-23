import math
import os

import numpy as np
import pytest
import yaml

from robot_control.ekf import (
    drive_jacobian, drive_model, expected_obs, initial_covariance, noise_from_config,
    obs_jacobian, pose_from_state, predict_drive, predict_spin, sigmas, state_from_pose, update,
)
from robot_control.landmark_map import load_map
from robot_control.se2 import Pose2, to_child, wrap_angle
from robot_control.tag_obs import make_obs

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'hw2')
with open(os.path.join(CONFIG, 'nav.yaml')) as f:
    NOISE = noise_from_config(yaml.safe_load(f)['ekf'])
MAP = load_map(os.path.join(CONFIG, 'landmarks.yaml'))


def true_obs(robot, lm, dx=0.0, dpsi=0.0):
    x, y = to_child(robot, lm.x, lm.y)
    return make_obs(lm.id, 0.0, x + dx, y, lm.yaw - robot.theta + dpsi)


def numeric_jacobian(f, x, eps=1e-6):
    J = np.zeros((3, 3))
    for i in range(3):
        d = np.zeros(3)
        d[i] = eps
        diff = f(x + d) - f(x - d)
        diff[2] = wrap_angle(diff[2])
        J[:, i] = diff / (2 * eps)
    return J


def is_spd(P):
    return np.allclose(P, P.T) and np.all(np.linalg.eigvalsh(P) > 0)


@pytest.mark.parametrize('x', [np.array([0.2, -0.1, 0.3]), np.array([1.0, 1.9, 3.1])])
def test_obs_jacobian_matches_numeric(x):
    lm = MAP.get(1)
    assert np.allclose(obs_jacobian(x, lm), numeric_jacobian(lambda v: expected_obs(v, lm), x),
                       atol=1e-6)


def test_drive_jacobian_matches_numeric():
    x = np.array([0.5, 0.2, 1.1])
    assert np.allclose(drive_jacobian(x, 0.7), numeric_jacobian(lambda v: drive_model(v, 0.7), x),
                       atol=1e-6)


def test_expected_obs_at_wp1():
    h = expected_obs(np.array([1.0, 0.0, 0.0]), MAP.get(0))
    assert h[:2] == pytest.approx([0.5, 0.0])
    assert abs(wrap_angle(h[2] - math.pi)) < 1e-9


def test_predict_drive_and_spin():
    x, P = np.zeros(3), initial_covariance(NOISE)
    x, P = predict_spin(x, P, math.pi / 2, NOISE)
    x, P2 = predict_drive(x, P, 2.0, NOISE)
    assert x == pytest.approx([0.0, 2.0, math.pi / 2])
    sx, sy, st = sigmas(P2)
    assert sx > sy > 0.01           # facing +y: cross-track (world x) error grows faster
    assert st > sigmas(P)[2]
    assert is_spd(P2)


def test_update_pulls_to_truth():
    robot = Pose2(1.0, 0.0, 0.0)
    x = state_from_pose(Pose2(1.04, -0.03, math.radians(4)))
    P = np.diag([0.05 ** 2, 0.05 ** 2, math.radians(5) ** 2])
    r = update(x, P, true_obs(robot, MAP.get(0)), MAP.get(0), NOISE)
    assert r.accepted
    est = pose_from_state(r.x)
    # one update blends prior and measurement: 5 cm / 4 deg off -> about 1 cm / 1 deg
    assert math.hypot(est.x - 1.0, est.y) < 0.015
    assert abs(math.degrees(est.theta)) < 1.5
    assert np.trace(r.P) < np.trace(P)
    assert is_spd(r.P)


def test_heading_across_seam():
    robot = Pose2(1.0, 2.0, math.radians(179.5))
    x = state_from_pose(Pose2(1.0, 2.0, math.radians(-178)))
    P = np.diag([0.02 ** 2, 0.02 ** 2, math.radians(5) ** 2])
    r = update(x, P, true_obs(robot, MAP.get(1)), MAP.get(1), NOISE)
    assert r.accepted
    assert abs(wrap_angle(r.x[2] - robot.theta)) < math.radians(0.5)


def test_gate_rejects_inconsistent_obs():
    robot = Pose2(1.0, 0.0, 0.0)
    x, P = state_from_pose(robot), initial_covariance(NOISE)
    r = update(x, P, true_obs(robot, MAP.get(0), dx=0.3), MAP.get(0), NOISE)
    assert not r.accepted and r.d2 > NOISE.gate
    assert r.x is x
    # the same obs passes when the prior is very uncertain
    r = update(x, np.diag([1.0, 1.0, 1.0]), true_obs(robot, MAP.get(0), dx=0.3),
               MAP.get(0), NOISE)
    assert r.accepted


def test_noisy_updates_converge():
    rng = np.random.default_rng(0)
    robot = Pose2(1.0, 0.0, 0.0)
    x = state_from_pose(Pose2(1.1, 0.08, math.radians(8)))
    P = np.diag([0.1 ** 2, 0.1 ** 2, math.radians(10) ** 2])
    for _ in range(10):
        o = true_obs(robot, MAP.get(0), dx=rng.normal(0, 0.003), dpsi=rng.normal(0, 0.02))
        r = update(x, P, o, MAP.get(0), NOISE)
        x, P = r.x, r.P
    est = pose_from_state(x)
    assert math.hypot(est.x - robot.x, est.y - robot.y) < 0.01
    assert abs(math.degrees(est.theta)) < 1.5
    assert is_spd(P)


def test_far_tag_has_less_weight():
    P = np.diag([0.05 ** 2, 0.05 ** 2, math.radians(3) ** 2])
    lm = MAP.get(0)
    # same 5 cm estimate error, tag 0.5 m vs 1.5 m away
    near = update(state_from_pose(Pose2(1.05, 0.0, 0.0)), P,
                  true_obs(Pose2(1.0, 0.0, 0.0), lm), lm, NOISE)
    far = update(state_from_pose(Pose2(0.05, 0.0, 0.0)), P,
                 true_obs(Pose2(0.0, 0.0, 0.0), lm), lm, NOISE)
    assert near.accepted and far.accepted
    assert sigmas(near.P)[0] < sigmas(far.P)[0]
    assert abs(near.x[0] - 1.0) < abs(far.x[0] - 0.0)
