import math

import pytest

from robot_control.landmark_map import Landmark
from robot_control.localize import pose_from_tag, tag_from_pose
from robot_control.se2 import Pose2, to_child, wrap_angle
from robot_control.tag_obs import (
    aggregate, is_usable, make_obs, obs_from_transform, summarize,
)


def matrix_to_quat(R):
    """Rotation matrix -> (x, y, z, w); fine for the test rotations used here."""
    w = math.sqrt(max(0.0, 1 + R[0][0] + R[1][1] + R[2][2])) / 2
    if w > 1e-6:
        return ((R[2][1] - R[1][2]) / (4 * w), (R[0][2] - R[2][0]) / (4 * w),
                (R[1][0] - R[0][1]) / (4 * w), w)
    x = math.sqrt(max(0.0, 1 + R[0][0] - R[1][1] - R[2][2])) / 2
    return x, (R[0][1] + R[1][0]) / (4 * x), (R[0][2] + R[2][0]) / (4 * x), 0.0


def fake_transform(robot, lm, z_sign):
    """Translation and quaternion of a vertical tag in base_link, as TF would give."""
    tx, ty = to_child(robot, lm.x, lm.y)
    psi = lm.yaw - robot.theta
    n = (math.cos(psi), math.sin(psi), 0.0)
    z = [z_sign * v for v in n]
    y = (0.0, 0.0, z_sign * 1.0)
    x = (y[1] * z[2] - y[2] * z[1], y[2] * z[0] - y[0] * z[2], y[0] * z[1] - y[1] * z[0])
    R = [[x[i], y[i], z[i]] for i in range(3)]
    return (tx, ty, 0.0), matrix_to_quat(R)


TAG0 = Landmark(0, 1.5, 0.0, math.pi)


@pytest.mark.parametrize('z_sign', [1, -1])
def test_obs_from_transform_face_on(z_sign):
    t, q = fake_transform(Pose2(1.0, 0.0, 0.0), TAG0, z_sign)    # robot at WP1
    o = obs_from_transform(0, 5.0, t, q, z_sign)
    assert (o.x, o.y) == pytest.approx((0.5, 0.0))
    assert abs(wrap_angle(o.psi - math.pi)) < 1e-9        # normal points back at robot
    assert o.dist == pytest.approx(0.5)
    assert o.oblique == pytest.approx(0.0, abs=1e-9)


def test_wrong_z_sign_flips_normal():
    t, q = fake_transform(Pose2(1.0, 0.0, 0.0), TAG0, 1)
    o = obs_from_transform(0, 0.0, t, q, -1)
    assert o.psi == pytest.approx(0.0, abs=1e-9)
    assert o.oblique == pytest.approx(math.pi)


def test_oblique_angle():
    # tag 1 m ahead, normal pointing 45° off the line back to the robot
    o = make_obs(0, 0.0, 1.0, 0.0, math.radians(135))
    assert math.degrees(o.oblique) == pytest.approx(45.0)


def test_is_usable():
    assert is_usable(make_obs(0, 0, 1.0, 0.0, math.pi))
    assert not is_usable(make_obs(0, 0, 0.2, 0.0, math.pi))           # too close
    assert not is_usable(make_obs(0, 0, 3.5, 0.0, math.pi))           # too far
    assert not is_usable(make_obs(0, 0, 1.0, 0.0, math.radians(110)))  # 70° oblique


def test_summarize_median_and_seam():
    frames = [make_obs(1, t, 1.0 + d, 0.0, math.radians(a))
              for t, d, a in ((1, 0.0, 179), (2, 0.004, -179), (3, -0.004, 180), (4, 0.001, 179))]
    o, reason = summarize(frames)
    assert reason is None
    assert o.x == pytest.approx(1.0005)
    assert abs(wrap_angle(o.psi - math.pi)) < math.radians(0.6)
    assert (o.count, o.stamp) == (4, 4)


def test_summarize_rejects():
    few = [make_obs(1, 0, 1.0, 0.0, math.pi)] * 2
    assert summarize(few)[0] is None
    shaky = [make_obs(1, 0, 1.0 + d, 0.0, math.pi) for d in (0.0, 0.08, -0.08)]
    o, reason = summarize(shaky)
    assert o is None and 'unstable' in reason


def test_aggregate_splits_ids():
    frames = [make_obs(0, t, 1.0, 0.0, math.pi) for t in range(4)]
    frames += [make_obs(2, 0, 2.0, 0.0, math.pi)]
    accepted, rejected = aggregate(frames)
    assert list(accepted) == [0]
    assert list(rejected) == [2]


@pytest.mark.parametrize('robot', [Pose2(1.0, 0.0, 0.0), Pose2(0.2, -0.3, 0.4),
                                   Pose2(1.1, 1.9, math.radians(170))])
def test_pose_from_tag_recovers_robot(robot):
    t, q = fake_transform(robot, TAG0, 1)
    est = pose_from_tag(obs_from_transform(0, 0, t, q, 1), TAG0)
    assert (est.x, est.y) == pytest.approx((robot.x, robot.y))
    assert abs(wrap_angle(est.theta - robot.theta)) < 1e-9


def test_tag_from_pose_recovers_map():
    robot = Pose2(0.3, 0.1, -0.2)
    t, q = fake_transform(robot, TAG0, 1)
    tag = tag_from_pose(obs_from_transform(0, 0, t, q, 1), robot)
    assert (tag.x, tag.y) == pytest.approx((TAG0.x, TAG0.y))
    assert abs(wrap_angle(tag.theta - TAG0.yaw)) < 1e-9
