import math

import pytest

from robot_control.se2 import (
    Pose2, compose, inverse, quat_to_matrix, to_child, to_parent, wrap_angle, yaw_to_quat,
)


def approx_pose(p, q, tol=1e-9):
    return (p.x == pytest.approx(q.x, abs=tol) and p.y == pytest.approx(q.y, abs=tol)
            and abs(wrap_angle(p.theta - q.theta)) < tol)


def test_wrap_angle_range():
    assert wrap_angle(math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-math.pi) == pytest.approx(math.pi)
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert wrap_angle(-7 * math.pi) == pytest.approx(math.pi)
    assert wrap_angle(0.1) == pytest.approx(0.1)


def test_to_parent_and_back():
    p = Pose2(1.0, 2.0, math.pi / 2)
    # 1 m ahead of a robot facing +y is one step up in y
    assert to_parent(p, 1.0, 0.0) == pytest.approx((1.0, 3.0))
    assert to_child(p, *to_parent(p, 0.3, -0.7)) == pytest.approx((0.3, -0.7))


def test_compose_known():
    a = Pose2(1.0, 0.0, math.pi / 2)
    b = Pose2(2.0, 0.0, math.pi / 2)
    assert approx_pose(compose(a, b), Pose2(1.0, 2.0, math.pi))


def test_inverse_gives_identity():
    p = Pose2(0.4, -1.2, 2.5)
    assert approx_pose(compose(p, inverse(p)), Pose2(0, 0, 0))
    assert approx_pose(compose(inverse(p), p), Pose2(0, 0, 0))


def test_quat_matrix_yaw():
    yaw = 0.7
    R = quat_to_matrix(*yaw_to_quat(yaw))
    assert R[0][0] == pytest.approx(math.cos(yaw))
    assert R[1][0] == pytest.approx(math.sin(yaw))
    assert R[2][2] == pytest.approx(1.0)


def test_quat_matrix_optical_frame():
    # base_link -> camera_frame from the launch file (yaw -90, roll -90)
    cr, sr = math.cos(-math.pi / 4), math.sin(-math.pi / 4)
    cy, sy = math.cos(-math.pi / 4), math.sin(-math.pi / 4)
    q = (sr * cy, sr * sy, cr * sy, cr * cy)      # q_yaw * q_roll as (x, y, z, w)
    R = quat_to_matrix(*q)
    col = [[R[i][j] for i in range(3)] for j in range(3)]
    assert col[2] == pytest.approx([1, 0, 0], abs=1e-9)    # camera z = base forward
    assert col[0] == pytest.approx([0, -1, 0], abs=1e-9)   # camera x = base right
    assert col[1] == pytest.approx([0, 0, -1], abs=1e-9)   # camera y = base down
