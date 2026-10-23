"""2D rigid-body geometry (no ROS). Angles in radians.

A Pose2 (x, y, theta) is the pose of a child frame in a parent frame:
its origin is (x, y) and its x axis points at theta.
"""
import math
from typing import NamedTuple


class Pose2(NamedTuple):
    x: float
    y: float
    theta: float


def wrap_angle(a):
    """Wrap to (-pi, pi]."""
    a = math.fmod(a, 2.0 * math.pi)
    if a <= -math.pi:
        a += 2.0 * math.pi
    elif a > math.pi:
        a -= 2.0 * math.pi
    return a


def to_parent(p, px, py):
    """Point given in frame p -> parent frame."""
    c, s = math.cos(p.theta), math.sin(p.theta)
    return p.x + c * px - s * py, p.y + s * px + c * py


def to_child(p, px, py):
    """Point given in the parent frame -> frame p."""
    dx, dy = px - p.x, py - p.y
    c, s = math.cos(p.theta), math.sin(p.theta)
    return c * dx + s * dy, -s * dx + c * dy


def compose(a, b):
    """a ⊕ b: pose b (given in frame a) expressed in a's parent frame."""
    x, y = to_parent(a, b.x, b.y)
    return Pose2(x, y, wrap_angle(a.theta + b.theta))


def inverse(p):
    """Pose of the parent frame expressed in frame p."""
    x, y = to_child(p, 0.0, 0.0)
    return Pose2(x, y, wrap_angle(-p.theta))


def quat_to_matrix(x, y, z, w):
    """Unit quaternion -> 3x3 rotation matrix (list of rows)."""
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def yaw_to_quat(yaw):
    """Rotation about z -> quaternion (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
