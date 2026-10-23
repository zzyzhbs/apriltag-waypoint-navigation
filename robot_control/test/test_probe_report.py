import math

import pytest

from robot_control.landmark_map import Landmark
from robot_control.localize import fuse_poses, tag_weight
from robot_control.probe_report import (
    circular_std, format_stats, survey_diff, survey_yaml, tag_stats,
)
from robot_control.se2 import Pose2, wrap_angle
from robot_control.tag_obs import make_obs


def test_fuse_poses_weighted():
    p = fuse_poses([Pose2(0.0, 0.0, math.radians(179)), Pose2(1.0, 0.0, math.radians(-179))],
                   [3.0, 1.0])
    assert p.x == pytest.approx(0.25)
    assert abs(wrap_angle(p.theta - math.radians(179.5))) < math.radians(0.05)


def test_tag_weight_prefers_close():
    near = make_obs(0, 0, 0.5, 0.0, math.pi)
    far = make_obs(0, 0, 2.0, 0.0, math.pi)
    assert tag_weight(near) == pytest.approx(16 * tag_weight(far))
    assert tag_weight(make_obs(0, 0, 0.1, 0.0, math.pi)) == pytest.approx(1 / 0.09)


def test_circular_std_across_seam():
    s = circular_std([math.radians(a) for a in (179, -179, 179, -179)])
    assert math.degrees(s) == pytest.approx(1.0)


def test_tag_stats():
    group = [make_obs(1, t, 1.0 + d, 0.1, math.pi) for t, d in enumerate((-0.002, 0.0, 0.002))]
    s = tag_stats(group, 2.0)
    assert (s['n'], s['rate']) == (3, 1.5)
    assert s['x'] == pytest.approx(1.0)
    assert s['x_std'] == pytest.approx(math.sqrt(8e-6 / 3))
    assert s['y_std'] == pytest.approx(0.0)
    assert 'tag 1' in format_stats(1, s)


def test_survey_diff_and_yaml():
    lm = Landmark(0, 1.5, 0.0, math.pi)
    dx, dy, d, dyaw = survey_diff(lm, Pose2(1.53, -0.04, math.radians(-178)))
    assert (dx, dy, d) == pytest.approx((0.03, -0.04, 0.05))
    assert math.degrees(dyaw) == pytest.approx(2.0)
    data = survey_yaml(0.166, {0: Pose2(1.53, -0.04, math.pi)}, Pose2(0, 0, 0))
    assert data['landmarks'] == [{'id': 0, 'x': 1.53, 'y': -0.04, 'yaw_deg': 180.0}]
