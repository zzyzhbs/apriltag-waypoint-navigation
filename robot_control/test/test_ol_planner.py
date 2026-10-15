import math
import os

import pytest

from robot_control.ol_model import Calibration
from robot_control.ol_planner import load_waypoints, plan, plan_motions, ready
from robot_control.ol_primitive import PrimitiveRunner, wheel_cmds

WAYPOINTS = os.path.join(os.path.dirname(__file__), '..', '..', 'HW1', 'waypoints.txt')


def simple(legs):
    return [[(s.kind, s.dir, round(s.amount, 1)) for s in leg.steps] for leg in legs]


def test_hw1_plan():
    legs = plan_motions(load_waypoints(WAYPOINTS))
    assert simple(legs) == [
        [('drive', 'fwd', 1.0)],
        [('spin', 'ccw', 90.0), ('drive', 'fwd', 1.0)],
        [('spin', 'cw', 90.0), ('drive', 'fwd', 1.0)],
        [('spin', 'ccw', 90.0), ('drive', 'fwd', 1.0), ('spin', 'ccw', 180.0)],
        [('spin', 'cw', 45.0), ('drive', 'fwd', 1.4), ('spin', 'ccw', 90.3)],
        [('spin', 'cw', 90.3), ('drive', 'fwd', 1.4), ('spin', 'ccw', 135.0)],
    ]
    assert legs[3].steps[2].amount == pytest.approx(180.0 + math.degrees(math.pi / 2 - 1.57))
    assert legs[3].end[2] == pytest.approx(-1.57)
    x, y, th = legs[-1].end
    assert (x, y, th) == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)
    for leg in legs:
        assert leg.end[:2] == pytest.approx(leg.goal[:2], abs=1e-6)


def test_shortest_direction_and_skip():
    legs = plan_motions([(0, 0, 0), (0, 0, math.radians(270)), (0, 0, math.radians(271))])
    assert simple(legs) == [[('spin', 'cw', 90.0)], []]
    legs = plan_motions([(0, 0, 0), (0, 0, math.radians(1.5))])
    assert legs[0].steps == []
    legs = plan_motions([(0, 0, 0), (0.01, 0, 0)])  # within pos_tol
    assert legs[0].steps == []


def test_timing_and_ready(tmp_path):
    c = Calibration.load(tmp_path / 'f.yaml', 'f', create=True)
    for T in (2.0, 4.0):
        c.drive_trials().append({'cmd': 0.3, 'T': T, 'trim': 0, 'x': 0.375 * T, 'y': 0})
    wps = [(0, 0, 0), (1, 0, 0), (1, 1, math.pi / 2)]
    legs = plan(wps, c)
    assert legs[0].steps[0].T == pytest.approx(1.0 / 0.375)
    assert not ready(legs)                     # spin not calibrated
    for d in ('ccw', 'cw'):
        for T in (0.5, 1.5):
            c.spin_trials(d).append({'cmd': 0.35, 'T': T, 'angle': 120 * T})
    assert ready(plan(wps, c))


def test_wheel_cmds():
    assert wheel_cmds('drive', 'fwd', 0.3, 0.1) == pytest.approx((0.27, 0.33))
    assert wheel_cmds('spin', 'ccw', 0.35) == (-0.35, 0.35)
    assert wheel_cmds('spin', 'cw', 0.35) == (0.35, -0.35)


def test_runner_timing():
    t = [0.0]
    sent = []

    def sleep(s):
        t[0] += s

    r = PrimitiveRunner(lambda L, R: sent.append((t[0], L, R)), settle_s=1.0,
                        clock=lambda: t[0], sleep=sleep)
    t0, t1 = r.run(0.3, 0.3, 0.51)
    moving = [s for s in sent if s[1] != 0]
    assert t1 - t0 == pytest.approx(0.51)
    assert len(moving) == 26                   # 50 Hz, last one at 0.50 s
    assert sent[-1][1:] == (0.0, 0.0)
    assert t[0] == pytest.approx(0.51 + 5 * 0.02 + 1.0)
