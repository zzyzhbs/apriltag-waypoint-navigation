import csv
import math
import os
import random

import pytest

from robot_control.heading import GyroHeading

DIAG_CSV = os.path.join(os.path.dirname(__file__), '..', 'config', 'diag',
                        'imu_diag_carpet_20261013_020300.csv')


def feed(h, t0, dur, gz_fn, stationary, hz=50.0):
    n = int(dur * hz)
    for i in range(n):
        t = t0 + i / hz
        h.update(t, gz_fn(t), stationary)
    return t0 + n / hz


def test_constant_rate_with_scale():
    h = GyroHeading(gyro_scale=2.0)
    feed(h, 0.0, 2.0, lambda t: 45.0, stationary=False)  # 45 units * 2 = 90 deg/s
    assert math.degrees(h.theta) == pytest.approx(180.0, abs=2.0)


def test_stationary_freezes_and_learns_bias():
    h = GyroHeading()
    rng = random.Random(0)
    feed(h, 0.0, 5.0, lambda t: 2.0 + rng.gauss(0, 13), stationary=True)
    assert h.theta == 0.0
    assert h.bias == pytest.approx(2.0, abs=1.0)


def test_zupt_removes_bias_error():
    bias = 3.0
    h_zupt, h_raw = GyroHeading(), GyroHeading()
    t = feed(h_zupt, 0.0, 5.0, lambda t: bias, stationary=True)
    feed(h_zupt, t, 2.0, lambda t: 45.0 + bias, stationary=False)
    feed(h_raw, t, 2.0, lambda t: 45.0 + bias, stationary=False)
    assert math.degrees(h_zupt.theta) == pytest.approx(90.0, abs=1.0)
    assert math.degrees(h_raw.theta) == pytest.approx(96.0, abs=1.0)


def test_bursty_timestamps_and_duplicates():
    # samples arrive in bursts of 3 identical values every 60 ms, like the robot
    h = GyroHeading()
    t = 0.0
    while t < 2.0:
        for k in range(3):
            h.update(t + k * 0.001, 90.0, stationary=False)
        t += 0.06
    assert math.degrees(h.theta) == pytest.approx(180.0, abs=6.0)


def test_bad_dt_skipped():
    h = GyroHeading()
    h.update(0.0, 90.0, False)
    h.update(5.0, 90.0, False)   # 5 s gap: skipped
    h.update(4.0, 90.0, False)   # time going backwards: skipped
    assert h.theta == 0.0


# (true deg, motion window s) from .agent/notes/imu-diag-2026-10-13.md; truth is hand-aligned
DIAG_SEGMENTS = [(90, (23.5, 25.6)), (90, (39.0, 41.0)), (180, (51.5, 54.6)),
                 (-360, (73.2, 79.6)), (90, (92.2, 93.6)), (-90, (102.0, 104.6))]


@pytest.mark.skipif(not os.path.exists(DIAG_CSV), reason='diag CSV not available')
def test_replay_real_diag():
    rows = [tuple(map(float, r[:4])) for r in list(csv.reader(open(DIAG_CSV)))[1:]]
    t0 = rows[0][0]
    h = GyroHeading()
    starts = {}
    ends = {}
    for t_abs, _, _, gz in rows:
        t = t_abs - t0
        moving = any(a <= t < b for _, (a, b) in DIAG_SEGMENTS)
        h.update(t, gz, stationary=not moving)
        for i, (_, (a, b)) in enumerate(DIAG_SEGMENTS):
            if t < a:
                starts[i] = h.theta
            if t < b:
                ends[i] = h.theta
    errs = [math.degrees(ends[i] - starts[i]) - true for i, (true, _) in enumerate(DIAG_SEGMENTS)]
    print('replay errors (deg):', [round(e, 1) for e in errs])
    # truth is hand-aligned (+-5 deg); causal bias -> last fast CW turn is worst (~-9 deg)
    assert all(abs(e) < 10.0 for e in errs)
    assert abs(sum(errs)) < 5.0  # loop closure: started and ended at the 0 deg mark


def test_bias_accumulates_across_still_periods():
    # short still windows are noisy; bias should average over all still periods
    h = GyroHeading()
    rng = random.Random(1)
    t = 0.0
    for _ in range(4):
        t = feed(h, t, 3.0, lambda t: 1.0 + rng.gauss(0, 13), stationary=True)
        t = feed(h, t, 1.0, lambda t: 50.0, stationary=False)
    assert h.bias == pytest.approx(1.0, abs=0.7)
