import math
import os
import random

import pytest

from robot_control.heading import GyroHeading
from robot_control.kinematics import FloorProfile, wrap_angle
from robot_control.navigator import NavParams, Navigator, load_waypoints

WAYPOINTS = os.path.join(os.path.dirname(__file__), '..', '..', 'HW1', 'waypoints.txt')
HW1_WPS = [(0, 0, 0), (1, 0, 0), (1, 1, 1.57), (2, 1, 0), (2, 2, -1.57), (1, 1, -0.78), (0, 0, 0)]


class SimRobot:
    """Wheels follow [L, R] (m/s) through a first-order lag; gyro has bias + noise.

    track=0.4: skid steering, spin at rot_cmd 0.3 gives ~1.5 rad/s like the real robot.
    """

    def __init__(self, track=0.4, tau=0.15, bias=-4.0, noise=13.0, seed=0):
        self.x = self.y = self.th = 0.0
        self.vl = self.vr = 0.0
        self.track, self.tau, self.bias, self.noise = track, tau, bias, noise
        self.rng = random.Random(seed)

    def advance(self, L, R, dt):
        a = dt / self.tau
        self.vl += (L - self.vl) * a
        self.vr += (R - self.vr) * a
        v, w = (self.vl + self.vr) / 2, (self.vr - self.vl) / self.track
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        self.th += w * dt
        return math.degrees(w) + self.bias + self.rng.gauss(0, self.noise)


def run(wps, profile, robot, params=None, t_max=300.0):
    robot.x, robot.y, robot.th = wps[0]
    nav = Navigator(wps, profile, params)
    heading = GyroHeading(profile.gyro_scale)
    heading.reset(wps[0][2])
    cmd, t, dt = (0.0, 0.0), 0.0, 0.002
    next_imu, next_ctrl = 0.0, 0.0
    true_at = []
    while t < t_max and not nav.done:
        gz = robot.advance(*cmd, dt)
        t += dt
        if t >= next_imu:
            heading.update(t, gz, nav.stationary(t))
            next_imu += 0.02
        if t >= next_ctrl:
            n = len(nav.arrivals)
            cmd = nav.step(t, heading.theta)
            if len(nav.arrivals) > n:
                true_at.append((robot.x, robot.y, robot.th))
            next_ctrl += 0.05
    return nav, true_at


SIM_PROFILE = FloorProfile(floor='sim', track_eff=0.4)


def test_load_waypoints():
    if os.path.exists(WAYPOINTS):
        assert load_waypoints(WAYPOINTS) == pytest.approx(HW1_WPS)


@pytest.mark.parametrize('mode', ['gyro', 'model'])
@pytest.mark.parametrize('coast', [0.0, 0.3])  # uncalibrated / roughly calibrated
def test_full_path_noiseless(mode, coast):
    profile = FloorProfile(floor='sim', track_eff=0.4, rot_coast_ccw=coast, rot_coast_cw=coast)
    nav, true_at = run(HW1_WPS, profile, SimRobot(noise=0.0), NavParams(drive_heading=mode))
    assert nav.done and len(true_at) == len(HW1_WPS)
    for (gx, gy, gth), (x, y, th) in zip(HW1_WPS, true_at):
        assert math.hypot(gx - x, gy - y) < 0.05
        assert abs(math.degrees(wrap_angle(gth - th))) < 5.0


@pytest.mark.parametrize('seed', range(3))
def test_full_path_completes_with_real_noise(seed):
    # accuracy is limited by gyro noise (see spec); only check the state machine finishes
    nav, true_at = run(HW1_WPS, SIM_PROFILE, SimRobot(seed=seed, noise=13.0))
    print('\n'.join(nav.events))
    assert nav.done and len(true_at) == len(HW1_WPS)


def test_stops_and_holds_after_done():
    nav, _ = run([(0, 0, 0), (0.5, 0, 0)], SIM_PROFILE, SimRobot(noise=0, bias=0))
    assert nav.done
    assert nav.step(nav._t + 0.05, nav.theta) == (0.0, 0.0)
