import csv
import math
import os

import numpy as np
import pytest
import yaml
from sim_rover import SimRover
from test_stop_look import GOALS, MAP, NOISE, P, make_calib

from robot_control.nav_log import CsvNavLog, TeeLog, TrajectorySampler, interpolate
from robot_control.se2 import Pose2
from robot_control.stop_look import Navigator, NavLog
from robot_control.tag_obs import filter_window, make_obs

LANDMARKS = os.path.join(os.path.dirname(__file__), '..', 'config', 'hw2', 'landmarks.yaml')
LOOK = {'min_count': 3, 'max_spread': 0.02, 'min_dist': 0.25, 'max_dist': 3.0,
        'max_oblique_deg': 60.0}


def read_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def test_interpolate_wraps_angle():
    p = interpolate(Pose2(0, 0, math.radians(170)), Pose2(1, 0, math.radians(-170)), 0.5)
    assert p.x == pytest.approx(0.5)
    assert abs(abs(p.theta) - math.pi) < 1e-9


def test_sampler_hold_and_move():
    rows = []
    s = TrajectorySampler(lambda t, pose, sig, src: rows.append((round(t, 2), pose, src)))
    s.set(0.0, Pose2(0, 0, 0), (0, 0, 0), 'start')
    s.move(1.2, 2.2, Pose2(1, 0, 0), (0, 0, 0))     # drive 1 m during [1.2, 2.2]
    s.hold(3.0)
    times = [r[0] for r in rows]
    assert times == [0.0, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    by_t = {r[0]: r for r in rows[1:]}
    assert by_t[1.0][1].x == 0.0 and by_t[1.0][2] == 'hold'
    assert by_t[1.5][1].x == pytest.approx(0.3) and by_t[1.5][2] == 'predict'
    assert by_t[2.5][1].x == pytest.approx(1.0)


def test_filter_window():
    raw = [make_obs(0, t, 1.0, 0.0, math.pi) for t in range(4)]
    raw += [make_obs(0, 9, 3.5, 0.0, math.pi)]                     # one frame too far
    raw += [make_obs(1, t, 1.0, 0.0, math.radians(100)) for t in range(4)]   # 80 deg oblique
    raw += [make_obs(7, 0, 1.0, 0.0, math.pi)]                     # not in the map
    accepted, rejected = filter_window(raw, {0, 1}, LOOK)
    assert list(accepted) == [0] and accepted[0].count == 4
    assert set(rejected) == {1, 7}
    assert 'oblique' in rejected[1] and rejected[7] == 'not in map'


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def write_sim_run(tmp_path, seed=3):
    """Simulated rover through all waypoints with a fake clock -> (dir, results, rover, t)."""
    clock = FakeClock()
    rover = SimRover(MAP, Pose2(0, 0, 0), np.random.default_rng(seed))

    def look():
        clock.t += 1.0
        return rover.look()

    def execute(step):
        t0 = clock.t
        clock.t += step.T
        rover.execute(step)
        t1 = clock.t
        clock.t += 1.0          # settle
        return t0, t1

    run_dir = str(tmp_path / 'run')
    log = CsvNavLog(run_dir, Pose2(0, 0, 0), (0.01, 0.01, 0.02), clock)
    with open(LANDMARKS) as f:
        log.write_config({'map': yaml.safe_load(f)})
    seen = []

    class Spy(NavLog):
        def on_waypoint(self, result):
            seen.append(result.index)

    nav = Navigator(MAP, make_calib(tmp_path), P, NOISE, Pose2(0, 0, 0), look, execute,
                    TeeLog(log, Spy()))
    results = nav.run(GOALS)
    log.close()
    assert seen == [1, 2, 3]
    return run_dir, results, rover, clock.t - 100.0


def test_csv_log_full_sim_run(tmp_path):
    run_dir, results, rover, t_end = write_sim_run(tmp_path)
    traj = read_csv(os.path.join(run_dir, 'trajectory.csv'))
    steps = read_csv(os.path.join(run_dir, 'steps.csv'))
    obs = read_csv(os.path.join(run_dir, 'observations.csv'))
    wps = read_csv(os.path.join(run_dir, 'waypoints.csv'))
    assert len(steps) == len(rover.steps)
    assert [int(w['wp']) for w in wps] == [1, 2, 3] and all(w['reached'] == '1' for w in wps)
    assert any(o['accepted'] == '1' for o in obs)
    grid = [float(r['t']) for r in traj if r['source'] in ('hold', 'predict')]
    assert grid == pytest.approx([0.5 * k for k in range(len(grid))])
    assert grid[-1] >= t_end - 0.5
    last = traj[-1]
    assert float(last['x']) == pytest.approx(results[-1].pose.x, abs=1e-3)
    assert float(last['y']) == pytest.approx(results[-1].pose.y, abs=1e-3)
