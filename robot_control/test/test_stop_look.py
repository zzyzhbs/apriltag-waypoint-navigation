import math
import os

import numpy as np
import pytest
import yaml
from sim_rover import SimNoise, SimRover

from robot_control.ekf import noise_from_config
from robot_control.landmark_map import Landmark, LandmarkMap, load_map
from robot_control.ol_model import Calibration
from robot_control.ol_planner import Step, load_waypoints
from robot_control.se2 import Pose2, wrap_angle
from robot_control.stop_look import (
    Move, NavigationHalt, Navigator, NavLog, align_tol, below_min_step, drive_length,
    move_to_primitive, next_move, params_from_config, search_move, split_spin,
    step_motion, to_step,
)

HERE = os.path.dirname(__file__)
CONFIG = os.path.join(HERE, '..', 'config', 'hw2')
with open(os.path.join(CONFIG, 'nav.yaml')) as f:
    NAV = yaml.safe_load(f)
P = params_from_config(NAV)
NOISE = noise_from_config(NAV['ekf'])
MAP = load_map(os.path.join(CONFIG, 'landmarks.yaml'))
WPS = load_waypoints(os.path.join(HERE, '..', '..', 'HW2', 'waypoints.txt'))
GOALS = [Pose2(*w) for w in WPS[1:]]


def make_calib(tmp_path):
    """Coarse levels like study_wood2, plus fine levels (min 1.5 cm / 3 deg)."""
    c = Calibration.load(tmp_path / 'sim.yaml', 'sim', create=True)
    for T in (2.0, 3.0, 4.0):
        c.drive_trials().append({'cmd': 0.3, 'T': T, 'x': 0.396 * T - 0.024, 'y': 0.0})
    for T in (0.2, 0.4, 0.6):
        c.drive_trials().append({'cmd': 0.2, 'T': T, 'x': 0.1 * T - 0.005, 'y': 0.0})
    for d in ('ccw', 'cw'):
        for T in (0.4, 0.8, 1.6):
            c.spin_trials(d).append({'cmd': 0.35, 'T': T, 'angle': 125.0 * T - 6.0})
        for T in (0.2, 0.4, 0.6):
            c.spin_trials(d).append({'cmd': 0.25, 'T': T, 'angle': 20.0 * T - 1.0})
    return c


# ---------- planning ----------

def test_fine_spin_only_for_small_turns(tmp_path):
    c = make_calib(tmp_path)
    # Medium/large turns must not extrapolate the fine 0.25 fit.
    assert to_step(Move('spin', math.radians(30.0)), c, P).cmd == 0.35
    assert to_step(Move('spin', math.radians(39.38)), c, P).cmd == 0.35
    # The fine level is still available for the last small correction.
    assert to_step(Move('spin', math.radians(10.0)), c, P).cmd == 0.25


def test_next_move_turn_drive_turn():
    goal = Pose2(1.0, 2.0, math.pi)
    m = next_move(Pose2(1.0, 0.0, 0.0), goal, P)
    assert m.kind == 'spin' and m.value == pytest.approx(math.pi / 2)
    m = next_move(Pose2(1.0, 0.0, math.radians(91)), goal, P)
    assert m.kind == 'drive' and m.value == pytest.approx(P.max_drive)
    m = next_move(Pose2(1.0, 1.98, math.pi / 2), goal, P)
    assert m.kind == 'spin' and m.value == pytest.approx(math.pi / 2)
    assert next_move(Pose2(1.01, 1.99, math.radians(178)), goal, P) is None


def coarse_feasible(tmp_path):
    """Only the HW1 levels: min drive ~5.5 cm, min spin ~24 deg."""
    c = Calibration.load(tmp_path / 'coarse.yaml', 'coarse', create=True)
    for T in (2.0, 3.0, 4.0):
        c.drive_trials().append({'cmd': 0.3, 'T': T, 'x': 0.396 * T - 0.024, 'y': 0.0})
    for d in ('ccw', 'cw'):
        for T in (0.4, 0.8, 1.6):
            c.spin_trials(d).append({'cmd': 0.35, 'T': T, 'angle': 125.0 * T - 6.0})
    return lambda m: not below_min_step(m, c, P)


def test_next_move_drives_when_align_turn_too_small(tmp_path):
    # run_20261023_071302: 2 m short of WP2, 3.7 deg off -> must drive, not stop
    ok = coarse_feasible(tmp_path)
    m = next_move(Pose2(0.966, 0.015, math.radians(92.7)), Pose2(1.0, 2.0, math.pi), P, ok)
    assert m.kind == 'drive' and m.value == pytest.approx(P.max_drive)


def test_next_move_skips_to_heading_when_drive_too_small(tmp_path):
    # 3.4 cm short of WP1: no turn towards it; fix the final heading instead
    ok = coarse_feasible(tmp_path)
    goal = Pose2(1.0, 0.0, 0.0)
    m = next_move(Pose2(0.969, 0.015, math.radians(-26.1)), goal, P, ok)
    assert m.kind == 'spin' and m.value == pytest.approx(math.radians(26.1))
    assert next_move(Pose2(0.969, 0.015, math.radians(-0.4)), goal, P, ok) is None


def test_drive_length_undershoots_and_caps():
    assert drive_length(0.2, P) == pytest.approx(0.2)               # short: as is
    assert drive_length(0.8, P) == pytest.approx(0.8 - 0.03 * 0.8 - 0.08)
    assert drive_length(2.0, P) == pytest.approx(P.max_drive)


def test_align_tol_clamped():
    assert align_tol(5.0, P) == pytest.approx(P.align_min)
    assert align_tol(0.04, P) == pytest.approx(P.align_max)


def test_search_move_from_wp2():
    # at WP2 facing 180: tag 1 is 0.5 m away (needs -90 deg); tag 2 is too far, tag 0 too oblique
    m, tag_id = search_move(Pose2(1.0, 2.0, math.pi), MAP.landmarks.values(), P)
    assert tag_id == 1
    assert m.kind == 'spin' and m.value == pytest.approx(-math.pi / 2)
    assert search_move(Pose2(1.0, 2.0, math.pi), MAP.landmarks.values(), P, {1}) == (None, None)


def test_search_move_none_when_no_tag_reachable():
    far = [Landmark(9, 10.0, 0.0, math.pi)]
    assert search_move(Pose2(0, 0, 0), far, P) == (None, None)


def test_split_spin_keeps_tag_in_view_at_wp2():
    # final turn at WP2 (90 -> 180 deg) would leave no tag in view: stop where tag 1
    # is at the view edge, i.e. after view_half
    m = split_spin(Pose2(1.0, 2.0, math.pi / 2), Move('spin', math.pi / 2),
                   MAP.landmarks.values(), P)
    assert m.value == pytest.approx(P.view_half)
    # the remainder is not split again (tag 1 is already at the edge)
    rest = Move('spin', math.pi / 2 - P.view_half)
    pose = Pose2(1.0, 2.0, math.pi / 2 + P.view_half)
    assert split_spin(pose, rest, MAP.landmarks.values(), P) == rest


def test_split_spin_not_needed_when_tag_ahead():
    # WP1 -> leg 2: after turning to 90 deg tag 1 is straight ahead
    m = Move('spin', math.pi / 2)
    assert split_spin(Pose2(1.0, 0.0, 0.0), m, MAP.landmarks.values(), P) == m
    assert split_spin(Pose2(1.0, 0.0, 0.0), Move('drive', 1.0), [], P) == Move('drive', 1.0)


def test_move_step_conversions():
    assert move_to_primitive(Move('spin', -math.pi / 2)) == ('spin', 'cw', pytest.approx(90.0))
    assert step_motion(Step('spin', 'cw', 90.0)).value == pytest.approx(-math.pi / 2)
    assert step_motion(Step('drive', 'fwd', 0.4)) == Move('drive', 0.4)


def test_below_min_step(tmp_path):
    c = make_calib(tmp_path)
    assert below_min_step(Move('spin', math.radians(2.5)), c, P)
    assert not below_min_step(Move('spin', math.radians(4.0)), c, P)
    assert below_min_step(Move('drive', 0.01), c, P)


# ---------- closed loop with the simulated rover ----------

class RecordLog(NavLog):
    def __init__(self):
        self.looks, self.steps, self.waypoints = [], [], []

    def on_look(self, goal_index, records, pose, P):
        self.looks.append((goal_index, records))

    def on_step(self, goal_index, step, before, after, P, timing=None):
        self.steps.append((goal_index, step))

    def on_waypoint(self, result):
        self.waypoints.append(result)


def run_sim(tmp_path, seed, landmark_map=MAP, noise=None):
    rover = SimRover(landmark_map, Pose2(0.0, 0.0, 0.0), np.random.default_rng(seed), noise)
    log = RecordLog()
    nav = Navigator(landmark_map, make_calib(tmp_path), P, NOISE, Pose2(0.0, 0.0, 0.0),
                    rover.look, rover.execute, log)
    results = []
    for i, goal in enumerate(GOALS, 1):
        results.append(nav.run_goal(i, goal))
        results[-1].true = rover.true
    return results, rover, log


@pytest.mark.parametrize('seed', range(8))
def test_sim_reaches_all_waypoints(tmp_path, seed):
    results, rover, log = run_sim(tmp_path, seed)
    for r in results:
        assert r.reached, r.note
        assert r.tag_based
        assert r.iters < P.max_iters
        err = math.hypot(r.true.x - r.goal.x, r.true.y - r.goal.y)
        derr = abs(wrap_angle(r.true.theta - r.goal.theta))
        assert err < 0.04, (r.index, err)
        # WP2's last turn (to 180 deg) ends with no tag in view, so it is not corrected
        assert derr < math.radians(8.0 if r.index == 2 else 4.0), (r.index, math.degrees(derr))


def test_sim_wp2_heading_uses_prediction(tmp_path):
    # WP2 faces 180 deg where no tag is visible: its last look sees nothing,
    # but the waypoint still counts as tag-based (tag 1 seen before the final turn)
    results, _, log = run_sim(tmp_path, 0)
    wp2_looks = [recs for i, recs in log.looks if i == 2]
    assert wp2_looks[-1] == []
    assert results[1].tag_based


def test_sim_searches_when_no_tag_at_waypoint(tmp_path):
    # only tag 1 exists; at WP1 (facing 0) it is 90 deg to the left -> search turn
    only_tag1 = LandmarkMap(0.166, {1: MAP.get(1)})
    rover = SimRover(only_tag1, Pose2(0.0, 0.0, 0.0), np.random.default_rng(1))
    log = RecordLog()
    nav = Navigator(only_tag1, make_calib(tmp_path), P, NOISE, Pose2(0.0, 0.0, 0.0),
                    rover.look, rover.execute, log)
    r = nav.run_goal(1, GOALS[0])
    spins = [s for _, s in log.steps if s.kind == 'spin']
    assert spins[0].dir == 'ccw' and spins[0].amount > 80     # turned to tag 1
    assert r.reached and r.tag_based and r.tags_used == [1]


def test_sim_prediction_only_when_no_tag_reachable(tmp_path):
    far_tag = LandmarkMap(0.166, {5: Landmark(5, 5.0, 5.0, math.pi)})
    rover = SimRover(far_tag, Pose2(0.0, 0.0, 0.0), np.random.default_rng(1))
    nav = Navigator(far_tag, make_calib(tmp_path), P, NOISE, Pose2(0.0, 0.0, 0.0),
                    rover.look, rover.execute)
    r = nav.run_goal(1, GOALS[0])
    assert r.reached and not r.tag_based
    assert 'prediction only' in r.note


def test_each_tag_searched_once(tmp_path):
    # tags exist in the map but the camera never sees them
    rover = SimRover(MAP, Pose2(0.0, 0.0, 0.0), np.random.default_rng(0))
    log = RecordLog()
    nav = Navigator(MAP, make_calib(tmp_path), P, NOISE, Pose2(0.0, 0.0, 0.0),
                    lambda: ({}, {}), rover.execute, log)
    r = nav.run_goal(3, Pose2(0.0, 0.0, 0.0))     # already there: only searching is left
    big_turns = [s for _, s in log.steps if s.kind == 'spin' and s.amount > 30]
    assert r.reached and not r.tag_based
    assert len(big_turns) <= 2 * P.max_searches     # out and back, no repeats


def test_halt_when_map_is_wrong(tmp_path):
    shifted = LandmarkMap(0.166, {i: Landmark(i, lm.x + 0.5, lm.y, lm.yaw)
                                  for i, lm in MAP.landmarks.items()})
    rover = SimRover(MAP, Pose2(0.0, 0.0, 0.0), np.random.default_rng(0))
    nav = Navigator(shifted, make_calib(tmp_path), P, NOISE, Pose2(0.0, 0.0, 0.0),
                    rover.look, rover.execute)
    with pytest.raises(NavigationHalt):
        nav.run(GOALS)


def test_sim_perfect_rover_needs_few_steps(tmp_path):
    exact = SimNoise(drive_scale=1.0, drive_sigma=0.0, drift=0.0, spin_scale=1.0,
                     spin_sigma=0.0, obs_xy=0.0, obs_psi=0.0)
    results, rover, log = run_sim(tmp_path, 0, noise=exact)
    kinds = [(i, s.kind) for i, s in log.steps]
    # long drives: capped at max_drive and stopped short, then a short fine drive;
    # WP2's final turn is split in two to look at tag 1 in between
    assert kinds == [(1, 'drive'), (1, 'drive'),
                     (2, 'spin'), (2, 'drive'), (2, 'drive'), (2, 'drive'),
                     (2, 'spin'), (2, 'spin'),
                     (3, 'spin'), (3, 'drive'), (3, 'drive'), (3, 'drive'), (3, 'spin')]
    assert all(s.dir in ('fwd', 'ccw') for _, s in log.steps)
    assert sum(s.amount for _, s in log.steps if s.kind == 'drive') == pytest.approx(
        1 + 2 + math.sqrt(5), abs=0.01)
    assert sum(s.amount for _, s in log.steps if s.kind == 'spin') == pytest.approx(360, abs=1)
