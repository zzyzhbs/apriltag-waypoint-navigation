import importlib.util
import os

import pytest
import yaml
from test_nav_log import write_sim_run

PLOT_RUN = os.path.join(os.path.dirname(__file__), '..', '..', 'HW2', 'plot_run.py')
spec = importlib.util.spec_from_file_location('plot_run', PLOT_RUN)
plot_run = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plot_run)


def test_wrap_and_pose_error():
    assert plot_run.wrap_deg(190) == pytest.approx(-170)
    d, dth = plot_run.pose_error((1.03, 0.04, 179.0), (1.0, 0.0, -179.0))
    assert d == pytest.approx(0.05) and dth == pytest.approx(-2.0)


def test_total_motion_matches_steps(tmp_path):
    run_dir, _, rover, _ = write_sim_run(tmp_path)
    run = plot_run.load_run(run_dir)
    drive, spin = plot_run.total_motion(run['steps'])
    assert drive == pytest.approx(sum(s.amount for s in rover.steps if s.kind == 'drive'),
                                  abs=1e-3)
    assert spin == pytest.approx(sum(s.amount for s in rover.steps if s.kind == 'spin'),
                                 abs=1e-2)
    # commanded totals: the sim rover drives 3 % too far, so less than 5.236 m is commanded
    assert 4.9 < drive < 5.6 and spin >= 359
    sm = plot_run.smoothness(run['steps'], run['trajectory'])
    assert sm['primitives'] == len(rover.steps)
    assert 0 < sm['moving_time'] < sm['run_time']


def test_waypoint_errors_with_truth(tmp_path):
    run_dir, results, rover, _ = write_sim_run(tmp_path)
    true = rover.true      # where the simulated robot really ended (WP3)
    truth = {3: {'wp': 3, 'x': true.x, 'y': true.y, 'theta_deg': 0.0}}
    rows = plot_run.waypoint_rows(plot_run.load_run(run_dir)['waypoints'], truth)
    assert [r['wp'] for r in rows] == [1, 2, 3]
    assert 'true' not in rows[0]
    loc_m, _ = rows[2]['localization']
    arr_m, _ = rows[2]['arrival']
    est = results[2].pose
    assert loc_m == pytest.approx(((est.x - true.x) ** 2 + (est.y - true.y) ** 2) ** 0.5,
                                  abs=1e-3)
    assert arr_m == pytest.approx((true.x ** 2 + true.y ** 2) ** 0.5, abs=1e-3)


def test_outputs_written(tmp_path):
    run_dir, _, _, _ = write_sim_run(tmp_path)
    with open(os.path.join(run_dir, 'truth.yaml'), 'w') as f:
        yaml.safe_dump({'waypoints': [{'wp': 1, 'x': 1.01, 'y': 0.0, 'theta_deg': 1.0}]}, f)
    run = plot_run.load_run(run_dir)
    truth = plot_run.load_truth(os.path.join(run_dir, 'truth.yaml'))
    png = os.path.join(run_dir, 'trajectory.png')
    plot_run.plot_trajectory(run, truth, png)
    md = plot_run.summary_markdown(run_dir, run, truth)
    assert os.path.getsize(png) > 10000
    assert 'localization error' in md and '| 1 |' in md
    assert 'driven: **' in md
