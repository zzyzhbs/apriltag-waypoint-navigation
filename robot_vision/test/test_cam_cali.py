import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import cam_cali as cc  # noqa: E402

SIZE = (1280, 720)
K_TRUE = np.array([[490.0, 0, 640.0], [0, 492.0, 357.0], [0, 0, 1]])
D_TRUE = np.array([[-0.30, 0.10, 0.001, -0.001, -0.015]])
BOARD = cc.BoardSpec(9, 6, 0.025)


def random_poses(n, seed=0):
    """Board poses whose projected corners stay inside the image."""
    rng = np.random.default_rng(seed)
    objp = BOARD.object_points()
    poses = []
    while len(poses) < n:
        rvec = np.radians(rng.uniform([-35, -35, -20], [35, 35, 20]))
        z = rng.uniform(0.25, 0.6)
        t = np.array([rng.uniform(-0.6, 0.4) * z, rng.uniform(-0.4, 0.25) * z, z])
        img = cv2.projectPoints(objp, rvec, t, K_TRUE, D_TRUE)[0].reshape(-1, 2)
        if (img.min() > 10 and img[:, 0].max() < SIZE[0] - 10
                and img[:, 1].max() < SIZE[1] - 10):
            poses.append((rvec, t))
    return poses


def project(poses, noise=0.1, seed=1):
    rng = np.random.default_rng(seed)
    objp = BOARD.object_points()
    out = []
    for r, t in poses:
        p = cv2.projectPoints(objp, r, t, K_TRUE, D_TRUE)[0]
        out.append((p + rng.normal(0, noise, p.shape)).astype(np.float32))
    return out


def render(rvec, t, s=80, margin=80, ss=4):
    """Distortion-free image of the board seen from pose (rvec, t) with K_TRUE.

    Rendered at ss x resolution and area-downsampled to avoid aliasing bias.
    """
    w, h = (BOARD.cols + 1) * s + 2 * margin, (BOARD.rows + 1) * s + 2 * margin
    flat = np.full((h, w), 255, np.uint8)
    for j in range(BOARD.rows + 1):
        for i in range(BOARD.cols + 1):
            if (i + j) % 2 == 0:
                x, y = margin + i * s, margin + j * s
                flat[y:y + s, x:x + s] = 0
    k = BOARD.square / s
    o = -(margin + s - 0.5) * k  # square edges sit on pixel boundaries
    S = np.array([[k, 0, o], [0, k, o], [0, 0, 1]])
    R = cv2.Rodrigues(np.asarray(rvec, float))[0]
    up = np.array([[ss, 0, (ss - 1) / 2], [0, ss, (ss - 1) / 2], [0, 0, 1]])
    H = up @ K_TRUE @ np.column_stack([R[:, 0], R[:, 1], t]) @ S
    big = cv2.warpPerspective(flat, H, (SIZE[0] * ss, SIZE[1] * ss),
                              flags=cv2.INTER_LINEAR, borderValue=255)
    return cv2.resize(big, SIZE, interpolation=cv2.INTER_AREA)


def test_synthetic_points_recover_intrinsics():
    res = cc.calibrate_points(project(random_poses(20)), BOARD, SIZE)
    for got, want in zip((res.K[0, 0], res.K[1, 1], res.K[0, 2], res.K[1, 2]),
                         (490.0, 492.0, 640.0, 357.0)):
        assert got == pytest.approx(want, rel=0.01)
    assert res.rms < 0.3
    assert res.D.shape == (1, 5)


def test_rational_model_has_8_coeffs():
    res = cc.calibrate_points(project(random_poses(20)), BOARD, SIZE, model='rational')
    assert res.D.shape == (1, 8)


def test_too_few_images():
    with pytest.raises(SystemExit):
        cc.calibrate_points(project(random_poses(5)), BOARD, SIZE)


def test_outlier_rejected():
    pts = project(random_poses(20))
    pts[3] = pts[3] + np.random.default_rng(2).normal(0, 4.0, pts[3].shape).astype(np.float32)
    names = [f'img_{i:03d}.png' for i in range(20)]
    res = cc.calibrate_points(pts, BOARD, SIZE, names=names)
    assert [n for n, _ in res.rejected] == ['img_003.png']
    assert 'img_003.png' not in res.per_image


def test_find_corners_on_rendered_images():
    for rvec, t in random_poses(4, seed=3):
        c = cc.find_corners(render(rvec, t), BOARD)
        assert c is not None and c.shape == (BOARD.cols * BOARD.rows, 1, 2)
        assert cc.find_corners(render(rvec, t), BOARD, fast=True) is not None
    assert cc.find_corners(np.full((SIZE[1], SIZE[0]), 255, np.uint8), BOARD) is None


def test_yaml_round_trip(tmp_path):
    for model, n in (('plumb_bob', 5), ('rational', 8)):
        res = cc.calibrate_points(project(random_poses(15)), BOARD, SIZE, model=model)
        path = tmp_path / f'{model}.yaml'
        cc.write_yaml(path, res)
        text = path.read_text()
        assert text.startswith('%YAML:1.0')
        assert '!!opencv-matrix' in text
        got = cc.read_yaml(path)
        assert got['size'] == SIZE
        assert np.allclose(got['K'], res.K)
        assert got['D'].shape == (1, n) and np.allclose(got['D'], res.D)
        assert got['P'].shape == (3, 4)
        assert got['rms'] == pytest.approx(res.rms)
        assert got['model'] == ('rational_polynomial' if model == 'rational' else 'plumb_bob')


def test_projection_matches_camera_node(tmp_path):
    res = cc.calibrate_points(project(random_poses(15)), BOARD, SIZE)
    cc.write_yaml(tmp_path / 'c.yaml', res)
    P = cc.read_yaml(tmp_path / 'c.yaml')['P']
    newK = cv2.getOptimalNewCameraMatrix(res.K, res.D, SIZE, 0.0, SIZE)[0]
    assert np.allclose(P[:, :3], newK)
    assert np.allclose(P[:, 3], 0)


def test_is_novel():
    c = project(random_poses(2, seed=4), noise=0)
    m0 = cc.view_metrics(c[0], SIZE, BOARD)
    assert np.all((m0 >= 0) & (m0 <= 1))
    assert cc.is_novel(m0, [])
    assert not cc.is_novel(m0 + 0.01, [m0])
    shifted = c[0] + np.float32([400, 0])
    assert cc.is_novel(cc.view_metrics(shifted, SIZE, BOARD), [m0])


def test_holdout_rms():
    poses = random_poses(25)
    pts = project(poses)
    res = cc.calibrate_points(pts[:20], BOARD, SIZE)
    good, per = cc.holdout_rms_points(pts[20:], BOARD, res.K, res.D)
    assert len(per) == 5 and good < 0.3
    K_bad = res.K.copy()
    K_bad[0, 0] *= 1.05
    bad, _ = cc.holdout_rms_points(pts[20:], BOARD, K_bad, res.D)
    assert bad > 3 * good
    assert cc.holdout_verdict(good, res.rms)[1] == 'PASS'
    assert cc.holdout_verdict(bad, res.rms)[1].startswith('FAIL')


def test_line_straightness():
    flat = project(random_poses(1, seed=5), noise=0)[0]
    objp = BOARD.object_points()
    pinhole = cv2.projectPoints(objp, np.zeros(3), np.array([-0.1, -0.06, 0.4]),
                                K_TRUE, None)[0]
    assert cc.line_straightness(pinhole, BOARD) < 1e-6
    distorted = cv2.projectPoints(objp, np.zeros(3), np.array([-0.1, -0.06, 0.4]),
                                  K_TRUE, D_TRUE)[0]
    assert cc.line_straightness(distorted, BOARD) > cc.line_straightness(pinhole, BOARD)
    assert flat.shape[0] == BOARD.cols * BOARD.rows


def test_cli_end_to_end(tmp_path):
    img_dir = tmp_path / 'imgs'
    (img_dir / 'holdout').mkdir(parents=True)
    poses = random_poses(18, seed=6)
    for i, (r, t) in enumerate(poses[:14]):
        cv2.imwrite(str(img_dir / f'img_{i:03d}.png'), render(r, t))
    for i, (r, t) in enumerate(poses[14:]):
        cv2.imwrite(str(img_dir / 'holdout' / f'img_{i:03d}.png'), render(r, t))
    out = tmp_path / 'camera_parameter.yaml'
    common = ['--images', str(img_dir), '--cols', '9', '--rows', '6', '--square', '0.025']
    assert cc.main(['calibrate', *common, '--out', str(out)]) == 0
    got = cc.read_yaml(out)
    assert got['K'][0, 0] == pytest.approx(490.0, rel=0.02)
    assert got['K'][1, 1] == pytest.approx(492.0, rel=0.02)
    assert (img_dir / 'calib_report.txt').exists()
    assert cc.main(['verify', *common, '--calib', str(out)]) == 0
    assert len(list((img_dir / 'verify').glob('*.png'))) == 14
    assert 'holdout RMS' in (img_dir / 'calib_report.txt').read_text()


def test_multistart_wide_angle_frontal_views():
    # Robot-like case (2026-10-21): strong barrel distortion, near-frontal views.
    # Plain calibrateCamera without a guess lands on fx ~1160 here.
    K = np.array([[643.0, 0, 630], [0, 646, 312], [0, 0, 1]])
    D = np.array([[-0.33, 0.116, 0, 0.001, -0.019]])
    board = cc.BoardSpec(9, 6, 0.028)
    objp = board.object_points()
    rng = np.random.default_rng(0)
    pts = []
    while len(pts) < 30:
        r = np.radians(rng.uniform([-12, -12, -8], [12, 12, 8]))
        z = rng.uniform(0.3, 0.9)
        t = np.array([rng.uniform(-0.9, 0.6) * z, rng.uniform(-0.5, 0.4) * z, z])
        p = cv2.projectPoints(objp, r, t, K, D)[0]
        q = p.reshape(-1, 2)
        if q.min() > 5 and q[:, 0].max() < SIZE[0] - 5 and q[:, 1].max() < SIZE[1] - 5:
            pts.append((p + rng.normal(0, 0.2, p.shape)).astype(np.float32))
    res = cc.calibrate_points(pts, board, SIZE)
    assert res.K[0, 0] == pytest.approx(643.0, rel=0.02)
    assert res.K[1, 1] == pytest.approx(646.0, rel=0.02)
    assert res.rms < 0.4
