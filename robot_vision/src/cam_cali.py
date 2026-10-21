#!/usr/bin/env python3
"""Chessboard camera calibration for robot_vision_camera.

python3 cam_cali.py capture   --cols 9 --rows 6 --out-dir ~/cali_imgs
python3 cam_cali.py calibrate --images ~/cali_imgs --cols 9 --rows 6 --square 0.025
python3 cam_cali.py verify    --images ~/cali_imgs --cols 9 --rows 6 --square 0.025

capture needs rclpy and the camera node (launched with image_rectify:=false).
calibrate / verify only need OpenCV + numpy.
"""
import argparse
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

DEFAULT_YAML = Path(__file__).resolve().parent.parent / 'config' / 'camera_parameter.yaml'
IMG_EXTS = ('.png', '.jpg', '.jpeg')
MIN_IMAGES = 10
NOVELTY = 0.2
STILL_PX_PER_S = 20.0
PROCESS_DT = 0.2
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)


@dataclass
class BoardSpec:
    cols: int
    rows: int
    square: float = 0.025

    @property
    def pattern(self):
        return (self.cols, self.rows)

    def object_points(self):
        grid = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        objp = np.zeros((self.cols * self.rows, 3), np.float32)
        objp[:, :2] = grid * self.square
        return objp


@dataclass
class CalibResult:
    K: np.ndarray
    D: np.ndarray
    newK: np.ndarray
    size: tuple
    rms: float
    model: str
    per_image: dict = field(default_factory=dict)
    rejected: list = field(default_factory=list)


# ---------- detection ----------

def to_gray(img):
    return img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def find_corners(gray, board, fast=False):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    if fast:
        flags |= cv2.CALIB_CB_FAST_CHECK
    ok, corners = cv2.findChessboardCorners(gray, board.pattern, flags=flags)
    if not ok:
        return None
    # Subpix window scales with corner spacing (same idea as ROS camera_calibration)
    pts = corners.reshape(-1, 2)
    spacing = np.linalg.norm(np.diff(pts.reshape(board.rows, board.cols, 2), axis=1), axis=2)
    r = int(max(2, min(11, math.ceil(spacing.min() * 0.5))))
    return cv2.cornerSubPix(gray, corners, (r, r), (-1, -1), SUBPIX_CRITERIA)


def outer_corners(corners, board):
    pts = corners.reshape(board.rows, board.cols, 2)
    return pts[0, 0], pts[0, -1], pts[-1, -1], pts[-1, 0]


def view_metrics(corners, size, board):
    """(x, y, size, skew), each in [0, 1]."""
    w, h = size
    ul, ur, dr, dl = outer_corners(corners, board)
    quad = np.array([ul, ur, dr, dl], np.float32)
    area = cv2.contourArea(quad)
    border = math.sqrt(area)
    mean = corners.reshape(-1, 2).mean(axis=0)
    px = min(1.0, max(0.0, (mean[0] - border / 2) / max(1.0, w - border)))
    py = min(1.0, max(0.0, (mean[1] - border / 2) / max(1.0, h - border)))
    p_size = math.sqrt(area / (w * h))
    a, b = ul - ur, dr - ur
    ang = math.acos(np.clip(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)), -1, 1))
    skew = min(1.0, 2.0 * abs(math.pi / 2 - ang))
    return np.array([px, py, p_size, skew])


def novelty(m, saved):
    """L1 distance to the closest saved view (inf if none)."""
    return min((float(np.abs(m - s).sum()) for s in saved), default=math.inf)


def is_novel(m, saved, thresh=NOVELTY):
    return novelty(m, saved) > thresh


def corner_speed(c, prev, dt):
    """Mean corner motion in px/s; tolerant to OpenCV flipping the corner order."""
    d = min(np.linalg.norm(c - prev, axis=2).mean(),
            np.linalg.norm(c[::-1] - prev, axis=2).mean())
    return d / max(dt, 1e-3)


def coverage(saved):
    if not saved:
        return 'no views yet'
    a = np.array(saved)
    names = ('x', 'y', 'size', 'skew')
    return '  '.join(f'{n} [{lo:.2f}, {hi:.2f}]' for n, lo, hi in zip(names, a.min(0), a.max(0)))


def list_images(d):
    return sorted(p for p in Path(d).iterdir() if p.is_file() and p.suffix.lower() in IMG_EXTS)


def load_corners(paths, board):
    """Detect corners in each image. Returns (names, corners, size)."""
    names, pts, size = [], [], None
    for p in paths:
        img = cv2.imread(str(p))
        if img is None:
            print(f'  skip {p.name}: unreadable')
            continue
        s = (img.shape[1], img.shape[0])
        if size is None:
            size = s
        elif s != size:
            raise SystemExit(f'{p.name} is {s}, expected {size}; all images must be the same size')
        c = find_corners(to_gray(img), board)
        if c is None:
            print(f'  skip {p.name}: board not found')
            continue
        names.append(p.name)
        pts.append(c)
    return names, pts, size


# ---------- calibration ----------

def rms_of(img_pts, proj):
    err = np.linalg.norm(img_pts.reshape(-1, 2) - proj.reshape(-1, 2), axis=1)
    return float(np.sqrt(np.mean(err ** 2)))


def _calib_once(img_pts, board, size, flags):
    objp = board.object_points()
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        [objp] * len(img_pts), img_pts, size, None, None, flags=flags)
    per = [rms_of(c, cv2.projectPoints(objp, r, t, K, D)[0])
           for c, r, t in zip(img_pts, rvecs, tvecs)]
    # OpenCV may return 14 coeffs; the extra ones are 0 unless their flags are set
    n = 8 if flags & cv2.CALIB_RATIONAL_MODEL else 5
    return rms, K, D.reshape(1, -1)[:, :n], per


def calibrate_points(img_pts, board, size, model='plumb_bob', names=None):
    names = names or [f'view_{i:03d}' for i in range(len(img_pts))]
    if len(img_pts) < MIN_IMAGES:
        raise SystemExit(f'only {len(img_pts)} usable images, need at least {MIN_IMAGES}')
    flags = cv2.CALIB_RATIONAL_MODEL if model == 'rational' else 0
    rms, K, D, per = _calib_once(img_pts, board, size, flags)

    # One round of outlier rejection
    thresh = max(2.0 * float(np.median(per)), 1.0)
    keep = [i for i, e in enumerate(per) if e <= thresh]
    rejected = [(names[i], per[i]) for i in range(len(per)) if per[i] > thresh]
    if rejected and len(keep) >= MIN_IMAGES:
        img_pts = [img_pts[i] for i in keep]
        names = [names[i] for i in keep]
        rms, K, D, per = _calib_once(img_pts, board, size, flags)
    else:
        rejected = []

    newK = optimal_new_k(K, D, size)
    return CalibResult(K=K, D=D, newK=newK, size=tuple(size), rms=float(rms), model=model,
                       per_image=dict(zip(names, per)), rejected=rejected)


def calibrate(img_paths, board, model='plumb_bob'):
    names, pts, size = load_corners(img_paths, board)
    return calibrate_points(pts, board, size, model, names)


def optimal_new_k(K, D, size):
    # Same call as robot_vision_camera.cpp (alpha = 0)
    return cv2.getOptimalNewCameraMatrix(K, D, size, 0.0, size)[0]


def undistort_maps(K, D, size):
    newK = optimal_new_k(K, D, size)
    map1, map2 = cv2.initUndistortRectifyMap(K, D, None, newK, size, cv2.CV_16SC2)
    return map1, map2, newK


def fov_deg(K, size):
    w, h = size
    return (math.degrees(2 * math.atan(w / (2 * K[0, 0]))),
            math.degrees(2 * math.atan(h / (2 * K[1, 1]))))


# ---------- YAML (cv::FileStorage format, read by the C++ node) ----------

def write_yaml(path, result, name='camera'):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    fs.write('image_width', int(result.size[0]))
    fs.write('image_height', int(result.size[1]))
    fs.write('camera_name', name)
    fs.write('camera_matrix', result.K.astype(np.float64))
    fs.write('distortion_model',
             'rational_polynomial' if result.model == 'rational' else 'plumb_bob')
    fs.write('distortion_coefficients', result.D.reshape(1, -1).astype(np.float64))
    fs.write('rectification_matrix', np.eye(3))
    fs.write('projection_matrix', np.hstack([result.newK, np.zeros((3, 1))]))
    fs.write('reprojection_rms', float(result.rms))
    fs.release()


def read_yaml(path):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise SystemExit(f'cannot open {path}')
    node = fs.getNode('reprojection_rms')
    out = {
        'size': (int(fs.getNode('image_width').real()), int(fs.getNode('image_height').real())),
        'K': fs.getNode('camera_matrix').mat(),
        'D': fs.getNode('distortion_coefficients').mat().reshape(1, -1),
        'P': fs.getNode('projection_matrix').mat(),
        'model': fs.getNode('distortion_model').string(),
        'rms': None if node.empty() else node.real(),
    }
    fs.release()
    return out


# ---------- verification ----------

def _line_dist(pts):
    c = pts - pts.mean(axis=0)
    normal = np.linalg.svd(c)[2][1]
    return np.abs(c @ normal)


def line_straightness(corners, board):
    """Mean distance (px) of corners to best-fit lines along rows and columns."""
    g = corners.reshape(board.rows, board.cols, 2).astype(np.float64)
    d = [_line_dist(g[r]) for r in range(board.rows)]
    d += [_line_dist(g[:, c]) for c in range(board.cols)]
    return float(np.concatenate(d).mean())


def holdout_rms_points(img_pts, board, K, D):
    objp = board.object_points()
    per, sq = [], []
    for c in img_pts:
        _, r, t = cv2.solvePnP(objp, c, K, D)
        proj = cv2.projectPoints(objp, r, t, K, D)[0]
        err = np.linalg.norm(c.reshape(-1, 2) - proj.reshape(-1, 2), axis=1)
        per.append(float(np.sqrt(np.mean(err ** 2))))
        sq.append(err ** 2)
    total = float(np.sqrt(np.mean(np.concatenate(sq)))) if sq else float('nan')
    return total, per


def holdout_rms(img_paths, board, K, D):
    names, pts, _ = load_corners(img_paths, board)
    total, per = holdout_rms_points(pts, board, K, D)
    return total, dict(zip(names, per))


def holdout_verdict(total, calib_rms):
    ratio = total / calib_rms if calib_rms else float('nan')
    if total > 1.0 or ratio > 2.0:
        return ratio, 'FAIL (overfit or too few views): re-shoot, or try plumb_bob'
    if ratio > 1.5:
        return ratio, 'WARN: holdout error noticeably higher than calibration error'
    return ratio, 'PASS'


def rms_verdict(rms):
    if rms < 0.5:
        return 'good'
    if rms <= 1.0:
        return 'usable'
    return 'poor, re-shoot recommended'


# ---------- commands ----------

def format_report(res, out_path):
    hf, vf = fov_deg(res.K, res.size)
    nhf, nvf = fov_deg(res.newK, res.size)
    lines = [
        f'output: {out_path}',
        f'model: {res.model}  size: {res.size[0]}x{res.size[1]}  images: {len(res.per_image)}',
        f'RMS: {res.rms:.4f} px ({rms_verdict(res.rms)})',
        'K:', np.array2string(res.K, precision=4, suppress_small=True),
        'D: ' + np.array2string(res.D.ravel(), precision=6, suppress_small=True),
        'newK (P, used for rectified images):',
        np.array2string(res.newK, precision=4, suppress_small=True),
        f'FOV (K): {hf:.1f} x {vf:.1f} deg   FOV (newK): {nhf:.1f} x {nvf:.1f} deg',
        'rejected: ' + (', '.join(f'{n} ({e:.2f})' for n, e in res.rejected) or 'none'),
        'per-image RMS:',
    ]
    lines += [f'  {n}: {e:.4f}' for n, e in res.per_image.items()]
    return '\n'.join(lines) + '\n'


def cmd_calibrate(args, board):
    img_dir = Path(args.images).expanduser()
    paths = list_images(img_dir)
    print(f'{len(paths)} images in {img_dir}')
    res = calibrate(paths, board, args.model)
    out = Path(args.out).expanduser() if args.out else DEFAULT_YAML
    write_yaml(out, res, args.name)
    report = format_report(res, out)
    (img_dir / 'calib_report.txt').write_text(report)
    print(report)
    print(f'report: {img_dir / "calib_report.txt"}')
    print('next: python3 cam_cali.py verify --images', img_dir)
    return 0


def cmd_verify(args, board):
    img_dir = Path(args.images).expanduser()
    calib = read_yaml(Path(args.calib).expanduser() if args.calib else DEFAULT_YAML)
    K, D, size = calib['K'], calib['D'], calib['size']
    map1, map2, newK = undistort_maps(K, D, size)
    if not np.allclose(newK, calib['P'][:, :3], atol=1e-3):
        print('WARN: projection_matrix != getOptimalNewCameraMatrix(alpha=0); '
              'apriltag distances will be off')

    out_dir = Path(args.out_dir).expanduser() if args.out_dir else img_dir / 'verify'
    out_dir.mkdir(parents=True, exist_ok=True)
    raw_s, rect_s = [], []
    print('straightness (mean px from fitted lines)   raw -> rectified')
    for p in list_images(img_dir):
        img = cv2.imread(str(p))
        if img is None or (img.shape[1], img.shape[0]) != tuple(size):
            print(f'  skip {p.name}: unreadable or size != {size}')
            continue
        rect = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
        pair = cv2.resize(np.hstack([img, rect]), None, fx=0.5, fy=0.5)
        cv2.imwrite(str(out_dir / p.name), pair)
        c0 = find_corners(to_gray(img), board)
        c1 = find_corners(to_gray(rect), board)
        if c0 is None or c1 is None:
            continue
        raw_s.append(line_straightness(c0, board))
        rect_s.append(line_straightness(c1, board))
        print(f'  {p.name}: {raw_s[-1]:.3f} -> {rect_s[-1]:.3f}')
    if rect_s:
        print(f'mean straightness: {np.mean(raw_s):.3f} -> {np.mean(rect_s):.3f} px '
              f'({"ok" if np.mean(rect_s) < 1.0 else "high, check D"})')
    print(f'side-by-side images: {out_dir}')

    lines = []
    hold_dir = Path(args.holdout_dir).expanduser() if args.holdout_dir else img_dir / 'holdout'
    status = 0
    if not hold_dir.is_dir() or not list_images(hold_dir):
        print(f'WARN: no holdout images in {hold_dir}, skipping holdout check')
    else:
        total, per = holdout_rms(list_images(hold_dir), board, K, D)
        if not per:
            print('WARN: board not found in any holdout image')
        else:
            lines.append(f'holdout ({hold_dir}): {len(per)} images')
            lines += [f'  {n}: {e:.4f}' for n, e in per.items()]
            if calib['rms']:
                ratio, verdict = holdout_verdict(total, calib['rms'])
                lines.append(f'holdout RMS: {total:.4f} px  calib RMS: {calib["rms"]:.4f} px  '
                             f'ratio: {ratio:.2f}  -> {verdict}')
                status = 0 if not verdict.startswith('FAIL') else 1
            else:
                lines.append(f'holdout RMS: {total:.4f} px (no reprojection_rms in YAML)')
    if rect_s:
        lines.insert(0, f'straightness: {np.mean(raw_s):.3f} -> {np.mean(rect_s):.3f} px')
    if lines:
        text = '\n'.join(['', '# verify ' + time.strftime('%Y-%m-%d %H:%M:%S')] + lines) + '\n'
        print(text)
        with open(img_dir / 'calib_report.txt', 'a') as f:
            f.write(text)
    return status


def msg_to_bgr(msg):
    ch = 1 if msg.encoding in ('mono8', '8UC1') else 3
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    return a[:, :msg.width * ch].reshape(msg.height, msg.width, ch).squeeze()


def cmd_capture(args, board):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image

    out_dir = Path(args.out_dir or f'cali_{time.strftime("%Y%m%d_%H%M%S")}').expanduser()
    hold_dir = out_dir / 'holdout'
    out_dir.mkdir(parents=True, exist_ok=True)

    class Capture(Node):
        def __init__(self):
            super().__init__('cam_cali')
            self.sub = self.create_subscription(Image, args.topic, self.on_image,
                                                qos_profile_sensor_data)
            self.dbg = self.create_publisher(CompressedImage, args.debug_topic, 1)
            self.last_t = 0.0
            self.last_save = 0.0
            self.prev = None
            self.prev_stamp = 0.0
            self.last_status = ''
            self.saved = {'calib': [], 'holdout': []}
            self.phase = 'calib' if args.target > 0 else 'holdout'
            self.done = False
            self.got_image = False

        def on_image(self, msg):
            now = time.monotonic()
            if self.done or now - self.last_t < PROCESS_DT:
                return
            self.last_t = now
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if not self.got_image:
                self.got_image = True
                print(f'receiving {msg.width}x{msg.height} {msg.encoding}')
            img = msg_to_bgr(msg)
            gray = to_gray(img)
            size = (gray.shape[1], gray.shape[0])
            c = find_corners(gray, board, fast=True)
            status = 'no board'
            if c is not None:
                speed = (math.inf if self.prev is None
                         else corner_speed(c, self.prev, stamp - self.prev_stamp))
                m = view_metrics(c, size, board)
                saved = self.saved[self.phase]
                dist = novelty(m, saved)
                if speed > args.still:
                    status = 'hold still' + ('' if math.isinf(speed) else f' ({speed:.0f} px/s)')
                elif dist <= args.novelty:
                    status = f'move to a new view (dist {dist:.2f} <= {args.novelty})'
                elif stamp - self.last_save < args.period:
                    status = 'wait'
                else:
                    self.last_save = stamp
                    d = out_dir if self.phase == 'calib' else hold_dir
                    d.mkdir(exist_ok=True)
                    name = f'img_{len(saved):03d}.png'
                    cv2.imwrite(str(d / name), img)
                    saved.append(m)
                    status = f'saved {self.phase}/{name}'
                    self.report()
                    self.advance()
            self.prev, self.prev_stamp = c, stamp
            self.publish_debug(img, c, status)
            key = status.split(' (')[0]
            if key != self.last_status and not key.startswith('saved'):
                print(f'  {status}')
            self.last_status = key

        def report(self):
            n = len(self.saved[self.phase])
            goal = args.target if self.phase == 'calib' else args.holdout
            print(f'[{self.phase} {n}/{goal}] {coverage(self.saved[self.phase])}')

        def advance(self):
            if self.phase == 'calib' and len(self.saved['calib']) >= args.target:
                if args.holdout > 0:
                    self.phase = 'holdout'
                    print(f'\n=== calibration set done. Now capture {args.holdout} HOLDOUT '
                          'images (cover center, edges, corners) ===\n')
                else:
                    self.done = True
            elif self.phase == 'holdout' and len(self.saved['holdout']) >= args.holdout:
                self.done = True

        def publish_debug(self, img, c, status):
            vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img.copy()
            if c is not None:
                cv2.drawChessboardCorners(vis, board.pattern, c, True)
            n = len(self.saved[self.phase])
            cv2.putText(vis, f'{self.phase} {n}: {status}', (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
            vis = cv2.resize(vis, None, fx=0.5, fy=0.5)
            ok, buf = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                m = CompressedImage()
                m.header.stamp = self.get_clock().now().to_msg()
                m.format = 'jpeg'
                m.data = buf.tobytes()
                self.dbg.publish(m)

    rclpy.init()
    node = Capture()
    print(f'listening on {args.topic}, saving to {out_dir}; debug view on {args.debug_topic}')
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        print('\ncalib:  ', len(node.saved['calib']), 'images,', coverage(node.saved['calib']))
        print('holdout:', len(node.saved['holdout']), 'images,', coverage(node.saved['holdout']))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f'next: python3 cam_cali.py calibrate --images {out_dir} '
          f'--cols {board.cols} --rows {board.rows} --square <meters>')
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--cols', type=int, default=9, help='inner corners per row')
    common.add_argument('--rows', type=int, default=6, help='inner corners per column')
    common.add_argument('--square', type=float, default=0.025, help='square size (m)')
    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('capture', parents=[common])
    p.add_argument('--out-dir')
    p.add_argument('--topic', default='/camera/image_raw')
    p.add_argument('--target', type=int, default=40)
    p.add_argument('--holdout', type=int, default=8)
    p.add_argument('--period', type=float, default=1.0, help='min seconds between saves')
    p.add_argument('--still', type=float, default=STILL_PX_PER_S,
                   help='max mean corner speed (px/s) to count as still')
    p.add_argument('--novelty', type=float, default=NOVELTY,
                   help='min L1 distance to every saved view')
    p.add_argument('--debug-topic', default='/cam_cali/debug')

    p = sub.add_parser('calibrate', parents=[common])
    p.add_argument('--images', required=True)
    p.add_argument('--model', choices=('plumb_bob', 'rational'), default='plumb_bob')
    p.add_argument('--out', help=f'default {DEFAULT_YAML}')
    p.add_argument('--name', default='camera')

    p = sub.add_parser('verify', parents=[common])
    p.add_argument('--images', required=True)
    p.add_argument('--holdout-dir', help='default <images>/holdout')
    p.add_argument('--calib', help=f'default {DEFAULT_YAML}')
    p.add_argument('--out-dir', help='default <images>/verify')

    args = ap.parse_args(argv)
    board = BoardSpec(args.cols, args.rows, args.square)
    cmd = {'capture': cmd_capture, 'calibrate': cmd_calibrate, 'verify': cmd_verify}[args.cmd]
    return cmd(args, board)


if __name__ == '__main__':
    sys.exit(main())
