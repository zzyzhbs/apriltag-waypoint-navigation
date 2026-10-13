"""Floor calibration tool. Run with `ros2 run` (needs stdin), motor_control must be running.

ros2 run robot_control calibrate --ros-args -p floor:=carpet -p test:=straight
tests: straight, arc, rotate, deadband, all
diagnostics (not in all): imu_diag  (hand rotation, motors idle)
"""
import csv
import math
import os
import statistics
import threading
import time
from dataclasses import replace
from datetime import datetime

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

from robot_control.heading import GyroHeading
from robot_control.kinematics import (
    DEFAULT_PROFILE_DIR, load_profile, load_trials, quat_to_yaw, rotate_command, save_profile,
    wrap_angle,
)

RATE_HZ = 20.0
STEADY_FROM_S = 1.0  # skip spin-up transient when measuring steady rates
STILL_S = 3.0        # still time before each run (gyro bias via ZUPT)

# test: (default cmd, default duration s); rotate cmd defaults to profile.rot_cmd
DEFAULTS = {
    'straight': (0.3, 4.0),
    'arc': (0.3, 3.0),          # base cmd; differential is ARC_D
    'rotate': (None, 3.0),
    'deadband': (0.15, 1.0),    # start cmd / seconds per step
    'imu_diag': (0.0, 10.0),    # static window
}
ALL_ORDER = ['straight', 'arc', 'rotate', 'deadband']
ARC_D = 0.05
DEADBAND_MAX = 0.35
DEADBAND_RATE = math.radians(20.0)  # gyro noise over ~0.7 s is ~+-6 deg/s


class Abort(Exception):
    pass


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.floor = self.declare_parameter('floor', '').value  # required, no default
        self.test = self.declare_parameter('test', 'all').value
        self.profile_dir = self.declare_parameter('profile_dir', DEFAULT_PROFILE_DIR).value
        self.cmd_override = self.declare_parameter('cmd', -1.0).value
        self.dur_override = self.declare_parameter('duration', -1.0).value
        self.settle_s = self.declare_parameter('settle', 1.0).value

        self.profile = load_profile(self.profile_dir, self.floor, allow_missing=True)
        self.heading = GyroHeading(self.profile.gyro_scale)

        self.pub = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.create_subscription(Imu, 'imu_data', self.imu_cb, 10)

        self.lock = threading.Lock()
        self.commanding = False
        self.stop_time = 0.0
        self.imu_count = 0
        self.board_yaw = 0.0       # unwrapped board yaw (reference only), rad
        self.last_board = None
        self.log = None            # [(t, board_yaw, gz)] while recording

    # ---------- IMU ----------

    def stationary(self):
        return not self.commanding and time.monotonic() - self.stop_time > self.settle_s

    def imu_cb(self, msg):
        q = msg.orientation
        board = quat_to_yaw(q.w, q.x, q.y, q.z)
        gz = msg.angular_velocity.z
        st = msg.header.stamp
        t = st.sec + st.nanosec * 1e-9 or time.monotonic()
        still = self.stationary()
        with self.lock:
            if self.last_board is not None:
                self.board_yaw += wrap_angle(board - self.last_board)
            self.last_board = board
            self.heading.update(t, gz, still)
            self.imu_count += 1
            if self.log is not None:
                self.log.append((t, self.board_yaw, gz))

    def get_theta(self):
        with self.lock:
            return self.heading.theta

    def wait_for_imu(self, timeout=3.0):
        t0 = time.monotonic()
        while self.imu_count < 5:
            if time.monotonic() - t0 > timeout:
                raise Abort('No /imu_data received. Is motor_control running?')
            time.sleep(0.05)

    def wait_still(self, seconds=STILL_S):
        """Stay idle so ZUPT re-estimates the gyro bias."""
        time.sleep(self.settle_s + seconds)
        print(f'  gyro bias {self.heading.bias:.3f}')

    # ---------- motion ----------

    def send(self, L, R):
        msg = Float32MultiArray()
        msg.data = [float(L), float(R)]
        self.pub.publish(msg)

    def stop(self):
        for _ in range(5):
            self.send(0.0, 0.0)
            time.sleep(1.0 / RATE_HZ)
        self.commanding = False
        self.stop_time = time.monotonic()

    def drive(self, L, R, duration, stop=True):
        """Publish [L, R] at RATE_HZ for duration; return samples [(t, theta)]."""
        self.commanding = True
        samples = []
        t0 = time.monotonic()
        while True:
            t = time.monotonic() - t0
            samples.append((t, self.get_theta()))
            if t >= duration:
                break
            self.send(L, R)
            time.sleep(1.0 / RATE_HZ)
        if stop:
            self.stop()
        return samples

    def settle_theta(self):
        """Heading after coasting has ended (heading freezes once stationary)."""
        time.sleep(self.settle_s + 0.3)
        return self.get_theta()

    @staticmethod
    def rate(samples, t_from):
        s = [p for p in samples if p[0] >= t_from]
        if len(s) < 2 or s[-1][0] - s[0][0] <= 0:
            return 0.0
        return (s[-1][1] - s[0][1]) / (s[-1][0] - s[0][0])

    # ---------- helpers ----------

    def params_for(self, test):
        cmd, dur = DEFAULTS[test]
        if cmd is None:
            cmd = self.profile.rot_cmd
        if self.cmd_override > 0:
            cmd = self.cmd_override
        if self.dur_override > 0:
            dur = self.dur_override
        return cmd, dur

    def save(self, trial, updates):
        print('\n  Result:')
        for k, v in updates.items():
            print(f'    {k}: {getattr(self.profile, k)} -> {v}')
        if not ask_yes('  Save to profile?'):
            print('  Not saved.')
            return
        for k, v in updates.items():
            setattr(self.profile, k, v)
        trial['time'] = datetime.now().isoformat(timespec='seconds')
        path = save_profile(self.profile_dir, self.profile, trial)
        print(f'  Saved: {path}')

    def mean_with(self, test, key, new_value, **match):
        """Average of key over saved trials (matching fields) plus the new value."""
        vals = [t[key] for t in load_trials(self.profile_dir, self.floor, test)
                if key in t and all(t.get(k) == v for k, v in match.items())]
        return statistics.mean(vals + [new_value]), len(vals) + 1

    def recomputed_trim(self, track):
        trials = load_trials(self.profile_dir, self.floor, 'straight')
        trims = [trim_from_lateral(t['lateral_m'], t['dist_m'], track)
                 for t in trials if 'lateral_m' in t]
        return round(statistics.mean(trims), 4) if trims else None

    # ---------- tests ----------

    def run_straight(self):
        cmd, dur = self.params_for('straight')
        prompt(f'Robot will drive straight at [{cmd}, {cmd}] for {dur:.1f} s '
               f'(~{cmd * dur:.2f} m).\n'
               '  Align the robot with a reference line (e.g. a floor plank seam), '
               'mark the start, clear the path.')
        self.wait_still()
        th0 = self.get_theta()
        self.drive(cmd, cmd, dur)
        dth = self.settle_theta() - th0
        print(f'  gyro heading change: {math.degrees(dth):.1f} deg '
              '(info only, gyro noise ~+-7 deg)')
        x = ask_float('  Forward distance along the reference line (m): ')
        y = ask_float('  Lateral offset from the line (cm, left = +): ') / 100.0

        dist = math.hypot(x, y)
        scale = dist / (cmd * dur)
        trim = trim_from_lateral(y, dist, self.profile.track_eff)
        print(f'  this trial: linear_scale={scale:.4f}, trim={trim:.4f}')
        mean_scale, n = self.mean_with('straight', 'linear_scale', scale)
        mean_trim, _ = self.mean_with('straight', 'trim', trim)
        print(f'  averaged over {n} trial(s)')
        self.save({'test': 'straight', 'cmd': cmd, 'duration': dur, 'forward_m': x,
                   'lateral_m': y, 'dist_m': dist, 'dtheta_gyro': dth,
                   'linear_scale': scale, 'trim': trim},
                  {'linear_scale': round(mean_scale, 4), 'trim': round(mean_trim, 4)})

    def run_arc(self):
        c, dur = self.params_for('arc')
        d = ARC_D
        print(f'  Two gentle arcs at [{c - d:.2f}, {c + d:.2f}] (left) and '
              f'[{c + d:.2f}, {c - d:.2f}] (right), {dur:.1f} s each. Needs ~1.5 m of space.')
        ks, ws = [], []
        for L, R, name in ((c - d, c + d, 'left'), (c + d, c - d, 'right')):
            prompt(f'{name.upper()} arc: align the robot with the reference line '
                   'at the start mark.')
            self.wait_still(2.0)
            samples = self.drive(L, R, dur)
            ws.append(self.rate(samples, STEADY_FROM_S))
            x = ask_float('  End point: forward distance along the line (m): ')
            y = ask_float('  End point: lateral offset (cm, left = +): ') / 100.0
            ks.append(arc_curvature(x, y))
            print(f'  {name}: curvature {ks[-1]:.3f} 1/m '
                  f'(gyro rate {math.degrees(ws[-1]):.1f} deg/s, info only)')
        if ks[0] <= 0 or ks[1] >= 0:
            print('  WARNING: arc direction unexpected; check the lateral sign (left = +).')
        track = 2.0 * d / (c * (ks[0] - ks[1]) / 2.0)
        track_gyro = 2.0 * d * self.profile.linear_scale / ((abs(ws[0]) + abs(ws[1])) / 2.0)
        print(f'  this trial: track_eff={track:.4f} m (gyro estimate {track_gyro:.4f}, '
              'nominal 0.172)')
        mean_track, n = self.mean_with('arc', 'track_eff', track)
        print(f'  averaged over {n} trial(s)')
        updates = {'track_eff': round(mean_track, 4)}
        trim = self.recomputed_trim(mean_track)  # trim depends on track_eff
        if trim is not None:
            updates['trim'] = trim
        self.save({'test': 'arc', 'cmd': c, 'd': d, 'duration': dur, 'k_left': ks[0],
                   'k_right': ks[1], 'w_left_gyro': ws[0], 'w_right_gyro': ws[1],
                   'track_eff': track}, updates)

    def run_rotate(self):
        cmd, dur = self.params_for('rotate')
        prompt(f'Robot will spin in place at {cmd}: CCW {dur:.1f} s, stop, '
               f'then CW {dur:.1f} s, stop.')
        prof = replace(self.profile, rot_cmd=cmd)
        res = {}
        for direction, name in ((+1, 'ccw'), (-1, 'cw')):
            self.wait_still(2.0)
            samples = self.drive(*rotate_command(direction, prof), dur)
            th_stop = samples[-1][1]
            th_end = self.settle_theta()
            rate = abs(self.rate(samples, STEADY_FROM_S))
            coast = abs(th_end - th_stop)
            print(f'  {name.upper()}: steady {math.degrees(rate):.1f} deg/s, '
                  f'coast {math.degrees(coast):.1f} deg after stop')
            res[name] = (rate, coast)

        updates = {'rot_cmd': cmd}
        for name, (rate, coast) in res.items():
            updates[f'rot_rate_{name}'] = round(
                self.mean_with('rotate', f'rate_{name}', rate, cmd=cmd)[0], 4)
            updates[f'rot_coast_{name}'] = round(
                self.mean_with('rotate', f'coast_{name}', coast, cmd=cmd)[0], 4)
        self.save({'test': 'rotate', 'cmd': cmd, 'duration': dur,
                   'rate_ccw': res['ccw'][0], 'coast_ccw': res['ccw'][1],
                   'rate_cw': res['cw'][0], 'coast_cw': res['cw'][1]}, updates)

    def run_deadband(self):
        start, step_s = self.params_for('deadband')
        prompt(f'Robot will spin in place (CCW) with command rising from {start} '
               f'by 0.01 every {step_s:.1f} s until it rotates.')
        self.wait_still(2.0)
        cmd, found = start, None
        try:
            while cmd <= DEADBAND_MAX + 1e-9:
                samples = self.drive(-cmd, cmd, step_s, stop=False)
                w = abs(self.rate(samples, 0.3 * step_s))
                print(f'  cmd {cmd:.2f}: {math.degrees(w):.1f} deg/s')
                if w > DEADBAND_RATE:
                    found = cmd
                    break
                cmd = round(cmd + 0.01, 3)
        finally:
            self.stop()
        if found is None:
            print(f'  No rotation detected up to {DEADBAND_MAX}; not saved.')
            return
        self.save({'test': 'deadband', 'rot_deadband': found}, {'rot_deadband': found})

    # ---------- IMU diagnostic (hand rotation) ----------

    DIAG_SEGMENTS = [
        ('CCW 90 deg', 90.0),
        ('CCW another 90 deg (total 180)', 90.0),
        ('CCW another 180 deg (total 360)', 180.0),
        ('CW 360 deg back to start', -360.0),
        ('FAST CCW 90 deg (~0.5 s)', 90.0),
        ('FAST CW 90 deg back to start (~0.5 s)', -90.0),
    ]

    def run_imu_diag(self):
        _, static_s = self.params_for('imu_diag')
        print('  Motors stay idle. You rotate the robot BY HAND on the floor (do not lift it).')
        print('  For each step: rotate, align to the mark, keep still ~3 s, then press Enter.')
        marks = []  # (log index, label)
        with self.lock:
            self.log = []

        def mark(label):
            with self.lock:
                marks.append((len(self.log), label))

        prompt(f'Keep the robot still for {static_s:.0f} s (gyro noise / bias).')
        mark('static_start')
        time.sleep(static_s)
        mark('static_end')
        input('  Align the robot to the 0 deg mark, keep still, press Enter...')
        mark('start')
        for label, _ in self.DIAG_SEGMENTS:
            input(f'  -> Rotate {label}; align, wait ~3 s, press Enter...')
            mark(label)
        with self.lock:
            log, self.log = self.log, None

        path = self.diag_dump(log, marks)
        print(f'  Raw samples: {path}')
        static = (marks[0][0], marks[1][0])
        noise = statistics.pstdev(r[2] for r in log[static[0]:static[1]])
        bounds = [i for i, _ in marks[2:]]
        rows, bias = diag_analyze(log, static, bounds, self.DIAG_SEGMENTS)
        print(f'  gz noise std {noise:.2f}, bias (all still samples) {bias:.3f}')
        scale = diag_report(rows)
        updates = {'gyro_scale': round(scale, 4)} if scale else {}
        self.save({'test': 'imu_diag', 'gz_noise': noise, 'gz_bias': bias, 'gyro_scale': scale,
                   'segments': rows, 'csv': os.path.basename(path)}, updates)

    def diag_dump(self, log, marks):
        d = os.path.join(os.path.dirname(os.path.expanduser(self.profile_dir)), 'diag')
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, f'imu_diag_{self.floor}_{datetime.now():%Y%m%d_%H%M%S}.csv')
        by_index = {i: label for i, label in marks}
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'board_yaw_rad', 'gz_raw', 'mark'])
            for i, row in enumerate(log):
                w.writerow([*row, by_index.get(i, '')])
        return path

    def run(self):
        if not self.floor:
            raise Abort('Set the floor: -p floor:=<name> (e.g. carpet, wood).')
        self.wait_for_imu()
        print(f'\nFloor: {self.floor}   profile dir: {self.profile_dir}')
        print(f'Current profile: {self.profile}')
        tests = ALL_ORDER if self.test == 'all' else [self.test]
        for test in tests:
            if test not in DEFAULTS:
                raise Abort(f'Unknown test "{test}". Choose from {list(DEFAULTS)} or all.')
            print(f'\n===== {test} =====')
            getattr(self, f'run_{test}')()


# ---------- pure helpers (no ROS) ----------

def trim_from_lateral(y, dist, track):
    """Trim cancelling a constant-curvature drift that ended y (left +) off after dist.

    curvature k = 2y/d^2; a trim t gives k = 2t/track  ->  t = -y*track/d^2
    """
    return -y * track / (dist * dist)


def arc_curvature(x, y):
    """Signed curvature of a circular arc from (0,0) heading +x that ends at (x, y)."""
    return 2.0 * y / (x * x + y * y)


def diag_window(log, i0, i1, bias, smooth_s=0.5, pad_s=0.3, rel_thr=0.25, merge_gap_s=2.0):
    """(t_from, t_to) of the actual rotation inside log[i0:i1], or None."""
    seg = log[i0:i1]
    sm, j, acc = [], 0, 0.0  # trailing moving average of |gz - bias|
    for k, r in enumerate(seg):
        acc += r[2] - bias
        while seg[j][0] < r[0] - smooth_s:
            acc -= seg[j][2] - bias
            j += 1
        sm.append(abs(acc / (k - j + 1)))

    if max(sm) <= 0.0:
        return None  # no gyro signal
    # block above threshold around the peak; merge blocks split by short pauses
    thr = rel_thr * max(sm)
    blocks, start = [], None
    for k, v in enumerate(sm + [0.0]):
        if v > thr and start is None:
            start = k
        elif v <= thr and start is not None:
            blocks.append([start, k - 1])
            start = None
    merged = [blocks[0]]
    for blk in blocks[1:]:
        if seg[blk[0]][0] - seg[merged[-1][1]][0] < merge_gap_s:
            merged[-1][1] = blk[1]
        else:
            merged.append(blk)
    k_peak = sm.index(max(sm))
    lo, hi = next(b for b in merged if b[0] <= k_peak <= b[1])
    return seg[lo][0] - smooth_s - pad_s, seg[hi][0] + pad_s


def diag_analyze(log, static, bounds, segments):
    """Per-segment gyro integral over the rotation window.

    Bias: mean of ALL still samples (outside rotation windows); short windows are too noisy.
    """
    bias0 = statistics.mean(r[2] for r in log[static[0]:static[1]])
    windows = []
    for k in range(len(segments)):
        i0, i1 = bounds[k], bounds[k + 1]
        windows.append(diag_window(log, i0, i1, bias0) if i1 - i0 >= 10 else None)
    still = [r[2] for r in log[static[0]:]
             if not any(w and w[0] <= r[0] < w[1] for w in windows)]
    bias = statistics.mean(still)

    rows = []
    for k, (label, true_deg) in enumerate(segments):
        row = {'label': label, 'true_deg': true_deg}
        w = windows[k]
        if w:
            seg = log[bounds[k]:bounds[k + 1]]
            integral = sum((seg[i][2] - bias) * (seg[i + 1][0] - seg[i][0])
                           for i in range(len(seg) - 1) if w[0] <= seg[i][0] < w[1])
            row.update(motion_s=round(w[1] - w[0], 2), gz_int=integral,
                       board_deg=round(math.degrees(seg[-1][1] - seg[0][1]), 1))
        rows.append(row)
    return rows, bias


def diag_report(rows):
    """Print table; return least-squares gyro_scale (deg/s per gz unit) or None."""
    pairs = [(r['true_deg'], r['gz_int']) for r in rows if 'gz_int' in r]
    if not pairs or all(abs(g) < 1e-6 for _, g in pairs):
        print('\n  gz is always 0: firmware does not send gyro -> gz NOT available.')
        return None
    scale = sum(t * g for t, g in pairs) / sum(g * g for _, g in pairs)

    print(f'\n  fitted gyro_scale {scale:.4f} deg/s per gz unit')
    print('  segment                                  true   gyro    err  board  motion_s')
    errs = []
    for r in rows:
        if 'gz_int' not in r:
            print(f"  {r['label']:<40} (too few samples)")
            continue
        gyro = scale * r['gz_int']
        errs.append(gyro - r['true_deg'])
        print(f"  {r['label']:<40} {r['true_deg']:>5.0f} {gyro:>6.1f} {errs[-1]:>6.1f} "
              f"{r['board_deg']:>6.1f} {r['motion_s']:>8.2f}")
    closure = scale * sum(g for _, g in pairs) - sum(t for t, _ in pairs)
    print(f'\n  Loop closure (all segments, should be ~0): {closure:.1f} deg')
    print(f'  Max segment error: {max(abs(e) for e in errs):.1f} deg '
          '(includes hand-alignment error, typically +-5 deg)')
    print('  Board yaw column is for reference only (known heading-dependent error).')
    return scale


def prompt(text):
    input(f'  {text}\n  Press Enter to start (Ctrl-C to abort)...')


def ask_yes(text):
    return input(f'{text} [Y/n] ').strip().lower() in ('', 'y', 'yes')


def ask_float(text):
    while True:
        try:
            return float(input(text))
        except ValueError:
            print('  Please enter a number.')


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        node.run()
    except Abort as e:
        node.get_logger().error(str(e))
    except (KeyboardInterrupt, EOFError):
        print('\nAborted.')
    finally:
        # context may already be invalid after Ctrl-C; motor_control's watchdog stops anyway
        try:
            node.stop()
        except Exception:
            pass
        executor.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    main()
