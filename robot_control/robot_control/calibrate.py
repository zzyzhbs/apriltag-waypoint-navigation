"""Floor calibration tool. Run with `ros2 run` (needs stdin), motor_control must be running.

ros2 run robot_control calibrate --ros-args -p floor:=wood -p test:=straight
tests: drift, yaw_check, straight, rotate, deadband, all
"""
import math
import statistics
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

from robot_control.kinematics import (
    DEFAULT_PROFILE_DIR, load_profile, load_trials, quat_to_yaw, save_profile, wrap_angle,
)

RATE_HZ = 20.0
SETTLE_S = 1.0  # skip spin-up transient when measuring steady rotation rate

# test: (default cmd, default duration s)
DEFAULTS = {
    'drift': (0.0, 30.0),
    'yaw_check': (0.1, 25.0),   # duration = timeout
    'straight': (0.3, 4.0),
    'rotate': (0.2, 4.0),
    'deadband': (0.01, 1.0),    # start cmd / seconds per step
}
ALL_ORDER = ['drift', 'yaw_check', 'straight', 'rotate', 'deadband']


class Abort(Exception):
    pass


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.floor = self.declare_parameter('floor', 'wood').value
        self.test = self.declare_parameter('test', 'all').value
        self.profile_dir = self.declare_parameter('profile_dir', DEFAULT_PROFILE_DIR).value
        self.cmd_override = self.declare_parameter('cmd', -1.0).value
        self.dur_override = self.declare_parameter('duration', -1.0).value

        self.pub = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.create_subscription(Imu, 'imu_data', self.imu_cb, 10)

        # continuous (unwrapped) raw IMU yaw, rad
        self.lock = threading.Lock()
        self.last_raw = None
        self.yaw = 0.0
        self.imu_count = 0

        self.profile = load_profile(self.profile_dir, self.floor, allow_missing=True)

    # ---------- IMU ----------

    def imu_cb(self, msg):
        q = msg.orientation
        raw = quat_to_yaw(q.w, q.x, q.y, q.z)
        with self.lock:
            if self.last_raw is not None:
                self.yaw += wrap_angle(raw - self.last_raw)
            self.last_raw = raw
            self.imu_count += 1

    def get_yaw(self):
        with self.lock:
            return self.yaw

    def wait_for_imu(self, timeout=3.0):
        t0 = time.monotonic()
        while self.imu_count < 5:
            if time.monotonic() - t0 > timeout:
                raise Abort('No /imu_data received. Is motor_control running?')
            time.sleep(0.05)

    # ---------- motion ----------

    def send(self, L, R):
        msg = Float32MultiArray()
        msg.data = [float(L), float(R)]
        self.pub.publish(msg)

    def stop(self):
        for _ in range(5):
            self.send(0.0, 0.0)
            time.sleep(1.0 / RATE_HZ)

    def drive(self, L, R, duration, until=None):
        """Publish [L, R] at RATE_HZ; return [(t, yaw)]. until(t, yaw) -> True stops early."""
        samples = []
        t0 = time.monotonic()
        while True:
            t = time.monotonic() - t0
            yaw = self.get_yaw()
            samples.append((t, yaw))
            if t >= duration or (until and until(t, yaw)):
                break
            self.send(L, R)
            time.sleep(1.0 / RATE_HZ)
        self.stop()
        return samples

    @staticmethod
    def rate(samples, t_from):
        """Average yaw rate over samples with t >= t_from."""
        s = [p for p in samples if p[0] >= t_from]
        if len(s) < 2 or s[-1][0] - s[0][0] <= 0:
            return 0.0
        return (s[-1][1] - s[0][1]) / (s[-1][0] - s[0][0])

    # ---------- helpers ----------

    def params_for(self, test):
        cmd, dur = DEFAULTS[test]
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

    def mean_with(self, test, key, new_value):
        """Average of key over saved trials of this test plus the new value."""
        vals = [t[key] for t in load_trials(self.profile_dir, self.floor, test) if key in t]
        return statistics.mean(vals + [new_value]), len(vals) + 1

    # ---------- tests ----------

    def run_drift(self):
        _, dur = self.params_for('drift')
        prompt(f'Keep the robot still for {dur:.0f} s.')
        samples = self.drive(0.0, 0.0, dur)
        drift = (samples[-1][1] - samples[0][1]) / samples[-1][0]
        print(f'  yaw change {math.degrees(samples[-1][1] - samples[0][1]):.2f} deg '
              f'-> drift {math.degrees(drift):.4f} deg/s')
        mean, n = self.mean_with('drift', 'drift', drift)
        self.save({'test': 'drift', 'duration': round(samples[-1][0], 2), 'drift': drift},
                  {'yaw_drift': round(mean, 6)})

    def run_yaw_check(self):
        cmd, timeout = self.params_for('yaw_check')
        prompt(f'Robot will turn counterclockwise (seen from above) at [-{cmd}, {cmd}] '
               'until the IMU reports 360 deg. Watch the robot.')
        y0 = self.get_yaw()
        samples = self.drive(-cmd, cmd, timeout,
                             until=lambda t, yaw: abs(yaw - y0) >= 2 * math.pi)
        dyaw = samples[-1][1] - y0
        print(f'  IMU yaw change: {math.degrees(dyaw):.1f} deg in {samples[-1][0]:.1f} s')
        if abs(dyaw) < 2 * math.pi:
            print('  WARNING: timeout before IMU reached 360 deg; check IMU units / data.')
        ok = ask_yes('  Did the robot physically turn exactly ~1 full turn CCW?')
        sign = 1 if dyaw > 0 else -1
        print(f'  IMU yaw {"increases" if sign > 0 else "DECREASES"} when turning CCW '
              f'-> yaw_sign = {sign}')
        self.save({'test': 'yaw_check', 'cmd': cmd, 'duration': round(samples[-1][0], 2),
                   'dyaw': dyaw, 'user_confirmed': ok},
                  {'yaw_sign': sign})

    def run_straight(self):
        cmd, dur = self.params_for('straight')
        prompt(f'Robot will drive straight at [{cmd}, {cmd}] for {dur:.1f} s '
               f'(~{cmd * dur:.2f} m). Mark the start position, clear the path.')
        samples = self.drive(cmd, cmd, dur)
        dyaw = self.profile.yaw_sign * (samples[-1][1] - samples[0][1])
        print(f'  heading change: {math.degrees(dyaw):.2f} deg (+ = drifted left)')
        dist = ask_float('  Measured straight-line distance (m): ')

        scale = dist / (cmd * dur)
        trim = trim_from(dyaw, cmd, dur, scale, self.profile.track_eff)
        print(f'  this trial: linear_scale={scale:.4f}, trim={trim:.4f}')
        mean_scale, n = self.mean_with('straight', 'linear_scale', scale)
        mean_trim, _ = self.mean_with('straight', 'trim', trim)
        print(f'  averaged over {n} trial(s)')
        self.save({'test': 'straight', 'cmd': cmd, 'duration': dur, 'measured_m': dist,
                   'dyaw': dyaw, 'linear_scale': scale, 'trim': trim},
                  {'linear_scale': round(mean_scale, 4), 'trim': round(mean_trim, 4)})

    def run_rotate(self):
        cmd, dur = self.params_for('rotate')
        prompt(f'Robot will spin in place: CCW [-{cmd}, {cmd}] {dur:.1f} s, '
               f'then CW [{cmd}, -{cmd}] {dur:.1f} s.')
        ws = []
        for L, R, name in ((-cmd, cmd, 'CCW'), (cmd, -cmd, 'CW')):
            samples = self.drive(L, R, dur)
            w = self.profile.yaw_sign * self.rate(samples, SETTLE_S)
            print(f'  {name}: steady yaw rate {math.degrees(w):.1f} deg/s')
            ws.append(w)
            time.sleep(1.0)
        if ws[0] <= 0 or ws[1] >= 0:
            print('  WARNING: rotation sign unexpected; run yaw_check first.')
        w_mean = (abs(ws[0]) + abs(ws[1])) / 2.0
        track = 2.0 * cmd * self.profile.linear_scale / w_mean
        print(f'  this trial: track_eff={track:.4f} m (nominal 0.172)')
        mean_track, n = self.mean_with('rotate', 'track_eff', track)
        print(f'  averaged over {n} trial(s)')
        updates = {'track_eff': round(mean_track, 4)}
        # trim depends on track_eff: recompute it from raw straight trials
        straight = load_trials(self.profile_dir, self.floor, 'straight')
        if straight:
            trims = [trim_from(t['dyaw'], t['cmd'], t['duration'], t['linear_scale'], mean_track)
                     for t in straight]
            updates['trim'] = round(statistics.mean(trims), 4)
        self.save({'test': 'rotate', 'cmd': cmd, 'duration': dur,
                   'w_ccw': ws[0], 'w_cw': ws[1], 'track_eff': track}, updates)

    def run_deadband(self, threshold=math.radians(3.0), max_cmd=0.15):
        start, step_s = self.params_for('deadband')
        prompt(f'Robot will spin in place with command rising from {start} '
               f'by 0.01 every {step_s:.1f} s until it rotates.')
        cmd = start
        found = None
        while cmd <= max_cmd + 1e-9:
            samples = self.drive(-cmd, cmd, step_s)
            w = abs(self.rate(samples, step_s / 2.0))
            print(f'  cmd {cmd:.2f}: {math.degrees(w):.1f} deg/s')
            if w > threshold:
                found = cmd
                break
            cmd = round(cmd + 0.01, 3)
        if found is None:
            print(f'  No motion detected up to {max_cmd}; not saved.')
            return
        self.save({'test': 'deadband', 'min_cmd': found}, {'min_cmd': found})

    def run(self):
        self.wait_for_imu()
        print(f'\nFloor: {self.floor}   profile dir: {self.profile_dir}')
        print(f'Current profile: {self.profile}')
        tests = ALL_ORDER if self.test == 'all' else [self.test]
        for test in tests:
            if test not in DEFAULTS:
                raise Abort(f'Unknown test "{test}". Choose from {ALL_ORDER} or all.')
            print(f'\n===== {test} =====')
            getattr(self, f'run_{test}')()


def trim_from(dyaw, cmd, dur, scale, track):
    """Trim that cancels the heading drift seen while driving straight at [cmd, cmd]."""
    return -(dyaw / dur) * track / (2.0 * cmd * scale)


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
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
