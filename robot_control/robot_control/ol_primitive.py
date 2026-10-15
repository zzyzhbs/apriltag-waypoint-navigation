"""Primitive executor shared by ol_calibrate and velocity_control (no ROS; publish is injected).

Every primitive: start from rest -> hold [L, R] for T s -> stop -> wait settle_s.
"""
import time

RATE_HZ = 50.0
STOP_REPEATS = 5


def wheel_cmds(kind, direction, cmd, trim=0.0):
    if kind == 'drive':
        return cmd * (1.0 - trim), cmd * (1.0 + trim)
    if direction == 'ccw':
        return -cmd, cmd
    if direction == 'cw':
        return cmd, -cmd
    raise ValueError(f'bad spin direction {direction!r}')


class PrimitiveRunner:
    def __init__(self, publish, settle_s=1.0, rate_hz=RATE_HZ,
                 clock=time.monotonic, sleep=time.sleep):
        self.publish = publish      # publish(L, R)
        self.settle_s = settle_s
        self.dt = 1.0 / rate_hz
        self.clock = clock
        self.sleep = sleep

    def stop(self):
        for _ in range(STOP_REPEATS):
            self.publish(0.0, 0.0)
            self.sleep(self.dt)

    def run(self, L, R, T):
        """Returns (t_start, t_stop) on the monotonic clock; t_stop = first stop command."""
        t0 = self.clock()
        k = 0
        while True:
            now = self.clock()
            if now - t0 >= T:
                break
            self.publish(L, R)
            k += 1
            self.sleep(max(0.0, min(t0 + k * self.dt, t0 + T) - self.clock()))
        t1 = self.clock()
        self.stop()
        self.sleep(self.settle_s)
        return t0, t1

    def run_step(self, kind, direction, cmd, T, trim=0.0):
        L, R = wheel_cmds(kind, direction, cmd, trim)
        return (L, R), self.run(L, R, T)
