"""Gyro-integrated heading with zero-velocity bias updates (ZUPT). No ROS dependency."""
from __future__ import annotations

import math
from collections import deque

MAX_DT = 0.5


class GyroHeading:
    """Integrate angular_velocity.z into a continuous (unwrapped) heading.

    gz is in raw gyro units; gyro_scale converts to deg/s (measured ~1.0 on our IMU).
    While stationary the heading is frozen and the bias is re-estimated (ZUPT).
    """

    def __init__(self, gyro_scale: float = 1.0, bias_window: int = 3000,
                 min_bias_samples: int = 50, bias: float = 0.0):
        self.gyro_scale = gyro_scale
        self.min_bias_samples = min_bias_samples
        self.bias = bias
        self.theta = 0.0
        self._last_t = None
        self._last_gz = 0.0
        # time-weighted stationary samples across ALL still periods (~60 s):
        # gyro noise has slow components, so short windows give poor bias estimates;
        # weighting by hold time keeps the bias consistent with the ZOH integration
        self._window = deque(maxlen=bias_window)  # (gz * w, w)
        self._sum_gw = 0.0
        self._sum_w = 0.0

    def reset(self, theta: float = 0.0):
        self.theta = theta

    def rate(self, gz: float) -> float:
        """Bias-corrected angular rate, rad/s."""
        return math.radians(self.gyro_scale * (gz - self.bias))

    def update(self, t: float, gz: float, stationary: bool) -> float:
        dt = None if self._last_t is None else t - self._last_t
        valid = dt is not None and 0.0 < dt <= MAX_DT
        if stationary:
            if valid:  # previous sample held for dt
                self._add_bias_sample(self._last_gz, dt)
        elif valid:
            # zero-order hold: previous sample holds until this timestamp
            self.theta += self.rate(self._last_gz) * dt
        self._last_t, self._last_gz = t, gz
        return self.theta

    def _add_bias_sample(self, gz: float, w: float):
        if len(self._window) == self._window.maxlen:
            gw, w0 = self._window[0]
            self._sum_gw -= gw
            self._sum_w -= w0
        self._window.append((gz * w, w))
        self._sum_gw += gz * w
        self._sum_w += w
        if len(self._window) >= self.min_bias_samples and self._sum_w > 0:
            self.bias = self._sum_gw / self._sum_w
