"""Simulated rover for closed-loop tests: imperfect primitives + noisy tag views."""
import math
from dataclasses import dataclass

from robot_control.se2 import Pose2, angle_between, to_child, wrap_angle
from robot_control.tag_obs import make_obs


@dataclass
class SimNoise:
    drive_scale: float = 1.03       # systematic: drives 3 % too far
    drive_sigma: float = 0.01       # m per drive
    drift: float = math.radians(1.5)    # rad per m driven (veers left)
    spin_scale: float = 0.95        # turns 5 % short
    spin_sigma: float = math.radians(2.0)
    obs_xy: float = 0.003           # m, per look (already averaged)
    obs_psi: float = math.radians(1.0)
    half_fov: float = math.radians(45.0)
    max_dist: float = 3.0
    max_oblique: float = math.radians(60.0)


class SimRover:
    def __init__(self, landmark_map, start, rng, noise=None):
        self.map = landmark_map
        self.true = start
        self.rng = rng
        self.n = noise or SimNoise()
        self.steps = []

    def execute(self, step):
        self.steps.append(step)
        if step.kind == 'drive':
            self.drive(step.amount)
        else:
            sign = 1.0 if step.dir == 'ccw' else -1.0
            self.spin(sign * math.radians(step.amount))

    def drive(self, d):
        n = self.n
        actual = d * n.drive_scale + self.rng.normal(0.0, n.drive_sigma)
        dth = n.drift * actual
        mid = self.true.theta + dth / 2
        self.true = Pose2(self.true.x + actual * math.cos(mid),
                          self.true.y + actual * math.sin(mid),
                          wrap_angle(self.true.theta + dth))

    def spin(self, a):
        actual = a * self.n.spin_scale + self.rng.normal(0.0, self.n.spin_sigma)
        self.true = Pose2(self.true.x, self.true.y, wrap_angle(self.true.theta + actual))

    def visible(self, lm):
        x, y = to_child(self.true, lm.x, lm.y)
        d = math.hypot(x, y)
        oblique = angle_between(math.cos(lm.yaw), math.sin(lm.yaw),
                                self.true.x - lm.x, self.true.y - lm.y)
        return (0.25 <= d <= self.n.max_dist and abs(math.atan2(y, x)) <= self.n.half_fov
                and oblique <= self.n.max_oblique)

    def look(self):
        seen = {}
        for lm in self.map.landmarks.values():
            if not self.visible(lm):
                continue
            x, y = to_child(self.true, lm.x, lm.y)
            seen[lm.id] = make_obs(
                lm.id, 0.0, x + self.rng.normal(0.0, self.n.obs_xy),
                y + self.rng.normal(0.0, self.n.obs_xy),
                lm.yaw - self.true.theta + self.rng.normal(0.0, self.n.obs_psi), 10)
        return seen, {}
