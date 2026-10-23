"""Landmark map: known AprilTag poses in the world frame (no ROS).

YAML format (config/hw2/landmarks.yaml):
    tag_size: 0.166
    landmarks:
      - {id: 0, x: 1.5, y: 0.0, yaw_deg: 180}   # yaw = tag's outward normal
"""
import math
from dataclasses import dataclass

import yaml

from robot_control.se2 import Pose2, wrap_angle


@dataclass(frozen=True)
class Landmark:
    id: int
    x: float
    y: float
    yaw: float      # rad, outward normal in world

    def pose(self):
        return Pose2(self.x, self.y, self.yaw)


@dataclass
class LandmarkMap:
    tag_size: float
    landmarks: dict     # id -> Landmark

    def get(self, tag_id):
        return self.landmarks.get(tag_id)

    def ids(self):
        return sorted(self.landmarks)


def parse_landmark(entry):
    for key in ('id', 'x', 'y', 'yaw_deg'):
        if key not in entry:
            raise ValueError(f'landmark {entry} has no {key!r}')
    return Landmark(int(entry['id']), float(entry['x']), float(entry['y']),
                    wrap_angle(math.radians(float(entry['yaw_deg']))))


def parse_map(data):
    landmarks = {}
    for entry in data.get('landmarks') or []:
        lm = parse_landmark(entry)
        if lm.id in landmarks:
            raise ValueError(f'duplicate landmark id {lm.id}')
        landmarks[lm.id] = lm
    if not landmarks:
        raise ValueError('map has no landmarks')
    return LandmarkMap(float(data.get('tag_size', 0.0)), landmarks)


def load_map(path):
    with open(path) as f:
        return parse_map(yaml.safe_load(f) or {})
