"""HW2 static checks with the robot standing still (motors are never used).
Needs hw2_perception_launch.py running.

ros2 run robot_control hw2_probe --ros-args -p mode:=localize
ros2 run robot_control hw2_probe --ros-args -p mode:=survey \
    -p robot_x:=0.0 -p robot_y:=0.0 -p robot_theta_deg:=0.0 [-p write:=true]
ros2 run robot_control hw2_probe --ros-args -p mode:=stats -p duration_s:=10.0
"""
import math
import os
import time

import rclpy
import yaml
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from robot_control.landmark_map import load_map
from robot_control.localize import fuse_poses, pose_from_tag, tag_from_pose, tag_weight
from robot_control.probe_report import (
    fmt_pose, format_latency, format_stats, format_survey_row, survey_yaml, tag_stats,
)
from robot_control.se2 import Pose2
from robot_control.tag_listener import TagListener
from robot_control.tag_obs import aggregate, group_by_id, is_usable

DEFAULT_LANDMARKS = '~/ros2_ws/src/CSE276A/robot_control/config/hw2/landmarks.yaml'
MODES = ('localize', 'survey', 'stats')


class ProbeNode(Node):
    def __init__(self):
        super().__init__('hw2_probe')
        self.mode = self.param('mode', 'localize')
        self.map_path = os.path.expanduser(self.param('landmarks', DEFAULT_LANDMARKS))
        self.window_s = float(self.param('window_s', 1.0))
        self.duration_s = float(self.param('duration_s', 10.0))
        self.robot = Pose2(float(self.param('robot_x', 0.0)), float(self.param('robot_y', 0.0)),
                           math.radians(float(self.param('robot_theta_deg', 0.0))))
        self.write = bool(self.param('write', False))
        self.map = load_map(self.map_path)
        self.tags = TagListener(self, int(self.param('tag_z_sign', 1)))
        if self.mode not in MODES:
            raise ValueError(f'mode must be one of {MODES}')
        print(f'map: {self.map_path}  tags {self.map.ids()}')

    def param(self, name, default):
        return self.declare_parameter(
            name, default, ParameterDescriptor(dynamic_typing=True)).value

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def gather(self, seconds):
        """Raw observations whose image was taken during the next `seconds`."""
        since = self.tags.now()
        self.spin_for(seconds)
        return self.tags.collect(since)

    def usable_known(self, obs_list):
        return [o for o in obs_list if self.map.get(o.id) is not None and is_usable(o)]

    # ---------- modes ----------

    def run_localize(self):
        """Every window: robot pose from each tag, and their weighted mean."""
        while rclpy.ok():
            raw, _ = self.gather(self.window_s)
            accepted, rejected = aggregate(self.usable_known(raw))
            print(f'--- {len(raw)} frames')
            poses, weights = [], []
            for tag_id, o in sorted(accepted.items()):
                p = pose_from_tag(o, self.map.get(tag_id))
                poses.append(p)
                weights.append(tag_weight(o))
                print(f'tag {tag_id} (d {o.dist:.2f} m, {o.count} frames): {fmt_pose(p)}')
            for tag_id, reason in sorted(rejected.items()):
                print(f'tag {tag_id}: rejected, {reason}')
            if poses:
                print(f'fused: {fmt_pose(fuse_poses(poses, weights))}')

    def run_survey(self):
        """Robot at a known pose: measure where each tag is and compare with the map."""
        print(f'robot at {fmt_pose(self.robot)}; collecting {self.duration_s:g} s ...')
        raw, _ = self.gather(self.duration_s)
        accepted, rejected = aggregate(self.usable_known(raw))
        measured = {i: tag_from_pose(o, self.robot) for i, o in accepted.items()}
        for tag_id, pose in sorted(measured.items()):
            print(format_survey_row(self.map.get(tag_id), pose))
        for tag_id, reason in sorted(rejected.items()):
            print(f'tag {tag_id}: rejected, {reason}')
        if self.write and measured:
            self.write_survey(measured)

    def write_survey(self, measured):
        path = os.path.join(os.path.dirname(self.map_path), 'landmarks_survey.yaml')
        with open(path, 'w') as f:
            yaml.safe_dump(survey_yaml(self.map.tag_size, measured, self.robot), f,
                           sort_keys=False, default_flow_style=None)
        print(f'written: {path}')

    def run_stats(self):
        """Raw per-tag noise, detection rate and latency (no filtering)."""
        print(f'collecting {self.duration_s:g} s ...')
        raw, latencies = self.gather(self.duration_s)
        for tag_id, group in sorted(group_by_id(raw).items()):
            print(format_stats(tag_id, tag_stats(group, self.duration_s)))
        print(format_latency(latencies))
        print(f'TF lookup failures: {self.tags.lookup_failures}')

    def run(self):
        getattr(self, f'run_{self.mode}')()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ProbeNode()
        node.run()
    except (ValueError, OSError) as e:
        print(f'ERROR: {e}')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass    # Ctrl-C
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    main()
