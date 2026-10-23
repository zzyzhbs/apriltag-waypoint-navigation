"""HW2 closed-loop waypoint runner (stop-look-correct).
Needs hw2_perception_launch.py and motor_control running (or use hw2_launch.py).

ros2 run robot_control hw2_nav --ros-args -p floor:=study_wood2 [-p dry_run:=true]

dry_run: never moves; looks a few times and prints the estimate and the planned
first step, to check tags, map and calibration before a real run.
"""
import math
import os
import threading
import time
from datetime import datetime

import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.executors import SingleThreadedExecutor
from tf2_ros import TransformBroadcaster

from robot_control import ekf, ol_node
from robot_control.landmark_map import load_map
from robot_control.nav_log import CsvNavLog, TeeLog
from robot_control.ol_model import DEFAULT_CALIB_DIR, Calibration, ModelError
from robot_control.ol_planner import load_waypoints
from robot_control.se2 import Pose2, yaw_to_quat
from robot_control.stop_look import (
    NavigationHalt, Navigator, NavLog, next_move, params_from_config, to_step,
)
from robot_control.tag_listener import TagListener
from robot_control.tag_obs import filter_window

SRC = '~/ros2_ws/src/CSE276A'
DEFAULT_WAYPOINTS = f'{SRC}/HW2/waypoints.txt'
DEFAULT_LANDMARKS = f'{SRC}/robot_control/config/hw2/landmarks.yaml'
DEFAULT_NAV = f'{SRC}/robot_control/config/hw2/nav.yaml'
DEFAULT_LOG_DIR = f'{SRC}/HW2/logs'
COUNTDOWN_S = 3
DRY_LOOKS = 3


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


class PosePublisher(NavLog):
    """Publishes the estimate as /hw2/pose and TF world -> base_link (for Foxglove)."""

    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(PoseWithCovarianceStamped, 'hw2/pose', 10)
        self.tf = TransformBroadcaster(node)

    def publish(self, pose, P):
        stamp = self.node.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quat(pose.theta)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp, msg.header.frame_id = stamp, 'world'
        p = msg.pose.pose
        p.position.x, p.position.y = pose.x, pose.y
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        cov = [0.0] * 36
        for i, j in ((0, 0), (1, 1), (0, 1), (1, 0)):
            cov[6 * i + j] = float(P[i, j])
        cov[35] = float(P[2, 2])        # yaw
        msg.pose.covariance = cov
        self.pub.publish(msg)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'world', 'base_link'
        t.transform.translation.x, t.transform.translation.y = pose.x, pose.y
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = qx, qy, qz, qw
        self.tf.sendTransform(t)

    def on_look(self, goal_index, records, pose, P):
        self.publish(pose, P)

    def on_step(self, goal_index, step, pose_before, pose_after, P, timing=None):
        self.publish(pose_after, P)


class Hw2NavNode(ol_node.MotorNode):
    def __init__(self):
        super().__init__('hw2_nav')
        self.floor = self.param('floor', 'study_wood2')
        self.paths = {
            'calib_dir': os.path.expanduser(self.param('calib_dir', DEFAULT_CALIB_DIR)),
            'waypoints': os.path.expanduser(self.param('waypoints', DEFAULT_WAYPOINTS)),
            'landmarks': os.path.expanduser(self.param('landmarks', DEFAULT_LANDMARKS)),
            'nav_config': os.path.expanduser(self.param('nav_config', DEFAULT_NAV)),
        }
        self.log_root = os.path.expanduser(self.param('log_dir', DEFAULT_LOG_DIR))
        self.dry_run = bool(self.param('dry_run', False))
        self.z_sign = int(self.param('tag_z_sign', 1))
        self.run_ = self.runner(self.float_param('settle_s', 1.0))
        self.wp_pause_s = self.float_param('wp_pause_s', 1.0)

        self.cfg = load_yaml(self.paths['nav_config'])
        self.params = params_from_config(self.cfg)
        self.noise = ekf.noise_from_config(self.cfg['ekf'])
        self.map = load_map(self.paths['landmarks'])
        self.calib = Calibration.for_floor(self.paths['calib_dir'], self.floor)
        wps = load_waypoints(self.paths['waypoints'])
        self.start = Pose2(*wps[0])
        self.goals = [Pose2(*w) for w in wps[1:]]
        self.tags = TagListener(self, self.z_sign)
        self.pose_pub = PosePublisher(self)
        print(f'calibration: {self.calib.path}\nmap: {self.paths["landmarks"]} '
              f'tags {self.map.ids()}\nwaypoints: {self.paths["waypoints"]}')

    # ---------- callbacks given to Navigator ----------

    def look(self):
        """Stand still for one window and return the averaged tag observations."""
        look_cfg = self.cfg['look']
        since = self.tags.now()
        time.sleep(look_cfg['window_s'])
        raw, _ = self.tags.collect(since)
        accepted, rejected = filter_window(raw, set(self.map.ids()), look_cfg)
        seen = ', '.join(f'{i}@{o.dist:.2f}m' for i, o in sorted(accepted.items()))
        print(f'  look: {len(raw)} frames, tags [{seen}]'
              + (f', rejected {rejected}' if rejected else ''))
        return accepted, rejected

    def execute(self, step):
        print(f'  {step.describe()}')
        _, timing = self.run_.run_step(step.kind, step.dir, step.cmd, step.T)
        return timing

    # ---------- runs ----------

    def new_log(self):
        run_dir = os.path.join(self.log_root, f'run_{datetime.now():%Y%m%d_%H%M%S}')
        log = CsvNavLog(run_dir, self.start, ekf.sigmas(ekf.initial_covariance(self.noise)))
        log.write_config({'floor': self.floor, 'tag_z_sign': self.z_sign, **self.paths,
                          'nav': self.cfg, 'map': load_yaml(self.paths['landmarks'])})
        return log

    def navigator(self, log):
        return Navigator(self.map, self.calib, self.params, self.noise, self.start,
                         self.look, self.execute, log)

    def run_dry(self):
        """Look a few times and print the estimate and the first planned step."""
        nav = self.navigator(self.pose_pub)
        for _ in range(DRY_LOOKS):
            nav.update_from_look(1)
            pose = nav.pose()
            sx, sy, st = ekf.sigmas(nav.P)
            move = next_move(pose, self.goals[0], self.params)
            plan = to_step(move, self.calib, self.params).describe() if move else 'reached'
            print(f'estimate ({pose.x:+.3f}, {pose.y:+.3f}, {math.degrees(pose.theta):+.1f} deg)'
                  f' sigma ({sx:.3f}, {sy:.3f}, {math.degrees(st):.1f} deg) -> {plan}')

    def run(self):
        if self.dry_run:
            return self.run_dry()
        log = self.new_log()
        nav = self.navigator(TeeLog(log, self.pose_pub))
        print(f'log: {log.dir}')
        for i in range(COUNTDOWN_S, 0, -1):
            print(f'starting in {i}...')
            time.sleep(1.0)
        try:
            for i, goal in enumerate(self.goals, 1):
                print(f'--- waypoint {i}: ({goal.x:g}, {goal.y:g}, '
                      f'{math.degrees(goal.theta):.0f} deg)')
                print_result(nav.run_goal(i, goal))
                time.sleep(self.wp_pause_s)
        finally:
            log.close()
        print(f'done; log: {log.dir}')


def print_result(r):
    p = r.pose
    print(f'>>> waypoint {r.index} {"reached" if r.reached else "NOT reached"}: '
          f'estimate ({p.x:+.3f}, {p.y:+.3f}, {math.degrees(p.theta):+.1f} deg), '
          f'{r.iters} looks, tags {r.tags_used} {r.note}')


def spin_in_background(node):
    """Receive /detections and TF while the main thread drives."""
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return executor


def main(args=None):
    ol_node.init(args)
    node, executor = None, None
    try:
        node = Hw2NavNode()
        executor = spin_in_background(node)
        node.run()
    except NavigationHalt as e:
        print(f'HALT: {e}')
    except (ModelError, ValueError, KeyError, OSError) as e:
        print(f'ERROR: {e}')
    except KeyboardInterrupt:
        print('\nAborted.')
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
        ol_node.shutdown(node)
    return 0


if __name__ == '__main__':
    main()
