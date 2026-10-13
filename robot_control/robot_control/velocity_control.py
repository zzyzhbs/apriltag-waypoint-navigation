"""HW1 waypoint follower node. motor_control must be running.

ros2 run robot_control velocity_control --ros-args -p floor:=carpet
Any NavParams field can be overridden, e.g. -p v_max:=0.2 -p drive_heading:=gyro
"""
import csv
import math
import os
from dataclasses import fields
from datetime import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

from robot_control.heading import GyroHeading
from robot_control.kinematics import DEFAULT_PROFILE_DIR, load_profile, wrap_angle
from robot_control.navigator import STOP, NavParams, Navigator, load_waypoints

RATE_HZ = 20.0
IMU_TIMEOUT_S = 0.5
DEFAULT_WAYPOINTS = '~/ros2_ws/src/CSE276A/HW1/waypoints.txt'
DEFAULT_LOG_DIR = '~/ros2_ws/src/CSE276A/HW1/logs'


class VelocityControlNode(Node):
    def __init__(self):
        super().__init__('velocity_control_node')
        floor = self.declare_parameter('floor', '').value  # required, no default
        wp_path = self.declare_parameter('waypoints', DEFAULT_WAYPOINTS).value
        profile_dir = self.declare_parameter('profile_dir', DEFAULT_PROFILE_DIR).value
        log_dir = self.declare_parameter('log_dir', DEFAULT_LOG_DIR).value
        if not floor:
            raise RuntimeError('Set the floor: -p floor:=<name> (e.g. carpet).')

        defaults = NavParams()
        params = NavParams(**{
            f.name: self.declare_parameter(f.name, getattr(defaults, f.name)).value
            for f in fields(NavParams)})
        self.profile = load_profile(profile_dir, floor)
        wps = load_waypoints(os.path.expanduser(wp_path))
        self.nav = Navigator(wps, self.profile, params)
        self.heading = GyroHeading(self.profile.gyro_scale)
        self.heading.reset(wps[0][2])

        self.pub = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.create_subscription(Imu, 'imu_data', self.imu_cb, 10)
        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)
        self.last_imu = None
        self.n_events = 0
        self.stop_ticks = 0

        log_dir = os.path.expanduser(log_dir)
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(log_dir, f'run_{floor}_{datetime.now():%Y%m%d_%H%M%S}.csv')
        self.log_file = open(self.log_path, 'w', newline='')
        self.log = csv.writer(self.log_file)
        self.log.writerow(
            ['t', 'phase', 'wp', 'x', 'y', 'theta_deg', 'gyro_deg', 'bias', 'L', 'R'])

        self.get_logger().info(f'profile: {self.profile}')
        self.get_logger().info(f'params: {params}')
        self.get_logger().info(f'{len(wps)} waypoints from {wp_path}; log: {self.log_path}')
        self.get_logger().info(f'Keep the robot still for {params.start_still_s:.0f} s...')

    @staticmethod
    def now_s(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def imu_cb(self, msg):
        t = self.now_s(msg.header.stamp)
        self.heading.update(t, msg.angular_velocity.z, self.nav.stationary(t))
        self.last_imu = t

    def send(self, L, R):
        msg = Float32MultiArray()
        msg.data = [float(L), float(R)]
        self.pub.publish(msg)

    def tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.nav.done:
            self.send(*STOP)
            self.stop_ticks += 1
            if self.stop_ticks >= 10:
                self.timer.cancel()
                self.summary()
                raise SystemExit
            return
        if self.last_imu is None or t - self.last_imu > IMU_TIMEOUT_S:
            # hold still (motor_control watchdog would stop anyway); don't advance the plan
            self.send(*STOP)
            if self.last_imu is not None:
                self.get_logger().warn('IMU timeout, holding', throttle_duration_sec=1.0)
            return

        L, R = self.nav.step(t, self.heading.theta)
        self.send(L, R)
        n = self.nav
        self.log.writerow([f'{t:.3f}', n.phase, n.wp_index, f'{n.x:.4f}', f'{n.y:.4f}',
                           f'{math.degrees(n.theta):.2f}',
                           f'{math.degrees(self.heading.theta):.2f}',
                           f'{self.heading.bias:.3f}', f'{L:.3f}', f'{R:.3f}'])
        for e in n.events[self.n_events:]:
            self.get_logger().info(e)
        self.n_events = len(n.events)

    def summary(self):
        self.get_logger().info('Estimated pose at each waypoint '
                               '(measure the real one for the report):')
        for i, x, y, th in self.nav.arrivals:
            gx, gy, gth = self.nav.wps[i]
            self.get_logger().info(
                f'  wp{i} goal ({gx}, {gy}, {math.degrees(gth):.0f} deg)  '
                f'est ({x:.3f}, {y:.3f}, {math.degrees(wrap_angle(th)):.1f} deg)')
        self.get_logger().info(f'log: {self.log_path}')

    def close(self):
        self.log_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VelocityControlNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    except RuntimeError as e:
        print(f'ERROR: {e}')
    finally:
        if node is not None:
            # context may already be invalid after Ctrl-C; motor_control's watchdog stops anyway
            try:
                for _ in range(5):
                    node.send(*STOP)
            except Exception:
                pass
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    main()
