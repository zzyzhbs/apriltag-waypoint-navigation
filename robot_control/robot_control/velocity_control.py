"""HW1 open-loop waypoint runner. motor_control must be running.

ros2 run robot_control velocity_control --ros-args -p floor:=wood [-p dry_run:=true]
"""
import csv
import math
import os
import time
from datetime import datetime

from robot_control import ol_node
from robot_control.ol_model import DEFAULT_CALIB_DIR, Calibration, ModelError
from robot_control.ol_planner import format_plan, load_waypoints, plan, ready

DEFAULT_WAYPOINTS = '~/ros2_ws/src/CSE276A/HW1/waypoints.txt'
DEFAULT_LOG_DIR = '~/ros2_ws/src/CSE276A/HW1/logs'
COUNTDOWN_S = 3


class VelocityControlNode(ol_node.MotorNode):
    def __init__(self):
        super().__init__('velocity_control_node')
        self.floor = self.param('floor', '')
        calib_dir = self.param('calib_dir', DEFAULT_CALIB_DIR)
        wp_path = os.path.expanduser(self.param('waypoints', DEFAULT_WAYPOINTS))
        self.log_dir = os.path.expanduser(self.param('log_dir', DEFAULT_LOG_DIR))
        self.dry_run = bool(self.param('dry_run', False))
        self.wp_pause_s = self.float_param('wp_pause_s', 1.0)
        self.run_ = self.runner(self.float_param('settle_s', 1.0))

        self.calib = Calibration.for_floor(calib_dir, self.floor)
        self.legs = plan(load_waypoints(wp_path), self.calib,
                         drive_cmd=self.float_param('drive_cmd', 0.3),
                         spin_cmd=self.float_param('spin_cmd', 0.35),
                         pos_tol=self.float_param('pos_tol', 0.02),
                         min_turn_deg=self.float_param('min_turn_deg', 2.0))
        print(f'calibration: {self.calib.path}\nwaypoints: {wp_path}')
        print(format_plan(self.legs, self.calib.trim))

    def run(self):
        if not ready(self.legs):
            raise ModelError('plan has uncalibrated steps (see !! above)')
        if self.dry_run:
            return
        os.makedirs(self.log_dir, exist_ok=True)
        path = os.path.join(self.log_dir, f'run_{self.floor}_{datetime.now():%Y%m%d_%H%M%S}.csv')
        for i in range(COUNTDOWN_S, 0, -1):
            print(f'starting in {i}...')
            time.sleep(1.0)
        trim = self.calib.trim
        with open(path, 'w', newline='') as f:
            log = csv.writer(f)
            log.writerow(['wp', 'kind', 'dir', 'amount', 'cmd', 'L', 'R', 'T',
                          't_start', 't_stop'])
            t_ref = time.monotonic()
            for leg in self.legs:
                for s in leg.steps:
                    print(f'wp{leg.index}: {s.describe()}')
                    (L, R), (t0, t1) = self.run_.run_step(
                        s.kind, s.dir, s.cmd, s.T, trim if s.kind == 'drive' else 0.0)
                    log.writerow([leg.index, s.kind, s.dir, f'{s.amount:.4f}', s.cmd,
                                  f'{L:.4f}', f'{R:.4f}', f'{s.T:.3f}',
                                  f'{t0 - t_ref:.3f}', f'{t1 - t_ref:.3f}'])
                    f.flush()
                gx, gy, gth = leg.goal
                print(f'>>> Reached waypoint {leg.index} '
                      f'({gx:g}, {gy:g}, {math.degrees(gth):.0f} deg)')
                time.sleep(self.wp_pause_s)
        print(f'done; log: {path}')


def main(args=None):
    ol_node.init(args)
    node = None
    try:
        node = VelocityControlNode()
        node.run()
    except (ModelError, ValueError, OSError) as e:
        print(f'ERROR: {e}')
    except KeyboardInterrupt:
        print('\nAborted.')
    finally:
        ol_node.shutdown(node)
    return 0


if __name__ == '__main__':
    main()
