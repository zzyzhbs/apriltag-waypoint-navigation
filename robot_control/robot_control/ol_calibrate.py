"""Open-loop calibration tool. Needs stdin (use ros2 run); motor_control must be running.

ros2 run robot_control ol_calibrate --ros-args -p floor:=wood -p mode:=drive \
    -p cmds:="[0.3]" -p durations:="[2.0, 3.0, 4.0, 5.0]" -p repeats:=2
ros2 run robot_control ol_calibrate --ros-args -p floor:=wood -p mode:=spin \
    -p dir:=both -p cmds:="[0.35]" -p durations:="[0.5, 1.0, 1.5, 2.0]" -p repeats:=2
ros2 run robot_control ol_calibrate --ros-args -p floor:=wood -p mode:=verify
ros2 run robot_control ol_calibrate --ros-args -p floor:=wood -p mode:=fit
"""
import itertools
import random

from robot_control import ol_node
from robot_control.ol_model import (
    DEFAULT_CALIB_DIR, DIRS, Calibration, ModelError, format_fit,
)

DEFAULT_DURATIONS = {'drive': [2.0, 3.0, 4.0, 5.0], 'spin': [0.5, 1.0, 1.5, 2.0]}
VERIFY = [('drive', 'fwd', 1.0), ('drive', 'fwd', 1.414),
          ('spin', 'ccw', 90.0), ('spin', 'cw', 90.0), ('spin', 'ccw', 180.0)]


class Redo(Exception):
    pass


class Skip(Exception):
    pass


def wait_start(text):
    a = input(f'\n{text}\n  Align, clear the area, Enter = start, s = skip: ').strip().lower()
    if a == 's':
        raise Skip


def ask_num(text):
    while True:
        a = input(f'  {text} (r = redo, s = skip): ').strip().lower()
        if a == 'r':
            raise Redo
        if a == 's':
            raise Skip
        try:
            return float(a)
        except ValueError:
            print('  Please enter a number.')


class CalibrateNode(ol_node.MotorNode):
    def __init__(self):
        super().__init__('ol_calibrate_node')
        floor = self.param('floor', '')
        self.mode = self.param('mode', 'fit')
        calib_dir = self.param('calib_dir', DEFAULT_CALIB_DIR)
        self.cmds = self.floats_param('cmds', [0.3 if self.mode == 'drive' else 0.35])
        self.durations = self.floats_param('durations', DEFAULT_DURATIONS.get(self.mode, [1.0]))
        self.repeats = int(self.param('repeats', 1))
        self.dir = self.param('dir', 'both')
        self.drive_cmd = self.float_param('drive_cmd', 0.3)
        self.spin_cmd = self.float_param('spin_cmd', 0.35)
        self.run_ = self.runner(self.float_param('settle_s', 1.0))
        self.calib = Calibration.for_floor(calib_dir, floor, create=True)
        print(f'calibration file: {self.calib.path}')

    def trial(self, title, execute, measure, store):
        """prompt -> execute -> measure -> store; r redoes, s skips."""
        while True:
            try:
                wait_start(title)
                execute()
                store(measure())
                return True
            except Redo:
                continue
            except Skip:
                print('  skipped')
                return False

    # ---------- modes ----------

    def run_drive(self):
        jobs = list(itertools.product(self.cmds, self.durations, range(self.repeats)))
        random.shuffle(jobs)
        for i, (c, T, _) in enumerate(jobs, 1):
            def measure():
                x = ask_num('x along the line (m)')
                y = ask_num('y lateral offset (cm, left +)') / 100.0
                return x, y

            def store(m):
                x, y = m
                self.calib.add(self.calib.drive_trials(), {
                    'cmd': c, 'T': T, 'x': round(x, 4), 'y': round(y, 4)})
                print('  saved')

            self.trial(f'[{i}/{len(jobs)}] drive cmd {c:g} T {T:g} s (~{c * T:.2f} m)',
                       lambda: self.run_.run_step('drive', 'fwd', c, T),
                       measure, store)
            self.level_done(jobs[i:], c, 'drive')

    def run_spin(self):
        dirs = DIRS if self.dir == 'both' else (self.dir,)
        if any(d not in DIRS for d in dirs):
            raise ModelError(f'dir must be ccw, cw or both, got {self.dir!r}')
        jobs = list(itertools.product(dirs, self.cmds, self.durations, range(self.repeats)))
        random.shuffle(jobs)
        for i, (d, c, T, _) in enumerate(jobs, 1):
            def store(a):
                self.calib.add(self.calib.spin_trials(d), {'cmd': c, 'T': T, 'angle': abs(a)})
                print('  saved')

            self.trial(f'[{i}/{len(jobs)}] spin {d} cmd {c:g} T {T:g} s',
                       lambda: self.run_.run_step('spin', d, c, T),
                       lambda: ask_num('angle turned (deg, positive, may exceed 360)'),
                       store)
            self.level_done([j[1:] for j in jobs[i:] if j[0] == d], c, 'spin', d)

    def level_done(self, remaining, c, kind, d=None):
        """Print the level's fit once its last trial has run."""
        if any(abs(j[0] - c) < 1e-9 for j in remaining):
            return
        fits = self.calib.drive_fits() if kind == 'drive' else self.calib.spin_fits(d)
        if c in fits:
            name = kind if d is None else f'spin {d}'
            print('  ' + format_fit(name, c, fits[c], 'm' if kind == 'drive' else 'deg'))

    def run_fit(self):
        for c, f in sorted(self.calib.drive_fits().items()):
            print(format_fit('drive', c, f, 'm'))
        for d in DIRS:
            for c, f in sorted(self.calib.spin_fits(d).items()):
                print(format_fit(f'spin {d}', c, f, 'deg'))

    def run_verify(self):
        for kind, d, target in VERIFY:
            c = self.drive_cmd if kind == 'drive' else self.spin_cmd
            if kind == 'drive':
                T, warn = self.calib.drive_time(target, c)
            else:
                T, warn = self.calib.spin_time(d, target, c)
            rec = {'kind': kind, 'dir': d, 'target': target, 'cmd': c, 'T': round(T, 3)}

            def measure():
                if kind == 'drive':
                    return {'x': ask_num('x along the line (m)'),
                            'y': ask_num('y lateral offset (cm, left +)') / 100.0}
                return {'angle': abs(ask_num('angle turned (deg)'))}

            def store(m):
                self.calib.add(self.calib.data['verify'], {**rec, **m})
                got = m.get('x', m.get('angle'))
                print(f'  saved; error {got - target:+.3f}')

            self.trial(f'verify {kind} {d} {target:g} -> cmd {c:g} T {T:.2f} s'
                       + (f'  !! {warn}' if warn else ''),
                       lambda: self.run_.run_step(kind, d, c, T),
                       measure, store)

    def run(self):
        modes = {'drive': self.run_drive, 'spin': self.run_spin,
                 'fit': self.run_fit, 'verify': self.run_verify}
        if self.mode not in modes:
            raise ModelError(f'mode must be one of {list(modes)}, got {self.mode!r}')
        modes[self.mode]()


def main(args=None):
    ol_node.init(args)
    node = None
    try:
        node = CalibrateNode()
        node.run()
    except ModelError as e:
        print(f'ERROR: {e}')
    except (KeyboardInterrupt, EOFError):
        print('\nAborted.')
    finally:
        ol_node.shutdown(node)
    return 0


if __name__ == '__main__':
    main()
