"""Offline command mapping and stop-path tests; no ROS or robot required."""

import contextlib
import io
import unittest

from ssh_timed_motion import STOP, main, motor_command, run_motion


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def clock(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakePublisher:
    def __init__(self, readiness=None):
        self.commands = []
        self.readiness = iter(readiness) if readiness is not None else None
        self.last_ready = True

    def ready(self):
        if self.readiness is not None:
            self.last_ready = next(self.readiness, self.last_ready)
        return self.last_ready

    def publish(self, command):
        self.commands.append(command)


class TimedMotionTests(unittest.TestCase):
    def test_direction_mapping(self):
        self.assertEqual(motor_command("turn", "cw", 0.3), (0.3, -0.3))
        self.assertEqual(motor_command("turn", "ccw", 0.3), (-0.3, 0.3))
        self.assertEqual(motor_command("forward", None, 0.3), (0.3, 0.3))

    def test_normal_motion_is_bounded_and_ends_with_stop(self):
        ticker = FakeClock()
        publisher = FakePublisher()
        elapsed = run_motion(
            publisher, (0.3, 0.3), 0.12,
            clock=ticker.clock, sleep=ticker.sleep,
        )
        self.assertAlmostEqual(elapsed, 0.12)
        self.assertEqual(publisher.commands[:-1], [(0.3, 0.3)] * 3)
        self.assertEqual(publisher.commands[-1], STOP)

    def test_motor_path_loss_stops_immediately(self):
        ticker = FakeClock()
        publisher = FakePublisher([True, True, False])
        with self.assertRaisesRegex(RuntimeError, "Motor path changed"):
            run_motion(publisher, (-0.3, 0.3), 1.0,
                       clock=ticker.clock, sleep=ticker.sleep)
        self.assertEqual(publisher.commands[-1], STOP)
        self.assertEqual(publisher.commands.count((-0.3, 0.3)), 1)

    def test_unavailable_motor_never_receives_nonzero(self):
        ticker = FakeClock()
        publisher = FakePublisher([False])
        with self.assertRaisesRegex(RuntimeError, "Motor controller unavailable"):
            run_motion(publisher, (0.3, -0.3), 1.0,
                       clock=ticker.clock, sleep=ticker.sleep)
        self.assertTrue(publisher.commands)
        self.assertTrue(all(command == STOP for command in publisher.commands))

    def test_dry_run_exits_without_ros_or_motion(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(main(["turn", "cw", "1.5"]), 0)
        self.assertIn("DRY RUN", output.getvalue())
        self.assertIn("L=+0.30 R=-0.30", output.getvalue())

    def test_invalid_duration_is_rejected(self):
        for argv in (("forward", "0"), ("forward", "6"),
                     ("turn", "cw", "8"), ("turn", "ccw", "nan")):
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    main(list(argv))


if __name__ == "__main__":
    unittest.main()
