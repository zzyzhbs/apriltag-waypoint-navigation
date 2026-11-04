"""Safety-state and dry-run terminal tests that do not need ROS or a robot."""

import os
import pty
import select
import subprocess
import sys
import time
import unittest
from pathlib import Path

from ssh_keyboard_teleop import KEY_COMMANDS, PULSE_SECONDS, STOP, PulseState


class PulseStateTests(unittest.TestCase):
    def test_turn_command_is_symmetric_and_below_motor_limit(self):
        self.assertEqual(KEY_COMMANDS["a"], (-0.30, 0.30))
        self.assertEqual(KEY_COMMANDS["d"], (0.30, -0.30))

    def test_disarmed_keys_do_not_move(self):
        state = PulseState()
        state.accept("w", 1.0, motor_ready=True)
        self.assertEqual(state.current(1.01, motor_ready=True), STOP)

    def test_one_key_expires_without_release_event(self):
        state = PulseState()
        state.accept("g", 1.0, motor_ready=True)
        state.accept("w", 2.0, motor_ready=True)
        self.assertEqual(state.current(2.01, motor_ready=True), KEY_COMMANDS["w"])
        self.assertEqual(
            state.current(2.0 + PULSE_SECONDS, motor_ready=True), STOP
        )

    def test_repeated_key_extends_only_from_last_event(self):
        state = PulseState()
        state.accept("g", 1.0, motor_ready=True)
        state.accept("a", 2.0, motor_ready=True)
        state.accept("a", 2.1, motor_ready=True)
        self.assertEqual(state.current(2.2, motor_ready=True), KEY_COMMANDS["a"])
        self.assertEqual(state.current(2.31, motor_ready=True), STOP)

    def test_stop_disarms_and_requires_new_arm(self):
        state = PulseState()
        state.accept("g", 1.0, motor_ready=True)
        state.accept("d", 2.0, motor_ready=True)
        state.accept(" ", 2.01, motor_ready=True)
        state.accept("d", 2.02, motor_ready=True)
        self.assertEqual(state.current(2.03, motor_ready=True), STOP)
        self.assertFalse(state.armed)

    def test_discovery_loss_disarms(self):
        state = PulseState()
        state.accept("g", 1.0, motor_ready=True)
        state.accept("s", 2.0, motor_ready=True)
        self.assertEqual(state.current(2.01, motor_ready=False), STOP)
        self.assertFalse(state.armed)

    def test_cannot_arm_without_motor_controller(self):
        state = PulseState()
        state.accept("g", 1.0, motor_ready=False)
        self.assertFalse(state.armed)

    def test_quit_stops(self):
        state = PulseState()
        state.accept("g", 1.0, motor_ready=True)
        state.accept("w", 2.0, motor_ready=True)
        state.accept("q", 2.01, motor_ready=True)
        self.assertTrue(state.quit_requested)
        self.assertEqual(state.current(2.02, motor_ready=True), STOP)

    def test_unrecognized_and_uppercase_keys_do_not_move(self):
        state = PulseState()
        state.accept("g", 1.0, motor_ready=True)
        state.accept("W", 2.0, motor_ready=True)
        self.assertEqual(state.current(2.01, motor_ready=True), STOP)


class TerminalDryRunTests(unittest.TestCase):
    def test_mac_keyboard_bytes_reach_dry_run_and_expire(self):
        master, slave = pty.openpty()
        script = Path(__file__).with_name("ssh_keyboard_teleop.py")
        process = subprocess.Popen(
            [sys.executable, str(script)],
            stdin=slave,
            stdout=slave,
            stderr=slave,
        )
        os.close(slave)
        try:
            output = bytearray()
            ready_deadline = time.monotonic() + 3.0
            while b"Do not run any other publisher" not in output:
                self.assertLess(time.monotonic(), ready_deadline, output)
                readable, _, _ = select.select([master], [], [], 0.1)
                if readable:
                    output.extend(os.read(master, 4096))
            os.write(master, b"gw")
            time.sleep(PULSE_SECONDS + 0.2)
            os.write(master, b"q")
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                readable, _, _ = select.select([master], [], [], 0.1)
                if readable:
                    try:
                        output.extend(os.read(master, 4096))
                    except OSError:
                        break
                if process.poll() is not None and not readable:
                    break
            self.assertEqual(process.wait(timeout=2), 0)
            self.assertIn(b"DRY RUN", output)
            self.assertIn(b"L=+0.16 R=+0.16", output)
            self.assertIn(b"L=+0.00 R=+0.00", output)
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=2)
            os.close(master)


if __name__ == "__main__":
    unittest.main()
