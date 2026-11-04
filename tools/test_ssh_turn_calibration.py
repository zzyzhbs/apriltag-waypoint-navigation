"""Offline timing, safety, and PTY tests; no ROS or robot needed."""

import json
import csv
import os
import pty
import select
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from ssh_turn_calibration import (
    TURN_PLAN,
    direction_speeds,
    label_angle,
    stop_and_log,
    turn_rows,
    turn_command,
)


class FakePublisher:
    def __init__(self):
        self.commands = []

    def publish(self, command):
        self.commands.append(command)


class TurnLogicTests(unittest.TestCase):
    def test_commands_match_shared_keyboard_controller(self):
        self.assertEqual(turn_command("ccw", 0.30), (-0.30, 0.30))
        self.assertEqual(turn_command("cw", 0.30), (0.30, -0.30))

    def test_angle_classification(self):
        self.assertEqual(label_angle(94, 90, 5), "on_target")
        self.assertEqual(label_angle(98, 90, 5), "overshoot")
        self.assertEqual(label_angle(83, 90, 5), "undershoot")

    def test_fixed_eight_step_plan(self):
        self.assertEqual(TURN_PLAN, (("ccw", 90.0), ("cw", 90.0),
                                     ("ccw", 180.0), ("cw", 180.0),
                                     ("ccw", 135.0), ("cw", 135.0),
                                     ("ccw", 360.0), ("cw", 360.0)))

    def test_each_angle_and_attempt_is_its_own_table_row(self):
        events = [
            {"event": "stop", "trial_id": 1, "step": 1, "direction": "ccw", "target_deg": 90,
             "command": 0.3, "reason": "manual", "command_seconds": 0.8},
            {"event": "observation", "trial_id": 1, "observed_deg": 88},
            {"event": "stop", "trial_id": 2, "step": 2, "direction": "cw", "target_deg": 90,
             "command": 0.3, "reason": "timeout", "command_seconds": 2.5},
            {"event": "stop", "trial_id": 3, "step": 2, "direction": "cw", "target_deg": 90,
             "command": 0.3, "reason": "manual", "command_seconds": 1.3},
            {"event": "observation", "trial_id": 3, "observed_deg": 105},
        ]
        rows = turn_rows(events)
        self.assertEqual(len(rows), 3)
        self.assertEqual([r["observed_deg"] for r in rows], [88, "", 105])
        self.assertEqual([r["error_deg"] for r in rows], [-2, "", 15])
        self.assertEqual([r["status"] for r in rows], ["measured", "rejected", "measured"])

    def test_direction_speed_uses_total_angle_over_total_time(self):
        events = [
            {"event": "stop", "trial_id": 1, "step": 1, "direction": "ccw",
             "target_deg": 90, "command": 0.3, "command_seconds": 1.0,
             "reason": "manual"},
            {"event": "observation", "trial_id": 1, "observed_deg": 90},
            {"event": "stop", "trial_id": 2, "step": 3, "direction": "ccw",
             "target_deg": 180, "command": 0.3, "command_seconds": 3.0,
             "reason": "manual"},
            {"event": "observation", "trial_id": 2, "observed_deg": 180},
            {"event": "stop", "trial_id": 3, "step": 2, "direction": "cw",
             "target_deg": 90, "command": 0.3, "command_seconds": 2.0,
             "reason": "manual"},
            {"event": "observation", "trial_id": 3, "observed_deg": 100},
        ]
        speeds = direction_speeds(events)
        self.assertEqual(speeds["ccw"]["count"], 2)
        self.assertEqual(speeds["ccw"]["effective_mean_deg_s"], 67.5)
        self.assertEqual(speeds["cw"]["effective_mean_deg_s"], 50.0)

    def test_stop_command_precedes_disk_logging(self):
        publisher = FakePublisher()
        events = []
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "trials.jsonl"
            table_path = path.with_suffix(".csv")
            with patch("ssh_turn_calibration.time.monotonic", return_value=11.25):
                event = stop_and_log(
                    publisher, path, table_path, events, trial_id=1, step=2, direction="cw",
                    command=0.3, target=90, started_at=10.0, reason="manual",
                )
            self.assertEqual(publisher.commands, [(0.0, 0.0)])
            self.assertEqual(event["command_seconds"], 1.25)
            self.assertEqual(json.loads(path.read_text())["event"], "stop")
            with table_path.open(newline="") as stream:
                self.assertEqual(list(csv.DictReader(stream))[0]["status"], "awaiting_observation")


class TerminalDryRunTests(unittest.TestCase):
    def test_fixed_eight_step_flow_preserves_all_angles(self):
        with tempfile.TemporaryDirectory() as tmp:
            output_path = Path(tmp) / "six.jsonl"
            master, slave = pty.openpty()
            script = Path(__file__).with_name("ssh_turn_calibration.py")
            process = subprocess.Popen(
                [sys.executable, str(script), "--floor", "carpet2",
                 "--output", str(output_path)],
                stdin=slave, stdout=slave, stderr=slave,
            )
            os.close(slave)
            output = bytearray()
            cursor = 0

            def read_until(marker):
                nonlocal cursor
                deadline = time.monotonic() + 4.0
                while output.find(marker, cursor) < 0:
                    self.assertLess(time.monotonic(), deadline, output)
                    readable, _, _ = select.select([master], [], [], 0.1)
                    if readable:
                        output.extend(os.read(master, 4096))
                cursor = output.find(marker, cursor) + len(marker)

            try:
                read_until(b"ready for g")
                os.write(master, b"gd")
                read_until(b"Expected CCW for step 1; ignored")
                for step, (direction, target) in enumerate(TURN_PLAN, start=1):
                    key = b"a" if direction == "ccw" else b"d"
                    os.write(master, b"g" + key)
                    read_until(f"trial {step}:".encode())
                    time.sleep(0.02)
                    os.write(master, b" ")
                    read_until(b"observed turn in degrees")
                    os.write(master, f"{target - 1:g}\n".encode())
                    if step < len(TURN_PLAN):
                        read_until(f"Step {step + 1}/{len(TURN_PLAN)}".encode())
                    else:
                        read_until(b"All 8 measured steps complete")
                self.assertEqual(process.wait(timeout=3), 0, output)
                with output_path.with_suffix(".csv").open(newline="") as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(len(rows), 8)
                self.assertEqual([r["direction"] for r in rows],
                                 [direction for direction, _ in TURN_PLAN])
                self.assertEqual([float(r["target_deg"]) for r in rows],
                                 [target for _, target in TURN_PLAN])
                self.assertEqual([float(r["observed_deg"]) for r in rows],
                                 [target - 1 for _, target in TURN_PLAN])
                self.assertTrue(all(r["status"] == "measured" for r in rows))
                summary_path = output_path.with_name("six_summary.json")
                summary = json.loads(summary_path.read_text())
                self.assertEqual(summary["directions"]["ccw"]["count"], 4)
                self.assertEqual(summary["directions"]["cw"]["count"], 4)
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=2)
                os.close(master)

    def test_turn_stop_observe_and_persist(self):
        with tempfile.TemporaryDirectory() as tmp:
            output_path = Path(tmp) / "turns.jsonl"
            master, slave = pty.openpty()
            script = Path(__file__).with_name("ssh_turn_calibration.py")
            process = subprocess.Popen(
                [sys.executable, str(script), "--floor", "carpet2",
                 "--output", str(output_path)],
                stdin=slave, stdout=slave, stderr=slave,
            )
            os.close(slave)
            output = bytearray()

            def read_until(marker):
                deadline = time.monotonic() + 4.0
                while marker not in output:
                    self.assertLess(time.monotonic(), deadline, output)
                    readable, _, _ = select.select([master], [], [], 0.1)
                    if readable:
                        output.extend(os.read(master, 4096))

            try:
                read_until(b"ready for g")
                os.write(master, b"ga")
                read_until(b"started. SPACE to stop")
                time.sleep(0.15)
                os.write(master, b" ")
                read_until(b"observed turn in degrees")
                os.write(master, b"91\n")
                read_until(b"Step 2/8")
                os.write(master, b"q")
                self.assertEqual(process.wait(timeout=3), 0, output)
                events = [json.loads(line) for line in output_path.read_text().splitlines()]
                self.assertEqual([e["event"] for e in events], ["session", "stop", "observation"])
                self.assertEqual(events[0]["floor"], "carpet2")
                self.assertEqual(events[0]["mode"], "dry_run")
                self.assertEqual(events[1]["reason"], "manual")
                self.assertEqual(events[1]["direction"], "ccw")
                self.assertGreater(events[1]["command_seconds"], 0)
                self.assertEqual(events[2]["observed_deg"], 91.0)
                with output_path.with_suffix(".csv").open(newline="") as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(len(rows), 1)
                self.assertEqual(rows[0]["status"], "measured")
                self.assertEqual(rows[0]["step"], "1")
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=2)
                os.close(master)

    def test_timeout_rejects_trial_without_observation(self):
        with tempfile.TemporaryDirectory() as tmp:
            output_path = Path(tmp) / "timeout.jsonl"
            master, slave = pty.openpty()
            script = Path(__file__).with_name("ssh_turn_calibration.py")
            process = subprocess.Popen(
                [sys.executable, str(script), "--floor", "carpet2",
                 "--max-seconds", "0.12", "--output", str(output_path)],
                stdin=slave, stdout=slave, stderr=slave,
            )
            os.close(slave)
            output = bytearray()
            try:
                deadline = time.monotonic() + 3.0
                while b"ready for g" not in output:
                    self.assertLess(time.monotonic(), deadline, output)
                    readable, _, _ = select.select([master], [], [], 0.1)
                    if readable:
                        output.extend(os.read(master, 4096))
                os.write(master, b"ga")
                while b"retry step 1" not in output:
                    self.assertLess(time.monotonic(), deadline, output)
                    readable, _, _ = select.select([master], [], [], 0.1)
                    if readable:
                        output.extend(os.read(master, 4096))
                os.write(master, b"q")
                self.assertEqual(process.wait(timeout=3), 0, output)
                events = [json.loads(line) for line in output_path.read_text().splitlines()]
                self.assertEqual(len(events), 2)
                self.assertEqual(events[1]["reason"], "timeout")
                self.assertEqual(events[1]["direction"], "ccw")
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=2)
                os.close(master)


if __name__ == "__main__":
    unittest.main()
