"""Offline tests for five human-referenced one-meter forward trials."""

import csv
import json
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

from ssh_straight_calibration import averages, stop_and_log, straight_rows


class FakePublisher:
    def __init__(self):
        self.commands = []

    def publish(self, command):
        self.commands.append(command)


class StraightLogicTests(unittest.TestCase):
    def test_five_measurements_average_but_raw_rows_remain(self):
        events = []
        for trial_id, (seconds, distance) in enumerate(
            ((2.0, 0.95), (2.2, 1.0), (2.4, 1.05),
             (2.6, 1.0), (2.8, 1.0)), start=1
        ):
            events.append({
                "event": "stop", "trial_id": trial_id, "repetition": trial_id,
                "command": 0.3, "command_seconds": seconds, "reason": "manual",
            })
            events.append({"event": "observation", "trial_id": trial_id,
                           "observed_m": distance})
        rows = straight_rows(events)
        self.assertEqual([row["observed_m"] for row in rows],
                         [0.95, 1.0, 1.05, 1.0, 1.0])
        result = averages(events)
        self.assertEqual(result["count"], 5)
        self.assertAlmostEqual(result["mean_command_seconds"], 2.4)
        self.assertAlmostEqual(result["mean_observed_m"], 1.0)
        self.assertAlmostEqual(result["seconds_per_observed_meter"], 2.4)

    def test_no_mean_before_five_valid_measurements(self):
        events = [
            {"event": "stop", "trial_id": 1, "repetition": 1,
             "command": 0.3, "command_seconds": 2.4, "reason": "timeout"},
            {"event": "stop", "trial_id": 2, "repetition": 1,
             "command": 0.3, "command_seconds": 2.5, "reason": "manual"},
            {"event": "observation", "trial_id": 2, "observed_m": 1.0},
        ]
        self.assertEqual([r["status"] for r in straight_rows(events)],
                         ["rejected", "measured"])
        self.assertIsNone(averages(events))

    def test_stop_precedes_writing_result(self):
        publisher = FakePublisher()
        events = []
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "straight.jsonl"
            with patch("ssh_straight_calibration.time.monotonic", return_value=12.0):
                event = stop_and_log(
                    publisher, path, path.with_suffix(".csv"), events,
                    trial_id=1, repetition=1, command=0.3,
                    started_at=10.0, reason="manual",
                )
            self.assertEqual(publisher.commands, [(0.0, 0.0)])
            self.assertEqual(event["command_seconds"], 2.0)
            self.assertEqual(json.loads(path.read_text())["event"], "stop")


class TerminalDryRunTests(unittest.TestCase):
    def test_five_repetitions_complete_and_table_keeps_each(self):
        with tempfile.TemporaryDirectory() as tmp:
            output_path = Path(tmp) / "straight.jsonl"
            master, slave = pty.openpty()
            script = Path(__file__).with_name("ssh_straight_calibration.py")
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
                for repetition in range(1, 6):
                    os.write(master, b"gw")
                    read_until(f"trial {repetition} started".encode())
                    time.sleep(0.05)
                    os.write(master, b" ")
                    read_until(b"observed forward distance")
                    os.write(master, f"{0.95 + 0.01 * repetition:.2f}\n".encode())
                    if repetition < 5:
                        read_until(f"Repetition {repetition + 1}/5: reset".encode())
                    else:
                        read_until(b"Five measured repetitions complete")
                self.assertEqual(process.wait(timeout=3), 0, output)
                with output_path.with_suffix(".csv").open(newline="") as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(len(rows), 5)
                self.assertEqual([r["repetition"] for r in rows],
                                 ["1", "2", "3", "4", "5"])
                self.assertEqual([r["observed_m"] for r in rows],
                                 ["0.96", "0.97", "0.98", "0.99", "1.0"])
                self.assertTrue(all(r["status"] == "measured" for r in rows))
                summary = json.loads(
                    output_path.with_name("straight_summary.json").read_text()
                )
                self.assertEqual(summary["measured_count"], 5)
                self.assertAlmostEqual(summary["result"]["mean_observed_m"], 0.98)
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=2)
                os.close(master)


if __name__ == "__main__":
    unittest.main()
