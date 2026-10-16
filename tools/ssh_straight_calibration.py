#!/usr/bin/env python3
"""Five human-referenced 1 m forward trials over an interactive SSH terminal.

Each trial starts on W and stops on Space at a measured 1 m floor mark. The
operator reports the settled final distance. All attempts are stored as table
rows; means are calculated only after five valid observations. No IMU or floor
profile is read or modified.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import signal
import sys
import termios
import time
import tty
from datetime import datetime, timezone
from pathlib import Path

from ssh_keyboard_teleop import (
    PUBLISH_SECONDS,
    STOP,
    DryRunPublisher,
    MotorPublisher,
    RosMotorPublisher,
)
from ssh_turn_calibration import append_event, read_key


REPETITIONS = 5
TARGET_METERS = 1.0
DEFAULT_COMMAND = 0.30
DEFAULT_MAX_SECONDS = 3.5
HARD_MAX_SECONDS = 5.0
TABLE_FIELDS = (
    "trial_id", "repetition", "target_m", "command", "command_seconds",
    "observed_m", "error_m", "status", "stop_reason",
)


def straight_rows(events: list[dict]) -> list[dict]:
    rows = []
    trials = {}
    for event in events:
        if event.get("event") == "stop":
            row = {
                "trial_id": event["trial_id"], "repetition": event["repetition"],
                "target_m": TARGET_METERS, "command": event["command"],
                "command_seconds": event["command_seconds"],
                "observed_m": "", "error_m": "",
                "status": "awaiting_observation" if event["reason"] == "manual" else "rejected",
                "stop_reason": event["reason"],
            }
            rows.append(row)
            trials[event["trial_id"]] = row
        elif event.get("event") == "observation":
            row = trials.get(event["trial_id"])
            if row is None:
                continue
            distance = event.get("observed_m")
            if distance is None:
                row["status"] = "invalid"
            else:
                row["observed_m"] = distance
                row["error_m"] = round(distance - TARGET_METERS, 4)
                row["status"] = ("measured" if row["command_seconds"] > 0
                                 else "invalid_duration")
    return rows


def write_table(path: Path, events: list[dict]) -> None:
    temp_path = path.with_name(path.name + ".tmp")
    with temp_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=TABLE_FIELDS)
        writer.writeheader()
        writer.writerows(straight_rows(events))
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp_path, path)


def averages(events: list[dict]) -> dict | None:
    measured = [r for r in straight_rows(events) if r["status"] == "measured"]
    if len(measured) != REPETITIONS:
        return None
    times = [float(r["command_seconds"]) for r in measured]
    distances = [float(r["observed_m"]) for r in measured]
    return {
        "count": REPETITIONS,
        "mean_command_seconds": sum(times) / REPETITIONS,
        "mean_observed_m": sum(distances) / REPETITIONS,
        "seconds_per_observed_meter": sum(times) / sum(distances),
    }


def write_summary(path: Path, events: list[dict]) -> None:
    session = next(e for e in events if e.get("event") == "session")
    measured_count = sum(r["status"] == "measured" for r in straight_rows(events))
    summary = {
        "floor": session["floor"], "command": session["command"],
        "mode": session["mode"], "target_m": TARGET_METERS,
        "measured_count": measured_count,
        "method": "five measured runs; mean time, mean final distance, total time / total distance",
        "result": averages(events),
    }
    temp_path = path.with_name(path.name + ".tmp")
    with temp_path.open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp_path, path)


def stop_and_log(
    publisher: MotorPublisher, path: Path, table_path: Path,
    events: list[dict], *, trial_id: int, repetition: int,
    command: float, started_at: float, reason: str,
) -> dict:
    stopped_at = time.monotonic()
    publisher.publish(STOP)
    event = {
        "event": "stop", "trial_id": trial_id, "repetition": repetition,
        "utc": datetime.now(timezone.utc).isoformat(),
        "command": command, "target_m": TARGET_METERS, "reason": reason,
        "command_seconds": round(stopped_at - started_at, 4),
    }
    append_event(path, event)
    events.append(event)
    write_table(table_path, events)
    return event


def run(
    publisher: MotorPublisher, *, path: Path, floor: str, command: float,
    max_seconds: float, live: bool,
) -> None:
    if not sys.stdin.isatty():
        raise RuntimeError("Interactive TTY required; connect with ssh -tt")
    fd = sys.stdin.fileno()
    original = termios.tcgetattr(fd)
    table_path = path.with_suffix(".csv")
    summary_path = path.with_name(path.stem + "_summary.json")
    old_handlers = {}
    events: list[dict] = []
    trial_id = 0
    completed = 0
    armed = False
    moving = False
    started_at = 0.0

    def interrupt(_signum: int, _frame: object) -> None:
        raise KeyboardInterrupt

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        old_handlers[sig] = signal.signal(sig, interrupt)

    session = {
        "event": "session", "utc": datetime.now(timezone.utc).isoformat(),
        "floor": floor, "command": command, "target_m": TARGET_METERS,
        "repetitions": REPETITIONS, "max_seconds": max_seconds,
        "mode": "live" if live else "dry_run",
    }
    append_event(path, session)
    events.append(session)
    write_table(table_path, events)
    write_summary(summary_path, events)
    print(f"Floor {floor}; five 1 m trials; motor command +{command:g}")
    print(f"Raw events: {path}\nPer-attempt table: {table_path}\nMean summary: {summary_path}")
    print("g arm | w start forward | SPACE stop | x emergency stop | q quit")
    print(f"Each run stops automatically after {max_seconds:g} s; never hold W.")
    print("Use a 1 m floor mark; measure the final distance after stopping.")
    try:
        tty.setcbreak(fd)
        print("Repetition 1/5: align start; ready for g.", flush=True)
        while True:
            ready = publisher.ready()
            if moving and (not ready or time.monotonic() - started_at >= max_seconds):
                reason = "motor_unavailable" if not ready else "timeout"
                event = stop_and_log(
                    publisher, path, table_path, events, trial_id=trial_id,
                    repetition=completed + 1, command=command,
                    started_at=started_at, reason=reason,
                )
                moving = armed = False
                print(f"\nSTOP ({reason}) after {event['command_seconds']:.3f} s; retry repetition {completed + 1}.", flush=True)
            elif not ready and armed:
                armed = False
                print("\nMotor path unavailable; disarmed.", flush=True)

            data = read_key(fd, PUBLISH_SECONDS)
            if data == b"":
                if moving:
                    stop_and_log(publisher, path, table_path, events,
                                 trial_id=trial_id, repetition=completed + 1,
                                 command=command, started_at=started_at,
                                 reason="terminal_eof")
                return
            for byte in data or b"":
                key = chr(byte).lower()
                if key in ("q", "\x1b", "\x04"):
                    if moving:
                        stop_and_log(publisher, path, table_path, events,
                                     trial_id=trial_id, repetition=completed + 1,
                                     command=command, started_at=started_at,
                                     reason="quit")
                    return
                if key in (" ", "x"):
                    if moving:
                        event = stop_and_log(
                            publisher, path, table_path, events,
                            trial_id=trial_id, repetition=completed + 1,
                            command=command, started_at=started_at,
                            reason="manual" if key == " " else "emergency",
                        )
                        moving = armed = False
                        print(f"\nSTOP after {event['command_seconds']:.3f} s", flush=True)
                        if key == " ":
                            termios.tcsetattr(fd, termios.TCSADRAIN, original)
                            try:
                                raw = input(
                                    "After settling, observed forward distance in meters "
                                    "(Enter=exactly 1.00 m, x=invalid): "
                                ).strip().lower()
                            finally:
                                tty.setcbreak(fd)
                            if raw == "x":
                                distance = None
                            elif raw == "":
                                distance = TARGET_METERS
                            else:
                                try:
                                    distance = float(raw)
                                    if not math.isfinite(distance) or not 0 < distance <= 5:
                                        raise ValueError
                                except ValueError:
                                    distance = None
                                    print("Invalid distance; trial not counted.", flush=True)
                            observation = {
                                "event": "observation", "trial_id": trial_id,
                                "utc": datetime.now(timezone.utc).isoformat(),
                                "observed_m": distance,
                            }
                            append_event(path, observation)
                            events.append(observation)
                            write_table(table_path, events)
                            write_summary(summary_path, events)
                            if distance is not None and event["command_seconds"] > 0:
                                completed += 1
                                print(f"Measured {distance:.3f} m; {completed}/5 complete.", flush=True)
                        else:
                            print("Emergency stop; trial rejected.", flush=True)
                        termios.tcflush(fd, termios.TCIFLUSH)
                        if completed == REPETITIONS:
                            print("Five measured repetitions complete.", flush=True)
                            return
                        print(f"Repetition {completed + 1}/5: reset to start; press g.", flush=True)
                        break
                    else:
                        armed = False
                        publisher.publish(STOP)
                        print("\nSTOP; disarmed.", flush=True)
                    continue
                if key == "g" and not moving:
                    armed = ready
                    print("\nArmed." if armed else "\nMotor path unavailable; cannot arm.", flush=True)
                    continue
                if key == "w" and armed and not moving and ready:
                    trial_id += 1
                    started_at = time.monotonic()
                    publisher.publish((command, command))
                    moving = True
                    print(f"\nRepetition {completed + 1}/5, trial {trial_id} started. SPACE to stop.", flush=True)
            if moving:
                publisher.publish((command, command))
            else:
                publisher.publish(STOP)
    except KeyboardInterrupt:
        if moving:
            stop_and_log(publisher, path, table_path, events,
                         trial_id=trial_id, repetition=completed + 1,
                         command=command, started_at=started_at,
                         reason="interrupt")
    finally:
        for _ in range(5):
            publisher.publish(STOP)
            time.sleep(PUBLISH_SECONDS)
        termios.tcsetattr(fd, termios.TCSADRAIN, original)
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        result = averages(events)
        if result is not None:
            print(f"5-trial mean command time: {result['mean_command_seconds']:.3f} s")
            print(f"5-trial mean final distance: {result['mean_observed_m']:.3f} m")
            print(f"Observed seconds per meter: {result['seconds_per_observed_meter']:.3f} s/m")
        else:
            print(f"Only {completed}/5 measured repetitions; no five-trial mean.")
        if not live:
            print("DRY RUN: these trials did not move the robot; do not use them for calibration.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--floor", required=True, help="floor label, e.g. carpet2")
    parser.add_argument("--cmd", type=float, default=DEFAULT_COMMAND)
    parser.add_argument("--max-seconds", type=float, default=DEFAULT_MAX_SECONDS)
    parser.add_argument("--output", type=Path, help="JSONL output and adjacent CSV table")
    parser.add_argument("--live", action="store_true", help="publish motor commands (default dry-run)")
    args = parser.parse_args()
    if not re.fullmatch(r"[a-zA-Z0-9_-]+", args.floor):
        parser.error("floor must contain only letters, digits, underscore, or hyphen")
    if not 0 < args.cmd <= 0.5:
        parser.error("cmd must be >0 and <=0.5")
    if not 0 < args.max_seconds <= HARD_MAX_SECONDS:
        parser.error("max-seconds must be >0 and <=5")
    if not sys.stdin.isatty():
        parser.error("interactive terminal required; use ssh -tt")
    path = args.output or Path.home() / (
        f"straight_calibration_{args.floor}_{datetime.now():%Y%m%d_%H%M%S}.jsonl"
    )
    if (path.exists() or path.with_suffix(".csv").exists()
            or path.with_name(path.stem + "_summary.json").exists()):
        parser.error("output, table, or summary already exists; choose a new file")
    if args.live:
        print("LIVE: do not run any other motor_commands publisher.", flush=True)
        try:
            publisher: MotorPublisher = RosMotorPublisher()
        except Exception as exc:
            print(f"Cannot start ROS motor publisher: {exc}", file=sys.stderr)
            return 1
    else:
        print("DRY RUN: no ROS import or motor commands.", flush=True)
        publisher = DryRunPublisher()
    try:
        run(publisher, path=path, floor=args.floor, command=args.cmd,
            max_seconds=args.max_seconds, live=args.live)
    finally:
        publisher.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
