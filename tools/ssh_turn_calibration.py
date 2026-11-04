#!/usr/bin/env python3
"""Eight-step human-referenced turn calibration over an SSH terminal.

The fixed plan is CCW/CW 90°, 180°, 135°, and 360°. Each observation is stored
as its own table row. Valid observations also yield a time-weighted effective
angular speed per direction. No IMU data or floor profile is read or written.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import select
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

MAX_COMMAND = 0.5
DEFAULT_MAX_SECONDS = 5.0
HARD_MAX_SECONDS = 7.0
TURN_PLAN = (("ccw", 90.0), ("cw", 90.0),
             ("ccw", 180.0), ("cw", 180.0),
             ("ccw", 135.0), ("cw", 135.0),
             ("ccw", 360.0), ("cw", 360.0))
TABLE_FIELDS = (
    "trial_id", "step", "direction", "target_deg", "command",
    "command_seconds", "observed_deg", "error_deg", "effective_deg_s",
    "status", "stop_reason",
)


def turn_command(direction: str, magnitude: float) -> tuple[float, float]:
    return (-magnitude, magnitude) if direction == "ccw" else (magnitude, -magnitude)


def label_angle(observed: float, target: float, tolerance: float) -> str:
    error = observed - target
    if abs(error) <= tolerance:
        return "on_target"
    return "overshoot" if error > 0 else "undershoot"


def turn_rows(events: list[dict]) -> list[dict]:
    """Join stop and observation events, preserving every attempt as a row."""
    trials = {}
    rows = []
    for event in events:
        if event.get("event") == "stop":
            row = {
                "trial_id": event["trial_id"], "step": event["step"],
                "direction": event["direction"], "target_deg": event["target_deg"],
                "command": event["command"],
                "command_seconds": event["command_seconds"],
                "observed_deg": "", "error_deg": "", "effective_deg_s": "",
                "status": "awaiting_observation" if event["reason"] == "manual" else "rejected",
                "stop_reason": event["reason"],
            }
            trials[event["trial_id"]] = row
            rows.append(row)
        elif event.get("event") == "observation":
            row = trials.get(event["trial_id"])
            if row is None:
                continue
            angle = event.get("observed_deg")
            if angle is None:
                row["status"] = "invalid"
            else:
                row["observed_deg"] = angle
                row["error_deg"] = round(angle - row["target_deg"], 3)
                if row["command_seconds"] > 0:
                    row["effective_deg_s"] = round(angle / row["command_seconds"], 4)
                    row["status"] = "measured"
                else:
                    row["status"] = "invalid_duration"
    return rows


def write_turn_table(path: Path, events: list[dict]) -> None:
    """Replace the CSV atomically so an interrupted write keeps the old table."""
    temp_path = path.with_name(path.name + ".tmp")
    with temp_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=TABLE_FIELDS)
        writer.writeheader()
        writer.writerows(turn_rows(events))
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp_path, path)


def direction_speeds(events: list[dict]) -> dict:
    """Total observed angle / total command time, not steady-state gyro rate."""
    rows = turn_rows(events)
    result = {}
    for direction in ("ccw", "cw"):
        valid = [r for r in rows if r["direction"] == direction
                 and r["status"] == "measured"]
        angle = sum(float(r["observed_deg"]) for r in valid)
        seconds = sum(float(r["command_seconds"]) for r in valid)
        result[direction] = {
            "count": len(valid),
            "total_observed_deg": round(angle, 4),
            "total_command_seconds": round(seconds, 4),
            "effective_mean_deg_s": round(angle / seconds, 6) if seconds else None,
            "effective_mean_rad_s": round(math.radians(angle / seconds), 6)
            if seconds else None,
        }
    return result


def write_turn_summary(path: Path, events: list[dict]) -> None:
    session = next(e for e in events if e.get("event") == "session")
    summary = {
        "floor": session["floor"], "command": session["command"],
        "mode": session["mode"], "plan": session["plan"],
        "method": "sum(observed_deg) / sum(command_seconds), by direction; includes startup and coast",
        "directions": direction_speeds(events),
    }
    temp_path = path.with_name(path.name + ".tmp")
    with temp_path.open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp_path, path)


def append_event(path: Path, event: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(event, ensure_ascii=False, sort_keys=True) + "\n")
        stream.flush()
        os.fsync(stream.fileno())


def read_key(fd: int, timeout: float) -> bytes | None:
    readable, _, _ = select.select([fd], [], [], timeout)
    return os.read(fd, 64) if readable else None


def stop_and_log(
    publisher: MotorPublisher, path: Path, table_path: Path, events: list[dict], *,
    trial_id: int, step: int, direction: str, command: float, target: float,
    started_at: float, reason: str,
) -> dict:
    stopped_at = time.monotonic()
    publisher.publish(STOP)
    event = {
        "event": "stop", "trial_id": trial_id, "step": step,
        "utc": datetime.now(timezone.utc).isoformat(),
        "direction": direction, "command": command,
        "target_deg": target, "reason": reason,
        "command_seconds": round(stopped_at - started_at, 4),
    }
    append_event(path, event)
    events.append(event)
    write_turn_table(table_path, events)
    return event


def run(
    publisher: MotorPublisher, *, path: Path, floor: str, command: float,
    tolerance: float, max_seconds: float, live: bool,
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
    step_index = 0
    armed = False
    turning = False
    direction = ""
    target = 0.0
    started_at = 0.0

    def interrupt(_signum: int, _frame: object) -> None:
        raise KeyboardInterrupt

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        old_handlers[sig] = signal.signal(sig, interrupt)

    session = {
        "event": "session", "utc": datetime.now(timezone.utc).isoformat(),
        "floor": floor, "command": command, "plan": TURN_PLAN,
        "tolerance_deg": tolerance, "max_seconds": max_seconds,
        "mode": "live" if live else "dry_run",
    }
    append_event(path, session)
    events.append(session)
    write_turn_table(table_path, events)
    write_turn_summary(summary_path, events)
    print(f"Floor {floor}; {len(TURN_PLAN)}-step plan; motor command ±{command:g}")
    print(f"Raw events: {path}\nPer-attempt table: {table_path}\nSpeed summary: {summary_path}")
    print("g arm | a start CCW | d start CW | SPACE stop | x emergency stop | q quit")
    print(f"Each spin stops automatically after {max_seconds:g} s; never hold a key.")
    print("Use floor angle marks; report the final angle after coasting has settled.")
    try:
        tty.setcbreak(fd)
        print(f"Step 1/{len(TURN_PLAN)}: CCW 90°. Align start; ready for g.", flush=True)
        while True:
            ready = publisher.ready()
            if turning and (not ready or time.monotonic() - started_at >= max_seconds):
                reason = "motor_unavailable" if not ready else "timeout"
                event = stop_and_log(
                    publisher, path, table_path, events, trial_id=trial_id,
                    step=step_index + 1,
                    direction=direction, command=command, target=target,
                    started_at=started_at, reason=reason,
                )
                turning = armed = False
                print(f"\nSTOP ({reason}) after {event['command_seconds']:.3f} s; retry step {step_index + 1}.", flush=True)
            elif not ready and armed:
                armed = False
                print("\nMotor path unavailable; disarmed.", flush=True)

            data = read_key(fd, PUBLISH_SECONDS)
            if data == b"":
                if turning:
                    stop_and_log(publisher, path, table_path, events, trial_id=trial_id,
                                 step=step_index + 1,
                                 direction=direction, command=command, target=target,
                                 started_at=started_at, reason="terminal_eof")
                return
            for byte in data or b"":
                key = chr(byte).lower()
                if key in ("q", "\x1b", "\x04"):
                    if turning:
                        stop_and_log(publisher, path, table_path, events, trial_id=trial_id,
                                     step=step_index + 1,
                                     direction=direction, command=command, target=target,
                                     started_at=started_at, reason="quit")
                    return
                if key in (" ", "x"):
                    if turning:
                        event = stop_and_log(
                            publisher, path, table_path, events, trial_id=trial_id,
                            step=step_index + 1,
                            direction=direction, command=command, target=target,
                            started_at=started_at,
                            reason="manual" if key == " " else "emergency",
                        )
                        turning = armed = False
                        print(f"\nSTOP after {event['command_seconds']:.3f} s", flush=True)
                        if key == " ":
                            termios.tcsetattr(fd, termios.TCSADRAIN, original)
                            try:
                                raw = input(
                                    "After settling, observed turn in degrees "
                                    f"(positive magnitude, Enter={target:g}°, x=invalid): "
                                ).strip().lower()
                            finally:
                                tty.setcbreak(fd)
                            if raw == "x":
                                angle = None
                            elif raw == "":
                                angle = target
                            else:
                                try:
                                    angle = float(raw)
                                    if not math.isfinite(angle) or not 0 < angle <= 540:
                                        raise ValueError
                                except ValueError:
                                    angle = None
                                    print("Invalid angle; trial not accepted.", flush=True)
                            observation = {
                                "event": "observation", "trial_id": trial_id,
                                "utc": datetime.now(timezone.utc).isoformat(),
                                "observed_deg": angle,
                            }
                            append_event(path, observation)
                            events.append(observation)
                            write_turn_table(table_path, events)
                            write_turn_summary(summary_path, events)
                            if angle is not None and event["command_seconds"] > 0:
                                print(f"Result: {label_angle(angle, target, tolerance)}", flush=True)
                                step_index += 1
                        else:
                            print("Emergency stop; trial rejected.", flush=True)
                        termios.tcflush(fd, termios.TCIFLUSH)
                        if step_index == len(TURN_PLAN):
                            print(f"All {len(TURN_PLAN)} measured steps complete.", flush=True)
                            return
                        next_direction, next_target = TURN_PLAN[step_index]
                        print(f"Step {step_index + 1}/{len(TURN_PLAN)}: {next_direction.upper()} "
                              f"{next_target:g}°. Align start; press g.", flush=True)
                        break
                    else:
                        armed = False
                        publisher.publish(STOP)
                        print("\nSTOP; disarmed.", flush=True)
                    continue
                if key == "g" and not turning:
                    armed = ready
                    print("\nArmed." if armed else "\nMotor path unavailable; cannot arm.", flush=True)
                    continue
                if key in ("a", "d") and armed and not turning and ready:
                    requested = "ccw" if key == "a" else "cw"
                    direction, target = TURN_PLAN[step_index]
                    if requested != direction:
                        print(f"Expected {direction.upper()} for step {step_index + 1}; ignored.", flush=True)
                        continue
                    trial_id += 1
                    started_at = time.monotonic()
                    publisher.publish(turn_command(direction, command))
                    turning = True
                    print(f"\nStep {step_index + 1}/{len(TURN_PLAN)}, trial {trial_id}: "
                          f"{direction.upper()} {target:g}° started. SPACE to stop.", flush=True)
            if turning:
                publisher.publish(turn_command(direction, command))
            else:
                publisher.publish(STOP)
    except KeyboardInterrupt:
        if turning:
            stop_and_log(publisher, path, table_path, events, trial_id=trial_id,
                         step=step_index + 1,
                         direction=direction, command=command, target=target,
                         started_at=started_at, reason="interrupt")
    finally:
        for _ in range(5):
            publisher.publish(STOP)
            time.sleep(PUBLISH_SECONDS)
        termios.tcsetattr(fd, termios.TCSADRAIN, original)
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        speeds = direction_speeds(events)
        for name in ("ccw", "cw"):
            item = speeds[name]
            if item["effective_mean_deg_s"] is None:
                print(f"{name.upper()}: no measured trials; no speed estimate.", flush=True)
            else:
                print(f"{name.upper()}: {item['count']} measured trials; effective mean "
                      f"{item['effective_mean_deg_s']:.2f} deg/s "
                      f"({item['effective_mean_rad_s']:.4f} rad/s)", flush=True)
        print("Individual angles remain in the CSV; speed includes startup and coast.", flush=True)
        if not live:
            print("DRY RUN: these trials did not move the robot; do not use them for calibration.", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--floor", required=True, help="floor label, e.g. carpet2")
    parser.add_argument("--cmd", type=float, default=0.30, help="motor magnitude (default 0.30)")
    parser.add_argument("--tolerance-deg", type=float, default=5.0)
    parser.add_argument("--max-seconds", type=float, default=DEFAULT_MAX_SECONDS)
    parser.add_argument("--output", type=Path, help="JSONL output and adjacent CSV table")
    parser.add_argument("--live", action="store_true", help="publish motor commands (default dry-run)")
    args = parser.parse_args()
    if not re.fullmatch(r"[a-zA-Z0-9_-]+", args.floor):
        parser.error("floor must contain only letters, digits, underscore, or hyphen")
    if not 0 < args.cmd <= MAX_COMMAND:
        parser.error("cmd must be >0 and <=0.5")
    if not math.isfinite(args.tolerance_deg) or not 0 <= args.tolerance_deg < 90:
        parser.error("tolerance-deg must be >=0 and <90")
    if not 0 < args.max_seconds <= HARD_MAX_SECONDS:
        parser.error("max-seconds must be >0 and <=7")
    if not sys.stdin.isatty():
        parser.error("interactive terminal required; use ssh -tt")
    path = args.output or Path.home() / (
        f"turn_calibration_{args.floor}_{datetime.now():%Y%m%d_%H%M%S}.jsonl"
    )
    if path.exists():
        parser.error(f"output already exists: {path}; choose a new file")
    if path.with_suffix(".csv").exists():
        parser.error(f"table already exists: {path.with_suffix('.csv')}; choose a new file")
    if path.with_name(path.stem + "_summary.json").exists():
        parser.error("summary already exists; choose a new output file")
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
            tolerance=args.tolerance_deg,
            max_seconds=args.max_seconds, live=args.live)
    finally:
        publisher.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
