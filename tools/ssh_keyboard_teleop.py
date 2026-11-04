#!/usr/bin/env python3
"""Conservative keyboard teleop over an interactive SSH terminal.

Run this file on the robot through ``ssh -tt``. The Mac keyboard reaches the
robot as terminal bytes; no GUI, pynput, ROS package install, or floor profile
is needed. Dry-run is the default. ``--live`` is required to publish ROS motor
commands, and ``g`` must then be pressed to arm.

Each movement key authorizes only a short pulse. Holding a key may generate
repeat events, but a lost terminal or key-release event cannot leave a motor
command latched indefinitely.
"""

from __future__ import annotations

import argparse
import os
import select
import signal
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Protocol


STOP = (0.0, 0.0)
FORWARD_CMD = 0.16
# Older motor trials barely turned at 0.20 but turned repeatably at 0.30.
# This is a conservative open-loop command, not a carpet2 turn calibration.
TURN_CMD = 0.30
PULSE_SECONDS = 0.20
PUBLISH_SECONDS = 0.05
KEY_COMMANDS = {
    "w": (FORWARD_CMD, FORWARD_CMD),
    "s": (-FORWARD_CMD, -FORWARD_CMD),
    "a": (-TURN_CMD, TURN_CMD),
    "d": (TURN_CMD, -TURN_CMD),
}


@dataclass
class PulseState:
    armed: bool = False
    quit_requested: bool = False
    command: tuple[float, float] = STOP
    expires_at: float = 0.0

    def disarm(self) -> None:
        self.armed = False
        self.command = STOP
        self.expires_at = 0.0

    def accept(self, key: str, now: float, motor_ready: bool) -> str | None:
        if key in ("q", "\x1b", "\x04"):
            self.disarm()
            self.quit_requested = True
            return "Quit and stop"
        if key in (" ", "x"):
            self.disarm()
            return "STOP; disarmed (press g to arm again)"
        if key == "g":
            if not motor_ready:
                self.disarm()
                return "Motor path unavailable or another publisher exists; cannot arm"
            self.armed = True
            self.command = STOP
            self.expires_at = 0.0
            return "Armed; tap W/A/S/D for one short pulse"
        if key in KEY_COMMANDS:
            if not motor_ready:
                self.disarm()
                return "Motor path unavailable or another publisher exists; cannot move"
            if not self.armed:
                return "Disarmed; press g first"
            self.command = KEY_COMMANDS[key]
            self.expires_at = now + PULSE_SECONDS
        return None

    def current(self, now: float, motor_ready: bool) -> tuple[float, float]:
        if not motor_ready:
            self.disarm()
            return STOP
        if not self.armed or now >= self.expires_at:
            self.command = STOP
            return STOP
        return self.command


class MotorPublisher(Protocol):
    def ready(self) -> bool: ...
    def publish(self, command: tuple[float, float]) -> None: ...
    def close(self) -> None: ...


class DryRunPublisher:
    def ready(self) -> bool:
        return True

    def publish(self, command: tuple[float, float]) -> None:
        pass

    def close(self) -> None:
        pass


class RosMotorPublisher:
    def __init__(self) -> None:
        # Import ROS only in explicitly requested live mode. Dry-run also works
        # on a Mac without ROS installed.
        import rclpy
        from std_msgs.msg import Float32MultiArray

        self.rclpy = rclpy
        self.message_type = Float32MultiArray
        rclpy.init()
        try:
            self.node = rclpy.create_node("ssh_keyboard_teleop")
            self.publisher = self.node.create_publisher(
                Float32MultiArray, "motor_commands", 10
            )
        except Exception:
            rclpy.try_shutdown()
            raise

    def ready(self) -> bool:
        self.rclpy.spin_once(self.node, timeout_sec=0.0)
        return (
            self.node.count_subscribers("motor_commands") > 0
            and self.node.count_publishers("motor_commands") == 1
            and "motor_controller_node" in self.node.get_node_names()
        )

    def publish(self, command: tuple[float, float]) -> None:
        msg = self.message_type()
        msg.data = [float(command[0]), float(command[1])]
        self.publisher.publish(msg)

    def close(self) -> None:
        # Explicit zeros on a normal exit; motor_control's 0.15 s timeout is
        # the fallback if SSH, this process, or the robot crashes abruptly.
        for _ in range(5):
            try:
                self.publish(STOP)
            except Exception:
                break
            time.sleep(PUBLISH_SECONDS)
        try:
            self.node.destroy_node()
        finally:
            self.rclpy.try_shutdown()


def run(publisher: MotorPublisher) -> None:
    if not sys.stdin.isatty():
        raise RuntimeError("Interactive TTY required; connect with ssh -tt")

    fd = sys.stdin.fileno()
    original_settings = termios.tcgetattr(fd)
    state = PulseState()
    last_command = STOP
    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, request_stop)

    try:
        tty.setcbreak(fd)
        print("Keys: g arm | w forward | s back | a left | d right", flush=True)
        print("Each key gives at most 0.20 s of motion; repeats extend it.", flush=True)
        print("Space/x: STOP and disarm | q/Esc/Ctrl-C: quit", flush=True)
        print("Do not run any other publisher on motor_commands.", flush=True)
        while not stop_requested and not state.quit_requested:
            motor_ready = publisher.ready()
            if not motor_ready and state.armed:
                state.disarm()
                print("Motor path changed; STOP and disarmed", flush=True)

            readable, _, _ = select.select([fd], [], [], PUBLISH_SECONDS)
            if readable:
                data = os.read(fd, 64)
                if not data:
                    break
                for byte in data:
                    message = state.accept(chr(byte), time.monotonic(), motor_ready)
                    if message:
                        print(message, flush=True)
                    if state.quit_requested:
                        break

            command = state.current(time.monotonic(), motor_ready)
            publisher.publish(command)
            if command != last_command:
                print(f"L={command[0]:+.2f} R={command[1]:+.2f}", flush=True)
                last_command = command
    finally:
        state.disarm()
        termios.tcsetattr(fd, termios.TCSADRAIN, original_settings)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--live", action="store_true", help="actually publish motor commands"
    )
    args = parser.parse_args()

    if not sys.stdin.isatty():
        parser.error("interactive terminal required; use ssh -tt")

    if args.live:
        print("LIVE MODE: motor commands enabled, but disarmed until g.", flush=True)
        try:
            publisher: MotorPublisher = RosMotorPublisher()
        except Exception as exc:
            print(f"Cannot start ROS motor publisher: {exc}", file=sys.stderr)
            return 1
    else:
        print("DRY RUN: no ROS import and no motor commands.", flush=True)
        publisher = DryRunPublisher()

    try:
        run(publisher)
    except Exception as exc:
        print(f"Teleop stopped: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        publisher.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
