#!/usr/bin/env python3
"""Run one bounded open-loop motor command on the robot, then stop.

Examples:
  python3 -B ssh_timed_motion.py turn cw 1.5 --live
  python3 -B ssh_timed_motion.py turn ccw 1.5 --live
  python3 -B ssh_timed_motion.py forward 2.0 --live

Dry-run is the default. This tool does not read any floor profile or IMU.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from typing import Callable, Protocol


STOP = (0.0, 0.0)
DEFAULT_CMD = 0.30
MAX_CMD = 0.50
TURN_MAX_SECONDS = 7.0
FORWARD_MAX_SECONDS = 5.0
PUBLISH_SECONDS = 0.05
DISCOVERY_SECONDS = 3.0


class Publisher(Protocol):
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


class RosPublisher:
    def __init__(self) -> None:
        import rclpy
        from std_msgs.msg import Float32MultiArray

        self.rclpy = rclpy
        self.message_type = Float32MultiArray
        rclpy.init()
        try:
            self.node = rclpy.create_node("ssh_timed_motion")
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
        message = self.message_type()
        message.data = [float(command[0]), float(command[1])]
        self.publisher.publish(message)

    def close(self) -> None:
        # The controller also applies its own 0.15 s command watchdog if this
        # process or its SSH connection dies before these zeros can be sent.
        try:
            for _ in range(5):
                try:
                    self.publish(STOP)
                except Exception:
                    break
                time.sleep(PUBLISH_SECONDS)
            self.node.destroy_node()
        finally:
            self.rclpy.try_shutdown()


def motor_command(action: str, direction: str | None, magnitude: float) -> tuple[float, float]:
    if action == "forward":
        return magnitude, magnitude
    if action == "turn" and direction == "ccw":
        return -magnitude, magnitude
    if action == "turn" and direction == "cw":
        return magnitude, -magnitude
    raise ValueError("Unsupported action or direction")


def run_motion(
    publisher: Publisher,
    command: tuple[float, float],
    duration: float,
    *,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> float:
    """Publish at 20 Hz while ready; return actual command-on seconds."""
    try:
        discovery_deadline = clock() + DISCOVERY_SECONDS
        while not publisher.ready():
            publisher.publish(STOP)
            if clock() >= discovery_deadline:
                raise RuntimeError(
                    "Motor controller unavailable or another motor_commands publisher is active"
                )
            sleep(PUBLISH_SECONDS)

        start = clock()
        deadline = start + duration
        while clock() < deadline:
            if not publisher.ready():
                raise RuntimeError("Motor path changed during motion")
            publisher.publish(command)
            sleep(min(PUBLISH_SECONDS, max(0.0, deadline - clock())))
        return clock() - start
    finally:
        publisher.publish(STOP)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="action", required=True)

    turn = subparsers.add_parser("turn", help="spin in place for a specified time")
    turn.add_argument("direction", choices=("cw", "ccw"))
    turn.add_argument("seconds", type=float)
    turn.add_argument("--cmd", type=float, default=DEFAULT_CMD)
    turn.add_argument("--live", action="store_true")

    forward = subparsers.add_parser("forward", help="drive straight for a specified time")
    forward.add_argument("seconds", type=float)
    forward.add_argument("--cmd", type=float, default=DEFAULT_CMD)
    forward.add_argument("--live", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    maximum = TURN_MAX_SECONDS if args.action == "turn" else FORWARD_MAX_SECONDS
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= maximum:
        parser.error(f"seconds must be >0 and <={maximum:g} for {args.action}")
    if not math.isfinite(args.cmd) or not 0 < args.cmd <= MAX_CMD:
        parser.error("cmd must be >0 and <=0.5")
    command = motor_command(args.action, getattr(args, "direction", None), args.cmd)
    label = f"turn {args.direction}" if args.action == "turn" else "forward"
    print(f"{label}: L={command[0]:+.2f} R={command[1]:+.2f} for {args.seconds:g} s", flush=True)

    if args.live:
        print("LIVE: motor commands enabled; clear the area before running.", flush=True)
        try:
            publisher: Publisher = RosPublisher()
        except Exception as exc:
            print(f"Cannot start ROS publisher: {exc}", file=sys.stderr, flush=True)
            return 1
    else:
        print("DRY RUN: no ROS import or motor commands. Add --live to move.", flush=True)
        return 0

    previous_handlers = {}

    def interrupt(_signum: int, _frame: object) -> None:
        raise KeyboardInterrupt

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous_handlers[sig] = signal.signal(sig, interrupt)
    try:
        elapsed = run_motion(publisher, command, args.seconds)
        print(f"STOP after {elapsed:.3f} s of commanded motion", flush=True)
        return 0
    except KeyboardInterrupt:
        print("Interrupted; STOP", file=sys.stderr, flush=True)
        return 130
    except Exception as exc:
        print(f"Aborted; STOP: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        publisher.close()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    raise SystemExit(main())
