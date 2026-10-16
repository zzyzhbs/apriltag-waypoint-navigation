# Robot-side SSH tools

This directory contains four robot-side terminal tools:

| Script | Purpose |
| --- | --- |
| `ssh_keyboard_teleop.py` | Short manual drive/turn pulses. |
| `ssh_timed_motion.py` | One fixed-duration clockwise/counterclockwise turn or forward drive. |
| `ssh_turn_calibration.py` | Eight human-judged turns with a per-trial angle table and directional speed summary. |
| `ssh_straight_calibration.py` | Five human-judged 1 m forward runs with a five-run mean. |

The calibration scripts do not replace the ROS floor profile automatically.

`ssh_keyboard_teleop.py` runs on the robot and reads keys sent from a Mac over
an interactive SSH terminal. It uses the existing ROS 2 `motor_commands` topic;
it does not require `pynput`, a GUI, a floor profile, or firmware flashing.

Copy the standalone script to a persistent path on the robot (replace the IP):

```sh
scp tools/ssh_keyboard_teleop.py ubuntu@<robot-ip>:/home/ubuntu/cse276a_ssh_keyboard_teleop.py
```

Check terminal input without publishing any motor commands:

```sh
ssh -tt ubuntu@<robot-ip> 'python3 -B /home/ubuntu/cse276a_ssh_keyboard_teleop.py'
```

For physical control, first ensure the robot is on a clear floor, the motor
controller node is running, and no other node publishes `motor_commands`:

```sh
ssh -tt ubuntu@<robot-ip> "bash -lc 'source /opt/ros/jazzy/setup.bash; source /home/ubuntu/ros2_ws/install/setup.bash; python3 -B /home/ubuntu/cse276a_ssh_keyboard_teleop.py --live'"
```

Press `g` to arm; `w`/`s` give a short forward/backward pulse at `±0.16`, and
`a`/`d` give a short left/right pulse at `±0.30`. A pulse lasts at most 0.20 s
unless another key-repeat event arrives. Space or `x` stops and disarms; `q`,
Esc, or Ctrl-C exits. Lost input also expires automatically, and the existing
motor controller has its own 0.15 s command timeout.

These are open-loop, normalized motor commands, **not** carpet2-calibrated
turn angles. Do not use this script simultaneously with `keyboard_control.py`
or another publisher on `motor_commands`.

Offline safety and terminal tests:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tools -p 'test_*.py'
```

## Fixed-duration terminal commands

`ssh_timed_motion.py` is standalone and runs **on the robot**. It publishes the
same normalized left/right commands as the other tools, but does not estimate
distance or angle. Dry-run is the default; `--live` is required for motion.
Install without changing the ROS workspace:

```sh
scp tools/ssh_timed_motion.py ubuntu@<robot-ip>:/home/ubuntu/ssh_timed_motion.py
```

Check command parsing first without moving:

```sh
ssh -tt ubuntu@<robot-ip> 'python3 -B /home/ubuntu/ssh_timed_motion.py turn cw 1.5'
```

With a clear floor, start `ros2 run robot_control motor_control` in a separate
SSH/tmux session after sourcing ROS. Do not use `robot_teleop_launch.py` here:
it also starts `keyboard_control`, a competing `motor_commands` publisher.
With only `motor_controller_node` subscribing, run **one command at a time**:

```sh
ssh -tt ubuntu@<robot-ip> "bash -lc 'source /opt/ros/jazzy/setup.bash; source /home/ubuntu/ros2_ws/install/setup.bash; ros2 run robot_control motor_control'"
```

Keep that controller terminal running, then use a second terminal for motion:

```sh
ssh -tt ubuntu@<robot-ip> "bash -lc 'source /opt/ros/jazzy/setup.bash; source /home/ubuntu/ros2_ws/install/setup.bash; python3 -B /home/ubuntu/ssh_timed_motion.py turn cw 1.5 --live'"
ssh -tt ubuntu@<robot-ip> "bash -lc 'source /opt/ros/jazzy/setup.bash; source /home/ubuntu/ros2_ws/install/setup.bash; python3 -B /home/ubuntu/ssh_timed_motion.py turn ccw 1.5 --live'"
ssh -tt ubuntu@<robot-ip> "bash -lc 'source /opt/ros/jazzy/setup.bash; source /home/ubuntu/ros2_ws/install/setup.bash; python3 -B /home/ubuntu/ssh_timed_motion.py forward 2.0 --live'"
```

Replace `1.5` or `2.0` with seconds; `--cmd 0.30` may be added after the
duration (default 0.30; allowed range >0 to 0.50). Direction is CW or CCW
*in place*. Forward and turn commands are open loop, not 1 m/90° targets.
Forward duration is capped at 5 s and turn duration at 7 s. The script sends
zeros on completion, motor-path loss, and normal interruption; the controller's
0.15 s watchdog is the fallback if the process or SSH connection dies.
Never run this alongside another `motor_commands` publisher, and allow enough
clearance for the full configured duration before using `--live`.

## Human-referenced floor calibration

`ssh_turn_calibration.py` and `ssh_straight_calibration.py` run **on the robot**
over `ssh -tt`. Both need `ssh_keyboard_teleop.py` beside them; the straight
script also imports the turn script's small logging helpers. Copy all three to
one persistent directory, or use a checkout containing all three. Neither
reads the IMU nor automatically changes a floor YAML profile. First run each
without `--live` to check terminal input without sending motor commands.

From a Mac checkout, copy the files to the robot if its checkout does not yet
contain this branch's `tools/` directory:

```sh
scp tools/ssh_keyboard_teleop.py tools/ssh_turn_calibration.py tools/ssh_straight_calibration.py ubuntu@<robot-ip>:/home/ubuntu/
```

Then test terminal input without driving:

```sh
ssh -tt ubuntu@<robot-ip> 'python3 -B /home/ubuntu/ssh_turn_calibration.py --floor carpet2'
ssh -tt ubuntu@<robot-ip> 'python3 -B /home/ubuntu/ssh_straight_calibration.py --floor carpet2'
```

Run these in separate sessions. Dry-run files are marked `dry_run` and must
not be used as physical calibration data.

For live motion, clear the area, ensure the motor controller node is running,
and stop any other publisher to `motor_commands`. If you copied the files to
`/home/ubuntu/`, run one of these commands at a time:

```sh
ssh -tt ubuntu@<robot-ip> "bash -lc 'source /opt/ros/jazzy/setup.bash; source /home/ubuntu/ros2_ws/install/setup.bash; python3 -B /home/ubuntu/ssh_turn_calibration.py --floor carpet2 --cmd 0.30 --live'"
ssh -tt ubuntu@<robot-ip> "bash -lc 'source /opt/ros/jazzy/setup.bash; source /home/ubuntu/ros2_ws/install/setup.bash; python3 -B /home/ubuntu/ssh_straight_calibration.py --floor carpet2 --cmd 0.30 --live'"
```

If using the robot's repo checkout instead, replace the script path with
`/home/ubuntu/ros2_ws/src/CSE276A/tools/<script-name>.py` after pulling the
branch containing these tools.

These are **separate sessions**; do not launch them simultaneously. All keys
are taps, not holds. `g` arms; `a`/`d` starts the indicated turn, or `w` starts
forward motion. Space stops immediately, `x` stops and rejects the attempt,
and `q` quits. An observed terminal EOF or timed-out trial sends zero; the
motor controller's 0.15 s watchdog remains the fallback for sudden process
or network failure. The turn cap defaults to 5 s (hard maximum 7 s), and the
straight cap to 3.5 s (hard maximum 5 s). An SSH delay can bias the human stop
time, so repeat any visibly lagged trial.

### Turn sequence

The fixed order is CCW 90°, CW 90°, CCW 180°, CW 180°, CCW 135°, CW 135°,
CCW 360°, CW 360°. Here "positive" means CCW. At each step align the robot
to a known start heading and the appropriate floor angle mark; only the
expected direction key starts the motors. On Space, wait for coasting to end
and enter the **actual final angle magnitude**, even if it missed the target.
Enter alone means you observed exactly the displayed target; `x` means the
angle cannot be judged and repeats that step. For 360°, visually count the
full revolution or use video: merely ending at the start heading cannot tell
0° from 360°. Earlier pairs do not automatically reset the robot.

Each attempt gets its own row in
`~/turn_calibration_<floor>_<timestamp>.csv`; rejected attempts also remain
visible. The matching `.jsonl` preserves stop times even if SSH drops while
you are entering an angle. `_summary.json` stores CCW and CW separately as
`sum(actual final angles) / sum(command-on seconds)` from valid measurements.
This is a time-weighted **effective** turn speed including startup and coast,
not the steady gyro-derived `rot_rate_*` field of `calibrate.py`. Do not copy
it into that field without adapting the downstream controller.

### Five 1 m forward runs

Mark a measured 1 m path and leave runout beyond the line. For each of five
repetitions, reset the robot to the start, tap `g` then `w`, and tap Space as
the robot reaches the 1 m mark. After it stops, enter its actual final forward
distance in meters (Enter only if it ended at exactly 1.00 m). Invalid or
timed-out attempts are saved but do not count toward the five. The `.csv`
keeps every attempt; after five measured runs the program reports mean command
time, mean final distance, and total command seconds divided by total observed
meters as an empirical seconds-per-meter estimate. The same result is saved
in `_summary.json` only after all five measured runs are complete. Keep the
motor command and battery condition consistent across the set.
