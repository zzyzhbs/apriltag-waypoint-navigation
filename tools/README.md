# Robot-side SSH tools

This directory contains two robot-side terminal tools:

| Script | Purpose |
| --- | --- |
| `ssh_keyboard_teleop.py` | Short manual drive/turn pulses. |
| `ssh_timed_motion.py` | One fixed-duration clockwise/counterclockwise turn or forward drive. |

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
