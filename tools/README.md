# SSH keyboard teleop

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
