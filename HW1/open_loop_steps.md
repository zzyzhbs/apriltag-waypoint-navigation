# Open-loop HW1: calibrate and run

Branch `open_loop`. On the robot: `git fetch && git checkout open_loop && colcon build`.
Calibration data: `robot_control/config/open_loop/<floor>.yaml` (saved after every trial).

```bash
# pane 1
source ~/ros2_ws/install/setup.bash
ros2 run robot_control motor_control

# pane 2
source ~/ros2_ws/install/setup.bash
C="ros2 run robot_control ol_calibrate --ros-args -p floor:=wood"
# 1. sweep levels
$C -p mode:=drive -p cmds:="[0.15, 0.2, 0.3, 0.4]" -p durations:="[2.0, 4.0]"
$C -p mode:=spin -p dir:=both -p cmds:="[0.3, 0.35, 0.4]" -p durations:="[0.5, 1.5]"
# 2. densify the working levels
$C -p mode:=drive -p cmds:="[0.3]" -p durations:="[2.0, 3.0, 4.0, 5.0]" -p repeats:=2
$C -p mode:=spin -p dir:=both -p cmds:="[0.35]" -p durations:="[0.4, 0.8, 1.2, 1.6]" -p repeats:=2
# 3. trim (secant): one run with a guessed trim, then fit, then verify with the new trim
$C -p mode:=drive -p cmds:="[0.3]" -p durations:="[4.0]" -p trim:=0.02
$C -p mode:=fit
# 4. accuracy for the report
$C -p mode:=verify
```

Prompts: Enter = start, `s` = skip, `r` (at a measurement) = redo.
- drive: x = distance along the reference line (m), y = lateral offset (cm, left +)
- spin: angle actually turned (deg, positive). Pick T so the angle stays in 30-330 deg.

Plan only (also works on the Mac):
```bash
python -m robot_control.ol_planner HW1/waypoints.txt robot_control/config/open_loop/wood.yaml
ros2 run robot_control velocity_control --ros-args -p floor:=wood -p dry_run:=true
```

Run (robot at (0,0) facing +x):
```bash
ros2 launch robot_control hw1_launch.py floor:=wood
# or, with motor_control running:
ros2 run robot_control velocity_control --ros-args -p floor:=wood
```
Log: `HW1/logs/run_<floor>_<time>.csv`. The robot pauses 1 s at each waypoint.
