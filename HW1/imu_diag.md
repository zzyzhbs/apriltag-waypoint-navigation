# Calibration steps

On the robot, two tmux panes:

```bash
# pane 1
source ~/ros2_ws/install/setup.bash
ros2 run robot_control motor_control

# pane 2
source ~/ros2_ws/install/setup.bash
C="ros2 run robot_control calibrate --ros-args -p floor:=carpet -p test:="
${C}imu_diag   # x1, rotate by hand on the floor (don't lift); keep still ~3 s before each Enter
${C}straight   # x3, measure distance
${C}arc        # x1-2, measure distance (left arc + right arc)
${C}rotate     # x2
${C}deadband   # x1
```

Measuring (straight / arc): tape a ~1.5 m straight line, start each run aligned on it.
After the robot stops, measure the same point on the robot:
- x = distance along the line (m)
- y = offset from the line (cm, left = +)

Don't touch the robot during the 3-4 s still period before each run.
Answer Y to save. Output: `robot_control/config/floors/<floor>.yaml`, `config/diag/imu_diag_<floor>_<time>.csv`.
