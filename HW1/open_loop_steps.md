# Open-loop HW1: calibrate and run

Branch `main` (the open-loop code; the old IMU closed-loop code is in `imu_version`).
Final calibration: **`study_wood2`** (study-room wood floor, 2026-10-14 night).

| File | What |
|---|---|
| `robot_control/config/open_loop/study_wood2.yaml` | final calibration (trials + verify + `cw.time_scale: 1.15`) |
| `robot_control/config/open_loop/study_wood.yaml` | first attempt on the same floor (spin rms 11-18 deg, replaced) |
| `robot_control/config/open_loop/wood.yaml` | single migrated drive trial from the old gyro branch (not used) |
| `HW1/logs/run_study_wood2_20261015_072805.csv` | the demo run in the video |
| `HW1/logs/run_study_wood2_*.csv` | all runs while tuning `time_scale` |

## Setup on the robot

```bash
cd ~/ros2_ws/src/CSE276A && git fetch && git checkout main && git pull
cd ~/ros2_ws && colcon build --packages-select robot_control
```

Rebuild after every pull. Each new terminal (ssh session) must source the workspace first:
`source ~/ros2_ws/install/setup.bash`

## Re-run the demo

Robot at (0, 0) facing +x, ~2.5 m x 2.5 m clear area.

Terminal 1 — motor driver (keep it running; Ctrl-C stops it):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_control motor_control
```

Terminal 2 — waypoint runner:
```bash
source ~/ros2_ws/install/setup.bash
# check the plan first (no motion); T values should match the demo log
ros2 run robot_control velocity_control --ros-args -p floor:=study_wood2 -p dry_run:=true
# run
ros2 run robot_control velocity_control --ros-args -p floor:=study_wood2
```

Alternative (one terminal): `ros2 launch robot_control hw1_launch.py floor:=study_wood2`
starts motor_control and velocity_control together.

The robot pauses 1 s at each waypoint and prints `>>> Reached waypoint i`.
Log: `HW1/logs/run_<floor>_<time>.csv`.

Plan on the Mac (no ROS):
```bash
cd robot_control
python -m robot_control.ol_planner ../HW1/waypoints.txt config/open_loop/study_wood2.yaml
```

## Calibrate a new floor (what we did for study_wood2)

Terminal 1: `motor_control` as above. Terminal 2:

```bash
source ~/ros2_ws/install/setup.bash
C="ros2 run robot_control ol_calibrate --ros-args -p floor:=<new_floor>"
$C -p mode:=drive -p cmds:="[0.3]" -p durations:="[2.0, 3.0, 4.0]" -p repeats:=2
$C -p mode:=spin -p dir:=both -p cmds:="[0.35]" -p durations:="[0.4, 0.8, 1.2, 1.6]" -p repeats:=2
$C -p mode:=fit       # print fits
$C -p mode:=verify    # 1.0 m, 1.414 m, ccw 90, cw 90, ccw 180 -> accuracy for the report
```

Prompts: Enter = start, `s` = skip, `r` (at a measurement) = redo. Each trial is saved immediately.
- drive: x = distance along the reference line (m), y = lateral offset (cm, left +)
- spin: angle actually turned (deg, positive). Keep angles within 30-330 deg.

Optional:
- `time_scale`: if one spin direction is consistently short / long in the full run, add
  `time_scale: <k>` under `spin: cw:` (or `ccw:`) in the yaml; every spin time in that direction
  is multiplied by k. study_wood2 uses `cw.time_scale: 1.15` (CW turned ~70 deg instead of 90 in
  the full run at 1.0; 1.28 over-turned). Raw trials are left unchanged.
