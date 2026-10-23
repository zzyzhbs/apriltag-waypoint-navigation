# HW2: closed-loop waypoint demo with AprilTags (stop-look-correct)

Branch `HW2`. Waypoints (0,0,0) → (1,0,0) → (1,2,π) → (0,0,0).
After every primitive the robot is still: it looks at the tags, corrects its pose
estimate (EKF), re-plans the next primitive, executes it, and repeats until each
waypoint is within 3 cm / 3°.

| File | What |
|---|---|
| `HW2/waypoints.txt` | the 4 waypoints (first line = start pose) |
| `robot_control/config/hw2/landmarks.yaml` | tag map (world frame); edit after measuring |
| `robot_control/config/hw2/apriltag.yaml` | apriltag node parameters (ids 0/1/2, size 0.166) |
| `robot_control/config/hw2/nav.yaml` | tolerances, command levels, look window, EKF noise |
| `robot_control/config/open_loop/study_wood2.yaml` | primitive calibration (HW1 + fine levels) |
| `robot_control/launch/hw2_perception_launch.py` | camera (rectified) + apriltag + `base_link → camera_frame` TF |
| `robot_control/launch/hw2_launch.py` | perception + `motor_control` + `hw2_nav` |
| `HW2/logs/run_<time>/` | one folder per run (trajectory, steps, observations, waypoints) |
| `HW2/plot_run.py` | figure + tables for the report (run on the Mac) |

## 0. Setup on the robot

```bash
cd ~/ros2_ws/src/CSE276A && git fetch && git checkout HW2 && git pull
cd ~/ros2_ws && colcon build --packages-select robot_control
```

Rebuild after every pull. Every new terminal (ssh session) must source first:
`source ~/ros2_ws/install/setup.bash`

`hw2_nav` and `hw2_probe` read `landmarks.yaml`, `nav.yaml`, `waypoints.txt` and the
calibration from the **source** folder (`~/ros2_ws/src/CSE276A/...`), so editing those
needs no rebuild. `apriltag.yaml` and the launch files are installed: rebuild after
changing them.

Before starting anything, make sure no old camera node holds the camera
(it ignores Ctrl-C and can stay alive after a launch is stopped):
```bash
pgrep -af robot_vision_camera_node      # if it prints a PID: kill -9 <pid>
```

## 1. Place the tags and the robot

World frame: origin = start point, x = robot heading at the start, y = left.
`base_link` = robot centre. The camera is 0.075 m ahead of the centre, 0.125 m above the floor.

Each tag stands on the extension of one leg, 0.5 m past the leg's end, facing back
along the leg. Tag centre at camera height (~12.5 cm), tag vertical.

| tag | leg | position (x, y) | faces (outward normal) |
|---|---|---|---|
| 0 | (0,0) → (1,0) | (1.5, 0) | 180° (towards −x) |
| 1 | (1,0) → (1,2) | (1, 2.5) | −90° (towards −y) |
| 2 | (1,2) → (0,0) | (−0.224, −0.447) | 63.4° (towards (1,2)) |

The area is about 1.9 m (x) × 3.1 m (y) including the boxes.
Put small floor markers at the 3 waypoint positions for the video
(if they are AprilTags, do **not** use ids 0, 1, 2).
Place the robot centre on (0, 0), facing +x.

If a tag cannot go exactly there, put it where it fits, measure it, and write the
measured values into `landmarks.yaml` (`yaw_deg` = direction the tag face points to).

## 2. Check perception (no motion)

Terminal 1:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_control hw2_perception_launch.py
```

Terminal 2: put tag 0 straight in front of the robot, about 1 m away:
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic hz /detections                 # ~10 Hz
ros2 run tf2_ros tf2_echo base_link tag_0
```
Expect translation x ≈ distance from robot centre, y ≈ 0, z ≈ 0.

Tag z sign: `hw2_probe` / `hw2_nav` use `tag_z_sign` (default 1 = the tag's z axis points
out of the tag face, towards the camera). Check with `hw2_probe` in step 3: if every tag
is rejected as `oblique ~180 deg`, the sign is wrong → add `-p tag_z_sign:=-1` to every
`hw2_probe` / `hw2_nav` command (or `tag_z_sign:=-1` to `hw2_launch.py`).

Foxglove (optional): open `ws://<robot ip>:8765`. Image panel: `/camera/image_raw/compressed`
(calibration None). 3D panel: frames `base_link`, `camera_frame`, `tag_0..2`
(and `world`, `/hw2/pose` while `hw2_nav` runs).

## 3. Static localization checks (`hw2_probe`, never moves)

Keep terminal 1 (perception) running. Terminal 2:

```bash
P="ros2 run robot_control hw2_probe --ros-args"
# robot pose from each tag every second (+ weighted mean); compare with the tape
$P -p mode:=localize
# robot at a known pose: where are the tags? compare with landmarks.yaml
$P -p mode:=survey -p robot_x:=0.0 -p robot_y:=0.0 -p robot_theta_deg:=0.0
# same, and save the measured map as config/hw2/landmarks_survey.yaml (map is not overwritten)
$P -p mode:=survey -p write:=true
# raw noise, detection rate and latency per tag (for the EKF noise in nav.yaml)
$P -p mode:=stats -p duration_s:=10.0
```

Targets: localize error < 3 cm / 3° at a few tape-measured poses; survey within ~2 cm / 2°
of the map. A large survey difference means a tag is placed or measured wrong: fix the
tag or `landmarks.yaml`.

Run `stats` at ~0.5, 1.5 and 2.5 m and note the std of x/y (mm) and psi (deg).
Then adjust `ekf: obs:` in `nav.yaml` (σ_xy = a + b·d², σ_psi = c + e·d, roughly 2–3× the
measured std, since tags are averaged over one look but placement errors remain).

## 4. Calibrate the fine command levels (once per floor)

The HW1 levels can't make small corrections (smallest move ≈ 5.5 cm / 24°).
Add a fine level for driving and for spinning. Terminal 1: `motor_control` (perception not needed):

```bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_control motor_control
```

Terminal 2:
```bash
source ~/ros2_ws/install/setup.bash
C="ros2 run robot_control ol_calibrate --ros-args -p floor:=study_wood2"
$C -p mode:=drive -p cmds:="[0.2]" -p durations:="[0.2, 0.4, 0.6]" -p repeats:=2
$C -p mode:=spin -p dir:=both -p cmds:="[0.25]" -p durations:="[0.2, 0.4, 0.6]" -p repeats:=2
$C -p mode:=fit
```

Prompts: Enter = start, `s` = skip, `r` = redo; drive: x in m, y in cm; spin: degrees turned.
First check that 0.2 / 0.25 actually move the robot (above the dead band); if not, try higher
values and put them into `nav.yaml` `levels:` (fast → slow).

Goal: the smallest move (the fit at T = 0.2 s) ≤ 3 cm and ≤ 3°. If it is larger, allow a
shorter minimum run for that level in `study_wood2.yaml`, e.g. under `spin: ccw:` and `cw:`
add `min_T: {0.25: 0.1}` (seconds).

Check on the Mac what the planner will pick:
```bash
cd robot_control
python -c "
from robot_control.ol_model import Calibration
from robot_control.step_timing import pick_level, min_step
c = Calibration.load('config/open_loop/study_wood2.yaml')
print(min_step(c, 'drive', 'fwd', (0.3, 0.2)), min_step(c, 'spin', 'ccw', (0.35, 0.25)))
print(pick_level(c, 'spin', 'ccw', 5.0, (0.35, 0.25)).describe())"
```
Before the fine calibration this prints `0.055 24.2` and `cmd 0.25 not calibrated`;
afterwards both minimums should be ≤ 0.03 / 3 and the 5° spin should get a T.

## 5. Run the demo

Robot centre on (0, 0) facing +x, tags placed, area clear.

**One terminal** (perception + motor_control + hw2_nav):
```bash
source ~/ros2_ws/install/setup.bash
# check first: looks 3 times, prints the estimate and the first planned step, never moves
ros2 launch robot_control hw2_launch.py dry_run:=true
# run
ros2 launch robot_control hw2_launch.py
```
Arguments: `floor:=study_wood2` (default), `tag_z_sign:=1`, `dry_run:=false`.
Everything shuts down when `hw2_nav` finishes. Afterwards run `pgrep -af robot_vision_camera_node`.

**Or three terminals** (easier to restart only the runner):
```bash
# T1
ros2 launch robot_control hw2_perception_launch.py
# T2
ros2 run robot_control motor_control
# T3
ros2 run robot_control hw2_nav --ros-args -p floor:=study_wood2 -p dry_run:=true
ros2 run robot_control hw2_nav --ros-args -p floor:=study_wood2
```

What it prints:
```
--- waypoint 1: (1, 0, 0 deg)
  look: 10 frames, tags [0@1.50m]
  drive fwd   0.890 m   cmd 0.30  T 2.31 s
  look: 10 frames, tags [0@0.61m]
  drive fwd   0.105 m   cmd 0.20  T ...
  look: ...
>>> waypoint 1 reached: estimate (+1.004, -0.006, +0.8 deg), 3 looks, tags [0]
```
Each primitive is followed by a 1 s settle and a 1 s look. `HALT: ...` means two looks in a
row disagreed with the estimate (map / tag sign / extrinsic wrong): check step 2–3.
Ctrl-C stops the motors.

**During the video**: after each `>>> waypoint i reached`, measure the robot centre (x, y)
and heading with the tape (the robot pauses `wp_pause_s` = 1 s; give it longer with
`-p wp_pause_s:=10.0` on `hw2_nav` if you need time).

## 6. Figures and tables for the report (on the Mac)

The robot cannot push to GitHub, so copy the run folder:
```bash
scp -r ubuntu@<robot ip>:ros2_ws/src/CSE276A/HW2/logs/run_<time> HW2/logs/
```

Write the tape measurements to `HW2/logs/run_<time>/truth.yaml`:
```yaml
waypoints:
  - {wp: 1, x: 1.01, y: -0.02, theta_deg: 1.5}
  - {wp: 2, x: 0.99, y: 2.01, theta_deg: 178.0}
  - {wp: 3, x: 0.01, y: 0.00, theta_deg: -2.0}
```

Then:
```bash
python HW2/plot_run.py HW2/logs/run_<time>      # or no argument = newest run
```
Outputs in the run folder:
- `trajectory.png`: estimated pose every 0.5 s, tag updates, waypoints, tags, tape truth
- `summary.md`: total distance driven / turned, smoothness numbers, and per waypoint the
  **localization error** (estimate − tape) and **arrival error** (tape − goal)

Raw data if needed: `trajectory.csv` (pose every 0.5 s), `steps.csv` (every primitive),
`observations.csv` (every tag in every look, accepted / rejected and why),
`waypoints.csv`, `config.yaml` (all parameters and the map used).

## 7. Tuning (`nav.yaml`, no rebuild)

| Symptom | Knob |
|---|---|
| a waypoint needs many looks / "max_iters reached" | `control.max_iters`; check fine-level calibration |
| drives often stop far short or overshoot | `control.undershoot_k`, `undershoot_c` |
| heading drifts on the 2 m legs | `control.max_drive` (smaller = one more look per leg) |
| tags rejected as unstable | `look.max_spread`, `look.window_s` |
| good tags rejected by the gate (`d2` large in observations.csv) | increase `ekf.obs` noise, or `ekf.motion` noise if it happens after long moves |
| robot accepts a waypoint while visibly off | `tolerance.pos`, `tolerance.ang_deg` |

Known limit: WP2's final heading (180°) faces no tag, so its last turn (the last ~55°) is not
corrected. A fourth tag at (0.5, 2) facing +x would fix it (add it to `landmarks.yaml` and
`apriltag.yaml`).
