# HW2 run `run_20261024_055456`

## Total motion

- driven: **4.924 m**
- turned: **506.2°** (8.83 rad)
- planned minimum: 5.236 m and 360°

## Smoothness

- primitives: 16 (6 drives, 10 spins); each one starts and ends at rest
- per command level: 0.25: 4, 0.3: 6, 0.35: 6
- run time 59.5 s, moving 16.8 s (28 %)

## Waypoints

| wp | goal | estimate | estimate − goal | tags | note |
|---|---|---|---|---|---|
| 1 | (+1.000, +0.000, +0.0°) | (+0.972, +0.012, +0.5°) | 3.1 cm, +0.5° | 0 |  |
| 2 | (+1.000, +2.000, +180.0°) | (+0.996, +2.033, +180.0°) | 3.3 cm, +0.0° | 0 1 |  |
| 3 | (+0.000, +0.000, +0.0°) | (-0.051, -0.003, +0.5°) | 5.1 cm, +0.5° | 0 1 2 |  |

No truth.yaml: add the tape-measured pose of each waypoint to get localization and arrival errors.
