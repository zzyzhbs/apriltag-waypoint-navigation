# HW2 run `run_20261024_045017`

## Total motion

- driven: **5.170 m**
- turned: **689.7°** (12.04 rad)
- planned minimum: 5.236 m and 360°

## Smoothness

- primitives: 16 (8 drives, 8 spins); each one starts and ends at rest
- per command level: 0.3: 8, 0.35: 8
- run time 60.5 s, moving 17.8 s (29 %)

## Waypoints

| wp | goal | estimate | estimate − goal | tags | note |
|---|---|---|---|---|---|
| 1 | (+1.000, +0.000, +0.0°) | (+0.987, +0.010, +1.4°) | 1.6 cm, +1.4° | 0 |  |
| 2 | (+1.000, +2.000, +180.0°) | (+1.000, +2.000, +180.0°) | 0.0 cm, +0.0° | 0 1 |  |
| 3 | (+0.000, +0.000, +0.0°) | (+0.000, +0.000, +0.0°) | 0.0 cm, +0.0° | 2 |  |

No truth.yaml: add the tape-measured pose of each waypoint to get localization and arrival errors.
