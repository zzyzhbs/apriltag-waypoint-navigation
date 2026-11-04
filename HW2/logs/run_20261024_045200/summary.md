# HW2 run `run_20261024_045200`

## Total motion

- driven: **5.288 m**
- turned: **349.8°** (6.11 rad)
- planned minimum: 5.236 m and 360°

## Smoothness

- primitives: 14 (8 drives, 6 spins); each one starts and ends at rest
- per command level: 0.3: 8, 0.35: 6
- run time 55.0 s, moving 16.4 s (30 %)

## Waypoints

| wp | goal | estimate | estimate − goal | tags | note |
|---|---|---|---|---|---|
| 1 | (+1.000, +0.000, +0.0°) | (+0.980, -0.033, -0.8°) | 3.8 cm, -0.8° | 0 |  |
| 2 | (+1.000, +2.000, +180.0°) | (+1.000, +2.000, +163.2°) | 0.0 cm, -16.8° | 0 1 | remaining spin is below the smallest step |
| 3 | (+0.000, +0.000, +0.0°) | (-0.017, -0.012, -23.4°) | 2.1 cm, -23.4° | 0 2 | remaining spin is below the smallest step |

No truth.yaml: add the tape-measured pose of each waypoint to get localization and arrival errors.
