# HW2 run `run_20261024_044445`

## Total motion

- driven: **5.132 m**
- turned: **467.6°** (8.16 rad)
- planned minimum: 5.236 m and 360°

## Smoothness

- primitives: 13 (7 drives, 6 spins); each one starts and ends at rest
- per command level: 0.3: 7, 0.35: 6
- run time 52.5 s, moving 16.4 s (31 %)

## Waypoints

| wp | goal | estimate | estimate − goal | tags | note |
|---|---|---|---|---|---|
| 1 | (+1.000, +0.000, +0.0°) | (+0.989, +0.004, +1.0°) | 1.2 cm, +1.0° | 0 |  |
| 2 | (+1.000, +2.000, +180.0°) | (+1.000, +2.000, +180.0°) | 0.0 cm, +0.0° | 0 1 |  |
| 3 | (+0.000, +0.000, +0.0°) | (+0.114, +0.027, -131.9°) | 11.7 cm, -131.9° | 2 | remaining spin is below the smallest step |

No truth.yaml: add the tape-measured pose of each waypoint to get localization and arrival errors.
