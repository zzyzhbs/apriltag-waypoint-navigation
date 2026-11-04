# HW2 run `run_20261023_072308`

## Total motion

- driven: **5.297 m**
- turned: **504.5°** (8.81 rad)
- planned minimum: 5.236 m and 360°

## Smoothness

- primitives: 15 (8 drives, 7 spins); each one starts and ends at rest
- per command level: 0.3: 8, 0.35: 7
- run time 58.0 s, moving 17.7 s (30 %)

## Waypoints

| wp | goal | estimate | estimate − goal | tags | note |
|---|---|---|---|---|---|
| 1 | (+1.000, +0.000, +0.0°) | (+1.006, +0.013, -0.5°) | 1.4 cm, -0.5° | 0 |  |
| 2 | (+1.000, +2.000, +180.0°) | (+1.000, +2.000, -165.8°) | 0.0 cm, +14.2° | 0 1 | remaining spin is below the smallest step |
| 3 | (+0.000, +0.000, +0.0°) | (+0.000, +0.000, +0.0°) | 0.0 cm, +0.0° | 2 |  |

No truth.yaml: add the tape-measured pose of each waypoint to get localization and arrival errors.
