# HW2 run `run_20261023_071302`

## Total motion

- driven: **2.043 m**
- turned: **558.6°** (9.75 rad)
- planned minimum: 5.236 m and 360°

## Smoothness

- primitives: 12 (5 drives, 7 spins); each one starts and ends at rest
- per command level: 0.3: 5, 0.35: 7
- run time 43.5 s, moving 9.5 s (22 %)

## Waypoints

| wp | goal | estimate | estimate − goal | tags | note |
|---|---|---|---|---|---|
| 1 | (+1.000, +0.000, +0.0°) | (+0.967, +0.018, -25.1°) | 3.8 cm, -25.1° | 0 | remaining drive is below the smallest step |
| 2 | (+1.000, +2.000, +180.0°) | (+0.966, +0.015, +92.7°) | 198.5 cm, -87.3° | 0 1 | remaining spin is below the smallest step |
| 3 | (+0.000, +0.000, +0.0°) | (-0.002, -0.019, +0.0°) | 1.9 cm, +0.0° | 0 1 |  |

No truth.yaml: add the tape-measured pose of each waypoint to get localization and arrival errors.
