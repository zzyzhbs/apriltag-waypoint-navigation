# HW2 run `run_20261024_035816`

## Total motion

- driven: **5.003 m**
- turned: **562.0°** (9.81 rad)
- planned minimum: 5.236 m and 360°

## Smoothness

- primitives: 14 (7 drives, 7 spins); each one starts and ends at rest
- per command level: 0.3: 7, 0.35: 7
- run time 55.5 s, moving 17.2 s (31 %)

## Waypoints

| wp | goal | estimate | estimate − goal | tags | note |
|---|---|---|---|---|---|
| 1 | (+1.000, +0.000, +0.0°) | (+0.981, -0.018, -0.1°) | 2.7 cm, -0.1° | 0 |  |
| 2 | (+1.000, +2.000, +180.0°) | (+1.000, +2.000, +180.0°) | 0.0 cm, +0.0° | 0 1 |  |
| 3 | (+0.000, +0.000, +0.0°) | (+0.000, +0.000, +0.0°) | 0.0 cm, +0.0° | 2 |  |

No truth.yaml: add the tape-measured pose of each waypoint to get localization and arrival errors.
