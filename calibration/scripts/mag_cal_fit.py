#!/usr/bin/env python3
"""Fit magnetometer hard/soft-iron calibration from Phase B2 data and
validate by comparing the calibrated heading integral with the board's yaw."""
import math
import sys

import numpy as np


def unwrap(deg_series):
    out = [deg_series[0]]
    for v in deg_series[1:]:
        d = v - out[-1]
        while d > 180:
            d -= 360
        while d < -180:
            d += 360
        out.append(out[-1] + d)
    return out


def main():
    rows = []
    for line in open(sys.argv[1]):
        if line.startswith('t'):
            continue
        t, mx, my, mz, y, gz = map(float, line.split())
        rows.append((t, mx, my, mz, y, gz))
    n = len(rows)
    print('n=%d samples' % n)
    mx = np.array([r[1] for r in rows])
    my = np.array([r[2] for r in rows])
    yaw = [r[4] for r in rows]

    # --- variant 1: crude min/max ---
    x0 = (mx.max() + mx.min()) / 2
    y0 = (my.max() + my.min()) / 2
    sx = (mx.max() - mx.min()) / 2
    sy = (my.max() - my.min()) / 2
    print('crude: center=(%.1f, %.1f) scale=(%.1f, %.1f)' % (x0, y0, sx, sy))
    h1 = [math.degrees(math.atan2((my[i] - y0) / sy, (mx[i] - x0) / sx))
          for i in range(n)]

    # --- variant 2: least-squares conic center ---
    A = np.column_stack([mx**2, mx * my, my**2, mx, my])
    D, *_ = np.linalg.lstsq(A, np.ones(n), rcond=None)
    a, b, c, d, e = D
    M = np.array([[2 * a, b], [b, 2 * c]])
    cx, cy = np.linalg.solve(M, np.array([-d, -e]))
    print('conic: center=(%.1f, %.1f)' % (cx, cy))
    # scale after centering
    ux = mx - cx
    uy = my - cy
    sx2 = (ux.max() - ux.min()) / 2
    sy2 = (uy.max() - uy.min()) / 2
    h2 = [math.degrees(math.atan2((uy[i]) / sy2, (ux[i]) / sx2))
          for i in range(n)]

    # --- variant 3: conic center + eigen rotation ---
    Q = np.array([[a, b / 2], [b / 2, c]])
    w, V = np.linalg.eigh(Q)
    h3 = []
    for i in range(n):
        u = V.T @ np.array([mx[i] - cx, my[i] - cy])
        h3.append(math.degrees(math.atan2(u[1], u[0])))

    yaw_uw = unwrap(yaw)
    for name, h in [('v1_crude', h1), ('v2_conic', h2), ('v3_rot', h3)]:
        uw = unwrap(h)
        print('%s: total=%+8.1f deg, first=%+.1f last=%+.1f'
              % (name, uw[-1] - uw[0], uw[0], uw[-1]))
    print('board yaw: total=%+8.1f deg' % (yaw_uw[-1] - yaw_uw[0]))


if __name__ == '__main__':
    main()
