#!/usr/bin/env python3
"""Analyze a single-turn window: our calibrated heading delta vs board yaw
delta, comparing rest periods before and after the turn."""
import math
import sys

CX, CY, SX, SY = -14.5, -15.0, 20.5, 22.0


def heading(mx, my):
    # CCW positive
    return -math.degrees(math.atan2((my - CY) / SY, (mx - CX) / SX))


def unwrap(s):
    out = [s[0]]
    for v in s[1:]:
        d = v - out[-1]
        while d > 180:
            d -= 360
        while d < -180:
            d += 360
        out.append(out[-1] + d)
    return out


rows = []
for line in open(sys.argv[1]):
    if line.startswith('t'):
        continue
    t, mx, my, mz, y, gz = map(float, line.split())
    rows.append((t, mx, my, mz, y, gz))

hs = [heading(r[1], r[2]) for r in rows]
ys = [r[4] for r in rows]
uw = unwrap(hs)
yuw = unwrap(ys)

# rest windows: first 3s and last 3s (per-sample ~0.22s)
def mean(series, lo, hi):
    seg = [v for i, v in enumerate(series) if lo <= rows[i][0] <= hi]
    return sum(seg) / len(seg) if seg else None

span = rows[-1][0]
h_pre = mean(uw, 0, 3)
h_post = mean(uw, span - 3, span)
y_pre = mean(yuw, 0, 3)
y_post = mean(yuw, span - 3, span)
print('our heading: pre=%+.1f post=%+.1f delta=%+.1f deg'
      % (h_pre, h_post, h_post - h_pre))
print('board yaw:   pre=%+.1f post=%+.1f delta=%+.1f deg'
      % (y_pre, y_post, y_post - y_pre))
# levelness
mzs = [r[3] for r in rows]
print('mz spread: %.1f (tilt indicator)' % (max(mzs) - min(mzs)))
