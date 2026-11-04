#!/usr/bin/env python3
"""Phase B v2: 25s window, full data saved to /tmp/phase_b_data.txt.
Locates the hand turn via the yaw step and integrates gz around it."""
import json
import sys
import time

import serial

BIAS = -1.51
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.3)
t0 = time.time()
recs = []
while time.time() - t0 < DURATION:
    s.write(b'{"T":126}\n')
    line = s.readline()
    if line:
        try:
            d = json.loads(line)
            if d.get('T') == 1002:
                recs.append((time.time() - t0, d.get('gz'), d.get('y')))
        except Exception:
            pass
    time.sleep(0.1)
s.close()

with open('/tmp/phase_b_data.txt', 'w') as f:
    for t, g, y in recs:
        f.write('%.2f %.4f %.2f\n' % (t, g, y))

n = len(recs)
yaw = [r[2] for r in recs]
uw = [yaw[0]]
for i in range(1, n):
    d = yaw[i] - yaw[i - 1]
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    uw.append(uw[-1] + d)

best = None
for i in range(n - 2):
    d = uw[i + 2] - uw[i]
    if best is None or abs(d) > abs(best[0]):
        best = (d, i)
tot_dy, i0 = best
print('n=%d' % n)
print('yaw step: samples %d..%d (t=%.1f..%.1f) delta=%+.1f deg'
      % (i0, i0 + 2, recs[i0][0], recs[i0 + 2][0], tot_dy))
print('yaw total unwrapped delta over window: %+.1f deg' % (uw[-1] - uw[0]))
ang = 0.0
for i in range(1, n):
    dt = recs[i][0] - recs[i - 1][0]
    ang += 0.5 * ((recs[i][1] - BIAS) + (recs[i - 1][1] - BIAS)) * dt
print('gz integral over full window: %+.1f deg' % ang)
a = max(0, i0 - 2)
b = min(n - 1, i0 + 8)
ang2 = 0.0
for i in range(a + 1, b + 1):
    dt = recs[i][0] - recs[i - 1][0]
    ang2 += 0.5 * ((recs[i][1] - BIAS) + (recs[i - 1][1] - BIAS)) * dt
print('gz integral around turn: %+.1f deg' % ang2)
