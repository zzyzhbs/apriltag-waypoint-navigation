#!/usr/bin/env python3
"""Phase A: stationary gyro bias measurement. Robot must be still, motors off.
Polls IMU at ~10Hz for DURATION seconds, logs gz/yaw, reports bias statistics
and detects frozen output (consecutive identical samples)."""
import json
import sys
import time

import serial

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.3)
t0 = time.time()
records = []      # (t, gz, y)
echo_count = 0
frozen_pairs = 0
prev_gz = None

while time.time() - t0 < DURATION:
    s.write(b'{"T":126}\n')
    line = s.readline()
    if line:
        try:
            d = json.loads(line)
            if d.get('T') == 1002:
                t = time.time() - t0
                gz, y = d.get('gz'), d.get('y')
                records.append((t, gz, y))
                if prev_gz is not None and abs(gz - prev_gz) < 1e-9:
                    frozen_pairs += 1
                prev_gz = gz
            else:
                echo_count += 1
        except Exception:
            pass
    time.sleep(0.1)
s.close()

n = len(records)
print('n_data=%d echoes=%d' % (n, echo_count))
if n == 0:
    print('NO DATA')
    sys.exit(1)
dur = records[-1][0] - records[0][0]
print('span=%.1fs effective_rate=%.2fHz' % (dur, (n - 1) / dur if dur > 0 else 0))
print('first_10:')
for t, gz, y in records[:10]:
    print('  t=%6.2f gz=%+9.4f y=%+9.2f' % (t, gz, y))
gzs = [r[1] for r in records]
mean = sum(gzs) / n
std = (sum((g - mean) ** 2 for g in gzs) / n) ** 0.5
ys = [r[2] for r in records]
print('gz_mean=%+.4f deg/s  gz_std=%.4f deg/s' % (mean, std))
print('yaw first=%+.2f last=%+.2f drift=%+.3f deg' % (ys[0], ys[-1], ys[-1] - ys[0]))
print('frozen_consecutive_pairs=%d' % frozen_pairs)
