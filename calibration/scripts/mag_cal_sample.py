#!/usr/bin/env python3
"""Phase B2: magnetometer calibration data collection. The user rotates the
robot slowly through ~360 deg. Logs mx,my,mz (+yaw,gz for reference) to
/tmp/phase_b2_data.txt."""
import json
import sys
import time

import serial

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/phase_b2_data.txt'
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.3)
t0 = time.monotonic()
recs = []
while time.monotonic() - t0 < DURATION:
    s.write(b'{"T":126}\n')
    line = s.readline()
    if line:
        try:
            d = json.loads(line)
            if d.get('T') == 1002:
                recs.append((time.monotonic() - t0, d.get('mx'), d.get('my'),
                             d.get('mz'), d.get('y'), d.get('gz')))
        except Exception:
            pass
    time.sleep(0.1)
s.close()

with open(OUT, 'w') as f:
    f.write('t mx my mz y gz\n')
    for r in recs:
        f.write('%.2f %.1f %.1f %.1f %.2f %.2f\n' % r)
print('n=%d saved to %s' % (len(recs), OUT))
