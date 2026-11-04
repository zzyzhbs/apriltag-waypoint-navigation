#!/usr/bin/env python3
"""Motorized calibration v2 — adapted to observed weak-motor reality:
longer durations, bigger dead-zone probes, settle-curve sampling after each
turn, c=0.4 comparison point, and a battery-drift check (c=0.3 at start/end).
Ground truth: self-calibrated magnetometer heading at rest."""
import json
import math
import threading
import time

import serial

CX, CY, SX, SY = -14.5, -15.0, 20.5, 22.0
BIAS = -1.51

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.3)
records = []
stop_flag = [False]


def sampler():
    t0 = time.monotonic()
    while not stop_flag[0]:
        s.write(b'{"T":126}\n')
        line = s.readline()
        if line:
            try:
                d = json.loads(line)
                if d.get('T') == 1002:
                    records.append((time.monotonic() - t0, d.get('mx'),
                                    d.get('my'), d.get('mz'), d.get('y'),
                                    d.get('gz')))
            except Exception:
                pass
        time.sleep(0.05)


th = threading.Thread(target=sampler, daemon=True)
th.start()


def send(l, r):
    s.write(json.dumps({'T': 1, 'L': l, 'R': r}).encode() + b'\n')


def heading(mx, my):
    return -math.degrees(math.atan2((my - CY) / SY, (mx - CX) / SX))


def read_heading(n=4, timeout=6.0):
    t0 = time.monotonic()
    while len(records) < n and time.monotonic() - t0 < timeout:
        time.sleep(0.1)
    if len(records) < n:
        return None
    hs = [heading(r[1], r[2]) for r in records[-n:]]
    x = sum(math.cos(math.radians(h)) for h in hs) / n
    y = sum(math.sin(math.radians(h)) for h in hs) / n
    return math.degrees(math.atan2(y, x))


def delta(a, b):
    return (b - a + 180) % 360 - 180


def settle_reads(h0, n0):
    """heading deltas at +0.5s, +1.5s, +3s after stop; gz integral of turn"""
    time.sleep(0.5)
    d05 = delta(h0, read_heading())
    time.sleep(1.0)
    d15 = delta(h0, read_heading())
    time.sleep(1.5)
    d30 = delta(h0, read_heading())
    seg = records[n0:]
    ang = 0.0
    for i in range(1, len(seg)):
        dt = seg[i][0] - seg[i - 1][0]
        ang += 0.5 * ((seg[i][5] - BIAS) + (seg[i - 1][5] - BIAS)) * dt
    return d05, d15, d30, ang


def battery_probe(tag):
    h0 = read_heading()
    n0 = len(records)
    send(-0.3, 0.3)
    time.sleep(0.8)
    send(0.0, 0.0)
    d05, d15, d30, ang = settle_reads(h0, n0)
    print('battery_probe %s: turn(d30)=%+6.1f deg, gz_int=%+6.1f'
          % (tag, d30, ang))


try:
    time.sleep(1.5)
    print('=== battery probe (start) ===')
    battery_probe('start')

    print('=== E3 dead zone (one-sided pivot, longer) ===')
    for side, name in [('L', 'left'), ('R', 'right')]:
        prev = read_heading()
        for c in [0.06, 0.08, 0.10, 0.12, 0.15]:
            send(c if side == 'L' else 0.0, c if side == 'R' else 0.0)
            time.sleep(3.0)
            send(0.0, 0.0)
            time.sleep(1.5)
            h = read_heading()
            d = delta(prev, h)
            print('dead_zone %s c=%.2f -> %+.1f deg' % (name, c, d))
            prev = h
            if abs(d) > 5:
                print('  => %s side moves at c>=%.2f' % (name, c))
                break

    print('=== E1/E2 turn-rate calibration (long durations) ===')
    levels = [(0.05, 12.0), (0.10, 8.0), (0.15, 5.0), (0.20, 3.0),
              (0.30, 1.0), (0.40, 0.5)]
    for c, T in levels:
        for rep in range(2):
            for direction in ['ccw', 'cw']:
                h0 = read_heading()
                n0 = len(records)
                if direction == 'ccw':
                    send(-c, c)
                else:
                    send(c, -c)
                time.sleep(T)
                send(0.0, 0.0)
                d05, d15, d30, ang = settle_reads(h0, n0)
                w = d30 / T
                print('%s c=%.2f T=%4.1fs: d05=%+6.1f d15=%+6.1f d30=%+6.1f '
                      'omega=%+7.1f deg/s | gz_int=%+7.1f'
                      % (direction, c, T, d05, d15, d30, w, ang))

    print('=== battery probe (end) ===')
    battery_probe('end')
finally:
    send(0.0, 0.0)
    time.sleep(0.3)
    stop_flag[0] = True
    s.close()
    print('DONE (motors stopped)')
