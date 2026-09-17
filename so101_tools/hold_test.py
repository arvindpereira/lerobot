# ruff: noqa: D103
"""Safe runaway test for position-mode servos.

    uv run python so101_tools/hold_test.py <port> [motor ids...]      (default: 1..6)

For each motor: torque on with the goal set to the CURRENT position, so no
motion is requested.  A healthy servo sits still at ~0 current.  A servo with a
stale internal turn counter (or an inverted loop) drives off at full current;
the test aborts within ~0.1 s and reports RUNAWAY.  Then a small +-120 tick
tracking move is done on healthy motors.  Torque is left OFF afterwards.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import feetech_raw as fr  # noqa: E402


def hold(ser, mid: int, secs: float = 1.0):
    fr.torque(ser, mid, True)
    p = None
    for _ in range(10):
        p = fr.read(ser, mid, fr.REG_PRESENT_POSITION, 2)
        if p is not None:
            break
    fr.write_u16(ser, mid, fr.REG_GOAL_POSITION, p)
    t0, maxcur, maxdev, bad = time.time(), 0, 0, False
    while time.time() - t0 < secs:
        q = fr.read(ser, mid, fr.REG_PRESENT_POSITION, 2)
        c = fr.read(ser, mid, fr.REG_PRESENT_CURRENT, 2)
        maxcur = max(maxcur, c or 0)
        maxdev = max(maxdev, abs(q - p) if q is not None else 0)
        if maxcur > 400 or maxdev > 150:
            bad = True
            break
        time.sleep(0.03)
    if bad:
        fr.torque(ser, mid, False)
    return p, bad, maxcur, maxdev


def track(ser, mid: int):
    p1 = fr.read(ser, mid, fr.REG_PRESENT_POSITION, 2)
    mn = fr.read(ser, mid, fr.REG_MIN_LIMIT, 2) or 0
    mx = fr.read(ser, mid, fr.REG_MAX_LIMIT, 2) or 4095
    step = 120 if p1 < (mn + mx) / 2 else -120
    g = p1 + step
    fr.write_u16(ser, mid, fr.REG_GOAL_POSITION, g)
    time.sleep(1.0)
    r1 = fr.read(ser, mid, fr.REG_PRESENT_POSITION, 2)
    fr.write_u16(ser, mid, fr.REG_GOAL_POSITION, p1)
    time.sleep(1.0)
    r2 = fr.read(ser, mid, fr.REG_PRESENT_POSITION, 2)
    fr.torque(ser, mid, False)
    ok = r1 is not None and r2 is not None and abs(r1 - g) < 40 and abs(r2 - p1) < 40
    return p1, g, r1, r2, ok


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    port = sys.argv[1]
    ids = [int(x) for x in sys.argv[2:]] or list(range(1, 7))
    ser = fr.open_port(port)
    for m in range(1, 7):
        fr.torque(ser, m, False)
    failures = 0
    for mid in ids:
        name = fr.MOTOR_NAMES.get(mid, f"id{mid}")
        p, bad, mc, md = hold(ser, mid)
        if bad:
            failures += 1
            print(
                f'id{mid} {name:14s} RUNAWAY  (goal {p}, max current {mc}, moved {md} ticks) -> run: make unstick IDS="{mid}"'
            )
            continue
        p1, g, r1, r2, ok = track(ser, mid)
        failures += not ok
        print(
            f"id{mid} {name:14s} holds (current {mc}); move {p1}->{g}: reached {r1}, back {r2} -> {'TRACKS' if ok else 'NOT TRACKING'}"
        )
    ser.close()
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
