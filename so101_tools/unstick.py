# ruff: noqa: D103
"""Recover follower joints that pin against a stop at full load in position mode.

    uv run python so101_tools/unstick.py <port> [motor ids...]      (default: 1..6)
    uv run python so101_tools/unstick.py <port> --torque-off-only

What happened on this arm (2026-09-16): two STS3215 servos kept a stale
internal multi-turn counter, so in position mode every goal looked a full
revolution away and they drove into the hard stop.  Speed mode does not use
that counter.  Briefly running the motor in speed mode and switching back to
position mode cleared the state; a power cycle of the motor supply does too.

The joint moves a few degrees each way.  Keep the arm clear.  Torque is left OFF.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import feetech_raw as fr  # noqa: E402


def set_mode(ser, mid: int, mode: int) -> None:
    fr.eeprom_write(ser, mid, fr.REG_OPERATING_MODE, [mode])


def set_velocity(ser, mid: int, v: int) -> None:
    raw = abs(v) | (0x8000 if v < 0 else 0)
    fr.write_u16(ser, mid, fr.REG_GOAL_VELOCITY, raw)


def nudge(ser, mid: int, speed: int = 150, secs: float = 0.35) -> tuple[int | None, int | None]:
    set_mode(ser, mid, 1)  # speed mode
    set_velocity(ser, mid, 0)
    fr.torque(ser, mid, True)
    p0 = fr.read(ser, mid, fr.REG_PRESENT_POSITION, 2)
    for v in (speed, -speed):
        set_velocity(ser, mid, v)
        time.sleep(secs)
        set_velocity(ser, mid, 0)
        time.sleep(0.1)
    p1 = fr.read(ser, mid, fr.REG_PRESENT_POSITION, 2)
    fr.torque(ser, mid, False)
    set_mode(ser, mid, 0)  # back to position mode
    set_velocity(ser, mid, 0)
    return p0, p1


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    port = sys.argv[1]
    ser = fr.open_port(port)
    for m in range(1, 7):
        fr.torque(ser, m, False)
    if "--torque-off-only" in sys.argv:
        print("torque disabled on motors 1-6")
        ser.close()
        return
    ids = [int(x) for x in sys.argv[2:]] or list(range(1, 7))
    for mid in ids:
        p0, p1 = nudge(ser, mid)
        mode = fr.read(ser, mid, fr.REG_OPERATING_MODE, 1)
        print(
            f"id{mid} {fr.MOTOR_NAMES.get(mid, ''):14s} speed-mode nudge {p0} -> {p1}, restored position mode (mode={mode})"
        )
    ser.close()
    print('now verify with: make hold-test IDS="' + " ".join(map(str, ids)) + '"')


if __name__ == "__main__":
    main()
