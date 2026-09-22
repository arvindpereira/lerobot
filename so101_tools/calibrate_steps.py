# ruff: noqa: D103
"""Non-interactive, two-step calibration (same maths as lerobot-calibrate).

    uv run python so101_tools/calibrate_steps.py center <port> <follower|leader> <id>
    uv run python so101_tools/calibrate_steps.py sweep  <port> <follower|leader> <id> [--seconds N]

`center`: with every joint at the middle of its travel, writes a homing offset
so each joint reads half a turn (2047).  `sweep`: records min/max of every
joint for N seconds while you move each joint to both hard stops, then writes
the calibration to the motors and saves the json lerobot expects.  Readings
that leave the 0..4095 circle during the sweep are reported, because on some
firmware they mean the servo's turn counter has tripped (see unstick.py).
"""

from __future__ import annotations

import argparse
import time

FULL_TURN_MOTOR = "wrist_roll"


def make_device(port: str, kind: str, cal_id: str):
    if kind == "follower":
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

        return SO101Follower(SO101FollowerConfig(port=port, id=cal_id))
    from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

    return SO101Leader(SO101LeaderConfig(port=port, id=cal_id))


def connect_bus(dev):
    from lerobot.motors.feetech import OperatingMode

    dev.bus.connect(handshake=True)
    dev.bus.disable_torque()
    for motor in dev.bus.motors:
        dev.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)


def center(dev) -> None:
    before = dev.bus.sync_read("Present_Position", normalize=False)
    bad = {m: p for m, p in before.items() if p > 4095}
    if bad:
        print(
            f"ABORT: readings outside 0..4095 (turn counter tripped): {bad}. Run unstick / power-cycle first."
        )
        raise SystemExit(2)
    homings = dev.bus.set_half_turn_homings()
    after = dev.bus.sync_read("Present_Position", normalize=False)
    print(f"{'joint':14s} {'raw before':>10s} {'offset':>7s} {'reads now':>9s}")
    for m in dev.bus.motors:
        print(f"{m:14s} {before[m]:10d} {homings[m]:7d} {after[m]:9d}")
    print("centre recorded. Next: sweep every joint to both hard stops during `make cal-sweep`.")


def sweep(dev, seconds: float) -> None:
    from lerobot.motors import MotorCalibration

    motors = list(dev.bus.motors)
    mins = dev.bus.sync_read("Present_Position", motors, normalize=False)
    maxes = dict(mins)
    last = dict(mins)
    anomalies: list[str] = []
    t0 = time.time()
    next_print = 0.0
    while time.time() - t0 < seconds:
        pos = dev.bus.sync_read("Present_Position", motors, normalize=False, num_retry=5)
        for m, p in pos.items():
            if p > 4095 or abs(p - last[m]) > 2000:
                anomalies.append(f"{m}: {last[m]} -> {p} at {time.time() - t0:.1f}s")
            last[m] = p
            mins[m] = min(mins[m], p)
            maxes[m] = max(maxes[m], p)
        if time.time() - t0 >= next_print:
            remaining = seconds - (time.time() - t0)
            print(
                f"  {remaining:4.0f}s left  " + "  ".join(f"{m}={mins[m]}..{maxes[m]}" for m in motors),
                flush=True,
            )
            next_print += 5.0
        time.sleep(0.02)
    mins[FULL_TURN_MOTOR], maxes[FULL_TURN_MOTOR] = 0, 4095
    homings = dev.bus.sync_read("Homing_Offset", motors, normalize=False)
    calibration = {
        m: MotorCalibration(
            id=dev.bus.motors[m].id,
            drive_mode=0,
            homing_offset=homings[m],
            range_min=mins[m],
            range_max=maxes[m],
        )
        for m in motors
    }
    print(f"\n{'joint':14s} {'offset':>7s} {'min':>5s} {'max':>5s} {'span deg':>9s}")
    for m in motors:
        print(f"{m:14s} {homings[m]:7d} {mins[m]:5d} {maxes[m]:5d} {(maxes[m] - mins[m]) * 360 / 4095:9.1f}")
    if anomalies:
        print("\nWARNING: readings left the single-turn range during the sweep (turn counter tripped?):")
        for a in anomalies[:10]:
            print("  " + a)
        print("NOT saving. Run `make unstick`, then redo cal-center and cal-sweep.")
        raise SystemExit(3)
    dev.bus.write_calibration(calibration)
    dev.calibration = calibration
    dev._save_calibration()
    print(f"\nCalibration written to motors and saved to {dev.calibration_fpath}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("step", choices=["center", "sweep"])
    ap.add_argument("port")
    ap.add_argument("kind", choices=["follower", "leader"])
    ap.add_argument("cal_id")
    ap.add_argument("--seconds", type=float, default=30.0)
    a = ap.parse_args()
    dev = make_device(a.port, a.kind, a.cal_id)
    connect_bus(dev)
    try:
        if a.step == "center":
            center(dev)
        else:
            sweep(dev, a.seconds)
    finally:
        dev.bus.disable_torque()
        dev.bus.disconnect(disable_torque=True)


if __name__ == "__main__":
    main()
