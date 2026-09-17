# ruff: noqa: D103
"""Program ONE physically isolated motor with its ID and baudrate, then verify.

    uv run python so101_tools/setup_one_motor.py <port> <follower|leader> <motor_name>

Equivalent to one step of `lerobot-setup-motors`, but non-interactive, so you
can do the motors in any order or redo a single one.  The motor must be the
only device on the bus (unplug the daisy-chain cable from its second port).
"""

from __future__ import annotations

import sys


def main() -> None:
    if len(sys.argv) != 4:
        print(__doc__)
        sys.exit(1)
    port, kind, motor = sys.argv[1:4]
    if kind == "follower":
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

        dev = SO101Follower(SO101FollowerConfig(port=port, id="setup_tmp"))
    elif kind == "leader":
        from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

        dev = SO101Leader(SO101LeaderConfig(port=port, id="setup_tmp"))
    else:
        print("second argument must be 'follower' or 'leader'")
        sys.exit(1)
    if motor not in dev.bus.motors:
        print(f"unknown motor '{motor}'. Choose from: {list(dev.bus.motors)}")
        sys.exit(1)

    bus = dev.bus
    bus._connect(handshake=False)
    bus.set_baudrate(bus.default_baudrate)
    before = bus.broadcast_ping(num_retry=3)
    print("before:", before)
    if before and len(before) > 1:
        print(f"ABORT: {len(before)} motors answer on the bus; isolate the '{motor}' motor first.")
        bus.port_handler.closePort()
        sys.exit(2)
    bus.setup_motor(motor)
    target = bus.motors[motor].id
    bus.set_baudrate(bus.default_baudrate)
    after = bus.broadcast_ping(num_retry=3)
    print("after :", after)
    ok = after is not None and list(after) == [target]
    print(f"RESULT: '{motor}' -> id {target} at {bus.default_baudrate} baud: {'OK' if ok else 'MISMATCH'}")
    bus.port_handler.closePort()
    sys.exit(0 if ok else 3)


if __name__ == "__main__":
    main()
