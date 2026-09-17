# ruff: noqa: D103
"""Probe a Feetech servo bus and explain what it finds.

    uv run python so101_tools/bus_probe.py /dev/tty.usbmodemXXXX [--deep]

Quick mode broadcasts a ping at 1 Mbaud and lists the motor IDs found.
--deep additionally scans every baudrate and collects raw reply statistics per
ID, which separates the three states lerobot's own scanner cannot tell apart:

  * silent bus        -> no motor powered / cable not seated / dead motor
  * collisions        -> several motors share one ID (factory default is 1)
  * healthy           -> clean checksum-valid replies
"""

from __future__ import annotations

import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import feetech_raw as fr  # noqa: E402


def broadcast(port: str) -> dict[int, int] | None:
    from lerobot.motors.feetech import FeetechMotorsBus

    bus = FeetechMotorsBus(port, {})
    bus._connect(handshake=False)
    bus.set_baudrate(fr.BAUD)
    found = bus.broadcast_ping(num_retry=3)
    bus.port_handler.closePort()
    return found


def deep_scan(port: str) -> None:
    from lerobot.motors.feetech import FeetechMotorsBus

    print("Scanning all baudrates ...")
    print("  ", FeetechMotorsBus.scan_port(port) or "nothing found at any baudrate")


def raw_stats(port: str, ids=range(1, 8), n: int = 20) -> None:
    ser = fr.open_port(port)
    any_reply = False
    print(f"Raw reply statistics ({n} pings + {n // 2} position reads per ID):")
    for mid in ids:
        pings = Counter(fr.txrx(ser, fr.ping_packet(mid), 0.012).hex() for _ in range(n))
        replies = sum(v for k, v in pings.items() if k)
        if replies == 0:
            continue
        any_reply = True
        clean = sum(v for k, v in pings.items() if k and fr.valid_status(bytes.fromhex(k)))
        pos_ok = 0
        values: Counter = Counter()
        for _ in range(n // 2):
            pkt = fr.txrx(ser, fr.read_packet(mid, fr.REG_PRESENT_POSITION, 2), 0.012)
            if fr.valid_status(pkt) and len(pkt) >= 8:
                pos_ok += 1
                values[int.from_bytes(pkt[5:7], "little")] += 1
        verdict = "healthy" if clean == replies and pos_ok == n // 2 else "COLLISIONS or noisy bus"
        print(
            f"  id {mid}: {clean}/{replies} clean pings, {pos_ok}/{n // 2} valid position reads {dict(values)} -> {verdict}"
        )
    ser.close()
    if not any_reply:
        print("  no motor answered any ID -> bus is silent (check motor power, cable seating, board header)")


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    port, deep = sys.argv[1], "--deep" in sys.argv
    print(f"PORT {port}")
    try:
        found = broadcast(port)
    except Exception as e:  # noqa: BLE001
        print(f"  could not open port: {e}")
        sys.exit(1)
    if found:
        print(f"  motors answering at {fr.BAUD} baud: {sorted(found)}  (model numbers {found})")
    else:
        print(f"  no valid replies at {fr.BAUD} baud")
    if deep or not found:
        if deep:
            deep_scan(port)
        raw_stats(port)
    time.sleep(0.1)


if __name__ == "__main__":
    main()
