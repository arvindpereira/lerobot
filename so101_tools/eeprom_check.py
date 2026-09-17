# ruff: noqa: D103
"""Verify a follower/leader arm's motor EEPROM matches its lerobot calibration file.

    uv run python so101_tools/eeprom_check.py <port> <follower|leader> <calibration id>

Checks homing offset, position limits, Phase (must be 12), operating mode
(must be 0 = position) and the error status byte on every motor.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import feetech_raw as fr  # noqa: E402


def main() -> None:
    if len(sys.argv) != 4:
        print(__doc__)
        sys.exit(1)
    port, kind, cal_id = sys.argv[1:4]
    from lerobot.utils.constants import HF_LEROBOT_CALIBRATION

    sub = "robots/so_follower" if kind == "follower" else "teleoperators/so_leader"
    path = HF_LEROBOT_CALIBRATION / sub / f"{cal_id}.json"
    if not path.exists():
        print(f"no calibration file at {path}")
        sys.exit(1)
    cal = json.loads(path.read_text())
    ser = fr.open_port(port)
    problems = 0
    print(f"EEPROM vs {path}")
    for name, c in cal.items():
        mid = c["id"]
        phase = fr.read(ser, mid, fr.REG_PHASE, 1)
        mode = fr.read(ser, mid, fr.REG_OPERATING_MODE, 1)
        status = fr.read(ser, mid, fr.REG_STATUS, 1)
        off_raw = fr.read(ser, mid, fr.REG_HOMING_OFFSET, 2)
        off = fr.sign_magnitude(off_raw, 11) if off_raw is not None else None
        mn = fr.read(ser, mid, fr.REG_MIN_LIMIT, 2)
        mx = fr.read(ser, mid, fr.REG_MAX_LIMIT, 2)
        ok = (
            phase == 12
            and mode == 0
            and status == 0
            and off == c["homing_offset"]
            and mn == c["range_min"]
            and mx == c["range_max"]
        )
        problems += not ok
        print(
            f"  id{mid} {name:14s} phase={phase} mode={mode} status={status} "
            f"offset={off} (file {c['homing_offset']}) limits={mn}..{mx} "
            f"(file {c['range_min']}..{c['range_max']}) {'OK' if ok else 'MISMATCH'}"
        )
    ser.close()
    print("all motors match" if not problems else f"{problems} motor(s) differ from the calibration file")
    sys.exit(1 if problems else 0)


if __name__ == "__main__":
    main()
