# ruff: noqa: D103
"""Minimal raw-serial helpers for Feetech/Waveshare STS3215-class servos.

Used by the diagnostic scripts in this folder. Talks to the bus directly with
pyserial so it keeps working even when the SDK rejects corrupted packets
(which is exactly the situation we want to diagnose).
"""

from __future__ import annotations

import time

import serial

BAUD = 1_000_000

# Register addresses (STS3215 control table)
REG_PHASE = 18
REG_MIN_LIMIT = 9
REG_MAX_LIMIT = 11
REG_HOMING_OFFSET = 31
REG_OPERATING_MODE = 33
REG_TORQUE_ENABLE = 40
REG_GOAL_POSITION = 42
REG_GOAL_VELOCITY = 46
REG_LOCK = 55
REG_PRESENT_POSITION = 56
REG_PRESENT_LOAD = 60
REG_STATUS = 65
REG_PRESENT_CURRENT = 69


def open_port(port: str, baud: int = BAUD, timeout: float = 0.05) -> serial.Serial:
    return serial.Serial(port, baud, timeout=timeout)


def checksum(body: list[int]) -> int:
    return (~sum(body)) & 0xFF


def ping_packet(motor_id: int) -> bytes:
    body = [motor_id, 2, 1]
    return bytes([0xFF, 0xFF] + body + [checksum(body)])


def read_packet(motor_id: int, addr: int, length: int) -> bytes:
    body = [motor_id, 4, 2, addr, length]
    return bytes([0xFF, 0xFF] + body + [checksum(body)])


def write_packet(motor_id: int, addr: int, data: list[int]) -> bytes:
    body = [motor_id, len(data) + 3, 3, addr] + data
    return bytes([0xFF, 0xFF] + body + [checksum(body)])


def valid_status(pkt: bytes) -> bool:
    """True if pkt is a complete status packet with a correct checksum."""
    if len(pkt) < 6 or pkt[0:2] != b"\xff\xff":
        return False
    n = pkt[3]
    return len(pkt) == 4 + n and checksum(list(pkt[2 : 3 + n])) == pkt[3 + n]


def txrx(ser: serial.Serial, pkt: bytes, settle: float = 0.008) -> bytes:
    ser.reset_input_buffer()
    ser.write(pkt)
    time.sleep(settle)
    return ser.read(64)


def read(ser: serial.Serial, motor_id: int, addr: int, length: int, retries: int = 5) -> int | None:
    for _ in range(retries):
        pkt = txrx(ser, read_packet(motor_id, addr, length))
        if valid_status(pkt) and len(pkt) >= 6 + length:
            return int.from_bytes(pkt[5 : 5 + length], "little")
    return None


def write(ser: serial.Serial, motor_id: int, addr: int, data: list[int], repeats: int = 2) -> None:
    for _ in range(repeats):
        txrx(ser, write_packet(motor_id, addr, data))


def write_u16(ser: serial.Serial, motor_id: int, addr: int, value: int) -> None:
    write(ser, motor_id, addr, [value & 0xFF, (value >> 8) & 0xFF])


def torque(ser: serial.Serial, motor_id: int, on: bool) -> None:
    write(ser, motor_id, REG_TORQUE_ENABLE, [1 if on else 0])


def eeprom_write(ser: serial.Serial, motor_id: int, addr: int, data: list[int]) -> None:
    """Unlock EEPROM, write, re-lock. Torque must be off."""
    torque(ser, motor_id, False)
    write(ser, motor_id, REG_LOCK, [0])
    time.sleep(0.03)
    write(ser, motor_id, addr, data)
    time.sleep(0.03)
    write(ser, motor_id, REG_LOCK, [1])
    time.sleep(0.1)


def sign_magnitude(value: int, sign_bit: int) -> int:
    mask = 1 << sign_bit
    return -(value & (mask - 1)) if value & mask else value


MOTOR_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}
