"""Host-side reference codec for RadarWire SPI v1 (48 bytes).

Intended for protocol interoperability tests and UNO1/UNO2 developer tooling.
SPI physical transfers are performed by Uno firmware, not by this Python module.
"""
from dataclasses import dataclass
import math
import struct

SIZE = 48
SYNC = 0xA5
VERSION = 1
MAX_OBJECTS = 3


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 255 if crc & 0x80 else (crc << 1) & 255
    return crc


def validate(frame):
    if len(frame) != SIZE:
        raise ValueError(f"expected {SIZE} bytes, got {len(frame)}")
    if frame[:2] != bytes((SYNC, VERSION)):
        raise ValueError("invalid sync or protocol version")
    if frame[47] != crc8(frame[:47]):
        raise ValueError("CRC-8 mismatch")


def encode_command(sequence, mode, target=0, flags=0):
    if not 0 <= sequence <= 255 or not 0 <= mode <= 4 or not 0 <= target <= 255:
        raise ValueError("sequence/command/target outside allowed range")
    if flags & ~1:
        raise ValueError("unknown v1 command flag")
    frame = bytearray(SIZE)
    frame[:6] = bytes((SYNC, VERSION, sequence, mode, flags, target))
    frame[47] = crc8(frame[:47])
    return bytes(frame)


@dataclass(frozen=True)
class Object:
    id: int
    flags: int
    x_mm: float
    y_mm: float
    r_mm: float
    theta_deg: float
    vx_mm_s: float
    vy_mm_s: float
    vr_mm_s: float
    vt_mm_s: float
    quality: int
    age_scans: int


@dataclass(frozen=True)
class Telemetry:
    sequence: int
    mode: int
    status: int
    state: int
    hall_strikes: int
    selected_target: int
    command_ack: int
    error: int
    cylindrical_wire: bool
    objects: tuple


def decode_telemetry(frame):
    validate(frame)
    count = frame[5]
    if count > MAX_OBJECTS:
        raise ValueError("object count exceeds 3")
    cylindrical = bool(frame[4] & 0x20)
    objects = []
    for index in range(count):
        at = 6 + 12 * index
        ident, flags = struct.unpack_from("<BB", frame, at)
        if ident == 0:
            raise ValueError("occupied slot has zero ID")
        if cylindrical:
            r, angle, vr, vt = struct.unpack_from("<HHhh", frame, at + 2)
            if not 0 <= angle < 36000:
                raise ValueError("cylindrical theta out of range")
            radians = math.radians(angle / 100)
            c, s = math.cos(radians), math.sin(radians)
            x, y = 500 + r * c, 500 + r * s
            vx, vy = vr * c - vt * s, vr * s + vt * c
            theta = angle / 100.0
        else:
            x, y, vx, vy = struct.unpack_from("<hhhh", frame, at + 2)
            dx, dy = x - 500, y - 500
            r = math.hypot(dx, dy)
            theta = math.degrees(math.atan2(dy, dx)) % 360
            inv = 1/r if r > 0 else 0
            vr, vt = (dx * vx + dy * vy) * inv, (-dy * vx + dx * vy) * inv
        quality, age = struct.unpack_from("<BB", frame, at + 10)
        objects.append(Object(ident, flags, x, y, r, theta, vx, vy, vr, vt, quality, age))
    return Telemetry(sequence=frame[2], mode=frame[3], status=frame[4],
        state=frame[42], hall_strikes=frame[43], selected_target=frame[44],
        command_ack=frame[45], error=frame[46],
        cylindrical_wire=cylindrical, objects=tuple(objects))
