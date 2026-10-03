"""Command builders (ground station -> aircraft).

Each builder returns a :class:`Command`: the destination module, the exact
payload bytes, and whether the aircraft acknowledges it.  Payloads start with
``cmdset, cmdid`` (the reply echoes these plus the sequence number) and most
have two reserved bytes before the arguments.  Everything is little-endian.

The layouts come from the stock X8M app (see ``docs/protocol/02_commands.md``).
Builders whose meaning is inferred rather than proven say so in their
docstring.
"""

from __future__ import annotations

import datetime as _dt
import struct
from dataclasses import dataclass
from enum import Enum, IntEnum

from .modules import Module


@dataclass(frozen=True)
class Command:
    dest: int
    payload: bytes
    ack: bool = True
    src: int = Module.GCS
    name: str = ""
    timeout: float = 0.5  # per attempt, as in the app
    retries: int = 5
    flags: int | None = None  # header byte 3; default 1 if ack else 0

    @property
    def header_flags(self) -> int:
        return (1 if self.ack else 0) if self.flags is None else self.flags

    @property
    def group(self) -> int:
        return self.payload[0]

    @property
    def msg_id(self) -> int:
        return self.payload[1]

    def __str__(self) -> str:
        label = self.name or "command"
        return (
            f"{label} [{Module.name_of(self.dest)} {self.group}/{self.msg_id}] {self.payload.hex()}"
        )


def _cmd(dest: int, payload: bytes | list[int], name: str, **kw) -> Command:
    return Command(
        dest,
        bytes(b & 0xFF for b in payload) if isinstance(payload, list) else bytes(payload),
        name=name,
        **kw,
    )


def _fc(payload, name, **kw) -> Command:
    return _cmd(Module.FC, payload, name, **kw)


# ---------------------------------------------------------------------------
# Flight controller: flight actions
# ---------------------------------------------------------------------------


def takeoff() -> Command:
    """Auto take-off (FC 3/16)."""
    return _fc([3, 16, 0, 0], "takeoff")


def cancel_takeoff() -> Command:
    return _fc([3, 19, 0, 0], "cancel_takeoff")


def land() -> Command:
    """Auto land (FC 3/21). Also what the app uses for low-battery landing."""
    return _fc([3, 21, 0, 0], "land")


def cancel_land() -> Command:
    return _fc([3, 24, 0, 0], "cancel_land")


def return_home() -> Command:
    """Return to home (FC 3/26)."""
    return _fc([3, 26], "return_home")


def cancel_return_home() -> Command:
    return _fc([3, 29], "cancel_return_home")


# ---------------------------------------------------------------------------
# Flight controller: waypoint missions (point frames are in mission.py)
# ---------------------------------------------------------------------------


def mission_start() -> Command:
    """Fly the uploaded waypoint route (FC 3/32)."""
    return _fc([3, 32], "mission_start")


def mission_stop() -> Command:
    """Exit the running route (FC 3/35). There is no proven pause/resume."""
    return _fc([3, 35], "mission_stop")


def mission_3_33() -> Command:
    """Unlabelled run-overlay button (FC 3/33). Possibly pause; UNCONFIRMED."""
    return _fc([3, 33], "mission_3_33")


def mission_3_34() -> Command:
    """Unlabelled run-overlay button (FC 3/34). Possibly resume; UNCONFIRMED."""
    return _fc([3, 34], "mission_3_34")


def mission_read_point(index: int) -> Command:
    """Read back waypoint ``index`` (reply: FC 3/38, decoded as AiLinePoint)."""
    return _fc([3, 38, 0, 0, index], "mission_read_point")


def mission_read_action(index: int) -> Command:
    """Read back the action of waypoint ``index`` (reply: FC 3/39)."""
    return _fc([3, 39, 0, 0, index], "mission_read_action")


def fly_to(lat: float, lon: float, alt_m: float, speed_ms: float) -> Command:
    """Tap-to-fly / point-to-point GO (FC 3/52).

    Altitude is sent in decimetres and speed in decimetres per second, exactly
    as the app's caller scales them.  Wire order is longitude first.
    """
    p = bytearray(25)
    p[0:2] = bytes((3, 52))
    struct.pack_into("<ddh", p, 4, lon, lat, int(round(alt_m * 10)))
    p[24] = int(round(speed_ms * 10)) & 0xFF
    return _fc(bytes(p), "fly_to")


def fly_to_confirm() -> Command:
    """Arm/confirm tap-to-fly (FC 3/48), sent by the app before GO."""
    return _fc([3, 48], "fly_to_confirm")


def fly_to_exit() -> Command:
    return _fc([3, 51], "fly_to_exit")


# ---------------------------------------------------------------------------
# Flight controller: settings
# ---------------------------------------------------------------------------


class FlightMode(Enum):
    NORMAL = (8, 0)
    SMOOTH = (8, 1)  # "cinematic"
    SPORT = (7, 1)


def set_flight_mode(mode: FlightMode) -> Command:
    a, b = mode.value
    return _fc([4, 3, 0, 0, a, b], "set_flight_mode")


def set_beginner_mode(enabled: bool) -> Command:
    return _fc([4, 1, 0, 0, 0 if enabled else 2], "set_beginner_mode")


class FcParam(IntEnum):
    MAX_SPEED = 3
    MAX_HEIGHT = 5
    MAX_DISTANCE = 7


def set_fc_param(param: FcParam, value: float) -> Command:
    """Set a float FC limit (FC 4/5). Units follow the app slider (m, m/s)."""
    return _fc(bytes((4, 5, 0, 0, int(param))) + struct.pack("<f", value), "set_fc_param")


def get_fc_param(param: FcParam) -> Command:
    return _fc([4, 6, 0, 0, int(param)], "get_fc_param")


def set_rth_altitude(metres: float) -> Command:
    return _fc(bytes((4, 8, 0, 0)) + struct.pack("<f", metres), "set_rth_altitude")


def get_rth_altitude() -> Command:
    return _fc([4, 9, 0, 0], "get_rth_altitude")


def set_rc_lost_action(action: int) -> Command:
    """What the aircraft does on RC signal loss (FC 4/12); values per app UI."""
    return _fc([4, 12, 0, 0, action], "set_rc_lost_action")


def get_rc_lost_action() -> Command:
    return _fc([4, 13, 0, 0], "get_rc_lost_action")


def set_accurate_landing(enabled: bool) -> Command:
    return _fc([4, 51 if enabled else 52, 0, 0], "set_accurate_landing")


def get_pilot_mode() -> Command:
    return _fc([4, 2, 0, 0], "get_pilot_mode")


def sync_time(now: _dt.datetime | None = None) -> Command:
    """Push the local clock to the FC (FC 8/4, no ack), as the app does."""
    t = now or _dt.datetime.now()
    p = (
        bytes((8, 4, 0, 0))
        + struct.pack("<H", t.year)
        + bytes((t.month, t.day, t.hour, t.minute, t.second))
    )
    return _fc(p, "sync_time", ack=False)


# ---------------------------------------------------------------------------
# Virtual sticks
# ---------------------------------------------------------------------------

STICK_MIN, STICK_CENTRE, STICK_MAX = 0, 512, 1023

#: Key word of an idle RC as captured from a real RCX6E (bits 2..5 set).  The
#: app's own virtual-stick frame uses 0x1F1E instead, but on the real RC bit 1
#: of this word is the return-home button, and 0x1F1E has it set.
RC_KEYS_IDLE = 0x003C


def virtual_sticks(
    roll: int, pitch: int, throttle: int, yaw: int, *, wheel: int = 512, keys: int = RC_KEYS_IDLE
) -> Command:
    """Stick frame (11/2) addressed to the FC with the RC as source, no ack.

    This is the same frame the RC streams to the app with its own stick
    positions (``telemetry.RcSticks``): six channels 0..1023 (512 centred) and a
    key word.  Channel order and directions measured on a real RCX6E in mode 2:

    ========  =================  =================  ====================
    channel   stick              0                  1023
    ========  =================  =================  ====================
    roll      right, horizontal  left               right
    pitch     right, vertical    **forward (up)**   back (down)
    throttle  left, vertical     **up**             down
    yaw       left, horizontal   left               right
    ========  =================  =================  ====================

    The fifth channel is unused (always 512); the sixth is the gimbal wheel.
    Whether the aircraft obeys these frames while the physical RC is also
    sending its sticks is UNCONFIRMED: test with the propellers removed.
    """
    vals = [max(STICK_MIN, min(STICK_MAX, int(v))) for v in (roll, pitch, throttle, yaw)]
    p = bytes((11, 2, 0, 0)) + struct.pack("<4h", *vals)
    p += struct.pack("<hhH", 512, max(STICK_MIN, min(STICK_MAX, int(wheel))), keys & 0xFFFF)
    return Command(Module.FC, p, ack=False, src=Module.RC, name="virtual_sticks")


# ---------------------------------------------------------------------------
# Gimbal
# ---------------------------------------------------------------------------


def gimbal_pitch(degrees: float, rate: int = 1000) -> Command:
    """Realtime gimbal tilt (GIMBAL 9/6). 0 = level, -90 = straight down.

    Pitch goes on the wire as centidegrees (UNCONFIRMED but consistent with the
    app's ruler and the waypoint field).  ``rate`` is the slew value the app
    uses: 1000 for a tap/step, 20000 for hold-to-limit.
    """
    p = bytearray(17)
    p[0:5] = bytes((9, 6, 0, 0, 10))
    struct.pack_into("<h", p, 7, int(rate))
    struct.pack_into("<h", p, 13, int(round(degrees * 100)))
    return _cmd(Module.GIMBAL, bytes(p), "gimbal_pitch")


def gimbal_get_pitch_speed() -> Command:
    return _cmd(Module.GIMBAL, [9, 41, 0, 0], "gimbal_get_pitch_speed")


def gimbal_set_pitch_speed(value: int) -> Command:
    return _cmd(Module.GIMBAL, [9, 40, 0, 0, value], "gimbal_set_pitch_speed")


def gimbal_reset_params() -> Command:
    return _cmd(Module.GIMBAL, [9, 47], "gimbal_reset_params")


def gimbal_calibrate(start: bool = True) -> Command:
    """Start (or abort) gimbal calibration (GIMBAL 9/44). Aircraft on the ground."""
    return _cmd(Module.GIMBAL, [9, 44, 0, 0, 0 if start else 1], "gimbal_calibrate")


def gimbal_calibration_state() -> Command:
    return _cmd(Module.GIMBAL, [9, 45, 0, 0], "gimbal_calibration_state")


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------


def _cam(payload, name, **kw) -> Command:
    return _cmd(Module.CAMERA, payload, name, **kw)


def take_photo() -> Command:
    return _cam([2, 4], "take_photo")


def abort_photo() -> Command:
    return _cam([2, 5], "abort_photo")


def start_recording() -> Command:
    return _cam([2, 2], "start_recording")


def stop_recording() -> Command:
    return _cam([2, 3], "stop_recording")


class CameraKey(IntEnum):
    """Keys for :func:`set_camera_param` (value = index into the app's option list)."""

    VIDEO_RESOLUTION = 23
    EV = 25
    SHUTTER = 27
    ISO = 29
    VIDEO_QUALITY = 52
    WHITE_BALANCE = 65
    COLOUR = 67
    VIDEO_CODEC = 75
    PHOTO_SIZE = 85
    PHOTO_FORMAT = 87
    PANORAMA_MODE = 91
    RECORD_MODE = 101  # photo <-> video capture mode
    MODE_SUB = 103


def set_camera_param(key: int, index: int) -> Command:
    """Generic camera setting (CAMERA 2/key) by option index."""
    return _cam([2, int(key), 0, 0, index], "set_camera_param")


def get_camera_params() -> Command:
    """Request current camera parameters (reply CAMERA 2/64)."""
    return _cam([2, 64, 0, 0], "get_camera_params")


def set_fpv_stream(enable: bool = True, width: int = 1280, height: int = 720) -> Command:
    """Configure/start the FPV stream (CAMERA 2/114), sent by the app on connect."""
    return _cam(
        bytes((2, 114, 0, 0)) + struct.pack("<3i", int(enable), width, height), "set_fpv_stream"
    )


def set_camera_clock(now: _dt.datetime | None = None) -> Command:
    """Set the camera clock and UTC offset (CAMERA 2/135), as on connect."""
    t = (now or _dt.datetime.now()).astimezone()
    offset = int(t.utcoffset().total_seconds()) if t.utcoffset() else 0
    p = bytes((2, 135, 0, 0, t.second, t.minute, t.hour, t.day, t.month))
    p += struct.pack("<Hi", t.year, offset)
    return _cam(p, "set_camera_clock", ack=False, flags=1)


def format_sd_card() -> Command:
    """Format the aircraft SD card (CAMERA 2/9). Destroys all media."""
    return _cam([2, 9], "format_sd_card")


# ---------------------------------------------------------------------------
# One-time product activation (not needed for an already activated aircraft)
# ---------------------------------------------------------------------------

ACTIVATION_STATES = {1: b"UNACTIVATED", 2: b"ACTIVATED", 3: b"LOCKED", 4: b"DISPOSABLE"}


def activate_device(serial: bytes, state: int = 2) -> Command:
    """Device activation (FC 1/23), as the app's one-time activation dialog.

    The key is ``0x91 0x87`` + the first 14 bytes of the serial string; the
    plaintext is the state word zero-padded to 16 bytes; one AES-ECB block.
    Requires the optional ``cryptography`` package.
    """
    from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

    key = bytes((0x91, 0x87)) + bytes(serial[:14]).ljust(14, b"\0")
    plain = ACTIVATION_STATES.get(state, b"").ljust(16, b"\0")[:16]
    enc = Cipher(algorithms.AES(key), modes.ECB()).encryptor()
    block = enc.update(plain) + enc.finalize()
    return _fc(bytes((1, 23, 0, 0)) + block, "activate_device")


def raw(
    dest: int, payload: bytes, *, ack: bool = True, src: int = Module.GCS, name: str = "raw"
) -> Command:
    """Escape hatch for experimenting with opcodes not wrapped here."""
    return Command(dest, bytes(payload), ack=ack, src=src, name=name)
