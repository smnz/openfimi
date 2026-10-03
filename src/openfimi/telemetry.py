"""Decoders for messages the aircraft sends (telemetry pushes and replies).

Messages are keyed by ``(src module, group, msg_id)``.  Offsets below are into
the message *body* (payload offset 4).  Scales marked UNCERTAIN are exposed raw
(``*_raw``) alongside the best current interpretation, so captures can settle
them later without breaking callers.
"""

from __future__ import annotations

import struct
from collections.abc import Callable
from dataclasses import dataclass, fields

from .framing import Frame
from .modules import Module


class _Msg:
    """Base for decoded messages."""

    KEY: tuple[int, int, int] = (-1, -1, -1)

    def as_dict(self) -> dict:
        return {f.name: getattr(self, f.name) for f in fields(self)}  # type: ignore[arg-type]


def _need(body: bytes, n: int, name: str) -> None:
    if len(body) < n:
        raise ValueError(f"{name}: body {len(body)} B < {n} B")


# --- FC group 12: the continuous telemetry push -----------------------------


@dataclass
class FcHeart(_Msg):
    """Flight phase and control state (FC 12/1)."""

    KEY = (Module.FC, 12, 1)
    flight_time: int
    startup_time: int
    ctrl_type: int
    candidate_ctrl_type: int
    flight_phase: int
    ctrl_model: int
    system_phase: int
    disarm_count: int
    power_con_rate: int
    takeoff_cap: int
    auto_takeoff_cap: int

    @property
    def flying(self) -> bool:
        return self.flight_phase in (2, 3, 4)

    @property
    def on_ground(self) -> bool:
        return self.flight_phase in (0, 1, 5)

    @property
    def can_take_off(self) -> bool:
        return bool(self.takeoff_cap and self.auto_takeoff_cap)

    @classmethod
    def decode(cls, b: bytes) -> FcHeart:
        _need(b, 13, "FcHeart")
        ft, st = struct.unpack_from("<hh", b, 0)
        return cls(ft, st, *b[4:13])


@dataclass
class FcSportState(_Msg):
    """Position, height, speed, attitude (FC 12/2). The primary OSD message."""

    KEY = (Module.FC, 12, 2)
    lat: float
    lon: float
    height_m: float  # relative to take-off
    ground_speed_raw: int  # UNCERTAIN scale
    down_velocity_raw: int  # UNCERTAIN scale
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    home_distance_m: float
    extra_raw: int  # UNCERTAIN meaning

    @classmethod
    def decode(cls, b: bytes) -> FcSportState:
        _need(b, 36, "FcSportState")
        lon, lat, h, gs, dv, r, p, y = struct.unpack_from("<ddfhhhhh", b, 0)
        (hd,) = struct.unpack_from("<f", b, 32)
        extra = struct.unpack_from("<h", b, 36)[0] if len(b) >= 38 else 0
        return cls(lat, lon, h, gs, dv, r / 10, p / 10, y / 10, hd, extra)


@dataclass
class FcSignalState(_Msg):
    """GPS satellites and RC link quality (FC 12/3)."""

    KEY = (Module.FC, 12, 3)
    satellites: int
    gps_acc1: int  # UNCERTAIN (DOP-like)
    gps_acc2: int  # UNCERTAIN
    gps_acc3: int  # UNCERTAIN
    rc_signal: int  # 0..100 %
    rc_aux_raw: int

    @classmethod
    def decode(cls, b: bytes) -> FcSignalState:
        _need(b, 8, "FcSignalState")
        return cls(b[0], b[1], b[2], b[4], b[6], b[7])


@dataclass
class FcErrCode(_Msg):
    """Four 32-bit status/fault bitmasks (FC 12/4); bit meanings unknown."""

    KEY = (Module.FC, 12, 4)
    a: int
    b: int
    c: int
    d: int

    @classmethod
    def decode(cls, b: bytes) -> FcErrCode:
        _need(b, 16, "FcErrCode")
        return cls(*struct.unpack_from("<4I", b, 0))


@dataclass
class FcBattery(_Msg):
    """Battery (FC 12/5)."""

    KEY = (Module.FC, 12, 5)
    cells_v: tuple[float, ...]
    capacity_mah: int
    total_capacity_mah: int
    current_raw: int  # UNCERTAIN unit
    temperature_c: float
    remaining_time_raw: int  # UNCERTAIN unit
    percent: int
    uvc: int
    rc_not_update_cnt: int
    cycle_raw: int  # UNCERTAIN (cycle or coulomb count)

    @property
    def voltage(self) -> float:
        return round(sum(self.cells_v), 3)

    @classmethod
    def decode(cls, b: bytes) -> FcBattery:
        _need(b, 17, "FcBattery")
        cells = tuple(round(c / 100 + 2.0, 2) for c in b[0:4] if c)
        cap, total, cur, temp, rem = struct.unpack_from("<hhhhh", b, 4)
        cc = struct.unpack_from("<h", b, 22)[0] if len(b) >= 24 else 0
        return cls(cells, cap, total, cur, temp / 10, rem, b[14], b[15], b[16], cc)


@dataclass
class HomeInfo(_Msg):
    """Home point (FC 12/6)."""

    KEY = (Module.FC, 12, 6)
    lat: float
    lon: float
    height_m: float
    accuracy: int
    point_type: int
    status: int

    @classmethod
    def decode(cls, b: bytes) -> HomeInfo:
        _need(b, 23, "HomeInfo")
        lon, lat, h = struct.unpack_from("<ddf", b, 0)
        return cls(lat, lon, h, b[20], b[21], b[22])


@dataclass
class ImuInfo(_Msg):
    """Raw IMU/baro/ToF sensors (FC 12/7); raw units, no scaling."""

    KEY = (Module.FC, 12, 7)
    imu_type: int
    imu_temp: int
    gyro: tuple[int, int, int]
    accel: tuple[int, int, int]
    mag: tuple[int, int, int]
    baro_temp: int
    baro_alt: int
    tof_distance: int
    tof_amp: int
    tof_temp: int
    tof_ambient: int

    @classmethod
    def decode(cls, b: bytes) -> ImuInfo:
        _need(b, 30, "ImuInfo")
        v = struct.unpack_from("<13h", b, 2)
        return cls(b[0], b[1], v[0:3], v[3:6], v[6:9], v[9], v[10], v[11], v[12], b[28], b[29])


# --- FC group 3: navigation / AI-fly ----------------------------------------


@dataclass
class NavigationState(_Msg):
    """Autopilot / mission state (FC 3/1). Value meanings to be mapped by capture."""

    KEY = (Module.FC, 3, 1)
    task_mode: int
    navi_task_state: int
    ap_status: int
    waypoint: int

    @classmethod
    def decode(cls, b: bytes) -> NavigationState:
        _need(b, 5, "NavigationState")
        return cls(b[0], b[1], b[2], struct.unpack_from("<H", b, 3)[0])


@dataclass
class AiLinePoint(_Msg):
    """Read-back of an uploaded waypoint (FC 3/38)."""

    KEY = (Module.FC, 3, 38)
    index: int
    count: int
    lat: float
    lon: float
    alt_m: float
    yaw_raw: int
    gimbal_pitch_raw: int
    speed_raw: int
    yaw_mode: int
    rotation: int
    gimbal_mode: int
    trajectory_mode: int
    finish_action: int
    rc_lost_action: int
    poi_lat: float
    poi_lon: float
    poi_alt_m: float

    @classmethod
    def decode(cls, b: bytes) -> AiLinePoint:
        _need(b, 54, "AiLinePoint")
        lon, lat, alt, yaw, gp = struct.unpack_from("<ddhhh", b, 4)
        nib = b[31]
        plon, plat, palt = struct.unpack_from("<ddh", b, 36)
        return cls(
            b[0],
            b[1],
            lat,
            lon,
            alt / 10,
            yaw,
            gp,
            b[26],
            nib & 0x0F,
            nib >> 4,
            b[32],
            b[33],
            b[34],
            b[35],
            plat,
            plon,
            palt / 10,
        )


# --- Gimbal / camera ---------------------------------------------------------


@dataclass
class GimbalState(_Msg):
    """Gimbal attitude (GIMBAL 9/1). Angle scale UNCERTAIN (0.1 or 0.01 deg)."""

    KEY = (Module.GIMBAL, 9, 1)
    error_code: int
    state_code: int
    roll_raw: int
    pitch_raw: int
    yaw_raw: int

    @classmethod
    def decode(cls, b: bytes) -> GimbalState:
        _need(b, 10, "GimbalState")
        err = struct.unpack_from("<h", b, 0)[0]
        r, p, y = struct.unpack_from("<hhh", b, 4)
        return cls(err, b[2], r, p, y)


@dataclass
class CameraState(_Msg):
    """Camera status (CAMERA 2/21): mode, recording time, card space."""

    KEY = (Module.CAMERA, 2, 21)
    state: int
    mode: int
    info: int
    rec_seconds: int
    free_space: int
    total_space: int

    @classmethod
    def decode(cls, b: bytes) -> CameraState:
        _need(b, 13, "CameraState")
        rt = struct.unpack_from("<H", b, 3)[0]
        secs = (rt & 63) + ((rt >> 6) & 63) * 60 + ((rt >> 12) & 63) * 3600
        free, total = struct.unpack_from("<ii", b, 5)
        return cls(b[0], b[1], b[2], secs, free, total)


@dataclass
class FpvInfo(_Msg):
    """FPV encoder info (CAMERA 2/115)."""

    KEY = (Module.CAMERA, 2, 115)
    en_type: int
    width: int
    height: int

    @classmethod
    def decode(cls, b: bytes) -> FpvInfo:
        _need(b, 12, "FpvInfo")
        return cls(*struct.unpack_from("<iii", b, 0))


# --- RC / relay ----------------------------------------------------------------


@dataclass
class RcSticks(_Msg):
    """The RC's own stick, wheel and button state (RC 11/2, about 8 Hz).

    Measured on an RCX6E (mode 2): channels are 0..1023, 512 centred; forward
    and up read LOW.  ``keys`` low bits: bit 1 = return-home (set while
    pressed), bit 2 = video and bit 3 = photo (cleared while pressed).  The
    upper bits of ``keys`` vary continuously and are not understood.
    """

    KEY = (Module.RC, 11, 2)
    roll: int
    pitch: int
    throttle: int
    yaw: int
    aux: int
    wheel: int  # gimbal tilt wheel
    keys: int

    @property
    def rth_pressed(self) -> bool:
        return bool(self.keys & 0x02)

    @property
    def video_pressed(self) -> bool:
        return not self.keys & 0x04

    @property
    def photo_pressed(self) -> bool:
        return not self.keys & 0x08

    @classmethod
    def decode(cls, b: bytes) -> RcSticks:
        _need(b, 14, "RcSticks")
        return cls(*struct.unpack_from("<6hH", b, 0))


@dataclass
class RcHeart(_Msg):
    """RC heartbeat (RC 11/1, 1 Hz).  The app reads only the first four bytes.

    On a real RCX6E the first field fell 397 -> 394 and the second 98 -> 97 over
    a few minutes on battery, so they look like RC battery centivolts and
    percent (UNCONFIRMED).
    """

    KEY = (Module.RC, 11, 1)
    voltage_raw: int
    percent: int
    flags: int
    rest: bytes

    @classmethod
    def decode(cls, b: bytes) -> RcHeart:
        _need(b, 4, "RcHeart")
        return cls(struct.unpack_from("<H", b, 0)[0], b[2], b[3], bytes(b[4:]))


@dataclass
class RcState(_Msg):
    KEY = (Module.RC, 11, 4)
    state: int
    error_state: int

    @classmethod
    def decode(cls, b: bytes) -> RcState:
        _need(b, 2, "RcState")
        return cls(b[0], b[1])


@dataclass
class RelayHeart(_Msg):
    """RC-side relay heartbeat (REPEATER_RC 14/5); signal = (status>>12)&3."""

    KEY = (Module.REPEATER_RC, 14, 5)
    status: int

    @property
    def signal(self) -> int:
        return (self.status >> 12) & 3

    @classmethod
    def decode(cls, b: bytes) -> RelayHeart:
        _need(b, 2, "RelayHeart")
        return cls(struct.unpack_from("<H", b, 0)[0])


@dataclass
class Version(_Msg):
    """Firmware version reply (any module, 16/177). Body kept raw for now."""

    KEY = (-1, 16, 177)
    module: int
    raw: bytes

    @classmethod
    def decode(cls, b: bytes) -> Version:
        return cls(-1, bytes(b))


DECODERS: dict[tuple[int, int, int], Callable[[bytes], _Msg]] = {
    cls.KEY: cls.decode
    for cls in (
        FcHeart,
        FcSportState,
        FcSignalState,
        FcErrCode,
        FcBattery,
        HomeInfo,
        ImuInfo,
        NavigationState,
        AiLinePoint,
        GimbalState,
        CameraState,
        FpvInfo,
        RcState,
        RcSticks,
        RcHeart,
        RelayHeart,
    )
}


def decode(frame: Frame) -> _Msg | None:
    """Decode a frame into a message object, or None if no decoder is known."""
    fn = DECODERS.get(frame.key)
    if fn is not None:
        return fn(frame.body)
    if (frame.group, frame.msg_id) == (16, 177):
        v = Version.decode(frame.body)
        v.module = frame.src
        return v
    return None
