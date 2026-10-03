"""Module (endpoint) identifiers carried in the FmLink4 header src/dest bytes."""

from __future__ import annotations

from enum import IntEnum


class Module(IntEnum):
    IDLE = 0
    UAV = 1
    FC = 2  # flight controller
    CAMERA = 3
    OPTFLOW = 4
    OBSAVOID = 5
    HTTP = 6
    GCS = 7  # the ground station: us
    GIMBAL = 8
    BLACKBOX = 9
    CV = 10  # vision / tracking
    SV_DWN = 11
    SV_FW = 12
    RC = 13
    REPEATER_VEHICLE = 14
    BATTERY = 15
    REPEATER_RC = 16
    NFZ = 17  # no-fly zones
    ESC = 18
    SERVO = 19
    DEFAULT_0X14 = 20
    DEFAULT_0X15 = 21
    ULTRASONIC = 22

    @classmethod
    def name_of(cls, value: int) -> str:
        try:
            return cls(value).name
        except ValueError:
            return f"MODULE_{value}"
