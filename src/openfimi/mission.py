"""Waypoint missions ("AiLine" routes).

A route is uploaded one point at a time (FC 3/36, 58-byte frame) followed by
one action frame per point (FC 3/37), then started with FC 3/32 and exited
with FC 3/35.  There is no separate route header: every point carries its
index and the point count, plus the route-wide settings.

Units on the wire: coordinates are WGS-84 f64 (longitude first), altitudes are
decimetres, speed is decimetres/second, yaw and gimbal pitch are centidegrees.

Flight-tested behaviour of the *stock* firmware (see the project notes):

* the per-waypoint gimbal pitch is transmitted but ignored on routes; tilt the
  gimbal yourself with :func:`openfimi.commands.gimbal_pitch` during a hover;
* a POI steers yaw only, and the photo taken at waypoint *k* faces the POI
  stored on waypoint *k+1*;
* the app sends one route-wide speed, heading mode, finish and RC-lost action.
  openfimi lets you set them per point, but behaviour beyond what the app
  sends is unverified; test before relying on it.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from enum import IntEnum

from . import commands
from .commands import Command
from .modules import Module


class Heading(IntEnum):
    """Aircraft yaw during the route (route TYPE / waypoint yawMode)."""

    FREE = 0  # yaw left to the pilot's stick; POIs are honoured in this mode
    WAYPOINT = 1  # per-waypoint yaw (only reachable from the app's fly-now screen)
    ROUTE = 2  # nose follows the route


class Rotation(IntEnum):
    """How the aircraft swings to a new heading."""

    MIN_ANGLE = 0
    CW = 1
    CCW = 2


class FinishAction(IntEnum):
    """Route-end behaviour (other raw values exist but are unnamed)."""

    HOVER = 0
    RETURN_HOME = 4


class LostAction(IntEnum):
    """Behaviour on RC signal loss during the route."""

    EXIT = 0
    CONTINUE = 1


class Action(IntEnum):
    """Wire action codes for a point action slot."""

    NONE = 0
    HOVER = 1
    PHOTO = 2
    VIDEO = 3
    SLOW_VIDEO = 4
    PANORAMA = 5


@dataclass
class PointAction:
    """What to do on arrival at a waypoint.

    The frame has two action slots and three parameter bytes.  Their meaning is
    inferred from the presets the app sends:

    =================  ======  ======  ==  ==  ==
    preset             first   second  p1  p2  p3
    =================  ======  ======  ==  ==  ==
    hover 10 s         HOVER   NONE    10  1   0
    record 10 s        VIDEO   NONE    10  1   0
    single photo       PHOTO   NONE    1   1   0
    burst of 3         PHOTO   NONE    1   3   0
    hover 5 s + photo  HOVER   PHOTO   5   1   1
    =================  ======  ======  ==  ==  ==

    i.e. p1 = duration in seconds (or 1 for a photo), p2 = repeat count for the
    first action, p3 = count for the second action.
    """

    first: Action = Action.NONE
    second: Action = Action.NONE
    p1: int = 0
    p2: int = 0
    p3: int = 0

    @classmethod
    def none(cls) -> PointAction:
        return cls()

    @classmethod
    def hover(cls, seconds: int = 10) -> PointAction:
        return cls(Action.HOVER, Action.NONE, seconds, 1, 0)

    @classmethod
    def record(cls, seconds: int = 10) -> PointAction:
        return cls(Action.VIDEO, Action.NONE, seconds, 1, 0)

    @classmethod
    def photo(cls, count: int = 1) -> PointAction:
        return cls(Action.PHOTO, Action.NONE, 1, count, 0)

    @classmethod
    def hover_then_photo(cls, seconds: int = 5, photos: int = 1) -> PointAction:
        return cls(Action.HOVER, Action.PHOTO, seconds, 1, photos)


@dataclass
class Waypoint:
    lat: float
    lon: float
    alt_m: float  # relative to take-off
    action: PointAction = field(default_factory=PointAction)
    poi: tuple[float, float, float] | None = None  # (lat, lon, alt_m)
    yaw_deg: float = 0.0  # used in Heading.WAYPOINT
    gimbal_pitch_deg: float = 0.0  # transmitted, ignored by stock firmware on routes
    rotation: Rotation = Rotation.MIN_ANGLE
    # Per-point overrides of the route settings (None = use the route value).
    speed_ms: float | None = None
    heading: Heading | None = None


@dataclass
class Mission:
    waypoints: list[Waypoint]
    speed_ms: float = 5.0
    heading: Heading = Heading.FREE
    finish: FinishAction | int = FinishAction.HOVER
    rc_lost: LostAction | int = LostAction.EXIT
    auto_record: bool = False
    coordinated_turn_off: bool = False

    MAX_POINTS = 255  # the index/count fields are bytes; the app caps at 20
    MAX_SPEED_MS = 14.0  # vehicle limit

    def validate(self) -> None:
        n = len(self.waypoints)
        if not 1 <= n <= self.MAX_POINTS:
            raise ValueError(f"route needs 1..{self.MAX_POINTS} waypoints, got {n}")
        for i, wp in enumerate(self.waypoints):
            spd = self.speed_ms if wp.speed_ms is None else wp.speed_ms
            if not 0 < spd <= self.MAX_SPEED_MS:
                raise ValueError(f"waypoint {i}: speed {spd} m/s outside (0, {self.MAX_SPEED_MS}]")
            if not (-90 <= wp.lat <= 90 and -180 <= wp.lon <= 180):
                raise ValueError(f"waypoint {i}: bad coordinates")
            if not -3276.8 <= wp.alt_m <= 3276.7:
                raise ValueError(f"waypoint {i}: altitude out of range")

    # ------------------------------------------------------------------
    def point_commands(self) -> list[Command]:
        n = len(self.waypoints)
        return [waypoint_frame(self, i, n) for i in range(n)]

    def action_commands(self) -> list[Command]:
        n = len(self.waypoints)
        return [action_frame(wp.action, i, n) for i, wp in enumerate(self.waypoints)]

    def upload_commands(self) -> list[Command]:
        """All frames in the app's upload order: every point, then every action."""
        self.validate()
        return self.point_commands() + self.action_commands()


def _wrap_yaw(deg: float) -> float:
    d = (deg + 180.0) % 360.0 - 180.0
    return d


def waypoint_frame(m: Mission, index: int, count: int) -> Command:
    wp = m.waypoints[index]
    heading = int(m.heading if wp.heading is None else wp.heading)
    speed = m.speed_ms if wp.speed_ms is None else wp.speed_ms
    p = bytearray(58)
    p[0], p[1] = 3, 36
    p[4], p[5] = index, count
    struct.pack_into("<dd", p, 8, wp.lon, wp.lat)
    # The app truncates the yaw to whole degrees before scaling by 100.
    struct.pack_into(
        "<hhh",
        p,
        24,
        int(round(wp.alt_m * 10)),
        int(_wrap_yaw(wp.yaw_deg)) * 100,
        int(round(wp.gimbal_pitch_deg * 100)),
    )
    p[30] = int(round(speed * 10)) & 0xFF
    p[34] = (
        (int(m.auto_record) << 4)
        | (int(m.coordinated_turn_off) << 2)
        | int(wp.poi is not None)
        | 0x02
    )
    p[35] = (heading & 0x0F) | ((int(wp.rotation) & 0x0F) << 4)
    p[36] = 1 if heading == Heading.WAYPOINT else 0
    p[38] = int(m.finish) & 0xFF
    p[39] = int(m.rc_lost) & 0xFF
    if wp.poi is not None:
        plat, plon, palt = wp.poi
        struct.pack_into("<ddh", p, 40, plon, plat, int(round(palt * 10)))
    return Command(Module.FC, bytes(p), name=f"mission_point[{index}]")


def action_frame(a: PointAction, index: int, count: int) -> Command:
    p = bytearray(56)
    p[0], p[1] = 3, 37
    p[4], p[5] = index, count
    p[8], p[9] = int(a.first), int(a.second)
    p[24], p[25], p[26] = a.p1 & 0xFF, a.p2 & 0xFF, a.p3 & 0xFF
    return Command(Module.FC, bytes(p), name=f"mission_action[{index}]")


start = commands.mission_start
stop = commands.mission_stop


# ---------------------------------------------------------------------------
# JSON-friendly form
# ---------------------------------------------------------------------------

_ACTION_PRESETS = {
    "none": PointAction.none,
    "hover": PointAction.hover,
    "record": PointAction.record,
    "photo": PointAction.photo,
    "hover_photo": PointAction.hover_then_photo,
}


def _action_from(obj) -> PointAction:
    if obj is None:
        return PointAction()
    if isinstance(obj, str):
        return _ACTION_PRESETS[obj]()
    if "preset" in obj:
        args = {k: v for k, v in obj.items() if k != "preset"}
        return _ACTION_PRESETS[obj["preset"]](**args)
    return PointAction(
        Action[obj.get("first", "NONE").upper()],
        Action[obj.get("second", "NONE").upper()],
        obj.get("p1", 0),
        obj.get("p2", 0),
        obj.get("p3", 0),
    )


def _enum(cls, value):
    if value is None or isinstance(value, cls):
        return value
    if isinstance(value, str):
        return cls[value.upper()]
    try:
        return cls(value)
    except ValueError:
        return int(value)  # raw, unnamed firmware value


def mission_from_dict(d: dict) -> Mission:
    """Build a Mission from plain data (e.g. JSON).

    ``{"speed_ms": 5, "heading": "free", "finish": "hover", "rc_lost": "exit",
       "waypoints": [{"lat": .., "lon": .., "alt_m": 30, "action": "photo",
                      "poi": [lat, lon, alt_m]}]}``
    """
    wps = []
    for w in d["waypoints"]:
        wps.append(
            Waypoint(
                lat=w["lat"],
                lon=w["lon"],
                alt_m=w["alt_m"],
                action=_action_from(w.get("action")),
                poi=tuple(w["poi"]) if w.get("poi") else None,
                yaw_deg=w.get("yaw_deg", 0.0),
                gimbal_pitch_deg=w.get("gimbal_pitch_deg", 0.0),
                rotation=_enum(Rotation, w.get("rotation", "min_angle")),
                speed_ms=w.get("speed_ms"),
                heading=_enum(Heading, w.get("heading")),
            )
        )
    return Mission(
        wps,
        speed_ms=d.get("speed_ms", 5.0),
        heading=_enum(Heading, d.get("heading", "free")),
        finish=_enum(FinishAction, d.get("finish", "hover")),
        rc_lost=_enum(LostAction, d.get("rc_lost", "exit")),
        auto_record=d.get("auto_record", False),
    )


# ---------------------------------------------------------------------------
# Routes saved by the FIMI app (its SQLite database, as pulled from the phone)
# ---------------------------------------------------------------------------

#: POINT_ACTION_CMD values the app's route editor writes, and what it uploads.
FIMI_DB_ACTIONS = {
    0: PointAction.none,
    1: lambda: PointAction.hover(10),
    2: lambda: PointAction.record(10),
    3: PointAction.none,  # "4x slow motion": the app uploads nothing for it
    4: lambda: PointAction.photo(1),
    5: lambda: PointAction.hover_then_photo(5, 1),
    6: lambda: PointAction.photo(3),
}


def mission_from_fimi_db(path: str, route: str | int) -> Mission:
    """Load a route from the FIMI app database by name or ``_id``.

    Mirrors what the app uploads for a saved route: route TYPE is the heading
    mode, EXCUTE_END the finish action, DISCONNECT_TYPE the RC-lost action,
    POINT_ACTION_CMD the waypoint action.  Speed comes from the waypoint SPEED
    column (decimetres/s) rather than the integer route speed.
    """
    import sqlite3

    db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    try:
        col = "_id" if isinstance(route, int) else "NAME"
        head = db.execute(
            f"SELECT _id, TYPE, SPEED, DISCONNECT_TYPE, EXCUTE_END FROM X8_AI_LINE_POINT_INFO "
            f"WHERE {col} = ? ORDER BY TIME DESC LIMIT 1",
            (route,),
        ).fetchone()
        if head is None:
            raise KeyError(f"route {route!r} not found in {path}")
        line_id, rtype, rspeed, disconnect, end = head
        rows = db.execute(
            "SELECT LATITUDE, LONGITUDE, ALTITUDE, SPEED, POINT_ACTION_CMD, RORATION, "
            "LATITUDE_POI, LONGITUDE_POI, ALTITUDE_POI, GIMBAL_PITCH "
            "FROM X8_AI_LINE_POINT_LATLNG_INFO WHERE LINE_ID = ? ORDER BY NUMBER",
            (line_id,),
        ).fetchall()
    finally:
        db.close()
    wps = []
    for lat, lon, alt, spd, act, rot, plat, plon, palt, gp in rows:
        poi = (plat, plon, float(palt)) if (plat or plon) else None
        wps.append(
            Waypoint(
                lat=lat,
                lon=lon,
                alt_m=float(alt),
                action=FIMI_DB_ACTIONS.get(act, PointAction.none)(),
                poi=poi,
                gimbal_pitch_deg=gp / 100,
                rotation=_enum(Rotation, rot),
                speed_ms=spd / 10 if spd else None,
            )
        )
    return Mission(
        wps,
        speed_ms=float(rspeed),
        heading=_enum(Heading, rtype),
        finish=_enum(FinishAction, end),
        rc_lost=_enum(LostAction, disconnect),
    )
