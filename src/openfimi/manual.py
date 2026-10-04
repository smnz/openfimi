"""Keyboard / joystick-style flying through the RC link, with one-point routes.

Virtual-stick frames (11/2) do nothing when the aircraft is flown through its
remote controller: the FIMI app only sends them over the direct Wi-Fi link, and
a flight test confirmed the aircraft ignores them over the RC link (318 frames,
no movement).  :class:`ManualFlight` therefore flies every move as a one-waypoint
route, which does work over the RC link and, unlike fly-to, can be replaced in
flight (flight test 2026-10-04):

* a fly-to (3/52 + 3/48) cannot be re-targeted while it is flying (code 21),
  and a route cannot be uploaded during one (code 41);
* "stop route (3/35), upload point + action (3/36, 3/37), start (3/32)" was
  accepted repeatedly in mid-flight, and with heading mode Free and a POI the
  nose turned to face the POI (seen in the video).

Each move is a waypoint some metres ahead in the direction of the held keys
(relative to the *desired heading*), at the requested altitude, with a POI
500 m away along the desired heading, so the nose holds the desired heading the
whole time and yaw keys turn it.  A new route is sent when the input changes, or
while it is held and the aircraft gets near the waypoint.  Releasing every
input, or none arriving for ``deadman`` seconds, stops the route and the
aircraft hovers.

Flight-verified (three flights, 2026-10-04): turns in place in both directions
to within 1 deg, straight moves along the turned heading, straight climbs and
descents that stop on release, mid-move direction changes.  Each route starts
after about 1 s and accelerates at about 0.5 m/s^2, so it suits deliberate
positioning (a 3 s press moves 1-2 m, 6 s about 10 m) rather than stick-like
flying.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Callable

from . import commands
from .mission import FinishAction, Heading, LostAction, Mission, Waypoint

log = logging.getLogger(__name__)

M_PER_DEG = 111_320.0
ROUTE_TASK_MODE, FLY_TO_TASK_MODE = 1, 2


def offset(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    return (lat + north_m / M_PER_DEG, lon + east_m / (M_PER_DEG * math.cos(math.radians(lat))))


def angle_diff(a: float, b: float) -> float:
    """Signed smallest difference a - b in degrees (-180..180)."""
    return (a - b + 180.0) % 360.0 - 180.0


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    dn = (b[0] - a[0]) * M_PER_DEG
    de = (b[1] - a[1]) * M_PER_DEG * math.cos(math.radians(a[0]))
    return math.hypot(dn, de)


class ManualFlight:
    def __init__(
        self,
        drone,
        *,
        max_speed_ms: float = 5.0,
        max_climb_ms: float = 2.0,
        yaw_rate_dps: float = 30.0,
        lookahead_s: float = 5.0,
        min_carrot_m: float = 8.0,
        min_alt_m: float = 3.0,
        max_alt_m: float = 120.0,
        poi_distance_m: float = 500.0,
        refresh_fraction: float = 0.4,
        deadman: float = 0.6,
        on_event: Callable[[str], None] | None = None,
    ) -> None:
        self.drone = drone
        self.max_speed_ms = max_speed_ms
        self.max_climb_ms = max_climb_ms
        self.yaw_rate_dps = yaw_rate_dps
        self.lookahead_s = lookahead_s
        self.min_carrot_m = min_carrot_m
        self.min_alt_m = min_alt_m
        self.max_alt_m = max_alt_m
        self.poi_distance_m = poi_distance_m
        self.refresh_fraction = refresh_fraction
        self.deadman = deadman
        self.say = on_event or (lambda msg: log.info(msg))
        self._input = (0.0, 0.0, 0.0, 0.0)  # forward, right, up, yaw in -1..1
        self._updated = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.heading: float | None = None  # desired heading, degrees
        self.moving = False
        self._sent: tuple[float, float, float] | None = None
        self._sent_heading: float | None = None
        self._target: tuple[float, float] | None = None
        self._leg_m = 0.0
        self._climbing = False  # last route changed altitude
        self._started_at = 0.0

    def set(
        self, roll: float = 0.0, pitch: float = 0.0, throttle: float = 0.0, yaw: float = 0.0
    ) -> None:
        """Same convention as the stick streamer: +roll right, +pitch forward,
        +throttle up, +yaw clockwise; each -1..1."""
        c = lambda v: max(-1.0, min(1.0, float(v or 0.0)))  # noqa: E731
        with self._lock:
            self._input = (c(pitch), c(roll), c(throttle), c(yaw))
            self._updated = time.monotonic()

    def centre(self) -> None:
        self.set()

    @property
    def frame_yaw(self) -> float | None:
        """The heading that 'forward' refers to (the desired heading)."""
        return self.heading

    def start(self) -> ManualFlight:
        sp = self.drone.state.sport
        self.heading = sp.yaw_deg if sp else 0.0
        self._sent_heading = self.heading
        self._started_at = time.monotonic()
        self.say(f"manual flight: heading {self.heading:.0f} deg")
        if self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="openfimi-manual", daemon=True)
            self._thread.start()
        return self

    def stop(self, halt: bool = True) -> None:
        """Stop manual flight.  ``halt`` ends the move in progress (hover); pass
        ``halt=False`` when another command (land, return home) is about to take
        over, so nothing is sent that could interfere with it."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)
            self._thread = None
        if self.moving and halt:
            self._halt()
        self.moving = False

    def __enter__(self) -> ManualFlight:
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    def _emergency(self) -> bool:
        st = self.drone.state
        notice = getattr(st, "notice", None)
        at = getattr(st, "updated", {}).get("notice", 0.0)
        return bool(notice and notice.get("event") == "emergency_rth" and at >= self._started_at)

    def _end_current(self) -> None:
        """End whatever autopilot task is running, so a new route is accepted."""
        nav = self.drone.state.navigation
        mode = nav.task_mode if nav else None
        if mode == ROUTE_TASK_MODE:
            self.drone.send(commands.mission_stop(), timeout=3)
        elif mode == FLY_TO_TASK_MODE:
            # Also after arrival: the aircraft refused route uploads (41) for
            # ~1.6 s after a fly-to arrived until it was exited.
            self.drone.send(commands.fly_to_exit(), timeout=3)

    def _halt(self) -> None:
        try:
            self._end_current()
        except Exception as e:  # noqa: BLE001 - keep going; the pilot has the RC
            self.say(f"stop failed: {e}")
        self.moving = False
        self._sent = None
        self._target = None
        self.say("manual: holding position")

    def _send_route(self, move: tuple[float, float, float]) -> bool:
        s = self.drone.state.sport
        fwd, right, up = move
        hn = math.hypot(fwd, right)
        h = min(1.0, hn)
        speed = max(1.0, h * self.max_speed_ms)
        yaw = math.radians(self.heading or 0.0)
        if hn > 0:
            dist = max(self.min_carrot_m, speed * self.lookahead_s)
            f, r = fwd / hn * dist, right / hn * dist
        else:
            dist, f, r = 0.0, 0.0, 0.0
        north = f * math.cos(yaw) - r * math.sin(yaw)
        east = f * math.sin(yaw) + r * math.cos(yaw)
        lat, lon = offset(s.lat, s.lon, north, east)
        alt = s.height_m + up * self.max_climb_ms * self.lookahead_s
        alt = max(self.min_alt_m, min(self.max_alt_m, alt))
        plat, plon = offset(
            s.lat, s.lon, self.poi_distance_m * math.cos(yaw), self.poi_distance_m * math.sin(yaw)
        )
        m = Mission(
            [Waypoint(lat, lon, alt, poi=(plat, plon, alt), speed_ms=speed)],
            speed_ms=speed,
            heading=Heading.FREE,
            finish=FinishAction.HOVER,
            rc_lost=LostAction.CONTINUE,
        )
        self._end_current()
        try:
            self.drone.upload_mission(m, check=True)
        except Exception as e:  # noqa: BLE001 - e.g. CommandRejected
            self.say(f"manual: route refused ({e})")
            return False
        r = self.drone.start_mission(timeout=3)
        if r is not None and not r.ok:
            self.say(f"manual: route start refused (code {r.code})")
            return False
        self._target = (lat, lon)
        self._leg_m = dist
        self._climbing = abs(up) >= 0.05
        return True

    def _run(self) -> None:
        last = time.monotonic()
        while not self._stop.wait(0.2):
            now = time.monotonic()
            dt, last = now - last, now
            s = self.drone.state.sport
            if s is None:
                continue
            if self._emergency():
                # The bridge's emergency return home (an operator override):
                # stand down without sending anything that could cancel it.
                self.say("manual: emergency return home from the bridge; stopping")
                self.moving = False
                return
            with self._lock:
                inp = self._input
                if now - self._updated > self.deadman:
                    inp = (0.0, 0.0, 0.0, 0.0)
            fwd, right, up, yaw_in = inp
            if abs(yaw_in) >= 0.05:
                self.heading = angle_diff(
                    (self.heading or 0.0) + yaw_in * self.yaw_rate_dps * dt, 0
                )
            move = (fwd, right, up)
            moving_keys = any(abs(v) >= 0.05 for v in move)
            turned = abs(angle_diff(self.heading, self._sent_heading or 0.0)) > 5.0
            try:
                if not moving_keys and abs(yaw_in) < 0.05 and not turned:
                    # Released: stop any travel.  A finished pure turn needs nothing.
                    if self.moving and (self._leg_m > 0 or self._climbing):
                        self._halt()
                    else:
                        self.moving = False
                    continue
                changed = self._sent is None or any(
                    abs(a - b) > 0.2 for a, b in zip(move, self._sent, strict=True)
                )
                near = (
                    self._target is not None
                    and self._leg_m > 0
                    and moving_keys
                    and _dist((s.lat, s.lon), self._target) < self.refresh_fraction * self._leg_m
                )
                if not (changed or turned or near):
                    continue
                if self._send_route(move):
                    if not self.moving:
                        self.say("manual: moving" if moving_keys else "manual: turning")
                    self.moving = True
                    self._sent = move
                    self._sent_heading = self.heading
            except Exception as e:  # noqa: BLE001 - e.g. ack timeout; try again next tick
                self.say(f"manual: {e}")
