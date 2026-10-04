"""Keyboard / joystick-style flying through the RC link, with autopilot commands.

Virtual-stick frames (11/2) do nothing when the aircraft is flown through its
remote controller: the FIMI app only sends them over the direct Wi-Fi link, and
a flight test confirmed the aircraft ignores them over the RC link (318 frames,
no movement).  :class:`ManualFlight` therefore turns held stick inputs into
autopilot targets, which do work over the RC link:

* **Moving** (forward/back/left/right, up/down): a fly-to target (3/52 then
  3/48) a few metres ahead, re-targeted while the input is held.  Directions
  are relative to the *desired heading* below, not the nose, because fly-to
  turns the nose toward its travel direction.
* **Turning** (yaw): yaw input turns a desired heading.  While the aircraft's
  heading differs from it, moves are sent instead as a one-waypoint route with
  heading mode Free and a POI 500 m away along the desired heading; the
  aircraft yaws to face a POI (flight-tested on routes, with the bearing taken
  from its live position).  The route is stopped (3/35), uploaded and started
  (3/32) each time.
* Releasing every input, or none arriving for ``deadman`` seconds, cancels the
  move and the aircraft hovers.

Moves are autopilot legs, so they start and stop gently (about 0.5 m/s^2 in
flight tests).  Not yet flight-tested: re-targeting a fly-to in progress, very
short or vertical-only targets, and whether a one-point route *at the current
position* turns the nose (on routes the POI yaw happened during the leg toward
the waypoint).  If it does not, set ``yaw_creep_m`` to a few metres so each turn
also moves the aircraft slightly along the new heading.
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
ROUTE_TASK_MODE = 1


def offset(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    return (lat + north_m / M_PER_DEG, lon + east_m / (M_PER_DEG * math.cos(math.radians(lat))))


def angle_diff(a: float, b: float) -> float:
    """Signed smallest difference a - b in degrees (-180..180)."""
    return (a - b + 180.0) % 360.0 - 180.0


class ManualFlight:
    def __init__(
        self,
        drone,
        *,
        max_speed_ms: float = 5.0,
        max_climb_ms: float = 2.0,
        yaw_rate_dps: float = 30.0,
        lookahead_s: float = 3.0,
        min_carrot_m: float = 5.0,
        min_alt_m: float = 3.0,
        max_alt_m: float = 120.0,
        poi_distance_m: float = 500.0,
        yaw_creep_m: float = 0.0,
        yaw_tolerance_deg: float = 5.0,
        retarget_s: float = 1.0,
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
        self.yaw_creep_m = yaw_creep_m
        self.yaw_tolerance_deg = yaw_tolerance_deg
        self.retarget_s = retarget_s
        self.deadman = deadman
        self.say = on_event or (lambda msg: log.info(msg))
        self._input = (0.0, 0.0, 0.0, 0.0)  # forward, right, up, yaw in -1..1
        self._updated = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.heading: float | None = None  # desired heading, degrees
        self.mode: str | None = None  # "fly_to" or "route" while moving
        self.moving = False
        self._translating = False
        self._turn_anchor: tuple[float, float] | None = None  # where a pure turn began
        self._sent_input: tuple[float, float, float] | None = None
        self._sent_heading: float | None = None
        self._sent_at = 0.0

    # -- input --------------------------------------------------------------------
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

    # -- lifecycle ------------------------------------------------------------------
    def start(self) -> ManualFlight:
        sp = self.drone.state.sport
        self.heading = sp.yaw_deg if sp else 0.0
        self.say(f"manual flight: forward = {self.heading:.0f} deg")
        if self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="openfimi-manual", daemon=True)
            self._thread.start()
        return self

    def stop(self, halt: bool = True) -> None:
        """Stop manual flight.  ``halt`` cancels a move in progress (hover); pass
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

    # -- helpers ----------------------------------------------------------------------
    def _halt(self) -> None:
        cmd = commands.mission_stop() if self.mode == "route" else commands.fly_to_exit()
        try:
            self.drone.send(cmd, timeout=3)
        except Exception as e:  # noqa: BLE001 - keep going; the pilot has the RC
            self.say(f"stop failed: {e}")
        self.moving = False
        self.mode = None
        self._sent_input = None
        self._turn_anchor = None
        self.say("manual: holding position")

    def _yaw_error(self) -> float:
        sp = self.drone.state.sport
        if sp is None or self.heading is None:
            return 0.0
        return angle_diff(self.heading, sp.yaw_deg)

    def _target(self, inp: tuple[float, float, float]):
        s = self.drone.state.sport
        fwd, right, up = inp
        h = math.hypot(fwd, right)
        speed = max(1.0, min(1.0, h) * self.max_speed_ms) if h > 0 else 1.0
        yaw = math.radians(self.heading or 0.0)
        if h > 0:
            dist = max(self.min_carrot_m, speed * self.lookahead_s)
            f, r = fwd / h * dist, right / h * dist
        else:
            f, r = 0.0, 0.0
        north = f * math.cos(yaw) - r * math.sin(yaw)
        east = f * math.sin(yaw) + r * math.cos(yaw)
        lat, lon = offset(s.lat, s.lon, north, east)
        alt = s.height_m + up * self.max_climb_ms * self.lookahead_s
        alt = max(self.min_alt_m, min(self.max_alt_m, alt))
        return lat, lon, alt, speed, h > 0

    def _check(self, reply, what: str) -> bool:
        if reply is not None and not reply.ok:
            self.say(f"{what} refused (code {reply.code})")
            return False
        return True

    def _go_fly_to(self, inp) -> bool:
        lat, lon, alt, speed, translating = self._target(inp)
        if self.mode == "route":
            self.drone.send(commands.mission_stop(), timeout=3)
        if not self._check(
            self.drone.send(commands.fly_to(lat, lon, alt, speed), timeout=3), "fly-to target"
        ):
            return False
        if not self._check(self.drone.send(commands.fly_to_start(), timeout=3), "fly-to start"):
            return False
        self.mode = "fly_to"
        self._translating = translating
        return True

    def _go_route(self, inp) -> bool:
        lat, lon, alt, speed, translating = self._target(inp)
        s = self.drone.state.sport
        yaw = math.radians(self.heading or 0.0)
        if not translating and self.yaw_creep_m > 0:
            # Creep from where the turn began, so re-sends don't add up.
            if self._turn_anchor is None:
                self._turn_anchor = (s.lat, s.lon)
            a_lat, a_lon = self._turn_anchor
            lat, lon = offset(
                a_lat, a_lon, self.yaw_creep_m * math.cos(yaw), self.yaw_creep_m * math.sin(yaw)
            )
            translating = True
        elif translating:
            self._turn_anchor = None
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
        nav = self.drone.state.navigation
        if nav is not None and nav.task_mode == ROUTE_TASK_MODE:
            self.drone.send(commands.mission_stop(), timeout=3)
        elif self.mode == "fly_to" and self.moving:
            self.drone.send(commands.fly_to_exit(), timeout=3)
        try:
            self.drone.upload_mission(m, check=True)
        except Exception as e:  # noqa: BLE001 - e.g. CommandRejected
            self.say(f"turn: route upload refused ({e})")
            return False
        if not self._check(self.drone.start_mission(timeout=3), "turn: route start"):
            return False
        self.mode = "route"
        self._translating = translating
        return True

    # -- control loop -----------------------------------------------------------------
    def _run(self) -> None:
        last = time.monotonic()
        while not self._stop.wait(0.2):
            now = time.monotonic()
            dt, last = now - last, now
            if self.drone.state.sport is None:
                continue
            with self._lock:
                inp = self._input
                if now - self._updated > self.deadman:
                    inp = (0.0, 0.0, 0.0, 0.0)
            fwd, right, up, yaw_in = inp
            if abs(yaw_in) >= 0.05:
                self.heading = (self.heading + yaw_in * self.yaw_rate_dps * dt) % 360.0
                if self.heading > 180.0:
                    self.heading -= 360.0
            move = (fwd, right, up)
            idle = all(abs(v) < 0.05 for v in inp)
            turning = abs(self._yaw_error()) > self.yaw_tolerance_deg
            try:
                if idle:
                    # A pure turn (route to the current position) finishes on its
                    # own; anything travelling is cancelled to hover.
                    if self.moving and (self._translating or self.mode == "fly_to"):
                        self._halt()
                    elif self.moving and not turning:
                        self.moving, self.mode = False, None
                    continue
                if all(abs(v) < 0.05 for v in move) and not turning:
                    continue  # yaw held but already facing the desired heading
                changed = (
                    self._sent_input is None
                    or any(abs(a - b) > 0.2 for a, b in zip(move, self._sent_input, strict=True))
                    or (
                        self._sent_heading is not None
                        and abs(angle_diff(self.heading, self._sent_heading)) > 10.0
                    )
                )
                if not (changed or now - self._sent_at > self.retarget_s):
                    continue
                ok = self._go_route(move) if turning else self._go_fly_to(move)
                if ok:
                    if not self.moving:
                        self.say("manual: moving" if not turning else "manual: turning")
                    self.moving = True
                    self._sent_input = move
                    self._sent_heading = self.heading
                    self._sent_at = now
            except Exception as e:  # noqa: BLE001 - e.g. ack timeout; try again next tick
                self.say(f"manual: {e}")
