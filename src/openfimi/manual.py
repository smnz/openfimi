"""Keyboard / joystick-style flying through the RC link, using fly-to targets.

Virtual-stick frames (11/2) do nothing when the aircraft is flown through its
remote controller: the FIMI app only sends them over the direct Wi-Fi link, and
a flight test confirmed the aircraft ignores them over the RC link (318 frames,
no movement).  Fly-to (3/52 target, then 3/48 go) does work over the RC link,
so :class:`ManualFlight` turns held stick inputs into a moving fly-to target
("carrot") ahead of the aircraft:

* forward/back and left/right move in a frame locked to the aircraft's heading
  when manual flight starts (fly-to turns the nose toward the travel direction,
  so a frame that followed the nose would spiral while strafing);
* up/down moves the target altitude;
* yaw is not available this way (no yaw command over the RC link is known);
* when every input is released, or none arrives for ``deadman`` seconds, the
  fly-to is cancelled (3/51) and the aircraft hovers.

Movement is an autopilot leg, so it starts and stops gently (about 0.5 m/s^2 in
flight tests) rather than responding like sticks.  Re-targeting a fly-to while
one is in progress, very short targets, and vertical-only targets are not yet
flight-tested.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Callable

from . import commands

log = logging.getLogger(__name__)

M_PER_DEG = 111_320.0


def offset(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    return (lat + north_m / M_PER_DEG, lon + east_m / (M_PER_DEG * math.cos(math.radians(lat))))


class ManualFlight:
    def __init__(
        self,
        drone,
        *,
        max_speed_ms: float = 5.0,
        max_climb_ms: float = 2.0,
        lookahead_s: float = 3.0,
        min_carrot_m: float = 5.0,
        min_alt_m: float = 3.0,
        max_alt_m: float = 120.0,
        retarget_s: float = 1.0,
        deadman: float = 0.6,
        on_event: Callable[[str], None] | None = None,
    ) -> None:
        self.drone = drone
        self.max_speed_ms = max_speed_ms
        self.max_climb_ms = max_climb_ms
        self.lookahead_s = lookahead_s
        self.min_carrot_m = min_carrot_m
        self.min_alt_m = min_alt_m
        self.max_alt_m = max_alt_m
        self.retarget_s = retarget_s
        self.deadman = deadman
        self.say = on_event or (lambda msg: log.info(msg))
        self._input = (0.0, 0.0, 0.0)  # forward, right, up in -1..1
        self._updated = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.frame_yaw: float | None = None
        self.moving = False
        self._sent_input: tuple[float, float, float] | None = None
        self._sent_at = 0.0
        self._yaw_warned = False

    # -- input --------------------------------------------------------------------
    def set(
        self, roll: float = 0.0, pitch: float = 0.0, throttle: float = 0.0, yaw: float = 0.0
    ) -> None:
        """Same convention as the stick streamer: +roll right, +pitch forward,
        +throttle up, +yaw clockwise; each -1..1.  Yaw is ignored (see module doc)."""
        if yaw and not self._yaw_warned:
            self._yaw_warned = True
            self.say("yaw is not available over the RC link; ignoring it")
        c = lambda v: max(-1.0, min(1.0, float(v or 0.0)))  # noqa: E731
        with self._lock:
            self._input = (c(pitch), c(roll), c(throttle))
            self._updated = time.monotonic()

    def centre(self) -> None:
        self.set()

    # -- lifecycle ------------------------------------------------------------------
    def start(self) -> ManualFlight:
        sp = self.drone.state.sport
        self.frame_yaw = sp.yaw_deg if sp else 0.0
        self.say(f"manual flight: forward = {self.frame_yaw:.0f} deg")
        if self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="openfimi-manual", daemon=True)
            self._thread.start()
        return self

    def stop(self, halt: bool = True) -> None:
        """Stop manual flight.  ``halt`` cancels a fly-to in progress (hover);
        pass ``halt=False`` when another command (land, return home) is about to
        take over, so nothing is sent that could interfere with it."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3)
            self._thread = None
        if self.moving and halt:
            self._halt()
        self.moving = False

    def __enter__(self) -> ManualFlight:
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    # -- control loop ------------------------------------------------------------------
    def _halt(self) -> None:
        try:
            self.drone.send(commands.fly_to_exit(), timeout=3)
        except Exception as e:  # noqa: BLE001 - keep going; the pilot has the RC
            self.say(f"stop failed: {e}")
        self.moving = False
        self._sent_input = None
        self.say("manual: holding position")

    def _target(self, inp: tuple[float, float, float]):
        s = self.drone.state.sport
        fwd, right, up = inp
        h = math.hypot(fwd, right)
        speed = max(1.0, min(1.0, h) * self.max_speed_ms) if h > 0 else 1.0
        if h > 0:
            dist = max(self.min_carrot_m, speed * self.lookahead_s)
            yaw = math.radians(self.frame_yaw or 0.0)
            f, r = fwd / h * dist, right / h * dist
            north = f * math.cos(yaw) - r * math.sin(yaw)
            east = f * math.sin(yaw) + r * math.cos(yaw)
            lat, lon = offset(s.lat, s.lon, north, east)
        else:
            lat, lon = s.lat, s.lon
        alt = s.height_m + up * self.max_climb_ms * self.lookahead_s
        alt = max(self.min_alt_m, min(self.max_alt_m, alt))
        return lat, lon, alt, speed

    def _go(self, inp) -> None:
        lat, lon, alt, speed = self._target(inp)
        r = self.drone.send(commands.fly_to(lat, lon, alt, speed), timeout=3)
        if r is not None and not r.ok:
            self.say(f"fly-to target refused (code {r.code})")
            return
        r = self.drone.send(commands.fly_to_start(), timeout=3)
        if r is not None and not r.ok:
            self.say(f"fly-to start refused (code {r.code})")
            return
        if not self.moving:
            self.say("manual: moving")
        self.moving = True
        self._sent_input = inp
        self._sent_at = time.monotonic()

    def _run(self) -> None:
        while not self._stop.wait(0.2):
            if self.drone.state.sport is None:
                continue
            with self._lock:
                inp = self._input
                if time.monotonic() - self._updated > self.deadman:
                    inp = (0.0, 0.0, 0.0)
            idle = all(abs(v) < 0.05 for v in inp)
            try:
                if idle:
                    if self.moving:
                        self._halt()
                    continue
                changed = self._sent_input is None or any(
                    abs(a - b) > 0.2 for a, b in zip(inp, self._sent_input, strict=True)
                )
                if changed or time.monotonic() - self._sent_at > self.retarget_s:
                    self._go(inp)
            except Exception as e:  # noqa: BLE001 - e.g. ack timeout; try again next tick
                self.say(f"manual: {e}")
