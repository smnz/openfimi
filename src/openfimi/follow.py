"""Per-waypoint gimbal pitch for routes, applied from the ground station.

The aircraft accepts a gimbal pitch in every waypoint frame but ignores it on
routes (flight-tested).  :class:`GimbalFollower` restores the feature: while a
route is flying it watches the navigation telemetry and position, and drives
the gimbal with realtime pitch commands (GIMBAL 9/6), which do work.  It needs
the ground-station link for the whole route, i.e. the aircraft within range of
the remote.

Each waypoint has a :class:`~openfimi.mission.GimbalMode`:

* ``NONE`` (0, the default): never touch the gimbal for this waypoint; it
  stays wherever the pilot or an earlier waypoint left it.
* ``BEFORE_ARRIVAL`` (1): be at the waypoint's pitch ``lead_s`` seconds (15)
  before arriving, for photos.  The estimate is the remaining distance divided
  by the planned leg speed, recomputed from the live position; because the
  aircraft slows on the approach the real lead is a little longer, never
  shorter.  On legs shorter than ``lead_s`` the pitch is set as soon as the
  aircraft has left the previous waypoint (> ``depart_radius_m``), so a hover
  or photo at the previous waypoint keeps its own pitch.
* ``ON_ARRIVAL`` (2): set the pitch when the waypoint is reached, e.g. for
  video between waypoints.  Not for photo waypoints: in flight the photo
  action fired about 0.7 s *before* the reached counter ticked, so the photo
  is always taken at the previous pitch.

``NavigationState.waypoint`` counts waypoints *reached* and ticks on arrival,
before that waypoint's action runs (flight telemetry).
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Callable

from . import commands
from .mission import GimbalMode, Mission

log = logging.getLogger(__name__)

ROUTE_TASK_MODE = 1  # NavigationState.task_mode while a route is flying
NO_WAYPOINT = 0xFFFF  # NavigationState.waypoint just after a route starts
PITCH_MIN, PITCH_MAX = -90.0, 10.0
_R = 6372800.0


def ground_distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Haversine distance in metres between (lat, lon) pairs."""
    p1, p2 = math.radians(a[0]), math.radians(b[0])
    dl = math.radians(b[1] - a[1])
    h = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * _R * math.asin(math.sqrt(h))


class GimbalFollower:
    def __init__(
        self,
        drone,
        mission: Mission,
        *,
        lead_s: float = 15.0,
        depart_radius_m: float = 3.0,
        verify_after_s: float = 1.5,
        on_event: Callable[[str], None] | None = None,
    ) -> None:
        self.drone = drone
        self.mission = mission
        self.lead_s = lead_s
        self.depart_radius_m = depart_radius_m
        self.verify_after_s = verify_after_s
        self.on_event = on_event or (lambda msg: log.info(msg))
        wps = mission.waypoints
        self.modes = [GimbalMode(w.gimbal_mode) for w in wps]
        self.pitches = [max(PITCH_MIN, min(PITCH_MAX, w.gimbal_pitch_deg)) for w in wps]
        self.points = [(w.lat, w.lon) for w in wps]
        self.speeds = [w.speed_ms or mission.speed_ms for w in wps]
        self.start_pos: tuple[float, float] | None = None
        self.done: set[int] = set()
        self.commanded: float | None = None
        self._sent_at = 0.0
        self._retried = False
        self._last_reached = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.route_seen = False
        self.finished = threading.Event()

    @property
    def active(self) -> bool:
        return any(m != GimbalMode.NONE for m in self.modes)

    def plan(self) -> list[str]:
        """Human-readable summary of what will happen at each waypoint."""
        out = []
        for k, (mode, pitch) in enumerate(zip(self.modes, self.pitches, strict=True)):
            if mode == GimbalMode.NONE:
                continue
            if mode == GimbalMode.ON_ARRIVAL:
                out.append(f"wp{k}: {pitch:g} deg on arrival")
                continue
            if k == 0:
                out.append(f"wp{k}: {pitch:g} deg at route start")
                continue
            secs = ground_distance(self.points[k - 1], self.points[k]) / max(0.1, self.speeds[k])
            when = (
                f"{self.lead_s:g} s before arrival (leg ~{secs:.0f} s)"
                if secs >= self.lead_s
                else f"on leaving wp{k - 1} (leg ~{secs:.0f} s, shorter than {self.lead_s:g} s)"
            )
            out.append(f"wp{k}: {pitch:g} deg {when}")
        return out

    # -- the rule, separated for testing --------------------------------------------
    def decide(self, reached: int, pos: tuple[float, float]) -> int | None:
        """Waypoint whose pitch should be commanded now, or None.

        ``reached`` = waypoints reached so far (NavigationState.waypoint).
        Marks the returned waypoint as done.
        """
        n = len(self.points)
        if reached < 0 or reached > n + 1:
            return None  # 65535 = "no count yet", seen at route start in flight
        choice = None
        # Waypoints reached since the last call: ON_ARRIVAL fires now, and a
        # BEFORE_ARRIVAL that never fired (route joined late) fires as a fallback.
        for k in range(self._last_reached, min(reached, n)):
            if self.modes[k] != GimbalMode.NONE and k not in self.done:
                choice = k
            self.done.add(k)
        self._last_reached = max(self._last_reached, reached)
        if choice is not None:
            return choice
        k = reached  # the waypoint being flown toward
        if k >= n or k in self.done or self.modes[k] != GimbalMode.BEFORE_ARRIVAL:
            return None
        prev = self.points[k - 1] if k > 0 else (self.start_pos or pos)
        speed = max(0.1, self.speeds[k])
        leg = ground_distance(prev, self.points[k])
        if k == 0 or leg / speed < self.lead_s:
            fire = k == 0 or ground_distance(pos, prev) >= self.depart_radius_m
        else:
            fire = ground_distance(pos, self.points[k]) / speed <= self.lead_s
        if fire:
            self.done.add(k)
            return k
        return None

    # -- running ----------------------------------------------------------------------
    def start(self) -> GimbalFollower:
        self._thread = threading.Thread(target=self._run, name="openfimi-gimbal", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)

    def __enter__(self) -> GimbalFollower:
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    def _command(self, k: int) -> None:
        pitch = self.pitches[k]
        how = "before arrival" if self.modes[k] == GimbalMode.BEFORE_ARRIVAL else "on arrival"
        self.on_event(f"gimbal -> {pitch:g} deg for waypoint {k} ({how})")
        self.drone.link.send(commands.gimbal_pitch(pitch))
        self.commanded, self._sent_at, self._retried = pitch, time.monotonic(), False

    def _run(self) -> None:
        while not self._stop.wait(0.2):
            s = self.drone.state
            nav, sport = s.navigation, s.sport
            if nav is None or sport is None:
                continue
            pos = (sport.lat, sport.lon)
            if nav.task_mode != ROUTE_TASK_MODE:
                if self.route_seen:
                    k = self.decide(len(self.points), pos)  # a final ON_ARRIVAL
                    if k is not None:
                        self._command(k)
                    self.on_event("route finished; gimbal follower stopping")
                    self.finished.set()
                    return
                continue
            if not self.route_seen:
                self.route_seen = True
                self.start_pos = pos
                self.on_event("route started; gimbal follower active")
            if nav.waypoint == NO_WAYPOINT:
                continue  # the aircraft reports 65535 for a moment as the route starts
            k = self.decide(nav.waypoint, pos)
            if k is not None:
                self._command(k)
                continue
            # Re-send once if the gimbal has not got there (lost command).
            g = s.gimbal
            if (
                self.commanded is not None
                and g is not None
                and not self._retried
                and self.verify_after_s
                < time.monotonic() - self._sent_at
                < self.verify_after_s + 2.0  # never fight a later change (wheel, pilot)
                and abs(g.pitch_deg - self.commanded) > 2.0
            ):
                self.on_event(f"gimbal at {g.pitch_deg:.1f}, re-sending {self.commanded:g}")
                self.drone.link.send(commands.gimbal_pitch(self.commanded))
                self._retried = True
