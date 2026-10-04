"""Per-waypoint gimbal pitch for routes, applied from the ground station.

The aircraft accepts a gimbal pitch in every waypoint frame but ignores it on
routes (flight-tested).  :class:`GimbalFollower` restores the feature: while a
route is flying it watches the navigation telemetry and position, and drives
the gimbal with realtime pitch commands (GIMBAL 9/6), which do work.  It needs
the ground-station link for the whole route, i.e. the aircraft within range of
the remote.

Timing, from flight telemetry: ``NavigationState.waypoint`` counts waypoints
*reached* and ticks on arrival, before that waypoint's action runs.  So the
follower keeps waypoint *k*'s pitch until the aircraft has actually left *k*
(more than ``depart_radius_m`` away), then moves to waypoint *k+1*'s pitch for
the leg, so the gimbal is already there when *k+1*'s photo is taken.

Modes:

* ``"lead"`` (default): keep the previous waypoint's pitch along the leg and
  switch to the next waypoint's pitch ``lead_s`` seconds (15) before the
  estimated arrival.  The estimate is the remaining distance divided by the
  planned leg speed, recomputed from the live position; because the aircraft
  slows on the approach the real lead time is a little longer, never shorter.
  Legs shorter than ``lead_s`` at planned speed fall back to ``"step"``.
* ``"step"``: switch to the next waypoint's pitch as soon as the aircraft has
  left the previous waypoint.
* ``"interpolate"``: blend from the previous waypoint's pitch to the next one
  along the leg, by distance (smooth for video).
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Callable

from . import commands
from .mission import Mission

log = logging.getLogger(__name__)

ROUTE_TASK_MODE = 1  # NavigationState.task_mode while a route is flying
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
        mode: str = "lead",
        lead_s: float = 15.0,
        depart_radius_m: float = 3.0,
        min_change_deg: float = 1.0,
        verify_after_s: float = 1.5,
        on_event: Callable[[str], None] | None = None,
    ) -> None:
        if mode not in ("lead", "step", "interpolate"):
            raise ValueError("mode must be 'lead', 'step' or 'interpolate'")
        self.lead_s = lead_s
        self.speeds = [w.speed_ms or mission.speed_ms for w in mission.waypoints]
        self.drone = drone
        self.mission = mission
        self.mode = mode
        self.depart_radius_m = depart_radius_m
        self.min_change_deg = min_change_deg
        self.verify_after_s = verify_after_s
        self.on_event = on_event or (lambda msg: log.info(msg))
        self.pitches = [
            max(PITCH_MIN, min(PITCH_MAX, w.gimbal_pitch_deg)) for w in mission.waypoints
        ]
        self.points = [(w.lat, w.lon) for w in mission.waypoints]
        self.commanded: float | None = None
        self._sent_at = 0.0
        self._retried = False
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.route_seen = False
        self.finished = threading.Event()

    # -- the rule, separated for testing --------------------------------------------
    def target_pitch(self, reached: int, pos: tuple[float, float]) -> float:
        """Pitch for the current state: ``reached`` waypoints done, at ``pos``."""
        n = len(self.points)
        if reached <= 0:
            return self.pitches[0]
        if reached >= n:
            return self.pitches[-1]
        prev, nxt = reached - 1, reached
        if ground_distance(pos, self.points[prev]) < self.depart_radius_m:
            return self.pitches[prev]  # still at (or acting at) the waypoint just reached
        leg = ground_distance(self.points[prev], self.points[nxt])
        speed = max(0.1, self.speeds[nxt])
        if self.mode == "step" or (self.mode == "lead" and leg / speed < self.lead_s):
            return self.pitches[nxt]
        if self.mode == "lead":
            eta = ground_distance(pos, self.points[nxt]) / speed
            return self.pitches[nxt] if eta <= self.lead_s else self.pitches[prev]
        if leg < 1e-3:
            return self.pitches[nxt]
        f = 1.0 - ground_distance(pos, self.points[nxt]) / leg
        f = max(0.0, min(1.0, f))
        return self.pitches[prev] + f * (self.pitches[nxt] - self.pitches[prev])

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

    def _command(self, pitch: float) -> None:
        self.drone.link.send(commands.gimbal_pitch(pitch))
        self.commanded, self._sent_at, self._retried = pitch, time.monotonic(), False

    def _run(self) -> None:
        while not self._stop.wait(0.2):
            s = self.drone.state
            nav, sport = s.navigation, s.sport
            if nav is None or sport is None:
                continue
            if nav.task_mode != ROUTE_TASK_MODE:
                if self.route_seen:
                    self.on_event("route finished; gimbal follower stopping")
                    self.finished.set()
                    return
                continue
            if not self.route_seen:
                self.route_seen = True
                self.on_event("route started; gimbal follower active")
            pitch = self.target_pitch(nav.waypoint, (sport.lat, sport.lon))
            if self.commanded is None or abs(pitch - self.commanded) >= self.min_change_deg:
                self.on_event(f"gimbal -> {pitch:.1f} deg (reached {nav.waypoint})")
                self._command(pitch)
                continue
            # Re-send once if the gimbal has not got there (lost command, wheel nudge).
            g = s.gimbal
            if (
                g is not None
                and not self._retried
                and time.monotonic() - self._sent_at > self.verify_after_s
                and abs(g.pitch_deg - self.commanded) > 2.0
            ):
                self.on_event(f"gimbal at {g.pitch_deg:.1f}, re-sending {self.commanded:.1f}")
                self.drone.link.send(commands.gimbal_pitch(self.commanded))
                self._retried = True
