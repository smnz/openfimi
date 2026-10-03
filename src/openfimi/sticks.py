"""Streaming virtual sticks with a dead-man timeout.

The app streams the virtual-stick frame every 200 ms.  :class:`StickStreamer`
does the same from a background thread, taking normalised inputs in -1..1.
If the caller stops updating for ``deadman`` seconds the sticks re-centre, so a
crashed controller (or a stalled AI loop) leaves the aircraft hovering rather
than holding the last command.

Inputs: +roll = right, +pitch = forward, +throttle = up, +yaw = clockwise.  The
mapping to wire values was measured from a real RC; whether the aircraft obeys
virtual sticks alongside the physical RC is not yet confirmed, so test with
the propellers removed before flying.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from . import commands
from .commands import STICK_CENTRE, STICK_MAX, STICK_MIN


@dataclass
class Sticks:
    roll: float = 0.0
    pitch: float = 0.0
    throttle: float = 0.0
    yaw: float = 0.0

    def raw(self) -> tuple[int, int, int, int]:
        """Wire values. Inputs are intuitive (+ = right, forward, up, clockwise);
        the RC encodes forward and up as LOW values, so those two are inverted."""

        def conv(v: float, invert: bool = False) -> int:
            v = max(-1.0, min(1.0, v))
            if invert:
                v = -v
            span = (STICK_MAX - STICK_CENTRE) if v > 0 else (STICK_CENTRE - STICK_MIN)
            return int(round(STICK_CENTRE + v * span))

        return (conv(self.roll), conv(self.pitch, True), conv(self.throttle, True), conv(self.yaw))


class StickStreamer:
    def __init__(self, link, period: float = 0.2, deadman: float = 0.6) -> None:
        self.link = link
        self.period = period
        self.deadman = deadman
        self._sticks = Sticks()
        self._updated = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread: threading.Thread | None = None

    def set(
        self, roll: float = 0.0, pitch: float = 0.0, throttle: float = 0.0, yaw: float = 0.0
    ) -> None:
        with self._lock:
            self._sticks = Sticks(roll, pitch, throttle, yaw)
            self._updated = time.monotonic()
        self._wake.set()  # send now rather than at the next tick

    def centre(self) -> None:
        self.set()

    def start(self) -> StickStreamer:
        if self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="openfimi-sticks", daemon=True)
            self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None
        # Leave the aircraft with centred sticks.
        self.link.send(commands.virtual_sticks(*Sticks().raw()))

    def _run(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                s = self._sticks
                if time.monotonic() - self._updated > self.deadman:
                    s = Sticks()
            try:
                self.link.send(commands.virtual_sticks(*s.raw()))
            except Exception:
                pass
            self._stop.wait(0.05)  # at most 20 Hz however often set() is called
            self._wake.wait(self.period)
            self._wake.clear()

    def __enter__(self) -> StickStreamer:
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()
