"""High-level API: one object for telemetry, flight actions, missions, gimbal,
camera, sticks and video.

    from openfimi import Drone, transport

    with Drone(transport.TcpTransport("pizero.local")) as d:
        d.wait_for_telemetry()
        print(d.state.position, d.state.battery)
        d.takeoff()
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field

from . import commands, telemetry
from .commands import Command
from .link import AckTimeout, Link, Reply
from .mission import Mission
from .sticks import StickStreamer
from .transport.base import Transport
from .video import VideoPacket

log = logging.getLogger(__name__)


class CommandRejected(RuntimeError):
    def __init__(self, reply: Reply):
        super().__init__(f"{reply.command.name} rejected with code {reply.code}")
        self.reply = reply


@dataclass
class DroneState:
    """Latest decoded telemetry, updated from the receive thread."""

    heart: telemetry.FcHeart | None = None
    sport: telemetry.FcSportState | None = None
    signal: telemetry.FcSignalState | None = None
    errors: telemetry.FcErrCode | None = None
    battery: telemetry.FcBattery | None = None
    home: telemetry.HomeInfo | None = None
    navigation: telemetry.NavigationState | None = None
    gimbal: telemetry.GimbalState | None = None
    camera: telemetry.CameraState | None = None
    rc_sticks: telemetry.RcSticks | None = None
    rc_heart: telemetry.RcHeart | None = None
    updated: dict[str, float] = field(default_factory=dict)

    _FIELDS = {
        telemetry.FcHeart: "heart",
        telemetry.FcSportState: "sport",
        telemetry.FcSignalState: "signal",
        telemetry.FcErrCode: "errors",
        telemetry.FcBattery: "battery",
        telemetry.HomeInfo: "home",
        telemetry.NavigationState: "navigation",
        telemetry.GimbalState: "gimbal",
        telemetry.CameraState: "camera",
        telemetry.RcSticks: "rc_sticks",
        telemetry.RcHeart: "rc_heart",
    }

    def apply(self, msg: object) -> None:
        name = self._FIELDS.get(type(msg))
        if name:
            setattr(self, name, msg)
            self.updated[name] = time.monotonic()

    @property
    def position(self) -> tuple[float, float, float] | None:
        """(lat, lon, height above take-off in m)."""
        s = self.sport
        return (s.lat, s.lon, s.height_m) if s else None

    @property
    def flying(self) -> bool | None:
        return self.heart.flying if self.heart else None

    def age(self, name: str) -> float | None:
        t = self.updated.get(name)
        return None if t is None else time.monotonic() - t

    def summary(self) -> dict:
        out: dict = {}
        if self.sport:
            s = self.sport
            out.update(
                lat=round(s.lat, 7),
                lon=round(s.lon, 7),
                height_m=round(s.height_m, 1),
                yaw=s.yaw_deg,
                pitch=s.pitch_deg,
                roll=s.roll_deg,
                home_m=round(s.home_distance_m, 1),
            )
        if self.heart:
            out.update(phase=self.heart.flight_phase, flying=self.heart.flying)
        if self.battery:
            out.update(battery_pct=self.battery.percent, volts=self.battery.voltage)
        if self.signal:
            out.update(sats=self.signal.satellites, rc_signal=self.signal.rc_signal)
        if self.rc_heart:
            out.update(rc_battery_pct=self.rc_heart.percent)
        if self.navigation:
            n = self.navigation
            out.update(task=n.task_mode, nav=n.navi_task_state, ap=n.ap_status, wp=n.waypoint)
        return out


class Drone:
    def __init__(self, transport: Transport, *, init_camera: bool = True) -> None:
        self.link = Link(transport)
        self.state = DroneState()
        self.init_camera = init_camera
        self._telemetry_seen = threading.Event()
        self._sticks: StickStreamer | None = None
        self.link.on_message(self._on_message)

    # -- lifecycle -----------------------------------------------------------
    def connect(self) -> Drone:
        self.link.start()
        if self.init_camera:
            # What the app sends on connect: camera clock and the FPV stream config.
            for cmd in (commands.set_camera_clock(), commands.set_fpv_stream(True)):
                self.link.send(cmd)
        return self

    def close(self) -> None:
        if self._sticks is not None:
            self._sticks.stop()
        self.link.stop()

    def __enter__(self) -> Drone:
        return self.connect()

    def __exit__(self, *exc) -> None:
        self.close()

    def _on_message(self, msg: object, frame) -> None:
        self.state.apply(msg)
        if self.state.heart is not None and self.state.sport is not None:
            self._telemetry_seen.set()

    def wait_for_telemetry(self, timeout: float = 10.0) -> bool:
        """Block until FC telemetry (heartbeat and position) is flowing."""
        return self._telemetry_seen.wait(timeout)

    def wait_until(
        self, predicate: Callable[[DroneState], bool], timeout: float, poll: float = 0.1
    ) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                if predicate(self.state):
                    return True
            except (AttributeError, TypeError):
                pass  # a message the predicate reads has not arrived yet
            time.sleep(poll)
        return False

    # -- generic ---------------------------------------------------------------
    def send(
        self, cmd: Command, *, check: bool = False, timeout: float | None = None
    ) -> Reply | None:
        """Send a command and wait for its reply (None for no-ack commands).

        ``check=True`` raises :class:`CommandRejected` if the result code is
        non-zero.  Result-code meanings are not fully mapped yet.
        """
        if not cmd.ack:
            self.link.send(cmd)
            return None
        reply = self.link.request(cmd, timeout)
        if check and reply is not None and not reply.ok:
            raise CommandRejected(reply)
        return reply

    # -- flight ------------------------------------------------------------------
    def takeoff(self, **kw) -> Reply | None:
        return self.send(commands.takeoff(), **kw)

    def land(self, **kw) -> Reply | None:
        return self.send(commands.land(), **kw)

    def return_home(self, **kw) -> Reply | None:
        return self.send(commands.return_home(), **kw)

    def cancel_takeoff(self, **kw) -> Reply | None:
        return self.send(commands.cancel_takeoff(), **kw)

    def cancel_land(self, **kw) -> Reply | None:
        return self.send(commands.cancel_land(), **kw)

    def cancel_return_home(self, **kw) -> Reply | None:
        return self.send(commands.cancel_return_home(), **kw)

    def fly_to(self, lat: float, lon: float, alt_m: float, speed_ms: float = 5.0) -> Reply | None:
        """Point-to-point flight (the app's tap-to-fly): confirm, then GO."""
        self.send(commands.fly_to_confirm())
        return self.send(commands.fly_to(lat, lon, alt_m, speed_ms))

    # -- missions ------------------------------------------------------------------
    def upload_mission(
        self,
        mission: Mission,
        *,
        check: bool = True,
        progress: Callable[[int, int], None] | None = None,
    ) -> list[Reply]:
        """Upload a route frame by frame, waiting for each ack like the app.

        Unlike the app (which ignores a refused packet), a non-zero result code
        raises CommandRejected by default.
        """
        cmds = mission.upload_commands()
        replies = []
        for i, cmd in enumerate(cmds):
            r = self.send(cmd, check=check)
            replies.append(r)
            if progress:
                progress(i + 1, len(cmds))
        return replies

    def read_mission(self, count: int | None = None) -> list[telemetry.AiLinePoint]:
        """Read back the waypoints stored on the aircraft."""
        first = self.send(commands.mission_read_point(0))
        if first is None or not isinstance(first.message, telemetry.AiLinePoint):
            return []
        pts = [first.message]
        n = count if count is not None else first.message.count
        for i in range(1, n):
            r = self.send(commands.mission_read_point(i))
            if r is not None and isinstance(r.message, telemetry.AiLinePoint):
                pts.append(r.message)
        return pts

    def start_mission(self, **kw) -> Reply | None:
        return self.send(commands.mission_start(), **kw)

    def stop_mission(self, **kw) -> Reply | None:
        return self.send(commands.mission_stop(), **kw)

    # -- gimbal / camera ------------------------------------------------------------
    def gimbal_pitch(self, degrees: float, rate: int = 20000, **kw) -> Reply | None:
        return self.send(commands.gimbal_pitch(degrees, rate), **kw)

    def take_photo(self, **kw) -> Reply | None:
        return self.send(commands.take_photo(), **kw)

    def start_recording(self, **kw) -> Reply | None:
        return self.send(commands.start_recording(), **kw)

    def stop_recording(self, **kw) -> Reply | None:
        return self.send(commands.stop_recording(), **kw)

    # -- sticks -------------------------------------------------------------------
    @property
    def sticks(self) -> StickStreamer:
        """Virtual sticks; call ``drone.sticks.start()`` then ``.set(...)``."""
        if self._sticks is None:
            self._sticks = StickStreamer(self.link)
        return self._sticks

    # -- video ----------------------------------------------------------------------
    def on_video(self, cb: Callable[[VideoPacket], None]) -> None:
        self.link.on_video(cb)

    def video_packets(self, maxsize: int = 120) -> Iterator[VideoPacket]:
        """Iterate over video access units (drops the oldest if the consumer lags)."""
        q: queue.Queue[VideoPacket] = queue.Queue(maxsize)

        def push(pkt: VideoPacket) -> None:
            if not pkt.is_video:
                return
            if q.full():
                try:
                    q.get_nowait()
                except queue.Empty:
                    pass
            q.put_nowait(pkt)

        self.link.on_video(push)
        while not self.link.closed.is_set():
            try:
                yield q.get(timeout=0.5)
            except queue.Empty:
                continue


__all__ = ["AckTimeout", "CommandRejected", "Drone", "DroneState"]
