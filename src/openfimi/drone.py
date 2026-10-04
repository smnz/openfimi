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
from collections import deque
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field

from . import commands, telemetry
from .commands import Command
from .link import AckTimeout, Link, Reply
from .mission import Mission
from .modules import Module
from .sticks import StickStreamer
from .transport.base import Transport
from .video import VideoPacket

log = logging.getLogger(__name__)


class PreflightError(RuntimeError):
    """The aircraft is not ready to fly; the message says why."""


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
    notice: dict | None = None  # the latest bridge notice (e.g. its emergency RTH button)
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
            out.update(
                phase=self.heart.flight_phase,
                flying=self.heart.flying,
                takeoff_block=self.heart.takeoff_block,
            )
        if self.errors and self.errors.bits():
            out.update(faults=self.errors.bits())
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
        self.link.on_notice(self._on_notice)
        self.link.on_frame(self._on_frame)
        self._clock_sent = 0.0

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

    def _on_frame(self, frame) -> None:
        # The camera asks for the time (CAMERA 2/135, every 2 s) until it gets
        # an answer, and keeps what it is given until powered off.  Unanswered,
        # it stamps media with GPS time, i.e. UTC.  The on-connect clock misses a
        # camera switched on after connecting (hands-off launch), so answer
        # every request as the app does.
        if (
            self.init_camera
            and frame.src == Module.CAMERA
            and frame.dst == Module.GCS
            and frame.group == 2
            and frame.msg_id == 135
            and time.monotonic() - self._clock_sent > 1.0
        ):
            self._clock_sent = time.monotonic()
            self.link.send(commands.set_camera_clock())

    def _on_notice(self, notice: dict) -> None:
        self.state.notice = notice
        self.state.updated["notice"] = time.monotonic()
        if notice.get("event") == "emergency_rth" and self._sticks is not None:
            # The bridge has commanded return home: stop fighting it with sticks.
            self._sticks.stop()

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

    def emergency_rth(self, on_event: Callable[[str], None] | None = None) -> bool:
        """Abandon whatever the aircraft is doing and return home, then land.

        Stops virtual sticks (centred), exits a running route (3/35) or fly-to
        (3/51), then sends return home (3/26) until it is accepted, up to three
        times.  Returns True once accepted.  The bridge app's emergency button
        sends the same commands.  If the aircraft is on the ground there is
        nothing to do and the command is refused.
        """
        say = on_event or (lambda msg: log.info(msg))
        if self._sticks is not None:
            self._sticks.stop()
        nav = self.state.navigation
        task = nav.task_mode if nav else None
        exits = ((commands.mission_stop(), (1, None)), (commands.fly_to_exit(), (2, None)))
        for cmd, modes in exits:  # task None: state unknown, send both
            if task in modes:
                try:
                    r = self.send(cmd, timeout=1.5)
                    say(f"{cmd.name}: {'ok' if r is None or r.ok else f'code {r.code}'}")
                except AckTimeout:
                    say(f"{cmd.name}: no reply")
        for _ in range(3):
            try:
                r = self.send(commands.return_home(), timeout=1.5)
            except AckTimeout:
                say("return home: no reply")
                continue
            if r is None or r.ok:
                say("return home accepted")
                return True
            say(f"return home refused (code {r.code})")
            time.sleep(0.3)
        return False

    def cancel_takeoff(self, **kw) -> Reply | None:
        return self.send(commands.cancel_takeoff(), **kw)

    def cancel_land(self, **kw) -> Reply | None:
        return self.send(commands.cancel_land(), **kw)

    def cancel_return_home(self, **kw) -> Reply | None:
        return self.send(commands.cancel_return_home(), **kw)

    def fly_to(
        self, lat: float, lon: float, alt_m: float, speed_ms: float = 5.0, **kw
    ) -> Reply | None:
        """Point-to-point flight (the app's tap-to-fly): set the target, then go.

        Returns the reply to the go command; raises CommandRejected (with
        ``check=True``) if either step is refused.
        """
        r = self.send(commands.fly_to(lat, lon, alt_m, speed_ms), **kw)
        if r is not None and not r.ok:
            return r
        return self.send(commands.fly_to_start(), **kw)

    def fly_to_exit(self, **kw) -> Reply | None:
        return self.send(commands.fly_to_exit(), **kw)

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

    def preflight(self, min_battery: int = 30, min_satellites: int = 10) -> list[str]:
        """Reasons the aircraft should not take off now (empty list = ready)."""
        return [msg for msg, _ in self._preflight(min_battery, min_satellites)]

    def _preflight(self, min_battery: int, min_satellites: int) -> list[tuple[str, bool]]:
        """(problem, waitable) pairs; waitable problems can clear by themselves."""
        s = self.state
        if not (s.heart and s.battery and s.signal and s.errors):
            return [("telemetry incomplete", True)]
        problems = []
        if s.errors.sensor_overheat:
            problems.append(("sensor temperature too high: power off and cool down", False))
        if s.battery.percent < min_battery:
            problems.append((f"battery {s.battery.percent}% < {min_battery}%", False))
        if s.signal.satellites < min_satellites:
            problems.append((f"{s.signal.satellites}/{min_satellites} satellites", True))
        if s.home is None or (s.home.lat == 0 and s.home.lon == 0):
            problems.append(("no home point yet", True))
        if s.heart.takeoff_block:
            problems.append((f"aircraft refuses take-off (code {s.heart.takeoff_block})", True))
        return problems

    def wait_until_ready(
        self,
        timeout: float = 600.0,
        *,
        min_satellites: int = 10,
        min_battery: int = 30,
        settle_s: float = 5.0,
        level_deg: float = 8.0,
        on_event: Callable[[str], None] | None = None,
        cancel: threading.Event | None = None,
    ) -> None:
        """Wait until the pre-flight checks pass and the aircraft has settled.

        Keeps waiting while the only problems are ones that clear by themselves
        (satellites, home point, a take-off block code, not yet settled); raises
        PreflightError at once for ones that do not (overheat, low battery) and
        on timeout.

        *Settled* (``settle_s`` > 0): for the last ``settle_s`` seconds the
        aircraft has been level (roll and pitch within ``level_deg``), still
        (roll/pitch varying < 1 deg, yaw < 2 deg, height < 0.3 m) and not moving
        (GPS ground speed < 0.3 m/s).  Being carried breaks all of these, so a
        drone switched on while walking to the launch spot is not launched
        until it has been put down.

        Setting ``cancel`` stops the wait with PreflightError("cancelled").
        """
        say = on_event or (lambda msg: log.info(msg))
        end = time.monotonic() + timeout
        last = None
        samples: deque = deque()
        while True:
            if cancel is not None and cancel.is_set():
                raise PreflightError("cancelled")
            problems = self._preflight(min_battery, min_satellites)
            sp = self.state.sport
            if settle_s > 0 and sp is not None:
                now = time.monotonic()
                samples.append(
                    (now, sp.roll_deg, sp.pitch_deg, sp.yaw_deg, sp.height_m, sp.ground_speed_ms)
                )
                while samples and now - samples[0][0] > settle_s:
                    samples.popleft()
                why = self._unsettled(samples, settle_s, level_deg)
                if why:
                    problems.append((why, True))
            if not problems:
                s = self.state
                say(
                    f"ready: {s.signal.satellites} satellites, home point set, "
                    f"battery {s.battery.percent}% {s.battery.temperature_c:.0f} C"
                    + (", settled" if settle_s > 0 else "")
                )
                return
            hard = [msg for msg, waitable in problems if not waitable]
            if hard:
                raise PreflightError("; ".join(hard))
            text = "; ".join(msg for msg, _ in problems)
            if text != last:
                say(f"waiting: {text}")
                last = text
            if time.monotonic() > end:
                raise PreflightError(f"not ready after {timeout:.0f} s: {text}")
            time.sleep(0.2)

    @staticmethod
    def _unsettled(samples, settle_s: float, level_deg: float) -> str | None:
        if not samples:
            return "not settled (no attitude yet)"
        t, roll, pitch, yaw, h, gs = samples[-1]
        if abs(roll) > level_deg or abs(pitch) > level_deg:
            return f"not level (roll {roll:.0f}, pitch {pitch:.0f})"
        if gs >= 0.3:
            return f"moving ({gs:.1f} m/s)"

        def spread(i):
            vals = [x[i] for x in samples]
            return max(vals) - min(vals)

        def yaw_spread():
            ys = [x[3] for x in samples]
            ref = ys[0]
            d = [((y - ref + 180) % 360) - 180 for y in ys]
            return max(d) - min(d)

        if spread(1) >= 1.0 or spread(2) >= 1.0 or yaw_spread() >= 2.0 or spread(4) >= 0.3:
            return "not settled (being moved)"
        if t - samples[0][0] < settle_s - 0.3:
            return f"settling ({t - samples[0][0]:.0f}/{settle_s:.0f} s)"
        return None

    def fly_route(
        self,
        mission: Mission,
        *,
        takeoff: bool = True,
        wait_ready: float = 0.0,
        min_satellites: int = 10,
        settle_s: float = 5.0,
        follow_gimbal: bool | None = None,
        gimbal_lead_s: float = 15.0,
        wait: bool = True,
        timeout: float = 1800.0,
        on_event: Callable[[str], None] | None = None,
        cancel: threading.Event | None = None,
        stop_at_end: bool = False,
    ) -> dict:
        """Fly a route end to end, the way the flight tests did it.

        Take off if on the ground (after the pre-flight checks; with
        ``wait_ready`` seconds, first wait for GPS / home point / take-off
        clearance as in :meth:`wait_until_ready`), upload the route in
        the air, read it back and compare, start it, and (with ``wait``) monitor
        until it ends, landed if its finish action is return home.  Any failure
        before the route starts lands the aircraft if this call launched it.

        ``follow_gimbal`` drives the per-waypoint gimbal pitch the aircraft
        itself ignores (see :mod:`openfimi.follow`); by default it is on when any
        waypoint has a gimbal mode other than NONE.

        Setting ``cancel`` before take-off (while waiting for the aircraft or for
        it to be ready) abandons the launch with PreflightError("cancelled").
        After take-off but before the route starts it stops the route from being
        started (RuntimeError("cancelled")) and leaves the aircraft hovering for
        the caller, e.g. after :meth:`emergency_rth`; during the route it only
        stops the monitoring and the gimbal follower.

        ``stop_at_end`` returns as soon as the route itself ends, without
        waiting for its finish action (a return home, say), so a caller can
        chain the next route: ``fly_route(next, takeoff=False)`` uploads it in
        the air.  The result's ``completed`` says whether the route really ran
        to its last waypoint; it is False if it was cut short (pilot, RC
        return home, signal loss), when nothing should be chained.
        """
        from .follow import GimbalFollower, ground_distance

        say = on_event or (lambda msg: log.info(msg))
        mission.validate()
        s = self.state
        launched = False

        def bail(msg: str):
            say(f"ABORT: {msg}")
            if launched:
                say("landing")
                self.send(commands.land())
            raise RuntimeError(msg)

        if wait_ready > 0 and (s.heart is None or s.sport is None):
            # The aircraft may not even be switched on yet: wait for it.
            say("waiting for the aircraft to power on")
            if not self.wait_until(
                lambda st: (cancel is not None and cancel.is_set()) or (st.heart and st.sport),
                wait_ready,
            ):
                raise PreflightError(f"no aircraft telemetry after {wait_ready:.0f} s")
            if cancel is not None and cancel.is_set():
                raise PreflightError("cancelled")
            say("aircraft telemetry received")
        if s.heart is None or s.sport is None:
            raise PreflightError("no telemetry")
        if not s.flying:
            if not takeoff:
                raise PreflightError("aircraft is on the ground and takeoff=False")
            if wait_ready > 0:
                self.wait_until_ready(
                    wait_ready,
                    min_satellites=min_satellites,
                    settle_s=settle_s,
                    on_event=say,
                    cancel=cancel,
                )
            if cancel is not None and cancel.is_set():
                raise PreflightError("cancelled")
            problems = self.preflight(min_satellites=min_satellites)
            if problems:
                raise PreflightError("; ".join(problems))
            r = self.takeoff(timeout=5)
            if r is None or not r.ok:
                raise PreflightError(f"take-off refused (code {r.code if r else '?'})")
            launched = True
            say("taking off")
            if not self.wait_until(
                lambda st: st.heart.flight_phase == 3 and st.sport.height_m > 2, 30
            ):
                bail("did not reach a hover")

        def check_cancel():
            if cancel is not None and cancel.is_set():
                say("cancelled in the air: route not started")
                raise RuntimeError("cancelled")

        check_cancel()
        try:
            self.upload_mission(
                mission,
                check=True,
                progress=lambda i, n: say(f"uploaded {i}/{n}") if i == n else None,
            )
            pts = self.read_mission(len(mission.waypoints))
        except Exception as e:  # noqa: BLE001 - land on any upload failure
            bail(f"upload failed: {e}")
        bad = [
            i
            for i, (p, w) in enumerate(zip(pts, mission.waypoints, strict=False))
            if abs(p.lat - w.lat) > 1e-7
            or abs(p.lon - w.lon) > 1e-7
            or abs(p.alt_m - w.alt_m) > 0.15
        ]
        if len(pts) != len(mission.waypoints) or bad:
            bail(f"read-back mismatch at waypoints {bad or 'count'}")
        say(f"route verified ({len(pts)} waypoints)")
        check_cancel()

        follow = (
            follow_gimbal
            if follow_gimbal is not None
            else any(w.gimbal_mode for w in mission.waypoints)
        )
        follower = (
            GimbalFollower(self, mission, lead_s=gimbal_lead_s, on_event=say) if follow else None
        )
        if follower:
            for line in follower.plan():
                say(f"gimbal plan: {line}")
            follower.start()
        r = self.start_mission(timeout=5)
        if r is None or not r.ok:
            if follower:
                follower.stop()
            bail(f"route start refused (code {r.code if r else '?'})")
        say("route started")
        result = {"verified": True, "started": True, "gimbal_follow": follow}
        if not wait:
            result["follower"] = follower
            return result

        end = time.monotonic() + timeout
        last_wp, seen_route = -1, False
        try:
            while time.monotonic() < end:
                time.sleep(0.5)
                if cancel is not None and cancel.is_set():
                    say("cancelled: no longer following the route")
                    break
                st = self.state
                nav = st.navigation
                if nav and nav.task_mode == 1:
                    seen_route = True
                    if nav.waypoint != last_wp and nav.waypoint != 0xFFFF:
                        last_wp = nav.waypoint
                        if last_wp > 0:
                            say(f"reached waypoint {last_wp - 1}")
                elif seen_route and nav and nav.task_mode != 1:
                    n = len(mission.waypoints)
                    if "completed" not in result:
                        last = mission.waypoints[-1]
                        near = ground_distance((st.sport.lat, st.sport.lon), (last.lat, last.lon))
                        # apStatus 5 = the route's own finish (a commanded RTH reads 2)
                        at_last = last_wp >= n - 1 and near < 15
                        done = last_wp >= n or nav.ap_status == 5 or at_last
                        result["completed"] = done
                        if done and last_wp < n:
                            say(f"reached waypoint {n - 1} (route complete)")
                        elif not done:
                            say(f"route left after {max(last_wp, 0)}/{n} waypoints (interrupted)")
                        last_wp = n
                    if stop_at_end:
                        break
                    if int(mission.finish) != 4:
                        say("route finished")
                        break
                    if st.heart.flight_phase == 1 and st.sport.height_m < 0.5:
                        home = (
                            ground_distance(
                                (st.sport.lat, st.sport.lon), (st.home.lat, st.home.lon)
                            )
                            if st.home
                            else float("nan")
                        )
                        say(f"landed {home:.1f} m from home")
                        result["landed"] = True
                        break
        finally:
            if follower:
                follower.stop()
        return result

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
