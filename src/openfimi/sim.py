"""A crude simulated aircraft that speaks the wire protocol.

It acknowledges commands, pushes the FC telemetry set at 5 Hz, and moves in
a plausible (not physical) way: take-off climbs to 1.2 m, land descends, RTH
flies home, a started mission visits the uploaded waypoints, virtual sticks
nudge position, and the gimbal follows pitch commands.  Run it as a TCP server
so any client can use ``-u tcp://localhost:10052`` exactly as with a bridge::

    openfimi sim

Its message bodies are built from the same layouts the decoders read, so it
tests the plumbing, not the assumptions about the real aircraft.
"""

from __future__ import annotations

import math
import socket
import struct
import threading
import time

from .framing import InnerDecoder, OuterDecoder, StreamType, encode_inner, encode_outer
from .modules import Module

M_PER_DEG = 111_320.0


class SimAircraft:
    """Telemetry codes follow what the real aircraft sent in flight tests:
    flightPhase 1 ground / 2 taking off / 3 flying / 4 landing; taskMode 4
    take-off and hover, 2 fly-to, 1 route, 3 return home, 5 land; during a
    route ``wpNUM`` counts the waypoints reached.
    """

    TAKEOFF_ALT = 2.5
    RTH_ALT = 30.0

    def __init__(self, lat: float = -43.5321, lon: float = 172.6362, rate_hz: float = 5.0) -> None:
        self.home = (lat, lon)
        self.lat, self.lon, self.alt = lat, lon, 0.0
        self.yaw = 0.0
        self.activity = "ground"  # ground takeoff hover fly_to route rth land
        self.target: tuple[float, float, float] | None = None
        self.speed = 5.0
        self.gimbal_pitch = 0.0
        self.gimbal_log: list[float] = []
        self.battery = 100.0
        self.satellites = 18
        self.carried = False  # jiggle the attitude/position as if hand-carried
        self.home_set = True
        self.waypoints: dict[int, bytes] = {}
        self.actions: dict[int, bytes] = {}
        self.route: list[tuple[float, float, float]] = []
        self.route_i = 0
        self.reached = 0
        self.dwell_until = 0.0
        self.task_mode = 0
        self.ap_status = 0
        self.photos: list[tuple[int, float]] = []  # (waypoint, gimbal pitch) per photo
        self.sticks = (512, 512, 512, 512)
        self.sticks_at = 0.0
        self.rate_hz = rate_hz
        self.out = None  # callable(bytes) that delivers to the ground station
        self._seq = 0
        self._fly_to = None
        self._outer, self._inner = OuterDecoder(), InnerDecoder()
        self._lock = threading.Lock()
        self.log: list[str] = []

    @property
    def phase(self) -> int:
        return {"ground": 1, "takeoff": 2, "land": 4}.get(self.activity, 3)

    @property
    def flying(self) -> bool:
        return self.activity != "ground"

    # -- wire helpers --------------------------------------------------------------
    def _send(self, src: int, payload: bytes, seq: int | None = None) -> None:
        if seq is None:
            self._seq = (self._seq + 1) % 32766
            seq = self._seq
        if self.out:
            self.out(encode_outer(encode_inner(src, Module.GCS, seq, payload)))

    def _ack(self, frame, code: int = 0, body: bytes = b"") -> None:
        p = bytes((frame.group, frame.msg_id, (code & 0x0F) << 4, (code >> 4) & 0xFF)) + body
        self._send(frame.dst, p, frame.seq)

    # -- receive -------------------------------------------------------------------
    def receive(self, data: bytes) -> None:
        for stype, body in self._outer.feed(data):
            if stype != StreamType.FMLINK:
                continue
            for f in self._inner.feed(body):
                with self._lock:
                    self._handle(f)

    def _go(self, activity: str, target, task_mode: int, ap_status: int = 0) -> None:
        self.activity, self.target, self.task_mode, self.ap_status = (
            activity,
            target,
            task_mode,
            ap_status,
        )

    def _handle(self, f) -> None:
        key = (f.dst, f.group, f.msg_id)
        self.log.append(f"{f.group}/{f.msg_id}")
        body = f.body
        if key == (Module.FC, 11, 2):  # virtual sticks, no ack
            self.sticks = struct.unpack_from("<4h", body, 0)
            self.sticks_at = time.monotonic()
            return
        if key[1:] == (8, 4):  # time sync, no ack
            return
        code = 0
        reply = b""
        if key == (Module.FC, 3, 16):
            if self.activity == "ground":
                self.speed = 1.0
                self._go("takeoff", (self.lat, self.lon, self.TAKEOFF_ALT), 4)
            else:
                code = 1
        elif key == (Module.FC, 3, 21):
            if self.flying:
                self._go("land", (self.lat, self.lon, 0.0), 5, 32)
            else:
                code = 22  # what the real aircraft answers on the ground
        elif key == (Module.FC, 3, 26):
            if self.flying:
                self._rth(ap_status=2)
            else:
                code = 1
        elif key in (
            (Module.FC, 3, 19),
            (Module.FC, 3, 24),
            (Module.FC, 3, 29),
            (Module.FC, 3, 35),
            (Module.FC, 3, 51),
        ):
            if self.flying:
                self._go("hover", None, 4)
        elif key == (Module.FC, 3, 36):
            self.waypoints[body[0]] = body
        elif key == (Module.FC, 3, 37):
            self.actions[body[0]] = body
        elif key == (Module.FC, 3, 32):
            if not self.waypoints or not self.flying:
                code = 1
            else:
                self.route = []
                for i in sorted(self.waypoints):
                    lon, lat, alt = struct.unpack_from("<ddh", self.waypoints[i], 4)
                    self.speed = max(0.5, self.waypoints[i][26] / 10)
                    self.route.append((lat, lon, alt / 10))
                self.route_i = self.reached = 0
                # Real aircraft: wpNUM reads 65535 for a moment as the route starts.
                self._sentinel_until = time.monotonic() + 0.5
                self._go("route", self.route[0], 1)
        elif key == (Module.FC, 3, 38):
            wp = self.waypoints.get(body[0])
            if wp is None:
                code = 1
            else:
                reply = wp  # the read-back body has the upload body's layout
        elif key == (Module.FC, 3, 52):
            lon, lat, alt = struct.unpack_from("<ddh", body, 0)
            self.speed = max(0.5, body[20] / 10)
            self._fly_to = (lat, lon, alt / 10)
        elif key == (Module.FC, 3, 48):
            if self._fly_to is None or not self.flying:
                code = 30  # what the real aircraft answers without a target
            else:
                self._go("fly_to", self._fly_to, 2)
        elif key == (Module.GIMBAL, 9, 6):
            self.gimbal_pitch = struct.unpack_from("<h", body, 9)[0] / 100
            self.gimbal_log.append(self.gimbal_pitch)
        if f.flags & 1:
            self._ack(f, code, reply)

    def _rth(self, ap_status: int) -> None:
        self.speed = 5.0
        self._go("rth", (self.lat, self.lon, max(self.alt, self.RTH_ALT)), 3, ap_status)
        self._rth_stage = 0

    # -- physics and telemetry -----------------------------------------------------
    def step(self, dt: float) -> None:
        with self._lock:
            if self.flying and time.monotonic() - self.sticks_at < 0.6:
                r, p, t, y = ((v - 512) / 512 for v in self.sticks)
                p, t = -p, -t  # the RC encodes forward and up as low values
                self.yaw = (self.yaw + y * 60 * dt + 180) % 360 - 180
                fwd, right = p * 8 * dt, r * 8 * dt
                h = math.radians(self.yaw)
                self.lat += (fwd * math.cos(h) - right * math.sin(h)) / M_PER_DEG
                self.lon += (fwd * math.sin(h) + right * math.cos(h)) / (
                    M_PER_DEG * math.cos(math.radians(self.lat))
                )
                self.alt = max(0.0, self.alt + t * 3 * dt)
            if self.target is not None and time.monotonic() >= self.dwell_until:
                tl, tn, ta = self.target
                dn = (tl - self.lat) * M_PER_DEG
                de = (tn - self.lon) * M_PER_DEG * math.cos(math.radians(self.lat))
                dist = math.hypot(dn, de)
                step = self.speed * dt
                if dist > 0.3:
                    k = min(1.0, step / dist)
                    self.lat += dn * k / M_PER_DEG
                    self.lon += de * k / (M_PER_DEG * math.cos(math.radians(self.lat)))
                    self.yaw = math.degrees(math.atan2(de, dn))
                dz = ta - self.alt
                self.alt += max(-3 * dt, min(3 * dt, dz))
                if dist <= 0.3 and abs(dz) < 0.1:
                    self._arrived()
            self.battery = max(0.0, self.battery - (0.02 if self.flying else 0.001) * dt)
            self._push()

    def _arrived(self) -> None:
        a = self.activity
        if a == "takeoff":
            self._go("hover", None, 4)
        elif a == "fly_to":
            self._go("hover", None, 2)
        elif a == "route":
            i = self.route_i
            self.reached = i + 1  # the real wpNUM ticks on arrival, before the action
            act = self.actions.get(i)
            if act is not None and act[4] in (1, 2):  # HOVER or PHOTO in the first slot
                self.dwell_until = time.monotonic() + 1.0
                if act[4] == 2 or act[5] == 2:
                    self.photos.append((i, self.gimbal_pitch))
            if i + 1 < len(self.route):
                self.route_i = i + 1
                self.target = self.route[self.route_i]
            elif self.waypoints[i][34] == 4:  # finish action: return home
                self._rth(ap_status=5)
            else:  # hover at the last waypoint
                self._go("hover", None, 4)
        elif a == "rth":
            if getattr(self, "_rth_stage", 0) == 0:
                self._rth_stage = 1
                self.target = (*self.home, self.alt)
            else:
                self.speed = 1.5
                self._go("land", (*self.home, 0.0), 3, self.ap_status)
        elif a == "land" and self.alt <= 0.05:
            self.alt = 0.0
            self.activity, self.target = "ground", None

    def _push(self) -> None:
        fc = Module.FC
        hdr = lambda g, m: bytes((g, m, 0, 0))
        self._send(
            fc, hdr(12, 1) + struct.pack("<hh", 0, 0) + bytes((0, 0, self.phase, 0, 0, 0, 0, 0, 0))
        )
        home_d = math.hypot(
            (self.lat - self.home[0]) * M_PER_DEG,
            (self.lon - self.home[1]) * M_PER_DEG * math.cos(math.radians(self.lat)),
        )
        moving = self.target is not None and time.monotonic() >= self.dwell_until
        jig = math.sin(time.monotonic() * 7) if self.carried else 0.0
        roll, pitch = 12 * jig, 6 * jig
        walk = 130 if self.carried else 0
        self._send(
            fc,
            hdr(12, 2)
            + struct.pack(
                "<ddfhhhhhbbfh",
                self.lon,
                self.lat,
                self.alt,
                int(self.speed * 100) if moving else walk,
                0,
                int(roll * 10),
                int(pitch * 10),
                int((self.yaw + 20 * jig) * 10),
                0,
                0,
                home_d,
                0,
            ),
        )
        self._send(fc, hdr(12, 3) + bytes((self.satellites, 8, 10, 0, 12, 0, 100, 10)))
        self._send(fc, hdr(12, 4) + bytes(16))  # no faults
        cell = int((3.5 + 0.7 * self.battery / 100 - 2.0) * 100)
        self._send(
            fc,
            hdr(12, 5)
            + bytes((cell, cell, 0, 0))
            + struct.pack("<hhhhh", int(2250 * self.battery / 100), 2250, 50, 300, 1200)
            + bytes((int(self.battery), 0, 0, 0))
            + struct.pack("<hhh", 0, 0, 12),
        )
        self._send(
            fc,
            hdr(12, 6)
            + (struct.pack("<ddf", self.home[1], self.home[0], 0.0) if self.home_set else bytes(20))
            + bytes((1, 0, 1)),
        )
        wp = self.reached if self.activity == "route" else 0
        if self.activity == "route" and time.monotonic() < getattr(self, "_sentinel_until", 0):
            wp = 0xFFFF
        self._send(
            fc,
            hdr(3, 1) + bytes((self.task_mode, 2, self.ap_status)) + struct.pack("<H", wp),
        )
        self._send(
            Module.GIMBAL,
            hdr(9, 1) + struct.pack("<hBBhhh", 0, 0, 0, 0, int(self.gimbal_pitch * 100), 0),
        )

    def run(self, stop: threading.Event) -> None:
        dt = 1.0 / self.rate_hz
        while not stop.is_set():
            self.step(dt)
            stop.wait(dt)


def serve(host: str = "127.0.0.1", port: int = 10052, **kw) -> None:
    """Serve one persistent simulated aircraft to TCP clients.

    The aircraft keeps flying between connections (like a real one), and all
    connected clients see its telemetry.
    """
    sim = SimAircraft(**kw)
    clients: list[socket.socket] = []
    lock = threading.Lock()

    def out(data: bytes) -> None:
        with lock:
            for c in list(clients):
                try:
                    c.sendall(data)
                except OSError:
                    clients.remove(c)

    def client(conn: socket.socket, peer) -> None:
        try:
            while True:
                data = conn.recv(65536)
                if not data:
                    break
                sim.receive(data)
        except OSError:
            pass
        finally:
            with lock:
                if conn in clients:
                    clients.remove(conn)
            conn.close()
            print(f"client {peer[0]}:{peer[1]} disconnected", flush=True)

    sim.out = out
    threading.Thread(target=sim.run, args=(threading.Event(),), daemon=True).start()
    srv = socket.create_server((host, port))
    print(f"openfimi simulator on {host}:{port}  (use -u tcp://{host}:{port})", flush=True)
    while True:
        conn, peer = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"client {peer[0]}:{peer[1]} connected", flush=True)
        with lock:
            clients.append(conn)
        threading.Thread(target=client, args=(conn, peer), daemon=True).start()
