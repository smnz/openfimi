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
    def __init__(self, lat: float = -43.5321, lon: float = 172.6362, rate_hz: float = 5.0) -> None:
        self.home = (lat, lon)
        self.lat, self.lon, self.alt = lat, lon, 0.0
        self.yaw = 0.0
        self.phase = 0  # 0 ground, 2 flying
        self.target: tuple[float, float, float] | None = None
        self.speed = 5.0
        self.gimbal_pitch = 0.0
        self.battery = 100.0
        self.waypoints: dict[int, bytes] = {}
        self.actions: dict[int, bytes] = {}
        self.route: list[tuple[float, float, float]] = []
        self.task_mode = 0
        self.wp_index = 0
        self.sticks = (512, 512, 512, 512)
        self.sticks_at = 0.0
        self.rate_hz = rate_hz
        self.out = None  # callable(bytes) that delivers to the ground station
        self._seq = 0
        self._outer, self._inner = OuterDecoder(), InnerDecoder()
        self._lock = threading.Lock()
        self.log: list[str] = []

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
            if self.phase == 0:
                self.phase, self.target, self.task_mode = 2, (self.lat, self.lon, 1.2), 2
            else:
                code = 1
        elif key == (Module.FC, 3, 21):
            if self.phase == 2:
                self.target, self.task_mode = (self.lat, self.lon, 0.0), 3
            else:
                code = 1
        elif key == (Module.FC, 3, 26):
            self.target, self.task_mode = (*self.home, max(self.alt, 30.0)), 7
        elif key in (
            (Module.FC, 3, 19),
            (Module.FC, 3, 24),
            (Module.FC, 3, 29),
            (Module.FC, 3, 35),
        ):
            self.target, self.route, self.task_mode = None, [], 0
        elif key == (Module.FC, 3, 36):
            self.waypoints[body[0]] = body
        elif key == (Module.FC, 3, 37):
            self.actions[body[0]] = body
        elif key == (Module.FC, 3, 32):
            if not self.waypoints or self.phase != 2:
                code = 1
            else:
                self.route = []
                for i in sorted(self.waypoints):
                    lon, lat, alt = struct.unpack_from("<ddh", self.waypoints[i], 4)
                    self.speed = max(0.5, self.waypoints[i][26] / 10)
                    self.route.append((lat, lon, alt / 10))
                self.task_mode, self.wp_index = 6, 0
                self.target = self.route[0]
        elif key == (Module.FC, 3, 38):
            wp = self.waypoints.get(body[0])
            if wp is None:
                code = 1
            else:
                reply = wp  # the read-back body has the upload body's layout
        elif key == (Module.FC, 3, 52):
            lon, lat, alt = struct.unpack_from("<ddh", body, 0)
            self.speed = max(0.5, body[20] / 10)
            self.target, self.task_mode = (lat, lon, alt / 10), 4
        elif key == (Module.GIMBAL, 9, 6):
            self.gimbal_pitch = struct.unpack_from("<h", body, 9)[0] / 100
        if f.flags & 1:
            self._ack(f, code, reply)

    # -- physics and telemetry -----------------------------------------------------
    def step(self, dt: float) -> None:
        with self._lock:
            if self.phase == 2 and time.monotonic() - self.sticks_at < 0.6:
                r, p, t, y = ((v - 512) / 512 for v in self.sticks)
                self.yaw = (self.yaw + y * 60 * dt + 180) % 360 - 180
                fwd, right = p * 8 * dt, r * 8 * dt
                h = math.radians(self.yaw)
                self.lat += (fwd * math.cos(h) - right * math.sin(h)) / M_PER_DEG
                self.lon += (fwd * math.sin(h) + right * math.cos(h)) / (
                    M_PER_DEG * math.cos(math.radians(self.lat))
                )
                self.alt = max(0.0, self.alt + t * 3 * dt)
            if self.target is not None:
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
            self.battery = max(0.0, self.battery - (0.02 if self.phase == 2 else 0.001) * dt)
            self._push()

    def _arrived(self) -> None:
        if self.task_mode == 6 and self.wp_index + 1 < len(self.route):
            self.wp_index += 1
            self.target = self.route[self.wp_index]
            return
        if self.task_mode == 7:  # RTH: land at home
            self.target, self.task_mode = (*self.home, 0.0), 3
            return
        if self.task_mode == 3 and self.alt <= 0.05:
            self.phase, self.alt = 0, 0.0
        self.target = None
        self.task_mode = 0

    def _push(self) -> None:
        fc = Module.FC
        hdr = lambda g, m: bytes((g, m, 0, 0))
        self._send(
            fc, hdr(12, 1) + struct.pack("<hh", 0, 0) + bytes((0, 0, self.phase, 0, 0, 0, 0, 1, 1))
        )
        home_d = math.hypot(
            (self.lat - self.home[0]) * M_PER_DEG,
            (self.lon - self.home[1]) * M_PER_DEG * math.cos(math.radians(self.lat)),
        )
        self._send(
            fc,
            hdr(12, 2)
            + struct.pack(
                "<ddfhhhhhbbfh",
                self.lon,
                self.lat,
                self.alt,
                int(self.speed * 10) if self.target else 0,
                0,
                0,
                0,
                int(self.yaw * 10),
                0,
                0,
                home_d,
                0,
            ),
        )
        self._send(fc, hdr(12, 3) + bytes((18, 8, 10, 0, 12, 0, 100, 10)))
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
            fc, hdr(12, 6) + struct.pack("<ddf", self.home[1], self.home[0], 0.0) + bytes((1, 0, 1))
        )
        self._send(
            fc,
            hdr(3, 1)
            + bytes((self.task_mode, 1 if self.target else 0, 0))
            + struct.pack("<H", self.wp_index),
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
