"""Relay the RC link over TCP.

Run on the gadget board (the Pi wired to the RC)::

    sudo openfimi bridge --listen 0.0.0.0:10052

and connect from any machine with ``TcpTransport("pi-host")``.  The bridge
copies bytes verbatim in both directions; every connected client receives the
full RC stream (telemetry and video) and may send commands.  Framing,
sequence numbers and retransmission all live in the client, so the Pi only
shuffles bytes.
"""

from __future__ import annotations

import logging
import socket
import threading

from .transport.base import Transport, TransportClosed

log = logging.getLogger(__name__)


class Bridge:
    def __init__(self, upstream: Transport, host: str = "0.0.0.0", port: int = 10052) -> None:
        self.upstream = upstream
        self.addr = (host, port)
        self._clients: list[socket.socket] = []
        self._lock = threading.Lock()
        self._running = False
        self.rx_bytes = 0
        self.tx_bytes = 0

    def serve_forever(self) -> None:
        self._running = True
        srv = socket.create_server(self.addr, reuse_port=False)
        log.info("bridge listening on %s:%d", *self.addr)
        threading.Thread(target=self._accept_loop, args=(srv,), daemon=True).start()
        try:
            self.upstream.open()
            log.info("upstream %s is up", self.upstream.name)
            while self._running:
                data = self.upstream.read(0.5)
                if not data:
                    continue
                self.rx_bytes += len(data)
                self._broadcast(data)
        except TransportClosed as e:
            log.warning("upstream closed: %s", e)
        finally:
            self._running = False
            srv.close()
            with self._lock:
                for c in self._clients:
                    c.close()
            self.upstream.close()

    def stop(self) -> None:
        self._running = False

    def _broadcast(self, data: bytes) -> None:
        with self._lock:
            dead = []
            for c in self._clients:
                try:
                    c.sendall(data)
                except OSError:
                    dead.append(c)
            for c in dead:
                self._clients.remove(c)
                c.close()

    def _accept_loop(self, srv: socket.socket) -> None:
        while self._running:
            try:
                conn, peer = srv.accept()
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            log.info("client connected: %s:%d", *peer)
            with self._lock:
                self._clients.append(conn)
            threading.Thread(target=self._client_loop, args=(conn, peer), daemon=True).start()

    def _client_loop(self, conn: socket.socket, peer) -> None:
        try:
            while self._running:
                data = conn.recv(65536)
                if not data:
                    break
                self.tx_bytes += len(data)
                self.upstream.write(data)
        except (OSError, TransportClosed) as e:
            log.info("client %s:%d: %s", peer[0], peer[1], e)
        finally:
            log.info("client disconnected: %s:%d", *peer)
            with self._lock:
                if conn in self._clients:
                    self._clients.remove(conn)
            conn.close()
