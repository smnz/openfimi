"""Network transports: the TCP bridge client and the Wi-Fi/UDP bench fallback."""

from __future__ import annotations

import select
import socket

from .base import Transport, TransportClosed

DRONE_WIFI_IP = "192.168.40.210"
UDP_PORT_CMD = 10051
UDP_PORT_AUX = 9397
BRIDGE_PORT = 10052


class TcpTransport(Transport):
    """Client for ``openfimi bridge`` running on the gadget board (e.g. a Pi).

    The bridge relays the raw RC byte stream both ways, so this behaves
    exactly like the USB link but can run on any machine on the network.
    """

    def __init__(self, host: str, port: int = BRIDGE_PORT, connect_timeout: float = 5.0):
        self.host, self.port, self.connect_timeout = host, port, connect_timeout
        self.name = f"tcp://{host}:{port}"
        self.sock: socket.socket | None = None

    def open(self) -> None:
        self.sock = socket.create_connection((self.host, self.port), self.connect_timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(None)

    def read(self, timeout: float | None = None) -> bytes:
        sock = self.sock
        if sock is None:
            raise TransportClosed(self.name)
        try:
            r, _, _ = select.select([sock], [], [], timeout)
            if not r:
                return b""
            data = sock.recv(65536)
        except (OSError, ValueError) as e:  # ValueError: closed under us
            raise TransportClosed(str(e)) from e
        if not data:
            raise TransportClosed(f"{self.name}: peer closed")
        return data

    def write(self, data: bytes) -> None:
        if self.sock is None:
            raise TransportClosed(self.name)
        try:
            self.sock.sendall(data)
        except OSError as e:
            raise TransportClosed(str(e)) from e

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None


class UdpTransport(Transport):
    """Direct Wi-Fi link to the aircraft's access point.

    SHORT RANGE: this bypasses the RC radio entirely.  Only for bench work.
    Mirrors the app: one socket per port, each bound to the same local port it
    sends to; commands go out on 10051, and both sockets are read.
    """

    def __init__(
        self, host: str = DRONE_WIFI_IP, ports: tuple[int, ...] = (UDP_PORT_CMD, UDP_PORT_AUX)
    ):
        self.host, self.ports = host, ports
        self.name = f"udp://{host}:{','.join(map(str, ports))}"
        self.socks: list[socket.socket] = []

    def open(self) -> None:
        for port in self.ports:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("", port))
            self.socks.append(s)

    def read(self, timeout: float | None = None) -> bytes:
        socks = list(self.socks)
        if not socks:
            raise TransportClosed(self.name)
        try:
            r, _, _ = select.select(socks, [], [], timeout)
            if not r:
                return b""
            data, _addr = r[0].recvfrom(65536)
        except (OSError, ValueError) as e:
            raise TransportClosed(str(e)) from e
        return data

    def write(self, data: bytes) -> None:
        if not self.socks:
            raise TransportClosed(self.name)
        self.socks[0].sendto(data, (self.host, self.ports[0]))

    def close(self) -> None:
        for s in self.socks:
            s.close()
        self.socks = []
