"""A raw character device as transport (e.g. ``/dev/ttyDBC0`` from xHCI DbC)."""

from __future__ import annotations

import os
import select
import termios
import tty

from .base import Transport, TransportClosed


class TtyTransport(Transport):
    def __init__(self, path: str) -> None:
        self.path = path
        self.name = path
        self.fd: int | None = None

    def open(self) -> None:
        self.fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY)
        try:
            tty.setraw(self.fd)
            attrs = termios.tcgetattr(self.fd)
            attrs[3] &= ~termios.ECHO
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        except termios.error:
            pass  # not a real tty; plain char device is fine

    def read(self, timeout: float | None = None) -> bytes:
        if self.fd is None:
            raise TransportClosed(self.path)
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return b""
        try:
            data = os.read(self.fd, 16384)
        except OSError as e:
            raise TransportClosed(str(e)) from e
        if not data:
            raise TransportClosed(f"{self.path}: EOF")
        return data

    def write(self, data: bytes) -> None:
        if self.fd is None:
            raise TransportClosed(self.path)
        view = memoryview(data)
        while view:
            n = os.write(self.fd, view)
            view = view[n:]

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
