"""Capture files: record a live session, replay it later.

Format: the 8-byte magic ``OFCAP\\x00\\x01\\n`` then records of
``<B direction><d unix_time><I length><bytes>`` (little-endian), where
direction 0 = received from the RC, 1 = sent by us.
"""

from __future__ import annotations

import struct
import threading
import time
from collections.abc import Iterator
from typing import BinaryIO

from .base import Transport, TransportClosed

MAGIC = b"OFCAP\x00\x01\n"
RX, TX = 0, 1
_REC = struct.Struct("<BdI")


class CaptureWriter:
    def __init__(self, fp: BinaryIO) -> None:
        self.fp = fp
        self._lock = threading.Lock()
        fp.write(MAGIC)

    def record(self, direction: int, data: bytes) -> None:
        with self._lock:
            self.fp.write(_REC.pack(direction, time.time(), len(data)) + data)
            self.fp.flush()


def read_capture(fp: BinaryIO) -> Iterator[tuple[int, float, bytes]]:
    if fp.read(len(MAGIC)) != MAGIC:
        raise ValueError("not an openfimi capture file")
    while True:
        head = fp.read(_REC.size)
        if len(head) < _REC.size:
            return
        d, t, n = _REC.unpack(head)
        yield d, t, fp.read(n)


class RecordingTransport(Transport):
    """Wraps another transport and records both directions to a capture file."""

    def __init__(self, inner: Transport, fp: BinaryIO) -> None:
        self.inner = inner
        self.writer = CaptureWriter(fp)
        self.name = f"{inner.name} (recording)"

    def open(self) -> None:
        self.inner.open()

    def read(self, timeout: float | None = None) -> bytes:
        data = self.inner.read(timeout)
        if data:
            self.writer.record(RX, data)
        return data

    def write(self, data: bytes) -> None:
        self.writer.record(TX, data)
        self.inner.write(data)

    def close(self) -> None:
        self.inner.close()


class ReplayTransport(Transport):
    """Plays back the received side of a capture; writes are discarded.

    ``speed`` = 1.0 replays in real time, 0 replays as fast as possible.
    """

    def __init__(self, fp: BinaryIO, speed: float = 0.0) -> None:
        self.records = [(t, d) for direction, t, d in read_capture(fp) if direction == RX]
        self.speed = speed
        self.name = "replay"
        self._i = 0
        self._t0: float | None = None
        self.sent: list[bytes] = []

    def read(self, timeout: float | None = None) -> bytes:
        if self._i >= len(self.records):
            raise TransportClosed("end of capture")
        t, data = self.records[self._i]
        if self.speed > 0:
            if self._t0 is None:
                self._t0 = time.monotonic() - (t - self.records[0][0]) / self.speed
            delay = self._t0 + (t - self.records[0][0]) / self.speed - time.monotonic()
            if delay > 0:
                if timeout is not None and delay > timeout:
                    time.sleep(timeout)
                    return b""
                time.sleep(delay)
        self._i += 1
        return data

    def write(self, data: bytes) -> None:
        self.sent.append(data)


class LoopbackTransport(Transport):
    """In-memory transport for tests and simulators: feed() bytes in, inspect sent."""

    def __init__(self) -> None:
        import queue

        self.name = "loopback"
        self._q: queue.Queue[bytes | None] = queue.Queue()
        self.sent: list[bytes] = []
        self.on_write = None  # optional callback(bytes) e.g. a simulated aircraft

    def feed(self, data: bytes) -> None:
        self._q.put(data)

    def read(self, timeout: float | None = None) -> bytes:
        import queue

        try:
            item = self._q.get(timeout=timeout)
        except queue.Empty:
            return b""
        if item is None:
            raise TransportClosed("loopback closed")
        return item

    def write(self, data: bytes) -> None:
        self.sent.append(data)
        if self.on_write:
            self.on_write(data)

    def close(self) -> None:
        self._q.put(None)
