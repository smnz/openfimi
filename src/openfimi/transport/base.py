"""Transport interface.

A transport moves raw *outer-framed* bytes (the 0xAE-wrapped stream) between
the host and the RC/aircraft.  Stream transports (USB, TCP bridge, tty) may
split or merge frames arbitrarily; datagram transports (UDP) deliver one frame
per read.  The link layer's decoder copes with both.
"""

from __future__ import annotations

import abc


class TransportClosed(Exception):
    """Raised by read()/write() once the transport is closed or lost."""


class Transport(abc.ABC):
    #: Human-readable description for logs.
    name: str = "transport"

    def open(self) -> None:
        """Establish the connection (may block until a peer appears)."""

    @abc.abstractmethod
    def read(self, timeout: float | None = None) -> bytes:
        """Return the next chunk of received bytes, b"" on timeout.

        Raises TransportClosed when the transport is gone for good.
        """

    @abc.abstractmethod
    def write(self, data: bytes) -> None:
        """Send bytes (a fully framed outer packet, or several)."""

    def close(self) -> None:
        pass

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()
