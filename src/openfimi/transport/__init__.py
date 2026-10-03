"""Transports carry the raw RC byte stream. Pick one with :func:`from_url`."""

from __future__ import annotations

from .aoa import AoaGadgetTransport, GadgetConfig
from .base import Transport, TransportClosed
from .capture import LoopbackTransport, RecordingTransport, ReplayTransport
from .net import BRIDGE_PORT, TcpTransport, UdpTransport
from .tty import TtyTransport


def from_url(url: str) -> Transport:
    """Build a transport from a short URL.

    ``usb`` / ``aoa``            USB gadget on this machine (root; Pi etc.)
    ``tcp://host[:port]``        an ``openfimi bridge`` on the gadget board
    ``udp[://ip]``               direct Wi-Fi to the aircraft (short range!)
    ``tty:///dev/ttyDBC0``       a raw character device
    ``replay://file.ofcap``      play back a capture
    """
    if url in ("usb", "aoa", "gadget"):
        return AoaGadgetTransport()
    if url.startswith("tcp://"):
        host, _, port = url[6:].partition(":")
        return TcpTransport(host, int(port or BRIDGE_PORT))
    if url == "udp" or url.startswith("udp://"):
        host = url[6:] if url.startswith("udp://") else ""
        return UdpTransport(host) if host else UdpTransport()
    if url.startswith("tty://"):
        return TtyTransport(url[6:])
    if url.startswith("replay://"):
        return ReplayTransport(open(url[9:], "rb"), speed=1.0)
    raise ValueError(f"unknown transport URL: {url}")


__all__ = [
    "AoaGadgetTransport",
    "GadgetConfig",
    "LoopbackTransport",
    "RecordingTransport",
    "ReplayTransport",
    "TcpTransport",
    "Transport",
    "TransportClosed",
    "TtyTransport",
    "UdpTransport",
    "from_url",
]
