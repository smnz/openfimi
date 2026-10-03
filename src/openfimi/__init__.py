"""openfimi: drive a FIMI X8 Mini (V3) from Linux through its remote controller.

Layers, bottom up:

* :mod:`openfimi.transport`  raw byte links (USB gadget, TCP bridge, UDP, replay)
* :mod:`openfimi.framing`    0xAE outer wrapper and FmLink4 frames, CRCs
* :mod:`openfimi.link`       sequence numbers, ACKs, retransmission, demux
* :mod:`openfimi.commands`   command builders; :mod:`openfimi.mission` routes
* :mod:`openfimi.telemetry`  decoded aircraft messages
* :mod:`openfimi.video`      RTP to H.264/H.265 access units
* :class:`openfimi.Drone`    the high-level API
"""

from . import commands, mission, telemetry, transport
from .commands import Command
from .drone import CommandRejected, Drone, DroneState
from .framing import Frame
from .link import AckTimeout, Link, Reply
from .mission import Heading, Mission, PointAction, Waypoint
from .modules import Module

__version__ = "0.1.0"

__all__ = [
    "AckTimeout",
    "Command",
    "CommandRejected",
    "Drone",
    "DroneState",
    "Frame",
    "Heading",
    "Link",
    "Mission",
    "Module",
    "PointAction",
    "Reply",
    "Waypoint",
    "__version__",
    "commands",
    "mission",
    "telemetry",
    "transport",
]
