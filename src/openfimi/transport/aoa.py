"""USB gadget transport: impersonate the phone as an Android Open Accessory.

The RC is the USB *host* and the phone is a USB *device* that the RC switches
into AOA accessory mode.  To replace the phone, this machine must be a USB
device too, which needs a USB Device Controller: a Raspberry Pi Zero 2 W (OTG
port), Pi 4/5 (USB-C), or another board with dwc2/dwc3/musb in peripheral mode.
Desktop/laptop xHCI ports are host-only and cannot do this.

Implementation: Linux configfs + FunctionFS (requires root, ``libcomposite``).

1. A gadget with one vendor-specific interface (class 0xFF/0xFF/0, the same as
   Android's f_accessory) holding a bulk IN and a bulk OUT endpoint.
2. It enumerates as Google's accessory id 18d1:2d00 straight away, so a host
   that checks the id can skip the handshake.
3. If the host runs the AOA handshake anyway (GET_PROTOCOL 51, SEND_STRING 52,
   START 53) we answer it on ep0 and, after START, re-enumerate once, as a
   phone would.
4. After the bulk pipe comes up we write the single 0x00 byte the app writes
   on open, then exchange the 0xAE-framed stream.
"""

from __future__ import annotations

import errno
import logging
import os
import queue
import struct
import subprocess
import threading
import time
from pathlib import Path

from .base import Transport, TransportClosed

log = logging.getLogger(__name__)

CONFIGFS = Path("/sys/kernel/config/usb_gadget")
GOOGLE_VID = 0x18D1
AOA_PID = 0x2D00  # accessory
AOA_ADB_PID = 0x2D01  # accessory + adb

# AOA control requests (vendor, device recipient)
AOA_GET_PROTOCOL = 51
AOA_SEND_STRING = 52
AOA_START = 53
AOA_STRING_NAMES = ("manufacturer", "model", "description", "version", "uri", "serial")

# FunctionFS ABI (include/uapi/linux/usb/functionfs.h)
FFS_DESCRIPTORS_MAGIC_V2 = 3
FFS_STRINGS_MAGIC = 2
FFS_HAS_FS_DESC = 1
FFS_HAS_HS_DESC = 2
FFS_ALL_CTRL_RECIP = 64
FFS_CONFIG0_SETUP = 128
EV_BIND, EV_UNBIND, EV_ENABLE, EV_DISABLE, EV_SETUP, EV_SUSPEND, EV_RESUME = range(7)
EV_NAMES = ["BIND", "UNBIND", "ENABLE", "DISABLE", "SETUP", "SUSPEND", "RESUME"]
EVENT_SIZE = 12


def _interface_desc() -> bytes:
    # bLength, bDescriptorType=INTERFACE, bInterfaceNumber, bAlternateSetting,
    # bNumEndpoints, class, subclass, protocol, iInterface
    return bytes((9, 4, 0, 0, 2, 0xFF, 0xFF, 0x00, 1))


def _ep_desc(address: int, max_packet: int) -> bytes:
    return struct.pack("<BBBBHB", 7, 5, address, 0x02, max_packet, 0)


def ffs_descriptors() -> bytes:
    fs = _interface_desc() + _ep_desc(0x81, 64) + _ep_desc(0x02, 64)
    hs = _interface_desc() + _ep_desc(0x81, 512) + _ep_desc(0x02, 512)
    flags = FFS_HAS_FS_DESC | FFS_HAS_HS_DESC | FFS_ALL_CTRL_RECIP | FFS_CONFIG0_SETUP
    body = struct.pack("<II", 3, 3) + fs + hs
    return struct.pack("<III", FFS_DESCRIPTORS_MAGIC_V2, 12 + len(body), flags) + body


def ffs_strings(interface_name: str = "Android Accessory Interface") -> bytes:
    body = struct.pack("<IIH", 1, 1, 0x0409) + interface_name.encode() + b"\0"
    return struct.pack("<II", FFS_STRINGS_MAGIC, 8 + len(body)) + body


def _w(path: Path, value: str) -> None:
    path.write_text(value)


class GadgetConfig:
    """Creates/removes the configfs gadget and the FunctionFS mount."""

    def __init__(
        self,
        name: str = "openfimi",
        udc: str | None = None,
        manufacturer: str = "openfimi",
        product: str = "openfimi accessory",
        serial: str = "0123456789",
        vid: int = GOOGLE_VID,
        pid: int = AOA_PID,
    ):
        self.name = name
        self.udc = udc
        self.manufacturer, self.product, self.serial = manufacturer, product, serial
        self.vid, self.pid = vid, pid
        self.gadget = CONFIGFS / name
        self.mount = Path(f"/dev/ffs-{name}")

    @staticmethod
    def available_udcs() -> list[str]:
        p = Path("/sys/class/udc")
        return sorted(x.name for x in p.iterdir()) if p.exists() else []

    def create(self) -> None:
        if not CONFIGFS.exists():
            subprocess.run(["modprobe", "libcomposite"], check=False)
        if not CONFIGFS.exists():
            raise RuntimeError("configfs usb_gadget missing: mount configfs and load libcomposite")
        if not self.available_udcs():
            raise RuntimeError(
                "no USB device controller (/sys/class/udc is empty). On a Pi add "
                "'dtoverlay=dwc2' to config.txt and 'modules-load=dwc2' to cmdline.txt"
            )
        g = self.gadget
        if g.exists():
            self.destroy()
        g.mkdir()
        _w(g / "idVendor", f"0x{self.vid:04x}")
        _w(g / "idProduct", f"0x{self.pid:04x}")
        _w(g / "bcdDevice", "0x0100")
        _w(g / "bcdUSB", "0x0200")
        s = g / "strings/0x409"
        s.mkdir(parents=True)
        _w(s / "manufacturer", self.manufacturer)
        _w(s / "product", self.product)
        _w(s / "serialnumber", self.serial)
        c = g / "configs/c.1"
        c.mkdir(parents=True)
        (c / "strings/0x409").mkdir(parents=True)
        _w(c / "strings/0x409/configuration", "accessory")
        _w(c / "MaxPower", "100")
        f = g / f"functions/ffs.{self.name}"
        f.mkdir(parents=True)
        os.symlink(f, c / f"ffs.{self.name}")
        self.mount.mkdir(exist_ok=True)
        subprocess.run(["mount", "-t", "functionfs", self.name, str(self.mount)], check=True)

    def bind(self) -> None:
        udc = self.udc or self.available_udcs()[0]
        _w(self.gadget / "UDC", udc)

    def unbind(self) -> None:
        try:
            _w(self.gadget / "UDC", "\n")
        except OSError:
            pass

    def set_pid(self, pid: int) -> None:
        _w(self.gadget / "idProduct", f"0x{pid:04x}")

    def speed(self) -> str:
        udc = self.udc or (self.available_udcs() or [""])[0]
        try:
            return Path(f"/sys/class/udc/{udc}/current_speed").read_text().strip()
        except OSError:
            return "UNKNOWN"

    def destroy(self) -> None:
        g = self.gadget
        if not g.exists():
            return
        self.unbind()
        subprocess.run(["umount", str(self.mount)], check=False, stderr=subprocess.DEVNULL)
        c = g / "configs/c.1"
        for link in c.glob("ffs.*"):
            link.unlink()
        for d in (c / "strings/0x409", c, *(g / "functions").glob("*"), g / "strings/0x409", g):
            try:
                d.rmdir()
            except OSError:
                pass


class AoaGadgetTransport(Transport):
    """The phone side of the RC's USB link, on a gadget-capable board.

    Must run as root.  ``open()`` sets up the gadget and returns once the bulk
    endpoints are live (the RC has configured us); ``read``/``write`` raise
    TransportClosed if the RC disconnects.
    """

    def __init__(
        self,
        config: GadgetConfig | None = None,
        *,
        send_hello: bool = True,
        reenumerate_on_start: bool = True,
        enable_timeout: float | None = None,
    ):
        self.cfg = config or GadgetConfig()
        self.name = f"aoa-gadget:{self.cfg.name}"
        self.send_hello = send_hello
        self.reenumerate_on_start = reenumerate_on_start
        self.enable_timeout = enable_timeout
        self.accessory_strings: dict[str, str] = {}
        self.events: list[str] = []
        self._ep0 = self._in = self._out = -1
        self._enabled = threading.Event()
        self._closing = False
        self._reenumerated = False
        self._ep0_thread: threading.Thread | None = None
        self._write_lock = threading.Lock()
        self._max_packet = 512
        self._rxq: queue.Queue[bytes | None] = queue.Queue()

    # -- setup ---------------------------------------------------------------
    def open(self) -> None:
        if os.geteuid() != 0:
            raise PermissionError("the USB gadget transport must run as root")
        self.cfg.create()
        ep0 = self.cfg.mount / "ep0"
        self._ep0 = os.open(ep0, os.O_RDWR)
        os.write(self._ep0, ffs_descriptors())
        os.write(self._ep0, ffs_strings())
        self._in = os.open(self.cfg.mount / "ep1", os.O_RDWR)
        self._out = os.open(self.cfg.mount / "ep2", os.O_RDWR)
        self._ep0_thread = threading.Thread(target=self._ep0_loop, name="aoa-ep0", daemon=True)
        self._ep0_thread.start()
        threading.Thread(target=self._rx_loop, name="aoa-rx", daemon=True).start()
        self.cfg.bind()
        log.info(
            "gadget bound to %s, waiting for the RC to configure it",
            self.cfg.udc or self.cfg.available_udcs()[0],
        )
        if not self._enabled.wait(self.enable_timeout):
            raise TimeoutError("RC did not configure the gadget (is the cable in the RC's port?)")

    def _event(self, text: str) -> None:
        log.info("ep0: %s", text)
        self.events.append(f"{time.time():.3f} {text}")

    def _ep0_loop(self) -> None:
        while not self._closing:
            try:
                buf = os.read(self._ep0, EVENT_SIZE * 4)
            except OSError as e:
                if self._closing:
                    return
                if e.errno in (errno.EINTR, errno.EAGAIN):
                    continue
                log.error("ep0 read failed: %s", e)
                return
            for i in range(0, len(buf) - EVENT_SIZE + 1, EVENT_SIZE):
                ev = buf[i : i + EVENT_SIZE]
                etype = ev[8]
                if etype == EV_SETUP:
                    self._setup(*struct.unpack_from("<BBHHH", ev, 0))
                    continue
                self._event(EV_NAMES[etype] if etype < len(EV_NAMES) else f"EV{etype}")
                if etype == EV_ENABLE:
                    if "high" in self.cfg.speed():
                        self._max_packet = 512
                    else:
                        self._max_packet = 64
                    self._enabled.set()
                    if self.send_hello:
                        threading.Thread(target=self._hello, daemon=True).start()
                elif etype in (EV_DISABLE, EV_UNBIND):
                    self._enabled.clear()

    def _hello(self) -> None:
        try:
            self.write(b"\x00")
        except (TransportClosed, OSError) as e:
            log.warning("hello byte not sent: %s", e)

    def _setup(self, rtype: int, req: int, value: int, index: int, length: int) -> None:
        is_in = bool(rtype & 0x80)
        self._event(
            f"SETUP type=0x{rtype:02x} req={req} value=0x{value:04x} index={index} len={length}"
        )
        try:
            if (rtype & 0x60) == 0x40 and req == AOA_GET_PROTOCOL and is_in:
                os.write(self._ep0, struct.pack("<H", 2)[:length])
            elif (rtype & 0x60) == 0x40 and req == AOA_SEND_STRING and not is_in:
                data = os.read(self._ep0, length) if length else b""
                key = AOA_STRING_NAMES[index] if index < len(AOA_STRING_NAMES) else str(index)
                self.accessory_strings[key] = data.rstrip(b"\0").decode("utf-8", "replace")
                self._event(f"AOA string {key} = {self.accessory_strings[key]!r}")
            elif (rtype & 0x60) == 0x40 and req == AOA_START and not is_in:
                os.read(self._ep0, length)
                if self.reenumerate_on_start and not self._reenumerated:
                    self._reenumerated = True
                    threading.Thread(target=self._reenumerate, daemon=True).start()
            elif is_in:
                os.read(self._ep0, 0)  # wrong-direction op = STALL
            else:
                if length:
                    os.read(self._ep0, length)  # accept and ignore unknown OUT data
                else:
                    os.read(self._ep0, 0)
        except OSError as e:
            if e.errno != errno.EL2HLT:  # EL2HLT = the stall we asked for
                log.warning("ep0 setup handling failed: %s", e)

    def _reenumerate(self) -> None:
        self._event("re-enumerating after AOA START")
        time.sleep(0.05)
        self.cfg.unbind()
        time.sleep(0.5)
        self.cfg.bind()

    # -- data ----------------------------------------------------------------
    def _rx_loop(self) -> None:
        # Endpoint files do not support poll(), so block in a thread.
        while not self._closing:
            if not self._enabled.wait(0.5):
                continue
            try:
                data = os.read(self._out, 16384)
            except OSError as e:
                if self._closing:
                    break
                if e.errno in (errno.ESHUTDOWN, errno.ECONNRESET, errno.EINTR, errno.EAGAIN):
                    time.sleep(0.05)  # disabled or re-enumerating
                    continue
                log.error("bulk OUT read failed: %s", e)
                break
            if data:
                self._rxq.put(data)
        self._rxq.put(None)

    def read(self, timeout: float | None = None) -> bytes:
        try:
            data = self._rxq.get(timeout=timeout)
        except queue.Empty:
            return b""
        if data is None:
            self._rxq.put(None)
            raise TransportClosed(self.name)
        return data

    def write(self, data: bytes) -> None:
        if self._closing:
            raise TransportClosed(self.name)
        if not self._enabled.wait(2.0):
            raise TransportClosed("USB link not configured")
        with self._write_lock:
            try:
                os.write(self._in, data)
                if len(data) % self._max_packet == 0:
                    os.write(self._in, b"")  # zero-length packet ends the transfer
            except OSError as e:
                raise TransportClosed(str(e)) from e

    def close(self) -> None:
        self._closing = True
        for fd in (self._in, self._out, self._ep0):
            if fd >= 0:
                try:
                    os.close(fd)
                except OSError:
                    pass
        self.cfg.destroy()
