"""Wire framing: the outer USB wrapper and the inner FmLink4 frame.

On the RC (AOA) and Wi-Fi links every message is wrapped twice::

    0xAE | ver/len | TYPE | sum8 | <inner>              outer "UsbLinkPacket"
    0xFE | ver/len | flags | src | dst | r2 | r3 | seq | crc16 | crc32 | payload
                                                       inner "FmLink4"

The outer TYPE selects the stream: 0 = commands/telemetry (an FmLink4 frame
follows), 2 = live video (RTP), 6/7 = firmware/media, 11 = 4G video.  There is
no byte stuffing; both layers resynchronise by scanning for their start byte
and validating length and checksum.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from enum import IntEnum

from .crc import additive8, crc16_mcrf4xx, crc32_fimi
from .modules import Module

OUTER_SYNC = 0xAE
INNER_SYNC = 0xFE
INNER_VERSION = 4
INNER_HEADER_LEN = 16
INNER_MAX_LEN = 0x1FF  # 9-bit total length, header included
OUTER_MAX_LEN = 8692  # receive-side sanity limit used by the app


class StreamType(IntEnum):
    """Outer-wrapper TYPE byte (receive-side routing)."""

    FMLINK = 0
    VIDEO = 2
    NOTICE = 5
    FW_UPLOAD = 6
    MEDIA = 7
    VIDEO_4G = 11


@dataclass
class Frame:
    """A decoded or to-be-encoded FmLink4 frame."""

    src: int
    dst: int
    seq: int
    payload: bytes
    flags: int = 0x01
    reserve2: int = 0
    reserve3: int = 0
    crc_ok: bool = field(default=True, compare=False)

    # Payload header accessors -------------------------------------------------
    @property
    def group(self) -> int:
        """payload[0]: the command set ("cmdset" / "groupID")."""
        return self.payload[0] if self.payload else -1

    @property
    def msg_id(self) -> int:
        """payload[1]: the command / message id."""
        return self.payload[1] if len(self.payload) > 1 else -1

    @property
    def version(self) -> int:
        return self.payload[2] & 0x0F if len(self.payload) > 2 else 0

    @property
    def report(self) -> int:
        """12-bit result field in payload[2..3]; on ACKs it is the status code."""
        if len(self.payload) < 4:
            return 0
        return (self.payload[3] << 4) | (self.payload[2] >> 4)

    @property
    def body(self) -> bytes:
        """Message body (payload after the 4-byte payload header)."""
        return self.payload[4:]

    @property
    def key(self) -> tuple[int, int, int]:
        """Demux key (src, group, msg_id)."""
        return (self.src, self.group, self.msg_id)

    def describe(self) -> str:
        return (
            f"{Module.name_of(self.src)}->{Module.name_of(self.dst)} "
            f"seq={self.seq} {self.group}/{self.msg_id} rpt={self.report} "
            f"len={len(self.payload)} {self.payload[4:].hex()}"
        )

    # Encoding -----------------------------------------------------------------
    def encode(self) -> bytes:
        return encode_inner(
            self.src,
            self.dst,
            self.seq,
            self.payload,
            flags=self.flags,
            reserve2=self.reserve2,
            reserve3=self.reserve3,
        )


def encode_inner(
    src: int,
    dst: int,
    seq: int,
    payload: bytes,
    *,
    flags: int = 0x01,
    reserve2: int = 0,
    reserve3: int = 0,
) -> bytes:
    total = len(payload) + INNER_HEADER_LEN
    if total > INNER_MAX_LEN:
        raise ValueError(f"payload too long for one FmLink4 frame ({len(payload)} B)")
    verlen = (INNER_VERSION & 0x1F) | ((total & 0x1FF) << 6)
    head = struct.pack(
        "<BHBBBBBH",
        INNER_SYNC,
        verlen,
        flags & 0xFF,
        src & 0xFF,
        dst & 0xFF,
        reserve2 & 0xFF,
        reserve3 & 0xFF,
        seq & 0xFFFF,
    )
    head += struct.pack("<HI", crc16_mcrf4xx(head), crc32_fimi(payload))
    return head + bytes(payload)


def encode_outer(inner: bytes, stream: int = StreamType.FMLINK) -> bytes:
    total = len(inner) + 5
    if total > 0xFFF:
        raise ValueError("outer frame too long for the 12-bit length field")
    b1 = 0x01 | ((total & 0x0F) << 4)
    b2 = (total >> 4) & 0xFF
    head = bytes((OUTER_SYNC, b1, b2, stream & 0xFF))
    return head + bytes((additive8(head),)) + bytes(inner)


def encode_wire(frame: Frame, stream: int = StreamType.FMLINK) -> bytes:
    """Fully framed bytes as written to the AOA endpoint or a UDP socket."""
    return encode_outer(frame.encode(), stream)


def decode_inner(data: bytes, *, verify: bool = True) -> Frame | None:
    """Decode one complete inner frame (must start at offset 0)."""
    if len(data) < INNER_HEADER_LEN or data[0] != INNER_SYNC:
        return None
    _, verlen, flags, src, dst, r2, r3, seq, _crc16, crc32 = struct.unpack_from("<BHBBBBBHHI", data)
    if verlen & 0x1F != INNER_VERSION:
        return None
    total = (verlen >> 6) & 0x1FF
    if total < INNER_HEADER_LEN or len(data) < total:
        return None
    payload = bytes(data[INNER_HEADER_LEN:total])
    ok = crc32_fimi(payload) == crc32
    if verify and not ok:
        return None
    return Frame(src, dst, seq, payload, flags, r2, r3, crc_ok=ok)


class OuterDecoder:
    """Streaming decoder for the 0xAE outer wrapper.

    Feed arbitrary byte chunks; returns ``(stream_type, body)`` records.
    Mirrors the app's NetDecoder, including its version-2 8-byte header.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.bad_checksums = 0
        self.skipped_bytes = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        out: list[tuple[int, bytes]] = []
        buf = self._buf
        buf += data
        i = 0
        n = len(buf)
        while True:
            j = buf.find(OUTER_SYNC, i)
            if j < 0:
                self.skipped_bytes += n - i
                i = n
                break
            self.skipped_bytes += j - i
            i = j
            if n - i < 5:
                break
            word = buf[i + 1] | (buf[i + 2] << 8)
            version = word & 0x0F
            if version == 1:
                hdr, total = 5, (word >> 4) & 0xFFF
            elif version == 2:
                if n - i < 8:
                    break
                hdr, total = 8, buf[i + 4] | (buf[i + 5] << 8)
            else:
                i += 1
                self.skipped_bytes += 1
                continue
            if not hdr <= total <= OUTER_MAX_LEN:
                i += 1
                self.skipped_bytes += 1
                continue
            if additive8(buf[i : i + hdr - 1]) != buf[i + hdr - 1]:
                self.bad_checksums += 1
                i += 1
                self.skipped_bytes += 1
                continue
            if n - i < total:
                break
            out.append((buf[i + 3], bytes(buf[i + hdr : i + total])))
            i += total
        del buf[:i]
        return out


class InnerDecoder:
    """Streaming decoder for FmLink4 frames (handles split or batched frames)."""

    def __init__(self, verify: bool = True) -> None:
        self._buf = bytearray()
        self.verify = verify
        self.bad_crc = 0

    def feed(self, data: bytes) -> list[Frame]:
        out: list[Frame] = []
        buf = self._buf
        buf += data
        i = 0
        n = len(buf)
        while True:
            j = buf.find(INNER_SYNC, i)
            if j < 0:
                i = n
                break
            i = j
            if n - i < 3:
                break
            verlen = buf[i + 1] | (buf[i + 2] << 8)
            total = (verlen >> 6) & 0x1FF
            if verlen & 0x1F != INNER_VERSION or total < INNER_HEADER_LEN:
                i += 1
                continue
            if n - i < total:
                break
            frame = decode_inner(buf[i : i + total], verify=False)
            if frame is None or (self.verify and not frame.crc_ok):
                self.bad_crc += 1
                i += 1
                continue
            out.append(frame)
            i += total
        del buf[:i]
        return out
