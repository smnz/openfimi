"""Live FPV video: RTP depacketizing into Annex-B access units.

Video arrives on outer stream TYPE 2.  Each record is an RTP packet::

    [0]      0x80 (RTP v2)
    [1]      payload type, bit 7 = RTP marker
    [2..3]   sequence (big-endian)
    [4..7]   timestamp (big-endian), used as the PTS
    [8..11]  SSRC
    [12]     0x7C for a normal fragment; anything else = self-contained packet
    [13]     flags: 0x80 start of access unit, 0x40 end (with RTP marker),
             0x08 AI tracking rectangle, 0x09 AI detections
    [14..]   Annex-B NAL bytes

This is a faithful port of the app's ``FPVUnpack``.  The codec (H.264 or
H.265) is detected from the first NAL of each access unit; parameter sets are
in-band, so the output can be fed straight to ffmpeg or written to a file.
"""

from __future__ import annotations

import struct
import subprocess
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import BinaryIO

CODEC_H264 = 1
CODEC_H265 = 2

_H265_START_TYPES = {32, 33, 34, 39, 19, 1}


@dataclass
class VideoPacket:
    """One reassembled unit: an access unit, or an AI overlay record."""

    data: bytes
    pts: int
    codec: int = CODEC_H265
    lost: bool = False  # a sequence gap was seen while assembling it
    is_rect: bool = False  # AI tracking rectangle (not video)
    is_objs: bool = False  # AI object detections (not video)

    @property
    def is_video(self) -> bool:
        return not (self.is_rect or self.is_objs)

    @property
    def codec_name(self) -> str:
        return "h264" if self.codec == CODEC_H264 else "hevc"


@dataclass
class _Pending:
    codec: int
    buf: bytearray = field(default_factory=bytearray)
    pts: int = 0
    lost: bool = False


class Depacketizer:
    def __init__(self) -> None:
        self._seq_prev: int | None = None
        self._pts_prev = -1
        self._cur = _Pending(CODEC_H265)
        self.lost_packets = 0
        self.bad_packets = 0

    def _sniff(self, pkt: bytes) -> None:
        if len(pkt) < 19 or pkt[0] != 0x80 or pkt[14:18] != b"\x00\x00\x00\x01":
            return
        if pkt[18] & 0x1F == 7:
            self._cur.codec = CODEC_H264
        elif (pkt[18] & 0x7E) >> 1 in _H265_START_TYPES:
            self._cur.codec = CODEC_H265

    def _emit(self, out: list[VideoPacket], pend: _Pending) -> None:
        out.append(VideoPacket(bytes(pend.buf), pend.pts, pend.codec, pend.lost))

    def _new(self) -> None:
        self._cur = _Pending(self._cur.codec)

    def feed(self, pkt: bytes) -> list[VideoPacket]:
        out: list[VideoPacket] = []
        n = len(pkt)
        if n <= 14 or pkt[0] != 0x80:
            self.bad_packets += 1
            return out
        seq, pts = struct.unpack_from(">HI", pkt, 2)
        cur = self._cur
        if self._seq_prev is None:
            # Wait for the start of an access unit.
            if n >= 19 and pkt[13] == 0x80:
                cur.buf += pkt[14:]
                cur.pts = pts
                self._seq_prev, self._pts_prev = seq, pts
                self._sniff(pkt)
            return out

        if (self._seq_prev + 1) & 0xFFFF != seq:
            cur.lost = True
            self.lost_packets += 1
        if self._pts_prev != pts and cur.buf:
            # Timestamp changed before the end flag: flush what we have.
            self._emit(out, cur)
            self._new()
            cur = self._cur
        self._seq_prev, self._pts_prev = seq, pts

        sub, flags = pkt[12], pkt[13]
        if sub != 0x7C:
            # Self-contained packet: NAL data starts right after the RTP header.
            out.append(VideoPacket(bytes(pkt[12:]), pts, cur.codec))
            self._new()
        elif flags in (0x08, 0x09):
            if cur.buf:
                self._emit(out, cur)
                self._new()
            out.append(
                VideoPacket(
                    bytes(pkt[14:]),
                    pts,
                    self._cur.codec,
                    is_rect=flags == 0x08,
                    is_objs=flags == 0x09,
                )
            )
        elif flags & 0x80:
            cur.buf = bytearray(pkt[14:])
            cur.pts = pts
            self._sniff(pkt)
        elif not cur.buf:
            self.bad_packets += 1  # continuation without a start
        else:
            cur.buf += pkt[14:]
            if flags & 0x40 and pkt[1] & 0x80:
                cur.pts = pts
                self._emit(out, cur)
                self._new()
        return out


class AnnexBWriter:
    """Write video access units to a raw .h264/.h265 elementary stream."""

    def __init__(self, fp: BinaryIO) -> None:
        self.fp = fp

    def __call__(self, pkt: VideoPacket) -> None:
        if pkt.is_video:
            self.fp.write(pkt.data)
            self.fp.flush()


class FfmpegSink:
    """Pipe access units into an ffmpeg/ffplay process.

    ``FfmpegSink.ffplay()`` shows a low-latency window;
    ``FfmpegSink.ffmpeg("out.mp4")`` remuxes to a file without re-encoding.
    """

    def __init__(self, argv: list[str]) -> None:
        self.proc = subprocess.Popen(argv, stdin=subprocess.PIPE)

    @classmethod
    def ffplay(cls, codec: str = "hevc") -> FfmpegSink:
        return cls(
            [
                "ffplay",
                "-loglevel",
                "warning",
                "-fflags",
                "nobuffer",
                "-flags",
                "low_delay",
                "-framedrop",
                "-f",
                codec,
                "-i",
                "-",
            ]
        )

    @classmethod
    def ffmpeg(cls, path: str, codec: str = "hevc", fps: int = 30) -> FfmpegSink:
        return cls(
            [
                "ffmpeg",
                "-loglevel",
                "warning",
                "-y",
                "-f",
                codec,
                "-r",
                str(fps),
                "-i",
                "-",
                "-c",
                "copy",
                path,
            ]
        )

    def __call__(self, pkt: VideoPacket) -> None:
        if pkt.is_video and self.proc.stdin and self.proc.poll() is None:
            try:
                self.proc.stdin.write(pkt.data)
                self.proc.stdin.flush()
            except BrokenPipeError:
                pass

    def close(self) -> None:
        if self.proc.stdin:
            self.proc.stdin.close()
        self.proc.wait(timeout=5)


VideoCallback = Callable[[VideoPacket], None]
