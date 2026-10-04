"""The link layer: framing, sequence numbers, ACK matching, retransmission, demux.

A :class:`Link` owns a transport and two threads: a receiver that decodes the
byte stream into frames and video, and a retransmitter that resends
unacknowledged commands (500 ms x 5, as the app does).  ACKs are matched on
``(group, msg_id, seq)``.
"""

from __future__ import annotations

import concurrent.futures as cf
import json
import logging
import random
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field

from . import telemetry
from .commands import Command
from .framing import Frame, InnerDecoder, OuterDecoder, StreamType, encode_inner, encode_outer
from .modules import Module
from .transport.base import Transport, TransportClosed
from .video import Depacketizer, VideoPacket

log = logging.getLogger(__name__)

SEQ_WRAP = 32766


class AckTimeout(TimeoutError):
    def __init__(self, cmd: Command):
        super().__init__(f"no reply to {cmd}")
        self.command = cmd


@dataclass
class Reply:
    """The aircraft's answer to a command."""

    command: Command
    frame: Frame
    message: object | None = None  # decoded body, when a decoder exists

    @property
    def code(self) -> int:
        """Result code (payload report field). 0 appears to mean success."""
        return self.frame.report

    @property
    def ok(self) -> bool:
        return self.code == 0

    def __str__(self) -> str:
        return f"{self.command.name}: {'OK' if self.ok else f'code {self.code}'}"


@dataclass
class _Pending:
    cmd: Command
    wire: bytes
    future: cf.Future
    sent_at: float
    tries: int = 1


@dataclass
class LinkStats:
    rx_bytes: int = 0
    tx_bytes: int = 0
    frames: int = 0
    video_packets: int = 0
    acks: int = 0
    retransmits: int = 0
    timeouts: int = 0
    decode_errors: int = 0
    stream_types: dict[int, int] = field(default_factory=dict)


FrameCallback = Callable[[Frame], None]
MessageCallback = Callable[[object, Frame], None]
VideoCallback = Callable[[VideoPacket], None]
StreamCallback = Callable[[int, bytes], None]
NoticeCallback = Callable[[dict], None]


class Link:
    def __init__(self, transport: Transport, *, verify_crc: bool = True) -> None:
        self.transport = transport
        self.stats = LinkStats()
        self._outer = OuterDecoder()
        self._inner = InnerDecoder(verify=verify_crc)
        self._video = Depacketizer()
        # Random start, so several clients sharing a bridge rarely collide on
        # (group, msg_id, seq) when matching replies.
        self._seq = random.randrange(SEQ_WRAP)
        self._seq_lock = threading.Lock()
        self._pending: dict[tuple[int, int, int], _Pending] = {}
        self._pending_lock = threading.Lock()
        self._frame_cbs: list[FrameCallback] = []
        self._msg_cbs: list[MessageCallback] = []
        self._video_cbs: list[VideoCallback] = []
        self._stream_cbs: list[StreamCallback] = []
        self._notice_cbs: list[NoticeCallback] = []
        self._threads: list[threading.Thread] = []
        self._running = False
        self.closed = threading.Event()
        self.last_rx: float = 0.0
        self.error: Exception | None = None

    # -- lifecycle -------------------------------------------------------------
    def start(self) -> Link:
        self.transport.open()
        self._running = True
        for target, name in ((self._rx_loop, "openfimi-rx"), (self._retx_loop, "openfimi-retx")):
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def stop(self) -> None:
        self._running = False
        try:
            self.transport.close()
        finally:
            for t in self._threads:
                if t is not threading.current_thread():
                    t.join(timeout=2)
            with self._pending_lock:
                for p in self._pending.values():
                    if not p.future.done():
                        p.future.set_exception(TransportClosed("link stopped"))
                self._pending.clear()
            self.closed.set()

    def __enter__(self) -> Link:
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    # -- subscriptions -----------------------------------------------------------
    def on_frame(self, cb: FrameCallback) -> FrameCallback:
        """Every decoded FmLink frame, including ones not addressed to us."""
        self._frame_cbs.append(cb)
        return cb

    def on_message(self, cb: MessageCallback) -> MessageCallback:
        """Every frame for us that has a known decoder: cb(message, frame)."""
        self._msg_cbs.append(cb)
        return cb

    def on_video(self, cb: VideoCallback) -> VideoCallback:
        self._video_cbs.append(cb)
        return cb

    def on_notice(self, cb: NoticeCallback) -> NoticeCallback:
        """Bridge notices (outer type 0x40): cb(dict), e.g. the bridge app's
        emergency return-home button ``{"event": "emergency_rth", "stage": ...}``."""
        self._notice_cbs.append(cb)
        return cb

    def on_stream(self, cb: StreamCallback) -> StreamCallback:
        """Raw outer records of every stream type: cb(stream_type, body)."""
        self._stream_cbs.append(cb)
        return cb

    # -- sending -------------------------------------------------------------------
    def next_seq(self) -> int:
        with self._seq_lock:
            s = self._seq
            self._seq = (self._seq + 1) % SEQ_WRAP
            return s

    def send(self, cmd: Command) -> cf.Future:
        """Send a command. The future resolves to a :class:`Reply` (ack commands)
        or None (no-ack commands, once written)."""
        seq = self.next_seq()
        wire = encode_outer(
            encode_inner(cmd.src, cmd.dest, seq, cmd.payload, flags=cmd.header_flags)
        )
        fut: cf.Future = cf.Future()
        if cmd.ack:
            with self._pending_lock:
                self._pending[(cmd.group, cmd.msg_id, seq)] = _Pending(
                    cmd, wire, fut, time.monotonic()
                )
        try:
            self._write(wire)
        except TransportClosed as e:
            with self._pending_lock:
                self._pending.pop((cmd.group, cmd.msg_id, seq), None)
            fut.set_exception(e)
            return fut
        log.debug("tx seq=%d %s", seq, cmd)
        if not cmd.ack:
            fut.set_result(None)
        return fut

    def request(self, cmd: Command, timeout: float | None = None) -> Reply | None:
        """Send and wait for the reply. Raises AckTimeout after the retries."""
        fut = self.send(cmd)
        limit = timeout if timeout is not None else cmd.timeout * (cmd.retries + 1) + 1.0
        try:
            return fut.result(limit)
        except cf.TimeoutError:
            raise AckTimeout(cmd) from None

    def send_raw(self, wire: bytes) -> None:
        """Write already-framed bytes (for experiments and the bridge)."""
        self._write(wire)

    def _write(self, data: bytes) -> None:
        self.transport.write(data)
        self.stats.tx_bytes += len(data)

    # -- receiving -----------------------------------------------------------------
    def _rx_loop(self) -> None:
        try:
            while self._running:
                data = self.transport.read(0.25)
                if not data:
                    continue
                self.last_rx = time.monotonic()
                self.stats.rx_bytes += len(data)
                for stype, body in self._outer.feed(data):
                    self._on_record(stype, body)
        except TransportClosed as e:
            if self._running:
                log.warning("transport closed: %s", e)
                self.error = e
        except Exception as e:  # pragma: no cover - keep the reason visible
            log.exception("receiver crashed")
            self.error = e
        finally:
            self._running = False
            self.closed.set()

    def _on_record(self, stype: int, body: bytes) -> None:
        st = self.stats.stream_types
        st[stype] = st.get(stype, 0) + 1
        for cb in self._stream_cbs:
            self._safe(cb, stype, body)
        if stype == StreamType.FMLINK:
            for frame in self._inner.feed(body):
                self._on_frame(frame)
        elif stype == StreamType.BRIDGE_NOTICE:
            try:
                notice = json.loads(body.decode())
            except (UnicodeDecodeError, ValueError):
                log.debug("undecodable bridge notice %r", body[:80])
                return
            if isinstance(notice, dict):
                for cb in self._notice_cbs:
                    self._safe(cb, notice)
        elif stype == StreamType.VIDEO:
            for pkt in self._video.feed(body):
                self.stats.video_packets += 1
                for cb in self._video_cbs:
                    self._safe(cb, pkt)

    def _on_frame(self, frame: Frame) -> None:
        self.stats.frames += 1
        for cb in self._frame_cbs:
            self._safe(cb, frame)
        if frame.dst != Module.GCS:
            return
        try:
            msg = telemetry.decode(frame)
        except Exception as e:
            self.stats.decode_errors += 1
            log.debug("decode failed for %s: %s", frame.describe(), e)
            msg = None
        key = (frame.group, frame.msg_id, frame.seq)
        with self._pending_lock:
            pending = self._pending.pop(key, None)
        if pending is not None:
            self.stats.acks += 1
            if not pending.future.done():
                pending.future.set_result(Reply(pending.cmd, frame, msg))
        if msg is not None:
            for cb in self._msg_cbs:
                self._safe(cb, msg, frame)

    @staticmethod
    def _safe(cb, *args) -> None:
        try:
            cb(*args)
        except Exception:
            log.exception("callback %r failed", cb)

    # -- retransmission --------------------------------------------------------
    def _retx_loop(self) -> None:
        while self._running:
            time.sleep(0.05)
            now = time.monotonic()
            resend: list[_Pending] = []
            with self._pending_lock:
                for key, p in list(self._pending.items()):
                    if now - p.sent_at < p.cmd.timeout:
                        continue
                    if p.tries > p.cmd.retries:
                        del self._pending[key]
                        self.stats.timeouts += 1
                        if not p.future.done():
                            p.future.set_exception(AckTimeout(p.cmd))
                        continue
                    p.tries += 1
                    p.sent_at = now
                    resend.append(p)
            for p in resend:
                self.stats.retransmits += 1
                try:
                    self._write(p.wire)
                except TransportClosed:
                    break
