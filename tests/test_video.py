import struct

from openfimi.video import CODEC_H264, CODEC_H265, Depacketizer


def rtp(seq, ts, sub, flags, data, marker=False):
    return (
        struct.pack(">BBHII", 0x80, 0x60 | (0x80 if marker else 0), seq, ts, 1)
        + bytes((sub, flags))
        + data
    )


HEVC_VPS = b"\x00\x00\x00\x01\x40\x01\x0c"
H264_SPS = b"\x00\x00\x00\x01\x67\x42"


def test_reassembles_fragmented_access_unit():
    d = Depacketizer()
    out = d.feed(rtp(1, 100, 0x7C, 0x80, HEVC_VPS + b"A" * 10))
    out += d.feed(rtp(2, 100, 0x7C, 0x00, b"B" * 10))
    out += d.feed(rtp(3, 100, 0x7C, 0x40, b"C" * 10, marker=True))
    assert len(out) == 1
    assert out[0].data == HEVC_VPS + b"A" * 10 + b"B" * 10 + b"C" * 10
    assert out[0].codec == CODEC_H265 and out[0].pts == 100 and not out[0].lost


def test_detects_h264_and_loss():
    d = Depacketizer()
    out = d.feed(rtp(10, 5, 0x7C, 0x80, H264_SPS + b"x"))
    out += d.feed(rtp(12, 5, 0x7C, 0x40, b"y", marker=True))  # seq 11 missing
    assert out[0].codec == CODEC_H264 and out[0].lost


def test_timestamp_change_flushes_and_overlays_are_separate():
    d = Depacketizer()
    d.feed(rtp(1, 1, 0x7C, 0x80, HEVC_VPS))
    out = d.feed(rtp(2, 2, 0x7C, 0x08, b"rect"))
    assert len(out) == 2 and out[0].is_video and out[1].is_rect and out[1].data == b"rect"
