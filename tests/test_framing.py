import random

from openfimi.framing import (
    Frame,
    InnerDecoder,
    OuterDecoder,
    StreamType,
    decode_inner,
    encode_inner,
    encode_outer,
    encode_wire,
)
from openfimi.modules import Module


def test_inner_roundtrip_and_header_fields():
    payload = bytes([3, 16, 0, 0])
    raw = encode_inner(Module.GCS, Module.FC, 1234, payload)
    assert raw[0] == 0xFE
    assert len(raw) == 20
    verlen = raw[1] | raw[2] << 8
    assert verlen & 0x1F == 4 and verlen >> 6 == 20
    assert raw[3] == 1 and raw[4] == 7 and raw[5] == 2
    assert raw[8] | raw[9] << 8 == 1234
    f = decode_inner(raw)
    assert f == Frame(Module.GCS, Module.FC, 1234, payload)
    assert (f.group, f.msg_id) == (3, 16)


def test_corrupt_payload_rejected():
    raw = bytearray(encode_inner(7, 2, 1, bytes(range(10))))
    raw[-1] ^= 0xFF
    assert decode_inner(bytes(raw)) is None
    assert decode_inner(bytes(raw), verify=False).crc_ok is False


def test_outer_header():
    inner = encode_inner(7, 2, 5, b"\x03\x15\x00\x00")
    out = encode_outer(inner)
    assert out[0] == 0xAE
    total = ((out[1] | out[2] << 8) >> 4) & 0xFFF
    assert out[1] & 0x0F == 1 and total == len(out)
    assert out[3] == 0
    assert out[4] == sum(out[:4]) & 0xFF


def test_stream_decoding_with_noise_and_arbitrary_chunking():
    rng = random.Random(1)
    frames = [
        Frame(Module.FC, Module.GCS, i, bytes([12, 2, 0, 0]) + rng.randbytes(rng.randint(0, 60)))
        for i in range(50)
    ]
    stream = bytearray()
    for f in frames:
        stream += rng.randbytes(rng.randint(0, 5)).replace(b"\xae", b"\x00")  # junk between
        stream += encode_wire(f)
    video = encode_outer(b"\x80" + bytes(30), StreamType.VIDEO)
    stream += video
    outer, inner = OuterDecoder(), InnerDecoder()
    got, types = [], []
    i = 0
    while i < len(stream):
        n = rng.randint(1, 40)
        for t, body in outer.feed(bytes(stream[i : i + n])):
            types.append(t)
            if t == StreamType.FMLINK:
                got += inner.feed(body)
        i += n
    assert got == frames
    assert types.count(StreamType.VIDEO) == 1


def test_inner_frames_split_across_outer_records():
    a = encode_inner(2, 7, 1, b"\x0c\x01\x00\x00" + bytes(13))
    b = encode_inner(2, 7, 2, b"\x0c\x05\x00\x00" + bytes(24))
    blob = a + b
    inner = InnerDecoder()
    out = inner.feed(blob[:7]) + inner.feed(blob[7:30]) + inner.feed(blob[30:])
    assert [f.seq for f in out] == [1, 2]
