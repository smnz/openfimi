from pathlib import Path

from openfimi.crc import crc16_mcrf4xx, crc32_fimi

VECTORS = Path(__file__).parent / "data" / "crc_vectors.txt"


def _vectors():
    for line in VECTORS.read_text().splitlines():
        if line.startswith("check"):
            continue
        parts = line.split(" ")
        if len(parts) == 2:  # empty payload: hex field is blank
            parts = ["", *parts]
        yield bytes.fromhex(parts[0]), int(parts[1]), int(parts[2])


def test_crc_vectors_match_app_java_code():
    n = 0
    for data, c32, c16 in _vectors():
        assert crc32_fimi(data) == c32, data.hex()
        assert crc16_mcrf4xx(data) == c16, data.hex()
        n += 1
    assert n == 40


def test_crc16_check_value():
    assert crc16_mcrf4xx(b"123456789") == 0x6F91
