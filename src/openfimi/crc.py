"""Checksums used by the FIMI link.

Two different CRCs protect an inner (FmLink4) frame:

* the 16-byte header carries a CRC-16/MCRF4XX (reflected 0x1021, i.e. 0x8408,
  init 0xFFFF; avr-libc's ``_crc_ccitt_update``) over its first 10 bytes;
* the payload carries a CRC-32/MPEG-2 computed over the payload with every
  aligned 4-byte word byte-swapped and the tail zero-padded to 4 bytes.

The outer USB wrapper only uses an 8-bit additive checksum (see framing.py).
"""

from __future__ import annotations


def crc16_mcrf4xx(data: bytes) -> int:
    """CRC-16/MCRF4XX: reflected poly 0x8408, init 0xFFFF, no xorout (check 0x6F91)."""
    crc = 0xFFFF
    for b in data:
        x = b ^ (crc & 0xFF)
        x = (x ^ (x << 4)) & 0xFF
        crc = ((crc >> 8) ^ (x << 8) ^ (x << 3) ^ (x >> 4)) & 0xFFFF
    return crc


def _make_table() -> list[int]:
    table = []
    for i in range(256):
        c = i << 24
        for _ in range(8):
            c = ((c << 1) ^ 0x04C11DB7) if c & 0x80000000 else (c << 1)
        table.append(c & 0xFFFFFFFF)
    return table


_TABLE = _make_table()


def crc32_fimi(data: bytes) -> int:
    """The frame CRC: MPEG-2 CRC-32 over word-swapped, zero-padded payload."""
    rem = len(data) % 4
    if rem:
        data = bytes(data) + bytes(4 - rem)
    crc = 0xFFFFFFFF
    t = _TABLE
    for i in range(0, len(data), 4):
        # Each little-endian word is fed most-significant byte first.
        for b in (data[i + 3], data[i + 2], data[i + 1], data[i]):
            crc = ((crc << 8) & 0xFFFFFFFF) ^ t[((crc >> 24) ^ b) & 0xFF]
    return crc


def additive8(data: bytes) -> int:
    """8-bit additive checksum used by the outer USB wrapper header."""
    return sum(data) & 0xFF
