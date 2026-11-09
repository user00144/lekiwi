"""AWR2944 point/SNR TLVs; TLV length excludes its eight-byte header."""

import struct

MAGIC = b"\x02\x01\x04\x03\x06\x05\x08\x07"
HEADER_SIZE = 40
POINT_TLV = 1
SIDE_INFO_TLV = 7


def parse_header(packet: bytes) -> tuple[int, int, int]:
    fields = struct.unpack_from("<8s8I", packet)
    return fields[4], fields[6], fields[7]  # frame, object count, TLV count


def packet_is_valid(packet: bytes) -> bool:
    if len(packet) < HEADER_SIZE or packet[:8] != MAGIC:
        return False
    if struct.unpack_from("<I", packet, 12)[0] != len(packet):
        return False
    _, count, num_tlvs = parse_header(packet)
    if num_tlvs > 32:
        return False
    offset = HEADER_SIZE
    for _ in range(num_tlvs):
        if offset + 8 > len(packet):
            return False
        kind, size = struct.unpack_from("<II", packet, offset)
        if kind == POINT_TLV and size != count * 16:
            return False
        if kind == SIDE_INFO_TLV and size != count * 4:
            return False
        offset += 8 + size
        if offset > len(packet):
            return False
    return len(packet) - offset < 32  # Firmware pads packets to 32-byte boundaries.


def parse_packet(packet: bytes, count: int, num_tlvs: int) -> tuple[list, list]:
    """Decode a validated packet into x/y/z/doppler and SNR in dB."""
    offset = HEADER_SIZE
    points, snr = [], []
    for _ in range(num_tlvs):
        kind, size = struct.unpack_from("<II", packet, offset)
        offset += 8
        payload = packet[offset:offset + size]
        if kind == POINT_TLV:
            points = list(struct.iter_unpack("<4f", payload))
        elif kind == SIDE_INFO_TLV:
            snr = [value / 10 for value, _noise in struct.iter_unpack("<2h", payload)]
        offset += size
    if len(snr) != len(points):
        snr = [None] * len(points)
    return points, snr
