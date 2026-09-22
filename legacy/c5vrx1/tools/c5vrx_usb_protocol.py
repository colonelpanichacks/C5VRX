#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Versioned, CRC-protected C5VRX USB stream framing."""

from __future__ import annotations

import argparse
import struct
import zlib
from dataclasses import dataclass

MAGIC = b"\x00C5VRX\xA5\x5A"
VERSION = 1
HEADER = struct.Struct("<8sBBHIIQI")
HEADER_BYTES = HEADER.size
PAYLOAD_CRC = struct.Struct("<I")
FRAME_DESCRIPTOR = struct.Struct("<HHHBB")
IQ_DESCRIPTOR = struct.Struct("<II")  # word_count, flags
IQ_CHUNK_DESCRIPTOR = struct.Struct("<IIIII")
MAX_PAYLOAD_BYTES = 1024 * 1024

PACKET_STREAM_INFO = 1
PACKET_GRAY8_FRAME = 2
PACKET_STREAM_END = 3
PACKET_IQ_U32_BLOCK = 4
PACKET_IQ_U32_CHUNK = 5
PACKET_PHASE8_CHUNK = 6
PACKET_YUV411_FRAME = 7
PACKET_GRAY8_ROWS = 8
PIXEL_FORMAT_GRAY8 = 1
PIXEL_FORMAT_YUV411 = 2
IQ_FLAG_NONE = 0
GRAY8_ROW_MAX_RECORDS = 8


@dataclass(frozen=True)
class IQChunk:
    capture_id: int
    total_words: int
    offset_words: int
    flags: int
    words: tuple[int, ...]


@dataclass(frozen=True)
class Phase8Chunk:
    capture_id: int
    total_samples: int
    offset_samples: int
    flags: int
    phases: bytes


@dataclass(frozen=True)
class Packet:
    packet_type: int
    sequence: int
    timestamp_us: int
    payload: bytes


def yuv411_to_rgb(payload: bytes, width: int, height: int, stride: int) -> bytes:
    """Decode packed U-Y0-Y1-V-Y2-Y3 groups into RGB24."""
    if width <= 0 or height <= 0 or width % 4 or stride < width * 3 // 2:
        raise ValueError("invalid YUV411 geometry")
    if len(payload) != stride * height:
        raise ValueError("invalid YUV411 payload size")
    rgb = bytearray(width * height * 3)
    out = 0
    for row in range(height):
        line = payload[row * stride:(row + 1) * stride]
        for offset in range(0, width * 3 // 2, 6):
            u, y0, y1, v, y2, y3 = line[offset:offset + 6]
            d = u - 128
            e = v - 128
            for y in (y0, y1, y2, y3):
                c = max(0, y - 16)
                r = (298 * c + 409 * e + 128) >> 8
                g = (298 * c - 100 * d - 208 * e + 128) >> 8
                b = (298 * c + 516 * d + 128) >> 8
                rgb[out:out + 3] = bytes((
                    min(255, max(0, r)), min(255, max(0, g)),
                    min(255, max(0, b)),
                ))
                out += 3
    return bytes(rgb)


def encode_packet(packet_type: int, sequence: int, timestamp_us: int,
                  payload: bytes) -> bytes:
    """Reference encoder used by host tests and protocol tooling."""
    prefix = HEADER.pack(
        MAGIC, VERSION, packet_type, HEADER_BYTES, sequence,
        len(payload), timestamp_us, 0,
    )
    header_crc = zlib.crc32(prefix[:28]) & 0xFFFFFFFF
    header = prefix[:28] + struct.pack("<I", header_crc)
    return header + payload + PAYLOAD_CRC.pack(zlib.crc32(payload) & 0xFFFFFFFF)


def decode_iq_block(packet: Packet) -> list[int]:
    """Decode one PACKET_IQ_U32_BLOCK payload into packed C5 IQ words."""
    if packet.packet_type != PACKET_IQ_U32_BLOCK:
        raise ValueError("not an IQ block packet")
    if len(packet.payload) < IQ_DESCRIPTOR.size:
        raise ValueError("short IQ descriptor")
    word_count, flags = IQ_DESCRIPTOR.unpack_from(packet.payload)
    if flags != IQ_FLAG_NONE:
        raise ValueError(f"unsupported IQ flags: {flags}")
    raw = packet.payload[IQ_DESCRIPTOR.size:]
    if len(raw) != word_count * 4:
        raise ValueError(
            f"IQ payload size mismatch: words={word_count} bytes={len(raw)}"
        )
    if not word_count:
        return []
    return list(struct.unpack(f"<{word_count}I", raw))


def decode_iq_chunk(packet: Packet) -> IQChunk:
    """Decode one independently CRC-protected fragment of an IQ capture."""
    if packet.packet_type != PACKET_IQ_U32_CHUNK:
        raise ValueError("not an IQ chunk packet")
    if len(packet.payload) < IQ_CHUNK_DESCRIPTOR.size:
        raise ValueError("short IQ chunk descriptor")
    (capture_id, total_words, offset_words,
     chunk_words, flags) = IQ_CHUNK_DESCRIPTOR.unpack_from(packet.payload)
    raw = packet.payload[IQ_CHUNK_DESCRIPTOR.size:]
    if chunk_words == 0 or len(raw) != chunk_words * 4:
        raise ValueError(
            f"IQ chunk size mismatch: words={chunk_words} bytes={len(raw)}"
        )
    if offset_words > total_words or chunk_words > total_words - offset_words:
        raise ValueError(
            f"IQ chunk bounds invalid: offset={offset_words}"
            f" words={chunk_words} total={total_words}"
        )
    words = struct.unpack(f"<{chunk_words}I", raw)
    return IQChunk(capture_id, total_words, offset_words, flags, words)


def decode_phase8_chunk(packet: Packet) -> Phase8Chunk:
    """Decode one independently CRC-protected unsigned phase8 fragment."""
    if packet.packet_type != PACKET_PHASE8_CHUNK:
        raise ValueError("not a phase8 chunk packet")
    if len(packet.payload) < IQ_CHUNK_DESCRIPTOR.size:
        raise ValueError("short phase8 chunk descriptor")
    (capture_id, total_samples, offset_samples,
     chunk_samples, flags) = IQ_CHUNK_DESCRIPTOR.unpack_from(packet.payload)
    phases = packet.payload[IQ_CHUNK_DESCRIPTOR.size:]
    if chunk_samples == 0 or len(phases) != chunk_samples:
        raise ValueError(
            f"phase8 chunk size mismatch: samples={chunk_samples}"
            f" bytes={len(phases)}"
        )
    if (offset_samples > total_samples or
            chunk_samples > total_samples - offset_samples):
        raise ValueError(
            f"phase8 chunk bounds invalid: offset={offset_samples}"
            f" samples={chunk_samples} total={total_samples}"
        )
    return Phase8Chunk(
        capture_id, total_samples, offset_samples, flags, phases)


@dataclass(frozen=True)
class Gray8Row:
    row_index: int
    flags: int          # bit0 = last row of the frame; bit1 = grabber-locked
    pixels: bytes       # one full canvas row of GRAY8


def decode_gray8_rows(packet: Packet, row_bytes: int) -> list[Gray8Row]:
    """Decode one PACKET_GRAY8_ROWS payload into row records.

    Layout: u8 row_count, then row_count records of
    u8 row_index, u8 flags, row_bytes GRAY8 pixels (no padding).
    flags bit0 marks the last row of a frame (video row_bytes-height-1, or
    the snow frame's final row); flags bit1 (valid on bit0 rows) is the
    firmware's grabber-lock badge: set = real video, clear = snow/no-signal.
    """
    if packet.packet_type != PACKET_GRAY8_ROWS:
        raise ValueError("not a GRAY8 rows packet")
    if not packet.payload:
        raise ValueError("empty rows payload")
    count = packet.payload[0]
    if count == 0 or count > GRAY8_ROW_MAX_RECORDS:
        raise ValueError(f"invalid row_count: {count}")
    rec_bytes = 2 + row_bytes
    if len(packet.payload) != 1 + count * rec_bytes:
        raise ValueError(
            f"rows payload size mismatch: count={count} bytes={len(packet.payload)}"
        )
    rows = []
    for k in range(count):
        row_index, flags = packet.payload[1 + k * rec_bytes:1 + k * rec_bytes + 2]
        pixels = packet.payload[1 + k * rec_bytes + 2:1 + (k + 1) * rec_bytes]
        if len(pixels) != row_bytes:
            raise ValueError("truncated row record")
        rows.append(Gray8Row(row_index, flags, pixels))
    return rows


class StreamDecoder:
    """Incrementally separates ASCII console lines and binary packets.

    ``feed`` returns ordered ``("line", str)``, ``("packet", Packet)`` and
    ``("error", reason)`` events. A bad header or payload advances only far
    enough to search for the next magic marker, so a damaged or truncated
    packet cannot permanently desynchronise the Receiver Console.
    """

    def __init__(self) -> None:
        self._wire = bytearray()
        self._text = bytearray()

    @staticmethod
    def _magic_prefix_suffix(data: bytearray) -> int:
        for count in range(min(len(data), len(MAGIC) - 1), 0, -1):
            if data[-count:] == MAGIC[:count]:
                return count
        return 0

    def _consume_text(self, data: bytes, events: list[tuple[str, object]]) -> None:
        self._text.extend(data)
        while True:
            newline = self._text.find(b"\n")
            if newline < 0:
                break
            raw = bytes(self._text[:newline]).rstrip(b"\r")
            del self._text[:newline + 1]
            events.append(("line", raw.decode("utf-8", errors="replace")))
        if len(self._text) > 65536:
            self._text.clear()
            events.append(("error", "ASCII_BUFFER_LIMIT"))

    def feed(self, data: bytes) -> list[tuple[str, object]]:
        events: list[tuple[str, object]] = []
        self._wire.extend(data)
        while self._wire:
            marker = self._wire.find(MAGIC)
            if marker < 0:
                keep = self._magic_prefix_suffix(self._wire)
                text_bytes = len(self._wire) - keep
                if text_bytes:
                    self._consume_text(bytes(self._wire[:text_bytes]), events)
                    del self._wire[:text_bytes]
                break
            if marker:
                self._consume_text(bytes(self._wire[:marker]), events)
                del self._wire[:marker]
            if len(self._wire) < HEADER_BYTES:
                break

            header = bytes(self._wire[:HEADER_BYTES])
            (magic, version, packet_type, header_bytes, sequence,
             payload_bytes, timestamp_us, expected_header_crc) = HEADER.unpack(header)
            actual_header_crc = zlib.crc32(header[:28]) & 0xFFFFFFFF
            if (magic != MAGIC or version != VERSION or
                    header_bytes != HEADER_BYTES or
                    payload_bytes > MAX_PAYLOAD_BYTES or
                    actual_header_crc != expected_header_crc):
                del self._wire[0]
                events.append(("error", "INVALID_HEADER"))
                continue

            wire_bytes = HEADER_BYTES + payload_bytes + PAYLOAD_CRC.size
            if len(self._wire) < wire_bytes:
                break
            payload = bytes(self._wire[HEADER_BYTES:HEADER_BYTES + payload_bytes])
            expected_payload_crc, = PAYLOAD_CRC.unpack_from(
                self._wire, HEADER_BYTES + payload_bytes)
            actual_payload_crc = zlib.crc32(payload) & 0xFFFFFFFF
            if actual_payload_crc != expected_payload_crc:
                nested_marker = self._wire.find(MAGIC, 1, wire_bytes)
                if nested_marker >= 0:
                    del self._wire[:nested_marker]
                else:
                    del self._wire[:wire_bytes]
                events.append((
                    "error",
                    "PAYLOAD_CRC"
                    f" type={packet_type} sequence={sequence}"
                    f" bytes={payload_bytes}"
                    f" expected={expected_payload_crc:08x}"
                    f" actual={actual_payload_crc:08x}",
                ))
                continue
            del self._wire[:wire_bytes]
            events.append(("packet", Packet(
                packet_type=packet_type,
                sequence=sequence,
                timestamp_us=timestamp_us,
                payload=payload,
            )))
        return events


def self_test() -> None:
    descriptor = FRAME_DESCRIPTOR.pack(160, 120, 160, PIXEL_FORMAT_GRAY8, 1)
    frame = bytes((n * 17) & 0xFF for n in range(160 * 120))
    info = encode_packet(PACKET_STREAM_INFO, 3, 1000, descriptor)
    video = encode_packet(PACKET_GRAY8_FRAME, 4, 2000, descriptor + frame)
    iq_words = [0x2522304C, 0xB36DF7F4, 0x2520F45E]
    iq_payload = IQ_DESCRIPTOR.pack(len(iq_words), IQ_FLAG_NONE) + struct.pack(
        f"<{len(iq_words)}I", *iq_words
    )
    iq = encode_packet(PACKET_IQ_U32_BLOCK, 5, 2500, iq_payload)
    chunk_words = iq_words[:2]
    chunk_payload = IQ_CHUNK_DESCRIPTOR.pack(
        9, len(iq_words), 0, len(chunk_words), 1
    ) + struct.pack(f"<{len(chunk_words)}I", *chunk_words)
    iq_chunk = encode_packet(PACKET_IQ_U32_CHUNK, 6, 2600, chunk_payload)
    phase_payload = IQ_CHUNK_DESCRIPTOR.pack(10, 5, 0, 5, 3) + \
        bytes([0, 63, 127, 191, 255])
    phase_chunk = encode_packet(PACKET_PHASE8_CHUNK, 7, 2700, phase_payload)
    row_bytes = 16
    rows_payload = (
        bytes([2, 7, 0]) + bytes(range(row_bytes)) +
        bytes([8, 1]) + bytes(range(row_bytes, 2 * row_bytes))
    )
    rows = encode_packet(PACKET_GRAY8_ROWS, 8, 2800, rows_payload)
    yuv = bytes([128, 16, 64, 128, 128, 235])
    assert len(yuv411_to_rgb(yuv, 4, 1, 6)) == 12
    damaged = bytearray(video)
    damaged[-1] ^= 0x80
    end = encode_packet(PACKET_STREAM_END, 9, 3000, b"")
    wire = (b"boot\r\n" + info + bytes(damaged) + b"noise\n" + video +
            iq + iq_chunk + phase_chunk + rows + end)

    decoder = StreamDecoder()
    events: list[tuple[str, object]] = []
    for offset in range(0, len(wire), 37):
        events.extend(decoder.feed(wire[offset:offset + 37]))
    lines = [value for kind, value in events if kind == "line"]
    packets = [value for kind, value in events if kind == "packet"]
    errors = [value for kind, value in events if kind == "error"]
    assert "boot" in lines
    assert "noise" in lines
    assert any(str(error).startswith("PAYLOAD_CRC") for error in errors)
    assert [packet.packet_type for packet in packets] == [
        PACKET_STREAM_INFO, PACKET_GRAY8_FRAME, PACKET_IQ_U32_BLOCK,
        PACKET_IQ_U32_CHUNK, PACKET_PHASE8_CHUNK, PACKET_GRAY8_ROWS,
        PACKET_STREAM_END,
    ]
    assert packets[1].payload[FRAME_DESCRIPTOR.size:] == frame
    assert decode_iq_block(packets[2]) == iq_words
    assert list(decode_iq_chunk(packets[3]).words) == chunk_words
    assert decode_phase8_chunk(packets[4]).phases == bytes(
        [0, 63, 127, 191, 255])
    decoded_rows = decode_gray8_rows(packets[5], row_bytes)
    assert [row.row_index for row in decoded_rows] == [7, 8]
    assert [row.flags for row in decoded_rows] == [0, 1]
    assert decoded_rows[0].pixels == bytes(range(row_bytes))
    assert decoded_rows[1].pixels == bytes(range(row_bytes, 2 * row_bytes))
    print("c5vrx_usb_protocol: PASS")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
