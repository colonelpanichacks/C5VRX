# USB preview protocol v1

**IMPLEMENTED; TYPES 1-5 PHYSICALLY EXERCISED, TYPE 6 AWAITS BOARD TEST**

The USB Serial/JTAG byte stream carries normal newline-delimited diagnostics
and binary preview packets together. A Receiver Console parser scans for the
eight-byte magic, validates both CRCs, and resumes scanning after damage. It
never uses a binary payload byte as a delimiter.

All integers are little-endian. Each packet is:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 8 | magic `00 43 35 56 52 58 A5 5A` |
| 8 | 1 | protocol version (`1`) |
| 9 | 1 | packet type |
| 10 | 2 | header bytes (`32`) |
| 12 | 4 | monotonically increasing packet sequence |
| 16 | 4 | payload bytes |
| 20 | 8 | device timestamp in microseconds |
| 28 | 4 | CRC-32/ISO-HDLC of header bytes 0..27 |
| 32 | N | payload |
| 32+N | 4 | CRC-32/ISO-HDLC of the payload |

Payloads are bounded to 1 MiB by the host parser. Unknown packet types with
valid framing can be skipped without losing synchronisation.

Packet types:

| Type | Name | Payload |
|---:|---|---|
| 1 | `STREAM_INFO` | eight-byte image descriptor |
| 2 | `GRAY8_FRAME` | descriptor followed by `stride * height` bytes |
| 3 | `STREAM_END` | 64-bit count of device-dropped frames |
| 4 | `IQ_U32_BLOCK` | legacy IQ descriptor plus packed 32-bit I/Q words |
| 5 | `IQ_U32_CHUNK` | finite raw-IQ capture fragment |
| 6 | `PHASE8_CHUNK` | finite unsigned phase-byte capture fragment |

The image descriptor is `width:u16, height:u16, stride:u16, pixel_format:u8,
flags:u8`. Pixel format 1 is GRAY8. Flag bit 0 means the frame was assembled
while the adaptive horizontal and vertical sync tracker was locked.

Types 5 and 6 share the 20-byte descriptor `capture_id:u32,
total_samples:u32, offset_samples:u32, chunk_samples:u32, flags:u32`. For type
5 each sample is one packed little-endian `uint32_t`; type 6 carries one byte
per sample. Flag bit 0 marks the first fragment and bit 1 the last. Each chunk
has independent framing and CRC, and the host accepts a capture only when all
offsets form one contiguous range of the declared total.

The firmware holds the C stdio lock across a complete packet. CRC and magic
recovery remain necessary because USB disconnect, reset or transport loss can
still leave a partial packet. `tools/c5vrx_usb_protocol.py --self-test` covers
arbitrary chunk boundaries, CRC rejection and recovery into the next packet.
