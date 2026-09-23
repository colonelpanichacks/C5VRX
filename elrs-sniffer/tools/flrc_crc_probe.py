#!/usr/bin/env python3
"""
flrc_crc_probe.py — identify the LIVE SX1280 FLRC CRC variant from captures
of a KNOWN-phrase FLRC link (round 6 verification path).

The seed story (CRC Initial Value regs 0x9C8/0x9C9 = OtaCrcInitializer,
DS_SX1280-1 Rev3.2 Table 14-12) is datasheet-confirmed, but the silicon
LFSR's bit-level details are not in the register tables — elrs_crc24flrc.h
implements 4 explicit variants. Capture sync_frame JSONs from a link whose
bind phrase you know (U <phrase> on the sniffer, or the stock default), then:

    python3 tools/flrc_crc_probe.py --phrase "ExpressLRS" capture.jsonl

Prints which variant(s) validate — that variant id is then trustworthy for
offline phrase cracking of UNKNOWN links (--frames mode of phrase_crack.py).
"""
import argparse
import hashlib
import json
import sys

POLY = {"0": 0xFFFF00, "1": 0xFFFF00, "2": 0xFFFF00, "3": 0x00FFFF}

def crc24(seed, data, variant):
    if variant == 1:
        st, poly = 0xFF0000 | seed, 0x5D6DCB
    elif variant == 3:
        st, poly = (seed << 8) & 0xFFFFFF, 0x5D6DCB
    else:
        st, poly = (seed << 8) & 0xFFFFFF, 0x5D6DCB
    for b in data:
        st ^= b << 16
        for _ in range(8):
            st = ((st << 1) ^ poly) & 0xFFFFFF if (st >> 23) & 1 else (st << 1) & 0xFFFFFF
    return st

def uid_for_phrase(p):
    return hashlib.md5(('-DMY_BINDING_PHRASE="%s"' % p).encode()).digest()[:6]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--phrase", required=True, help="bind phrase of the captured link")
    ap.add_argument("capture", help="sync_frame JSONL capture (file or - for stdin)")
    args = ap.parse_args()
    uid = uid_for_phrase(args.phrase)
    seed = (((uid[4] << 8) | uid[5]) ^ 3) & 0xFFFF
    print("uid %s -> expected CRC seed 0x%04x" % (uid.hex(), seed))
    hits = {v: 0 for v in range(4)}
    total = 0
    fh = open(args.capture) if args.capture != "-" else sys.stdin
    for line in fh:
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            d = json.loads(line)
        except ValueError:
            continue
        if d.get("t") != "sync_frame" or d.get("band") != "flrc":
            continue
        frame = bytes.fromhex(d.get("hex", ""))
        if len(frame) < 4:
            continue
        total += 1
        got_be = (frame[-3] << 16) | (frame[-2] << 8) | frame[-1]
        got_le = (frame[-1] << 16) | (frame[-2] << 8) | frame[-3]
        for v in range(3):
            if crc24(seed, frame[:-3], v) in (got_be, got_le):
                hits[v] += 1
    print("%d FLRC frames checked" % total)
    for v, n in hits.items():
        if n:
            print("VARIANT %d MATCHES %d frame(s)  <-- live variant" % (v, n))
    if not any(hits.values()):
        print("no variant matched — the CRC model needs more variants; share the capture")
        return 1
    return 0

if __name__ == "__main__":
    sys.exit(main())
