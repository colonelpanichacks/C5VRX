#!/usr/bin/env python3
"""
phrase_crack.py — offline ExpressLRS bind-phrase dictionary cracker (OSINT).

Given a UID (or UID tail) captured passively by the ELRS sniffer — the sync
packet leaks UID[3..5] in the clear — try to recover the BINDING PHRASE that
generated it, using the exact ELRS derivation:

    UID = md5(('-DMY_BINDING_PHRASE="<phrase>"').encode()).digest()[:6]

Verified against the known vector: "ExpressLRS" -> 43 7f 2f b1 d3 39.

Legal posture: this is an OFFLINE attack on YOUR OWN passive captures, used
to suggest a human-readable label for a detected link ("pilot alias"). The
binding phrase is anti-collision, not a secret — but only run this against
captures you are legally allowed to possess (see README "OSINT" section).

Usage:
    python3 phrase_crack.py 61ace1                    # tail (UID3..5)
    python3 phrase_crack.py 437f2fb1d339              # full UID
    python3 phrase_crack.py --uids captures.txt       # one UID per line
    python3 phrase_crack.py tail --wordlist words.txt # custom dictionary

Exit code 0 if any phrase matched, 1 otherwise.
"""
import argparse
import hashlib
import sys
import time

# ---------------------------------------------------------------------------
# ELRS derivation (src/python/binary_configurator.py + community converters)
def uid_for_phrase(phrase: str) -> bytes:
    return hashlib.md5(('-DMY_BINDING_PHRASE="%s"' % phrase).encode()).digest()[:6]

# ---------------------------------------------------------------------------
# Built-in dictionary: defaults, FPV vocabulary, and common first names.
DEFAULT_PHRASES = ["ExpressLRS", "expresslrs", "default", "elrs", "bind", "password"]

FPV_WORDS = [
    "freestyle", "cinewhoop", "tinyhawk", "mobula", "drone", "fpv", "racing",
    "quad", "quadcopter", "whoop", "toothpick", "longrange", "longrange2024",
    "dji", "hdzero", "walksnail", "betafpv", "iflight", "geprc", "diatone",
    "tbs", "teamblacksheep", "radiomaster", "jumper", "tx16s", "pocket",
    "zoro", "nano", "micro", "5inch", "5in", "3inch", "rooster", "mongoose",
    "lumenier", "qav", "sourceone", "source", "apex", "sl5", "knight",
    "caddx", "runcam", "foxeer", "tmotor", "emma", "holybro", "siyi",
]

FIRST_NAMES = """
james mary robert patricia john jennifer michael linda david elizabeth
william barbara richard susan joseph jessica thomas sarah charles karen
christopher lisa daniel nancy matthew betty anthony margaret mark sandra
donald ashley steven kimberly paul emily andrew michelle joshua amanda
kevin melissa brian deborah brian stephanie george laura timothy heather
ronald angela jason michelle edward abigail ryan denise gary fiona
nicholas julie erica alexandra adam olivia nathan carol ethan janet
juan maria carlos rosa luis ana diego laura jose carmen miguel elena
alex sofia lucas valentina max emma leon hannah finn lina ben marie
jan peter anna hans ulrike klaus birgit stefan sabine marco chiara
luca giulia lucia diego camila antonio isabella victoria martina
sophia amelia oliver harry jack noah charlie jacob thomas alfie
ruby chloe isla poppy daisy freya erin hollie bethany megan cerys
aaron abby adam adrian alan albert aileen alina amir amy andre
andrea angelo anita archie arthur asha ashley barry beatrice bernard
beverly blake bonnie brandon brenda brian bridget brooke bruce bryan
caleb cameron carla caroline catherine cecilia cedric celia chelsea
chester chloe christian christina cindy clara clarence claudia clifford
clint colin colton connor cora corey cornelius craig crystal curtis
cynthia dale damian daniela daphne darius dean deborah declan denise
dennis derek derrick desiree diana dominic donna dorian doris douglas
duane duncan dustin dwight dylan earl edgar edith edmund edna edwin
eileen elaine elbert elena eli elias elijah elisa ella ellen elmer
eloise elsa elvira emanuel emerson emilia emmanuel enrico enrique
eric erica erik erika ernest estelle esther ethel eugene eula eunice
evan evelyn everett faith felix fern fiona flora fletcher flora freda
""".split()

SUFFIXES = ["", "1", "123", "2023", "2024", "2025", "!"]

BUILTIN_WORDS = DEFAULT_PHRASES + FPV_WORDS + FIRST_NAMES


def gen_candidates(words):
    seen = set()
    for w in words:
        w = w.strip()
        if not w:
            continue
        for s in SUFFIXES:
            p = w + s
            if p not in seen:
                seen.add(p)
                yield p


def parse_uid(text):
    h = text.strip().replace(" ", "").replace("0x", "").lower()
    if not h or len(h) % 2 or any(c not in "0123456789abcdef" for c in h):
        raise ValueError("bad UID hex: %r" % text)
    raw = bytes.fromhex(h)
    if not 1 <= len(raw) <= 6:
        raise ValueError("UID must be 1..6 bytes: %r" % text)
    return raw


def main():
    ap = argparse.ArgumentParser(description="ELRS bind-phrase dictionary cracker (offline OSINT)")
    ap.add_argument("uids", nargs="*", help="UID hex strings (full 6-byte or tail)")
    ap.add_argument("--uids", dest="uidfile", help="file with one UID per line")
    ap.add_argument("--wordlist", dest="wordlist", help="custom wordlist file")
    args = ap.parse_args()

    # self-check against the known vector
    assert uid_for_phrase("ExpressLRS") == bytes.fromhex("437f2fb1d339"), "self-check failed"

    targets = []
    for u in args.uids:
        targets.append(parse_uid(u))
    if args.uidfile:
        for line in open(args.uidfile):
            line = line.strip()
            if line and not line.startswith("#"):
                targets.append(parse_uid(line))
    if not targets:
        ap.error("no UIDs given (argv or --uids file)")
    for t in targets:
        print("target: %s (%s)" % (t.hex(), "full UID" if len(t) == 6 else "tail %d bytes" % len(t)))

    words = list(BUILTIN_WORDS)
    if args.wordlist:
        words += open(args.wordlist, encoding="utf-8", errors="replace").read().splitlines()
    print("dictionary: %d words x %d suffixes" % (len(words), len(SUFFIXES)))

    matches = []
    t0 = time.time()
    n = 0
    tail_hits = 0
    for cand in gen_candidates(words):
        u = uid_for_phrase(cand)
        n += 1
        for t in targets:
            if len(t) == 6:
                if u == t:
                    matches.append((t.hex(), cand, "FULL"))
            else:
                if u.endswith(t):
                    tail_hits += 1
                    matches.append((t.hex(), cand, "tail-only: PROBABLE (verify with more captures)"))

    dt = time.time() - t0
    rate = n / dt if dt > 0 else 0
    print("\n%d hashes in %.1fs (%.0f hashes/s)" % (n, dt, rate))
    if matches:
        print("\nMATCHES:")
        for uid, phrase, kind in matches:
            print("  %s  <-  %r   (%s)" % (uid, phrase, kind))
        return 0
    print("\nno match (dictionary exhausted; a GPU/wordlist attack on md5 is "
          "trivially possible but this dictionary covers the realistic OSINT cases)")
    return 1


if __name__ == "__main__":
    sys.exit(main())
