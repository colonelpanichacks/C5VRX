#!/usr/bin/env python3
# test_uid.py — host reference for the ELRS bind-phrase -> UID derivation
# and the FLRC identity values used by the sniffer.
# Run: python3 test_uid.py
import hashlib

OTA_VERSION_ID = 3
MODELMATCH_MASK = 0x3F

def uid_from_phrase(phrase: str) -> bytes:
    # src/python/binary_configurator.py:82 and community converters hash the
    # WRAPPED compile-flag string, not the phrase alone.
    return hashlib.md5(f'-DMY_BINDING_PHRASE="{phrase}"'.encode()).digest()[:6]

def uid_mac_seed(uid: bytes) -> int:
    # common.cpp uidMacSeedGet()
    return (uid[2] << 24) | (uid[3] << 16) | (uid[4] << 8) | (uid[5] ^ OTA_VERSION_ID)

def crc_init(uid: bytes) -> int:
    # OTA.cpp OtaUpdateCrcInitFromUid()
    return (((uid[4] << 8) | uid[5]) ^ OTA_VERSION_ID) & 0xFFFF

# vector 1: the default phrase (verified against device expectation)
u = uid_from_phrase("ExpressLRS")
assert u == bytes.fromhex("437f2fb1d339"), u.hex()
assert uid_mac_seed(u) == 0x2FB1D33A
assert crc_init(u) == 0xD33A

# vector 2: sanity — the WRAPPED string differs from md5(phrase) plain
assert uid_from_phrase("ExpressLRS") != hashlib.md5(b"ExpressLRS").digest()[:6]

# vector 3: round-trip style custom phrase (self-consistent + printable)
u2 = uid_from_phrase("test phrase 123")
print(f'test phrase 123 -> uid {u2.hex(" ")} macseed {uid_mac_seed(u2):08x} crcinit {crc_init(u2):04x}')

print("ALL UID TESTS PASSED")
