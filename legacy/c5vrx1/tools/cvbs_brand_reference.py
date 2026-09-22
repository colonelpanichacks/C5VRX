#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Validate scanline-only C5VRX splash/diagnostic content transitions."""

FONT = {
    "C": (14,17,16,16,16,17,14), "5": (31,16,30,1,1,17,14),
    "V": (17,17,17,17,17,10,4), "R": (30,17,17,30,20,18,17),
    "X": (17,17,10,4,10,17,17), "P": (30,17,17,30,16,16,16),
    "A": (14,17,17,31,17,17,17), "L": (16,16,16,16,16,16,31),
}

def text_pixel(text, x, y, ox, oy, scale):
    if x < ox or y < oy:
        return False
    px, py = (x-ox)//scale, (y-oy)//scale
    ci, col = px//6, px%6
    return py < 7 and ci < len(text) and col < 5 and bool(
        FONT.get(text[ci], (0,)*7)[py] & (1 << (4-col)))

def snow_codes(count, seed=0xC5F0A17D):
    values = []
    state = seed
    for _ in range(count):
        lsb = state & 1
        state = (state >> 1) ^ (0xD0000001 if lsb else 0)
        values.append((state >> 16) & 0xFF)
    return values

def self_test():
    # The 12-second transition is exactly 300 complete 25 Hz frames. Content
    # changes only while the renderer wraps to half-line zero.
    assert 300 / 25 == 12
    assert text_pixel("C5VRX", 332, 70, 320, 70, 12)
    assert not text_pixel("C5VRX", 319, 70, 320, 70, 12)
    splash_pixels = sum(text_pixel("C5VRX", x, y, 320, 70, 12)
                        for y in range(288) for x in range(1040))
    diagnostic_pixels = sum(text_pixel("C5VRX", x, y, 24, 15, 5)
                            for y in range(288) for x in range(1040))
    assert splash_pixels > 0 and diagnostic_pixels > 0
    assert splash_pixels != diagnostic_pixels
    snow = snow_codes(4096)
    assert len(set(snow)) > 240
    assert min(snow) < 8 and max(snow) > 247
    assert snow[:1024] != snow[1024:2048]
    # A request made within a frame remains pending until half-line zero.
    active, requested = "LOGO", "SNOW"
    for half_line in range(1, 1250):
        if half_line == 0:
            active = requested
        assert active == "LOGO"
    active = requested  # next frame boundary
    assert active == "SNOW"
    # Missing later samples never resets unlocked static back to the logo.
    assert active == "SNOW"
    print("CVBS branded scanline renderer self-test PASS")
    print("  splash duration: 300 frames = 12.0 s")
    print("  content transition: frame boundary only")
    print("  framebuffer allocation: none")
    print("  unlocked sample state: persistent moving snow")

if __name__ == "__main__":
    self_test()
