#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Print C5VRX analog FPV coverage and nearest standard Wi-Fi centers."""

BANDS = {
    "A": [5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725],
    "B": [5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866],
    "E": [5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945],
    "F": [5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880],
    "R": [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917],
}

WIFI = {
    132: 5660, 136: 5680, 140: 5700, 144: 5720,
    149: 5745, 153: 5765, 157: 5785, 161: 5805,
    165: 5825, 169: 5845, 173: 5865, 177: 5885,
}

RX_MIN = 5180
RX_MAX = 5885


def plan(freq: int):
    ch, center = min(WIFI.items(), key=lambda item: abs(freq - item[1]))
    return ch, center, freq - center


def main():
    print("C5VRX FPV coverage")
    print(f"C5 specified RX window used by project: {RX_MIN}-{RX_MAX} MHz\n")
    print(f"{'FPV':<4} {'MHz':>5}  {'C5':<3}  {'Wi-Fi':>6}  {'center':>6}  {'offset':>7}")
    print("-" * 43)
    for band, freqs in BANDS.items():
        for idx, freq in enumerate(freqs, 1):
            ch, center, offset = plan(freq)
            inside = RX_MIN <= freq <= RX_MAX
            print(f"{band}{idx:<3} {freq:>5}  {'IN' if inside else 'OUT':<3}  ch{ch:>3}  {center:>6}  {offset:+6}M")


if __name__ == "__main__":
    main()
