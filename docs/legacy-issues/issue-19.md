# Archive issue #19 — True continuous RF-to-AV output

| Field | Value |
|---|---|
| Repository | `Twotoz/C5VRX-archive` |
| Created | 2026-08-25 |
| Closed | 2026-08-29 |
| Original | [Issue #19](https://github.com/Twotoz/C5VRX-archive/issues/19) |

## Historical measurements

The implementation explored LP-core rearming, AV pacing/servo logic, and
automatic video output. Hardware runs accumulated thousands of successful
finite rearms, showing that the rearm primitive itself could be reliable.

## Historical conclusion

The issue was explicitly closed as superseded by the production work in issues
#5 and #22. Its own closing discussion says not to continue the old pacing and
synthetic-raster stack as the production receiver.

## Later status

- **Superseded:** periodic 16K capture/rearm is not modern C5VRX's source model.
- **Nuanced by later evidence:** modern work proved that a TX_START/dump-first
  producer can keep its hardware writer pointer wrapping after one start.
  However, its MAC-owned SRAM is not simultaneously readable as a useful live
  stream by CPU or AHB-GDMA, so MODEM_DIAG became the production IQ source.
- **Not proven by this issue:** gapless RF time or continuous recovered CVBS.

