# High-signal archive pull requests

Most archive pull requests are preserved by the joined Git history. The notes
below retain conclusions that existed primarily in review discussions.

| PR | Historical result | Current interpretation |
|---:|---|---|
| [#18](https://github.com/Twotoz/C5VRX-archive/pull/18) | Direct-gapless candidate reached PAL/DAC lock and exposed stack, FIFO, and one-shot-writer failures; many LP rearms succeeded. | Closed and superseded. Its 31–37 MS/s block throughput was not an RF sample-rate or gapless proof. |
| [#21](https://github.com/Twotoz/C5VRX-archive/pull/21) | Found an HP-TEE permission persistence failure: LP REE0 read-only permissions could survive reset and fault HP GDMA writes. | Confirmed historical hardware lesson. Keep HP/LP access permissions explicit; old pacing design is superseded. |
| [#23](https://github.com/Twotoz/C5VRX-archive/pull/23) | Exact old mode-6/bit-17 experiment showed no live fixed-address refresh and no phase-continuity pass. | Negative result for that configuration, not proof that all pre-trigger writers fail. Later TX_START/dump-first pointer wrapping succeeded, while live SRAM visibility still failed. |
| [#25](https://github.com/Twotoz/C5VRX-archive/pull/25) | Preserved useful REGDMA/rearm primitives and a 22,496-rearm zero-failure run. | Diagnostic donor only; finite rearm is not the production source. |
| [#26](https://github.com/Twotoz/C5VRX-archive/pull/26) | Open raw-RF-to-CVBS branch; its archive review never received the requested physical result. | Do not infer hardware proof from this PR. Modern canonical C5VRX later achieved live NTSC by a different topology. |

PR #6 also matters for attribution: it introduced a substantial XIAO A1 live
color-video proof and was authored by GitHub user `ItsReckliss`. That history
and attribution are preserved unchanged.
