# Issue 6 static analysis

This note records the corrected measurements behind the post-PR5 static work.
It deliberately separates offline evidence from hardware proof.

## Corrected frozen-capture result

The original winding metric wrapped both the sum of two adjacent phase deltas
and the endpoint delta. Those expressions are then algebraically equivalent,
so the tool incorrectly reported zero winding loss. The corrected comparison
keeps the adjacent sum unwrapped.

On `raw-vtx-on.bin` (16,384 Q4/I4 bytes at 40 MS/s):

- endpoint `n -> n+2` loses a `+/-2pi` winding on 8.351% of intervals;
- among the current conservative strong-IQ subset this is 0.285%;
- therefore winding is real, but a blanket branch correction is unsafe;
- the previous golden adjacent model also used phase5 rather than phase8
  units before the production gain function, understating its output by 8x.

## PR5 invalid-state regression

The signed-nibble test `I*I + Q*Q < 5` selects 13 of 256 raw cells, not six.
It also reused phase state 31 as the invalid marker and merged legitimate
phase-31 samples into phase zero. In the frozen capture this marked 11.853% of
samples and contaminated about 20% of endpoint discriminator intervals.
Replacing those intervals with pedestal creates visible dark/blanking specks.

The production candidate therefore preserves all 32 phase states. A future
confidence repair must have independent temporal state and must not alias a
valid circular phase code.

## Middle-sample feasibility result

The skipped middle Q4/I4 byte is present in the TX input window. A 10-bit
`phase4(previous) + middle quadrant + phase4(current)` lookup classified the
winding branch correctly on 96.2% of the frozen intervals and reduced large
offline errors. However, an analytic phase4 implementation produced only
seven DAC levels. That trades static for unacceptable posterisation and is not
used in production.

An 11-bit direct raw projection improved large errors only modestly and cannot
share the current 16-bit dual-purpose phase/output LUT. The search oracle is
`tools/search_middle_hint.py`; its results must be cross-validated on more
captures before any compressed middle-hint LUT is shipped.

## Hardware A/B candidate

The remaining lossless diagnostic is the opposite PARLIO RX sampling edge.
It changes neither rate nor DSP and can reveal GPIO-matrix setup/hold errors.
`CONFIG_C5VRX2_PARLIO_RX_NEG_EDGE` makes the physical edge selectable. The
quality test build enables the alternate edge; it is not considered better
until the same VTX/camera scene visibly or quantitatively beats the baseline.

Still unproven:

- whether either edge eliminates a meaningful share of static;
- a realtime middle-sample branch repair with full DAC resolution;
- colour-safe roofed de-emphasis in the C5 instruction/transport budget;
- the origin of continuous fine FM snow versus sparse hard clicks.
