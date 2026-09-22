# Proven donor policy

Modern C5VRX does not inherit the original C5VRX product state machine. The
historical tree under `/legacy/c5vrx1` is accepted as a donor only when a
primitive directly supports one of these needs:

- ESP32-C5 5 GHz RF initialization and A1/5865 MHz tuning;
- RF dump SRAM reservation and ownership;
- mode-0 packed Q10/I10 producer configuration;
- vendor-oracle pre-trigger/dump-first register state;
- PARLIO six-bit output on XIAO D4..D9;
- bounded counters and diagnostics outside the hot path.

Original commit `d17b2c56f1b6bb2973af5f96d60a6fa0e7b58837` remains the golden donor for
finite RF/IQ semantics and host-video proof. Its “live” IQ was host-chained
`CAPTURE 16384`, not chip-side continuous acquisition. Finite-rearm work is
diagnostic history, not the current stream architecture. Archive PR #26 is not
a realtime architecture donor; reuse only independently verified primitives.
