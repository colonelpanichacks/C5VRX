# WBFM discrimination and decimation

For complex input `x[n] = I[n] + jQ[n]`, the adjacent-sample discriminator is

```text
d[n] = arg(x[n] conj(x[n-1])).
```

For retained factor `N`, direct strided discrimination is

```text
y[k] = arg(x[kN] conj(x[(k-1)N]))
     = wrap(sum(d[kN-N+1 .. kN])).
```

Thus it integrates phase change across `N` original intervals, but the final
`arg()` wraps to `[-pi, pi)`. It is valid only when the total wanted phase step
does not cross that ambiguity and when the IQ presented to the downsampler is
band-limited to the new complex Nyquist interval.

## Two valid orderings

1. IQ low-pass -> decimate IQ -> discriminate. For factor `N`, the complex
   anti-alias passband must fit inside `|f| < Fs/(2N)` with transition margin.
2. Discriminate all adjacent IQ samples -> low-pass the real discriminator ->
   decimate. This removes IQ aliasing before rate reduction but requires the
   discriminator to see every high-rate input.

Simply loading every Nth word is not an anti-alias filter. Direct N-apart
discrimination does not make that omission safe; out-of-band complex energy
has already aliased at the retained sampling operation.

For analog FPV, required passband must be established from controlled VTX tone
and modulated-spectrum sweeps. A nominal channel label alone does not prove the
occupied WBFM bandwidth or the C5 tap's filter response. Until that response is
measured, `source_bandwidth_known=false` and sparse live operation fails closed.
`TONE RESPONSE PROBE <mode> <signed_offset_hz> <measured_rate_hz>` automates the
C5-side phase/coherence/magnitude record for each externally selected sweep
point; it never marks a factor safe by itself.

## Exact current BitScrambler 4:1 behavior

`c5vrx_wbfm_4to1.bsasm` consumes four 32-bit input words per output byte. It:

- derives a coarse 6-bit phase from word `x[4k]` using the high five bits of
  each signed 10-bit I/Q component and a 1024-entry LUT;
- subtracts the saved phase from the previous retained word `x[4(k-1)]`;
- reads and discards `x[4k+1]`, `x[4k+2]`, and `x[4k+3]`;
- emits one biased modulo-64 phase delta;
- contains no FIR, CIC, averaging, or other IQ anti-alias filter.

So its 4:1 means “compare samples four input positions apart,” not “compute
four adjacent deltas.” It is mathematically the wrapped accumulated phase over
four intervals. The first output is a priming value because previous phase is
reset state. The hardware self-test proves program/reference equivalence in
software; RF correctness and throughput are **IMPLEMENTED / NOT PHYSICALLY
TESTED**.

The loopback object and LUT are persistent across live blocks. This removes
per-block hardware construction overhead. Because a loopback run primes its
own first output, the live pipeline replaces exactly that byte with the same
coarse-LUT phase difference between the previous retained word and the new
block's first retained word. A marked source discontinuity instead emits bias
and resets the boundary state. Block-boundary statistics remain mandatory.

## Supported factors

The shipped hardware program supports exactly N=4. CPU sparse benchmarks cover
2/4/8, but those factors become allowed capabilities only after:

- measured source cadence;
- measured source/tap bandwidth satisfying anti-alias limits;
- coherent-tone phase continuity across wrap;
- no overrun at the selected guarded block size;
- at least 20% measured processing margin.
