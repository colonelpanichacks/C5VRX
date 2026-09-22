# Real-time feasibility model

These are design cases, not measured C5 throughput. `Fs` is complex samples/s,
one ring word is four bytes, CPU is 240 MHz, and the ring is 16,384 words.

| Candidate Fs | RF writer | Ring wrap | half-ring service budget | CPU all: read / cycles retained | CPU /2 | CPU /4 | CPU /8 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 10 MS/s | 40 MB/s | 1638.4 us | 819.2 us | 40 MB/s / 24 cyc | 20 MB/s / 48 cyc | 10 MB/s / 96 cyc | 5 MB/s / 192 cyc |
| 20 MS/s | 80 MB/s | 819.2 us | 409.6 us | 80 MB/s / 12 cyc | 40 MB/s / 24 cyc | 20 MB/s / 48 cyc | 10 MB/s / 96 cyc |
| 40 MS/s | 160 MB/s | 409.6 us | 204.8 us | 160 MB/s / 6 cyc | 80 MB/s / 12 cyc | 40 MB/s / 24 cyc | 20 MB/s / 48 cyc |
| 80 MS/s | 320 MB/s | 204.8 us | 102.4 us | 320 MB/s / 3 cyc | 160 MB/s / 6 cyc | 80 MB/s / 12 cyc | 40 MB/s / 24 cyc |

“Cycles retained” is `240 MHz / retained_rate`; it is the entire theoretical
CPU budget, including MMIO/pointer tracking, load, DSP, scheduling and output.
It is not a benchmark result. Sparse rows count only selected 32-bit reads.

| Strategy | Input bus read | Discriminator output | CPU relationship | CVBS consequence | Status |
|---|---|---|---|---|---|
| CPU all samples | `4 Fs` bytes/s | Fs | CPU touches every word | must filter/resample to 20 MS/s | high-rate cases **LIKELY** infeasible; benchmark required |
| CPU /2 sparse | `2 Fs` bytes/s | Fs/2 | twice all-sample cycle budget | direct CVBS only for Fs=40 MS/s | alias gate required |
| CPU /4 sparse | `Fs` bytes/s | Fs/4 | fourfold cycle budget | direct CVBS for 80 MS/s | current BS math, no anti-alias filter |
| CPU /8 sparse | `Fs/2` bytes/s | Fs/8 | eightfold cycle budget | upsample unless Fs=160 MS/s | alias/phase-wrap gate required |
| BitScrambler loopback | current program reads all `4 Fs` bytes/s | Fs/4 | CPU setup/monitor only after persistent descriptors | direct at 80 MS/s design case | DMA accessibility **PROVEN STATICALLY**; sustainable bandwidth **UNKNOWN** |
| lower-rate hardware tap | `4 Ftap` bytes/s | depends transform | avoids raw Fs consumer | prefer a 20–40 MS/s phase tap | no eligible lower-rate C5 tap found; mode 12 starts BLE RX |

Maximum safe service interval is smaller than a wrap and depends on actual lag,
block size, guard and measured copy/transform time. The guarded reader starts at
half-ring lag and rejects a copy if the conservative 320 MS/s bound permits an
unobserved wrap. At the 80 MS/s design case, 102.4 us is only a planning ceiling,
not a scheduling promise.

The C5 having a hardware 320 MB/s writer is not evidence the CPU or a second DMA
reader sustains that traffic. Conversely, three CPU cycles per raw sample does
not disprove the hardware-producer architecture. Benchmarks must separately
measure writer-only, sparse reads, loopback, PARLIO and simultaneous contention.
`BENCH RING PIPELINE 0 1000` is the bounded simultaneous-contention gate: unlike
the synthetic benchmarks, it runs the real writer, guarded immutable copies,
persistent BitScrambler, conditioner and PARLIO together for one second.
It also measures aggregate copy cycles/occupancy. Zero-copy is requested only
when that real run reports >=25% CPU in the copy itself or `COPY_AMBIGUOUS`;
otherwise the immutable-copy path remains the production candidate.
