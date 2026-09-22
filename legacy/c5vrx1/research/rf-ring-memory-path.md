# RF ring memory and DMA path

## Addressability

The fixed dump window `0x40830000..0x4083ffff` lies inside the C5 internal,
byte-accessible DMA address range (`SOC_DMA_LOW=0x40800000`,
`SOC_DMA_HIGH=0x40860000`). Public pointer predicates therefore classify it as
internal and DMA-capable. This is **PROVEN STATICALLY** for address eligibility,
not proof that simultaneous RF-write/GDMA-read arbitration meets throughput.

C5VRX reserves the complete window before heap initialization and asserts
`_bss_end <= 0x40830000` at link time. The producer refuses availability if the
runtime DMA/internal predicates fail. This preserves fail-closed ownership.

## BitScrambler loopback

The IDF v6.0.2 loopback driver builds reusable AHB GDMA TX/RX link lists and
mounts caller-provided memory buffers for each run. Therefore:

- a contiguous fixed-ring subrange can be the input without a CPU memcpy;
- input/output and descriptor alignment must satisfy the driver's four-byte DMA
  alignment and transfer-size rules;
- descriptors and channels are reusable sequentially;
- two ring segments can be submitted alternately, but wrap must be split into
  two contiguous runs;
- the loopback attachment peripheral's DMA cannot be used concurrently by
  another owner;
- the public attachment list has I2S0/SPI/UHCI/AES/SHA/ADC/PARLIO but no direct
  Wi-Fi/FE endpoint.

The persistent WBFM context implements descriptor/LUT reuse. The queued source
ABI still requires immutable blocks, so the first guarded ring source copies
short windows into owned DMA buffers. A zero-copy ring-to-BitScrambler path must
be synchronous: check writer distance, transform the contiguous window before
overwrite, re-read the writer pointer, and discard on ambiguity. It must not
hand raw ring memory to the existing asynchronous queue.

Zero-copy is intentionally deferred until the real simultaneous benchmark
demonstrates that copying is the limiting stage. `BENCH RING PIPELINE 0 1000`
reports `copy_bytes_per_second`, `copy_cycles_total`, `copy_cpu_percent` and a
machine-readable `zero_copy_action`. The current decision threshold is measured
copy occupancy >=25% or a `COPY_AMBIGUOUS` stop; without valid copied blocks the
result is `NO_RESULT`. This prevents replacing the safer immutable contract on
the basis of an unmeasured 80 MS/s design case.

## Ordering and coherency

Ring accesses use volatile loads and RISC-V I/O/read fences around producer
ownership and completed copies. The fixed HP SRAM window is accessed as
internal memory rather than through an external cached mapping. No explicit
cache-maintenance API is currently required by a public C5 mapping rule, but
that is **LIKELY**, not a measured coherency result. A first-board walking-data
and simultaneous-writer test must verify CPU and GDMA observations agree.

The immutable-copy implementation allocates five 4096-word DMA slots (80 KiB)
only while LIVE is active. The fixed-size CVBS chunker adds one 1024-byte block;
USB preview optionally adds two 19,200-byte grayscale frames. Every allocation
is bounded and a failure leaves the RF producer stopped.

## Contention verdict

RF writer + CPU/GDMA reader + BitScrambler output + PARLIO GDMA share internal
memory/bus resources. No public arbitration guarantee proves 320 MB/s write plus
320 MB/s read. Sustainable rates, stalls and whether PARLIO deadlines survive
are **UNKNOWN**. The correct evidence is the prepared cadence/soak/ring copy and
BitScrambler/PARLIO benchmarks while the producer is active.
