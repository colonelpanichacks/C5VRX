# RF-off EOF investigation

Physical tail=9 retest of e6cc418: still 32,764 received bytes, all equal
to the reference, four missing. TX40 timed out after 1,000,927 us and TX80
after 1,000,047 us. Both midstream IRQ snapshots zero. Capture SHA256:
`d171daec974dcf18b0f9e767cecbfe6980b99a762ca89e28cdfcc00e5783d40e`.
Changing 8 to 9 did NOT fix the issue.

The next diagnostic preserves the regular L80O capture first, then tests
upstream tails 8, 9, 10, 12, 16, 24, 32 on a RAM copy of the IDF v1 program.
Instructions/LUT remain unchanged. It also tests finite TX completion with
the existing Phase5 program at 40 MHz as a control. No live configuration
is selected automatically and no success gate is relaxed.

Read the 4096-byte trace partition at 0x111000 after running the test.
Records are 64 bytes: magic 0x52543243, version/size at 4, sequence at 8,
stage at 12, signed error at 16, and three detail words at 52/56/60.
For trial index i=0..6:

- 0x810+i: loopback error; details tail, written bytes, differing received bytes.
- 0x820+i: TX40 error; details tail, first transfer elapsed us, midstream IRQ.
- 0x830+i: TX80 error; same details.
- 0x840: Phase5 TX40 control error, details 40, elapsed us, IRQ.
- 0x84f: sweep reached its end (NOT a passing hardware gate).
- 0x800: unsupported binary header; sweep was not run.

Missing bytes remain failures even if differing received bytes is zero.
No timing rate can be inferred from a failed transaction. Each trace write
occurs after peripheral cleanup, outside measured transmission. The LED
continues to describe the original strict L80O test result, not the sweep.
Allow 30 seconds after boot; VTX stays off throughout.

## Physical sweep result

All seven loopback trials returned ESP_OK, with zero differing received
reference bytes. Tail 8/9 produced 32764 bytes; 10 produced exactly 32768;
12/16/24/32 produced 32772/32780/32796/32812 respectively. Tail 10 is now
selected for the embedded program. This fixes the finite loopback shortfall,
not the remaining TX issue.

Every linear80 finite TX trial timed out at both 40 and 80 MHz, with zero
midstream IRQ snapshots. Phase5 TX40 control completed successfully; first
transaction elapsed 707 us. End marker 0x84f was present. Thus the stock
test can complete for Phase5, but no linear80 throughput has been measured.

Next diagnostic additionally switches PARLIO to DATA_LEN EOF immediately
after transmit starts, setting the expanded output length to input_bytes*16
bits. This is isolated to the oracle and is NOT a live driver workaround.
Stages 0x850..853 are four TX40 transactions; 0x854..857 are TX80.
Details: input bytes, elapsed us, midstream IRQ; record error is the transfer
error (0xffffffff means unexecuted). The normal L80O gate still tests the
unmodified driver EOF path. A completion here would identify an EOF-mode
dependency, not establish correct physical DAC bytes or gapless operation.

## Physical counted-EOF result (81494e3)

All eight explicit DATA_LEN trials returned ESP_OK, with zero midstream IRQ
snapshots. Elapsed times include software setup and task wake-up:

| Requested clock | Input bytes | Elapsed us |
| --- | --- | --- |
| 40 MHz | 4096 | 924 |
| 40 MHz | 16384 | 1461 |
| 40 MHz | 4096 | 715 |
| 40 MHz | 16384 | 1338 |
| 80 MHz | 4096 | 779 |
| 80 MHz | 16384 | 1048 |
| 80 MHz | 4096 | 634 |
| 80 MHz | 16384 | 921 |

The same boot reproduced every DMA-EOF timeout in the sweep. Phase5 TX40
control completed (first elapsed 701 us). Tail 10 again produced exactly
32768 matching loopback bytes. End marker 0x84f present.

This establishes finite-transmission completion depends on EOF mode for this
program/setup. It does not prove why the DMA EOF signal fails, nor prove
continuous live throughput, pin-level byte correctness or analog settling.
In particular the CPU wall-time slopes are noisy and must not be promoted
to precise sample rates. The regular stock-driver L80O gate remains failed.
Next validation should observe the actual DAC bus and transmission timing,
then test simultaneous continuous RX/TX; do not insert finite DSP resets
into the live path. Raw recovery files remain in the local measurements directory:
`eof-counted-trace.bin` and `oracle160-tail10-counted.bin`.
