# First physical linear80 oracle result (2026-09-13)

Firmware: commit 2253641, RF-off oracle, CPU 160 MHz. VTX remained off.
Read-only recovery from COM10 in manual download mode:

```
python -m esptool --chip esp32c5 --port COM10 --baud 460800 --before no-reset --after no-reset read-flash 0x111000 0x12000 measurements/pr16-linear80/oracle160-failure-state.bin
python tools/analyze_linear80_oracle.py measurements/pr16-linear80/oracle160-failure-state.bin --offset 0x1000
```

The persisted L80O record and both payload hashes validate. Loopback completed
with ESP_OK and 32,764 output bytes. Every received byte matches the reference;
four final bytes are missing versus the strict 32,768-byte expectation.
This is not a passing full-output gate. The finite-stream tail needs investigation.

The first 4,096-input-byte decorated TX transaction timed out at both requested
rates: 40 MHz after 1,000,715 us, 80 MHz after 999,963 us (ESP_ERR_TIMEOUT,
0x107). Both midstream IRQ snapshots were zero. The six remaining rows were
never executed; their 0xffffffff values are sentinels, not measured FIFO errors
or timing. There is no valid incremental throughput measurement.

The app intentionally turns the active-low LED off on this failed oracle result
and waits forever. A saved complete record explains the apparent stopping;
it does not establish a reboot/crash. This image is a one-shot test, not live RF
firmware. USB silence does not by itself distinguish those states.

Do not infer a 40/80 MHz hardware speed limit or analog improvement from these
timeouts. Investigate finite EOF propagation/completion first, keeping the
strict byte gate. No replacement firmware was flashed during this recovery.
