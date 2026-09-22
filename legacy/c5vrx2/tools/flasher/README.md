# C5VRX minimal flasher

This intentionally keeps the hardware flow small:

1. connect to the ESP32-C5 ROM bootloader;
2. flash one merged image at `0x0`;
3. write a CRC-protected IQ/CVBS calibration record at `0x110000`;
4. open a raw 115200 serial terminal.

The calibration controls set discriminator pedestal, integer phase gain and
polarity. Live firmware acquires a coherent 2:1 subset of the approximately
80-MS/s MODEM bus at 40 MS/s, performs adjacent FM on every acquired sample,
then real-domain 2:1 filtering to the 20-MS/s AV stream. USB never paces it.

## Firmware

The local page includes `c5vrx2-iq-fm-idf601-40.bin`; click **Use continuous-IQ build**
instead of browsing for a file.

The `C5VRX realtime build` GitHub Action produces the compatibility-named
artifact `c5vrx2-full` containing:

```text
c5vrx2-full.bin
c5vrx2-full.bin.sha256
FIRMWARE-COMMIT.txt
TESTING.md
flasher.html
```

The image is generated with ESP-IDF 6.0.1 at 40 MHz DIO using `idf.py merge-bin`, so the web flasher does not need separate bootloader, partition-table or application offsets.
The checksum and commit file make the physical result traceable to one PR build.

## Run the flasher

Web Serial needs a secure context. From the repository root, or from the
unzipped artifact directory:

```bash
python -m http.server 8000
```

Then open the matching page:

```text
http://localhost:8000/tools/flasher/
http://localhost:8000/flasher.html
```

Use Chrome or Edge.

## Test flow

```text
click Use continuous-IQ build (or select another merged .bin)
-> Connect bootloader
-> Flash merged .bin
-> Open 115200 terminal
```

If the XIAO is not detected automatically, hold **BOOT**, press and release
**RESET**, then release **BOOT** before connecting again.

The page uses Espressif `esptool-js` 0.6.1 and keeps the flash mode/frequency/size encoded by the merged firmware.
