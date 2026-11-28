# p501d (C)

Board daemon for the P501, replacing `tools/p501d.py`: no Python, PYNQ or XRT at run time. Same TCP control protocol on
:5555, same streamer (`../p501_stream`), so the host module needs no change. Started by `../p501d_start.sh` (default,
`P501_DAEMON=c`; `P501_DAEMON=py` runs the Python daemon instead).

Build on the board: `make` (needs `-lm -lpthread`). `ad9361_tables.h` is generated from the Python driver's tables with
`python3 gen_tables.py ../../../notebooks`.

What replaces PYNQ:
- bitstream: `.bit` header stripped, words byte-swapped, written to `/lib/firmware/p501d.bin` and loaded through
  `/sys/class/fpga_manager/fpga0`; the IP addresses come from the `.hwh` next to the `.bit`. The PL clocks (100, 500 and
  375 MHz) are already set at boot, so they are not touched.
- DMA rings: `/dev/dma_heap/linux,cma`, physical address from `/proc/self/pagemap` (checked to be contiguous).
- AD9361: `ad9361.c`, a port of `notebooks/ad9361_driver.py` over `/dev/spidev1.0` and the EMIO GPIOs 640-642.

The bring-up writes the same SPI sequence as the Python daemon (checked with `P501_SPI_TRACE=<file>` against a traced
Python run: identical apart from one redundant write). Note: the Python daemon imports the older
`/home/xilinx/jupyter_notebooks/ad9361_driver.py`, which runs the RF PLLs from the 40 MHz reference (x1) and ignores
`P501_TRACKING`; the C daemon does the same by default. `P501_SYNTH_MAX_FREF=80008000` doubles the reference to 80 MHz
(the vendor default, as `notebooks/ad9361_driver.py` does) and `P501_TRACKING=1` turns on RX DC and quadrature tracking;
neither has been tested with phones.

Environment: `P501_BIT`, `P501_PORT`, `P501_RATE`, `P501_TIME_PHYS`, `P501_AGC`, `P501_AGC_TARGET`, `P501_AGC_MIN`,
`P501_AGC_MAX`, `P501_AGC_UL_SLOTS`, `P501_TRACKING`, `P501_SYNTH_MAX_FREF`, `P501_SPI_TRACE`, `P501_STREAMER`.
`p501d --bringup-only <LO Hz>` loads the bitstream, brings the AD9361 up once and exits (for tests).

Tested: 2x2 at 23.04 MS/s with two phones and the AGC on: DL 51.6 + 50.1 Mbit/s, UL 5.7 + 6.0 Mbit/s, as with the
Python daemon.
