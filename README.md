# p501UHD: a UHD 4.6 driver and USB3 streaming path for the PZSDR P501, tested with an OCUDU 5G gNB

This repository ports the **PZSDR P501** (Zynq UltraScale+ XCZU5EV + AD9361, 2T2R) to **UHD 4.6** so that software built
on UHD can use it like a USRP. The target application is a stock **OCUDU** 5G NR gNB with real phones, but the module is a
normal UHD device (`type=p501`) and works with `uhd_find_devices`, `benchmark_rate` and any other UHD program.

Nothing in OCUDU is modified. The gNB sees a UHD radio; everything board-specific lives in this repository.

> Status (2026-11-24): a cell on band n78 (3489.42 MHz, 20 MHz, 30 kHz SCS) runs with real phones, **1x1 and 2x2**, on stock
> OCUDU. 1x1: about 47 Mbit/s down, 12-15 Mbit/s up. 2x2: 26-31 Mbit/s down, 18-19 Mbit/s up, less stable. See
> [Results](#results) and [TODO](#todo).

## What the project is

The P501 has no host interface that UHD knows. This repository adds one, in three parts:

1. **A UHD device module on the host**, `host/uhd_p501`, built as `libuhd_p501.so` and found by UHD through
   `UHD_MODULE_PATH`. It implements device discovery, the property tree (frequency, gain, rate, antenna, sensors), RX and TX
   streamers with hardware time, and stream commands.
2. **A daemon and a streamer on the board.** `tools/p501d.py` brings up the AD9361 and answers a small TCP control protocol.
   `kernel/ffs/p501_stream_wd.c` moves IQ samples between the FPGA DMAs and a USB3 bulk gadget.
3. **FPGA changes** (`hdl_ts`, `p501/base_ts*`): a hardware timebase with timed RX start and a timed TX burst FIFO, and a
   deeper DAC buffer.

```
 Open5GS core        gNB host (x86)                                 P501 board (PYNQ Linux on the ARM cores)
 ───────────         ─────────────────────────                      ─────────────────────────────────────────────
                     OCUDU gnb                                      p501d.py ── AD9361 setup, gains, LO, TCP :5555
 AMF ──N2/N3──       └ UHD 4.6                                      p501_stream (C, RT threads, FunctionFS)
 10.53.1.2             └ libuhd_p501.so  ◄── control: TCP/ECM ──►     ┌──────────────┐   ┌──────────────────────────┐
                           │                 192.168.2.0/24         │ RX DMA ring  │◄──│ PL: AD9361 ADC, p501_time │
                           └────────────────  IQ: USB3 bulk ──────► │ TX DMA ring  │──►│     DAC FIFO, timed bursts│
                                  (FunctionFS gadget, 5 Gbit/s)     └──────────────┘   └──────────────────────────┘
```

The board enumerates as a USB device with two functions: **ECM** (a network link, host 192.168.2.10 / board 192.168.2.1) for
control, and a **FunctionFS bulk interface** (VID:PID 1209:0002) for samples. 2R2T at 30.72 MS/s needs about 245 MB/s each
way; the link measures 300-460 MB/s one way.

## How UHD is ported

UHD's device model is an interface, not a protocol, so a new radio needs a module that provides it. The pieces:

| UHD concept | In this port |
|---|---|
| Device discovery (`type=p501`) | the module registers a finder and a factory; it opens the USB device and the board's control port |
| Property tree | frequency, gain, rate, antenna, and the LO-locked sensor, each backed by a daemon command (`set_rx_freq`, `set_tx_gain`, `lo_locked`, ...) |
| RX streamer | 32 libusb bulk transfers in flight, 56-byte header + payload per message; the header carries a sequence number, the hardware timestamp and overflow flags; the module reports overflows to UHD |
| TX streamer | libusb bulk transfers of IQ with a timestamp per burst; the board plays a burst at its timestamp |
| Time (`time_spec`) | hardware time from the PL block `p501_time`, read over the control channel and extrapolated with the host clock; timed RX start (`REQ_RX_START_AT`) and timed TX bursts |
| Rates and formats | 7.68, 11.52, 15.36, 23.04 or 30.72 MS/s (`set_rx_rate`/`set_tx_rate` -> daemon `set_rate`, which re-runs the AD9361 bring-up), `fc32` and `sc16` on the host, 12-bit on the AD9361 port (`otw_format: sc12` in the gNB config) |

What a USRP gives for free had to be built here:

- **A hardware timebase.** OCUDU schedules TX from RX timestamps, so RX and TX must share one clock and a TX burst must go
  out at an exact sample. `hdl_ts/p501_time.v` adds a free-running counter, a timed RX start gate, and a timed TX burst FIFO
  with a late-burst tolerance register. The `base_ts_txbuf` bitstream also deepens the DAC DMA buffer to 128 KB, because one
  short DAC starvation otherwise makes every following contiguous burst late.
- **A transport.** USB3 bulk through the FunctionFS gadget (kernel module sources in `kernel/ffs`). `p501_stream` keeps USB
  OUT requests queued with AIO, flushes caches in a separate thread, drops bursts that are already late and resyncs the PL.
- **A timing budget.** The host adds `tx_advance_samples` to TX timestamps and the gNB compensates with
  `time_alignment_calibration`, which gives the gNB 3-3.5 ms of RX to TX processing time without patching OCUDU. The same
  advance has to leave the TX data enough lead to cross USB and reach the board in time, and the RAR must still fit the 10 ms
  window of a licensed band. Most of the tuning in this project was this budget; see
  [`examples/ocudu_gnb/README.md`](examples/ocudu_gnb/README.md) lessons 8 and 9.

Limits of the current module: 2R2T only (so at most 30.72 MS/s; 61.44 MS/s would need 1R1T mode), one LO frequency for all channels (a retune re-runs the
AD9361 bring-up, about 1 s), no asynchronous underflow or late-burst messages to UHD.

## Repository layout

| Path | Content |
|---|---|
| `host/uhd_p501/` | the UHD module (`p501_device.cpp`, CMake) |
| `kernel/ffs/` | board side: `p501_stream_wd.c` (streamer), `p501d_start.sh` / `p501d_stop.sh` (gadget + daemon), out-of-tree `usb_f_fs` sources, USB test programs |
| `tools/` | `p501d.py` (board daemon), gNB and board launchers (`run_gnb_p501.sh`, `full_restart_gnb.sh`, `board_restart_p501d.sh`, `core_watchdog.sh`), `usbout_test.cpp` (USB throughput), RF analysis scripts |
| `hdl_ts/` | Vivado sources of the PL time block and the DAC buffer (`p501_time.v`, `build_*.tcl`, testbench); simulation leftovers are not tracked |
| `p501/` | board support: PYNQ image definition, bitstreams (`base`, `base_dmac32`, `base_ts`, `base_ts_txbuf`), AD9361 bring-up notebooks and report |
| `examples/ocudu_gnb/` | the worked example: gNB configs (1x1 and 2x2), Open5GS config, results and lessons |
| `configs/`, `docs/BRINGUP_NOTES.md` | working configuration notes |

## Quick start

Prerequisites: a P501 running the PYNQ image from `p501/` with the repository deployed under `/home/xilinx/p501UHD`
(`deploy_to_board.sh`), a host with UHD 4.6 development files, libusb-1.0 and CMake, a USB3 cable, and a 10 MHz reference on
the board's REF IN for phones to sync (see [TODO](#todo)).

```bash
# 1. board (as root): load base_ts_txbuf, start the gadget and the daemon, raise the USB IRQ thread
bash tools/board_restart_p501d.sh              # installed on the board as /home/xilinx/restart3.sh

# 2. host: bring the USB network up, build the module, check it
sudo ip link set <iface> up; sudo ip addr add 192.168.2.10/24 dev <iface>
echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
cmake -S host/uhd_p501 -B host/uhd_p501/build -DUHD_ROOT=/usr/local && cmake --build host/uhd_p501/build -j
export UHD_MODULE_PATH=$PWD/host/uhd_p501/build/libuhd_p501.so
uhd_find_devices --args type=p501
/usr/local/lib/uhd/examples/benchmark_rate --args type=p501 --rx_rate 30.72e6 --tx_rate 30.72e6 \
    --rx_channels 0,1 --tx_channels 0,1

# 3. gNB: stock OCUDU (commit bd0d2cd27c) and Open5GS, see examples/ocudu_gnb/README.md
METRICS=1 bash tools/run_gnb_p501.sh examples/ocudu_gnb/gnb_p501_2x2_n78_20mhz.yaml
```

Set the host CPU governor to `performance`: the gNB's real-time threads miss deadlines under `powersave`.
The 1x1 config is `gnb_p501_1x1_n78_20mhz.yaml`. Details, the Open5GS setup and the phone steps are in the example README.

## Results

Cell n78, ARFCN 632628, 20 MHz, 30 kHz SCS, stock OCUDU, TCP with iperf3 (one 20-30 s run per figure, so a few Mbit/s
either way is noise). Sum of both phones where two are attached.

| Setup | Phones | Downlink | Uplink | Notes |
|---|---|---|---|---|
| 1x1, 30.72 MS/s, TX 69 / RX 45 | 1 | about 47 Mbit/s (MCS 26-27) | 12-15 Mbit/s (SINR about 23 dB) | 3 ms TX advance |
| 2x2, 30.72 MS/s, TX 69 / RX 45 | 2 | 26-31 Mbit/s per phone (rank 2, MCS 20-23, ~12% HARQ failures) | 18-19 Mbit/s | first 2x2 result; ~3% of TX messages late at the board |
| 2x2, 30.72 MS/s, TX 69 / RX 25, phones next to the antennas | 2 | 65 + 73 Mbit/s | 4.7 + 4.5 Mbit/s | RX 45 overloaded the AD9361 at this range (`ovl`, UL SINR -2 dB) |
| 2x2, 30.72 MS/s, TX 82 / RX 40, threads pinned | 2 | 42.5 + 48.2 Mbit/s | 5.6 + 5.8 Mbit/s | ~60 `PUxCH request late` per test, 10-15 ms RX lag spikes |
| **2x2, 23.04 MS/s**, TX 82 / RX 40, pinned, `P501_RXMAXSAMPS=5760` | 1 | **78.9 Mbit/s** (rank 2, MCS 27, 5% HARQ failures) | 11.2 Mbit/s | no late-uplink warnings |
| 2x2, 23.04 MS/s, same | 2 | 50.1 + 51.0 Mbit/s (rank 2, MCS 27) | 3.5 + 3.6 Mbit/s | UL MCS 8-10: RX gain too low for this range |
| 2x2, 23.04 MS/s, RX 50 (manual sweep) | 2 | - | 7.2 + 8.0 Mbit/s | sweep 30-60: 50 best, 55 clips, 30 too weak |
| 2x2, 23.04 MS/s, board AGC on | 1 | - | 11.4-12.2 Mbit/s | AGC went 35 -> 43 and 62 -> 44-46 |

What moved the numbers:
- **RX gain.** The best `rx_gain` depends on the distance to the phones (25 next to the antennas, 45-50 a few metres
  away). Too high clips the AD9361 (`ovl` in the gNB metrics, UL MCS 1); too low leaves UL MCS at 6-10. The board AGC
  (`P501_AGC=1`) now sets it from the uplink slots.
- **TX advance and the OCUDU patch.** The 1x1 downlink went from 27 to 47 Mbit/s when the TX advance went from 6.5 to 3 ms
  and the OCUDU patch was dropped. 2x2 needed short TX messages and a different TX ring, see the example README.
- **TX gain.** With the phones a few metres away, TX 82 instead of 69 took 2x2 DL from 28 + 35 to 41 + 43 Mbit/s.
- **Thread pinning** (`tools/pin_gnb.sh`) cut the late-uplink warnings about 8x (392 -> 48 per test).
- **23.04 MS/s.** At 30.72 MS/s the board's USB IN path is about 90% busy (a 256 KiB write takes 0.95 ms for 1.07 ms of
  samples), so any stall left 10-15 ms of RX backlog; 23.04 MS/s gives it 30% headroom. RX lag spikes dropped to about
  6 ms and the late-uplink warnings to zero, but only with a small gNB RX buffer (`P501_RXMAXSAMPS=5760`, 0.25 ms):
  with the default 65536 samples (2.8 ms at this rate) the gNB logs hundreds of them per second.
- **Board temperature** is a suspect for worse results in a warm room (a hot board and reference clock gave lower DL MCS
  and more HARQ failures); not measured yet, the board exposes no temperature sensor to Linux.

Open: the uplink is lower in 2x2 with two phones than with one, and one 23.04 MS/s bring-up came up with CQI 4 and MCS 0
on the downlink (the next restart was fine), probably a random AD9361 calibration state.

## Documentation

- [`examples/ocudu_gnb/README.md`](examples/ocudu_gnb/README.md): setup, what each P501 setting means, lessons learned, 2x2.
- [`docs/BRINGUP_NOTES.md`](docs/BRINGUP_NOTES.md): the working configuration and the open problems.
- [`p501/README.md`](p501/README.md) and [`p501/docs/AD9361_BRINGUP_REPORT.md`](p501/docs/AD9361_BRINGUP_REPORT.md): board support and the AD9361 bring-up.
- `p501/base_ts*/README.md`: what each bitstream changes.

## TODO

**RF and radio**
- [ ] Frequency accuracy: phones only sync with a 10 MHz reference on REF IN (without it the carrier is about 10 ppm off); add the external clock option to the module and the gNB config, and document the reference setup.
- [ ] Measure output power and spurious emissions; confirm `tx_gain` against a power meter or spectrum analyzer.
- [x] Automatic RX gain: a slow loop in `p501d` (`P501_AGC=1`, on in `board_restart_p501d.sh`) driven by the streamer's per-slot level meter, uplink slots only. From `rx_gain` 35 it reached 43 in about 20 s and from 62 it dropped to 44 in about 6 s; one phone then ran 12 Mbit/s UL.
- [ ] AGC: detect the UL slots from the gNB's TDD pattern instead of `P501_AGC_UL_SLOTS` (7,8,9 for the default 5 ms DDDDDDSUUU), and tune the target per phone count.
- [ ] Per-channel LO and gain; retune without re-running the full bring-up (now about 1 s and streams must stop).
- [x] More sample rates: 7.68, 11.52, 15.36, 23.04 and 30.72 MS/s (2R2T). Only 23.04 and 30.72 are tested with phones; the LVDS delays were tuned at 30.72.
- [ ] 1R1T mode for 61.44 MS/s (40 MHz cells) and other bands.
- [ ] Check that the RF switch bits `rf_sw[2]` and `[3]` reach the TRX2 and RX2 switches in the `base_ts` bitstreams.

**5G performance and stability**
- [ ] 2x2: stop the board dropping about 3% of TX messages (they arrive with under 0.2 ms of lead). Candidates: pin the gNB lower-PHY threads to dedicated cores, flush TX per symbol group instead of per slot, shorten the host-to-board path.
- [ ] 2x2: remove the remaining `PUxCH request late` and `Discarded uplink slot` warnings and the occasional phone re-attach under load.
- [ ] 2R1T (1 TX, 2 RX) was unstable on stock OCUDU; retest with the 2x2 fixes.
- [ ] Downlink: the phones report CQI 5 in 1x1 whatever the signal (rank 2 reports 13-14); check the CSI-RS and single-port CSI path. Reduce the 3-5% HARQ failures at MCS 27.
- [ ] Long-run stability test (hours, several phones) and a soak test of the USB link with the gNB idle and loaded.
- [ ] Reach OAI UE (B210) attach: it gets to RRC connected and then SRB1 hits the maximum RLC retransmissions; the phones are fine.

**Driver and board software**
- [ ] Asynchronous messages: report underflow, late burst and overflow to UHD through the async message queue.
- [ ] The tolerance in the PL hides DAC DMA starvation (up to about 0.6 ms with heavy host logging); find and remove the cause.
- [ ] Time: the host reads the board time over the control channel at most every 100 ms and extrapolates with its own clock in between (a burst of control transfers can wedge the USB IN pipe); take the time from the RX stream headers instead.
- [ ] Persist the host settings that now have to be made by hand (the CPU governor; `usbfs_memory_mb` is persistent already) and make the launcher scripts independent of the author's paths.
- [ ] TX ring sized in time instead of messages, chosen automatically from the host message length (`P501_TXP_KIB` is a manual setting).
- [ ] Reduce USB data with 12-bit packing in the PL (25% less TX and RX data), or move to 10G Ethernet (needs a MAC and DMA in the PL; only worth it if USB latency becomes the limit).
- [ ] Test other UHD versions (4.7 and later) and other OCUDU or srsRAN builds; add `benchmark_rate` and loopback tests to a CI job.

**Core network and tooling**
- [ ] Open5GS AMF assertion bug: the container exits with code 139 and `core_watchdog.sh` only restarts it.
- [ ] Clean the repository: drop the Vivado simulation leftovers in `hdl_ts/`, remove the `.pre_rfsw` bitstream backups, add a `LICENSE` and a `.gitignore`.
- [ ] Add a script that builds and deploys the board streamer (`gcc -O2 -pthread -o p501_stream p501_stream_wd.c -lm`), the way `deploy_to_board.sh` does for the notebooks.

## Credits

Hardware bring-up builds on [fllay/puzi501iio](https://github.com/fllay/puzi501iio) (Vivado HDL and PetaLinux BSP) and the PYNQ
board definition in `p501/`. The gNB is [OCUDU](https://gitlab.com/ocudu/ocudu); the core network is Open5GS.
