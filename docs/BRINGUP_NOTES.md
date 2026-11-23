# P501 + UHD module + stock-ish OCUDU: working configuration (2026-11-23)

Phones connect to the cell. Things that must be right:

* **10 MHz reference on the P501 REF IN.** Without it the carrier is about 36 kHz (10 ppm) off and no UE can sync. With it the
  offset to a B210 is about 1 ppm. Put the same source on the B210 for a clean test.
* **RF switches, `rf_sw` = 0xC** (`/home/xilinx/rfsw.py 12`; also the PL reset default). TRX1 SMA <- PA (TX1), RX1 chain <- RX1 SMA,
  TRX2 SMA <- PA (TX2), RX2 chain <- RX2 SMA, as in the ADI design `system_top.v`. gNB TX on TX1: do not set `P501_TX_SWAP`.
  In bitstream `base_ts` (`.ab_v2`) rf_sw[2] and [3] still go to the old AB10 / AB9 pins; not checked whether they reach the TRX2 / RX2 switches.
* **PL late tolerance** (reg 0x38 of p501_time). Default 64 ticks (2 us): one short DAC starvation and every following contiguous
  burst is discarded (cascade; half the runs bad). `p501_stream` now writes 30720 ticks (1 ms) at start (`P501_LATE_TOL` overrides).
* **1-channel streaming:** only the used RX / TX channel crosses USB (4 byte frames, 128 KB messages) via the PL packers
  (ADC enable / DAC DATA_SEL set by `p501_stream`). `P501_ALLCH=1` (any value) forces the old 2-channel stream.
* **P501 rx_gain 20** (40 saturates with a B210 UE at full power: Msg3 lost). `tx_gain 69`.
* Board start: fresh `p501d_start.sh` after a power-cycle (an in-place restart drops the USB link), then re-add 192.168.2.10/24 on the
  host interface. `p501_stream` prints TX latency histograms on stop (lead at read / at descriptor write, read to write latency).

## gNB (OCUDU) side

* `configs/p501_1x1_ocudu.yaml`: stock options only (`device_args: type=p501,tx_advance_samples=199680`,
  `time_alignment_calibration: -199680`, `max_proc_delay: 10`, `ra_resp_window: 80`).
* `ocudu/ocudu_p501.patch`: three small changes still needed because the RX data is about 3.4 ms stale: RAR window limit 40,
  UL pipeline depth 4 frames, lower-PHY request pool 64. Goal: remove them by cutting the staleness.

## Open

* OAI UE (B210) reaches RRC connected, then SRB1 hits max RLC retransmissions; the phones are fine.
* Starvation of the DAC DMA (up to ~0.6 ms with heavy host logging) is still there; the tolerance only hides it.
