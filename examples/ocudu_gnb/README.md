# Running an OCUDU gNB on the P501 through uhd_p501

This example runs a complete 5G SA cell on the PZSDR P501 (XCZU5EV + AD9361) with the `uhd_p501` UHD 4.6
module, an OCUDU gNB and an Open5GS core. It is the setup that was used to get real phones registered and
passing traffic, and the config files here are the ones that produced the numbers below.

| File | What it is |
|---|---|
| `gnb_p501_1x1_n78_20mhz.yaml` | OCUDU gNB config: band n78, ARFCN 632628 (3489.42 MHz), 20 MHz, 30 kHz SCS, PCI 3, TAC 7, PLMN 00101, 1T1R |
| `open5gs-5gc.yml` | Open5GS all-in-one core config used with it (AMF 10.53.1.2, GTP-U MTU 1400) |
| `../../tools/run_gnb_p501.sh` | persistent gNB launcher: graceful stop, waits for the USB re-enumeration, `METRICS=1` prints per-UE metrics |
| `../../tools/core_watchdog.sh` | restarts the Open5GS container (and the gNB) when the AMF dies |
| `../../tools/board_restart_p501d.sh` | restarts the board daemon (run as root on the board) |
| `../../tools/full_restart_gnb.sh` | graceful gNB stop, board restart, host IP, gNB start in one go |
| `../../tools/rx_noise_spectrum.py`, `../../tools/pusch_by_prb.py` | receiver noise capture and PUSCH SINR-by-allocation analysis |

## Result (stock OCUDU, one phone close to the antenna, RSRP about -87 dBm)

| | Throughput |
|---|---|
| Downlink TCP, `iperf3 -c <ue>` | about 47 Mbit/s steady (MCS 26-27), 3-5% HARQ failures |
| Uplink TCP, `iperf3 -c <ue> -R` | 12-15 Mbit/s (MCS 15-17 at about 23 dB SINR), `rx_gain` 45 |

For comparison: with the earlier 6.5 ms advance and the OCUDU patch the downlink was about 27 Mbit/s; at `rx_gain` 30 the
uplink was 5-6.5 Mbit/s with both the stock and the patched build (see lesson 3, the receive gain is what limits it).

## Topology

```
 Open5GS (docker)        gNB host (x86/arm64)                     P501 board (PYNQ)
 10.53.1.2  --N2/N3--    OCUDU gnb + libuhd_p501.so  --USB 3--    p501d + p501_stream, AD9361
                         192.168.2.10/24 (ECM)                    192.168.2.1/24
```

The board shows up as a USB ECM network interface (control channel on TCP port 5555) plus a FunctionFS bulk
interface that carries the IQ stream. The module talks to both.

## Steps

1. **Board** (as root): load the bitstream and start the daemon.
   `P501_BIT` must have a matching `.hwh` next to it. Use the bitstream with the deep DAC buffer:
   ```
   bash tools/board_restart_p501d.sh     # base_ts_txbuf bitstream, /tmp on tmpfs, AD9361 tracking on
   ```
   On the host, bring the USB network up: `ip link set <iface> up; ip addr add 192.168.2.10/24 dev <iface>`.
2. **UHD module** on the gNB host:
   ```
   cd host/uhd_p501 && cmake -B build -DUHD_ROOT=/usr/local && cmake --build build -j
   ```
   The launcher copies `build/libuhd_p501.so` to a private directory and sets `UHD_MODULE_PATH` to it.
   Raise `usbfs_memory_mb` (`echo 256 > /sys/module/usbcore/parameters/usbfs_memory_mb`) or the module warns.
3. **OCUDU**: stock build of commit `bd0d2cd27c`; no patch is needed.
4. **Core**: start Open5GS with `open5gs-5gc.yml`, add the SIM to the subscriber database and set the phone's
   APN. PLMN 00101 and 00102 are advertised.
5. **gNB**: `METRICS=1 bash tools/run_gnb_p501.sh examples/ocudu_gnb/gnb_p501_1x1_n78_20mhz.yaml`
   (the script has the author's paths for the OCUDU build, the module and the USB interface name; adjust them). Optionally run `tools/core_watchdog.sh` next to it.
6. Attach a phone, run `iperf3 -s` on it (or Termux) and test from the core host with `iperf3 -c <ue-ip>`
   (downlink) and `-R` (uplink).

## What the P501-specific settings mean

- `device_args: type=p501,tx_advance_samples=92160` and `time_alignment_calibration: -92160`: the module
  shifts every TX timestamp by 3 ms and removes the shift again before transmitting, so the gNB
  compensates with the same amount. This gives OCUDU the
  RX-to-TX processing time it needs, while the board plays each burst at its timestamp (timed TX in the
  `p501_time` PL block). Keep the two numbers equal and opposite.
- `otw_format: sc12`, `srate: 30.72`: the AD9361 port is 12 bit; 20 MHz at 30 kHz SCS needs 30.72 MS/s.
- `tx_gain` is `89.75 - attenuation_dB`. `rx_gain` is the AD9361 gain table index (0-76).
- `pdsch`/`pusch` OLLA keys: see "Lessons".

## Lessons from getting this to work

1. **Timed TX needs real-time behaviour on the board.** The board's `/tmp` was on the SD card; its ext4 journal
   commits stalled the streamer for 20-30 ms about every 4.6 s and caused TX underflows. `board_restart_p501d.sh`
   puts `/tmp` on tmpfs. The streamer (`kernel/ffs/p501_stream_wd.c`) also drops bursts that are already due and
   resynchronises the DMA and the PL when a late burst is discarded.
2. **A fixed DC offset wrecked the uplink.** The AD9361 receive path leaves a DC offset of about -37 dBFS that does
   not change with gain. It lands on the DC subcarrier (PRB 25 of 51). Every full-band PUSCH grant contains that
   PRB, so the SINR estimate fell to about -10 dB and the scheduler stayed at MCS 0-2 (about 1 Mbit/s uplink).
   Grants without that PRB reported +12 to +21 dB. Fix: `rx_streamer_impl::convert()` subtracts a slow (~75 Hz)
   running mean per channel (`P501_NO_DCBLOCK=1` disables it) and the board enables the AD9361 RX DC and quadrature
   tracking (`P501_TRACKING=1`, set by `board_restart_p501d.sh`). Use `tools/pusch_by_prb.py` on a gNB log written
   with `log: phy_level: info` to see the effect per allocation.
3. **`rx_gain` decides the uplink.** The receiver has a conversion noise floor that does not scale with gain, and a phone
   at its maximum power gives a low SINR when the gain is low. Sweeping the gain live (`set_rx_gain 0 <n>` on the board's
   control port 5555) with `iperf3 -R` and a phone at normal range gave: 30 -> 5-6 Mbit/s (SINR 15 dB, MCS 8), 36 -> 7,
   42 -> 8-11.5, 45 -> 12-14, 48 -> 14-15 (SINR 23 dB, MCS 15-17), 51 -> 12, 54 -> 5 and 60 -> 0.6 Mbit/s with the receiver
   overloaded (`ovl` in the metrics). 45 leaves 3 dB below the first overload. A phone very close to the antenna
   overloads earlier (40 did at about 1 m), so lower the gain there. Measure the noise floor with `tools/rx_noise_spectrum.py`.
4. **Link adaptation.** The default OLLA settings (1% target, 0.001 dB steps, limited range) leave both directions
   far below capacity here. The values in the YAML (10% target, faster steps, wider range) let the uplink reach MCS
   13-14 and the downlink MCS 25-27.
5. **The phone reports CQI 5 regardless of signal.** The gNB log shows `cqi 5` for every UE, and the PUCCH CSI payload
   is `0101` in thousands of reports, even when the TX power is swept over 35 dB. The phones nevertheless decode
   64QAM and 256QAM, so the real downlink is better than CQI 5. The cause is not found yet (likely the CSI-RS
   path). It is why the downlink OLLA range had to be widened.
6. **Open5GS can crash the AMF** with an assertion when a UE re-registers while its session is being released
   (Error Indication from the gNB). The gNB then drops the cell and phones see -130 dBm. `core_watchdog.sh` is a
   workaround, not a fix.
7. **Stopping and starting.** Stop the gNB with SIGINT and wait for it to exit. The board re-enumerates on USB when
   the gNB stops: wait about 12 s, then re-add the host IP. If the gNB is killed hard, the board's RX thread stays
   blocked and the next gNB reports "exceeded maximum number of timed out receive calls" and "PRACH buffer pool
   depleted": restart `p501d` on the board.
8. **No OCUDU patch: the RAR timing budget.** The RAR has to go out within `ra_resp_window` (at most 10 ms on licensed
   n78, so `ra_resp_window: 20` slots at 30 kHz). The RAR slot is roughly `1 + PRACH detection + tx advance + max_proc_delay`
   slots after the PRACH slot. With the earlier 6.5 ms advance and `max_proc_delay: 10` it was 27 slots, so
   a patch lifted the validator limit to 40 ms. With a 3 ms advance and the default `max_proc_delay: 5` the RAR fits.
   Limits found: `max_proc_delay` 3 gives "Downlink data late" (1000+ per minute); a 4 ms advance with delay 5 still missed
   the window by a slot; the TX lead at the USB write is then about 1 ms (median 1.2 ms, `P501_DEBUG_TX=1`), and the
   board's PL tolerates 1 ms more. Never set `OCUDU_RX_TO_TX_MS` (it only existed in the old patch).

## Known limits

- The phone reports CQI 5 whatever the signal (see lesson 5), and the downlink still has 3-5% HARQ failures at MCS 27.
- `rx_gain` is a compromise for one phone distance: the receiver overloads above about 50 at normal range and earlier when the phone is very close. An automatic gain mode has not been tried.
- 1T1R only: no 2T2R configuration is included in this example.
- Output power and the external reference clock input have not been measured or enabled yet.
