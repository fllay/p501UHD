# base_ts_txbuf — base_ts with a deep DAC (TX) DMA buffer

Only change vs base_ts: axi_ad9361_dac_dma FIFO_SIZE 8->32, MAX_BYTES_PER_BURST 128->4096
(store-and-forward memory 1 KB -> 128 KB, ~0.5 ms of 2ch data), script hdl_ts/build_txbuf.tcl.
p501_time.v is unchanged. Rationale: one ~0.6 ms DAC starvation (unf jumps ~18500) delays the
TX stream by D for the rest of the burst; chained contiguous bursts inherit D and every later
burst is "late" (done freezes, late rises ~280/s, gNB "Real-time failure: late"). The PL cannot
catch up (stream moves 1 frame/clk = real time), so the fix is to prevent starvation.

Files: system_top.bit (new, md5 58c77f6c...), system_top.hwh = base_ts hwh (DMA register map is
unchanged; freshly generated hwh is kept as system_top.hwh_generated, it fails PYNQ parsing per
base_ts/README). Timing: WNS +0.417, WHS +0.010 (setup/hold met); one pulse-width endpoint at -0.002 ns.
Previous build backups on the Vivado host: /home/pi/projects/p501UHD_hdl/backup_base_ts_1123/.

## Test tomorrow (board p501, nothing was changed on it)
1. scp p501/base_ts_txbuf/system_top.bit + system_top.hwh to /home/xilinx/p501UHD/base_ts_txbuf/
2. Power-cycle board; start p501d with P501_BIT=/home/xilinx/p501UHD/base_ts_txbuf/system_top.bit
   (hwh must sit next to the bit, else the USB gadget hangs).
3. Run gNB with P501_STREAM_DEBUG=1; watch /tmp/p501d.log mon lines: unf should stay flat,
   done keep increasing, late ~0. Check phone RSRP (-80) stays and no "Real-time failure: late".
4. If unf still jumps, the starvation is upstream of the DMA buffer (submitter/DDR) -> watchdog/ring.
