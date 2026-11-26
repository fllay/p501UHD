# uhd_p501 - UHD 4.6 module for the PZSDR P501

Board: start the daemon as root (FunctionFS gadget + AD9361 2R2T 30.72 MS/s + streamer + TCP control :5555):
    /home/xilinx/p501UHD/kernel/ffs/p501d_start.sh      (p501d_stop.sh to stop; log /tmp/p501d.log)
Host:
    cmake -B build -DUHD_ROOT=/usr/local && cmake --build build -j
    export UHD_MODULE_PATH=$PWD/build/libuhd_p501.so
    uhd_find_devices --args type=p501
    sudo -E /usr/local/lib/uhd/examples/benchmark_rate --args type=p501 --rx_rate 30.72e6 --tx_rate 30.72e6 --rx_channels 0,1 --tx_channels 0,1

Rates: 7.68, 11.52, 15.36, 23.04 and 30.72 MS/s (2R2T), set before the streams are created (the daemon re-runs the
AD9361 bring-up). Limits: one LO frequency for all channels (retune re-runs the AD9361 bring-up, ~1 s, stop
streams first), no async underflow/late messages. Hardware time (timed RX start, timed TX bursts) needs the `base_ts` or
`base_ts_txbuf` bitstream (`P501_BIT`); with the plain `base` bitstream the host clock is used and timed commands are ignored.
2-channel TX messages are 16384 frames (128 KiB) by default (`P501_TXFR` overrides); see ../../README.md and
../../examples/ocudu_gnb/README.md.
