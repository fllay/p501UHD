#!/bin/bash
# NOTE: run this from outside /tmp (e.g. /home/xilinx/restart3.sh); /tmp is replaced by tmpfs.
# On the P501 board, as root: restart p501d + streamer with the deep-DAC-buffer bitstream and debug output.
# The USB link drops for ~1 min; afterwards re-add the host IP on the gNB host:
#   sudo ip link set enx021a2b3c4d12 up; sudo ip addr add 192.168.2.10/24 dev enx021a2b3c4d12
bash /home/xilinx/p501UHD/kernel/ffs/p501d_stop.sh
sleep 3
# keep logs and flags on RAM: the SD card (ext4, 5 s journal commit) stalls the streamer for 20-30 ms about every 4.6 s -> TX underflows
mount | grep -q "tmpfs on /tmp " || mount -t tmpfs -o size=256m tmpfs /tmp
export P501_BIT=${P501_BIT:-/home/xilinx/p501UHD/base_ts_txbuf/system_top.bit}
export P501_STREAM_DEBUG=1
export P501_TRACKING=1   # AD9361 RX DC-offset + quadrature tracking (off by default in the driver)
# TX slot size in KiB: must be >= the host's TX message (uhd_p501 sends 128 KiB). The TX ring holds 8 x 4 MiB / slot messages, so a
# short slot buys more TX buffering in time (1 slot = 0.53 ms of 2-channel samples at 128 KiB); a 512 KiB slot gave only 34 ms for short messages
export P501_TXP_KIB=${P501_TXP_KIB:-128}
bash /home/xilinx/p501UHD/kernel/ffs/p501d_start.sh
# the USB (dwc3) interrupt thread runs at RT priority 50 on CPU0, below the streamer threads: raise it
sleep 4
chrt -f -p 90 $(pgrep irq/34-dwc3) 2>/dev/null
