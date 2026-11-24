#!/bin/bash
# Start the OCUDU gNB on the P501 cell (run on the gNB host, e.g. pi5gnuc). Stops a running gNB first.
#   run_gnb_p501.sh [config.yaml]      env: METRICS=1 adds stdout metrics, P501_DEBUG_TX=1 host TX lead trace to stderr (-> log)
# Do NOT set OCUDU_RX_TO_TX_MS: 10 makes the gNB discard UL slots (RLF on phones).
CFG=${1:-/home/pi/src/5gconfig/p501_1x1_new.yaml}
LOG=${GNB_LOG:-/tmp/gnb.log}
MODDIR=/tmp/uhdmod_auth
mkdir -p $MODDIR && cp /home/pi/projects/p501UHD/host/uhd_p501/build/libuhd_p501.so $MODDIR/
# graceful stop first: killing the gNB while it is still shutting down leaves the board's RX thread blocked (new gNB then gets
# 'exceeded maximum number of timed out receive calls'; fix = restart p501d on the board)
pkill -INT -x gnb
for i in $(seq 1 25); do pgrep -x gnb >/dev/null || break; sleep 1; done
pkill -KILL -x gnb
sleep 12                                   # the board re-enumerates USB when the streams stop; opening it earlier gives NO_DEVICE
ip -br a show enx021a2b3c4d12 | grep -q 192.168.2.10 || echo "warning: host IP 192.168.2.10/24 missing on enx021a2b3c4d12 (sudo ip addr add 192.168.2.10/24 dev enx021a2b3c4d12)"
cd "$(dirname "$CFG")"
EXTRA=""; [ -n "$METRICS" ] && EXTRA="metrics --autostart_stdout_metrics true"
UHD_MODULE_PATH=$MODDIR setsid nohup /home/pi/src/ocudu/build/apps/gnb/gnb -c "$CFG" $EXTRA > "$LOG" 2>&1 < /dev/null &
echo "gnb started, log $LOG"
