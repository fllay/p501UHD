#!/bin/bash
# sweep.sh g1 g2 ...: per RX gain, UL iperf on all phones (12 s) + the board's level line in the middle of the run
B() { SSH_ASKPASS=/tmp/ap2.sh SSH_ASKPASS_REQUIRE=force timeout 15 ssh -n -o StrictHostKeyChecking=no xilinx@192.168.2.1 "$@"; }
for g in "$@"; do
  python3 /home/pi/bin/p501_ctl.py "set_rx_gain 0 $g" "set_rx_gain 1 $g" >/dev/null; sleep 6
  UP=""; for i in 2 3; do ping -c1 -W1 10.45.0.$i >/dev/null 2>&1 && UP="$UP 10.45.0.$i"; done
  idle=$(B cat /tmp/p501_rxlvl)
  for ip in $UP; do (timeout 20 iperf3 -c $ip -R -t 12 -f m 2>&1 | grep receiver | tail -1 | awk '{print $7}' > /tmp/ul_$ip) & done
  sleep 7; load=$(B "cat /tmp/p501_rxlvl; sleep 1; cat /tmp/p501_rxlvl; sleep 1; cat /tmp/p501_rxlvl"); wait
  tot=0; for ip in $UP; do tot=$(echo "$tot + $(cat /tmp/ul_$ip)" | bc); done
  echo "gain $g UL total $tot Mbit/s (phones:$UP) | idle: $idle"
  echo "$load" | awk '{printf "   load p95 %s p50 %s max %s dBFS clip %.2e\n", $3, $4, $5, $6/($7+1)}'
done
