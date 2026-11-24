#!/bin/bash
# Open5GS AMF crashes (assertion n1buf in amf_namf_comm_handle_n1_n2_message_transfer) when a phone re-registers during
# session release; the gNB then drops the cell (phone sees -130). Restart the core and the gNB when the AMF is gone.
# Start:  setsid nohup /home/pi/bin/core_watchdog.sh >/dev/null 2>&1 &     Log: /tmp/corewd.log
while true; do
  if ! docker exec open5gs_5gc_l pgrep -f open5gs-amfd >/dev/null 2>&1; then
    echo "$(date +%T) amf down -> restart core + gnb" >> /tmp/corewd.log
    docker restart open5gs_5gc_l >/dev/null 2>&1
    sleep 15
    bash /home/pi/bin/run_gnb_p501.sh
    sleep 30
  fi
  sleep 2
done
