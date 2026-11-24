#!/bin/bash
# full_restart.sh [config]: graceful gNB stop, board daemon restart, host IP, gNB start (METRICS=1)
CFG=${1:-/home/pi/src/5gconfig/p501_1x1_new.yaml}
pkill -INT -x gnb; for i in $(seq 1 25); do pgrep -x gnb >/dev/null || break; sleep 1; done; pkill -KILL -x gnb
sleep 8
printf '#!/bin/sh\necho xilinx\n' > /tmp/ap2.sh; chmod +x /tmp/ap2.sh
SSH_ASKPASS=/tmp/ap2.sh SSH_ASKPASS_REQUIRE=force timeout 20 ssh -n -o StrictHostKeyChecking=no xilinx@192.168.2.1 'echo xilinx | sudo -S bash -c "nohup bash /home/xilinx/restart3.sh > /tmp/restart3.out 2>&1 &"' >/dev/null 2>&1
sleep 55
echo pi | sudo -S ip link set enx021a2b3c4d12 up 2>/dev/null
ip -br a show enx021a2b3c4d12 | grep -q 192.168.2.10 || echo pi | sudo -S ip addr add 192.168.2.10/24 dev enx021a2b3c4d12 2>/dev/null
sleep 2; ping -c1 -W2 192.168.2.1 >/dev/null && echo board up || echo BOARD DOWN
METRICS=1 /home/pi/bin/run_gnb_p501.sh "$CFG"
