#!/bin/bash
# root, on the board: FunctionFS gadget next to ECM (kept up) + the p501d daemon (control :5555, USB streams).
G=/sys/kernel/config/usb_gadget/g_ecm
UDC=$(cat $G/UDC)
[ -z "$UDC" ] && UDC=$(ls /sys/class/udc | head -1)
source /etc/profile.d/xrt_setup.sh
insmod /home/xilinx/p501UHD/kernel/ffs/usb_f_fs.ko 2>/dev/null
echo "" > $G/UDC
mkdir -p $G/functions/ffs.bulk; ln -sf $G/functions/ffs.bulk $G/configs/c.1/ffs.bulk
mkdir -p /dev/ffs; mountpoint -q /dev/ffs || mount -t functionfs bulk /dev/ffs
rm -f /tmp/stream_ready
cd /home/xilinx/p501UHD
nohup /usr/local/share/pynq-venv/bin/python3 tools/p501d.py > /tmp/p501d.log 2>&1 &
for i in $(seq 1 120); do [ -e /tmp/stream_ready ] && break; sleep 1; done
sleep 1
echo $UDC > $G/UDC
echo "p501d started (log /tmp/p501d.log)"
