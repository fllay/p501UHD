#!/bin/bash
# root, on the board: FunctionFS stream session (adds ffs.bulk next to ECM, restores ECM afterwards)
HOLD=${1:-90}; SECS=${2:-40}
G=/sys/kernel/config/usb_gadget/g_ecm
UDC=$(cat $G/UDC)
source /etc/profile.d/xrt_setup.sh
insmod /home/xilinx/p501UHD/kernel/ffs/usb_f_fs.ko 2>/dev/null
echo "" > $G/UDC
mkdir -p $G/functions/ffs.bulk; ln -s $G/functions/ffs.bulk $G/configs/c.1/ffs.bulk
mkdir -p /dev/ffs; mount -t functionfs bulk /dev/ffs
rm -f /tmp/stream_ready
cd /home/xilinx/p501UHD
/usr/local/share/pynq-venv/bin/python3 tools/stream_launcher.py $SECS > /tmp/stream.log 2>&1 &
LPID=$!
for i in $(seq 1 90); do [ -e /tmp/stream_ready ] && break; sleep 1; done
sleep 1
echo $UDC > $G/UDC
sleep $HOLD
pkill -f p501_stream; kill $LPID 2>/dev/null; sleep 1
echo "" > $G/UDC
rm -f $G/configs/c.1/ffs.bulk
umount /dev/ffs 2>/dev/null
rmdir $G/functions/ffs.bulk 2>/dev/null
echo $UDC > $G/UDC
