#!/bin/bash
# root, on the board: stop p501d + streamer and return the gadget to ECM only.
G=/sys/kernel/config/usb_gadget/g_ecm
UDC=$(cat $G/UDC); [ -z "$UDC" ] && UDC=$(ls /sys/class/udc | head -1)
pkill -f p501_stream; pkill -f tools/p501d.py; sleep 1
echo "" > $G/UDC
rm -f $G/configs/c.1/ffs.bulk
umount /dev/ffs 2>/dev/null
rmdir $G/functions/ffs.bulk 2>/dev/null
echo $UDC > $G/UDC
