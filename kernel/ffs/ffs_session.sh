#!/bin/sh
# Run as root on the board. Adds a FunctionFS bulk function next to ECM, holds it for $1 seconds, then restores ECM only.
HOLD=${1:-240}
G=/sys/kernel/config/usb_gadget/g_ecm
UDC=$(cat $G/UDC)
insmod /home/xilinx/p501UHD/kernel/ffs/usb_f_fs.ko 2>/dev/null
echo "" > $G/UDC
mkdir -p $G/functions/ffs.bulk
ln -s $G/functions/ffs.bulk $G/configs/c.1/ffs.bulk
mkdir -p /dev/ffs; mount -t functionfs bulk /dev/ffs
/home/xilinx/p501UHD/kernel/ffs/ffs_dev /dev/ffs > /tmp/ffs_dev.log 2>&1 &
DPID=$!
sleep 2
echo $UDC > $G/UDC
sleep $HOLD
kill $DPID 2>/dev/null
echo "" > $G/UDC
rm -f $G/configs/c.1/ffs.bulk
umount /dev/ffs 2>/dev/null
rmdir $G/functions/ffs.bulk 2>/dev/null
echo $UDC > $G/UDC
