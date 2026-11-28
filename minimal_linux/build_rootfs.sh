set -e
cd /home/pi/p501os
which dumpimage || true
R=rootfs; rm -rf $R; mkdir -p $R/{bin,sbin,usr/bin,usr/sbin,lib/aarch64-linux-gnu,lib/firmware,proc,sys,dev,tmp,run,etc,root,var/run,home/xilinx/p501UHD/kernel/ffs/p501d_c,home/xilinx/p501UHD/base_ts_txbuf,sys/kernel/config}
cp root_src/usr/bin/busybox $R/bin/busybox
for a in $(qemu-aarch64 --version >/dev/null 2>&1 && echo) ; do :; done
# applet links: take the list from the host's busybox (same version)
for a in $(busybox --list); do case $a in busybox) ;; *) ln -sf /bin/busybox $R/bin/$a;; esac; done
ln -sf /bin/busybox $R/bin/bash
mkdir -p $R/lib; cp board/ld-linux-aarch64.so.1 $R/lib/
cp board/libc.so.6 board/libm.so.6 board/libcrypt.so.1 $R/lib/aarch64-linux-gnu/
cp -a root_src_j/usr/lib/aarch64-linux-gnu/*.so.* root_src_j/lib/aarch64-linux-gnu/libz.so.1* $R/lib/aarch64-linux-gnu/
cp root_src_j/usr/sbin/dropbear $R/usr/sbin/
F=$R/home/xilinx/p501UHD
cp board/p501d $F/kernel/ffs/p501d_c/; cp board/p501_stream board/usb_f_fs.ko $F/kernel/ffs/
sed '1s|.*|#!/bin/sh|' board/p501d_start.sh > $F/kernel/ffs/p501d_start.sh; sed '1s|.*|#!/bin/sh|' board/p501d_stop.sh > $F/kernel/ffs/p501d_stop.sh
cp board/system_top.bit board/system_top.hwh $F/base_ts_txbuf/
sed '1s|.*|#!/bin/sh|' board/usb-gadget-ecm.sh > $R/usr/bin/usb-gadget-ecm.sh
chmod +x $F/kernel/ffs/*.sh $R/usr/bin/usb-gadget-ecm.sh
cat > $R/etc/passwd <<E
root:x:0:0:root:/root:/bin/sh
xilinx:x:0:0:xilinx:/home/xilinx:/bin/sh
E
H=$(openssl passwd -6 xilinx)
printf 'root:%s:19000:0:99999:7:::\nxilinx:%s:19000:0:99999:7:::\n' "$H" "$H" > $R/etc/shadow; chmod 600 $R/etc/shadow
echo "root:x:0:" > $R/etc/group; echo /bin/sh > $R/etc/shells
echo p501 > $R/etc/hostname
cat > $R/usr/bin/sudo <<'E'
#!/bin/sh
# minimal sudo stand-in (everything already runs as root); accepts -S and swallows the password line
[ "$1" = -S ] && { read -r _; shift; }
exec "$@"
E
chmod +x $R/usr/bin/sudo
cat > $R/etc/p501.env <<'E'
export P501_TXP_KIB=128 P501_BIT=/home/xilinx/p501UHD/base_ts_txbuf/system_top.bit P501_STREAM_DEBUG=1 P501_AGC=1
export P501_TRACKING=1 P501_SYNTH_MAX_FREF=80008000
E
cat > $R/init <<'E'
#!/bin/sh
export PATH=/bin:/sbin:/usr/bin:/usr/sbin
mount -t proc proc /proc; mount -t sysfs sysfs /sys
mount -t devtmpfs devtmpfs /dev 2>/dev/null
mkdir -p /dev/pts; mount -t devpts devpts /dev/pts
mount -t tmpfs tmpfs /tmp; mount -t tmpfs tmpfs /run; mkdir -p /run/dropbear /var
ln -sf /run /var/run
mount -t configfs none /sys/kernel/config
# optional overlay from the SD card FAT partition (lets files be updated without rebuilding the image)
mkdir -p /mnt/sd; if mount -t vfat -o ro /dev/mmcblk0p1 /mnt/sd 2>/dev/null; then [ -f /mnt/sd/overlay.tar ] && tar xf /mnt/sd/overlay.tar -C / ; umount /mnt/sd; fi
hostname p501
echo "p501 minimal linux: $(uname -r)" > /dev/console
# USB-ECM gadget (same ids/MACs as the PYNQ image) and static address
/usr/bin/usb-gadget-ecm.sh
ip link set lo up
ip addr add 192.168.2.1/24 dev usb0; ip link set usb0 up
# ssh (root / xilinx, password xilinx)
mkdir -p /etc/dropbear; /usr/sbin/dropbear -R -B 2>/dev/null || /usr/sbin/dropbear -R
# serial console shell
setsid sh -c 'exec sh </dev/ttyPS0 >/dev/ttyPS0 2>&1' &
# P501 daemon + streamer
. /etc/p501.env
sleep 1
sh /home/xilinx/p501UHD/kernel/ffs/p501d_start.sh > /tmp/start.out 2>&1
chrt -f -p 90 $(pgrep irq/34-dwc3) 2>/dev/null
while :; do sleep 3600; done
E
chmod +x $R/init
for a in mountpoint nohup pgrep pkill chrt; do rm -f $R/bin/$a; done
cp extra/* $R/usr/bin/; cp chrt $R/usr/bin/chrt; chmod +x $R/usr/bin/*
echo built; du -sh $R; du -sh $R/* | sort -h | tail -5
