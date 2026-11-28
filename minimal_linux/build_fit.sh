set -e
cd /home/pi/p501os; rm -rf fit; mkdir fit; cd fit
dumpimage -T flat_dt -p 0 -o Image ../../image.ub >/dev/null
dumpimage -T flat_dt -p 1 -o system.dtb ../../image.ub >/dev/null
ls -l Image system.dtb
fdtget system.dtb /chosen bootargs
fdtput -t s system.dtb /chosen bootargs 'earlycon console=ttyPS0,115200 clk_ignore_unused cpufreq.off=1 uio_pdrv_genirq.of_id="generic-uio" rdinit=/init'
fdtget system.dtb /chosen bootargs
Q=/axi/spi@ff0f0000
fdtput -t s system.dtb $Q status okay
for i in 0 1 2; do fdtput -r system.dtb $Q/flash@0/partition@$i; done
fdtput -t s system.dtb $Q/flash@0 compatible jedec,spi-nor
fdtput -t i system.dtb $Q/flash@0 reg 0
fdtput -t i system.dtb $Q/flash@0 '#address-cells' 1
fdtput -t i system.dtb $Q/flash@0 '#size-cells' 1
fdtput -t i system.dtb $Q/flash@0 spi-max-frequency 40000000
fdtput -t i system.dtb $Q/flash@0 spi-rx-bus-width 4
fdtput -t i system.dtb $Q/flash@0 spi-tx-bus-width 1
gzip -9 -k Image
(cd ../rootfs && find . | cpio -o -H newc -R 0:0 2>/dev/null) | gzip -9 > rootfs.cpio.gz
ls -l Image.gz rootfs.cpio.gz
cat > p501.its <<E
/dts-v1/;
/ { description = "P501 minimal Linux"; #address-cells = <1>;
 images {
  kernel-0 { description="Linux"; data=/incbin/("Image.gz"); type="kernel"; arch="arm64"; os="linux"; compression="gzip"; load=<0x00080000>; entry=<0x00080000>; hash-1 { algo="crc32"; }; };
  fdt-0 { description="DTB"; data=/incbin/("system.dtb"); type="flat_dt"; arch="arm64"; compression="none"; hash-1 { algo="crc32"; }; };
  ramdisk-0 { description="initramfs"; data=/incbin/("rootfs.cpio.gz"); type="ramdisk"; arch="arm64"; os="linux"; compression="none"; load=<0x04000000>; hash-1 { algo="crc32"; }; };
 };
 configurations { default="conf-0"; conf-0 { description="p501"; kernel="kernel-0"; fdt="fdt-0"; ramdisk="ramdisk-0"; }; };
};
E
mkimage -f p501.its image.ub >/dev/null
cat > boot.cmd <<E
echo P501 minimal Linux: loading image.ub
fatload \${devtype} \${devnum}:\${distro_bootpart} 0x10000000 image.ub
bootm 0x10000000
E
mkimage -A arm -T script -C none -d boot.cmd boot.scr >/dev/null
ls -l image.ub boot.scr
