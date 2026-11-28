#!/bin/sh
# Run ON THE BOARD (any Linux with /dev/mtd0 = the 32 MB QSPI, e.g. this minimal Linux booted from SD):
#   flash_qspi.sh p501_qspi_32MB.bin   (needs mtd_erase from this directory, static aarch64)
# Writes only the used ranges: 0..0x1200000 (BOOT.BIN + image.ub) and the 64 KiB block at 0x1f00000 (boot script).
# BACK UP FIRST: cat /dev/mtd0 > qspi_backup.bin
set -e
IMG=$1; E=${MTD_ERASE:-/tmp/mtd_erase}; D=/dev/mtd0
[ -s "$IMG" ] || { echo "usage: $0 image.bin"; exit 1; }
$E $D 0 0x1200000
dd if="$IMG" of=$D bs=64k count=288 conv=notrunc
$E $D 0x1f00000 0x10000
dd if="$IMG" of=$D bs=64k skip=496 seek=496 count=1 conv=notrunc
sync
dd if=$D bs=64k count=288 2>/dev/null | sha256sum; dd if="$IMG" bs=64k count=288 2>/dev/null | sha256sum
dd if=$D bs=64k skip=496 count=1 2>/dev/null | sha256sum; dd if="$IMG" bs=64k skip=496 count=1 2>/dev/null | sha256sum
echo "flashed; the two checksum pairs must match"
