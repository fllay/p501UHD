# P501 minimal Linux (RAM-only, boots from SD or QSPI)

A small initramfs Linux for the PZSDR P501 that boots straight into the C board daemon (`kernel/ffs/p501d_c`) and the
USB streamer. No PYNQ, no Python, no Ubuntu rootfs. Prebuilt images are on the GitHub release `minimal-linux-v1`
(`p501-minimal-linux-sd.zip`, `p501-minimal-linux-qspi.zip`).

* Kernel and device tree: the PYNQ 6.6.10-xilinx kernel from `image.ub`, unchanged, command line `rdinit=/init`
  (QSPI controller enabled in the kernel device tree so `/dev/mtd0` exists).
* Rootfs (about 4 MB gzip): BusyBox (Ubuntu noble static), dropbear + libs (Ubuntu **jammy**, the board libc is 2.35),
  `p501d`, `p501_stream`, `usb_f_fs.ko`, bitstream. Replacements for the missing BusyBox applets: `extra/` (pgrep, pkill,
  nohup, mountpoint) and `chrt.c` (static aarch64).
* Boot to ready: about 1 minute. USB-ECM gadget as in the PYNQ image (board 192.168.2.1, host MAC 02:1a:2b:3c:4d:12),
  ssh root/xilinx password `xilinx` (new host key every boot: `-o UserKnownHostsFile=/dev/null`), serial shell on ttyPS0.
* `reboot` does nothing (PID 1 is a shell script): use `reboot -f`.
* An optional `overlay.tar` in the FAT partition of the SD card is unpacked into `/` at boot.

Size: image.ub 15.7 MB + BOOT.BIN 1.8 MB, about 17.5 MB.

## SD card
FAT32 (MBR) card with `BOOT.BIN`, `boot.scr`, `image.ub` in the root, boot mode SD.

## QSPI (n25q256ax1, 32 MB)
U-Boot as shipped cannot use the flash: its control device tree has the QSPI controller disabled, and its default QSPI
script offset (0x3e80000) is beyond 32 MB. `patch_boot_bin.py` fixes both in a copy of BOOT.BIN (enables the node,
moves the offset to 0x1f00000). Flash layout:

| offset | contents |
|---|---|
| 0x000000 | BOOT_qspi.BIN |
| 0x200000 | image.ub (FIT) |
| 0x1f00000 | boot script (`qspi/boot.cmd`: `sf probe; sf read 0x10000000 0x200000 0x1000000; bootm`) |

In QSPI boot mode U-Boot builds its boot order from the boot mode, so an inserted SD card is ignored.
Flashing is done from Linux on the board (see `qspi/flash_qspi.sh`, `qspi/mtd_erase.c`). Back up the old contents first
(`cat /dev/mtd0 > qspi_backup.bin`). Boot modes: QSPI position = this UHD system; SD position = PYNQ or IIO card.

## Rebuild
On an x86 Linux host with `u-boot-tools device-tree-compiler gcc-aarch64-linux-gnu cpio` (see the scripts for paths):
`build_rootfs.sh` (rootfs staging), `build_fit.sh` (FIT with the QSPI-enabled device tree), `patch_boot_bin.py BOOT.BIN BOOT_qspi.BIN`,
`make_qspi_image.py BOOT_qspi.BIN image.ub p501_qspi_32MB.bin`. The scripts expect the working layout used when this was
built (`/home/pi/p501os`: board files in `board/`, debs in `root_src*/`) and are documentation of the procedure more than a
turnkey build.

## Measured (stock OCUDU gNB, 2x2 at 23.04 MS/s, two phones, QSPI boot)
5-minute soak: 0 late-slot warnings (Ubuntu image: 8-11 per 40 s); DL 48.7 + 48.9 Mbit/s, UL 6.8 + 5.5 Mbit/s.
