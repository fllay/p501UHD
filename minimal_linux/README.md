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

## Step-by-step: from power-on to a running gNB

Assumptions: the gNB host is an x86 Linux box (the author's is an i5-1340P NUC) with UHD 4.6 development files, libusb-1.0, CMake and
Docker; OCUDU (stock commit `bd0d2cd27c`) is built; Open5GS runs in a container; phones have a SIM in the core's subscriber DB.
Board and host are joined by one USB 3 cable (the board is the USB device).

### 1. Boot the board
1. Choose the boot source with the boot-mode switch: **QSPI** (flash written from the QSPI release zip, no SD card needed) or **SD**
   (card prepared from the SD release zip). Other cards (PYNQ, IIO) simply use the SD position with their own card.
2. Connect the USB 3 cable to the host, power the board. After about 1 minute the board is ready: the FPGA is loaded,
   `p501d` and the streamer are running, and the USB gadget is up.
3. On the host the board appears as a network interface named from its MAC: `enx021a2b3c4d12`. Give the host side its address
   (this has to be redone after every host reboot):
   ```
   sudo ip link set enx021a2b3c4d12 up
   sudo ip addr add 192.168.2.10/24 dev enx021a2b3c4d12
   ping -c2 192.168.2.1                      # the board
   ssh -o UserKnownHostsFile=/dev/null xilinx@192.168.2.1     # password: xilinx (optional: look around)
   ```
   The board's ssh host key changes at every boot, hence `UserKnownHostsFile=/dev/null`. Daemon log on the board: `/tmp/p501d.log`;
   `tail /tmp/p501d.log` should end with `descriptors written` and `endpoints open`.
   There is no `sudo` password and nothing to start by hand: everything starts from the board's `/init`.

### 2. Prepare the host (once, then after every host reboot)
```
# real-time threads miss deadlines under powersave
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
# libusb buffers: the module warns below this
echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
# route to the phones' address pool through the Open5GS container (adjust the gateway to your core)
sudo ip route add 10.45.0.0/24 via 10.53.1.2
```
Build the UHD module once (it is a UHD plug-in, `libuhd_p501.so`):
```
cmake -S host/uhd_p501 -B host/uhd_p501/build -DUHD_ROOT=/usr/local && cmake --build host/uhd_p501/build -j
export UHD_MODULE_PATH=$PWD/host/uhd_p501/build
uhd_find_devices --args type=p501                 # must list the board
```

### 3. Start the core
Start the Open5GS container with `examples/ocudu_gnb/open5gs-5gc.yml` (AMF 10.53.1.2) as described in
`examples/ocudu_gnb/README.md`, step 4. Check the AMF is up before starting the gNB.

### 4. Start the gNB
1. Copy `examples/ocudu_gnb/gnb_p501_2x2_n78_20mhz_2304.yaml` (2x2, 23.04 MS/s, the setup measured here) and set
   `cu_cp.amf.addr` / `bind_addr` for your core and host. Keep `tx_advance_samples=80640` and `time_alignment_calibration: -80640`
   equal and opposite.
2. Launch it. `P501_RXMAXSAMPS=5760` is **required at 23.04 MS/s** (small RX buffer; without it the log fills with `PUxCH request late`):
   ```
   P501_RXMAXSAMPS=5760 METRICS=1 bash tools/run_gnb_p501.sh examples/ocudu_gnb/gnb_p501_2x2_n78_20mhz_2304.yaml
   ```
   `tools/run_gnb_p501.sh` stops a previous gNB gracefully and waits 12 s for the board's USB to re-enumerate (opening it earlier
   gives NO_DEVICE). Edit the paths at the top for your OCUDU build and interface name.
3. About 30 s later, pin the real-time threads to their own cores (the script is written for an i5-1340P: P-cores 0-7, E-cores 8-15;
   edit the core numbers for another CPU):
   ```
   bash tools/pin_gnb.sh
   ```
4. Watch `/tmp/gnb.log`: a healthy run has no `late` / `Discarded uplink slot` lines (a few per minute at most).

### 5. Attach phones and test
Switch the phones off airplane mode (PLMN 00101, band n78, 3489.42 MHz), wait for the registration (`ping 10.45.0.x` works), run
`iperf3 -s` on the phone (Termux or an iperf app) and from the core host:
```
iperf3 -c <phone-ip> -t 20          # downlink
iperf3 -c <phone-ip> -t 20 -R       # uplink
```
With the board's AGC on (default) the receive gain follows the transmitting phone; `rx_gain` in the YAML is only the start value.
Measured on this setup, two phones, 5 minutes: DL about 49 + 49 Mbit/s, UL 6.8 + 5.5 Mbit/s, 0 late warnings.

### 6. Stop and restart
* Stop the gNB with `pkill -INT -x gnb` (graceful; killing it hard can leave the board's RX thread blocked).
* A clean board restart is `ssh ... xilinx@192.168.2.1 'reboot -f'` (about 1 minute; `reboot` without `-f` does nothing here). Then redo
  the host IP (step 1.3) if the interface went away, and start the gNB again. Do not use `tools/board_restart_p501d.sh` or
  `tools/full_restart_gnb.sh`: they are for the PYNQ image.
* Phones that cannot find the cell after a restart: check the host IP/route, restart the gNB, toggle airplane mode on the phone.

### Troubleshooting
| Symptom | Check |
|---|---|
| no `enx021a2b3c4d12` on the host | USB 3 cable and port; board powered; boot-mode switch; `lsusb` shows the board |
| `uhd_find_devices` finds nothing / NO_DEVICE | host IP set; wait 12 s after stopping a gNB; `UHD_MODULE_PATH` |
| thousands of `late` warnings | governor `performance`; `P501_RXMAXSAMPS=5760`; run `tools/pin_gnb.sh`; `usbfs_memory_mb` 256 |
| phones attach but no traffic | route to 10.45.0.0/24; core container running; AMF alive (`tools/core_watchdog.sh`) |
| board does not come up on QSPI | SD card out or not, switch position, serial console 115200 baud on the board's UART: U-Boot should print `Bootmode: QSPI_MODE` then `P501 QSPI boot` |

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
