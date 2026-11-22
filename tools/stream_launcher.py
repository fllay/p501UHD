#!/usr/bin/env python3
"""Board side: AD9361 2R2T @30.72 MS/s (true l_clk=4*fs chain), cached CMA rings for RX and TX, and the C streamer
(p501_stream) that serves the FunctionFS bulk endpoints (RX over EP1 IN, TX over EP2 OUT, vendor control on EP0).
Env: P501_BIT (overlay, default base/system_top.bit), P501_LOOPBACK=1 (AD9361 digital loopback TX->RX, no BIST tone),
     RATE.  Default (no loopback): RX BIST tone fs/32 as a continuous reference.
usage: stream_launcher.py [secs=40] [slot_mb=4] [nslots=16] [tx_nslots=8]"""
import os, sys, subprocess
secs = sys.argv[1] if len(sys.argv) > 1 else "40"
slot_mb = int(sys.argv[2]) if len(sys.argv) > 2 else 4
nslots = int(sys.argv[3]) if len(sys.argv) > 3 else 16
tx_nslots = int(sys.argv[4]) if len(sys.argv) > 4 else 8
sys.argv = [sys.argv[0], os.environ.get("RATE", "30.72")]
sys.path.insert(0, '/home/xilinx/p501UHD/notebooks')
from pynq import Overlay, allocate
import ad9361_mimo_test as mt
ol = Overlay(os.environ.get("P501_BIT", "base/system_top.bit"))
ad, rx_ok, tx_ok, res = mt.bring_up(ol, 700_000_000)
print(f"fs={mt.FS/1e6} locks={rx_ok},{tx_ok}", flush=True)
ad.reg_write(0x3F6, 0)
if os.environ.get("P501_LOOPBACK") == "1":
    ad.reg_write(0x3F4, 0)                                      # no BIST tone
    ad.reg_write(0x3F5, ad.reg_read(0x3F5) | 1)                 # digital loopback TX -> RX
    print("digital loopback ON", flush=True)
else:
    ad.reg_write(0x3F4, (2 << 2) | 1 | (1 << 1))                # BIST: control point 2 (RX out), enable, tone, level 0, freq fs/32
ring = allocate(shape=(nslots * slot_mb * 1024 * 1024 // 2,), dtype="int16")
txring = allocate(shape=(tx_nslots * slot_mb * 1024 * 1024 // 2,), dtype="int16")
txring[:] = 0
adc = ol.ip_dict["axi_ad9361_adc_dma"]["phys_addr"]; dac = ol.ip_dict["axi_ad9361_dac_dma"]["phys_addr"]
print(f"rx ring {ring.physical_address:#x} tx ring {txring.physical_address:#x} adc dma {adc:#x} dac dma {dac:#x}", flush=True)
try:
    subprocess.run(["/home/xilinx/p501UHD/kernel/ffs/p501_stream", "/dev/ffs", hex(ring.physical_address), str(slot_mb * 1024 * 1024),
                    str(nslots), hex(adc), secs, hex(ol.ip_dict["axi_ad9361"]["phys_addr"]),
                    hex(txring.physical_address), hex(dac), str(tx_nslots)])
finally:
    ad.reg_write(0x3F4, 0)
    ad.reg_write(0x3F5, ad.reg_read(0x3F5) & ~1)
    ad.close()
