#!/usr/bin/env python3
"""Board side: AD9361 2R2T @30.72 MS/s (true l_clk=4*fs chain) with the chip's RX BIST tone (fs/32) as a continuous
reference, a cached CMA ring, and the C streamer that feeds the FunctionFS bulk endpoint.
usage: stream_launcher.py [secs=40] [slot_mb=4] [nslots=16]"""
import os, sys, subprocess, time
secs = sys.argv[1] if len(sys.argv) > 1 else "40"
slot_mb = int(sys.argv[2]) if len(sys.argv) > 2 else 4
nslots = int(sys.argv[3]) if len(sys.argv) > 3 else 16
sys.argv = [sys.argv[0], os.environ.get("RATE", "30.72")]
sys.path.insert(0, '/home/xilinx/p501UHD/notebooks')
from pynq import Overlay, allocate
import ad9361_mimo_test as mt
ol = Overlay("base/system_top.bit")
ad, rx_ok, tx_ok, res = mt.bring_up(ol, 700_000_000)
print(f"fs={mt.FS/1e6} locks={rx_ok},{tx_ok}", flush=True)
ad.reg_write(0x3F6, 0)
ad.reg_write(0x3F4, (2 << 2) | 1 | (1 << 1))            # BIST: control point 2 (RX out), enable, tone, level 0, freq fs/32
ring = allocate(shape=(nslots * slot_mb * 1024 * 1024 // 2,), dtype="int16")
dma_phys = ol.ip_dict["axi_ad9361_adc_dma"]["phys_addr"]
print(f"ring phys {ring.physical_address:#x} dma {dma_phys:#x}", flush=True)
try:
    subprocess.run(["/home/xilinx/p501UHD/kernel/ffs/p501_stream", "/dev/ffs", hex(ring.physical_address), str(slot_mb * 1024 * 1024),
                    str(nslots), hex(dma_phys), secs])
finally:
    ad.reg_write(0x3F4, 0)
    ad.close()
