#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/xilinx/p501UHD/notebooks')
from pynq import Overlay
import ad9361_mimo_test as mt
import ad9361_rate_test as rt
ol = Overlay("base/system_top.bit")
ad, a, b, r = mt.bring_up(ol, 700_000_000)
try:
    for n in range(2):
        adc = ol.axi_ad9361.read(0x54); dac = ol.axi_ad9361.read(0x4054)
        print(f"axi_ad9361 clk counters: ADC=0x{adc:X} -> {adc/65536*100:.3f} MHz   DAC=0x{dac:X} -> {dac/65536*100:.3f} MHz   (fs={rt.FS/1e6})")
    for name in ("axi_ad9361_adc_dma", "axi_ad9361_dac_dma"):
        d = getattr(ol, name)
        ver = d.read(0x00); idr = d.read(0x04); desc = d.read(0x10)
        print(f"{name}: version=0x{ver:08X} id={idr} interface_desc=0x{desc:08X} "
              f"bytes_per_beat_log2 dest={(desc>>0)&0xF} src={(desc>>4)&0xF}  type dest={(desc>>8)&3} src={(desc>>10)&3} "
              f"(0=AXI_MM 1=AXI_STREAM 2=FIFO)  dest_w={(desc>>16)&0xF} src_w={(desc>>20)&0xF}")
    print("ip_dict adc dma:", {k: v for k, v in ol.ip_dict["axi_ad9361_adc_dma"].items() if k in ("phys_addr", "addr_range", "type")})
    print("clock dict:", {k: v for k, v in getattr(__import__("pynq").Clocks, "__dict__", {}).items() if k.startswith("fclk")})
    from pynq import Clocks
    print("PS clocks MHz:", Clocks.fclk0_mhz, Clocks.fclk1_mhz, Clocks.fclk2_mhz, Clocks.fclk3_mhz)
finally:
    ad.close()
