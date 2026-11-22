import sys, subprocess
from pynq import allocate
buf = allocate(shape=(32 * 1024 * 1024 // 2,), dtype="int16")
print("phys", hex(buf.physical_address), "cacheable?", getattr(buf, "cacheable", None))
for mode in (0, 1):
    print(subprocess.run(["/home/xilinx/p501UHD/kernel/ffs/membench", hex(buf.physical_address), str(32 * 1024 * 1024), str(mode)], capture_output=True, text=True).stdout.strip())
