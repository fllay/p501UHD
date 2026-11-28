#!/usr/bin/env python3
"""make_qspi_image.py BOOT_qspi.BIN image.ub OUT.bin   -> 32 MB flash image
 0x000000 BOOT_qspi.BIN (patch_boot_bin.py output)    0x200000 image.ub (FIT)    0x1f00000 boot script (qspi/boot.cmd)"""
import subprocess, sys, os, tempfile
boot, fit, out = sys.argv[1:4]
here = os.path.dirname(os.path.abspath(__file__))
with tempfile.TemporaryDirectory() as t:
    scr = os.path.join(t, 'boot.scr')
    subprocess.check_call(['mkimage', '-A', 'arm', '-T', 'script', '-C', 'none', '-d', os.path.join(here, 'qspi/boot.cmd'), scr], stdout=subprocess.DEVNULL)
    b, f, s = open(boot, 'rb').read(), open(fit, 'rb').read(), open(scr, 'rb').read()
assert len(b) <= 0x200000 and len(f) <= 0x1000000 and len(s) <= 0x80000   # boot.cmd reads 16 MB at 0x200000
img = bytearray(b'\xff' * 0x2000000)
img[0:len(b)] = b; img[0x200000:0x200000+len(f)] = f; img[0x1f00000:0x1f00000+len(s)] = s
open(out, 'wb').write(img)
print("wrote", out, len(img))
