#!/usr/bin/env python3
"""patch_boot_bin.py BOOT.BIN OUT.BIN

Make a PYNQ/PetaLinux ZynqMP BOOT.BIN able to boot from QSPI on the P501 (32 MB n25q256ax1):
 1. U-Boot's control device tree (embedded in BOOT.BIN) has the QSPI controller disabled, so `sf probe`
    fails. Enable it and describe the flash (jedec,spi-nor).
 2. U-Boot's default QSPI script offset is 0x3e80000 (beyond 32 MB). Change the immediate
    `movz x1,#0x3e8,lsl#16` to 0x1f0 (script at flash 0x1f00000).
Needs fdtput and dtc (device-tree-compiler)."""
import struct, subprocess, sys, tempfile, os

src, dst = sys.argv[1], sys.argv[2]
d = bytearray(open(src, 'rb').read())

# 1. device tree
pos = -1
while True:
    pos = d.find(b'\xd0\x0d\xfe\xed', pos + 1)
    if pos < 0: sys.exit("no control FDT with the QSPI node found")
    size = struct.unpack('>I', d[pos+4:pos+8])[0]
    if 4096 < size < 0x20000 and b'xlnx,zynqmp-qspi-1.0' in d[pos:pos+size]: break
with tempfile.TemporaryDirectory() as t:
    a, b = os.path.join(t, 'a.dtb'), os.path.join(t, 'b.dtb')
    open(a, 'wb').write(d[pos:pos+size])
    Q = '/axi/spi@ff0f0000'; F = Q + '/flash@0'
    def put(*args): subprocess.check_call(['fdtput'] + list(args))
    put('-t', 's', a, Q, 'status', 'okay')
    for i in range(3):
        subprocess.call(['fdtput', '-r', a, F + '/partition@%d' % i], stderr=subprocess.DEVNULL)
    put('-t', 's', a, F, 'compatible', 'jedec,spi-nor')
    for k, v in (('reg', 0), ('#address-cells', 1), ('#size-cells', 1), ('spi-max-frequency', 40000000),
                 ('spi-rx-bus-width', 4), ('spi-tx-bus-width', 1)):
        put('-t', 'i', a, F, k, str(v))
    subprocess.check_call(['dtc', '-I', 'dtb', '-O', 'dtb', '-q', '-S', str(size), a, '-o', b])
    new = open(b, 'rb').read()
    assert len(new) == size, (len(new), size)
    d[pos:pos+size] = new

# 2. default script offset (unique instruction in U-Boot)
hits = [o for o in range(0, len(d) - 3, 4) if struct.unpack_from('<I', d, o)[0] == 0xD2A07D01]
if len(hits) != 1: sys.exit("expected exactly one 'movz x1,#0x3e8,lsl#16', found %d" % len(hits))
struct.pack_into('<I', d, hits[0], 0xD2A03E01)
open(dst, 'wb').write(d)
print("patched DT at 0x%x (size %d), script offset instruction at 0x%x -> %s" % (pos, size, hits[0], dst))
