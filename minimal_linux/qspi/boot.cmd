echo P501 QSPI boot: reading image from flash
sf probe 0 0 0
sf read 0x10000000 0x200000 0x1000000
bootm 0x10000000
