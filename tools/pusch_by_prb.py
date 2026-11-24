#!/usr/bin/env python3
"""Summarise PUSCH SINR / CRC by allocation from a gNB log written with `log: phy_level: info`.
usage: pusch_by_prb.py /tmp/gnb.log
Found the DC-offset problem: every grant containing the centre PRB (25 of 51) reported ~-10 dB SINR."""
import re, sys, statistics as st, collections
d = collections.defaultdict(list)
pat = re.compile(r"PUSCH: rnti=0x\w+ harq_id=\d+ prb=\[(\d+), (\d+)\) symb=\[\d+, \d+\) mod=(\w+) rv=\d+ tbs=(\d+) crc=(\w+).*? sinr=([-\d.]+)dB")
for line in open(sys.argv[1] if len(sys.argv) > 1 else "/tmp/gnb.log", errors="ignore"):
    m = pat.search(line)
    if m:
        a, b, mod, tbs, crc, s = m.groups()
        d[(int(a), int(b))].append((float(s), crc, mod, int(tbs)))
print("range      w     n  med_sinr crcfail%  hasPRB25  mod  tbs")
for (a, b), v in sorted(d.items(), key=lambda kv: -len(kv[1]))[:20]:
    s = [x[0] for x in v]; f = 100 * sum(1 for x in v if x[1] != "OK") / len(v)
    print("[%2d,%2d)  %3d %5d  %7.1f  %6.1f    %-3s     %s %d" % (a, b, b - a, len(v), st.median(s), f,
          "YES" if a <= 25 < b else "no", v[0][2], st.median([x[3] for x in v])))
