#!/usr/bin/env python3
"""Collect raw captures for the report figures. usage: collect_report.py <part>
A: DAC datarate 0 vs 1 (comb vs clean) @3.84M     B: tone sweep @3.84M (settled)
C: LO sweep 433/700/915 @3.84M                    D: rates 3.84/30.72/61.44 (settled)
E: ENSM-toggle study: analog tone vs digital loopback @3.84M and @30.72M"""
import sys, os, importlib
PART = sys.argv[1]; RATE = sys.argv[2] if len(sys.argv) > 2 else "3.84"
sys.argv = [sys.argv[0], RATE]
sys.path.insert(0, '/home/xilinx/p501UHD/notebooks')
import numpy as np
from pynq import Overlay, allocate
from axi_dmac import AxiDmac
import ad9361_full_bypass_test as fb
import ad9361_rate_test as rt
from ad9361_driver import REG_ENSM_CONFIG_1, TO_ALERT, FORCE_ALERT_STATE
OUT = "/tmp/report"; os.makedirs(OUT, exist_ok=True)
ol = Overlay("base/system_top.bit")
txd = AxiDmac(ol.axi_ad9361_dac_dma); rxd = AxiDmac(ol.axi_ad9361_adc_dma)
N = rt.N

def make_tx(tgt, fs):
    cyc = round(tgt * N / fs); n = np.arange(N)
    tone = np.exp(2j * np.pi * cyc * n / N)
    tx = allocate(shape=(N * 2,), dtype=np.int16); sc = (2**15 - 1) * 0.5
    tx[0::2] = (tone.real * sc).astype(np.int16); tx[1::2] = (tone.imag * sc).astype(np.int16); tx.flush()
    return tx, cyc * fs / N

def grab(tx=None):
    rx = allocate(shape=(N * 2,), dtype=np.int16)
    if tx is not None: txd.transfer_from_memory(tx.physical_address, tx.nbytes, cyclic=True)
    rxd.transfer_to_memory(rx.physical_address, rx.nbytes); rx.invalidate()
    if tx is not None: txd.stop()
    return np.array(rx).copy()

def spec_peak_rej(raw, fs, f):
    a = raw; x = a[0::2].astype(float) + 1j * a[1::2].astype(float); x -= x.mean()
    fr = np.fft.fftshift(np.fft.fftfreq(N, 1 / fs))
    m = 20 * np.log10(np.abs(np.fft.fftshift(np.fft.fft(x * np.hanning(N)))) + 1e-9)
    p = m[np.abs(fr - f) < 40e3 * (fs / 3.84e6) ** 0.5].max(); i = m[np.abs(fr + f) < 40e3 * (fs / 3.84e6) ** 0.5].max()
    return float(p), float(p - i)

meta = {}
def clean_off(name):
    """true TX-idle baseline: separate bring-up in which the DMA is never started"""
    a0, *_ = rt.bring_up(ol)
    try: np.save(f"{OUT}/{name}.npy", grab(None))
    finally: a0.close()


# R: after each ALERT->FDD toggle, dump all AD9361 registers 0x000-0x3FF and the tone level/rejection,
#    then report which registers vary across toggles and how they line up with the tone level.
import collections
ad, *_ = rt.bring_up(ol)
try:
    tx, f = make_tx(rt.CFG["tones"][0], rt.FS)
    rows = []; dumps = []
    for k in range(int(os.environ.get("TOGGLES", "24"))):
        ad.reg_write(REG_ENSM_CONFIG_1, TO_ALERT | FORCE_ALERT_STATE)
        ad.enable_fdd(); fb.axi_ad9361_core_enable(ol.axi_ad9361, num_channels=2)
        d = [ad.reg_read(a) for a in range(0x400)]
        p, r = spec_peak_rej(grab(tx), rt.FS, f)
        rows.append((k, p, r)); dumps.append(d)
    np.save(f"{OUT}/R_{RATE}_rows.npy", np.array(rows)); np.save(f"{OUT}/R_{RATE}_regs.npy", np.array(dumps))
    lv = np.array([r[1] for r in rows]); rj = np.array([r[2] for r in rows])
    thr = (lv.max() + lv.min()) / 2
    print(f"rate {RATE}: tone {lv.min():.1f}..{lv.max():.1f}, rej {rj.min():.1f}..{rj.max():.1f}, split at {thr:.1f}")
    D = np.array(dumps)
    for a in range(0x400):
        col = D[:, a]
        if len(set(col)) > 1:
            hi = collections.Counter(col[lv >= thr].tolist()); lo = collections.Counter(col[lv < thr].tolist())
            print(f"reg 0x{a:03X}: high-level {dict(hi)}  low-level {dict(lo)}  rej-corr {np.corrcoef(col, rj)[0,1] if col.std() else 0:+.2f}")
finally: ad.close()
