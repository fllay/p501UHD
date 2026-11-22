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



# X: external-tone test. TX stays idle (DMA never started), RX LO = TX LO = rt.LO, an external CW source
#    (B210) sits ~1 MHz above the LO. Toggle ALERT->FDD and record the external tone level/frequency.
import collections
ad, *_ = rt.bring_up(ol)
try:
    fr = np.fft.fftshift(np.fft.fftfreq(N, 1 / rt.FS))
    rows = []
    for k in range(int(os.environ.get("TOGGLES", "40"))):
        ad.reg_write(REG_ENSM_CONFIG_1, TO_ALERT | FORCE_ALERT_STATE)
        ad.enable_fdd(); fb.axi_ad9361_core_enable(ol.axi_ad9361, num_channels=2)
        a = grab(None); x = a[0::2].astype(float) + 1j * a[1::2].astype(float)
        dc = float(abs(x.mean())); x = x - x.mean()
        m = 20 * np.log10(np.abs(np.fft.fftshift(np.fft.fft(x * np.hanning(N)))) + 1e-9)
        mm = m.copy(); mm[np.abs(fr) < 200e3] = -1e9
        i = int(mm.argmax()); fpk = fr[i]
        sel = np.abs(fr - fpk) < 30e3
        pwr = 10 * np.log10(np.sum(10 ** (m[sel] / 10)))          # integrated tone power (robust to bin position)
        mir = m[np.abs(fr + fpk) < 30e3].max()
        rows.append((m[i], pwr, m[i] - mir, fpk, dc))
    R = np.array(rows)
    np.save(f"{OUT}/X_{RATE}.npy", R)
    print(f"external tone: peak {R[:,0].min():.1f}..{R[:,0].max():.1f} dB, integrated {R[:,1].min():.2f}..{R[:,1].max():.2f} dB "
          f"(spread {R[:,1].max()-R[:,1].min():.2f} dB), mirror rej {R[:,2].min():.1f}..{R[:,2].max():.1f} dB, "
          f"freq {R[:,3].min()/1e3:.1f}..{R[:,3].max()/1e3:.1f} kHz, dc {R[:,4].min():.0f}..{R[:,4].max():.0f}")
    print("   integrated-level clusters:", sorted(collections.Counter(np.round(R[:,1] * 2) / 2).items()))
finally: ad.close()
