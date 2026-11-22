#!/usr/bin/env python3
"""Continuous RX capture test: can axi_dmac sustain 2R2T at the chosen rate without dropping samples?
usage: rx_stream_test.py [30.72|3.84|61.44]   (env: CHUNKS=120, CHUNK_MB=4)
Digital loopback (chip-internal) carries a TX1 tone; each completed chunk's tone phase must equal the
previous one (chunk length is a multiple of the tone period).  A dropped sample count d shifts the phase by 2*pi*k*d/8192."""
import os, sys, time
sys.path.insert(0, '/home/xilinx/p501UHD/notebooks')
import numpy as np
from pynq import Overlay, allocate
import ad9361_mimo_test as mt
import ad9361_rate_test as rt
from axi_dmac import (AxiDmac, REG_CTRL, REG_TRANSFER_ID, REG_START_TRANSFER, REG_FLAGS, REG_DEST_ADDRESS,
                      REG_X_LENGTH, REG_Y_LENGTH, REG_TRANSFER_DONE, CTRL_ENABLE, FLAG_LAST)

FS = rt.FS
if os.environ.get("FIX2R2T") == "1":       # proper 2R2T 30.72: ADC_CLK 122.88 MHz (=4*fs) -> the 61.44 chain with adc_div 8
    rt.CFG.update(adc_div=8, adc_clk=122_880_000, clk=30_720_000, chain=dict(rt.RATES[61.44]["chain"]))
ANALYZE = os.environ.get("ANALYZE", "0") == "1"
CHUNK = int(float(os.environ.get("CHUNK_MB", "4")) * 1024 * 1024)   # bytes
NCH = int(os.environ.get("CHUNKS", "120"))
RING = 16
BYTES_PER_SAMPLE = 8                      # I1 Q1 I2 Q2, 16 bit each
CS = CHUNK // BYTES_PER_SAMPLE
N = 8192
assert CS % N == 0

ol = Overlay("base/system_top.bit")
ad, rx_ok, tx_ok, res = mt.bring_up(ol, 700_000_000)
print(f"fs={FS/1e6:.2f} MS/s  locks={rx_ok},{tx_ok}  expected data rate {FS*BYTES_PER_SAMPLE/1e6:.1f} MB/s")
try:
    ad.reg_write(0x3F5, ad.reg_read(0x3F5) | 1)           # chip digital loopback
    cyc = round(3e6 * N / FS)
    t = np.exp(2j * np.pi * cyc * np.arange(N) / N) * 0.5 * (2**15 - 1)
    tx = allocate(shape=(N * 4,), dtype=np.int16)
    tx[0::4] = t.real.astype(np.int16); tx[1::4] = t.imag.astype(np.int16); tx[2::4] = 0; tx[3::4] = 0
    tx.flush()
    txd = AxiDmac(ol.axi_ad9361_dac_dma); rxd = AxiDmac(ol.axi_ad9361_adc_dma)
    txd.transfer_from_memory(tx.physical_address, tx.nbytes, cyclic=True)
    ring = allocate(shape=(RING * CHUNK // 2,), dtype=np.int16)
    ref_full = np.exp(-2j * np.pi * cyc * np.arange(CS) / N).astype(np.complex64)
    stats = {}
    dma = ol.axi_ad9361_adc_dma
    dma.write(REG_CTRL, CTRL_ENABLE)

    def submit(slot):
        if dma.read(REG_START_TRANSFER):
            return None                                    # queue full
        dma.write(REG_DEST_ADDRESS, ring.physical_address + slot * CHUNK)
        dma.write(REG_X_LENGTH, CHUNK - 1); dma.write(REG_Y_LENGTH, 0); dma.write(REG_FLAGS, FLAG_LAST)
        tid = dma.read(REG_TRANSFER_ID)
        dma.write(REG_START_TRANSFER, 1)
        return tid

    pending = {}            # tid -> chunk index
    nsub = 0; ndone = 0; phases = []; amps = []; times = []
    t0 = time.time()
    while ndone < NCH:
        while nsub < NCH and len(pending) < 4:
            tid = submit(nsub % RING)
            if tid is None: break
            pending[tid] = nsub; nsub += 1
        m = dma.read(REG_TRANSFER_DONE)
        for tid in list(pending):
            if m & (1 << tid):
                k = pending.pop(tid); times.append(time.time()); ndone += 1
                if ANALYZE and k % 8 in (0, 1):                               # analyse consecutive pairs (k, k+1) every 8 chunks
                    sl = (k % RING) * CHUNK // 2
                    c = (np.asarray(ring[sl:sl + 4 * CS:4], dtype=np.float32) + 1j * np.asarray(ring[sl + 1:sl + 4 * CS:4], dtype=np.float32))
                    ph = np.angle(c * ref_full)                     # tone phase per sample (constant if gap-free)
                    ph0 = float(np.angle(np.mean(c * ref_full)))
                    dev = np.angle(np.exp(1j * (ph - ph0)))
                    stats[k] = (ph0, float(np.max(np.abs(dev))), float(np.std(dev)), float(np.mean(np.abs(c))))
        if time.time() - t0 > 20: print("timeout"); break
    el = time.time() - t0
    mb = ndone * CHUNK / 1e6
    gaps = np.diff(times) * 1e3
    print(f"chunks={ndone}/{NCH}  {mb:.0f} MB in {el:.2f} s = {mb/el:.1f} MB/s  (expected {FS*BYTES_PER_SAMPLE/1e6:.1f})")
    adc_c = ol.axi_ad9361.read(0x54)
    print(f"ADC clock counter 0x{adc_c:X} = {adc_c/65536*100:.2f} MHz (l_clk must be 4*fs = {4*FS/1e6:.2f})")
    ks = sorted(stats) or [0]; stats = stats or {0: (0.0, 0.0, 0.0, 0.0)}; within = [stats[k][1] for k in ks]; wstd = [stats[k][2] for k in ks]
    bounds = [float(np.angle(np.exp(1j * (stats[k + 1][0] - stats[k][0])))) for k in ks if k % 8 == 0 and (k + 1) in stats]
    print(f"analysed {len(ks)} chunks: max in-chunk phase deviation {max(within):.3f} rad (std {max(wstd):.3f}), "
          f"mean |c| {np.mean([stats[k][3] for k in ks]):.0f}; boundary phase steps k->k+1 (rad): "
          + ", ".join(f"{x:+.3f}" for x in bounds))
    print(f"chunk interval ms: min {gaps.min():.1f} median {np.median(gaps):.1f} max {gaps.max():.1f} (ideal {CHUNK/(FS*BYTES_PER_SAMPLE)*1e3:.1f})")
    # spectrum of the last analysed chunk: where is the tone really?
    s0 = ((ndone - 1) % RING) * CHUNK // 2
    for ch, (io, qo) in enumerate(((0, 1), (2, 3))):
        c = np.asarray(ring[s0 + io:s0 + 4 * N:4], dtype=np.float64) + 1j * np.asarray(ring[s0 + qo:s0 + 4 * N:4], dtype=np.float64)
        sp = np.abs(np.fft.fft(c * np.hanning(N)))
        pk = int(np.argmax(sp)); print(f"ch{ch+1}: peak bin {pk} (expected {cyc}) -> apparent tone {np.fft.fftfreq(N, 1/FS)[pk]/1e6:.3f} MHz, level {20*np.log10(sp[pk]+1e-9):.1f} dB, rms {np.std(c):.0f}")
finally:
    try: txd.stop()
    except Exception: pass
    ad.close()
