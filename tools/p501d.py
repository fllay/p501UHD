#!/usr/bin/env python3
"""P501 board daemon: AD9361 2R2T @30.72 MS/s, the C streamer (p501_stream, USB FunctionFS) and a line-based TCP
control server for the UHD module.  Runs as root from p501d_start.sh (which sets up the FunctionFS gadget).

Protocol: one request per line, 'cmd arg...'; reply 'OK <tokens>' or 'ERR <message>'.
  info                      -> OK p501 <version> fs=<Hz> rx=2 tx=2 serial=<s>
  get_rx_freq|get_tx_freq c -> OK <Hz>            set_rx_freq|set_tx_freq c <Hz> -> OK <actual Hz>
  get_rx_gain|get_tx_gain c -> OK <dB>            set_rx_gain|set_tx_gain c <dB> -> OK <actual dB>
  get_*_gain_range c        -> OK <min> <max> <step>
  lo_locked                 -> OK 1|0
  loopback 0|1              -> OK <0|1>   (AD9361 digital loopback TX->RX, a test aid)
  quit
v1 limits: both LOs always share one frequency (a retune re-runs the full bring-up, a few seconds, streams must be
stopped), one sample rate (30.72 MS/s, 2R2T)."""
import os, sys, socket, threading, subprocess, time
RATE = 30.72
sys.argv = [sys.argv[0], str(RATE)]
sys.path.insert(0, '/home/xilinx/p501UHD/notebooks')
from pynq import Overlay, allocate
import ad9361_mimo_test as mt

PORT = int(os.environ.get("P501_PORT", "5555"))
BIT = os.environ.get("P501_BIT", "base_ts/system_top.bit")
SLOT_MB, NSLOTS, TX_NSLOTS = 4, 16, 8
VERSION = "2"
TIME_PHYS = int(os.environ.get("P501_TIME_PHYS", "0x99030000"), 0) if "base_ts" in BIT else 0
RX_GAIN_MAX, TX_GAIN_MAX = 76.0, 89.75
LO_MIN, LO_MAX = 70e6, 6000e6


class Board:
    def __init__(self):
        self.lock = threading.RLock()
        self.ol = Overlay(BIT)
        self.ad = None
        self.rx_gain = [40.0, 40.0]; self.tx_gain = [TX_GAIN_MAX - 30.0] * 2     # TX atten 30 dB
        self.lo = None; self.locked = False; self.loop = False
        self.retune(700e6)
        self.ring = allocate(shape=(NSLOTS * SLOT_MB * 1024 * 1024 // 2,), dtype="int16")
        self.txring = allocate(shape=(TX_NSLOTS * SLOT_MB * 1024 * 1024 // 2,), dtype="int16")
        self.txring[:] = 0

    def start_streamer(self):
        o = self.ol.ip_dict
        self.proc = subprocess.Popen(
            ["/home/xilinx/p501UHD/kernel/ffs/p501_stream", "/dev/ffs", hex(self.ring.physical_address),
             str(SLOT_MB * 1024 * 1024), str(NSLOTS), hex(o["axi_ad9361_adc_dma"]["phys_addr"]), "0",
             hex(o["axi_ad9361"]["phys_addr"]), hex(self.txring.physical_address),
             hex(o["axi_ad9361_dac_dma"]["phys_addr"]), str(TX_NSLOTS)] + ([hex(TIME_PHYS)] if TIME_PHYS else []))

    def apply(self):
        for i in (0, 1):
            self.ad.set_rx_manual_gain(int(round(self.rx_gain[i])), channels=(i + 1,))
            self.ad.set_tx_atten_db(TX_GAIN_MAX - self.tx_gain[i], channels=(i + 1,))
        self.ad.reg_write(0x3F4, 0)                                   # no BIST tone
        r = self.ad.reg_read(0x3F5)
        self.ad.reg_write(0x3F5, (r | 1) if self.loop else (r & ~1))

    def retune(self, f):
        with self.lock:
            f = int(round(f))
            if f == self.lo:
                return
            if self.ad is not None:
                self.ad.close()
            ad, rx_ok, tx_ok, res = mt.bring_up(self.ol, f)
            self.ad, self.lo, self.locked = ad, f, bool(rx_ok and tx_ok)
            self.apply()
            print(f"retuned to {f/1e6:.3f} MHz locked={self.locked}", flush=True)

    def handle(self, line):
        t = line.split()
        if not t:
            return "ERR empty"
        c = t[0]
        ch = int(t[1]) if len(t) > 1 and t[1].lstrip("-").isdigit() else 0
        if c == "info":
            return f"OK p501 {VERSION} fs={int(mt.FS)} rx=2 tx=2 time={1 if TIME_PHYS else 0} serial=P501-{os.uname().nodename}"
        if c in ("get_rx_freq", "get_tx_freq"):
            return f"OK {self.lo}"
        if c in ("set_rx_freq", "set_tx_freq"):
            f = float(t[2])
            if not LO_MIN <= f <= LO_MAX:
                return f"ERR frequency {f} outside {LO_MIN}..{LO_MAX}"
            self.retune(f)
            return f"OK {self.lo}"
        if c == "get_rx_gain":
            return f"OK {self.rx_gain[ch]}"
        if c == "get_tx_gain":
            return f"OK {self.tx_gain[ch]}"
        if c == "set_rx_gain":
            with self.lock:
                self.rx_gain[ch] = float(min(RX_GAIN_MAX, max(0.0, round(float(t[2])))))
                self.ad.set_rx_manual_gain(int(self.rx_gain[ch]), channels=(ch + 1,))
            return f"OK {self.rx_gain[ch]}"
        if c == "set_tx_gain":
            with self.lock:
                g = min(TX_GAIN_MAX, max(0.0, float(t[2])))
                self.tx_gain[ch] = round(g * 4) / 4
                self.ad.set_tx_atten_db(TX_GAIN_MAX - self.tx_gain[ch], channels=(ch + 1,))
            return f"OK {self.tx_gain[ch]}"
        if c == "get_rx_gain_range":
            return f"OK 0 {RX_GAIN_MAX} 1"
        if c == "get_tx_gain_range":
            return f"OK 0 {TX_GAIN_MAX} 0.25"
        if c == "lo_locked":
            return f"OK {int(self.locked)}"
        if c == "loopback":
            with self.lock:
                self.loop = t[1] == "1"
                self.apply()
            return f"OK {int(self.loop)}"
        return f"ERR unknown command {c}"


def serve(board):
    srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", PORT)); srv.listen(4)
    print(f"control on :{PORT}", flush=True)

    def client(sock):
        f = sock.makefile("rw", newline="\n")
        for line in f:
            line = line.strip()
            if line == "quit":
                break
            try:
                r = board.handle(line)
            except Exception as e:                                   # keep serving
                r = f"ERR {type(e).__name__}: {e}"
            f.write(r + "\n"); f.flush()
        sock.close()

    while True:
        s, _ = srv.accept()
        threading.Thread(target=client, args=(s,), daemon=True).start()


if __name__ == "__main__":
    b = Board()
    b.start_streamer()
    threading.Thread(target=lambda: (time.sleep(1.0), None), daemon=True).start()
    serve(b)
