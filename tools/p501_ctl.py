#!/usr/bin/env python3
"""Send commands to the board daemon (p501d, TCP :5555) while a UHD program runs, e.g.
  p501_ctl.py "set_rx_gain 0 50" "set_rx_gain 1 50" agc"""
import socket, sys
s = socket.create_connection(("192.168.2.1", 5555), 5); f = s.makefile("rw")
for cmd in sys.argv[1:]:
    f.write(cmd + "\n"); f.flush(); print(cmd, "->", f.readline().strip())
f.write("quit\n"); f.flush()
