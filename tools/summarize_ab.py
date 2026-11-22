"""Summarise the A/B runs: levels and image rejection over repeated ALERT->FDD entries."""
import glob, os, sys
import numpy as np


def clusters(v, gap=1.5):
    v = np.sort(np.asarray(v)); out = [[v[0]]]
    for x in v[1:]:
        if x - out[-1][-1] <= gap:
            out[-1].append(x)
        else:
            out.append([x])
    return [(round(float(np.mean(c)), 1), len(c)) for c in out]


d = sys.argv[1]
print("%-22s %5s  %-17s %-17s %-8s %s" % ("run", "n", "tone peak dB", "image rej dB", ">=50 dB", "level clusters (dB, count)"))
for f in sorted(glob.glob(os.path.join(d, "E_*.npy"))):
    r = np.load(f)                      # columns: k, peak, rej, loop_peak, loop_rej
    name = os.path.basename(f)[2:-4]
    print("%-22s %5d  %6.1f .. %-8.1f %5.1f .. %-8.1f %3d/%-3d  %s" % (
        name, len(r), r[:, 1].min(), r[:, 1].max(), r[:, 2].min(), r[:, 2].max(),
        int((r[:, 2] >= 50).sum()), len(r), clusters(r[:, 1])))
