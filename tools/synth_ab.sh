#!/bin/bash
# A/B test of the RF PLL reference fix (report section 6, "random analog state").
# Run ON THE BOARD, from the deployed folder:   sudo -E bash ~/p501UHD/tools/synth_ab.sh [TOGGLES] [RATES...]
#   default: 40 toggles, rates "3.84 30.72"
# For each rate it runs experiment E of docs/tools/collect_report_data.py (N x ALERT->FDD,
# tone level and image rejection per entry, plus the digital-loopback control) twice:
#   ref80 = new behaviour (vendor: reference doubler on, 80 MHz table)   [default of the driver]
#   ref40 = old behaviour (P501_SYNTH_MAX_FREF=40008000: x1, 40 MHz table)
set -e
HERE=$(cd "$(dirname "$0")/.." && pwd)
TOG=${1:-40}; [ $# -gt 0 ] && shift
RATES=${*:-"3.84 30.72"}
source /etc/profile.d/xrt_setup.sh
PY=/usr/local/share/pynq-venv/bin/python3
OUT=$HERE/results/$(date +%Y%m%d_%H%M%S); mkdir -p "$OUT"
cd "$HERE"
for rate in $RATES; do
  for tag in ref80 ref40; do
    fref=80008000; [ $tag = ref40 ] && fref=40008000
    echo "=== rate $rate MSPS  $tag  (P501_SYNTH_MAX_FREF=$fref, $TOG toggles)"
    P501_SYNTH_MAX_FREF=$fref TOGGLES=$TOG $PY tools/collect_report_data.py E "$rate" 2>&1 | tail -4
    mv /tmp/report/E_$rate.npy "$OUT/E_${rate}_$tag.npy"
  done
done
$PY tools/summarize_ab.py "$OUT" | tee "$OUT/summary.txt"
echo "results in $OUT"
