#!/bin/bash
# A/B: vendor tracking_control() after calibration (P501_TRACKING=1) vs driver default (off).
set -e
HERE=$(cd "$(dirname "$0")/.." && pwd)
TOG=${1:-40}; [ $# -gt 0 ] && shift
RATES=${*:-"3.84 30.72"}
source /etc/profile.d/xrt_setup.sh
PY=/usr/local/share/pynq-venv/bin/python3
OUT=$HERE/results/trk_$(date +%Y%m%d_%H%M%S); mkdir -p "$OUT"
cd "$HERE"
for rate in $RATES; do
  for tag in trk0 trk1; do
    t=0; [ $tag = trk1 ] && t=1
    echo "=== rate $rate $tag (P501_TRACKING=$t)"
    P501_TRACKING=$t TOGGLES=$TOG $PY tools/collect_report_data.py E "$rate" 2>&1 | tail -4
    mv /tmp/report/E_$rate.npy "$OUT/E_${rate}_$tag.npy"
  done
done
$PY tools/summarize_ab.py "$OUT" | tee "$OUT/summary.txt"
echo "results in $OUT"
