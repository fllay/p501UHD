#!/bin/bash
# Copy the patched notebooks + tools to ~/p501UHD on the board (never touches
# /home/xilinx/jupyter_notebooks). Run from the host that has key-based ssh to the board:
#   bash deploy_to_board.sh [xilinx@192.168.2.1]
set -e
BOARD=${1:-xilinx@192.168.2.1}
SRC=$(cd "$(dirname "$0")" && pwd)
ssh "$BOARD" "mkdir -p ~/p501UHD/notebooks ~/p501UHD/tools"
scp -q "$SRC"/p501/notebooks/*.py "$BOARD":p501UHD/notebooks/
scp -q "$SRC"/p501/docs/tools/collect_report_data.py "$SRC"/tools/summarize_ab.py "$SRC"/tools/synth_ab.sh "$BOARD":p501UHD/tools/
# the scripts hard-code the original folder: point them at the deployed copy
ssh "$BOARD" "sed -i \"s#/home/xilinx/jupyter_notebooks#/home/xilinx/p501UHD/notebooks#g\" ~/p501UHD/notebooks/*.py ~/p501UHD/tools/*.py"
echo "deployed to ~/p501UHD on $BOARD"
